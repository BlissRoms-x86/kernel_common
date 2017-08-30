/*
 * Intel INT3496 ACPI device extcon driver
 *
 * Copyright (c) 2016 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on android x86 kernel code which is:
 *
 * Copyright (c) 2014, Intel Corporation.
 * Author: David Cohen <david.a.cohen@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/acpi.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/platform_device.h>

#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>

#define INT3496_GPIO_USB_ID	0
#define INT3496_GPIO_VBUS_EN	1
#define INT3496_GPIO_USB_MUX	2
#define DEBOUNCE_TIME		msecs_to_jiffies(50)

struct int3496_data {
	struct device *dev;
	struct extcon_dev *edev;
	struct delayed_work work;
	struct gpio_desc *gpio_usb_id;
	struct gpio_desc *gpio_vbus_en;
	struct gpio_desc *gpio_usb_mux;
	struct mux_control *usb_mux;
	bool usb_mux_set;
	int usb_id_irq;
};

static const unsigned int int3496_cable[] = {
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static const struct acpi_gpio_params id_gpios = { INT3496_GPIO_USB_ID, 0, false };
static const struct acpi_gpio_params vbus_gpios = { INT3496_GPIO_VBUS_EN, 0, false };
static const struct acpi_gpio_params mux_gpios = { INT3496_GPIO_USB_MUX, 0, false };

static const struct acpi_gpio_mapping acpi_int3496_default_gpios[] = {
	{ "id-gpios", &id_gpios, 1 },
	{ "vbus-gpios", &vbus_gpios, 1 },
	{ "mux-gpios", &mux_gpios, 1 },
	{ },
};

static struct mux_lookup acpi_int3496_cht_mux_lookup[] = {
	{
		.provider = "intel_cht_usb_mux",
		.dev_id   = "INT3496:00",
		.mux_name = "usb-role-mux",
	},
};

#define ICPU(model)	{ X86_VENDOR_INTEL, 6, model, X86_FEATURE_ANY, }

static const struct x86_cpu_id cht_cpu_ids[] = {
	ICPU(INTEL_FAM6_ATOM_AIRMONT),		/* Braswell, Cherry Trail */
	{}
};

static bool int3496_soc_has_mux(void)
{
	return x86_match_cpu(cht_cpu_ids);
}

static void int3496_do_usb_id(struct work_struct *work)
{
	struct int3496_data *data =
		container_of(work, struct int3496_data, work.work);
	int ret, id = gpiod_get_value_cansleep(data->gpio_usb_id);

	/* id == 1: PERIPHERAL, id == 0: HOST */
	dev_dbg(data->dev, "Connected %s cable\n", id ? "PERIPHERAL" : "HOST");

	/*
	 * Peripheral: set USB mux to peripheral and disable VBUS
	 * Host: set USB mux to host and enable VBUS
	 */
	if (!IS_ERR(data->gpio_usb_mux))
		gpiod_direction_output(data->gpio_usb_mux, id);

	if (data->usb_mux) {
		/*
		 * The mux framework expects multiple competing users, we must
		 * release our previous setting before applying the new one.
		 */
		if (data->usb_mux_set)
			mux_control_deselect(data->usb_mux);

		ret = mux_control_select(data->usb_mux,
					 id ? MUX_USB_DEVICE : MUX_USB_HOST);
		if (ret)
			dev_err(data->dev, "Error setting mux: %d\n", ret);

		data->usb_mux_set = ret == 0;
	}

	if (!IS_ERR(data->gpio_vbus_en))
		gpiod_direction_output(data->gpio_vbus_en, !id);

	extcon_set_state_sync(data->edev, EXTCON_USB_HOST, !id);
}

static irqreturn_t int3496_thread_isr(int irq, void *priv)
{
	struct int3496_data *data = priv;

	/* Let the pin settle before processing it */
	mod_delayed_work(system_wq, &data->work, DEBOUNCE_TIME);

	return IRQ_HANDLED;
}

static int int3496_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct int3496_data *data;
	int ret;

	ret = devm_acpi_dev_add_driver_gpios(dev, acpi_int3496_default_gpios);
	if (ret) {
		dev_err(dev, "can't add GPIO ACPI mapping\n");
		return ret;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	INIT_DELAYED_WORK(&data->work, int3496_do_usb_id);

	if (int3496_soc_has_mux()) {
		mux_add_table(acpi_int3496_cht_mux_lookup,
			      ARRAY_SIZE(acpi_int3496_cht_mux_lookup));
		data->usb_mux = devm_mux_control_get(dev, "usb-role-mux");
		/* Doing this here keeps our error handling clean. */
		mux_remove_table(acpi_int3496_cht_mux_lookup,
				 ARRAY_SIZE(acpi_int3496_cht_mux_lookup));
		if (IS_ERR(data->usb_mux)) {
			ret = PTR_ERR(data->usb_mux);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "can't get mux: %d\n", ret);
			return ret;
		}
	}

	data->gpio_usb_id = devm_gpiod_get(dev, "id", GPIOD_IN);
	if (IS_ERR(data->gpio_usb_id)) {
		ret = PTR_ERR(data->gpio_usb_id);
		dev_err(dev, "can't request USB ID GPIO: %d\n", ret);
		return ret;
	} else if (gpiod_get_direction(data->gpio_usb_id) != GPIOF_DIR_IN) {
		dev_warn(dev, FW_BUG "USB ID GPIO not in input mode, fixing\n");
		gpiod_direction_input(data->gpio_usb_id);
	}

	data->usb_id_irq = gpiod_to_irq(data->gpio_usb_id);
	if (data->usb_id_irq < 0) {
		dev_err(dev, "can't get USB ID IRQ: %d\n", data->usb_id_irq);
		return data->usb_id_irq;
	}

	data->gpio_vbus_en = devm_gpiod_get(dev, "vbus", GPIOD_ASIS);
	if (IS_ERR(data->gpio_vbus_en))
		dev_info(dev, "can't request VBUS EN GPIO\n");

	data->gpio_usb_mux = devm_gpiod_get(dev, "mux", GPIOD_ASIS);
	if (IS_ERR(data->gpio_usb_mux))
		dev_info(dev, "can't request USB MUX GPIO\n");

	/* register extcon device */
	data->edev = devm_extcon_dev_allocate(dev, int3496_cable);
	if (IS_ERR(data->edev))
		return -ENOMEM;

	ret = devm_extcon_dev_register(dev, data->edev);
	if (ret < 0) {
		dev_err(dev, "can't register extcon device: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, data->usb_id_irq,
					NULL, int3496_thread_isr,
					IRQF_SHARED | IRQF_ONESHOT |
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					dev_name(dev), data);
	if (ret < 0) {
		dev_err(dev, "can't request IRQ for USB ID GPIO: %d\n", ret);
		return ret;
	}

	/* queue initial processing of id-pin */
	queue_delayed_work(system_wq, &data->work, 0);

	platform_set_drvdata(pdev, data);

	return 0;
}

static int int3496_remove(struct platform_device *pdev)
{
	struct int3496_data *data = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, data->usb_id_irq, data);
	cancel_delayed_work_sync(&data->work);

	return 0;
}

static const struct acpi_device_id int3496_acpi_match[] = {
	{ "INT3496" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, int3496_acpi_match);

static struct platform_driver int3496_driver = {
	.driver = {
		.name = "intel-int3496",
		.acpi_match_table = int3496_acpi_match,
	},
	.probe = int3496_probe,
	.remove = int3496_remove,
};

module_platform_driver(int3496_driver);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("Intel INT3496 ACPI device extcon driver");
MODULE_LICENSE("GPL");
