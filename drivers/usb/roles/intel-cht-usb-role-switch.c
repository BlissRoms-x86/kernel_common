// SPDX-License-Identifier: GPL-2.0+
/*
 * Intel Cherrytrail USB OTG role switch driver
 *
 * Copyright (c) 2016-2017 Hans de Goede <hdegoede@redhat.com>
 *
 * Loosely based on android x86 kernel code which is:
 *
 * Copyright (C) 2014 Intel Corp.
 *
 * Author: Wu, Hao
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/role.h>

/* register definition */
#define DUAL_ROLE_CFG0			0x68
#define SW_VBUS_VALID			(1 << 24)
#define SW_IDPIN_EN			(1 << 21)
#define SW_IDPIN			(1 << 20)

#define DUAL_ROLE_CFG1			0x6c
#define HOST_MODE			(1 << 29)

#define DUAL_ROLE_CFG1_POLL_TIMEOUT	1000

#define DRV_NAME			"intel_cht_usb_sw"

struct intel_cht_usb_data {
	struct usb_role_switch *role_sw;
	void __iomem *base;
};

static int intel_cht_usb_set_role(struct device *dev, enum usb_role role)
{
	struct intel_cht_usb_data *data = dev_get_drvdata(dev);
	unsigned long timeout;
	acpi_status status;
	u32 glk = -1U;
	u32 val;

	/*
	 * On many CHT devices ACPI event (_AEI) handlers read / modify /
	 * write the cfg0 register, just like we do. Take the ACPI lock
	 * to avoid us racing with the AML code.
	 */
	status = acpi_acquire_global_lock(ACPI_WAIT_FOREVER, &glk);
	if (ACPI_FAILURE(status) && status != AE_NOT_CONFIGURED) {
		dev_err(dev, "Error could not acquire lock\n");
		return -EIO;
	}

	/* Set idpin value as requested */
	val = readl(data->base + DUAL_ROLE_CFG0);
	switch (role) {
	case USB_ROLE_NONE:
		val |= SW_IDPIN;
		val &= ~SW_VBUS_VALID;
		break;
	case USB_ROLE_HOST:
		val &= ~SW_IDPIN;
		val &= ~SW_VBUS_VALID;
		break;
	case USB_ROLE_DEVICE:
		val |= SW_IDPIN;
		val |= SW_VBUS_VALID;
		break;
	}
	val |= SW_IDPIN_EN;

	writel(val, data->base + DUAL_ROLE_CFG0);

	acpi_release_global_lock(glk);

	/* In most case it takes about 600ms to finish mode switching */
	timeout = jiffies + msecs_to_jiffies(DUAL_ROLE_CFG1_POLL_TIMEOUT);

	/* Polling on CFG1 register to confirm mode switch.*/
	do {
		val = readl(data->base + DUAL_ROLE_CFG1);
		if (!!(val & HOST_MODE) == (role == USB_ROLE_HOST))
			return 0;

		/* Interval for polling is set to about 5 - 10 ms */
		usleep_range(5000, 10000);
	} while (time_before(jiffies, timeout));

	dev_warn(dev, "Timeout waiting for role-switch\n");
	return -ETIMEDOUT;
}

static enum usb_role intel_cht_usb_get_role(struct device *dev)
{
	struct intel_cht_usb_data *data = dev_get_drvdata(dev);
	enum usb_role role;
	u32 val;

	val = readl(data->base + DUAL_ROLE_CFG0);

	if (!(val & SW_IDPIN))
		role = USB_ROLE_HOST;
	else if (val & SW_VBUS_VALID)
		role = USB_ROLE_DEVICE;
	else
		role = USB_ROLE_NONE;

	return role;
}

static const struct usb_role_switch_desc sw_desc = {
	.set = intel_cht_usb_set_role,
	.get = intel_cht_usb_get_role,
	/* TODO: disable this for Type-C ports */
	.allow_userspace_control = true,
};

static int intel_cht_usb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_cht_usb_data *data;
	struct resource *res;
	resource_size_t size;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = (res->end + 1) - res->start;
	data->base = devm_ioremap_nocache(dev, res->start, size);
	if (IS_ERR(data->base)) {
		ret = PTR_ERR(data->base);
		dev_err(dev, "Error iomaping registers: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	data->role_sw = usb_role_switch_register(dev, &sw_desc);
	if (IS_ERR(data->role_sw)) {
		ret = PTR_ERR(data->role_sw);
		dev_err(dev, "Error registering role-switch: %d\n", ret);
		return ret;
	}

	return 0;
}

int intel_cht_usb_remove(struct platform_device *pdev)
{
	struct intel_cht_usb_data *data = platform_get_drvdata(pdev);

	usb_role_switch_unregister(data->role_sw);
	return 0;
}

static const struct platform_device_id intel_cht_usb_table[] = {
	{ .name = DRV_NAME },
	{},
};
MODULE_DEVICE_TABLE(platform, intel_cht_usb_table);

static struct platform_driver intel_cht_usb_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.id_table = intel_cht_usb_table,
	.probe = intel_cht_usb_probe,
	.remove = intel_cht_usb_remove,
};

module_platform_driver(intel_cht_usb_driver);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("Intel Cherrytrail USB role switch driver");
MODULE_LICENSE("GPL");
