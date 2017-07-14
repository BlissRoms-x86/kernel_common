/*
 * Driver for ChipOne icn8318 i2c touchscreen controller
 *
 * Copyright (c) 2015-2017 Red Hat Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Red Hat authors:
 * Hans de Goede <hdegoede@redhat.com>
 */

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define ICN8318_REG_POWER		4
#define ICN8318_REG_TOUCHDATA		16

#define ICN8318_POWER_ACTIVE		0
#define ICN8318_POWER_MONITOR		1
#define ICN8318_POWER_HIBERNATE		2

#define ICN8318_MAX_TOUCHES		5

#define ICN8505_REG_TOUCHDATA		0x1000
#define ICN8505_REG_CONFIGDATA		0x8000

enum icn8318_model {
	ICN8318,
	ICN8505,
};

struct icn8318_touch {
	__u8 slot;
	__u8 x[2];
	__u8 y[2];
	__u8 pressure;	/* Seems more like finger width then pressure really */
	__u8 event;
/* The difference between 2 and 3 is unclear */
#define ICN8318_EVENT_NO_DATA	1 /* No finger seen yet since wakeup */
#define ICN8318_EVENT_UPDATE1	2 /* New or updated coordinates */
#define ICN8318_EVENT_UPDATE2	3 /* New or updated coordinates */
#define ICN8318_EVENT_END	4 /* Finger lifted */
} __packed;

struct icn8318_touch_data {
	__u8 softbutton;
	__u8 touch_count;
	struct icn8318_touch touches[ICN8318_MAX_TOUCHES];
} __packed;

struct icn8318_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *wake_gpio;
	struct touchscreen_properties prop;
	enum icn8318_model model;
	int touchdata_reg;
	u16 (*coord_to_cpu)(const u8 *buf);
};

static int icn8318_read_data(struct icn8318_data *data, int reg,
			     void *buf, int len)
{
	u8 addr[2];
	struct i2c_msg msg[2] = {
		{
			.addr = data->client->addr,
			.buf = addr
		},
		{
			.addr = data->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf
		}
	};

	if (data->model == ICN8318) {
		addr[0] = reg;
		msg[0].len = 1;
	} else {
		addr[0] = reg >> 8;
		addr[1] = reg & 0xff;
		msg[0].len = 2;
	}

	return i2c_transfer(data->client->adapter, msg, 2);
}

static u16 icn8318_coord_to_cpu(const u8 *buf)
{
	return get_unaligned_be16(buf);
}

static u16 icn8505_coord_to_cpu(const u8 *buf)
{
	return get_unaligned_le16(buf);
}

static inline bool icn8318_touch_active(u8 event)
{
	return (event == ICN8318_EVENT_UPDATE1) ||
	       (event == ICN8318_EVENT_UPDATE2);
}

static irqreturn_t icn8318_irq(int irq, void *dev_id)
{
	struct icn8318_data *data = dev_id;
	struct device *dev = &data->client->dev;
	struct icn8318_touch_data touch_data;
	int i, ret;

	ret = icn8318_read_data(data, data->touchdata_reg,
				&touch_data, sizeof(touch_data));
	if (ret < 0) {
		dev_err(dev, "Error reading touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (touch_data.touch_count > ICN8318_MAX_TOUCHES) {
		dev_warn(dev, "Too much touches %d > %d\n",
			 touch_data.touch_count, ICN8318_MAX_TOUCHES);
		touch_data.touch_count = ICN8318_MAX_TOUCHES;
	}

	for (i = 0; i < touch_data.touch_count; i++) {
		struct icn8318_touch *touch = &touch_data.touches[i];
		bool act = icn8318_touch_active(touch->event);

		input_mt_slot(data->input, touch->slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, act);
		if (!act)
			continue;

		touchscreen_report_pos(data->input, &data->prop,
				       data->coord_to_cpu(touch->x),
				       data->coord_to_cpu(touch->y),
				       true);
	}

	input_mt_sync_frame(data->input);
	input_report_key(data->input, KEY_LEFTMETA, touch_data.softbutton == 1);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int icn8318_start(struct input_dev *dev)
{
	struct icn8318_data *data = input_get_drvdata(dev);

	enable_irq(data->client->irq);
	if (data->wake_gpio)
		gpiod_set_value_cansleep(data->wake_gpio, 1);

	return 0;
}

static void icn8318_stop(struct input_dev *dev)
{
	struct icn8318_data *data = input_get_drvdata(dev);

	disable_irq(data->client->irq);
	i2c_smbus_write_byte_data(data->client, ICN8318_REG_POWER,
				  ICN8318_POWER_HIBERNATE);
	if (data->wake_gpio)
		gpiod_set_value_cansleep(data->wake_gpio, 0);
}

#ifdef CONFIG_PM_SLEEP
static int icn8318_suspend(struct device *dev)
{
	struct icn8318_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		icn8318_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int icn8318_resume(struct device *dev)
{
	struct icn8318_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		icn8318_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(icn8318_pm_ops, icn8318_suspend, icn8318_resume);

#ifdef CONFIG_ACPI
static const struct acpi_device_id icn8318_acpi_match[] = {
	{ "CHPN0001", ICN8505 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, icn8318_acpi_match);

static int icn8318_probe_acpi(struct icn8318_data *data, struct device *dev)
{
	const struct acpi_device_id *id;
	struct acpi_device *adev;

	adev = ACPI_COMPANION(dev);
	id = acpi_match_device(icn8318_acpi_match, dev);
	if (!adev || !id)
		return -ENODEV;

	data->model = id->driver_data;

	/*
	 * Disable ACPI power management the _PS3 method is empty, so
	 * there is no powersaving when using ACPI power management.
	 * The _PS0 method resets the controller causing it to loose its
	 * firmware, which has been loaded by the BIOS and we do not
	 * know how to restore the firmware.
	 */
	adev->flags.power_manageable = 0;

	return 0;
}
#else
static int icn8318_probe_acpi(struct icn8318_data *data, struct device *dev)
{
	return -ENODEV;
}
#endif

static int icn8318_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct icn8318_data *data;
	struct input_dev *input;
	__le16 resolution[2];
	int error;

	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->wake_gpio = devm_gpiod_get_optional(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(data->wake_gpio)) {
		error = PTR_ERR(data->wake_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Error getting wake gpio: %d\n", error);
		return error;
	}

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	data->client = client;
	data->input = input;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->open = icn8318_start;
	input->close = icn8318_stop;
	input->dev.parent = dev;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);

	if (client->dev.of_node) {
		data->model = (long)of_device_get_match_data(dev);
	} else {
		error = icn8318_probe_acpi(data, dev);
		if (error)
			return error;
	}

	if (data->model == ICN8318) {
		data->touchdata_reg = ICN8318_REG_TOUCHDATA;
		data->coord_to_cpu  = icn8318_coord_to_cpu;
	} else {
		data->touchdata_reg = ICN8505_REG_TOUCHDATA;
		data->coord_to_cpu  = icn8505_coord_to_cpu;

		error = icn8318_read_data(data, ICN8505_REG_CONFIGDATA,
					  resolution, sizeof(resolution));
		if (error < 0) {
			dev_err(dev, "Error reading resolution: %d\n", error);
			return error;
		}

		input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				     le16_to_cpu(resolution[0]) - 1, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				     le16_to_cpu(resolution[1]) - 1, 0, 0);
		input_set_capability(input, EV_KEY, KEY_LEFTMETA);
	}

	touchscreen_parse_properties(input, true, &data->prop);
	if (!input_abs_get_max(input, ABS_MT_POSITION_X) ||
	    !input_abs_get_max(input, ABS_MT_POSITION_Y)) {
		dev_err(dev, "Error touchscreen-size-x and/or -y missing\n");
		return -EINVAL;
	}

	error = input_mt_init_slots(input, ICN8318_MAX_TOUCHES,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, client->irq, NULL, icn8318_irq,
					  IRQF_ONESHOT, client->name, data);
	if (error) {
		dev_err(dev, "Error requesting irq: %d\n", error);
		return error;
	}

	/* Stop device till opened */
	icn8318_stop(data->input);

	error = input_register_device(input);
	if (error)
		return error;

	i2c_set_clientdata(client, data);

	return 0;
}

static const struct of_device_id icn8318_of_match[] = {
	{ .compatible = "chipone,icn8318", .data = (void *)ICN8318 },
	{ .compatible = "chipone,icn8505", .data = (void *)ICN8505 },
	{ }
};
MODULE_DEVICE_TABLE(of, icn8318_of_match);

/* This is useless for OF-enabled devices, but it is needed by I2C subsystem */
static const struct i2c_device_id icn8318_i2c_id[] = {
	{ },
};
MODULE_DEVICE_TABLE(i2c, icn8318_i2c_id);

static struct i2c_driver icn8318_driver = {
	.driver = {
		.name	= "chipone_icn8318",
		.pm	= &icn8318_pm_ops,
		.acpi_match_table = ACPI_PTR(icn8318_acpi_match),
		.of_match_table = icn8318_of_match,
	},
	.probe = icn8318_probe,
	.id_table = icn8318_i2c_id,
};

module_i2c_driver(icn8318_driver);

MODULE_DESCRIPTION("ChipOne icn8318 I2C Touchscreen Driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
