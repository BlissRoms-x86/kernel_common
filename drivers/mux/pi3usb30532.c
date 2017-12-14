/*
 * Pericom PI3USB30532 Type-C cross switch / mux driver
 *
 * Copyright (c) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option)
 * any later version.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mux/consumer.h> /* For the MUX_USB_* defines */
#include <linux/mux/driver.h>

#define PI3USB30532_CONF			0x00

#define PI3USB30532_CONF_OPEN			0x00
#define PI3USB30532_CONF_SWAP			0x01
#define PI3USB30532_CONF_4LANE_DP		0x02
#define PI3USB30532_CONF_USB3			0x04
#define PI3USB30532_CONF_USB3_AND_2LANE_DP	0x06

struct pi3usb30532_mux {
	struct i2c_client *client;
};

static int pi3usb30532_mux_set_mux(struct mux_control *mux_ctrl, int state)
{
	struct pi3usb30532_mux *mux = mux_chip_priv(mux_ctrl->chip);
	u8 conf = PI3USB30532_CONF_OPEN;

	switch (state & ~MUX_USB_POLARITY_INV) {
	case MUX_USB_NONE:
		conf = PI3USB30532_CONF_OPEN;
		break;
	case MUX_USB_DEVICE:
	case MUX_USB_HOST:
		conf = PI3USB30532_CONF_USB3;
		break;
	case MUX_USB_HOST_AND_DP_SRC:
		conf = PI3USB30532_CONF_USB3_AND_2LANE_DP;
		break;
	case MUX_USB_DP_SRC:
		conf = PI3USB30532_CONF_4LANE_DP;
		break;
	}

	if (state & MUX_USB_POLARITY_INV)
		conf |= PI3USB30532_CONF_SWAP;

	return i2c_smbus_write_byte_data(mux->client, PI3USB30532_CONF, conf);
}

static const struct mux_control_ops pi3usb30532_mux_ops = {
	.set = pi3usb30532_mux_set_mux,
};

static int pi3usb30532_mux_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pi3usb30532_mux *mux;
	struct mux_chip *mux_chip;

	mux_chip = devm_mux_chip_alloc(dev, 1, sizeof(*mux));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	mux_chip->ops = &pi3usb30532_mux_ops;
	mux_chip->mux[0].states = MUX_USB_STATES;
	mux = mux_chip_priv(mux_chip);
	mux->client = client;

	return devm_mux_chip_register(dev, mux_chip);
}

static const struct i2c_device_id pi3usb30532_mux_table[] = {
	{ "pi3usb30532" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pi3usb30532_mux_table);

static struct i2c_driver pi3usb30532_mux_driver = {
	.driver = {
		.name = "pi3usb30532",
	},
	.probe_new = pi3usb30532_mux_probe,
	.id_table = pi3usb30532_mux_table,
};

module_i2c_driver(pi3usb30532_mux_driver);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("Pericom PI3USB30532 Type-C mux driver");
MODULE_LICENSE("GPL");
