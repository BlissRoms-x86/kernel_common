// SPDX-License-Identifier: GPL-2.0
/**
 * pi3usb302.c - Pericom PI3USB302_A Switch driver
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/typec_mux.h>
#include <linux/module.h>

struct pi3usb302 {
	struct typec_switch sw;
	struct gpio_desc *sel;
	struct gpio_desc *pd;
};

#define to_pi3usb302(f)	container_of(f, struct pi3usb302, sw)

static int
pi3usb302_set(struct typec_switch *sw, enum typec_orientation orientation)
{
	struct pi3usb302 *pi = to_pi3usb302(sw);

	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		gpiod_set_value_cansleep(pi->pd, 1);
		gpiod_set_value_cansleep(pi->sel, 0);
		break;
	case TYPEC_ORIENTATION_NORMAL:
		gpiod_set_value_cansleep(pi->pd, 0);
		gpiod_set_value_cansleep(pi->sel, 0);
		break;
	case TYPEC_ORIENTATION_REVERSE:
		gpiod_set_value_cansleep(pi->pd, 0);
		gpiod_set_value_cansleep(pi->sel, 1);
		break;
	}

	return 0;
}

static int pi3usb302_probe(struct platform_device *pdev)
{
	struct pi3usb302 *pi;
	int ret;

	pi = devm_kzalloc(&pdev->dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	/* Operation Mode Selection pin */
	pi->sel = devm_gpiod_get(&pdev->dev, "sel", GPIOD_OUT_LOW);
	if (IS_ERR(pi->sel))
		return PTR_ERR(pi->sel);

	/* Power Down pin */
	pi->pd = devm_gpiod_get_optional(&pdev->dev, "pd", GPIOD_OUT_LOW);
	if (IS_ERR(pi->pd))
		return PTR_ERR(pi->pd);

	pi->sw.dev = &pdev->dev;
	pi->sw.set = pi3usb302_set;

	ret = typec_switch_register(&pi->sw);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, pi);

	return 0;
}

static int pi3usb302_remove(struct platform_device *pdev)
{
	struct pi3usb302 *pi = platform_get_drvdata(pdev);

	typec_switch_unregister(&pi->sw);

	return 0;
}

static struct platform_driver pi3usb302_driver = {
	.driver = {
		.name		= "pi3usb302",
	},
	.probe			= pi3usb302_probe,
	.remove			= pi3usb302_remove,
};

module_platform_driver(pi3usb302_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Pericom PI3USB302_A Switch driver");
