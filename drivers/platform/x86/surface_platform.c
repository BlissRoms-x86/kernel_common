/*
 *  surface_platform.c - Microsoft Surface Platform Driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  The full GNU General Public License is included in this distribution in
 *  the file called "COPYING".
 */

#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#include <asm/unaligned.h>

struct surface_platform_data {
	struct device *dev;
};

static int surface_platform_probe(struct platform_device *pdev)
{
	struct surface_platform_data *pdata;

	platform_set_drvdata(pdev, pdata);
	return 0;
}

static int surface_platform_remove(struct platform_device *pdev)
{
	struct surface_platform_data *pdata = platform_get_drvdata(pdev);
}

static const struct acpi_device_id surface_platform_acpi_match[] = {
	{ "MSHW0091", 0 },
	{ "INT3403", 0 },
	{ "", 0 }
};
MODULE_DEVICE_TABLE(acpi, surface_platform_acpi_match);

static struct platform_driver surface_platform_driver = {
	.probe = surface_platform_probe,
	.remove = surface_platform_remove,
	.driver = {
		.name = "surface_platform",
		.acpi_match_table = ACPI_PTR(surface_platform_acpi_match),
	},
};
module_platform_driver(surface_platform_driver);

MODULE_AUTHOR("Jake Day");
MODULE_DESCRIPTION("Microsoft Surface Platform Driver");
MODULE_LICENSE("GPL");
