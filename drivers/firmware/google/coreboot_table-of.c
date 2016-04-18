/*
 * coreboot_table-acpi.c
 *
 * Coreboot table access through open firmware.
 *
 * Copyright 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "coreboot_table.h"

static int coreboot_table_of_probe(struct platform_device *pdev)
{
	struct device_node *fw_dn;
	void __iomem *ptr;

	fw_dn = of_find_compatible_node(NULL, NULL, "coreboot");
	if (!fw_dn)
		return -ENODEV;

	ptr = of_iomap(fw_dn, 0);
	of_node_put(fw_dn);
	if (!ptr)
		return -ENOMEM;
	return coreboot_table_init(ptr);
}

static int coreboot_table_of_remove(struct platform_device *pdev)
{
	return coreboot_table_exit();
}

static struct platform_driver coreboot_table_of_driver = {
	.probe = coreboot_table_of_probe,
	.remove = coreboot_table_of_remove,
	.driver = {
		.name = "coreboot_table_of",
	},
};

static int __init platform_coreboot_table_of_init(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple("coreboot_table_of", -1,
					       NULL, 0);
	if (pdev == NULL)
		return -ENODEV;

	platform_driver_register(&coreboot_table_of_driver);

	return 0;
}

module_init(platform_coreboot_table_of_init);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
