/*
 *  surface_vhf_keyboard.c - Microsoft Surface Virtual HID Framework Keyboard Device
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
 * The full GNU General Public License is included in the distribution in
 * the file called "COPYING".
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hid-sensor-hub.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/types.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jake Day");

static struct resource surface_vhf_keyboard_resources[] = {
	{
		.start	= 0x1a7bbaf9,
		.end	= 0x2d356b9e,
		.flags	= IORESOURCE_MEM,
		.name	= "io-memory"
	},
	{
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IRQ,
		.name	= "irq",
	}
};

static struct platform_device surface_vhf_keyboard = {
	.name 		= "surface_vhf",
	.resource	= surface_vhf_keyboard_resources,
	.num_resources	= ARRAY_SIZE(surface_vhf_keyboard_resources),
};

static int __init surface_hid_init(void)
{
	return platform_device_register(&surface_vhf_keyboard);
}
module_init(surface_hid_init);

static void __exit surface_hid_exit(void)
{
	platform_device_unregister(&surface_vhf_keyboard);
}
module_exit(surface_hid_exit);
