/*
 *  surface_vhf.c - Microsoft Surface Virtual HID Framework Driver
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

#define STATUS_REG	0x0C
#define DATA_REG	0x10
#define DATA_AVAIL	0x2

static const struct acpi_device_id surface_vhf_ids[] = {
	{"MSHW0096", 0},
	{"", 0},
};

struct surface_kbd {
	struct input_dev *input;
	struct resource *res;
	void __iomem *io_base;
	struct clk *clk;
	unsigned short keycodes[256];
};

struct kbd_platform_data {
	const struct matrix_keymap_data *keymap;
	bool rep;
};

static const struct key_entry surface_vhf_keymap[] = {
	{ KE_KEY, 1, { KEY_ESC } },
	{ KE_KEY, 2, { KEY_F1 } },
	{ KE_KEY, 3, { KEY_F2 } },
	{ KE_KEY, 4, { KEY_F3 } },
	{ KE_KEY, 5, { KEY_F4 } },
	{ KE_KEY, 6, { KEY_F5 } },
	{ KE_KEY, 7, { KEY_F6 } },
	{ KE_KEY, 8, { KEY_F7 } },
	{ KE_KEY, 9, { KEY_F8 } },
	{ KE_KEY, 10, { KEY_F9 } },
	{ KE_KEY, 11, { KEY_F10 } },
	{ KE_KEY, 12, { KEY_F11 } },
	{ KE_KEY, 13, { KEY_F12 } },
	{ KE_KEY, 14, { KEY_POWER } },
	{ KE_KEY, 15, { KEY_DELETE } },
	{ KE_KEY, 16, { KEY_GRAVE } },
	{ KE_KEY, 17, { KEY_1 } },
	{ KE_KEY, 18, { KEY_2 } },
	{ KE_KEY, 19, { KEY_3 } },
	{ KE_KEY, 20, { KEY_4 } },
	{ KE_KEY, 21, { KEY_5 } },
	{ KE_KEY, 23, { KEY_6 } },
	{ KE_KEY, 23, { KEY_7 } },
	{ KE_KEY, 24, { KEY_8 } },
	{ KE_KEY, 25, { KEY_9 } },
	{ KE_KEY, 26, { KEY_0 } },
	{ KE_KEY, 27, { KEY_MINUS } },
	{ KE_KEY, 28, { KEY_EQUAL } },
	{ KE_KEY, 29, { KEY_BACKSPACE } },
	{ KE_KEY, 30, { KEY_TAB } },
	{ KE_KEY, 31, { KEY_Q } },
	{ KE_KEY, 32, { KEY_W } },
	{ KE_KEY, 33, { KEY_E } },
	{ KE_KEY, 34, { KEY_R } },
	{ KE_KEY, 35, { KEY_T } },
	{ KE_KEY, 36, { KEY_Y } },
	{ KE_KEY, 37, { KEY_U } },
	{ KE_KEY, 38, { KEY_I } },
	{ KE_KEY, 39, { KEY_O } },
	{ KE_KEY, 40, { KEY_P } },
	{ KE_KEY, 41, { KEY_LEFTBRACE } },
	{ KE_KEY, 42, { KEY_RIGHTBRACE } },
	{ KE_KEY, 43, { KEY_BACKSLASH } },
	{ KE_KEY, 44, { KEY_CAPSLOCK } },
	{ KE_KEY, 45, { KEY_A } },
	{ KE_KEY, 46, { KEY_S } },
	{ KE_KEY, 47, { KEY_D } },
	{ KE_KEY, 48, { KEY_F } },
	{ KE_KEY, 49, { KEY_G } },
	{ KE_KEY, 50, { KEY_H } },
	{ KE_KEY, 51, { KEY_J } },
	{ KE_KEY, 52, { KEY_K } },
	{ KE_KEY, 53, { KEY_L } },
	{ KE_KEY, 54, { KEY_SEMICOLON } },
	{ KE_KEY, 55, { KEY_APOSTROPHE } },
	{ KE_KEY, 56, { KEY_ENTER } },
	{ KE_KEY, 57, { KEY_LEFTSHIFT } },
	{ KE_KEY, 58, { KEY_Z } },
	{ KE_KEY, 59, { KEY_X } },
	{ KE_KEY, 60, { KEY_C } },
	{ KE_KEY, 61, { KEY_V } },
	{ KE_KEY, 62, { KEY_B } },
	{ KE_KEY, 63, { KEY_N } },
	{ KE_KEY, 64, { KEY_M } },
	{ KE_KEY, 65, { KEY_COMMA } },
	{ KE_KEY, 66, { KEY_DOT } },
	{ KE_KEY, 67, { KEY_SLASH } },
	{ KE_KEY, 68, { KEY_RIGHTSHIFT } },
	{ KE_KEY, 69, { KEY_LEFTCTRL } },
	{ KE_KEY, 70, { KEY_FN } },
	{ KE_KEY, 71, { KEY_KPASTERISK } },
	{ KE_KEY, 72, { KEY_LEFTALT } },
	{ KE_KEY, 73, { KEY_SPACE } },
	{ KE_KEY, 74, { KEY_RIGHTALT } },
	{ KE_KEY, 75, { KEY_MENU } },
	{ KE_KEY, 76, { KEY_LEFT } },
	{ KE_KEY, 77, { KEY_UP } },
	{ KE_KEY, 78, { KEY_DOWN } },
	{ KE_KEY, 79, { KEY_RIGHT } },
	{ KE_END },
};


static irqreturn_t surface_kbd_interrupt(int irq, void *dev_id)
{
	struct surface_kbd *kbd = dev_id;
	struct input_dev *input = kbd->input;
	unsigned int key;
	u8 sts, val;

	sts = readb(kbd->io_base + STATUS_REG);
	if (!(sts & DATA_AVAIL))
		return IRQ_NONE;

	val = readb(kbd->io_base + DATA_REG);
	key = kbd->keycodes[val];

	input_event(input, EV_MSC, MSC_SCAN, val);
	input_report_key(input, key, 1);
	input_sync(input);

	writeb(0, kbd->io_base + STATUS_REG);

	return IRQ_HANDLED;
}

static int surface_vhf_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct surface_kbd *kbd;
	struct input_dev *input_dev;
	int error;
	int ret;
	int irq;

	pr_info("Surface VHF found\n");

	pr_info("Surface VHF resources: %u\n", pdev->num_resources);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Surface VHF: No keyboard resource defined\n");
		return -EBUSY;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "not able to get irq for the device\n");
		return irq;
	}

	kbd = kzalloc(sizeof(*kbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!kbd || !input_dev) {
		dev_err(&pdev->dev, "Surface VHF: Out of memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	kbd->input = input_dev;

	/*kbd->res = request_mem_region(res->start, resource_size(res),
				      pdev->name);
	if (!kbd->res) {
		dev_err(&pdev->dev, "keyboard region already claimed\n");
		error = -EBUSY;
		goto err_free_mem;
	}*/

	kbd->io_base = ioremap(res->start, resource_size(res));
	if (!kbd->io_base) {
		dev_err(&pdev->dev, "Surface VHF: ioremap failed for kbd region\n");
		error = -ENOMEM;
		goto err_release_mem_region;
	}

	kbd->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(kbd->clk)) {
		error = PTR_ERR(kbd->clk);
		goto err_iounmap;
	}

	input_dev->name = "Surface Laptop Keyboard";
	input_dev->phys = "keyboard/input0";
	input_dev->dev.parent = &pdev->dev;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x045e;
	input_dev->id.product = 0xf001;
	input_dev->id.version = 0x0001;

	__set_bit(EV_KEY, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_dev->keycode = kbd->keycodes;
	input_dev->keycodesize = sizeof(kbd->keycodes[0]);
	input_dev->keycodemax = ARRAY_SIZE(kbd->keycodes);

	input_set_drvdata(input_dev, kbd);

	ret = sparse_keymap_setup(input_dev, surface_vhf_keymap, NULL);
	if (ret)
		return ret;

	error = request_irq(irq, surface_kbd_interrupt, 0, "keyboard", kbd);
	if (error) {
		dev_err(&pdev->dev, "Surface VHF: Request_irq fail\n");
		goto err_put_clk;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "Surface VHF: Unable to register keyboard device\n");
		return 0;
	}

	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, kbd);

	return 0;

/*err_free_irq:
	free_irq(kbd->irq, kbd);*/
err_put_clk:
	clk_put(kbd->clk);
err_iounmap:
	iounmap(kbd->io_base);
err_release_mem_region:
	release_mem_region(res->start, resource_size(res));
err_free_mem:
	input_free_device(input_dev);
	kfree(kbd);

	return error;
}

static int surface_vhf_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, false);

	return 0;
}

static struct platform_driver surface_vhf_driver = {
	.driver = {
		.name = "surface_vhf",
		.acpi_match_table = surface_vhf_ids,
	},
	.probe = surface_vhf_probe,
	.remove = surface_vhf_remove,
};
MODULE_DEVICE_TABLE(acpi, surface_vhf_ids);

static acpi_status __init
check_acpi_dev(acpi_handle handle, u32 lvl, void *context, void **rv)
{
	const struct acpi_device_id *ids = context;
	struct acpi_device *dev;

	if (acpi_bus_get_device(handle, &dev) != 0)
		return AE_OK;

	if (acpi_match_device_ids(dev, ids) == 0)
		if (acpi_create_platform_device(dev, NULL))
			dev_info(&dev->dev,
				 "Surface VHF: Created platform device\n");

	return AE_OK;
}

static int __init surface_vhf_init(void)
{
	acpi_walk_namespace(ACPI_TYPE_DEVICE, ACPI_ROOT_OBJECT,
			    ACPI_UINT32_MAX, check_acpi_dev, NULL,
			    (void *)surface_vhf_ids, NULL);

	return platform_driver_register(&surface_vhf_driver);
}
module_init(surface_vhf_init);

static void __exit surface_vhf_exit(void)
{
	platform_driver_unregister(&surface_vhf_driver);
}
module_exit(surface_vhf_exit);
