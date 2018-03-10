/*
 * XHCI extended capability handling
 *
 * Copyright (c) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include "xhci.h"

static void xhci_intel_unregister_pdev(void *arg)
{
	platform_device_unregister(arg);
}

static int xhci_create_intel_cht_sw_pdev(struct xhci_hcd *xhci, u32 cap_offset)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct device *dev = hcd->self.controller;
	struct platform_device *pdev;
	struct resource	res = { 0, };
	int ret;

	pdev = platform_device_alloc("intel_cht_usb_sw", PLATFORM_DEVID_NONE);
	if (!pdev) {
		xhci_err(xhci, "couldn't allocate intel_cht_usb_sw pdev\n");
		return -ENOMEM;
	}

	res.start = hcd->rsrc_start + cap_offset;
	res.end	  = res.start + 0x3ff;
	res.name  = "intel_cht_usb_sw";
	res.flags = IORESOURCE_MEM;

	ret = platform_device_add_resources(pdev, &res, 1);
	if (ret) {
		dev_err(dev, "couldn't add resources to intel_cht_usb_sw pdev\n");
		platform_device_put(pdev);
		return ret;
	}

	pdev->dev.parent = dev;

	ret = platform_device_add(pdev);
	if (ret) {
		dev_err(dev, "couldn't register intel_cht_usb_sw pdev\n");
		platform_device_put(pdev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, xhci_intel_unregister_pdev, pdev);
	if (ret) {
		dev_err(dev, "couldn't add unregister action for intel_cht_usb_sw pdev\n");
		return ret;
	}

	return 0;
}

int xhci_ext_cap_init(struct xhci_hcd *xhci)
{
	void __iomem *base = &xhci->cap_regs->hc_capbase;
	u32 cap_offset, val;
	int ret;

	cap_offset = xhci_find_next_ext_cap(base, 0, 0);

	while (cap_offset) {
		val = readl(base + cap_offset);

		switch (XHCI_EXT_CAPS_ID(val)) {
		case XHCI_EXT_CAPS_VENDOR_INTEL:
			if (xhci->quirks & XHCI_INTEL_CHT_USB_MUX) {
				ret = xhci_create_intel_cht_sw_pdev(
							    xhci, cap_offset);
				if (ret)
					return ret;
			}
			break;
		}
		cap_offset = xhci_find_next_ext_cap(base, cap_offset, 0);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xhci_ext_cap_init);
