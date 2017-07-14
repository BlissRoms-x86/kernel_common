/*
 * Intel Vendor Defined XHCI extended capability handling
 *
 * Copyright (c) 2016) Hans de Goede <hdegoede@redhat.com>
 *
 * Loosely based on android x86 kernel code which is:
 *
 * Copyright (C) 2014 Intel Corp.
 *
 * Author: Wu, Hao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 */

#include <linux/platform_device.h>
#include "xhci.h"

/* Extended capability IDs for Intel Vendor Defined */
#define XHCI_EXT_CAPS_INTEL_HOST_CAP	192

static void xhci_intel_unregister_pdev(void *arg)
{
	platform_device_unregister(arg);
}

int xhci_intel_cap_init(struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct device *dev = hcd->self.controller;
	struct platform_device *pdev;
	struct resource	res = { 0, };
	int ret, ext_offset;

	ext_offset = xhci_find_next_ext_cap(&xhci->cap_regs->hc_capbase, 0,
					    XHCI_EXT_CAPS_INTEL_HOST_CAP);
	if (!ext_offset)
		return -ENODEV;

	/*
	 * If the Intel extended cap is present we create a platform device
	 * with its mmio region as resource for the platform/x86 mux driver.
	 */
	pdev = platform_device_alloc("intel_cht_usb_mux", PLATFORM_DEVID_NONE);
	if (!pdev) {
		xhci_err(xhci, "couldn't allocate intel_cht_usb_mux pdev\n");
		return -ENOMEM;
	}

	res.start = hcd->rsrc_start + ext_offset;
	res.end	  = res.start + 0x3ff;
	res.name  = "intel_cht_usb_mux";
	res.flags = IORESOURCE_MEM;

	ret = platform_device_add_resources(pdev, &res, 1);
	if (ret) {
		dev_err(dev, "couldn't add resources to intel_cht_usb_mux pdev\n");
		platform_device_put(pdev);
		return ret;
	}

	pdev->dev.parent = dev;

	ret = platform_device_add(pdev);
	if (ret) {
		dev_err(dev, "couldn't register intel_cht_usb_mux pdev\n");
		platform_device_put(pdev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, xhci_intel_unregister_pdev, pdev);
	if (ret) {
		dev_err(dev, "couldn't add unregister action for intel_cht_usb_mux pdev\n");
		return ret;
	}

	xhci_info(xhci, "Intel Vendor Defined Cap %d found, added intel_cht_usb_mux pdev\n",
		  XHCI_EXT_CAPS_INTEL_HOST_CAP);

	return 0;
}
