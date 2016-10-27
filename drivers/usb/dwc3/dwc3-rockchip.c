/**
 * dwc3-rockchip.c - Rockchip Specific Glue layer
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 *
 * Authors: William Wu <william.wu@rock-chips.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/extcon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "core.h"
#include "../host/xhci.h"

struct dwc3_rockchip {
	int			num_clocks;
	bool			connected;
	bool			suspended;
	struct device		*dev;
	struct clk		**clks;
	struct dwc3		*dwc;
	struct reset_control	*otg_rst;
	struct extcon_dev	*edev;
	struct notifier_block	host_nb;
	struct work_struct	otg_work;
	struct mutex		lock;
};

static u32 rockchip_dwc3_readl(void __iomem *base, u32 offset)
{
	/*
	 * We requested the mem region starting from the Globals address
	 * space, see dwc3_probe() in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	return readl(base + offset - DWC3_GLOBALS_REGS_START);
}

static void rockchip_dwc3_writel(void __iomem *base, u32 offset, u32 value)
{
	/*
	 * We requested the mem region starting from the Globals address
	 * space, see dwc3_probe() in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	writel(value, base + offset - DWC3_GLOBALS_REGS_START);
}

static void rockchip_dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = rockchip_dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	rockchip_dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static int dwc3_rockchip_host_notifier(struct notifier_block *nb,
				       unsigned long event, void *ptr)
{
	struct dwc3_rockchip *rockchip =
		container_of(nb, struct dwc3_rockchip, host_nb);

	if (!rockchip->suspended)
		schedule_work(&rockchip->otg_work);

	return NOTIFY_DONE;
}

static void dwc3_rockchip_otg_extcon_evt_work(struct work_struct *work)
{
	struct dwc3_rockchip	*rockchip =
		container_of(work, struct dwc3_rockchip, otg_work);
	struct dwc3		*dwc = rockchip->dwc;
	struct extcon_dev	*edev = rockchip->edev;
	struct usb_hcd		*hcd = dev_get_drvdata(&dwc->xhci->dev);
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	unsigned long		flags;

	mutex_lock(&rockchip->lock);

	if (extcon_get_cable_state_(edev, EXTCON_USB_HOST) > 0) {
		if (rockchip->connected)
			goto out;

		/*
		 * Revisit: Asserting the otg reset may affect dwc chip
		 * operation. The reset is known to clear the mode register.
		 * It is unknown if other registers are affected.
		 * An attempt to initialize the usb3 core with otg reset
		 * active failed.
		 * Also, there are no synchronization primitives, meaning
		 * the dwc3 core code could at least in theory access chip
		 * registers while the reset is asserted, with unknown impact.
		 */
		reset_control_assert(rockchip->otg_rst);
		reset_control_deassert(rockchip->otg_rst);

		/*
		 * Don't abort on errors. If powering on a phy fails,
		 * we still need to deassert reset and add the HCDs
		 * to avoid a crash when unloading the driver.
		 */
		if (phy_power_on(dwc->usb3_generic_phy) < 0)
			dev_err(dwc->dev, "Failed to power on usb3 phy\n");

		pm_runtime_get_sync(dwc->dev);

		/*
		 * NB: It is unclear what this lock actually protects,
		 * and why the chip access would require interrupts
		 * to be disabled. The lock is otherwise only used
		 * in gadget code. Leave in place for now since it
		 * does not hurt.
		 */
		spin_lock_irqsave(&dwc->lock, flags);
		rockchip_dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		spin_unlock_irqrestore(&dwc->lock, flags);

		/*
		 * The following sleep helps to ensure that inserted USB3
		 * Ethernet devices are discovered if already inserted
		 * when booting.
		 */
		msleep(10);

		if (hcd->state == HC_STATE_HALT) {
			usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
			usb_add_hcd(hcd->shared_hcd, hcd->irq, IRQF_SHARED);
		}

		rockchip->connected = true;
		dev_info(rockchip->dev, "USB HOST connected\n");
	} else {
		if (!rockchip->connected)
			goto out;

		/*
		 * xhci does not support runtime pm. If HCDs are not removed
		 * here and and re-added after a cable is inserted, USB3
		 * connections will not work.
		 * A clean(er) solution would be to implement runtime pm
		 * support in xhci. After that is available, this code should
		 * be removed.
		 * HCDs have to be removed here to prevent attempts by the
		 * xhci code to access xhci registers after the call to
		 * pm_runtime_put_sync_suspend(). On rk3399, this can result
		 * in a crash under certain circumstances (this was observed
		 * if the system is running on battery).
		 */
		if (hcd->state != HC_STATE_HALT) {
			xhci->xhc_state |= XHCI_STATE_REMOVING;
			usb_remove_hcd(hcd->shared_hcd);
			usb_remove_hcd(hcd);
		}

		pm_runtime_put_sync_suspend(dwc->dev);

		phy_power_off(dwc->usb3_generic_phy);

		rockchip->connected = false;
		dev_info(rockchip->dev, "USB HOST disconnected\n");
	}

out:
	mutex_unlock(&rockchip->lock);
}

static int dwc3_rockchip_extcon_register(struct dwc3_rockchip *rockchip)
{
	int			ret;
	struct device		*dev = rockchip->dev;
	struct extcon_dev	*edev;

	if (device_property_read_bool(dev, "extcon")) {
		edev = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(edev)) {
			if (PTR_ERR(edev) != -EPROBE_DEFER)
				dev_err(dev, "couldn't get extcon device\n");
			return PTR_ERR(edev);
		}

		INIT_WORK(&rockchip->otg_work,
			  dwc3_rockchip_otg_extcon_evt_work);

		rockchip->host_nb.notifier_call =
				dwc3_rockchip_host_notifier;
		ret = extcon_register_notifier(edev, EXTCON_USB_HOST,
					       &rockchip->host_nb);
		if (ret < 0) {
			dev_err(dev, "failed to register notifier for USB HOST\n");
			return ret;
		}

		rockchip->edev = edev;
	}

	return 0;
}

static void dwc3_rockchip_extcon_unregister(struct dwc3_rockchip *rockchip)
{
	if (!rockchip->edev)
		return;

	extcon_unregister_notifier(rockchip->edev, EXTCON_USB_HOST,
				   &rockchip->host_nb);
	cancel_work_sync(&rockchip->otg_work);
}

static int dwc3_rockchip_probe(struct platform_device *pdev)
{
	struct dwc3_rockchip	*rockchip;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node, *child;
	struct platform_device	*child_pdev;
	struct dwc3		*dwc;

	unsigned int		count;
	int			ret;
	int			i;

	rockchip = devm_kzalloc(dev, sizeof(*rockchip), GFP_KERNEL);
	if (!rockchip)
		return -ENOMEM;

	mutex_init(&rockchip->lock);

	rockchip->otg_rst = devm_reset_control_get(dev, "usb3-otg");
	if (IS_ERR(rockchip->otg_rst)) {
		if (PTR_ERR(rockchip->otg_rst) != -EPROBE_DEFER)
			dev_err(dev, "could not get reset controller\n");
		return PTR_ERR(rockchip->otg_rst);
	}

	child = of_get_child_by_name(np, "dwc3");
	if (!child) {
		dev_err(dev, "failed to find dwc3 core node\n");
		return -ENODEV;
	}

	count = of_clk_get_parent_count(np);
	if (!count)
		return -ENOENT;

	rockchip->num_clocks = count;

	rockchip->clks = devm_kcalloc(dev, rockchip->num_clocks,
				      sizeof(struct clk *), GFP_KERNEL);
	if (!rockchip->clks)
		return -ENOMEM;

	platform_set_drvdata(pdev, rockchip);

	rockchip->dev = dev;

	mutex_lock(&rockchip->lock);

	for (i = 0; i < rockchip->num_clocks; i++) {
		struct clk	*clk;

		clk = of_clk_get(np, i);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto err0;
		}

		ret = clk_prepare_enable(clk);
		if (ret < 0) {
			clk_put(clk);
			goto err0;
		}

		rockchip->clks[i] = clk;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "get_sync failed with err %d\n", ret);
		goto err1;
	}

	/* Allocate and initialize the core */
	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to create dwc3 core\n");
		goto err1;
	}

	child_pdev = of_find_device_by_node(child);
	if (!child_pdev) {
		dev_err(dev, "failed to find dwc3 core device\n");
		ret = -ENODEV;
		goto err2;
	}

	dwc = platform_get_drvdata(child_pdev);
	if (!dwc || !dwc->xhci) {
		dev_dbg(dev, "failed to get drvdata dwc3\n");
		ret = -EPROBE_DEFER;
		goto err2;
	}

	rockchip->dwc = dwc;

	if (dwc->dr_mode == USB_DR_MODE_HOST ||
	    dwc->dr_mode == USB_DR_MODE_OTG) {
		ret = dwc3_rockchip_extcon_register(rockchip);
		if (ret < 0)
			goto err2;

		if (rockchip->edev) {
			struct usb_hcd *hcd = dev_get_drvdata(&rockchip->dwc->xhci->dev);

			if (hcd->state != HC_STATE_HALT) {
				usb_remove_hcd(hcd->shared_hcd);
				usb_remove_hcd(hcd);
			}

			pm_runtime_dont_use_autosuspend(&child_pdev->dev);
			pm_runtime_allow(&child_pdev->dev);
			pm_runtime_put_sync(dev);

			phy_power_off(dwc->usb3_generic_phy);

			if (extcon_get_cable_state_(rockchip->edev,
						    EXTCON_USB_HOST) > 0)
				schedule_work(&rockchip->otg_work);
		}
	}

	mutex_unlock(&rockchip->lock);

	return ret;

err2:
	of_platform_depopulate(dev);

err1:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

err0:
	for (i = 0; i < rockchip->num_clocks && rockchip->clks[i]; i++) {
		if (!pm_runtime_status_suspended(dev))
			clk_disable(rockchip->clks[i]);
		clk_unprepare(rockchip->clks[i]);
		clk_put(rockchip->clks[i]);
	}

	mutex_unlock(&rockchip->lock);

	return ret;
}

static int dwc3_rockchip_remove(struct platform_device *pdev)
{
	struct dwc3_rockchip	*rockchip = platform_get_drvdata(pdev);
	struct device		*dev = &pdev->dev;
	int			i;

	dwc3_rockchip_extcon_unregister(rockchip);

	/* Restore hcd state before unregistering xhci */
	if (rockchip->edev && !rockchip->connected) {
		struct usb_hcd *hcd = dev_get_drvdata(&rockchip->dwc->xhci->dev);

		pm_runtime_get_sync(dev);

		/*
		 * The xhci code does not expect that HCDs have been removed.
		 * It will unconditionally call usb_remove_hcd() when the xhci
		 * driver is unloaded in of_platform_depopulate(). This results
		 * in a crash if the HCDs were already removed. To avoid this
		 * crash, add the HCDs here as dummy operation.
		 * This code should be removed after pm runtime support
		 * has been added to xhci.
		 */
		if (hcd->state == HC_STATE_HALT) {
			usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
			usb_add_hcd(hcd->shared_hcd, hcd->irq, IRQF_SHARED);
		}
	}

	of_platform_depopulate(dev);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	for (i = 0; i < rockchip->num_clocks; i++) {
		if (!pm_runtime_status_suspended(dev))
			clk_disable(rockchip->clks[i]);
		clk_unprepare(rockchip->clks[i]);
		clk_put(rockchip->clks[i]);
	}

	return 0;
}

#ifdef CONFIG_PM
static int dwc3_rockchip_runtime_suspend(struct device *dev)
{
	struct dwc3_rockchip	*rockchip = dev_get_drvdata(dev);
	int			i;

	for (i = 0; i < rockchip->num_clocks; i++)
		clk_disable(rockchip->clks[i]);

	return 0;
}

static int dwc3_rockchip_runtime_resume(struct device *dev)
{
	struct dwc3_rockchip	*rockchip = dev_get_drvdata(dev);
	int			i;

	for (i = 0; i < rockchip->num_clocks; i++)
		clk_enable(rockchip->clks[i]);

	return 0;
}

static int dwc3_rockchip_suspend(struct device *dev)
{
	struct dwc3_rockchip *rockchip = dev_get_drvdata(dev);

	rockchip->suspended = true;
	cancel_work_sync(&rockchip->otg_work);

	return 0;
}

static int dwc3_rockchip_resume(struct device *dev)
{
	struct dwc3_rockchip *rockchip = dev_get_drvdata(dev);

	rockchip->suspended = false;

	if (rockchip->edev)
		schedule_work(&rockchip->otg_work);

	return 0;
}

static const struct dev_pm_ops dwc3_rockchip_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_rockchip_suspend, dwc3_rockchip_resume)
	SET_RUNTIME_PM_OPS(dwc3_rockchip_runtime_suspend,
			   dwc3_rockchip_runtime_resume, NULL)
};

#define DEV_PM_OPS      (&dwc3_rockchip_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct of_device_id rockchip_dwc3_match[] = {
	{ .compatible = "rockchip,rk3399-dwc3" },
	{ /* Sentinel */ }
};

MODULE_DEVICE_TABLE(of, rockchip_dwc3_match);

static struct platform_driver dwc3_rockchip_driver = {
	.probe		= dwc3_rockchip_probe,
	.remove		= dwc3_rockchip_remove,
	.driver		= {
		.name	= "rockchip-dwc3",
		.of_match_table = rockchip_dwc3_match,
		.pm	= DEV_PM_OPS,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

module_platform_driver(dwc3_rockchip_driver);

MODULE_ALIAS("platform:rockchip-dwc3");
MODULE_AUTHOR("William Wu <william.wu@rock-chips.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 ROCKCHIP Glue Layer");
