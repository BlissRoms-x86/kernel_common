/*
 * Intel Cherrytrail USB OTG MUX driver
 *
 * Copyright (c) 2016 Hans de Goede <hdegoede@redhat.com>
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
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/* register definition */
#define DUAL_ROLE_CFG0			0x68
#define SW_VBUS_VALID			(1 << 24)
#define SW_IDPIN_EN			(1 << 21)
#define SW_IDPIN			(1 << 20)

#define DUAL_ROLE_CFG1			0x6c
#define HOST_MODE			(1 << 29)

#define DUAL_ROLE_CFG1_POLL_TIMEOUT	1000

#define DRV_NAME			"intel_cht_usb_mux"

enum mux_select { MUX_SEL_DEVICE, MUX_SEL_HOST };

struct intel_cht_usb_mux {
	struct device *dev;
	void __iomem *base;
	spinlock_t cfg0_lock;
	enum mux_select mux;
	struct extcon_dev *id_extcon;
	struct extcon_dev *vbus_extcon;
	struct notifier_block id_nb;
	struct notifier_block vbus_nb;
	struct work_struct id_work;
	struct work_struct vbus_work;
};

struct intel_cht_extcon_info {
	const char *hid;
	int hrv;
	const char *extcon;
};

struct intel_cht_extcon_info usb_id_providers[] = {
	{ "INT3496", -1, "INT3496:00" },
	{ "INT34D3",  3, "cht_wcove_pwrsrc" },
};

struct intel_cht_extcon_info vbus_providers[] = {
	{ "INT33F4", -1, "axp288_extcon" },
	{ "INT34D3",  3, "cht_wcove_pwrsrc" },
};

static const unsigned int vbus_cable_ids[] = {
	EXTCON_CHG_USB_SDP, EXTCON_CHG_USB_CDP, EXTCON_CHG_USB_DCP,
	EXTCON_CHG_USB_ACA, EXTCON_CHG_USB_FAST,
};

static void intel_cht_usb_mux_set_sw_mode(struct intel_cht_usb_mux *mux)
{
	u32 data;

	data = readl(mux->base + DUAL_ROLE_CFG0);
	if (!(data & SW_IDPIN_EN)) {
		data |= SW_IDPIN_EN;
		writel(data, mux->base + DUAL_ROLE_CFG0);
	}
}

static void intel_cht_usb_mux_set_mux(struct intel_cht_usb_mux *mux)
{
	unsigned long flags, timeout;
	u32 data;

	spin_lock_irqsave(&mux->cfg0_lock, flags);

	intel_cht_usb_mux_set_sw_mode(mux);

	/* Set idpin and vbus_valid as requested */
	data = readl(mux->base + DUAL_ROLE_CFG0);
	if (mux->mux == MUX_SEL_DEVICE)
		data |= SW_IDPIN;
	else
		data &= ~SW_IDPIN;
	writel(data, mux->base + DUAL_ROLE_CFG0);

	spin_unlock_irqrestore(&mux->cfg0_lock, flags);

	/* In most case it takes about 600ms to finish mode switching */
	timeout = jiffies + msecs_to_jiffies(DUAL_ROLE_CFG1_POLL_TIMEOUT);

	/* Polling on CFG1 register to confirm mode switch.*/
	while (1) {
		data = readl(mux->base + DUAL_ROLE_CFG1);
		if (mux->mux == MUX_SEL_DEVICE && !(data & HOST_MODE))
			break;
		if (mux->mux == MUX_SEL_HOST && (data & HOST_MODE))
			break;

		/* Interval for polling is set to about 5 - 10 ms */
		usleep_range(5000, 10000);

		if (time_after(jiffies, timeout)) {
			dev_warn(mux->dev, "Timeout waiting for mux to switch\n");
			break;
		}
	}
}

static void intel_cht_usb_mux_set_vbus_valid(struct intel_cht_usb_mux *mux,
					     bool valid)
{
	unsigned long flags;
	u32 data;

	spin_lock_irqsave(&mux->cfg0_lock, flags);

	intel_cht_usb_mux_set_sw_mode(mux);

	data = readl(mux->base + DUAL_ROLE_CFG0);
	if (valid)
		data |= SW_VBUS_VALID;
	else
		data &= ~SW_VBUS_VALID;
	writel(data, mux->base + DUAL_ROLE_CFG0);

	spin_unlock_irqrestore(&mux->cfg0_lock, flags);
}

static void intel_cht_usb_mux_id_work(struct work_struct *work)
{
	struct intel_cht_usb_mux *mux =
		container_of(work, struct intel_cht_usb_mux, id_work);

	intel_cht_usb_mux_set_mux(mux);
}

static void intel_cht_usb_mux_vbus_work(struct work_struct *work)
{
	struct intel_cht_usb_mux *mux =
		container_of(work, struct intel_cht_usb_mux, vbus_work);
	bool vbus_present = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(vbus_cable_ids); i++) {
		if (extcon_get_state(mux->vbus_extcon, vbus_cable_ids[i]) > 0) {
			vbus_present = true;
			break;
		}
	}

	intel_cht_usb_mux_set_vbus_valid(mux, vbus_present);
}

static int intel_cht_usb_mux_id_cable_evt(struct notifier_block *nb,
					  unsigned long event, void *param)
{
	struct intel_cht_usb_mux *mux =
		container_of(nb, struct intel_cht_usb_mux, id_nb);

	if (event == 1)
		mux->mux = MUX_SEL_HOST;
	else
		mux->mux = MUX_SEL_DEVICE;

	schedule_work(&mux->id_work);

	return NOTIFY_OK;
}

static int intel_cht_usb_mux_vbus_extcon_evt(struct notifier_block *nb,
					     unsigned long event, void *param)
{
	struct intel_cht_usb_mux *mux =
		container_of(nb, struct intel_cht_usb_mux, vbus_nb);

	schedule_work(&mux->vbus_work);

	return NOTIFY_OK;
}

static ssize_t intel_cht_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct intel_cht_usb_mux *mux = dev_get_drvdata(dev);

	if (mux->mux == MUX_SEL_DEVICE)
		return sprintf(buf, "device\n");
	else
		return sprintf(buf, "host\n");
}

static ssize_t intel_cht_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t n)
{
	struct intel_cht_usb_mux *mux = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "device")) {
		mux->mux = MUX_SEL_DEVICE;
	} else if (sysfs_streq(buf, "host")) {
		mux->mux = MUX_SEL_HOST;
	} else {
		return -EINVAL;
	}

	dev_info(mux->dev, "changing mode to %s\n", buf);
	intel_cht_usb_mux_set_mux(mux);

	return n;
}

static DEVICE_ATTR(mode, 0644, intel_cht_mode_show, intel_cht_mode_store);

static int intel_cht_usb_mux_probe(struct platform_device *pdev)
{
	struct intel_cht_usb_mux *mux;
	struct device *dev = &pdev->dev;
	struct resource *res;
	resource_size_t size;
	int i, ret;

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	mux->dev = dev;
	spin_lock_init(&mux->cfg0_lock);
	INIT_WORK(&mux->id_work, intel_cht_usb_mux_id_work);
	INIT_WORK(&mux->vbus_work, intel_cht_usb_mux_vbus_work);

	for (i = 0 ; i < ARRAY_SIZE(usb_id_providers); i++) {
		if (!acpi_dev_present(usb_id_providers[i].hid, NULL,
				      usb_id_providers[i].hrv))
			continue;

		mux->id_extcon = extcon_get_extcon_dev(
						usb_id_providers[i].extcon);
		if (mux->id_extcon == NULL)
			return -EPROBE_DEFER;

		dev_info(dev, "using extcon '%s' for usb-id\n",
			 usb_id_providers[i].extcon);
		break;
	}

	for (i = 0 ; i < ARRAY_SIZE(vbus_providers); i++) {
		if (!acpi_dev_present(vbus_providers[i].hid, NULL,
				      vbus_providers[i].hrv))
			continue;

		mux->vbus_extcon = extcon_get_extcon_dev(
						vbus_providers[i].extcon);
		if (mux->vbus_extcon == NULL)
			return -EPROBE_DEFER;

		dev_info(dev, "using extcon '%s' for vbus-valid\n",
			 vbus_providers[i].extcon);
		break;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = (res->end + 1) - res->start;
	mux->base = devm_ioremap_nocache(dev, res->start, size);
	if (IS_ERR(mux->base)) {
		ret = PTR_ERR(mux->base);
		dev_err(dev, "can't iomap registers: %d\n", ret);
		return ret;
	}

	/*
	 * mux->*_extcon may be NULL if no providers are present, in that
	 * case we still offer mux access through the sysfs mode attr.
	 */
	if (mux->id_extcon) {
		mux->id_nb.notifier_call = intel_cht_usb_mux_id_cable_evt;
		ret = devm_extcon_register_notifier(dev, mux->id_extcon,
					    EXTCON_USB_HOST, &mux->id_nb);
		if (ret) {
			dev_err(dev, "can't register id extcon notifier: %d\n",
				ret);
			return ret;
		}

		/* Sync initial mode */
		if (extcon_get_state(mux->id_extcon, EXTCON_USB_HOST) > 0)
			mux->mux = MUX_SEL_HOST;
		else
			mux->mux = MUX_SEL_DEVICE;

		schedule_work(&mux->id_work);
	}

	if (mux->vbus_extcon) {
		mux->vbus_nb.notifier_call = intel_cht_usb_mux_vbus_extcon_evt;
		ret = devm_extcon_register_notifier_all(dev, mux->vbus_extcon,
							&mux->vbus_nb);
		if (ret) {
			dev_err(dev, "can't register vbus extcon notifier: %d\n",
				ret);
			return ret;
		}

		/* Sync initial mode */
		schedule_work(&mux->vbus_work);
	}

	platform_set_drvdata(pdev, mux);
	device_create_file(dev, &dev_attr_mode);

	return 0;
}

static int intel_cht_usb_mux_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_mode);

	return 0;
}

static const struct platform_device_id intel_cht_usb_mux_table[] = {
	{ .name = DRV_NAME },
	{},
};
MODULE_DEVICE_TABLE(platform, intel_cht_usb_mux_table);

static struct platform_driver intel_cht_usb_mux_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.id_table = intel_cht_usb_mux_table,
	.probe = intel_cht_usb_mux_probe,
	.remove = intel_cht_usb_mux_remove,
};

module_platform_driver(intel_cht_usb_mux_driver);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("Intel Cherrytrail USB mux driver");
MODULE_LICENSE("GPL");
