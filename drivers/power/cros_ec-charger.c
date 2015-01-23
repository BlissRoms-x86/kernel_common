/*
 * Copyright (c) 2013 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Power supply consumer driver for ChromeOS EC Charger
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

/* Device Type 1 Reg */
#define TSU6721_TYPE_NONE		0x000000
#define TSU6721_TYPE_USB_HOST		0x000004
#define TSU6721_TYPE_CHG12		0x000010
#define TSU6721_TYPE_CDP		0x000020
#define TSU6721_TYPE_DCP		0x000040

/* Device Type 2 Reg */
#define TSU6721_TYPE_JIG_UART_ON	0x000400
#define TSU6721_TYPE_AUDIO3		0x008000

/* Device Type 3 Reg */
#define TSU6721_TYPE_APPLE_CHG		0x200000
#define TSU6721_TYPE_U200_CHG		0x400000
#define TSU6721_TYPE_NON_STD_CHG	0x040000

/* VBUS_DEBOUNCED might show up together with other type */
#define TSU6721_TYPE_VBUS_DEBOUNCED	0x020000

#define CHARGING_MASK (TSU6721_TYPE_USB_HOST | TSU6721_TYPE_CHG12 | \
		       TSU6721_TYPE_CDP | TSU6721_TYPE_DCP | \
		       TSU6721_TYPE_APPLE_CHG | TSU6721_TYPE_U200_CHG | \
		       TSU6721_TYPE_NON_STD_CHG | TSU6721_TYPE_JIG_UART_ON)

struct charger_data {
	struct device *dev;
	struct power_supply *charger;
	struct power_supply_desc psy_desc;
};

static enum power_supply_property cros_ec_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* charger is active or not */
	POWER_SUPPLY_PROP_CURRENT_NOW, /* current flowing out of charger */
	POWER_SUPPLY_PROP_VOLTAGE_NOW, /* voltage at charger */
	POWER_SUPPLY_PROP_POWER_NOW, /* product of voltage & current props */
};

static int is_debounced(struct ec_response_power_info *ec_data)
{
	return !!(ec_data->usb_dev_type & TSU6721_TYPE_VBUS_DEBOUNCED);
}

static void update_psu_type(struct power_supply *psy,
			    struct ec_response_power_info *ec_data)
{
	struct charger_data *charger_data = power_supply_get_drvdata(psy);

	dev_dbg(charger_data->dev, "dev_type = 0x%06x cur_limit = %d\n",
		ec_data->usb_dev_type & CHARGING_MASK,
		ec_data->usb_current_limit);

	charger_data->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	if (is_debounced(ec_data)) {
		switch (ec_data->usb_dev_type & CHARGING_MASK) {
		case TSU6721_TYPE_USB_HOST:
		case TSU6721_TYPE_JIG_UART_ON:
			charger_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			break;
		case TSU6721_TYPE_CDP:
			charger_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case TSU6721_TYPE_DCP:
		case TSU6721_TYPE_APPLE_CHG:
			charger_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case TSU6721_TYPE_CHG12:
			charger_data->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
			break;
		}
	}
}

static int get_ec_power_info(struct power_supply *psy,
			     struct ec_response_power_info *ec_info)
{
	struct charger_data *charger_data = power_supply_get_drvdata(psy);
	struct device *dev = charger_data->dev;
	struct cros_ec_device *ec = dev_get_drvdata(dev->parent);
	struct cros_ec_command *msg;
	int ret;

	msg = kzalloc(sizeof(*msg) + sizeof(*ec_info), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->version = 0;
	msg->command = EC_CMD_POWER_INFO;
	msg->insize = sizeof(*ec_info);
	ret = cros_ec_cmd_xfer_status(ec, msg);
	if (ret < 0) {
		dev_err(dev, "Unable to query EC power info (err:%d)\n",
			ret);
		goto error;
	}

	*ec_info = *((struct ec_response_power_info *)msg->data);
	ret = 0;

error:
	kfree(msg);
	return ret;
}

static void cros_ec_charger_power_changed(struct power_supply *psy)
{
	struct ec_response_power_info ec_info;

	get_ec_power_info(psy, &ec_info);
	update_psu_type(psy, &ec_info);
}

static int cros_ec_charger_get_prop(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct ec_response_power_info ec_info;
	int ret = get_ec_power_info(psy, &ec_info);

	if (ret)
		return -EINVAL;

	/* Zero properties unless we've detected presence of AC */
	if (!is_debounced(&ec_info)) {
		val->intval = 0;
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = ec_info.current_system * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ec_info.voltage_system * 1000;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = ec_info.voltage_system *
			ec_info.current_system;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *charger_supplied_to[] = {"cros-ec-charger"};

static int cros_ec_charger_probe(struct platform_device *pd)
{
	struct cros_ec_device *ec = dev_get_drvdata(pd->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct charger_data *charger_data;
	struct device *dev = &pd->dev;
	struct device_node *mfd_np, *charger_np;
	struct power_supply_desc *psy_desc;
	struct power_supply *psy;

	if (!ec) {
		dev_err(dev, "no EC device found\n");
		return -EINVAL;
	}

	mfd_np = dev->parent->of_node;
	if (!mfd_np) {
		dev_err(dev, "no device tree data available\n");
		return -EINVAL;
	}

	charger_np = of_find_node_by_name(mfd_np, "charger");
	if (!charger_np) {
		dev_err(dev, "no OF charger data found at %s\n",
			mfd_np->full_name);
		return -EINVAL;
	}

	charger_data = devm_kzalloc(dev, sizeof(struct charger_data),
				    GFP_KERNEL);
	if (!charger_data) {
		dev_err(dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}

	charger_data->dev = dev;

	psy_desc = &charger_data->psy_desc;
	psy_desc->name = "cros-ec-charger";
	psy_desc->type = POWER_SUPPLY_TYPE_MAINS;
	psy_desc->get_property = cros_ec_charger_get_prop;
	psy_desc->properties = cros_ec_charger_props;
	psy_desc->num_properties =
		ARRAY_SIZE(cros_ec_charger_props);
	psy_desc->external_power_changed = cros_ec_charger_power_changed;

	psy_cfg.supplied_to = charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(charger_supplied_to);
	psy_cfg.drv_data = charger_data;

	platform_set_drvdata(pd, charger_data);

	psy = power_supply_register(dev, psy_desc, &psy_cfg);
	if (IS_ERR(psy)) {
		dev_err(dev, "failed: power supply register\n");
		return PTR_ERR(psy);
	}
	charger_data->charger = psy;
	ec->charger = psy;

	return 0;
}

static int cros_ec_charger_remove(struct platform_device *pd)
{
	struct charger_data *charger_data = platform_get_drvdata(pd);

	power_supply_unregister(charger_data->charger);
	platform_set_drvdata(pd, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cros_ec_charger_resume(struct device *dev)
{
	struct charger_data *charger_data = dev_get_drvdata(dev);

	power_supply_changed(charger_data->charger);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cros_ec_charger_pm_ops, NULL,
			 cros_ec_charger_resume);

static struct platform_driver cros_ec_charger_driver = {
	.driver = {
		.name = "cros-ec-charger",
		.owner = THIS_MODULE,
		.pm = &cros_ec_charger_pm_ops,
	},
	.probe = cros_ec_charger_probe,
	.remove = cros_ec_charger_remove,
};

module_platform_driver(cros_ec_charger_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chrome EC charger");
MODULE_ALIAS("power_supply:cros-ec-charger");
