/**
 * drivers/extcon/extcon-cros_ec - ChromeOS Embedded Controller extcon
 *
 * Copyright (C) 2015 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/extcon.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/usb/class-dual-role.h>
#include <linux/wakelock.h>

#define CROS_EC_USB_POLLING_DELAY msecs_to_jiffies(1000)

/*
 * Timeout for a USB PD power swap execution
 * 1000 ms for tSwapRecovery : maximum time after Hard Reset to settle
 *  275 ms for tSrcTurnOn (VBUS going from 0V to 5V)
 *  650 ms for tSafe0V (VBUS going to 0V)
 *  500 ms of extra margin
 */
#define POWER_SWAP_TIMEOUT msecs_to_jiffies(2425)
/*
 * Timeout for USB PD data swap execution
 *   30 ms for tSenderResponse
 * 2x 1 ms for tReceive
 *   some margin for events and AP/EC communication
 */
#define DATA_SWAP_TIMEOUT msecs_to_jiffies(150)

struct cros_ec_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct cros_ec_device *ec;

	struct notifier_block notifier;
	struct wake_lock wakelock;
	bool wakelock_held;

	unsigned int dr; /* data role */
	unsigned int pr; /* power role */
	unsigned int power_type;
	unsigned int writeable;
	wait_queue_head_t role_wait;

	struct dual_role_phy_instance *drp_inst;
};

/* List of detectable cables */
enum {
	EXTCON_CABLE_USB = 0,
	EXTCON_CABLE_USB_HOST,

	EXTCON_CABLE_END,
};

static const unsigned int usb_type_c_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/**
 * cros_ec_pd_command() - Send a command to the EC.
 * @dev: PD device
 * @ec_dev: pointer to cros_ec_device structure to talk to the physical device
 * @command: EC command
 * @version: EC command version
 * @outdata: EC command output data
 * @outsize: Size of outdata
 * @indata: EC command input data
 * @insize: Size of indata
 *
 * Return: 0 on success, <0 on failure.
 */
static int cros_ec_pd_command(struct device *dev,
			      struct cros_ec_device *ec_dev,
			      unsigned int command,
			      unsigned int version,
			      uint8_t *outdata,
			      unsigned int outsize,
			      uint8_t *indata,
			      unsigned int insize)
{
	struct cros_ec_command *msg;
	int ret;

	msg = kzalloc(sizeof(*msg) + max(outsize, insize), GFP_KERNEL);

	msg->version = version;
	msg->command = command;
	msg->outsize = outsize;
	msg->insize = insize;

	if (outsize)
		memcpy(msg->data, outdata, outsize);

	ret = cros_ec_cmd_xfer_status(ec_dev, msg);
	if (ret >= 0 && insize)
		memcpy(indata, msg->data, insize);

	kfree(msg);
	return ret;
}

/**
 * cros_ec_usb_get_power_type() - Get power type info about PD device attached to
 * given port.
 * @dev: PD device
 * @ec_dev: pointer to cros_ec_device structure to talk to the physical device
 * @port: Port # on device
 * @power_type: Holds a value of usb_chg_type on command success
 *
 * Return: 0 on success, <0 on failure.
 */
static int cros_ec_usb_get_power_type(struct device *dev,
				      struct cros_ec_device *ec_dev,
				      unsigned int port,
				      unsigned int *power_type)
{
	struct ec_params_usb_pd_power_info req;
	struct ec_response_usb_pd_power_info resp;
	int ret;

	req.port = port;
	ret = cros_ec_pd_command(dev, ec_dev, EC_CMD_USB_PD_POWER_INFO, 0,
				 (uint8_t *)&req, sizeof(req),
				 (uint8_t *)&resp, sizeof(resp));
	if (ret < 0)
		return ret;

	*power_type = resp.type;

	return ret;
}



/**
 * cros_ec_usb_get_role() - Get role info about possible PD device attached to a
 * given port.
 * @dev: PD device
 * @ec_dev: pointer to cros_ec_device structure to talk to the physical device
 * @port: Port # on device
 * @role: data/power/vconn roles
 *
 *
 * Return: >0 on success, 0 if no cable is connected, <0 on failure.
 */
static int cros_ec_usb_get_role(struct device *dev,
				struct cros_ec_device *ec_dev,
				unsigned int port,
				unsigned int *role)
{
	struct ec_params_usb_pd_control pd_control;
	struct ec_response_usb_pd_control_v1 resp;
	int ret;

	pd_control.port = port;
	pd_control.role = USB_PD_CTRL_ROLE_NO_CHANGE;
	pd_control.mux = USB_PD_CTRL_MUX_NO_CHANGE;
	pd_control.swap = USB_PD_CTRL_SWAP_NONE;
	ret = cros_ec_pd_command(dev, ec_dev, EC_CMD_USB_PD_CONTROL, 1,
				 (uint8_t *)&pd_control, sizeof(pd_control),
				 (uint8_t *)&resp, sizeof(resp));
	if (ret < 0)
		return ret;

	*role = resp.role;
	return resp.enabled & (1 << 1);
}

/**
 * cros_ec_pd_get_num_ports() - Get number of EC charge ports.
 * @dev: PD device
 * @ec_dev: pointer to cros_ec_device structure to talk to the physical device
 * @num_ports: Holds number of ports, on command success
 *
 * Return: 0 on success, <0 on failure.
 */
static int cros_ec_pd_get_num_ports(struct device *dev,
				    struct cros_ec_device *ec_dev,
				    unsigned int *num_ports)
{
	struct ec_response_usb_pd_ports resp;
	int ret;

	ret = cros_ec_pd_command(dev, ec_dev, EC_CMD_USB_PD_PORTS,
				 0, NULL, 0,
				 (uint8_t *)&resp, sizeof(resp));
	if (ret == EC_RES_SUCCESS)
		*num_ports = resp.num_ports;
	return ret;
}

static const char *cros_ec_usb_role_string(unsigned int role)
{
	return DUAL_ROLE_PROP_DR_NONE == role ? "DISCONNECTED" :
		(DUAL_ROLE_PROP_DR_HOST == role ? "DFP" : "UFP");
}

static const char *cros_ec_usb_power_type_string(unsigned int type)
{
	switch (type) {
	case USB_CHG_TYPE_NONE:
		return "USB_CHG_TYPE_NONE";
		break;
	case USB_CHG_TYPE_PD:
		return "USB_CHG_TYPE_PD";
		break;
	case USB_CHG_TYPE_PROPRIETARY:
		return "USB_CHG_TYPE_PROPRIETARY";
		break;
	case USB_CHG_TYPE_C:
		return "USB_CHG_TYPE_C";
		break;
	case USB_CHG_TYPE_BC12_DCP:
		return "USB_CHG_TYPE_BC12_DCP";
		break;
	case USB_CHG_TYPE_BC12_CDP:
		return "USB_CHG_TYPE_BC12_CDP";
		break;
	case USB_CHG_TYPE_BC12_SDP:
		return "USB_CHG_TYPE_BC12_SDP";
		break;
	case USB_CHG_TYPE_OTHER:
		return "USB_CHG_TYPE_OTHER";
		break;
	case USB_CHG_TYPE_VBUS:
		return "USB_CHG_TYPE_VBUS";
		break;
	case USB_CHG_TYPE_UNKNOWN:
		return "USB_CHG_TYPE_UNKNOWN";
		break;
	default:
		return "USB_CHG_TYPE_UNKNOWN";
	}
}

static bool cros_ec_usb_power_type_is_wall_wart(unsigned int type,
						unsigned int role)
{
	switch (type) {
	/* FIXME : Guppy, Donnettes, and other chargers will be miscategorized
	 * because they identify with USB_CHG_TYPE_C, but we can't return true
	 * here from that code because that breaks Suzy-Q and other kinds of
	 * USB Type-C cables and peripherals.
	 */
	case USB_CHG_TYPE_PROPRIETARY:
	case USB_CHG_TYPE_BC12_DCP:
		return true;
		break;
	case USB_CHG_TYPE_PD:
#if 0		/* TODO(crosbug.com/p/45871) use USB comm bit when available */
		return !(role & PD_CTRL_RESP_ROLE_USB_COMM);
#else
		return false;
#endif
		break;
	case USB_CHG_TYPE_C:
	case USB_CHG_TYPE_BC12_CDP:
	case USB_CHG_TYPE_BC12_SDP:
	case USB_CHG_TYPE_OTHER:
	case USB_CHG_TYPE_VBUS:
	case USB_CHG_TYPE_UNKNOWN:
	case USB_CHG_TYPE_NONE:
	default:
		return false;
	}
}

static unsigned int cros_ec_usb_role_is_writeable(unsigned int role)
{
	unsigned int write_mask = 0;

	if (role & PD_CTRL_RESP_ROLE_DR_POWER)
		write_mask |= (1 << DUAL_ROLE_PROP_PR);
	if ((role & PD_CTRL_RESP_ROLE_DR_DATA) &&
	    (role & PD_CTRL_RESP_ROLE_USB_COMM))
		write_mask |= (1 << DUAL_ROLE_PROP_DR);

	return write_mask;
}

static void extcon_cros_ec_detect_cable(struct cros_ec_extcon_info *info)
{
	struct device *dev = info->dev;
	struct cros_ec_device *ec = info->ec;
	int err, res;
	unsigned int role, dr, pr, power_type;

	err = cros_ec_usb_get_power_type(dev, ec, 0, &power_type);
	if (err) {
		dev_err(dev, "failed getting power type err = %d\n", err);
		return;
	}

	res = cros_ec_usb_get_role(dev, ec, 0, &role);
	if (res < 0) {
		dev_err(dev, "failed getting role err = %d\n", res);
		return;
	}
	if (res) {
		dr = (role & PD_CTRL_RESP_ROLE_DATA) ?
			DUAL_ROLE_PROP_DR_HOST : DUAL_ROLE_PROP_DR_DEVICE;
		pr = (role & PD_CTRL_RESP_ROLE_POWER) ?
			DUAL_ROLE_PROP_PR_SRC : DUAL_ROLE_PROP_PR_SNK;
	} else {
		dr = DUAL_ROLE_PROP_DR_NONE;
		pr = DUAL_ROLE_PROP_PR_NONE;
	}
	/*
	 * When there is no USB host (e.g. USB PD charger),
	 * we are not really a UFP for the AP.
	 */
	if (dr == DUAL_ROLE_PROP_DR_DEVICE &&
	    cros_ec_usb_power_type_is_wall_wart(power_type, role))
		dr = DUAL_ROLE_PROP_DR_NONE;

	if (info->dr != dr || info->pr != pr ||
	    info->power_type != power_type) {
		bool host_connected = false, device_connected = false;

		dev_dbg(dev, "Type/Role switch! type = %s role = %s\n",
			cros_ec_usb_power_type_string(power_type),
			cros_ec_usb_role_string(dr));
		info->dr = dr;
		info->pr = pr;
		info->power_type = power_type;
		info->writeable = cros_ec_usb_role_is_writeable(role);

		if (dr == DUAL_ROLE_PROP_DR_DEVICE)
			device_connected = true;
		else if (dr == DUAL_ROLE_PROP_DR_HOST)
			host_connected = true;

		/* Set a wakelock if device is connected, otherwise release */
		if (device_connected && !info->wakelock_held) {
			wake_lock(&info->wakelock);
			info->wakelock_held = true;
		} else if (!device_connected && info->wakelock_held) {
			wake_unlock(&info->wakelock);
			info->wakelock_held = false;
		}

		extcon_set_cable_state_(info->edev, EXTCON_USB_HOST,
					host_connected);
		extcon_set_cable_state_(info->edev, EXTCON_USB,
					device_connected);
		wake_up_all(&info->role_wait);
		dual_role_instance_changed(info->drp_inst);
	}
}

static int extcon_cros_ec_event(struct notifier_block *nb,
	unsigned long queued_during_suspend, void *_notify)
{
	struct cros_ec_extcon_info *info;
	struct device *dev;
	struct cros_ec_device *ec;
	u32 host_event;

	info = container_of(nb, struct cros_ec_extcon_info, notifier);
	dev = info->dev;
	ec = info->ec;

	host_event = cros_ec_get_host_event(ec);
	if (host_event & EC_HOST_EVENT_MASK(EC_HOST_EVENT_PD_MCU)) {
		extcon_cros_ec_detect_cable(info);
		return NOTIFY_OK;
	} else {
		return NOTIFY_DONE;
	}
}

static bool extcon_cros_ec_has_vconn(struct cros_ec_extcon_info *info)
{
	struct device *dev = info->dev;
	struct cros_ec_device *ec = info->ec;
	unsigned int role;
	int res;

	res = cros_ec_usb_get_role(dev, ec, 0, &role);

	return (res > 0) && (role & PD_CTRL_RESP_ROLE_VCONN);
}

static int extcon_cros_ec_force_data_role(struct cros_ec_extcon_info *info,
					   unsigned int new_dr)
{
	struct device *dev = info->dev;
	struct cros_ec_device *ec = info->ec;
	struct ec_params_usb_pd_control pd_control;
	struct ec_response_usb_pd_control_v1 resp;
	int ret;

	dev_info(dev, "Force Data Role to %d (from %d)\n", new_dr, info->dr);

	if ((new_dr != DUAL_ROLE_PROP_DR_HOST) &&
	    (new_dr != DUAL_ROLE_PROP_DR_DEVICE))
		return -EINVAL;

	if (new_dr == info->dr)
		return 0;

	pd_control.port = 0;
	pd_control.role = USB_PD_CTRL_ROLE_NO_CHANGE;
	pd_control.mux = USB_PD_CTRL_MUX_NO_CHANGE;
	pd_control.swap = USB_PD_CTRL_SWAP_DATA;
	ret = cros_ec_pd_command(dev, ec, EC_CMD_USB_PD_CONTROL, 1,
				 (uint8_t *)&pd_control, sizeof(pd_control),
				 (uint8_t *)&resp, sizeof(resp));
	dev_dbg(dev, "EC data swap to %s = %d\n",
		new_dr == DUAL_ROLE_PROP_DR_HOST ? "dfp" : "ufp", ret);
	if (ret < 0)
		return ret;

	/* wait for the swap to happen or timeout */
	ret = wait_event_timeout(info->role_wait, new_dr == info->dr,
				 DATA_SWAP_TIMEOUT);
	dev_dbg(dev, "data swap %s role %s\n",
		ret == 0 ? "timed out" : "succeeded", info->dr ? "UFP" : "DFP");

	return ret == 0 ? -ETIMEDOUT : ret;
}

static int extcon_cros_ec_force_power_role(struct cros_ec_extcon_info *info,
					   unsigned int new_pr)
{
	struct device *dev = info->dev;
	struct cros_ec_device *ec = info->ec;
	struct ec_params_charge_port_override p;
	int ret;

	dev_info(dev, "Force Power Role to %d (from %d)\n", new_pr, info->pr);

	if (new_pr == info->pr)
		return 0;

	switch (new_pr) {
	case DUAL_ROLE_PROP_PR_SRC:
		p.override_port = OVERRIDE_DONT_CHARGE;
		break;
	case DUAL_ROLE_PROP_PR_SNK:
		p.override_port = 0;
		break;
	case DUAL_ROLE_PROP_PR_NONE:
	default:
		return -EINVAL;
	}

	ret = cros_ec_pd_command(dev, ec, EC_CMD_PD_CHARGE_PORT_OVERRIDE, 0,
				 (uint8_t *)&p, sizeof(p), NULL, 0);
	dev_dbg(dev, "EC charge port override to %d = %d\n",
		p.override_port, ret);
	if (ret < 0)
		return ret;

	/* wait for the swap to happen or timeout */
	ret = wait_event_timeout(info->role_wait, new_pr == info->pr,
				 POWER_SWAP_TIMEOUT);
	dev_dbg(dev, "power swap %s role %s\n",
		ret == 0 ? "timed out" : "succeed", info->pr ? "SNK" : "SRC");

	return ret == 0 ? -ETIMEDOUT : ret;
}

static int extcon_drp_get_prop(struct dual_role_phy_instance *inst,
			enum dual_role_property prop,
			unsigned int *val)
{
	struct cros_ec_extcon_info *info = dual_role_get_drvdata(inst);
	int ret = 0;

	if (!info)
		return -EINVAL;

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		*val = info->pr == DUAL_ROLE_PROP_PR_SRC ?
			DUAL_ROLE_PROP_MODE_DFP :
			(info->pr == DUAL_ROLE_PROP_PR_SNK ?
				DUAL_ROLE_PROP_MODE_UFP :
				DUAL_ROLE_PROP_MODE_NONE);
		break;
	case DUAL_ROLE_PROP_PR:
		*val = info->pr;
		break;
	case DUAL_ROLE_PROP_DR:
		*val = info->dr;
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		*val = extcon_cros_ec_has_vconn(info);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int extcon_drp_is_writeable(struct dual_role_phy_instance *inst,
				enum dual_role_property prop)
{
	struct cros_ec_extcon_info *info = dual_role_get_drvdata(inst);

	if (info)
		return info->writeable & (1 << prop);
	else /* Not initialized yet */
		return (prop == DUAL_ROLE_PROP_PR) ||
		       (prop == DUAL_ROLE_PROP_DR);
}

static int extcon_drp_set_prop(struct dual_role_phy_instance *inst,
				enum dual_role_property prop,
				const unsigned int *val)
{
	struct cros_ec_extcon_info *info = dual_role_get_drvdata(inst);
	int ret = 0;

	if (!info)
		return -EINVAL;

	switch (prop) {
	case DUAL_ROLE_PROP_PR:
		ret = extcon_cros_ec_force_power_role(info, *val);
		break;
	case DUAL_ROLE_PROP_DR:
		ret = extcon_cros_ec_force_data_role(info, *val);
		break;
	case DUAL_ROLE_PROP_MODE:
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum dual_role_property extcon_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
};

static struct dual_role_phy_desc extcon_drp_desc = {
		.name = "otg_default",
		.supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP,
		.properties = extcon_drp_properties,
		.num_properties = ARRAY_SIZE(extcon_drp_properties),
		.get_property = extcon_drp_get_prop,
		.set_property = extcon_drp_set_prop,
		.property_is_writeable = extcon_drp_is_writeable,
};

static int extcon_cros_ec_probe(struct platform_device *pdev)
{
	struct cros_ec_extcon_info *info;
	struct cros_ec_device *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	unsigned int numports = 0;
	int ret = 0;

	ret = cros_ec_pd_get_num_ports(dev, ec, &numports);
	if (ret) {
		dev_err(dev, "failed getting number of ports! ret = %d\n", ret);
		return -ENODEV;
	}

	if (numports != 1) {
		/* If we don't have exactly one port, fail */
		dev_err(dev, "This driver only supports exactly one port\n");
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->ec = ec;

	info->edev = devm_extcon_dev_allocate(dev, usb_type_c_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	info->dr = DUAL_ROLE_PROP_DR_NONE;
	info->pr = DUAL_ROLE_PROP_PR_NONE;
	init_waitqueue_head(&info->role_wait);

	platform_set_drvdata(pdev, info);

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		struct dual_role_phy_instance *inst;

		inst = devm_dual_role_instance_register(dev, &extcon_drp_desc);
		inst->drv_data = info;
		info->drp_inst = inst;
	}

	/* Get PD events from the EC */
	info->notifier.notifier_call = extcon_cros_ec_event;
	ret = blocking_notifier_chain_register(&info->ec->event_notifier,
					       &info->notifier);
	if (ret < 0)
		dev_warn(dev, "failed to register notifier\n");

	/* Initialize wakelock to hold off suspend when USB is attached */
	wake_lock_init(&info->wakelock, WAKE_LOCK_SUSPEND,
		       dev_name(&pdev->dev));
	info->wakelock_held = false;

	/* Perform initial detection */
	extcon_cros_ec_detect_cable(info);

	return 0;
}


static int extcon_cros_ec_remove(struct platform_device *pdev)
{
	struct cros_ec_extcon_info *info = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF))
		devm_dual_role_instance_unregister(dev, info->drp_inst);

	blocking_notifier_chain_unregister(&info->ec->event_notifier,
					   &info->notifier);

	if (info->wakelock_held)
		wake_unlock(&info->wakelock);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id extcon_cros_ec_of_match[] = {
	{ .compatible = "google,extcon-cros-ec" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, extcon_cros_ec_of_match);
#endif /* CONFIG_OF */

static struct platform_driver extcon_cros_ec_driver = {
	.driver = {
		.name  = "extcon-cros-ec",
		.of_match_table = of_match_ptr(extcon_cros_ec_of_match),
	},
	.remove  = extcon_cros_ec_remove,
	.probe   = extcon_cros_ec_probe,
};

module_platform_driver(extcon_cros_ec_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS Embedded Controller extcon driver");
