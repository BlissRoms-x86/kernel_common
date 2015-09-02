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
#include <linux/wakelock.h>

#define CROS_EC_USB_POLLING_DELAY msecs_to_jiffies(1000)

enum cros_ec_data_role {
	DATA_ROLE_DISCONNECTED = 0,
	DATA_ROLE_UFP = 1,
	DATA_ROLE_DFP = 2
};

struct cros_ec_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	struct cros_ec_device *ec;

	struct notifier_block notifier;
	struct wake_lock wakelock;
	bool wakelock_held;

	unsigned int role;
	unsigned int power_type;
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
 * cros_ec_usb_get_role() - Get role info aboout possible PD device attached to a
 * given port.
 * @dev: PD device
 * @ec_dev: pointer to cros_ec_device structure to talk to the physical device
 * @port: Port # on device
 * @role: Holds DATA_ROLE_DFP, DATA_ROLE_UFP, or DATA_ROLE_DISCONNECTED
 *	  on command success
 *
 * Return: 0 on success, <0 on failure.
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
	ret = cros_ec_pd_command(dev, ec_dev, EC_CMD_USB_PD_CONTROL, 1,
				 (uint8_t *)&pd_control, sizeof(pd_control),
				 (uint8_t *)&resp, sizeof(resp));
	if (ret < 0)
		return ret;

	if (resp.enabled & (1 << 1))
		*role = (resp.role & (1 << 1)) ? DATA_ROLE_DFP : DATA_ROLE_UFP;
	else
		*role = DATA_ROLE_DISCONNECTED;

	return ret;
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
	return DATA_ROLE_DISCONNECTED == role ? "DISCONNECTED" :
		(DATA_ROLE_DFP == role ? "DFP" : "UFP");
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

static bool cros_ec_usb_power_type_is_wall_wart(unsigned int type)
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
	case USB_CHG_TYPE_C:
	case USB_CHG_TYPE_PD:
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

static void extcon_cros_ec_detect_cable(struct cros_ec_extcon_info *info)
{
	struct device *dev = info->dev;
	struct cros_ec_device *ec = info->ec;
	int err;
	unsigned int role, power_type;

	err = cros_ec_usb_get_power_type(dev, ec, 0, &power_type);
	if (err) {
		dev_err(dev, "failed getting power type err = %d\n", err);
		return;
	}

	err = cros_ec_usb_get_role(dev, ec, 0, &role);
	if (err) {
		dev_err(dev, "failed getting role err = %d\n", err);
		return;
	}

	if (info->role != role || info->power_type != power_type) {
		bool host_connected = false, device_connected = false;

		dev_dbg(dev, "Type/Role switch! type = %s role = %s\n",
			cros_ec_usb_power_type_string(power_type),
			cros_ec_usb_role_string(role));
		info->role = role;
		info->power_type = power_type;

		if (role == DATA_ROLE_UFP &&
		    !cros_ec_usb_power_type_is_wall_wart(power_type))
			device_connected = true;
		else if (role == DATA_ROLE_DFP)
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

	info->role = DATA_ROLE_DISCONNECTED;

	platform_set_drvdata(pdev, info);

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
