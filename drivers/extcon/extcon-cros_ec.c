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
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

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

	struct delayed_work poll_work;

	unsigned int role;
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

static void extcon_cros_ec_detect_cable(struct work_struct *work)
{
	struct cros_ec_extcon_info *info;
	struct device *dev;
	struct cros_ec_device *ec;
	int err;
	unsigned int role;

	info = container_of(to_delayed_work(work),
			    struct cros_ec_extcon_info,
			    poll_work);
	dev = info->dev;
	ec = info->ec;

	err = cros_ec_usb_get_role(dev, ec, 0, &role);
	if (err) {
		dev_err(dev, "failed getting role err = %d\n", err);
		goto resched;
	}

	if (info->role != role) {
		bool host_connected = false, device_connected = false;

		dev_dbg(dev, "role switch! role = %s\n",
			cros_ec_usb_role_string(role));
		info->role = role;

		if (role == DATA_ROLE_UFP)
			device_connected = true;
		else if (role == DATA_ROLE_DFP)
			host_connected = true;

		extcon_set_cable_state_(info->edev, EXTCON_USB_HOST,
					host_connected);
		extcon_set_cable_state_(info->edev, EXTCON_USB,
					device_connected);
	}

resched:
	schedule_delayed_work(&info->poll_work, CROS_EC_USB_POLLING_DELAY);
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

	/* Poll for data role state periodically */
	INIT_DELAYED_WORK(&info->poll_work,
			  extcon_cros_ec_detect_cable);

	/* Perform initial detection */
	extcon_cros_ec_detect_cable(&info->poll_work.work);

	return 0;
}


static int extcon_cros_ec_remove(struct platform_device *pdev)
{
	struct cros_ec_extcon_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->poll_work);

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
