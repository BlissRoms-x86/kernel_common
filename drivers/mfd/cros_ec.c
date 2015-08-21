/*
 * ChromeOS EC multi-function device
 *
 * Copyright (C) 2012 Google, Inc
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
 * The ChromeOS EC multi function device is used to mux all the requests
 * to the EC device for its multiple features: keyboard controller,
 * battery charging and regulator control, firmware update.
 */

#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/cros_ec.h>

#include <asm/unaligned.h>

#define CROS_EC_DEV_EC_INDEX 0
#define CROS_EC_DEV_PD_INDEX 1

static struct cros_ec_dev_platform ec_p = {
	.ec_name = CROS_EC_DEV_NAME,
	.cmd_offset = EC_CMD_PASSTHRU_OFFSET(CROS_EC_DEV_EC_INDEX),
};

static int cros_ec_get_next_event(struct cros_ec_device *ec_dev)
{
	struct cros_ec_command *msg;
	struct ec_response_get_next_event *event;
	int ret;

	if (ec_dev->suspended) {
		dev_dbg(ec_dev->dev, "Device suspended.\n");
		return -EHOSTDOWN;
	}

	msg = kzalloc(sizeof(*msg) + sizeof(*event), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->version = 0;
	msg->command = EC_CMD_GET_NEXT_EVENT;
	msg->insize = ec_dev->max_response;

	event = (struct ec_response_get_next_event *)msg->data;

	ret = cros_ec_cmd_xfer(ec_dev, msg);
	if (ret > 0) {
		ec_dev->event_size = ret - 1;
		ec_dev->event_data = *event;
	}
	return ret;
}

static int cros_ec_get_keyboard_state_event(struct cros_ec_device *ec_dev)
{
	union ec_response_get_next_data *data;
	struct cros_ec_command *msg;
	int ret;

	msg = kzalloc(sizeof(*msg) + sizeof(*data), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->version = 0;
	msg->command = EC_CMD_MKBP_STATE;
	msg->insize = sizeof(ec_dev->event_data.data);

	data = (union ec_response_get_next_data *)msg->data;

	ec_dev->event_data.event_type = EC_MKBP_EVENT_KEY_MATRIX;

	ret = cros_ec_cmd_xfer(ec_dev, msg);
	if (ret > 0)
		ec_dev->event_data.data = *data;

	ec_dev->event_size = ret;
	return ret;
}

u32 cros_ec_get_host_event(struct cros_ec_device *ec_dev)
{
	u32 host_event;
	
	BUG_ON(!ec_dev->mkbp_event_supported);
	if (ec_dev->event_data.event_type != EC_MKBP_EVENT_HOST_EVENT)
		return 0;
	if (ec_dev->event_size != sizeof(host_event)) {
		dev_warn(ec_dev->dev, "Invalid host event size\n");
		return 0;
	}
	
	host_event = get_unaligned_le32(&ec_dev->event_data.data.host_event);
	return host_event;
}
EXPORT_SYMBOL(cros_ec_get_host_event);

static irqreturn_t ec_irq_thread(int irq, void *data)
{
	struct cros_ec_device *ec_dev = data;
	int ret;

	if (device_may_wakeup(ec_dev->dev))
		pm_wakeup_event(ec_dev->dev, 0);

	if (ec_dev->mkbp_event_supported)
		ret = cros_ec_get_next_event(ec_dev);
	else
		ret = cros_ec_get_keyboard_state_event(ec_dev);

	if (ret > 0)
		blocking_notifier_call_chain(&ec_dev->event_notifier,
					     0, ec_dev);
	return IRQ_HANDLED;
}

static struct cros_ec_dev_platform pd_p = {
	.ec_name = CROS_EC_DEV_PD_NAME,
	.cmd_offset = EC_CMD_PASSTHRU_OFFSET(CROS_EC_DEV_PD_INDEX),
};

static const struct mfd_cell ec_cell = {
	.name = "cros-ec-ctl",
	.platform_data = &ec_p,
	.pdata_size = sizeof(ec_p),
};

static const struct mfd_cell ec_pd_cell = {
	.name = "cros-ec-ctl",
	.platform_data = &pd_p,
	.pdata_size = sizeof(pd_p),
};

int cros_ec_register(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;
	int err = 0;

	BLOCKING_INIT_NOTIFIER_HEAD(&ec_dev->event_notifier);

	ec_dev->max_request = sizeof(struct ec_params_hello);
	ec_dev->max_response = sizeof(struct ec_response_get_protocol_info);
	ec_dev->max_passthru = 0;

	ec_dev->din = devm_kzalloc(dev, ec_dev->din_size, GFP_KERNEL);
	if (!ec_dev->din)
		return -ENOMEM;

	ec_dev->dout = devm_kzalloc(dev, ec_dev->dout_size, GFP_KERNEL);
	if (!ec_dev->dout)
		return -ENOMEM;

	mutex_init(&ec_dev->lock);

	cros_ec_query_all(ec_dev);

	if (ec_dev->irq) {
		err = request_threaded_irq(ec_dev->irq, NULL, ec_irq_thread,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "chromeos-ec", ec_dev);
		if (err) {
			dev_err(dev, "request irq %d: error %d\n",
				ec_dev->irq, err);
			return err;
		}
	}

	err = mfd_add_devices(ec_dev->dev, PLATFORM_DEVID_AUTO, &ec_cell, 1,
			      NULL, ec_dev->irq, NULL);
	if (err) {
		dev_err(dev,
			"Failed to register Embedded Controller subdevice %d\n",
			err);
		goto mfd_err;
	}

	if (ec_dev->max_passthru) {
		/*
		 * Register a PD device as well on top of this device.
		 * We make the following assumptions:
		 * - behind an EC, we have a pd
		 * - only one device added.
		 * - the EC is responsive at init time (it is not true for a
		 *   sensor hub.
		 */
		err = mfd_add_devices(ec_dev->dev, PLATFORM_DEVID_AUTO,
				      &ec_pd_cell, 1, NULL, ec_dev->irq, NULL);
		if (err) {
			dev_err(dev,
				"Failed to register Power Delivery subdevice %d\n",
				err);
			goto mfd_err;
		}
	}

	if (IS_ENABLED(CONFIG_OF) && dev->of_node) {
		err = of_platform_populate(dev->of_node, NULL, NULL, dev);
		if (err) {
			mfd_remove_devices(dev);
			dev_err(dev, "Failed to register sub-devices\n");
			goto mfd_err;
		}
	}

	dev_info(dev, "Chrome EC device registered\n");

	return 0;

mfd_err:
      	if (ec_dev->irq)
		free_irq(ec_dev->irq, ec_dev);
	return err;
}
EXPORT_SYMBOL(cros_ec_register);

int cros_ec_remove(struct cros_ec_device *ec_dev)
{
	mfd_remove_devices(ec_dev->dev);

	return 0;
}
EXPORT_SYMBOL(cros_ec_remove);

#ifdef CONFIG_PM_SLEEP
int cros_ec_suspend(struct cros_ec_device *ec_dev)
{
	struct device *dev = ec_dev->dev;

	if (device_may_wakeup(dev))
		ec_dev->wake_enabled = !enable_irq_wake(ec_dev->irq);

	disable_irq(ec_dev->irq);
	ec_dev->was_wake_device = ec_dev->wake_enabled;
	ec_dev->suspended = true;

	return 0;
}
EXPORT_SYMBOL(cros_ec_suspend);

static void cros_ec_drain_events(struct cros_ec_device *ec_dev)
{
	while (cros_ec_get_next_event(ec_dev) > 0)
		blocking_notifier_call_chain(&ec_dev->event_notifier,
					     1, ec_dev);
}

int cros_ec_resume(struct cros_ec_device *ec_dev)
{
	ec_dev->suspended = false;
	enable_irq(ec_dev->irq);

	/*
	 * In some case, we need to distinguish events that occur during
	 * suspend if the EC is not a wake source. For example, keypresses
	 * during suspend should be discarded if it does not wake the system.
	 *
	 * If the EC is not a wake source, drain the event queue and mark them
	 * as "queued during suspend".
	 */
	if (ec_dev->wake_enabled) {
		disable_irq_wake(ec_dev->irq);
		ec_dev->wake_enabled = 0;
	} else {
		cros_ec_drain_events(ec_dev);
	}

	return 0;
}
EXPORT_SYMBOL(cros_ec_resume);

#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS EC core driver");
