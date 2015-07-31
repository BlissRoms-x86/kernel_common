/*
 * cros_ec_sensors_core - Common function for Chrome OS EC sensor driver.
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
 * This driver uses the cros-ec interface to communicate with the Chrome OS
 * EC about accelerometer data. Accelerometer access is presented through
 * iio sysfs.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include "cros_ec_sensors_core.h"

int cros_ec_sensors_core_init(struct platform_device *pdev,
			      struct cros_ec_sensors_core_state *state)
{
	struct cros_ec_dev *ec = dev_get_drvdata(pdev->dev.parent);

	state->ec = ec->ec_dev;
	state->msg = devm_kzalloc(&pdev->dev,
				  max((u16)sizeof(struct ec_params_motion_sense),
				      state->ec->max_response), GFP_KERNEL);
	if (state->msg == NULL)
		return -ENOMEM;

	state->resp = (struct ec_response_motion_sense *)state->msg->data;

	mutex_init(&state->cmd_lock);
	/* Set up the host command structure. */
	state->msg->version = 2;
	state->msg->command = EC_CMD_MOTION_SENSE_CMD + ec->cmd_offset;
	state->msg->outsize = sizeof(struct ec_params_motion_sense);
	state->msg->insize = state->ec->max_response;

	return 0;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_core_init);

/*
 * send_motion_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
int send_motion_host_cmd(struct cros_ec_sensors_core_state *state)
{
	int ret;

	memcpy(state->msg->data, &state->param, sizeof(state->param));
	/* Send host command. */
	ret = cros_ec_cmd_xfer_status(state->ec, state->msg);
	if (ret <= 0)
		return -EIO;

	if (state->resp != (struct ec_response_motion_sense *)state->msg->data)
		memcpy(state->resp, state->msg->data, ret);

	return 0;
}
EXPORT_SYMBOL_GPL(send_motion_host_cmd);
