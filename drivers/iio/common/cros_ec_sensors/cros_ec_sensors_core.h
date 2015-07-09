/*
 * ChromeOS EC sensor hub
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
 */

#ifndef __CROS_EC_SENSORS_CORE_H
#define __CROS_EC_SENSORS_CORE_H

enum {
	X,
	Y,
	Z,
	MAX_AXIS,
};

/* State data for ec_sensors iio driver. */
struct cros_ec_sensors_core_state {
	struct cros_ec_device *ec;
	/*
	 *  Location to store command and response to the EC.
	 */
	struct mutex cmd_lock;

	/*
	 * Statically allocated command structure that holds parameters
	 * and response.
	 */
	struct cros_ec_command *msg;
	struct ec_params_motion_sense param;
	struct ec_response_motion_sense *resp;

};

/* Basic initialization of the core structure. */
int cros_ec_sensors_core_init(struct platform_device *pdev,
			      struct cros_ec_sensors_core_state *state);
/*
 * send_motion_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
int send_motion_host_cmd(struct cros_ec_sensors_core_state *state);

#endif  /* __CROS_EC_SENSORS_CORE_H */

