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

#include <linux/irqreturn.h>

enum {
	X,
	Y,
	Z,
	MAX_AXIS,
};

/*
 * EC returns sensor values using signed 16 bit registers
 */
#define CROS_EC_SENSOR_BITS 16

/*
 * 4 16 bit channels are allowed.
 * Good enough for current sensors, thye use up to 3 16 bit vectors.
 */
#define CROS_EC_SAMPLE_SIZE  (sizeof(s64) * 2)

/*
 * minimum sampling period to use when device is suspending.
 */
#define CROS_EC_MIN_SUSPEND_SAMPLING_FREQUENCY 1000  /* 1 second */

/*
 * Function to read the sensor data.
 *
 * Data can be retrieve using the cros ec command protocol.
 * Some machines also allow accessing some sensor data via
 * IO space.
 */
typedef int (cros_ec_sensors_read_t)(struct iio_dev *indio_dev,
		unsigned long scan_mask, s16 *data);

cros_ec_sensors_read_t cros_ec_sensors_read_lpc;
cros_ec_sensors_read_t cros_ec_sensors_read_cmd;

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

	/* Type of sensor */
	enum motionsensor_type type;
	enum motionsensor_location loc;

	/*
	 * Calibration parameters. Note that trigger captured data will always
	 * provide the calibrated values.
	 */
	struct calib_data {
		s16 offset;
	} calib[MAX_AXIS];

	/*
	 * Static array to hold data from a single capture. For each
	 * channel we need 2 bytes, except for the timestamp. The timestamp
	 * is always last and is always 8-byte aligned.
	 */
	u8 samples[CROS_EC_SAMPLE_SIZE];

	/* Pointer to function used for accessing sensors values. */
	cros_ec_sensors_read_t *read_ec_sensors_data;

	/* Current sampling period */
	int curr_sampl_freq;
};

/* Basic initialization of the core structure. */
int cros_ec_sensors_core_init(struct platform_device *pdev,
			      struct iio_dev *indio_dev,
			      bool physical_device);

/*
 * cros_ec_sensors_capture - the trigger handler function
 *
 * @irq: the interrupt number
 * @p: private data - always a pointer to the poll func.
 *
 * On a trigger event occurring, if the pollfunc is attached then this
 * handler is called as a threaded interrupt (and hence may sleep). It
 * is responsible for grabbing data from the device and pushing it into
 * the associated buffer.
 */
irqreturn_t cros_ec_sensors_capture(int irq, void *p);


/*
 * cros_ec_motion_send_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @opt_length: optional length: to reduce the response size,
 *    useful on the data path.
 *    Otherwise, the maximal allowed response size is used.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
int cros_ec_motion_send_host_cmd(struct cros_ec_sensors_core_state *st,
				 u16 opt_length);

/*
 * cros_ec_sensors_core_read/write: handler for core attributes.
 *
 * Handler for attributes identical among sensors:
 * - frequency,
 * - sampling_frequency.
 *
 * cmd_lock lock must be held.
 */
int cros_ec_sensors_core_read(struct cros_ec_sensors_core_state *st,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask);

int cros_ec_sensors_core_write(struct cros_ec_sensors_core_state *st,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask);

extern const struct dev_pm_ops cros_ec_sensors_pm_ops;

/* List of extended channel specification for all sensors */
extern const struct iio_chan_spec_ext_info cros_ec_sensors_ext_info[];
extern const struct iio_chan_spec_ext_info cros_ec_sensors_limited_info[];

#endif  /* __CROS_EC_SENSORS_CORE_H */

