/*
 * cros_ec_sensors - Driver for Chrome OS Embedded Controller sensors.
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
#include <linux/iio/triggered_buffer.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include "cros_ec_sensors_core.h"

static char *cros_ec_loc[] = {
	[MOTIONSENSE_LOC_BASE] = "base",
	[MOTIONSENSE_LOC_LID] = "lid",
	[MOTIONSENSE_LOC_MAX] = "unknown",
};

#define MAX_CHANNELS (MAX_AXIS + 1)
#define SAMPLE_SIZE  (round_up(sizeof(s16) * MAX_AXIS, sizeof(s64)) + \
		      sizeof(s64))

/*
 * EC returns sensor values using signed 16 bit registers
 */
#define SENSOR_BITS 16

typedef int (*read_ec_sensors_data_t)(struct iio_dev *indio_dev,
		unsigned long scan_mask, s16 *data);

/* State data for ec_sensors iio driver. */
struct cros_ec_sensors_state {
	/* Shared by all sensors */
	struct cros_ec_sensors_core_state core;

	/* Type of sensor */
	enum motionsensor_type type;
	enum motionsensor_location loc;

	/*
	 * Calibration parameters. Note that trigger captured data will always
	 * provide the calibrated values.
	 */
	struct calib_data {
		u16 scale;
		s16 offset;
	} calib[MAX_AXIS];

	/*
	 * Static array to hold data from a single capture. For each
	 * channel we need 2 bytes, except for the timestamp. The timestamp
	 * is always last and is always 8-byte aligned.
	 */
	u8 samples[SAMPLE_SIZE];

	struct iio_chan_spec channels[MAX_CHANNELS];

	/* Pointer to function used for accessing sensors values. */
	read_ec_sensors_data_t read_ec_sensors_data;
};

/*
 * idx_to_reg - convert sensor index into offset in shared memory region.
 *
 * @st: private data
 * @idx: sensor index (should be element of enum sensor_index)
 * @return address to read at.
 */
static unsigned idx_to_reg(struct cros_ec_sensors_state *st, unsigned idx)
{
	/*
	 * When using LPC interface, only space for 2 Accel and one Gyro.
	 */
	if (st->type == MOTIONSENSE_TYPE_ACCEL)
		return EC_MEMMAP_ACC_DATA + sizeof(u16) *
			(idx + st->core.param.info.sensor_num *
			 MAX_AXIS);
	else
		return EC_MEMMAP_GYRO_DATA + sizeof(u16) * idx;
}

static int ec_cmd_read_u8(struct cros_ec_device *ec, unsigned int offset,
			  u8 *dest)
{
        return ec->cmd_readmem(ec, offset, 1, dest);
}

static int ec_cmd_read_u16(struct cros_ec_device *ec, unsigned int offset,
			   u16 *dest)
{
	u16 tmp;
	int ret = ec->cmd_readmem(ec, offset, 2, &tmp);

	*dest = le16_to_cpu(tmp);

	return ret;
}

/*
 * read_ec_until_not_busy - read from EC status byte until it reads not busy.
 *
 * @st Pointer to state information for device.
 * @return 8-bit status if ok, -ve on error
 */
static int read_ec_until_not_busy(struct cros_ec_sensors_state *st)
{
	struct cros_ec_device *ec = st->core.ec;
	u8 status;
	int attempts = 0;

	ec_cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	while (status & EC_MEMMAP_ACC_STATUS_BUSY_BIT) {
		/* Give up after enough attempts, return error. */
		if (attempts++ >= 50)
			return -EIO;

		/* Small delay every so often. */
		if (attempts % 5 == 0)
			msleep(25);

		ec_cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return status;
}

/*
 * read_ec_sensors_data_unsafe - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 *
 * Note this is the unsafe function for reading the EC data. It does not
 * guarantee that the EC will not modify the data as it is being read in.
 */
static void read_ec_sensors_data_unsafe(struct iio_dev *indio_dev,
			 unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	struct cros_ec_device *ec = st->core.ec;
	unsigned i = 0;

	/*
	 * Read all sensors enabled in scan_mask. Each value is 2
	 * bytes.
	 */
	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		ec_cmd_read_u16(ec, idx_to_reg(st, i), data);
		data++;
	}
}

/*
 * read_ec_sensors_data - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 * @return 0 if ok, -ve on error
 *
 * Note: this is the safe function for reading the EC data. It guarantees
 * that the data sampled was not modified by the EC while being read.
 */
static int read_ec_sensors_data_lpc(struct iio_dev *indio_dev,
			      unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	struct cros_ec_device *ec = st->core.ec;
	u8 samp_id = 0xff, status = 0;
	int attempts = 0;

	/*
	 * Continually read all data from EC until the status byte after
	 * all reads reflects that the EC is not busy and the sample id
	 * matches the sample id from before all reads. This guarantees
	 * that data read in was not modified by the EC while reading.
	 */
	while ((status & (EC_MEMMAP_ACC_STATUS_BUSY_BIT |
			  EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK)) != samp_id) {
		/* If we have tried to read too many times, return error. */
		if (attempts++ >= 5)
			return -EIO;

		/* Read status byte until EC is not busy. */
		status = read_ec_until_not_busy(st);
		if (status < 0)
			return status;

		/*
		 * Store the current sample id so that we can compare to the
		 * sample id after reading the data.
		 */
		samp_id = status & EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK;

		/* Read all EC data, format it, and store it into data. */
		read_ec_sensors_data_unsafe(indio_dev, scan_mask, data);

		/* Read status byte. */
		ec_cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return 0;
}

static int read_ec_sensors_data_cmd(struct iio_dev *indio_dev,
			      unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	int ret;
	unsigned i = 0;

	/*
	 * read all sensor data through a command.
	 */
	st->core.param.cmd = MOTIONSENSE_CMD_DATA;
	ret = send_motion_host_cmd(&st->core);
	if (ret != 0) {
		dev_warn(&indio_dev->dev, "Unable to read sensor data\n");
		return ret;
	}

	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		*data = st->core.resp->data.data[i];
		data++;
	}
	return 0;
}


static int ec_sensors_read(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	s16 data = 0;
	s64 val64;
	int i;
	int ret = IIO_VAL_INT;
	int idx = chan->scan_index;

	mutex_lock(&st->core.cmd_lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (st->read_ec_sensors_data(indio_dev, 1 << idx, &data) < 0)
			ret = -EIO;
		*val = (s16)data;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_OFFSET;
		st->core.param.sensor_offset.flags = 0;

		if (send_motion_host_cmd(&st->core)) {
			ret = -EIO;
			break;
		}

		/* Save values */
		for (i = X; i < MAX_AXIS; i++)
			st->calib[i + (idx / MAX_AXIS) * MAX_AXIS].offset =
			     st->core.resp->sensor_offset.offset[i];

		*val = st->calib[idx].offset;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->core.param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->core.param.ec_rate.data =
			EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		else
			*val = st->core.resp->ec_rate.ret;
		break;
	case IIO_CHAN_INFO_SCALE:
		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		st->core.param.sensor_range.data =
			EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(&st->core)) {
			ret = -EIO;
			break;
		}
		val64 = st->core.resp->sensor_range.ret;
		switch (st->type) {
		case MOTIONSENSE_TYPE_ACCEL:
			/*
			 * EC returns data in g, iio exepects m/s^2.
			 * Do not use IIO_G_TO_M_S_2 to avoid precision loss.
			 */
			*val = (val64 * 980665) / 10;
			*val2 = 10000 << (SENSOR_BITS - 1);
			ret = IIO_VAL_FRACTIONAL;
			break;
		case MOTIONSENSE_TYPE_GYRO:
			/* EC returns date in dps, iio expects rad/s.
			 * Do not use IIO_DEGREE_TO_RAD to avoid precision
			 * loss. Round to the nearest integer.
			 */
			*val = (val64 * 314159 + 9000000ULL) / 1000;
			*val2 = 18000 << (SENSOR_BITS - 1);
			ret = IIO_VAL_FRACTIONAL;
			break;
		case MOTIONSENSE_TYPE_MAG:
			/* EC returns date in 16LSB / uT, iio expects Gauss */
			*val = val64;
			*val2 = 100 << (SENSOR_BITS - 1);
			ret = IIO_VAL_FRACTIONAL;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->core.param.sensor_odr.data =
			EC_MOTION_SENSE_NO_VALUE;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		else
			*val = st->core.resp->sensor_odr.ret;
		break;
	default:
		break;
	}
	mutex_unlock(&st->core.cmd_lock);
	return ret;
}

static int ec_sensors_write(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	int i;
	int ret = 0;
	int idx = chan->scan_index;

	mutex_lock(&st->core.cmd_lock);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		st->calib[idx].offset = val;
		/* Send to EC only when writing Z axis. */
		if (idx % MAX_AXIS != Z)
			break;

		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_OFFSET;
		st->core.param.sensor_offset.flags =
			MOTION_SENSE_SET_OFFSET;
		for (i = X; i < MAX_AXIS; i++)
			st->core.param.sensor_offset.offset[i] =
				st->calib[i + (idx / MAX_AXIS) *
				MAX_AXIS].offset;
		st->core.param.sensor_offset.temp =
			EC_MOTION_SENSE_INVALID_CALIB_TEMP;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->core.param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->core.param.ec_rate.data = val;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		break;
	case IIO_CHAN_INFO_SCALE:
		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_RANGE;
		st->core.param.sensor_range.data = val;

		/* Always roundup, so caller gets at least what it asks for. */
		st->core.param.sensor_range.roundup = 1;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		st->core.param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->core.param.sensor_odr.data = val;

		/* Always roundup, so caller gets at least what it asks for. */
		st->core.param.sensor_odr.roundup = 1;

		if (send_motion_host_cmd(&st->core))
			ret = -EIO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->core.cmd_lock);
	return ret;
}

static const struct iio_info ec_sensors_info = {
	.read_raw = &ec_sensors_read,
	.write_raw = &ec_sensors_write,
	.driver_module = THIS_MODULE,
};

/*
 * accel_capture - the trigger handler function
 *
 * @irq: the interrupt number
 * @p: private data - always a pointer to the poll func.
 *
 * On a trigger event occurring, if the pollfunc is attached then this
 * handler is called as a threaded interrupt (and hence may sleep). It
 * is responsible for grabbing data from the device and pushing it into
 * the associated buffer.
 */
static irqreturn_t accel_capture(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);

	mutex_lock(&st->core.cmd_lock);
	/* Clear capture data. */
	memset(st->samples, 0, indio_dev->scan_bytes);

	/* Read data based on which channels are enabled in scan mask. */
	st->read_ec_sensors_data(indio_dev, *(indio_dev->active_scan_mask),
			   (s16 *)st->samples);

	/* Store the timestamp last 8 bytes of data. */
	if (indio_dev->scan_timestamp)
		*(s64 *)&st->samples[round_down(indio_dev->scan_bytes -
						sizeof(s64),
				     sizeof(s64))] = iio_get_time_ns();

	iio_push_to_buffers(indio_dev, st->samples);

	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->core.cmd_lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_IIO_CROS_EC_SENSORS_RING
static ssize_t cros_ec_sensors_flush(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		const char *buf, size_t len)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	int ret = 0;
	bool flush;

	ret = strtobool(buf, &flush);
	if (ret < 0)
		return ret;
	if (!flush)
		return -EINVAL;

	mutex_lock(&st->core.cmd_lock);
	st->core.param.cmd = MOTIONSENSE_CMD_FIFO_FLUSH;
	ret = send_motion_host_cmd(&st->core);
	if (ret != 0)
		dev_warn(&indio_dev->dev, "Unable to flush sensor\n");
	mutex_unlock(&st->core.cmd_lock);
	return ret ? ret : len;
}
#endif
static ssize_t cros_ec_sensors_calibrate(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		const char *buf, size_t len)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);
	int ret, i;
	bool calibrate;
	int idx = chan->scan_index;

	ret = strtobool(buf, &calibrate);
	if (ret < 0)
		return ret;
	if (!calibrate)
		return -EINVAL;

	mutex_lock(&st->core.cmd_lock);
	st->core.param.cmd = MOTIONSENSE_CMD_PERFORM_CALIB;
	ret = send_motion_host_cmd(&st->core);
	if (ret != 0) {
		dev_warn(&indio_dev->dev, "Unable to calibrate sensor\n");
	} else {
		/* Save values */
		for (i = X; i < MAX_AXIS; i++)
			st->calib[i + (idx / MAX_AXIS) * MAX_AXIS].offset =
			     st->core.resp->perform_calib.offset[i];
	}
	mutex_unlock(&st->core.cmd_lock);
	return ret ? ret : len;
}

static ssize_t cros_ec_sensors_id(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		char *buf)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->core.param.info.sensor_num);
}

static ssize_t cros_ec_sensors_loc(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		char *buf)
{
	struct cros_ec_sensors_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%s\n", cros_ec_loc[st->loc]);
}

static const struct iio_chan_spec_ext_info cros_ec_sensors_ring_info[] = {
#ifdef CONFIG_IIO_CROS_EC_SENSORS_RING
	{
		.name = "flush",
		.shared = IIO_SHARED_BY_ALL,
		.write = cros_ec_sensors_flush
	},
#endif
	{
		.name = "calibrate",
		.shared = IIO_SHARED_BY_ALL,
		.write = cros_ec_sensors_calibrate
	},
	{
		.name = "id",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_id
	},
	{
		.name = "location",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_loc
	},
	{ },
};


static int cros_ec_sensors_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cros_ec_dev *ec_dev = dev_get_drvdata(dev->parent);
	struct cros_ec_device *ec_device;
	struct cros_ec_sensor_platform *sensor_platform = dev_get_platdata(dev);
	struct iio_dev *indio_dev;
	struct cros_ec_sensors_state *state;
	struct iio_chan_spec *channel;
	int ret, i;

	if (!ec_dev || !ec_dev->ec_dev) {
		dev_warn(&pdev->dev, "No CROS EC device found.\n");
		return -EINVAL;
	}
	ec_device = ec_dev->ec_dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*state));
	if (indio_dev == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, indio_dev);

	state = iio_priv(indio_dev);
	ret = cros_ec_sensors_core_init(pdev, &state->core);
	if (ret)
		return ret;

	state->core.param.cmd = MOTIONSENSE_CMD_INFO;
	state->core.param.info.sensor_num =
		sensor_platform->sensor_num;
	if (send_motion_host_cmd(&state->core)) {
		dev_warn(dev, "Can not access sensor %d info\n", i);
		return -EIO;
	}
	state->type = state->core.resp->info.type;
	state->loc = state->core.resp->info.location;
	for (channel = state->channels, i = X; i < MAX_AXIS; i++, channel++) {
		switch (state->type) {
		case MOTIONSENSE_TYPE_ACCEL:
			/* Offset is in 1/1024 of a g */
			state->calib[i].scale = 10;
			channel->type = IIO_ACCEL;
			break;
		case MOTIONSENSE_TYPE_GYRO:
			/* Offset is in 1/1024 of a g */
			state->calib[i].scale = 10;
			channel->type = IIO_ANGL_VEL;
			break;
		case MOTIONSENSE_TYPE_MAG:
			channel->type = IIO_MAGN;
			break;
		default:
			dev_warn(&pdev->dev, "unknown\n");
		}
		channel->modified = 1;
		channel->info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_CALIBBIAS);
		channel->info_mask_shared_by_all =
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_FREQUENCY) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ);
		channel->scan_type.sign = 's';
		channel->scan_type.realbits = SENSOR_BITS;
		channel->scan_type.storagebits = SENSOR_BITS;
		channel->scan_type.shift = 0;
		channel->channel2 = IIO_MOD_X + i;
		channel->scan_index = i;
		channel->ext_info = cros_ec_sensors_ring_info;
		state->calib[i].offset = 0;
	}

	/* Timestamp */
	channel->type = IIO_TIMESTAMP;
	channel->channel = -1;
	channel->scan_index = MAX_AXIS;
	channel->scan_type.sign = 's';
	channel->scan_type.realbits = 64;
	channel->scan_type.storagebits = 64;

	indio_dev->channels = state->channels;
	indio_dev->num_channels = MAX_CHANNELS;

	/* There is only enough room for accel and gyro in the io space */
	if ((state->core.ec->cmd_readmem != NULL) &&
	    (state->type != MOTIONSENSE_TYPE_MAG))
		state->read_ec_sensors_data = read_ec_sensors_data_lpc;
	else
		state->read_ec_sensors_data = read_ec_sensors_data_cmd;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ec_sensors_info;
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 accel_capture, NULL);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_uninit_buffer;
	return 0;

error_uninit_buffer:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}

static int cros_ec_sensors_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	return 0;
}

static const struct platform_device_id cros_ec_sensors_ids[] = {
	{
		.name = "cros-ec-accel",
	},
	{
		.name = "cros-ec-gyro",
	},
	{
		.name = "cros-ec-mag",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, cros_ec_sensors_ids);

static struct platform_driver cros_ec_sensors_platform_driver = {
	.driver = {
		.name	= "cros-ec-sensors",
		.owner	= THIS_MODULE,
	},
	.probe		= cros_ec_sensors_probe,
	.remove		= cros_ec_sensors_remove,
	.id_table	= cros_ec_sensors_ids,
};
module_platform_driver(cros_ec_sensors_platform_driver);

MODULE_DESCRIPTION("ChromeOS EC sensor hub driver");
MODULE_LICENSE("GPL");
