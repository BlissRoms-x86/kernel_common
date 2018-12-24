/*
 *  surface_i2c.c - Microsoft Surface I2C Driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  The full GNU General Public License is included in this distribution in
 *  the file called "COPYING".
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/uuid.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <asm/unaligned.h>

#include "surface_acpi.h"

#define POLL_INTERVAL		(HZ * 2)

struct surface_i2c_data {
	struct i2c_client *adp1;
	struct i2c_client *bat0;
	unsigned short notify_version;
	struct task_struct *poll_task;
	bool kthread_running;
	bool charging;
	bool bat_charging;
	u8 trip_point;
	s32 full_capacity;
};

struct surface_i2c_lookup {
	struct surface_i2c_data	*cdata;
	unsigned int n;
	unsigned int index;
	int addr;
};

struct surface_i2c_handler_data {
	struct acpi_connection_info info;
	struct i2c_client *client;
};

struct bix {
	u32	revision;
	u32	power_unit;
	u32	design_capacity;
	u32	last_full_charg_capacity;
	u32	battery_technology;
	u32	design_voltage;
	u32	design_capacity_of_warning;
	u32	design_capacity_of_low;
	u32	cycle_count;
	u32	measurement_accuracy;
	u32	max_sampling_time;
	u32	min_sampling_time;
	u32	max_average_interval;
	u32	min_average_interval;
	u32	battery_capacity_granularity_1;
	u32	battery_capacity_granularity_2;
	char model[10];
	char serial[10];
	char type[10];
	char OEM[10];
} __packed;

struct bst {
	u32 battery_state;
	s32 battery_present_rate;
	u32 battery_remaining_capacity;
	u32 battery_present_voltage;
} __packed;

struct gsb_command {
	u8 arg0;
	u8 arg1;
	u8 arg2;
} __packed;

struct gsb_buffer {
	u8 status;
	u8 len;
	u8 ret;
	union {
		struct gsb_command cmd;
		struct bst bst;
		struct bix bix;
	} __packed;
} __packed;

#define ACPI_BATTERY_STATE_DISCHARGING	0x1
#define ACPI_BATTERY_STATE_CHARGING		0x2
#define ACPI_BATTERY_STATE_CRITICAL		0x4

#define surface_i2c_CMD_DEST_BAT0		0x01
#define surface_i2c_CMD_DEST_ADP1		0x03

#define surface_i2c_CMD_BAT0_STA		0x01
#define surface_i2c_CMD_BAT0_BIX		0x02
#define surface_i2c_CMD_BAT0_BCT		0x03
#define surface_i2c_CMD_BAT0_BTM		0x04
#define surface_i2c_CMD_BAT0_BST		0x05
#define surface_i2c_CMD_BAT0_BTP		0x06
#define surface_i2c_CMD_ADP1_PSR		0x07
#define surface_i2c_CMD_BAT0_PSOC		0x09
#define surface_i2c_CMD_BAT0_PMAX		0x0A
#define surface_i2c_CMD_BAT0_PSRC		0x0B
#define surface_i2c_CMD_BAT0_CHGI		0x0C
#define surface_i2c_CMD_BAT0_ARTG		0x0D

#define surface_i2c_NOTIFY_GET_VERSION	0x00
#define surface_i2c_NOTIFY_ADP1			0x01
#define surface_i2c_NOTIFY_BAT0_BST		0x02
#define surface_i2c_NOTIFY_BAT0_BIX		0x05

#define surface_i2c_ADP1_REG_PSR		0x03

#define surface_i2c_BAT0_REG_CAPACITY			0x0c
#define surface_i2c_BAT0_REG_FULL_CHG_CAPACITY	0x0e
#define surface_i2c_BAT0_REG_DESIGN_CAPACITY	0x40
#define surface_i2c_BAT0_REG_VOLTAGE			0x08
#define surface_i2c_BAT0_REG_RATE				0x14
#define surface_i2c_BAT0_REG_OEM				0x45
#define surface_i2c_BAT0_REG_TYPE				0x4e
#define surface_i2c_BAT0_REG_SERIAL_NO			0x56
#define surface_i2c_BAT0_REG_CYCLE_CNT			0x6e

#define surface_i2c_EV_2_5				0x1ff

static int surface_i2c_read_block(struct i2c_client *client, u8 reg, u8 *buf,
				   int len)
{
	int status, i;

	for (i = 0; i < len; i++) {
		status = i2c_smbus_read_byte_data(client, reg + i);
		if (status < 0) {
			buf[i] = 0xff;
			continue;
		}

		buf[i] = (u8)status;
	}

	return 0;
}

static int
surface_i2c_notify(struct surface_i2c_data *cdata, u8 arg1, u8 arg2,
		unsigned int *ret_value)
{
	/*static const guid_t surface_i2c_guid =
	GUID_INIT(0x93b666c5, 0x70c6, 0x469f,
		  0xa2, 0x15, 0x3d, 0x48, 0x7c, 0x91, 0xab, 0x3c);*/

	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status;

	handle = ACPI_HANDLE(&cdata->adp1->dev);
	if (!handle || acpi_bus_get_device(handle, &adev))
		return -ENODEV;

	*ret_value = 0;

	status = surface_acpi_event_handler(arg2);
	if (ACPI_FAILURE(status)) {
		pr_err("surface_i2c: ACPI event failure status %s\n",
					acpi_format_exception(status));
	}

	return 0;
}

static const struct bix default_bix = {
	.revision = 0x00,
	.power_unit = 0x00,
	.design_capacity = 0x1734,
	.last_full_charg_capacity = 0x1734,
	.battery_technology = 0x01,
	.design_voltage = 0x1d92,
	.design_capacity_of_warning = 0xc8,
	.design_capacity_of_low = 0xc8,
	.battery_capacity_granularity_1 = 0x45,
	.battery_capacity_granularity_2 = 0x11,
	.cycle_count = 0x01,
	.measurement_accuracy = 0x00015F90,
	.max_sampling_time = 0x03E8,
	.min_sampling_time = 0x03E8,
	.max_average_interval = 0x03E8,
	.min_average_interval = 0x03E8,
	.model = "PNP0C0A",
	.serial = "1234567890",
	.type = "SDS-BAT",
	.OEM = "MICROSOFT",
};

static int surface_i2c_bix(struct surface_i2c_data *cdata, struct bix *bix)
{
	struct i2c_client *client = cdata->bat0;
	int ret;
	char buf[10];

	*bix = default_bix;

	/* get design capacity */
	ret = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_DESIGN_CAPACITY);
	if (ret < 0) {
		dev_err(&client->dev, "Error reading design capacity: %d\n", ret);
		return ret;
	}
	bix->design_capacity = le16_to_cpu(ret);

	/* get last full charge capacity */
	ret = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_FULL_CHG_CAPACITY);
	if (ret < 0) {
		dev_err(&client->dev, "Error reading last full charge capacity: %d\n", ret);
		return ret;
	}
	bix->last_full_charg_capacity = le16_to_cpu(ret);

	/* get serial number */
	ret = surface_i2c_read_block(client, surface_i2c_BAT0_REG_SERIAL_NO,
				      buf, 10);
	if (ret) {
		dev_err(&client->dev, "Error reading serial no: %d\n", ret);
		return ret;
	}
	memcpy(bix->serial, buf + 7, 3);
	memcpy(bix->serial + 3, buf, 6);
	bix->serial[9] = '\0';

	/* get cycle count */
	ret = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_CYCLE_CNT);
	if (ret < 0) {
		dev_err(&client->dev, "Error reading cycle count: %d\n", ret);
		return ret;
	}
	bix->cycle_count = le16_to_cpu(ret);

	/* get OEM name */
	ret = surface_i2c_read_block(client, surface_i2c_BAT0_REG_OEM, buf, 4);
	if (ret) {
		dev_err(&client->dev, "Error reading cycle count: %d\n", ret);
		return ret;
	}
	memcpy(bix->OEM, buf, 3);
	bix->OEM[4] = '\0';

	return 0;
}

static int surface_i2c_bst(struct surface_i2c_data *cdata, struct bst *bst)
{
	struct i2c_client *client = cdata->bat0;
	int rate, capacity, voltage, state;
	s16 tmp;

	rate = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_RATE);
	if (rate < 0)
		return rate;

	capacity = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_CAPACITY);
	if (capacity < 0)
		return capacity;

	voltage = i2c_smbus_read_word_data(client, surface_i2c_BAT0_REG_VOLTAGE);
	if (voltage < 0)
		return voltage;

	tmp = le16_to_cpu(rate);
	bst->battery_present_rate = abs((s32)tmp);

	state = 0;
	if ((s32) tmp > 0)
		state |= ACPI_BATTERY_STATE_CHARGING;
	else if ((s32) tmp < 0)
		state |= ACPI_BATTERY_STATE_DISCHARGING;
	bst->battery_state = state;

	bst->battery_remaining_capacity = le16_to_cpu(capacity);
	bst->battery_present_voltage = le16_to_cpu(voltage);

	return 0;
}

static int surface_i2c_adp_psr(struct surface_i2c_data *cdata)
{
	struct i2c_client *client = cdata->adp1;
	int ret;

	ret = i2c_smbus_read_byte_data(client, surface_i2c_ADP1_REG_PSR);
	if (ret < 0)
		return ret;

	return ret;
}

static int surface_i2c_isr(struct surface_i2c_data *cdata)
{
	struct bst bst;
	struct bix bix;
	int ret;
	bool status, bat_status;

	ret = surface_i2c_adp_psr(cdata);
	if (ret < 0)
		return ret;

	status = ret;

	if (status != cdata->charging)
		surface_i2c_notify(cdata, cdata->notify_version,
				surface_i2c_NOTIFY_ADP1, &ret);

	cdata->charging = status;

	ret = surface_i2c_bst(cdata, &bst);
	if (ret < 0)
		return ret;

	bat_status = bst.battery_state;

	if (bat_status != cdata->bat_charging)
		surface_i2c_notify(cdata, cdata->notify_version,
				surface_i2c_NOTIFY_BAT0_BST, &ret);

	cdata->bat_charging = bat_status;

	ret = surface_i2c_bix(cdata, &bix);
	if (ret < 0)
		return ret;
	if (bix.last_full_charg_capacity != cdata->full_capacity)
		surface_i2c_notify(cdata, cdata->notify_version,
				surface_i2c_NOTIFY_BAT0_BIX, &ret);

	cdata->full_capacity = bix.last_full_charg_capacity;

	return 0;
}

static int surface_i2c_poll_task(void *data)
{
	struct surface_i2c_data *cdata = data;
	int ret = 0;

	cdata->kthread_running = true;

	set_freezable();

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(POLL_INTERVAL);
		try_to_freeze();
		ret = surface_i2c_isr(data);
		if (ret)
			goto out;
	}

out:
	cdata->kthread_running = false;
	return ret;
}

static acpi_status
surface_i2c_space_handler(u32 function, acpi_physical_address command,
			u32 bits, u64 *value64,
			void *handler_context, void *region_context)
{
	struct gsb_buffer *gsb = (struct gsb_buffer *)value64;
	struct surface_i2c_handler_data *data = handler_context;
	struct acpi_connection_info *info = &data->info;
	struct acpi_resource_i2c_serialbus *sb;
	struct i2c_client *client = data->client;
	struct surface_i2c_data *cdata = i2c_get_clientdata(client);
	struct acpi_resource *ares;
	u32 accessor_type = function >> 16;
	acpi_status ret;
	int status = 1;

	ret = acpi_buffer_to_resource(info->connection, info->length, &ares);
	if (ACPI_FAILURE(ret))
		return ret;

	if (!value64 || ares->type != ACPI_RESOURCE_TYPE_SERIAL_BUS) {
		ret = AE_BAD_PARAMETER;
		goto err;
	}

	sb = &ares->data.i2c_serial_bus;
	if (sb->type != ACPI_RESOURCE_SERIAL_TYPE_I2C) {
		ret = AE_BAD_PARAMETER;
		goto err;
	}

	if (accessor_type != ACPI_GSB_ACCESS_ATTRIB_RAW_PROCESS) {
		ret = AE_BAD_PARAMETER;
		goto err;
	}

	if (gsb->cmd.arg0 == surface_i2c_CMD_DEST_ADP1 &&
	    gsb->cmd.arg1 == surface_i2c_CMD_ADP1_PSR) {
		ret = surface_i2c_adp_psr(cdata);
		if (ret >= 0) {
			status = ret;
			ret = 0;
		}
		goto out;
	}

	if (gsb->cmd.arg0 != surface_i2c_CMD_DEST_BAT0) {
		ret = AE_BAD_PARAMETER;
		goto err;
	}

	switch (gsb->cmd.arg1) {
	case surface_i2c_CMD_BAT0_STA:
		status = 1;
		ret = 0;
		break;
	case surface_i2c_CMD_BAT0_BIX:
		status = 1;
		ret = surface_i2c_bix(cdata, &gsb->bix);
		break;
	case surface_i2c_CMD_BAT0_BTP:
		status = 1;
		ret = 0;
		cdata->trip_point = gsb->cmd.arg2;
		break;
	case surface_i2c_CMD_BAT0_BST:
		status = 1;
		ret = surface_i2c_bst(cdata, &gsb->bst);
		break;
	default:
		pr_info("command(0x%02x) is not supported.\n", gsb->cmd.arg1);
		ret = AE_BAD_PARAMETER;
		goto err;
	}

 out:
	gsb->ret = status;
	gsb->status = 0;

 err:
	ACPI_FREE(ares);
	return ret;
}

static int surface_i2c_install_space_handler(struct i2c_client *client)
{
	acpi_handle handle;
	struct surface_i2c_handler_data *data;
	acpi_status status;

	handle = ACPI_HANDLE(&client->dev);

	if (!handle)
		return -ENODEV;

	data = kzalloc(sizeof(struct surface_i2c_handler_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	status = acpi_bus_attach_private_data(handle, (void *)data);
	if (ACPI_FAILURE(status)) {
		kfree(data);
		return -ENOMEM;
	}

	status = acpi_install_address_space_handler(handle,
				ACPI_ADR_SPACE_GSBUS,
				&surface_i2c_space_handler,
				NULL,
				data);
	if (ACPI_FAILURE(status)) {
		dev_err(&client->dev, "Error installing i2c space handler\n");
		acpi_bus_detach_private_data(handle);
		kfree(data);
		return -ENOMEM;
	}

	acpi_walk_dep_device_list(handle);
	return 0;
}

static void surface_i2c_remove_space_handler(struct i2c_client *client)
{
	acpi_handle handle;
	struct surface_i2c_handler_data *data;
	acpi_status status;

	handle = ACPI_HANDLE(&client->dev);

	if (!handle)
		return;

	acpi_remove_address_space_handler(handle,
				ACPI_ADR_SPACE_GSBUS,
				&surface_i2c_space_handler);

	status = acpi_bus_get_private_data(handle, (void **)&data);
	if (ACPI_SUCCESS(status))
		kfree(data);

	acpi_bus_detach_private_data(handle);
}

static int acpi_find_i2c(struct acpi_resource *ares, void *data)
{
	struct surface_i2c_lookup *lookup = data;

	if (ares->type != ACPI_RESOURCE_TYPE_SERIAL_BUS)
		return 1;

	if (lookup->n++ == lookup->index && !lookup->addr)
		lookup->addr = ares->data.i2c_serial_bus.slave_address;

	return 1;
}

static int surface_i2c_resource_lookup(struct surface_i2c_data *cdata,
					unsigned int index)
{
	struct i2c_client *client = cdata->adp1;
	struct acpi_device *adev = ACPI_COMPANION(&client->dev);
	struct surface_i2c_lookup lookup = {
		.cdata = cdata,
		.index = index,
	};
	struct list_head res_list;
	int ret;

	INIT_LIST_HEAD(&res_list);

	ret = acpi_dev_get_resources(adev, &res_list, acpi_find_i2c, &lookup);
	if (ret < 0)
		return ret;

	acpi_dev_free_resource_list(&res_list);

	if (!lookup.addr)
		return -ENOENT;

	return lookup.addr;
}

static void surface_i2c_dump_registers(struct i2c_client *client,
				    struct i2c_client *bat0)
{
	char rd_buf[60];
	int error, i, c;
	char buff[17 * 3 * 2] = {0};

	dev_info(&client->dev, "dumping registers 0x00 to 0x7F:\n");

	for (i = 0; i < 0x80; i += 0x20) {
		memset(rd_buf, 0, sizeof(rd_buf));
		error = surface_i2c_read_block(bat0, i, rd_buf, 0x20);
		dev_info(&client->dev, " read 0x%02x: %*ph|%*ph\n",
			 i,
			 0x10, rd_buf,
			 0x10, rd_buf + 0x10);
		for (c = 0; c < 0x20; c++) {
			if (rd_buf[c] >= 0x20 && rd_buf[c] <= 0x7e) {
				buff[c * 3 + 0] = ' ';
				buff[c * 3 + 1] = rd_buf[c];
			} else {
				buff[c * 3 + 0] = '-';
				buff[c * 3 + 1] = '-';
			}
			buff[c * 3 + 2] = (c + 1) % 0x10 ? ' ' : '|';
		}
		buff[0x1f * 3 + 2] = '\0';
		dev_info(&client->dev, "ascii 0x%02x: %s\n", i, buff);
	}
}

static int surface_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct i2c_client *bat0;
	struct surface_i2c_data *data;
	int error, version, addr;

	pr_info("surface_i2c: Probing for surface i2c device...\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->adp1 = client;
	i2c_set_clientdata(client, data);

	addr = surface_i2c_resource_lookup(data, 1);
	if (addr < 0)
		return addr;

	bat0 = i2c_new_dummy(client->adapter, addr);
	if (!bat0)
		return -ENOMEM;

	data->bat0 = bat0;
	i2c_set_clientdata(bat0, data);

	// debugging
	surface_i2c_dump_registers(client, bat0);

	pr_info("surface_i2c: Attaching device MSHW0124...");

	error = surface_i2c_notify(data, 1, surface_i2c_NOTIFY_GET_VERSION, &version);
	if (error)
		goto out_err;

	data->notify_version = version == surface_i2c_EV_2_5;

	data->poll_task = kthread_run(surface_i2c_poll_task, data, "surface_i2c_adp");
	if (IS_ERR(data->poll_task)) {
		error = PTR_ERR(data->poll_task);
		dev_err(&client->dev, "Unable to run kthread err %d\n", error);
		goto out_err;
	}

	//error = surface_i2c_install_space_handler(client);
	//if (error)
	//	goto out_err;

	return 0;

out_err:
	if (data->kthread_running)
		kthread_stop(data->poll_task);
	i2c_unregister_device(data->bat0);
	return error;
}

static int surface_i2c_remove(struct i2c_client *client)
{
	struct surface_i2c_data *cdata = i2c_get_clientdata(client);

	surface_i2c_remove_space_handler(client);

	if (cdata->kthread_running)
		kthread_stop(cdata->poll_task);

	i2c_unregister_device(cdata->bat0);

	return 0;
}

int surface_i2c_detect(struct i2c_client* client, struct i2c_board_info* board_info)
{
    pr_info("surface_i2c: Detecting surface_i2c device...");
    return 0;
}

static const struct acpi_device_id surface_i2c_acpi_match[] = {
	{ "MSHW0124", 0 },
	{ "", 0 }
};
MODULE_DEVICE_TABLE(acpi, surface_i2c_acpi_match);

static const struct i2c_device_id surface_i2c_id[] = {
	{ "MSHW0124:00", 0 },
	{ "MSHW0124:01", 0 },
	{ "", 0 }
};
MODULE_DEVICE_TABLE(i2c, surface_i2c_id);

static struct i2c_driver surface_i2c_driver = {
	.probe_new = surface_i2c_probe,
	.remove = surface_i2c_remove,
	.detect = surface_i2c_detect,
	.id_table = surface_i2c_id,
	.driver = {
		.name = "surface_i2c",
		.acpi_match_table = ACPI_PTR(surface_i2c_acpi_match),
	},
};
module_i2c_driver(surface_i2c_driver);

MODULE_AUTHOR("Jake Day");
MODULE_DESCRIPTION("Microsoft Surface I2C Driver");
MODULE_LICENSE("GPL");
