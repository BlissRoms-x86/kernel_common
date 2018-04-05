/*
 *  battery.c - ACPI Battery Driver (Revision: 2.0)
 *
 *  Copyright (C) 2007 Alexey Starikovskiy <astarikovskiy@suse.de>
 *  Copyright (C) 2004-2007 Vladimir Lebedev <vladimir.p.lebedev@intel.com>
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/async.h>
#include <linux/dmi.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <asm/unaligned.h>

#ifdef CONFIG_ACPI_PROCFS_POWER
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#endif

#include <linux/acpi.h>
//#include <linux/power/acpi.h>
#include <linux/power_supply.h>

#include "battery.h"

#define PREFIX "ACPI: "

#define ACPI_BATTERY_VALUE_UNKNOWN 0xFFFFFFFF

#define ACPI_BATTERY_DEVICE_NAME	"Battery"

/* Battery power unit: 0 means mW, 1 means mA */
#define ACPI_BATTERY_POWER_UNIT_MA	1

#define ACPI_BATTERY_STATE_DISCHARGING	0x1
#define ACPI_BATTERY_STATE_CHARGING	0x2
#define ACPI_BATTERY_STATE_CRITICAL	0x4

#define _COMPONENT		ACPI_BATTERY_COMPONENT


ACPI_MODULE_NAME("battery");

MODULE_AUTHOR("Paul Diefenbaugh");
MODULE_AUTHOR("Alexey Starikovskiy <astarikovskiy@suse.de>");
MODULE_AUTHOR("Nicole Faerber <nicole.faerber@id3p.com>");
MODULE_DESCRIPTION("ACPI Battery Driver");
MODULE_LICENSE("GPL");

enum init_state_enum { BAT_NONE, BAT_INITIALIZED, BAT_EXITED };

static enum init_state_enum init_state;
static DEFINE_MUTEX(init_state_mutex);
static async_cookie_t async_cookie;
static int battery_notification_delay_ms;
static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

#ifdef CONFIG_ACPI_PROCFS_POWER
extern struct proc_dir_entry *acpi_lock_battery_dir(void);
extern void *acpi_unlock_battery_dir(struct proc_dir_entry *acpi_battery_dir);

enum acpi_battery_files {
	info_tag = 0,
	state_tag,
	ACPI_BATTERY_NUMFILES,
};

#endif

static const struct acpi_device_id battery_device_ids[] = {
	{"PNP0C0A", 0},
	{"", 0},
};

MODULE_DEVICE_TABLE(acpi, battery_device_ids);

/* Lists of PMIC ACPI HIDs with an (often better) native battery driver */
static const char * const acpi_battery_blacklist[] = {
	"INT33F4", /* X-Powers AXP288 PMIC */
};

enum {
	ACPI_BATTERY_ALARM_PRESENT,
	ACPI_BATTERY_XINFO_PRESENT,
	ACPI_BATTERY_QUIRK_PERCENTAGE_CAPACITY,
	/* On Lenovo Thinkpad models from 2010 and 2011, the power unit
	   switches between mWh and mAh depending on whether the system
	   is running on battery or not.  When mAh is the unit, most
	   reported values are incorrect and need to be adjusted by
	   10000/design_voltage.  Verified on x201, t410, t410s, and x220.
	   Pre-2010 and 2012 models appear to always report in mWh and
	   are thus unaffected (tested with t42, t61, t500, x200, x300,
	   and x230).  Also, in mid-2012 Lenovo issued a BIOS update for
	   the 2011 models that fixes the issue (tested on x220 with a
	   post-1.29 BIOS), but as of Nov. 2012, no such update is
	   available for the 2010 models.  */
	ACPI_BATTERY_QUIRK_THINKPAD_MAH,
};

struct acpi_battery {
	struct mutex lock;
	struct mutex sysfs_lock;
	struct power_supply *bat;
	struct power_supply_desc bat_desc;
	struct acpi_device *device;
	struct notifier_block pm_nb;
	unsigned long update_time;
	int revision;
	int rate_now;
	int capacity_now;
	int voltage_now;
	int design_capacity;
	int full_charge_capacity;
	int technology;
	int design_voltage;
	int design_capacity_warning;
	int design_capacity_low;
	int cycle_count;
	int measurement_accuracy;
	int max_sampling_time;
	int min_sampling_time;
	int max_averaging_interval;
	int min_averaging_interval;
	int capacity_granularity_1;
	int capacity_granularity_2;
	int alarm;
	char model_number[32];
	char serial_number[32];
	char type[32];
	char oem_info[32];
	int state;
	int power_unit;
	unsigned long flags;
	struct i2c_adapter *i2c_adap;
	bool suspended;
};

#define to_acpi_battery(x) power_supply_get_drvdata(x)

/*
 * for ThinkPad Tablet 10 only,
 * ugly workaround strange ACPI operation region
 */
#include <linux/i2c.h>
#include <linux/delay.h>
/* the battery device sits on I2C bus 0 @0x70 */
#define TPT10BAT_I2C_ADDR	0x70
#define TPT10BAT_I2C_BUS	0

/* functions of the battery controller */
#define ECNR 0x80

static int tpt10_i2c_read(struct i2c_adapter *i2c_adap, u8 cmd, u8 arg0, u8 arg1, u8 *data, u8 data_len)
{
	struct i2c_msg msgs[2];
	u8 bufo[8];
	u8 bufi[8];
	int ret = 0;

	if (i2c_adap == NULL) {
		printk(KERN_ERR "  i2c_adap = NULL\n");
		return -ENODEV;
	}

	if (data_len > 8)
		data_len = 8;

	memset(bufo, 0, 8);
	memset(bufi, 0, 8);

	switch (cmd) {
	case ECNR:
		bufo[0] = 0x02; // len
		bufo[1] = ECNR; // cmd
		bufo[2] = arg0; // arg
		break;
	default:
		printk(KERN_ERR "tpt10bat: unknown I2C read cmd %d\n", cmd);
		break;
	}

	msgs[0].addr = TPT10BAT_I2C_ADDR;
	msgs[0].flags = 0;
        msgs[0].len = 5;
        msgs[0].buf = bufo;

        ret = i2c_transfer(i2c_adap, msgs, 1);
        if (ret < 0) {
                printk(KERN_ERR "i2c read failed\n");
                return ret;
	};

        msgs[0].addr = TPT10BAT_I2C_ADDR;
        msgs[0].flags = I2C_M_RD;
        msgs[0].len = 6;
        msgs[0].buf = bufi;

        ret = i2c_transfer(i2c_adap, msgs, 1);
        if (ret < 0) {
                printk(KERN_ERR "i2c read failed\n");
                return ret;
	} else
                memcpy(data, bufi, data_len);
	msleep(0x02);
        return ret;
}

static u8 bat_ec_rd(struct i2c_adapter *i2c_adap, u8 arg0)
{
	u8 data[8];
	int ret;

	ret = tpt10_i2c_read(i2c_adap, ECNR, arg0, 0, data, 2);

	return data[0];
}

static int tpt10_bat_get_status(struct acpi_battery *battery)
{
	unsigned int Local0, Local1, Local2, Local3, Local4, Local5, Local6, Local7;
	unsigned int PBST[4];
//	double rateW, remWh, volt, tbat;
#if 0
	int i;
#endif

	// do not try to update fif pm suspend has beend called before
	// else we might end up trying to access the already suspended
	// I2C bus
	if (battery->suspended)
		return 0;

	// this is basically a direct copy of the ACPI DSDT battery status method
	PBST[0] = 0x00;
	PBST[1] = 0xFFFFFFFF;
	PBST[2] = 0xFFFFFFFF;
	PBST[3] = 0xFFFFFFFF;

	Local0 = bat_ec_rd(battery->i2c_adap, 0xc1);
	PBST[0] = Local0;
	Local1 = bat_ec_rd(battery->i2c_adap, 0xd1);
	Local2 = bat_ec_rd(battery->i2c_adap, 0xd0) | (Local1 << 8);
	if (Local2 > 0x7FFF) {
		Local2 = (0x00010000 - Local2);
	}
	Local5 = bat_ec_rd(battery->i2c_adap, 0xc7);
	Local6 = bat_ec_rd(battery->i2c_adap, 0xc6) | (Local5 << 8);
	Local2 *= Local6;
	Local7 = Local2 % 0x03E8; // ??? Divide (Local2, 0x03E8, Local7, Local2)
	PBST[1] = Local2;
	Local3 = bat_ec_rd(battery->i2c_adap, 0xc3);
	Local4 = bat_ec_rd(battery->i2c_adap, 0xc2) | (Local3 << 8);
	Local4 *= 0x0a;
	PBST[2] = Local4;
	PBST[3] = Local6;

#if 0
	for (i=0; i<=3; i++) {
		printk(KERN_INFO "%1d:%9u 0x%08x\n", i, PBST[i], PBST[i]);
	}
	switch (PBST[0]) {
	case 0:
		printk(KERN_INFO "AC online   ");
		break;
	case 1:
		printk(KERN_INFO "discharging ");
		break;
	case 2:
		printk(KERN_INFO "charging    ");
		break;
	default:
		printk(KERN_INFO "unkknown %2d ", PBST[0]);
		break;
	}
#endif

#if 0
	rateW = (double)PBST[1] / 1000000.;
	remWh = (double)PBST[2] / 1000.;
	volt = (double)PBST[3] / 1000.;
	if (PBST[0] == 1) { // discharging, time till empty
		tbat = (remWh / rateW) * 60; // minutes
	} else
		tbat = 0.;
	printk(KERN_INFO "@ %2.2fW | Cap-remain %2.2fWh | Voltage %1.3fV | Remain %dh %dm\n", rateW, remWh, volt, (int)tbat / 60, (int)tbat % 60);
#endif
	battery->capacity_now = PBST[2];
	battery->voltage_now = PBST[3];
	battery->rate_now = PBST[1] / 1000;
	battery->state = PBST[0];

	return 0;
}


static inline int acpi_battery_present(struct acpi_battery *battery)
{
	return battery->device->status.battery_present;
}

static int acpi_battery_technology(struct acpi_battery *battery)
{
	if (!strcasecmp("NiCd", battery->type))
		return POWER_SUPPLY_TECHNOLOGY_NiCd;
	if (!strcasecmp("NiMH", battery->type))
		return POWER_SUPPLY_TECHNOLOGY_NiMH;
	if (!strcasecmp("LION", battery->type))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strncasecmp("LI-ION", battery->type, 6))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strcasecmp("LiP", battery->type))
		return POWER_SUPPLY_TECHNOLOGY_LIPO;
	return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static int acpi_battery_get_state(struct acpi_battery *battery);

static int acpi_battery_is_charged(struct acpi_battery *battery)
{
	/* charging, discharging or critical low */
	if (battery->state != 0)
		return 0;

	/* battery not reporting charge */
	if (battery->capacity_now == ACPI_BATTERY_VALUE_UNKNOWN ||
	    battery->capacity_now == 0)
		return 0;

	/* good batteries update full_charge as the batteries degrade */
	if (battery->full_charge_capacity == battery->capacity_now)
		return 1;

	/* fallback to using design values for broken batteries */
	if (battery->design_capacity == battery->capacity_now)
		return 1;

	/* we don't do any sort of metric based on percentages */
	return 0;
}

static int acpi_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct acpi_battery *battery = to_acpi_battery(psy);

	if (acpi_battery_present(battery)) {
		/* run battery update only if it is present */
		acpi_battery_get_state(battery);
	} else if (psp != POWER_SUPPLY_PROP_PRESENT)
		return -ENODEV;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (battery->state & ACPI_BATTERY_STATE_DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (battery->state & ACPI_BATTERY_STATE_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (acpi_battery_is_charged(battery))
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = acpi_battery_present(battery);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = acpi_battery_technology(battery);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = battery->cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (battery->design_voltage == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->design_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (battery->voltage_now == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->voltage_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_POWER_NOW:
		if (battery->rate_now == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->rate_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (battery->design_capacity == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->design_capacity * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (battery->full_charge_capacity == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->full_charge_capacity * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (battery->capacity_now == ACPI_BATTERY_VALUE_UNKNOWN)
			ret = -ENODEV;
		else
			val->intval = battery->capacity_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battery->capacity_now && battery->full_charge_capacity)
			val->intval = battery->capacity_now * 100/
					battery->full_charge_capacity;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (battery->state & ACPI_BATTERY_STATE_CRITICAL)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (test_bit(ACPI_BATTERY_ALARM_PRESENT, &battery->flags) &&
			(battery->capacity_now <= battery->alarm))
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (acpi_battery_is_charged(battery))
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = battery->model_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = battery->oem_info;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = battery->serial_number;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static enum power_supply_property charge_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static enum power_supply_property energy_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

/* --------------------------------------------------------------------------
                               Battery Management
   -------------------------------------------------------------------------- */
struct acpi_offsets {
	size_t offset;		/* offset inside struct acpi_sbs_battery */
	u8 mode;		/* int or string? */
};

static const struct acpi_offsets state_offsets[] = {
	{offsetof(struct acpi_battery, state), 0},
	{offsetof(struct acpi_battery, rate_now), 0},
	{offsetof(struct acpi_battery, capacity_now), 0},
	{offsetof(struct acpi_battery, voltage_now), 0},
};

static const struct acpi_offsets info_offsets[] = {
	{offsetof(struct acpi_battery, power_unit), 0},
	{offsetof(struct acpi_battery, design_capacity), 0},
	{offsetof(struct acpi_battery, full_charge_capacity), 0},
	{offsetof(struct acpi_battery, technology), 0},
	{offsetof(struct acpi_battery, design_voltage), 0},
	{offsetof(struct acpi_battery, design_capacity_warning), 0},
	{offsetof(struct acpi_battery, design_capacity_low), 0},
	{offsetof(struct acpi_battery, capacity_granularity_1), 0},
	{offsetof(struct acpi_battery, capacity_granularity_2), 0},
	{offsetof(struct acpi_battery, model_number), 1},
	{offsetof(struct acpi_battery, serial_number), 1},
	{offsetof(struct acpi_battery, type), 1},
	{offsetof(struct acpi_battery, oem_info), 1},
};

static const struct acpi_offsets extended_info_offsets[] = {
	{offsetof(struct acpi_battery, revision), 0},
	{offsetof(struct acpi_battery, power_unit), 0},
	{offsetof(struct acpi_battery, design_capacity), 0},
	{offsetof(struct acpi_battery, full_charge_capacity), 0},
	{offsetof(struct acpi_battery, technology), 0},
	{offsetof(struct acpi_battery, design_voltage), 0},
	{offsetof(struct acpi_battery, design_capacity_warning), 0},
	{offsetof(struct acpi_battery, design_capacity_low), 0},
	{offsetof(struct acpi_battery, cycle_count), 0},
	{offsetof(struct acpi_battery, measurement_accuracy), 0},
	{offsetof(struct acpi_battery, max_sampling_time), 0},
	{offsetof(struct acpi_battery, min_sampling_time), 0},
	{offsetof(struct acpi_battery, max_averaging_interval), 0},
	{offsetof(struct acpi_battery, min_averaging_interval), 0},
	{offsetof(struct acpi_battery, capacity_granularity_1), 0},
	{offsetof(struct acpi_battery, capacity_granularity_2), 0},
	{offsetof(struct acpi_battery, model_number), 1},
	{offsetof(struct acpi_battery, serial_number), 1},
	{offsetof(struct acpi_battery, type), 1},
	{offsetof(struct acpi_battery, oem_info), 1},
};

static int acpi_battery_get_status(struct acpi_battery *battery)
{
#ifdef DEBUG
	printk(KERN_ERR "acpi_battery_get_status\n");
#endif
	// do not do anything if we are still marked as suspended
	if (battery->suspended)
		return 0;
	/* we need to evaluate _STA in order to clear any ACPI events */
	if (acpi_bus_get_status(battery->device)) {
		ACPI_EXCEPTION((AE_INFO, AE_ERROR, "Evaluating _STA"));
		printk(KERN_ERR "  failed\n");
		return -ENODEV;
	}
	battery->device->status.battery_present = 1;
	msleep(20);
	tpt10_bat_get_status(battery);
#ifdef DEBUG
	printk(KERN_ERR "  OK\n");
#endif
	return 0;
}


static int acpi_battery_get_info(struct acpi_battery *battery)
{
	int result = 0 /*-ENODEV*/;

	if (!acpi_battery_present(battery))
		return 0;

	strcpy(battery->type, "LION");
	strcpy(battery->model_number, "TPT10");
	strcpy(battery->serial_number, "MP06SJAB");
	strcpy(battery->oem_info, "Tablet 10");
	battery->cycle_count = 123;
	battery->full_charge_capacity = 28000; // a little degraded
	battery->design_capacity = 33000;
	battery->design_capacity_warning = 2000;
	battery->design_capacity_low = 1000;
	battery->design_voltage = 3700;

	return result;
}

static int acpi_battery_get_state(struct acpi_battery *battery)
{
	int result = 0;

//	printk(KERN_INFO "get_state\n");
#if 0
	battery->capacity_now = 100;
	battery->full_charge_capacity = 32000;
	battery->design_voltage = 3700000;
	battery->voltage_now = 3700000;
	battery->rate_now = 0;
#endif
	if (battery->update_time &&
            time_before(jiffies, battery->update_time +
                        msecs_to_jiffies(cache_time)))
                return 0;

	battery->update_time = jiffies;

	return result;
}

static int sysfs_add_battery(struct acpi_battery *battery)
{
	struct power_supply_config psy_cfg = { .drv_data = battery, };

	if (battery->power_unit == ACPI_BATTERY_POWER_UNIT_MA) {
		battery->bat_desc.properties = charge_battery_props;
		battery->bat_desc.num_properties =
			ARRAY_SIZE(charge_battery_props);
	} else {
		battery->bat_desc.properties = energy_battery_props;
		battery->bat_desc.num_properties =
			ARRAY_SIZE(energy_battery_props);
	}

	battery->bat_desc.name = acpi_device_bid(battery->device);
	battery->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	battery->bat_desc.get_property = acpi_battery_get_property;

	battery->bat = power_supply_register_no_ws(&battery->device->dev,
				&battery->bat_desc, &psy_cfg);

	if (IS_ERR(battery->bat)) {
		int result = PTR_ERR(battery->bat);

		battery->bat = NULL;
		return result;
	}

	return 0;
}

static void sysfs_remove_battery(struct acpi_battery *battery)
{
	mutex_lock(&battery->sysfs_lock);
	if (!battery->bat) {
		mutex_unlock(&battery->sysfs_lock);
		return;
	}

	power_supply_unregister(battery->bat);
	battery->bat = NULL;
	mutex_unlock(&battery->sysfs_lock);
}

static int acpi_battery_update(struct acpi_battery *battery, bool resume)
{
	int result, old_present = acpi_battery_present(battery);

	// printk(KERN_ERR "acpi_battery_update\n");
	result = acpi_battery_get_status(battery);
	if (result) {
		printk(KERN_ERR "acpi_battery_get_status() failed\n");
		return result;
	}
	if (!acpi_battery_present(battery)) {
		sysfs_remove_battery(battery);
		battery->update_time = 0;
		printk(KERN_ERR "  bat not present\n");
		return 0;
	}

	if (resume)
		return 0;

	if (!battery->update_time ||
	    old_present != acpi_battery_present(battery)) {
		result = acpi_battery_get_info(battery);
		if (result) {
			printk(KERN_ERR "  _get_info failed\n");
			return result;
		}
	}

	result = acpi_battery_get_state(battery);
	if (result) {
		printk(KERN_ERR "  _get_state failed\n");
		return result;
	}

	if (!battery->bat) {
		result = sysfs_add_battery(battery);
		if (result) {
			printk(KERN_ERR "  _sysfs_add_battery failed\n");
			return result;
		}
	}

	/*
	 * Wakeup the system if battery is critical low
	 * or lower than the alarm level
	 */
	if ((battery->state & ACPI_BATTERY_STATE_CRITICAL) ||
	    (test_bit(ACPI_BATTERY_ALARM_PRESENT, &battery->flags) &&
            (battery->capacity_now <= battery->alarm)))
		pm_wakeup_event(&battery->device->dev, 0);

	return result;
}

static void acpi_battery_refresh(struct acpi_battery *battery)
{
	int power_unit;

	if (!battery->bat)
		return;

	power_unit = battery->power_unit;

	acpi_battery_get_info(battery);

	if (power_unit == battery->power_unit)
		return;

	/* The battery has changed its reporting units. */
	sysfs_remove_battery(battery);
	sysfs_add_battery(battery);
}

/* --------------------------------------------------------------------------
                              FS Interface (/proc)
   -------------------------------------------------------------------------- */

#ifdef CONFIG_ACPI_PROCFS_POWER
static struct proc_dir_entry *acpi_battery_dir;

static const char *acpi_battery_units(const struct acpi_battery *battery)
{
	return (battery->power_unit == ACPI_BATTERY_POWER_UNIT_MA) ?
		"mA" : "mW";
}

static int acpi_battery_print_info(struct seq_file *seq, int result)
{
	struct acpi_battery *battery = seq->private;

	if (result)
		goto end;

	seq_printf(seq, "present:                 %s\n",
		   acpi_battery_present(battery) ? "yes" : "no");
	if (!acpi_battery_present(battery))
		goto end;
	if (battery->design_capacity == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "design capacity:         unknown\n");
	else
		seq_printf(seq, "design capacity:         %d %sh\n",
			   battery->design_capacity,
			   acpi_battery_units(battery));

	if (battery->full_charge_capacity == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "last full capacity:      unknown\n");
	else
		seq_printf(seq, "last full capacity:      %d %sh\n",
			   battery->full_charge_capacity,
			   acpi_battery_units(battery));

	seq_printf(seq, "battery technology:      %srechargeable\n",
		   (!battery->technology)?"non-":"");

	if (battery->design_voltage == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "design voltage:          unknown\n");
	else
		seq_printf(seq, "design voltage:          %d mV\n",
			   battery->design_voltage);
	seq_printf(seq, "design capacity warning: %d %sh\n",
		   battery->design_capacity_warning,
		   acpi_battery_units(battery));
	seq_printf(seq, "design capacity low:     %d %sh\n",
		   battery->design_capacity_low,
		   acpi_battery_units(battery));
	seq_printf(seq, "cycle count:		  %i\n", battery->cycle_count);
	seq_printf(seq, "capacity granularity 1:  %d %sh\n",
		   battery->capacity_granularity_1,
		   acpi_battery_units(battery));
	seq_printf(seq, "capacity granularity 2:  %d %sh\n",
		   battery->capacity_granularity_2,
		   acpi_battery_units(battery));
	seq_printf(seq, "model number:            %s\n", battery->model_number);
	seq_printf(seq, "serial number:           %s\n", battery->serial_number);
	seq_printf(seq, "battery type:            %s\n", battery->type);
	seq_printf(seq, "OEM info:                %s\n", battery->oem_info);
      end:
	if (result)
		seq_printf(seq, "ERROR: Unable to read battery info\n");
	return result;
}

static int acpi_battery_print_state(struct seq_file *seq, int result)
{
	struct acpi_battery *battery = seq->private;

	if (result)
		goto end;

	seq_printf(seq, "present:                 %s\n",
		   acpi_battery_present(battery) ? "yes" : "no");
	if (!acpi_battery_present(battery))
		goto end;

	seq_printf(seq, "capacity state:          %s\n",
			(battery->state & 0x04) ? "critical" : "ok");
	if ((battery->state & 0x01) && (battery->state & 0x02))
		seq_printf(seq,
			   "charging state:          charging/discharging\n");
	else if (battery->state & 0x01)
		seq_printf(seq, "charging state:          discharging\n");
	else if (battery->state & 0x02)
		seq_printf(seq, "charging state:          charging\n");
	else
		seq_printf(seq, "charging state:          charged\n");

	if (battery->rate_now == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "present rate:            unknown\n");
	else
		seq_printf(seq, "present rate:            %d %s\n",
			   battery->rate_now, acpi_battery_units(battery));

	if (battery->capacity_now == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "remaining capacity:      unknown\n");
	else
		seq_printf(seq, "remaining capacity:      %d %sh\n",
			   battery->capacity_now, acpi_battery_units(battery));
	if (battery->voltage_now == ACPI_BATTERY_VALUE_UNKNOWN)
		seq_printf(seq, "present voltage:         unknown\n");
	else
		seq_printf(seq, "present voltage:         %d mV\n",
			   battery->voltage_now);
      end:
	if (result)
		seq_printf(seq, "ERROR: Unable to read battery state\n");

	return result;
}


typedef int(*print_func)(struct seq_file *seq, int result);

static print_func acpi_print_funcs[ACPI_BATTERY_NUMFILES] = {
	acpi_battery_print_info,
	acpi_battery_print_state,
};

static int acpi_battery_read(int fid, struct seq_file *seq)
{
	struct acpi_battery *battery = seq->private;
	int result = acpi_battery_update(battery, false);
	return acpi_print_funcs[fid](seq, result);
}

#define DECLARE_FILE_FUNCTIONS(_name) \
static int acpi_battery_read_##_name(struct seq_file *seq, void *offset) \
{ \
	return acpi_battery_read(_name##_tag, seq); \
} \
static int acpi_battery_##_name##_open_fs(struct inode *inode, struct file *file) \
{ \
	return single_open(file, acpi_battery_read_##_name, PDE_DATA(inode)); \
}

DECLARE_FILE_FUNCTIONS(info);
DECLARE_FILE_FUNCTIONS(state);

#undef DECLARE_FILE_FUNCTIONS

#define FILE_DESCRIPTION_RO(_name) \
	{ \
	.name = __stringify(_name), \
	.mode = S_IRUGO, \
	.ops = { \
		.open = acpi_battery_##_name##_open_fs, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release, \
		.owner = THIS_MODULE, \
		}, \
	}

#define FILE_DESCRIPTION_RW(_name) \
	{ \
	.name = __stringify(_name), \
	.mode = S_IFREG | S_IRUGO | S_IWUSR, \
	.ops = { \
		.open = acpi_battery_##_name##_open_fs, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.write = acpi_battery_write_##_name, \
		.release = single_release, \
		.owner = THIS_MODULE, \
		}, \
	}

static const struct battery_file {
	struct file_operations ops;
	umode_t mode;
	const char *name;
} acpi_battery_file[] = {
	FILE_DESCRIPTION_RO(info),
	FILE_DESCRIPTION_RO(state),
};

#undef FILE_DESCRIPTION_RO
#undef FILE_DESCRIPTION_RW

static int acpi_battery_add_fs(struct acpi_device *device)
{
	struct proc_dir_entry *entry = NULL;
	int i;

	printk(KERN_WARNING PREFIX "Deprecated procfs I/F for battery is loaded,"
			" please retry with CONFIG_ACPI_PROCFS_POWER cleared\n");
	if (!acpi_device_dir(device)) {
		acpi_device_dir(device) = proc_mkdir(acpi_device_bid(device),
						     acpi_battery_dir);
		if (!acpi_device_dir(device))
			return -ENODEV;
	}

	for (i = 0; i < ACPI_BATTERY_NUMFILES; ++i) {
		entry = proc_create_data(acpi_battery_file[i].name,
					 acpi_battery_file[i].mode,
					 acpi_device_dir(device),
					 &acpi_battery_file[i].ops,
					 acpi_driver_data(device));
		if (!entry)
			return -ENODEV;
	}
	return 0;
}

static void acpi_battery_remove_fs(struct acpi_device *device)
{
	int i;
	if (!acpi_device_dir(device))
		return;
	for (i = 0; i < ACPI_BATTERY_NUMFILES; ++i)
		remove_proc_entry(acpi_battery_file[i].name,
				  acpi_device_dir(device));

	remove_proc_entry(acpi_device_bid(device), acpi_battery_dir);
	acpi_device_dir(device) = NULL;
}

#endif

/* --------------------------------------------------------------------------
                                 Driver Interface
   -------------------------------------------------------------------------- */

static void acpi_battery_notify(struct acpi_device *device, u32 event)
{
	struct acpi_battery *battery = acpi_driver_data(device);
	struct power_supply *old;

	if (!battery)
		return;
	old = battery->bat;
#ifdef DEBUG
	printk(KERN_INFO "acpi_battery_notify event=0x%02x\n", event);
#endif
	/*
	* On Acer Aspire V5-573G notifications are sometimes triggered too
	* early. For example, when AC is unplugged and notification is
	* triggered, battery state is still reported as "Full", and changes to
	* "Discharging" only after short delay, without any notification.
	*/
	if (battery_notification_delay_ms > 0)
		msleep(battery_notification_delay_ms);
	if (event == ACPI_BATTERY_NOTIFY_INFO)
		acpi_battery_refresh(battery);
	acpi_battery_update(battery, false);
	acpi_bus_generate_netlink_event(device->pnp.device_class,
					dev_name(&device->dev), event,
					acpi_battery_present(battery));
	acpi_notifier_call_chain(device, event, acpi_battery_present(battery));
	/* acpi_battery_update could remove power_supply object */
	if (old && battery->bat)
		power_supply_changed(battery->bat);
}

static int battery_notify(struct notifier_block *nb,
			       unsigned long mode, void *_unused)
{
	struct acpi_battery *battery = container_of(nb, struct acpi_battery,
						    pm_nb);
	int result;

	switch (mode) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		if (!acpi_battery_present(battery))
			return 0;

		if (!battery->bat) {
			result = acpi_battery_get_info(battery);
			if (result)
				return result;

			result = sysfs_add_battery(battery);
			if (result)
				return result;
		} else
			acpi_battery_refresh(battery);

		acpi_battery_get_state(battery);
		break;
	}

	return 0;
}


/*
 * Some machines'(E,G Lenovo Z480) ECs are not stable
 * during boot up and this causes battery driver fails to be
 * probed due to failure of getting battery information
 * from EC sometimes. After several retries, the operation
 * may work. So add retry code here and 20ms sleep between
 * every retries.
 */
static int acpi_battery_update_retry(struct acpi_battery *battery)
{
	int retry, ret;

	for (retry = 5; retry; retry--) {
		ret = acpi_battery_update(battery, false);
		if (!ret)
			break;

		msleep(20);
	}
	return ret;
}

static int acpi_battery_add(struct acpi_device *device)
{
	int result = 0;
	struct acpi_battery *battery = NULL;

	if (!device)
		return -EINVAL;

	if (device->dep_unmet)
		return -EPROBE_DEFER;

	battery = kzalloc(sizeof(struct acpi_battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;
	battery->device = device;
	strcpy(acpi_device_name(device), ACPI_BATTERY_DEVICE_NAME);
	strcpy(acpi_device_class(device), ACPI_BATTERY_CLASS);
	device->driver_data = battery;
	mutex_init(&battery->lock);
	mutex_init(&battery->sysfs_lock);

	battery->i2c_adap = i2c_get_adapter(0);
	if (!battery->i2c_adap) {
		printk(KERN_ERR "tpt10bat failed to get i2c adapter\n");
		// return -ENODEV;
	}

	battery->suspended = FALSE;

	result = acpi_battery_update_retry(battery);
	if (result) {
		printk(KERN_ERR "acpi_battery_update_retry() failed\n");
		goto fail;
	}

#ifdef CONFIG_ACPI_PROCFS_POWER
	result = acpi_battery_add_fs(device);
#endif
	if (result) {
#ifdef CONFIG_ACPI_PROCFS_POWER
		acpi_battery_remove_fs(device);
#endif
		goto fail;
	}

	printk(KERN_INFO PREFIX "%s Slot [%s] (battery %s)\n",
		ACPI_BATTERY_DEVICE_NAME, acpi_device_bid(device),
		device->status.battery_present ? "present" : "absent");

	battery->pm_nb.notifier_call = battery_notify;
	register_pm_notifier(&battery->pm_nb);

	device_init_wakeup(&device->dev, 1);

	return result;

fail:
	sysfs_remove_battery(battery);
	mutex_destroy(&battery->lock);
	mutex_destroy(&battery->sysfs_lock);
	kfree(battery);
	return result;
}

static int acpi_battery_remove(struct acpi_device *device)
{
	struct acpi_battery *battery = NULL;

	if (!device || !acpi_driver_data(device))
		return -EINVAL;
	device_init_wakeup(&device->dev, 0);
	battery = acpi_driver_data(device);
	unregister_pm_notifier(&battery->pm_nb);
#ifdef CONFIG_ACPI_PROCFS_POWER
	acpi_battery_remove_fs(device);
#endif
	sysfs_remove_battery(battery);
	mutex_destroy(&battery->lock);
	mutex_destroy(&battery->sysfs_lock);
	kfree(battery);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* this is needed to learn about changes made in suspended state */
static int acpi_battery_suspend(struct device *dev)
{
	struct acpi_battery *battery;

	if (!dev)
		return -EINVAL;

	battery = acpi_driver_data(to_acpi_device(dev));
	if (!battery)
		return -EINVAL;

	battery->suspended = TRUE;

	return 0;
}

static int acpi_battery_resume(struct device *dev)
{
	struct acpi_battery *battery;

	if (!dev)
		return -EINVAL;

	battery = acpi_driver_data(to_acpi_device(dev));
	if (!battery)
		return -EINVAL;

	battery->update_time = 0;
	acpi_battery_update(battery, true);

	battery->suspended = FALSE;

	return 0;
}
#else
#define acpi_battery_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(acpi_battery_pm, acpi_battery_suspend, acpi_battery_resume);

static struct acpi_driver acpi_battery_driver = {
	.name = "battery",
	.class = ACPI_BATTERY_CLASS,
	.ids = battery_device_ids,
	.flags = ACPI_DRIVER_ALL_NOTIFY_EVENTS,
	.ops = {
		.add = acpi_battery_add,
		.remove = acpi_battery_remove,
		.notify = acpi_battery_notify,
		},
	.drv.pm = &acpi_battery_pm,
};

static void __init acpi_battery_init_async(void *unused, async_cookie_t cookie)
{
	unsigned int i;
	int result;

	for (i = 0; i < ARRAY_SIZE(acpi_battery_blacklist); i++)
		if (acpi_dev_present(acpi_battery_blacklist[i], "1", -1)) {
			pr_info(PREFIX ACPI_BATTERY_DEVICE_NAME
				": found native %s PMIC, not loading\n",
				acpi_battery_blacklist[i]);
			return;
		}

#ifdef CONFIG_ACPI_PROCFS_POWER
	acpi_battery_dir = acpi_lock_battery_dir();
	if (!acpi_battery_dir)
		return;
#endif
	result = acpi_bus_register_driver(&acpi_battery_driver);
#ifdef CONFIG_ACPI_PROCFS_POWER
	if (result < 0)
		acpi_unlock_battery_dir(acpi_battery_dir);
#endif
}

static int __init acpi_battery_init(void)
{
	if (acpi_disabled)
		return -ENODEV;

	/* Check if acpi_battery_unregister() got called before _init() */
	mutex_lock(&init_state_mutex);
	if (init_state != BAT_NONE)
		goto out_unlock;

	async_cookie = async_schedule(acpi_battery_init_async, NULL);
	init_state = BAT_INITIALIZED;

out_unlock:
	mutex_unlock(&init_state_mutex);

	return 0;
}

void acpi_battery_unregister(void)
{
	/* Check if _init() is done and only do unregister once */
	mutex_lock(&init_state_mutex);
	if (init_state != BAT_INITIALIZED)
		goto out_exit;

	async_synchronize_cookie(async_cookie + 1);
	acpi_bus_unregister_driver(&acpi_battery_driver);
#ifdef CONFIG_ACPI_PROCFS_POWER
	acpi_unlock_battery_dir(acpi_battery_dir);
#endif

out_exit:
	init_state = BAT_EXITED;
	mutex_unlock(&init_state_mutex);
}
EXPORT_SYMBOL_GPL(acpi_battery_unregister);

static void __exit acpi_battery_exit(void)
{
	acpi_battery_unregister();
}

module_init(acpi_battery_init);
module_exit(acpi_battery_exit);
