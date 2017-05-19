/*
 * Extcon USB-C cable detection driver for FUSB302 USB TYPE-C controller
 *
 * Copyright (C) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/extcon.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "extcon.h"

#define REG_DEVICE_ID				0x01
#define DEVICE_ID_VER_MASK			GENMASK(7, 4)
#define DEVICE_ID_FUSB302_VER			0x80

#define REG_SWITCHES0				0x02
#define SWITCHES0_PDWN1				BIT(0)
#define SWITCHES0_PDWN2				BIT(1)
#define SWITCHES0_PDWN				GENMASK(1, 0)
#define SWITCHES0_MEAS_CC1			BIT(2)
#define SWITCHES0_MEAS_CC2			BIT(3)
#define SWITCHES0_VCONN_CC1			BIT(4)
#define SWITCHES0_VCONN_CC2			BIT(5)
#define SWITCHES0_PU_EN1			BIT(6)
#define SWITCHES0_PU_EN2			BIT(7)

#define REG_MEASURE				0x04
#define MEASURE_MDAC_MASK			GENMASK(5, 0)
/* Datasheet says MDAC must be set to 0x34 / 2226mV for vRd-3.0 detection */
#define MEASURE_MDAC_SNK_VRD30			0x34
/* MDAC must be set to 0x25 / 1600mV for disconnect det. with 80uA host-cur */
#define MEASURE_MDAC_SRC_80UA_HOST_CUR		0x25

#define REG_CONTROL0				0x06
#define CONTROL0_HOST_CUR_MASK			GENMASK(3, 2)
#define CONTROL0_HOST_CUR_DISABLED		(0 << 2)
#define CONTROL0_HOST_CUR_80UA			(1 << 2)
#define CONTROL0_HOST_CUR_180UA			(2 << 2)
#define CONTROL0_HOST_CUR_330UA			(3 << 2)
#define CONTROL0_INT_MASK			BIT(5)

#define REG_CONTROL2				0x08
#define CONTROL2_TOGGLE				BIT(0)
#define CONTROL2_MODE_MASK			GENMASK(2, 1)
#define CONTROL2_MODE_DRP			(1 << 1)
#define CONTROL2_MODE_SNK			(2 << 1)
#define CONTROL2_MODE_SRC			(3 << 1)
#define CONTROL2_SAVE_PWR_MASK			GENMASK(7, 6)
#define CONTROL2_SAVE_PWR_DISABLED		(0 << 6)
#define CONTROL2_SAVE_PWR_40MS			(1 << 6)
#define CONTROL2_SAVE_PWR_80MS			(2 << 6)
#define CONTROL2_SAVE_PWR_160MS			(3 << 6)

#define REG_MASK1				0x0a
/* REG_MASK1 value for low-power / disabled state from datasheet */
#define MASK1_DISABLED				0xfe
#define MASK1_COMP_CHNG				BIT(5)
#define MASK1_VBUSOK				BIT(7)

#define REG_POWER				0x0b
/* REG_POWER values for disabled and normal state from datasheet */
#define POWER_DISABLED				BIT(0)
#define POWER_NORMAL				GENMASK(2, 0)

#define REG_RESET				0x0c
#define RESET_SW_RESET				BIT(0)

#define REG_OCP					0x0d

#define REG_MASKA				0x0e
/* REG_MASKA value for low-power / disabled state from datasheet */
#define MASKA_DISABLED				0xbf

#define REG_MASKB				0x0f
/* REG_MASKB value for low-power / disabled state from datasheet */
#define MASKB_DISABLED				0x01

#define REG_STATUS1A				0x3d
#define STATUS1A_TOGSS_MASK			GENMASK(5, 3)
#define STATUS1A_TOGSS_SRC_CC1			(1 << 3)
#define STATUS1A_TOGSS_SRC_CC2			(2 << 3)
#define STATUS1A_TOGSS_SNK_CC1			(5 << 3)
#define STATUS1A_TOGSS_SNK_CC2			(6 << 3)

#define REG_INTERRUPTA				0x3e
#define INTERRUPTA_TOGDONE			BIT(6)

#define REG_STATUS0				0x40
#define STATUS0_BC_LEVEL_MASK			GENMASK(1, 0)
#define STATUS0_BC_LEVEL_VRA			0
#define STATUS0_BC_LEVEL_VRDUSB			1
#define STATUS0_BC_LEVEL_VRD15			2
#define STATUS0_BC_LEVEL_VRD30			3
#define STATUS0_COMP				BIT(5)
#define STATUS0_VBUSOK				BIT(7)

#define REG_INTERRUPT				0x42
#define INTERRUPT_BC_LVL			BIT(0)
#define INTERRUPT_COMP_CHNG			BIT(5)
#define INTERRUPT_VBUSOK			BIT(7)

/* Timeouts from the FUSB302 datasheet */
#define TTOGCYCLE				msecs_to_jiffies(40 + 60 + 40)

/* Timeouts from the USB-C specification */
#define TPDDEBOUNCE				msecs_to_jiffies(20)
#define TDRPTRY_MS				75
#define TDRPTRY					msecs_to_jiffies(TDRPTRY_MS)
#define TDRPTRYWAIT				msecs_to_jiffies(400)
#define TVBUSON					msecs_to_jiffies(275)

enum typec_port_type {
	TYPEC_PORT_DFP,
	TYPEC_PORT_UFP,
	TYPEC_PORT_DRP,
};

enum typec_role {
	TYPEC_SINK,
	TYPEC_SOURCE,
};

enum fusb302_state {
	DISABLED_SNK, /* UFP */
	DISABLED_SRC, /* DFP */
	DISABLED_DRP,
	UNATTACHED_SNK,
	ATTACHED_SNK,
	UNATTACHED_SRC,
	ATTACHED_SRC,
};
/* For debugging */
static __maybe_unused const char *fusb302_state_str[] = {
	"DISABLED_SNK",
	"DISABLED_SRC",
	"DISABLED_DRP",
	"UNATTACHED_SNK",
	"ATTACHED_SNK",
	"UNATTACHED_SRC",
	"ATTACHED_SRC",
};

enum fusb302_event {
	TOGGLE_DONE,
	BC_LVL_CHANGE,
	VBUS_VALID,
	VBUS_INVALID,
	COMP_LOW,
	COMP_HIGH,
	TIMEOUT,
	FALLBACK_TIMEOUT,
};
/* For debugging */
static __maybe_unused const char *fusb302_event_str[] = {
	"TOGGLE_DONE",
	"BC_LVL_CHANGE",
	"VBUS_VALID",
	"VBUS_INVALID",
	"COMP_LOW",
	"COMP_HIGH",
	"TIMEOUT",
	"FALLBACK_TIMEOUT",
};

static const unsigned int fusb302_extcon_cables[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_CDP,
	EXTCON_CHG_USB_FAST,
	EXTCON_NONE,
};

struct fusb302_data {
	struct device *dev;
	struct regmap *regmap;
	struct extcon_dev *edev;
	struct mutex lock;
	struct delayed_work timeout;
	struct delayed_work fallback_timeout;
	struct delayed_work bc_work;
	enum fusb302_state state;
	enum typec_port_type port_type;
	enum typec_role preferred_role;
	int status0;
	int status1a;
	unsigned int snk_cable_id;
};

static void fusb302_write_reg(struct fusb302_data *data, int reg, int val)
{
	int ret;

	ret = regmap_write(data->regmap, reg, val);
	if (ret)
		dev_err(data->dev, "Error writing reg %02x: %d\n", reg, ret);
}

static int fusb302_read_reg(struct fusb302_data *data, int reg)
{
	int ret, val;

	ret = regmap_read(data->regmap, reg, &val);
	if (ret) {
		dev_err(data->dev, "Error reading reg %02x: %d\n", reg, ret);
		return 0;
	}

	return val;
}

static void fusb302_update_bits(struct fusb302_data *data, int reg,
				int mask, int val)
{
	int ret;

	ret = regmap_update_bits(data->regmap, reg, mask, val);
	if (ret)
		dev_err(data->dev, "Error modifying reg %02x: %d\n", reg, ret);
}

/* Small helper to sync EXTCON_CHG_USB_SDP and EXTCON_USB state */
static void fusb302_set_extcon_state(struct fusb302_data *data,
				     unsigned int cable, bool state)
{
	extcon_set_state_sync(data->edev, cable, state);
	if (cable == EXTCON_CHG_USB_SDP)
		extcon_set_state_sync(data->edev, EXTCON_USB, state);
}

/* Helper for the disabled states */
static void fusb302_disable(struct fusb302_data *data)
{
	fusb302_write_reg(data, REG_POWER, POWER_DISABLED);
	fusb302_write_reg(data, REG_MASK1, MASK1_DISABLED);
	fusb302_write_reg(data, REG_MASKA, MASKA_DISABLED);
	fusb302_write_reg(data, REG_MASKB, MASKB_DISABLED);
	fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE, 0);
}

/*
 * fusb302_set_state() and fusb302_handle_event() implement the 3 state
 * machines from the datasheet folded into 1 state-machine for code re-use.
 */
static void fusb302_set_state(struct fusb302_data *data,
			      enum fusb302_state state)
{
	int status, switches0 = 0;
	enum fusb302_state old_state = data->state;
	union extcon_property_value prop;

	/* Kill pending timeouts from previous state */
	cancel_delayed_work(&data->timeout);
	cancel_delayed_work(&data->fallback_timeout);
	cancel_delayed_work(&data->bc_work);

	dev_dbg(data->dev, "New state %s\n", fusb302_state_str[state]);

	switch (old_state) {
	case ATTACHED_SNK:
		fusb302_set_extcon_state(data, data->snk_cable_id, false);
		break;
	case ATTACHED_SRC:
		fusb302_set_extcon_state(data, EXTCON_USB_HOST, false);
		break;
	default:
		break; /* Do nothing */
	}

	switch (state) {
	case DISABLED_SNK:
		fusb302_disable(data);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_MODE_MASK,
				    CONTROL2_MODE_SNK);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE,
				    CONTROL2_TOGGLE);
		break;

	case DISABLED_SRC:
		fusb302_disable(data);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_MODE_MASK,
				    CONTROL2_MODE_SRC);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE,
				    CONTROL2_TOGGLE);
		break;

	case DISABLED_DRP:
		fusb302_disable(data);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_MODE_MASK,
				    CONTROL2_MODE_DRP);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE,
				    CONTROL2_TOGGLE);
		break;

	case UNATTACHED_SNK:
		fusb302_write_reg(data, REG_POWER, POWER_NORMAL);

		/* Enable pull-down on CC1 / CC2 based on orientation */
		switch (data->status1a & STATUS1A_TOGSS_MASK) {
		case STATUS1A_TOGSS_SNK_CC1:
			switches0 = SWITCHES0_PDWN1 | SWITCHES0_MEAS_CC1;
			prop.intval = USB_TYPEC_POLARITY_NORMAL;
			break;
		case STATUS1A_TOGSS_SNK_CC2:
			switches0 = SWITCHES0_PDWN2 | SWITCHES0_MEAS_CC2;
			prop.intval = USB_TYPEC_POLARITY_FLIP;
			break;
		}
		fusb302_write_reg(data, REG_SWITCHES0, switches0);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE, 0);
		extcon_set_property(data->edev, EXTCON_USB,
				    EXTCON_PROP_USB_TYPEC_POLARITY, prop);

		/* Enable VBUSOK and COMP irq at 2.24V for BC detection */
		fusb302_update_bits(data, REG_MASK1,
				    MASK1_VBUSOK | MASK1_COMP_CHNG, 0);
		fusb302_update_bits(data, REG_MEASURE, MEASURE_MDAC_MASK,
				    MEASURE_MDAC_SNK_VRD30);

		status = fusb302_read_reg(data, REG_STATUS0);
		if (status & STATUS0_VBUSOK) {
			/* Go straight to ATTACHED_SNK */
			fusb302_set_state(data, ATTACHED_SNK);
			return;
		}

		mod_delayed_work(system_wq, &data->timeout, TVBUSON);
		break;

	case ATTACHED_SNK:
		mod_delayed_work(system_wq, &data->bc_work, TPDDEBOUNCE);
		break;

	case UNATTACHED_SRC:
		fusb302_write_reg(data, REG_POWER, POWER_NORMAL);

		/* Enable pull-up / Vconn on CC1 / CC2 based on orientation */
		switch (data->status1a & STATUS1A_TOGSS_MASK) {
		case STATUS1A_TOGSS_SRC_CC1:
			switches0 = SWITCHES0_PU_EN1 | SWITCHES0_VCONN_CC2 |
				    SWITCHES0_MEAS_CC1;
			prop.intval = USB_TYPEC_POLARITY_NORMAL;
			break;
		case STATUS1A_TOGSS_SRC_CC2:
			switches0 = SWITCHES0_PU_EN2 | SWITCHES0_VCONN_CC1 |
				    SWITCHES0_MEAS_CC2;
			prop.intval = USB_TYPEC_POLARITY_FLIP;
			break;
		}
		fusb302_write_reg(data, REG_SWITCHES0, switches0);
		fusb302_update_bits(data, REG_CONTROL2, CONTROL2_TOGGLE, 0);
		extcon_set_property(data->edev, EXTCON_USB,
				    EXTCON_PROP_USB_TYPEC_POLARITY, prop);

		/* Enable COMP irq at 1.6V for detach detection */
		fusb302_update_bits(data, REG_MASK1, MASK1_COMP_CHNG, 0);
		fusb302_update_bits(data, REG_MEASURE, MEASURE_MDAC_MASK,
				    MEASURE_MDAC_SRC_80UA_HOST_CUR);

		status = fusb302_read_reg(data, REG_STATUS0);
		if (!(status & STATUS0_COMP)) {
			/* Go straight to ATTACHED_SRC */
			fusb302_set_state(data, ATTACHED_SRC);
			return;
		}

		mod_delayed_work(system_wq, &data->timeout, TPDDEBOUNCE);
		break;

	case ATTACHED_SRC:
		fusb302_set_extcon_state(data, EXTCON_USB_HOST, true);
		break;
	}

	data->state = state;
}

static void fusb302_set_state_disabled(struct fusb302_data *data)
{

	switch (data->port_type) {
	case TYPEC_PORT_UFP:
		fusb302_set_state(data, DISABLED_SNK);
		break;
	case TYPEC_PORT_DFP:
		fusb302_set_state(data, DISABLED_SRC);
		break;
	case TYPEC_PORT_DRP:
		fusb302_set_state(data, DISABLED_DRP);
		break;
	}
}

static void fusb302_handle_disabled_snk_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case TOGGLE_DONE:
		switch (data->status1a & STATUS1A_TOGSS_MASK) {
		case STATUS1A_TOGSS_SNK_CC1:
		case STATUS1A_TOGSS_SNK_CC2:
			fusb302_set_state(data, UNATTACHED_SNK);
			break;
		}
		break;

	case TIMEOUT:
		/* Cannot become snk fallback to src */
		fusb302_set_state(data, DISABLED_SRC);
		mod_delayed_work(system_wq, &data->fallback_timeout,
				 TDRPTRYWAIT);
		break;

	case FALLBACK_TIMEOUT:
		/* Both states failed return to disabled drp state */
		fusb302_set_state(data, DISABLED_DRP);
		break;
	}
}

static void fusb302_handle_disabled_src_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case TOGGLE_DONE:
		switch (data->status1a & STATUS1A_TOGSS_MASK) {
		case STATUS1A_TOGSS_SRC_CC1:
		case STATUS1A_TOGSS_SRC_CC2:
			fusb302_set_state(data, UNATTACHED_SRC);
			break;
		}
		break;

	case TIMEOUT:
		/* Cannot become src fallback to snk */
		fusb302_set_state(data, DISABLED_SNK);
		mod_delayed_work(system_wq, &data->fallback_timeout,
				 TDRPTRYWAIT);
		break;

	case FALLBACK_TIMEOUT:
		/* Both states failed return to disabled drp state */
		fusb302_set_state(data, DISABLED_DRP);
		break;
	}
}

static void fusb302_handle_disabled_drp_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case TOGGLE_DONE:
		switch (data->status1a & STATUS1A_TOGSS_MASK) {
		case STATUS1A_TOGSS_SNK_CC1:
		case STATUS1A_TOGSS_SNK_CC2:
			if (data->preferred_role == TYPEC_SINK) {
				/* Jump directly to UNATTACHED_SNK */
				fusb302_set_state(data, UNATTACHED_SNK);
			} else {
				/* Try to become src */
				fusb302_set_state(data, DISABLED_SNK);
				mod_delayed_work(system_wq, &data->timeout,
						 TDRPTRY);
			}
			break;
		case STATUS1A_TOGSS_SRC_CC1:
		case STATUS1A_TOGSS_SRC_CC2:
			if (data->preferred_role == TYPEC_SOURCE) {
				/* Jump directly to UNATTACHED_SRC */
				fusb302_set_state(data, UNATTACHED_SRC);
			} else {
				/*
				 * The USB-C spec says we must enable pull-downs
				 * and then wait tDRPTry before checking to
				 * avoid endless role-bouncing if both ends
				 * prefer the snk role.
				 */
				fusb302_write_reg(data, REG_SWITCHES0,
						  SWITCHES0_PDWN);
				fusb302_update_bits(data, REG_CONTROL2,
						    CONTROL2_TOGGLE, 0);
				msleep(TDRPTRY_MS);
				/*
				 * Use the toggle engine to do the src
				 * detection to keep things the same as for
				 * directly entering the src role.
				 */
				fusb302_set_state(data, DISABLED_SNK);
				mod_delayed_work(system_wq, &data->timeout,
						 TTOGCYCLE);
			}
			break;
		}
		break;
	}
}

static void fusb302_handle_unattached_snk_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case VBUS_VALID: /* Cable attached */
		fusb302_set_state(data, ATTACHED_SNK);
		break;
	case TIMEOUT:
		fusb302_set_state_disabled(data);
		break;
	}
}

static void fusb302_handle_attached_snk_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case BC_LVL_CHANGE:
	case COMP_LOW:
	case COMP_HIGH:
		mod_delayed_work(system_wq, &data->bc_work, TPDDEBOUNCE);
		break;
	case VBUS_INVALID: /* Cable detached */
		fusb302_set_state_disabled(data);
		break;
	}
}

static void fusb302_handle_unattached_src_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case COMP_LOW: /* Cable attached */
		fusb302_set_state(data, ATTACHED_SRC);
		break;
	case TIMEOUT:
		fusb302_set_state_disabled(data);
		break;
	}
}

static void fusb302_handle_attached_src_event(struct fusb302_data *data,
					      int event)
{
	switch (event) {
	case COMP_HIGH: /* Cable detached */
		fusb302_set_state_disabled(data);
		break;
	}
}

static void fusb302_handle_event(struct fusb302_data *data, int event)
{

	mutex_lock(&data->lock);

	dev_dbg(data->dev, "Handling state %s event %s status %02x %02x\n",
		fusb302_state_str[data->state], fusb302_event_str[event],
		fusb302_read_reg(data, REG_STATUS0),
		fusb302_read_reg(data, REG_STATUS1A));

	switch (data->state) {
	case DISABLED_SNK:
		fusb302_handle_disabled_snk_event(data, event);
		break;
	case DISABLED_SRC:
		fusb302_handle_disabled_src_event(data, event);
		break;
	case DISABLED_DRP:
		fusb302_handle_disabled_drp_event(data, event);
		break;
	case UNATTACHED_SNK:
		fusb302_handle_unattached_snk_event(data, event);
		break;
	case ATTACHED_SNK:
		fusb302_handle_attached_snk_event(data, event);
		break;
	case UNATTACHED_SRC:
		fusb302_handle_unattached_src_event(data, event);
		break;
	case ATTACHED_SRC:
		fusb302_handle_attached_src_event(data, event);
		break;
	}
	mutex_unlock(&data->lock);
}

static void fusb302_timeout(struct work_struct *work)
{
	struct fusb302_data *data =
		container_of(work, struct fusb302_data, timeout.work);

	fusb302_handle_event(data, TIMEOUT);
}

static void fusb302_fallback_timeout(struct work_struct *work)
{
	struct fusb302_data *data =
		container_of(work, struct fusb302_data, fallback_timeout.work);

	fusb302_handle_event(data, FALLBACK_TIMEOUT);
}

static void fusb302_report_bc_level(struct work_struct *work)
{
	struct fusb302_data *data =
		container_of(work, struct fusb302_data, bc_work.work);

	/* Clear old charger cable id */
	fusb302_set_extcon_state(data, data->snk_cable_id, false);

	if (data->status0 & STATUS0_COMP) {
		dev_warn(data->dev, "vRd over maximum, assuming 500mA source\n");
		data->snk_cable_id = EXTCON_CHG_USB_SDP;
	} else {
		switch (data->status0 & STATUS0_BC_LEVEL_MASK) {
		case STATUS0_BC_LEVEL_VRA:
		case STATUS0_BC_LEVEL_VRDUSB:
			data->snk_cable_id = EXTCON_CHG_USB_SDP;
			break;
		case STATUS0_BC_LEVEL_VRD15:
			data->snk_cable_id = EXTCON_CHG_USB_CDP;
			break;
		case STATUS0_BC_LEVEL_VRD30:
			/* Use CHG_USB_FAST to indicate 3A capability */
			data->snk_cable_id = EXTCON_CHG_USB_FAST;
			break;
		}
	}
	fusb302_set_extcon_state(data, data->snk_cable_id, true);
}

static irqreturn_t fusb302_irq_handler_thread(int irq, void *handler_data)
{
	struct fusb302_data *data = handler_data;
	int interrupt, interrupta;

	interrupt = fusb302_read_reg(data, REG_INTERRUPT);
	interrupta = fusb302_read_reg(data, REG_INTERRUPTA);

	if (interrupt)
		data->status0 = fusb302_read_reg(data, REG_STATUS0);

	if (interrupta)
		data->status1a = fusb302_read_reg(data, REG_STATUS1A);

	dev_dbg(data->dev, "Handling interrupt %02x %02x status %02x %02x\n",
		interrupt, interrupta, data->status0, data->status1a);

	if (interrupt & INTERRUPT_BC_LVL)
		fusb302_handle_event(data, BC_LVL_CHANGE);

	if (interrupt & INTERRUPT_COMP_CHNG) {
		if (data->status0 & STATUS0_COMP)
			fusb302_handle_event(data, COMP_HIGH);
		else
			fusb302_handle_event(data, COMP_LOW);
	}

	if (interrupt & INTERRUPT_VBUSOK) {
		if (data->status0 & STATUS0_VBUSOK)
			fusb302_handle_event(data, VBUS_VALID);
		else
			fusb302_handle_event(data, VBUS_INVALID);
	}

	if (interrupta & INTERRUPTA_TOGDONE)
		fusb302_handle_event(data, TOGGLE_DONE);

	return IRQ_HANDLED;
}

static const struct regmap_config fusb302_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static int fusb302_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fusb302_data *data;
	int ret;

	if (!client->irq) {
		dev_err(dev, "Error irq not specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	/* TODO make these 2 configurable using device-properties */
	data->port_type = TYPEC_PORT_DRP;
	data->preferred_role = TYPEC_SINK;

	data->regmap = devm_regmap_init_i2c(client, &fusb302_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(dev, "Error to initializing regmap: %d\n", ret);
		return ret;
	}

	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->timeout, fusb302_timeout);
	INIT_DELAYED_WORK(&data->fallback_timeout, fusb302_fallback_timeout);
	INIT_DELAYED_WORK(&data->bc_work, fusb302_report_bc_level);

	data->edev = devm_extcon_dev_allocate(dev, fusb302_extcon_cables);
	if (IS_ERR(data->edev))
		return PTR_ERR(data->edev);

	data->edev->name = "fusb302";

	ret = devm_extcon_dev_register(dev, data->edev);
	if (ret)
		return ret;

	extcon_set_property_capability(data->edev, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	fusb302_write_reg(data, REG_RESET, RESET_SW_RESET);
	usleep_range(10000, 20000);

	/* Enable power-saving */
	fusb302_update_bits(data, REG_CONTROL2, CONTROL2_SAVE_PWR_MASK,
			    CONTROL2_SAVE_PWR_40MS);

	fusb302_set_state_disabled(data);

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
			fusb302_irq_handler_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fusb302", data);
	if (ret) {
		dev_err(dev, "Error requesting irq: %d\n", ret);
		return ret;
	}

	fusb302_update_bits(data, REG_CONTROL0, CONTROL0_INT_MASK, 0);

	i2c_set_clientdata(client, data);
	return 0;
}

static int fusb302_remove(struct i2c_client *client)
{
	struct fusb302_data *data = i2c_get_clientdata(client);

	devm_free_irq(data->dev, client->irq, data);
	cancel_delayed_work_sync(&data->timeout);
	cancel_delayed_work_sync(&data->fallback_timeout);

	return 0;
}

static const struct i2c_device_id fusb302_i2c_ids[] = {
	{ "fusb302" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, fusb302_i2c_ids);

static struct i2c_driver fusb302_driver = {
	.probe_new = fusb302_probe,
	.remove = fusb302_remove,
	.id_table = fusb302_i2c_ids,
	.driver = {
		.name = "fusb302",
	},
};
module_i2c_driver(fusb302_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("FUSB302 USB TYPE-C controller Driver");
