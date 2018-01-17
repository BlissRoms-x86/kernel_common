/*
 * mux/consumer.h - definitions for the multiplexer consumer interface
 *
 * Copyright (C) 2017 Axentia Technologies AB
 *
 * Author: Peter Rosin <peda@axentia.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_MUX_CONSUMER_H
#define _LINUX_MUX_CONSUMER_H

#include <linux/compiler.h>

/*
 * Mux state values for USB muxes, used for both USB device/host role muxes
 * as well as for Type-C polarity/role/altmode muxes.
 *
 * MUX_USB_POLARITY_INV may be or-ed together with any other mux-state as
 * inverted-polarity (Type-C plugged in upside down) can happen with any
 * other mux-state.
 */
#define MUX_USB_POLARITY_INV	BIT(0)   /* Polarity inverted bit */
#define MUX_USB_NONE		(1 << 1) /* Mux open / not connected */
#define MUX_USB_DEVICE		(2 << 1) /* USB device mode */
#define MUX_USB_HOST		(3 << 1) /* USB host mode */
#define MUX_USB_HOST_AND_DP_SRC	(4 << 1) /* USB host + 2 lanes Display Port */
#define MUX_USB_DP_SRC		(5 << 1) /* 4 lanes Display Port source */
#define MUX_USB_STATES		(6 << 1)

struct device;
struct mux_control;

struct mux_lookup {
	struct list_head list;
	const char *provider;
	unsigned int index;
	const char *dev_id;
	const char *mux_name;
};

void mux_add_table(struct mux_lookup *table, size_t num);
void mux_remove_table(struct mux_lookup *table, size_t num);

unsigned int mux_control_states(struct mux_control *mux);
int __must_check mux_control_select(struct mux_control *mux,
				    unsigned int state);
int __must_check mux_control_try_select(struct mux_control *mux,
					unsigned int state);
int mux_control_deselect(struct mux_control *mux);

struct mux_control *mux_control_get(struct device *dev, const char *mux_name);
void mux_control_put(struct mux_control *mux);

struct mux_control *devm_mux_control_get(struct device *dev,
					 const char *mux_name);

#endif /* _LINUX_MUX_CONSUMER_H */
