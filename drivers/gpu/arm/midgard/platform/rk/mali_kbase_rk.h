/*
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#ifndef _KBASE_PLATFORM_H_
#define _KBASE_PLATFORM_H_

#include <linux/clk.h>
#include <mali_kbase.h>
#include "mali_kbase_rk_dvfs.h"

#define KBASE_RK_GPU_FREQ_KHZ_MAX               600000
#define KBASE_RK_GPU_FREQ_KHZ_MIN               100000

struct kbase_rk_fv {
	unsigned long freq;
	unsigned long volt;
};

struct kbase_rk {
	struct kbase_device *kbdev;
	struct clk *clk;
	struct regulator *regulator;
	struct kbase_rk_fv *fv_table;
	struct notifier_block thermal_change_nb;
	unsigned int fv_table_length;
	unsigned int current_level;
	unsigned int requested_level;
	unsigned int thermal_throttling_level;
	struct kbase_rk_dvfs dvfs;
	struct mutex set_level_lock;
	bool is_powered;
};

int kbase_rk_set_freq(struct kbase_device *kbdev, unsigned long freq);

#endif				/* _KBASE_PLATFORM_H_ */
