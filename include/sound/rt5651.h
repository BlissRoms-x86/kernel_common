/*
 * linux/sound/rt286.h -- Platform data for RT286
 *
 * Copyright 2013 Realtek Microelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_RT5651_H
#define __LINUX_SND_RT5651_H

enum rt5651_jd_src {
	RT5651_JD_NULL,
	RT5651_JD1_1,
	RT5651_JD1_2,
	RT5651_JD2,
};

struct rt5651_platform_data {
	/* IN2 can optionally be differential */
	bool in2_diff;
	/* Configure GPIO2 as DMIC1 SCL ? */
	bool dmic_en;
	/* Platform clock dapm supply name */
	const char *plat_clk_name;
	/* Jack detect source or JD_NULL to disable jack-detect */
	enum rt5651_jd_src jd_src;
};

#endif
