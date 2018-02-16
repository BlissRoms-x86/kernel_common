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

/* These mirror the RT5651_MIC1_OVTH_*UA consts and MUST be in the same order */
enum rt5651_ovth_curr {
	RT5651_OVTH_600UA,
	RT5651_OVTH_1500UA,
	RT5651_OVTH_2000UA,
};

/* These mirror the RT5651_MIC_OVCD_SF* consts and MUST be in the same order */
enum rt5651_ovcd_sf {
	RT5651_OVCD_SF_0P5,
	RT5651_OVCD_SF_0P75,
	RT5651_OVCD_SF_1P0,
	RT5651_OVCD_SF_1P5,
};

/*
 * Note testing on various boards has shown that good defaults for ovth_curr
 * and ovth_sf are 2000UA and 0.75. For an effective threshold of 1500UA,
 * this seems to be more reliable then 1500UA and 1.0. Some boards may need
 * different values because of a resistor on the board in serial with or
 * parallel to the jack mic contact.
 */
struct rt5651_platform_data {
	/* IN2 can optionally be differential */
	bool in2_diff;
	/* Configure GPIO2 as DMIC1 SCL */
	bool dmic_en;
	/* Jack detect is inverted */
	bool jd_inverted;
	/* Jack detect source or JD_NULL to disable jack-detect */
	enum rt5651_jd_src jd_src;
	/* Jack micbias overcurrent detect current threshold */
	enum rt5651_ovth_curr ovth_curr;
	/* Jack micbias overcurrent detect current scale-factor */
	enum rt5651_ovcd_sf ovth_sf;
	/* Platform clock dapm supply name */
	const char *clk;
};

#endif
