/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/

/* ************************************************************
 * include files
 * ************************************************************ */
#include "mp_precomp.h"
#include "phydm_precomp.h"


u8
odm_get_auto_channel_select_result(
	void			*p_dm_void,
	u8			band
)
{
	struct PHY_DM_STRUCT				*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ACS_					*p_acs = &p_dm_odm->dm_acs;

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	if (band == ODM_BAND_2_4G) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("[struct _ACS_] odm_get_auto_channel_select_result(): clean_channel_2g(%d)\n", p_acs->clean_channel_2g));
		return (u8)p_acs->clean_channel_2g;
	} else {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("[struct _ACS_] odm_get_auto_channel_select_result(): clean_channel_5g(%d)\n", p_acs->clean_channel_5g));
		return (u8)p_acs->clean_channel_5g;
	}
#else
	return (u8)p_acs->clean_channel_2g;
#endif

}

void
odm_auto_channel_select_setting(
	void			*p_dm_void,
	boolean			is_enable
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct PHY_DM_STRUCT					*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	u16						period = 0x2710;/* 40ms in default */
	u16						nhm_type = 0x7;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select_setting()=========>\n"));

	if (is_enable) {
		/* 20 ms */
		period = 0x1388;
		nhm_type = 0x1;
	}

	if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {
		/* PHY parameters initialize for ac series */
		odm_write_2byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC + 2, period);	/* 0x990[31:16]=0x2710	Time duration for NHM unit: 4us, 0x2710=40ms */
		/* odm_set_bb_reg(p_dm_odm, ODM_REG_NHM_TH9_TH10_11AC, BIT(8)|BIT9|BIT10, nhm_type);	 */ /* 0x994[9:8]=3			enable CCX */
	} else if (p_dm_odm->support_ic_type & ODM_IC_11N_SERIES) {
		/* PHY parameters initialize for n series */
		odm_write_2byte(p_dm_odm, ODM_REG_CCX_PERIOD_11N + 2, period);	/* 0x894[31:16]=0x2710	Time duration for NHM unit: 4us, 0x2710=40ms */
		/* odm_set_bb_reg(p_dm_odm, ODM_REG_NHM_TH9_TH10_11N, BIT(10)|BIT9|BIT8, nhm_type);	 */ /* 0x890[9:8]=3			enable CCX */
	}
#endif
}

void
odm_auto_channel_select_init(
	void			*p_dm_void
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct PHY_DM_STRUCT					*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ACS_						*p_acs = &p_dm_odm->dm_acs;
	u8						i;

	if (!(p_dm_odm->support_ability & ODM_BB_NHM_CNT))
		return;

	if (p_acs->is_force_acs_result)
		return;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select_init()=========>\n"));

	p_acs->clean_channel_2g = 1;
	p_acs->clean_channel_5g = 36;

	for (i = 0; i < ODM_MAX_CHANNEL_2G; ++i) {
		p_acs->channel_info_2g[0][i] = 0;
		p_acs->channel_info_2g[1][i] = 0;
	}

	if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {
		for (i = 0; i < ODM_MAX_CHANNEL_5G; ++i) {
			p_acs->channel_info_5g[0][i] = 0;
			p_acs->channel_info_5g[1][i] = 0;
		}
	}
#endif
}

void
odm_auto_channel_select_reset(
	void			*p_dm_void
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct PHY_DM_STRUCT					*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ACS_						*p_acs = &p_dm_odm->dm_acs;

	if (!(p_dm_odm->support_ability & ODM_BB_NHM_CNT))
		return;

	if (p_acs->is_force_acs_result)
		return;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select_reset()=========>\n"));

	odm_auto_channel_select_setting(p_dm_odm, true); /* for 20ms measurement */
	phydm_nhm_counter_statistics_reset(p_dm_odm);
#endif
}

void
odm_auto_channel_select(
	void			*p_dm_void,
	u8			channel
)
{
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
	struct PHY_DM_STRUCT					*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ACS_						*p_acs = &p_dm_odm->dm_acs;
	u8						channel_idx = 0, search_idx = 0;
	u16						max_score = 0;

	if (!(p_dm_odm->support_ability & ODM_BB_NHM_CNT)) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_DIG, ODM_DBG_LOUD, ("odm_auto_channel_select(): Return: support_ability ODM_BB_NHM_CNT is disabled\n"));
		return;
	}

	if (p_acs->is_force_acs_result) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_DIG, ODM_DBG_LOUD, ("odm_auto_channel_select(): Force 2G clean channel = %d, 5G clean channel = %d\n",
			p_acs->clean_channel_2g, p_acs->clean_channel_5g));
		return;
	}

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select(): channel = %d=========>\n", channel));

	phydm_get_nhm_counter_statistics(p_dm_odm);
	odm_auto_channel_select_setting(p_dm_odm, false);

	if (channel >= 1 && channel <= 14) {
		channel_idx = channel - 1;
		p_acs->channel_info_2g[1][channel_idx]++;

		if (p_acs->channel_info_2g[1][channel_idx] >= 2)
			p_acs->channel_info_2g[0][channel_idx] = (p_acs->channel_info_2g[0][channel_idx] >> 1) +
				(p_acs->channel_info_2g[0][channel_idx] >> 2) + (p_dm_odm->nhm_cnt_0 >> 2);
		else
			p_acs->channel_info_2g[0][channel_idx] = p_dm_odm->nhm_cnt_0;

		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select(): nhm_cnt_0 = %d\n", p_dm_odm->nhm_cnt_0));
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_auto_channel_select(): Channel_Info[0][%d] = %d, Channel_Info[1][%d] = %d\n", channel_idx, p_acs->channel_info_2g[0][channel_idx], channel_idx, p_acs->channel_info_2g[1][channel_idx]));

		for (search_idx = 0; search_idx < ODM_MAX_CHANNEL_2G; search_idx++) {
			if (p_acs->channel_info_2g[1][search_idx] != 0) {
				if (p_acs->channel_info_2g[0][search_idx] >= max_score) {
					max_score = p_acs->channel_info_2g[0][search_idx];
					p_acs->clean_channel_2g = search_idx + 1;
				}
			}
		}
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("(1)odm_auto_channel_select(): 2G: clean_channel_2g = %d, max_score = %d\n",
				p_acs->clean_channel_2g, max_score));

	} else if (channel >= 36) {
		/* Need to do */
		p_acs->clean_channel_5g = channel;
	}
#endif
}

#if (DM_ODM_SUPPORT_TYPE & ODM_AP)

void
phydm_auto_channel_select_setting_ap(
	void   *p_dm_void,
	u32  setting,             /* 0: STORE_DEFAULT_NHM_SETTING; 1: RESTORE_DEFAULT_NHM_SETTING, 2: ACS_NHM_SETTING */
	u32  acs_step
)
{
	struct PHY_DM_STRUCT           *p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct rtl8192cd_priv       *priv           = p_dm_odm->priv;
	struct _ACS_                    *p_acs         = &p_dm_odm->dm_acs;

	ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("odm_AutoChannelSelectSettingAP()=========>\n"));

	/* 3 Store Default setting */
	if (setting == STORE_DEFAULT_NHM_SETTING) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("STORE_DEFAULT_NHM_SETTING\n"));

		if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {  /* store reg0x990, reg0x994, reg0x998, reg0x99c, Reg0x9a0 */
			p_acs->reg0x990 = odm_read_4byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC);                /* reg0x990 */
			p_acs->reg0x994 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11AC);           /* reg0x994 */
			p_acs->reg0x998 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11AC);       /* reg0x998 */
			p_acs->reg0x99c = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11AC);       /* Reg0x99c */
			p_acs->reg0x9a0 = odm_read_1byte(p_dm_odm, ODM_REG_NHM_TH8_11AC);                   /* Reg0x9a0, u8 */
		} else if (p_dm_odm->support_ic_type & ODM_IC_11N_SERIES) {
			p_acs->reg0x890 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11N);             /* reg0x890 */
			p_acs->reg0x894 = odm_read_4byte(p_dm_odm, ODM_REG_CCX_PERIOD_11N);                  /* reg0x894 */
			p_acs->reg0x898 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11N);         /* reg0x898 */
			p_acs->reg0x89c = odm_read_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11N);         /* Reg0x89c */
			p_acs->reg0xe28 = odm_read_1byte(p_dm_odm, ODM_REG_NHM_TH8_11N);                     /* Reg0xe28, u8 */
		}
	}

	/* 3 Restore Default setting */
	else if (setting == RESTORE_DEFAULT_NHM_SETTING) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("RESTORE_DEFAULT_NHM_SETTING\n"));

		if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {  /* store reg0x990, reg0x994, reg0x998, reg0x99c, Reg0x9a0 */
			odm_write_4byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC,          p_acs->reg0x990);
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11AC,     p_acs->reg0x994);
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11AC, p_acs->reg0x998);
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11AC, p_acs->reg0x99c);
			odm_write_1byte(p_dm_odm, ODM_REG_NHM_TH8_11AC,             p_acs->reg0x9a0);
		} else if (p_dm_odm->support_ic_type & ODM_IC_11N_SERIES) {
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11N,     p_acs->reg0x890);
			odm_write_4byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC,          p_acs->reg0x894);
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11N, p_acs->reg0x898);
			odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11N, p_acs->reg0x89c);
			odm_write_1byte(p_dm_odm, ODM_REG_NHM_TH8_11N,             p_acs->reg0xe28);
		}
	}

	/* 3 struct _ACS_ setting */
	else if (setting == ACS_NHM_SETTING) {
		ODM_RT_TRACE(p_dm_odm, ODM_COMP_ACS, ODM_DBG_LOUD, ("ACS_NHM_SETTING\n"));
		u16  period;
		period = 0x61a8;
		p_acs->acs_step = acs_step;

		if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {
			/* 4 Set NHM period, 0x990[31:16]=0x61a8, Time duration for NHM unit: 4us, 0x61a8=100ms */
			odm_write_2byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC + 2, period);
			/* 4 Set NHM ignore_cca=1, ignore_txon=1, ccx_en=0 */
			odm_set_bb_reg(p_dm_odm, ODM_REG_NHM_TH9_TH10_11AC, BIT(8) | BIT(9) | BIT(10), 3);

			if (p_acs->acs_step == 0) {
				/* 4 Set IGI */
				odm_set_bb_reg(p_dm_odm, 0xc50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x3E);
				if (get_rf_mimo_mode(priv) != MIMO_1T1R)
					odm_set_bb_reg(p_dm_odm, 0xe50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x3E);

				/* 4 Set struct _ACS_ NHM threshold */
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11AC, 0x82786e64);
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11AC, 0xffffff8c);
				odm_write_1byte(p_dm_odm, ODM_REG_NHM_TH8_11AC, 0xff);
				odm_write_2byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11AC + 2, 0xffff);

			} else if (p_acs->acs_step == 1) {
				/* 4 Set IGI */
				odm_set_bb_reg(p_dm_odm, 0xc50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x2A);
				if (get_rf_mimo_mode(priv) != MIMO_1T1R)
					odm_set_bb_reg(p_dm_odm, 0xe50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x2A);

				/* 4 Set struct _ACS_ NHM threshold */
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11AC, 0x5a50463c);
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11AC, 0xffffff64);

			}

		} else if (p_dm_odm->support_ic_type & ODM_IC_11N_SERIES) {
			/* 4 Set NHM period, 0x894[31:16]=0x61a8, Time duration for NHM unit: 4us, 0x61a8=100ms */
			odm_write_2byte(p_dm_odm, ODM_REG_CCX_PERIOD_11AC + 2, period);
			/* 4 Set NHM ignore_cca=1, ignore_txon=1, ccx_en=0 */
			odm_set_bb_reg(p_dm_odm, ODM_REG_NHM_TH9_TH10_11N, BIT(8) | BIT(9) | BIT(10), 3);

			if (p_acs->acs_step == 0) {
				/* 4 Set IGI */
				odm_set_bb_reg(p_dm_odm, 0xc50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x3E);
				if (get_rf_mimo_mode(priv) != MIMO_1T1R)
					odm_set_bb_reg(p_dm_odm, 0xc58, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x3E);

				/* 4 Set struct _ACS_ NHM threshold */
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11N, 0x82786e64);
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11N, 0xffffff8c);
				odm_write_1byte(p_dm_odm, ODM_REG_NHM_TH8_11N, 0xff);
				odm_write_2byte(p_dm_odm, ODM_REG_NHM_TH9_TH10_11N + 2, 0xffff);

			} else if (p_acs->acs_step == 1) {
				/* 4 Set IGI */
				odm_set_bb_reg(p_dm_odm, 0xc50, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x2A);
				if (get_rf_mimo_mode(priv) != MIMO_1T1R)
					odm_set_bb_reg(p_dm_odm, 0xc58, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6), 0x2A);

				/* 4 Set struct _ACS_ NHM threshold */
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH3_TO_TH0_11N, 0x5a50463c);
				odm_write_4byte(p_dm_odm, ODM_REG_NHM_TH7_TO_TH4_11N, 0xffffff64);

			}
		}
	}

}

void
phydm_get_nhm_statistics_ap(
	void       *p_dm_void,
	u32      idx,                /* @ 2G, Real channel number = idx+1 */
	u32      acs_step
)
{
	struct PHY_DM_STRUCT	*p_dm_odm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct rtl8192cd_priv     *priv    = p_dm_odm->priv;
	struct _ACS_                  *p_acs    = &p_dm_odm->dm_acs;
	u32                value32 = 0;
	u8                i;

	p_acs->acs_step = acs_step;

	if (p_dm_odm->support_ic_type & ODM_IC_11N_SERIES) {
		/* 4 Check if NHM result is ready */
		for (i = 0; i < 20; i++) {

			ODM_delay_ms(1);
			if (odm_get_bb_reg(p_dm_odm, REG_FPGA0_PSD_REPORT, BIT(17)))
				break;
		}

		/* 4 Get NHM Statistics */
		if (p_acs->acs_step == 1) {

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT7_TO_CNT4_11N);

			p_acs->nhm_cnt[idx][9] = (value32 & MASKBYTE1) >> 8;
			p_acs->nhm_cnt[idx][8] = (value32 & MASKBYTE0);

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT_11N);   /* ODM_REG_NHM_CNT3_TO_CNT0_11N */

			p_acs->nhm_cnt[idx][7] = (value32 & MASKBYTE3) >> 24;
			p_acs->nhm_cnt[idx][6] = (value32 & MASKBYTE2) >> 16;
			p_acs->nhm_cnt[idx][5] = (value32 & MASKBYTE1) >> 8;

		} else if (p_acs->acs_step == 2) {

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT_11N);  /* ODM_REG_NHM_CNT3_TO_CNT0_11N */

			p_acs->nhm_cnt[idx][4] = odm_read_1byte(p_dm_odm, ODM_REG_NHM_CNT7_TO_CNT4_11N);
			p_acs->nhm_cnt[idx][3] = (value32 & MASKBYTE3) >> 24;
			p_acs->nhm_cnt[idx][2] = (value32 & MASKBYTE2) >> 16;
			p_acs->nhm_cnt[idx][1] = (value32 & MASKBYTE1) >> 8;
			p_acs->nhm_cnt[idx][0] = (value32 & MASKBYTE0);
		}
	} else if (p_dm_odm->support_ic_type & ODM_IC_11AC_SERIES) {
		/* 4 Check if NHM result is ready */
		for (i = 0; i < 20; i++) {

			ODM_delay_ms(1);
			if (odm_get_bb_reg(p_dm_odm, ODM_REG_NHM_DUR_READY_11AC, BIT(16)))
				break;
		}

		if (p_acs->acs_step == 1) {

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT7_TO_CNT4_11AC);

			p_acs->nhm_cnt[idx][9] = (value32 & MASKBYTE1) >> 8;
			p_acs->nhm_cnt[idx][8] = (value32 & MASKBYTE0);

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT_11AC);    /* ODM_REG_NHM_CNT3_TO_CNT0_11AC */

			p_acs->nhm_cnt[idx][7] = (value32 & MASKBYTE3) >> 24;
			p_acs->nhm_cnt[idx][6] = (value32 & MASKBYTE2) >> 16;
			p_acs->nhm_cnt[idx][5] = (value32 & MASKBYTE1) >> 8;

		} else if (p_acs->acs_step == 2) {

			value32 = odm_read_4byte(p_dm_odm, ODM_REG_NHM_CNT_11AC);     /* ODM_REG_NHM_CNT3_TO_CNT0_11AC */

			p_acs->nhm_cnt[idx][4] = odm_read_1byte(p_dm_odm, ODM_REG_NHM_CNT7_TO_CNT4_11AC);
			p_acs->nhm_cnt[idx][3] = (value32 & MASKBYTE3) >> 24;
			p_acs->nhm_cnt[idx][2] = (value32 & MASKBYTE2) >> 16;
			p_acs->nhm_cnt[idx][1] = (value32 & MASKBYTE1) >> 8;
			p_acs->nhm_cnt[idx][0] = (value32 & MASKBYTE0);
		}
	}

}

#endif
