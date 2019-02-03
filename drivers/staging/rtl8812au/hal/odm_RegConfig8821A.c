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
 ******************************************************************************/

#include "odm_precomp.h"

void
odm_ConfigRFReg_8821A(
	PDM_ODM_T				pDM_Odm,
	u32					Addr,
	u32					Data,
	ODM_RF_RADIO_PATH_E     RF_PATH,
	u32				    RegAddr
	)
{
    if(Addr == 0xfe || Addr == 0xffe)
	{
		#ifdef CONFIG_LONG_DELAY_ISSUE
		ODM_sleep_ms(50);
		#else
		ODM_delay_ms(50);
		#endif
	}
	else if (Addr == 0xfd)
	{
		ODM_delay_ms(5);
	}
	else if (Addr == 0xfc)
	{
		ODM_delay_ms(1);
	}
	else if (Addr == 0xfb)
	{
		ODM_delay_us(50);
	}
	else if (Addr == 0xfa)
	{
		ODM_delay_us(5);
	}
	else if (Addr == 0xf9)
	{
		ODM_delay_us(1);
	}
	else
	{
		ODM_SetRFReg(pDM_Odm, RF_PATH, RegAddr, bRFRegOffsetMask, Data);
		// Add 1us delay between BB/RF register setting.
		ODM_delay_us(1);
	}
}


void odm_ConfigRF_RadioA_8821A(PDM_ODM_T pDM_Odm, u32 Addr, u32 Data)
{
	u32  content = 0x1000; // RF_Content: radioa_txt
	u32	maskforPhySet= (u32)(content&0xE000);

	odm_ConfigRFReg_8821A(pDM_Odm, Addr, Data, ODM_RF_PATH_A, Addr|maskforPhySet);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_TRACE, ("===> ODM_ConfigRFWithHeaderFile: [RadioA] %08X %08X\n", Addr, Data));
}

// 8821 no RF B
/*
void
odm_ConfigRF_RadioB_8821A(
	PDM_ODM_T				pDM_Odm,
	u32					Addr,
	u32					Data
	)
{
	u32  content = 0x1001; // RF_Content: radiob_txt
	u32	maskforPhySet= (u32)(content&0xE000);

    odm_ConfigRFReg_8812A(pDM_Odm, Addr, Data, ODM_RF_PATH_B, Addr|maskforPhySet);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_TRACE, ("===> ODM_ConfigRFWithHeaderFile: [RadioB] %08X %08X\n", Addr, Data));

}
*/

void
odm_ConfigMAC_8821A(
	PDM_ODM_T	pDM_Odm,
	u32		Addr,
	u8		Data
	)
{
	ODM_Write1Byte(pDM_Odm, Addr, Data);
    ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_TRACE, ("===> ODM_ConfigMACWithHeaderFile: [MAC_REG] %08X %08X\n", Addr, Data));
}

void
odm_ConfigBB_AGC_8821A(
    PDM_ODM_T	pDM_Odm,
    u32		Addr,
    u32		Bitmask,
    u32		Data
    )
{
	ODM_SetBBReg(pDM_Odm, Addr, Bitmask, Data);
	// Add 1us delay between BB/RF register setting.
	ODM_delay_us(1);

    ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_TRACE, ("===> ODM_ConfigBBWithHeaderFile: [AGC_TAB] %08X %08X\n", Addr, Data));
}

void
odm_ConfigBB_PHY_REG_PG_8821A(
	PDM_ODM_T	pDM_Odm,
    u32		Addr,
    u32		Bitmask,
    u32		Data
    )
{
	if (Addr == 0xfe)
		#ifdef CONFIG_LONG_DELAY_ISSUE
		ODM_sleep_ms(50);
		#else
		ODM_delay_ms(50);
		#endif
	else if (Addr == 0xfd)
		ODM_delay_ms(5);
	else if (Addr == 0xfc)
		ODM_delay_ms(1);
	else if (Addr == 0xfb)
		ODM_delay_us(50);
	else if (Addr == 0xfa)
		ODM_delay_us(5);
	else if (Addr == 0xf9)
		ODM_delay_us(1);

	ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_LOUD, ("===> @@@@@@@ ODM_ConfigBBWithHeaderFile: [PHY_REG] %08X %08X %08X\n", Addr, Bitmask, Data));

	storePwrIndexDiffRateOffset(pDM_Odm->Adapter, Addr, Bitmask, Data);
}

void
odm_ConfigBB_PHY_8821A(
	PDM_ODM_T	pDM_Odm,
    u32		Addr,
    u32		Bitmask,
    u32		Data
    )
{
	if (Addr == 0xfe)
		#ifdef CONFIG_LONG_DELAY_ISSUE
		ODM_sleep_ms(50);
		#else
		ODM_delay_ms(50);
		#endif
	else if (Addr == 0xfd)
		ODM_delay_ms(5);
	else if (Addr == 0xfc)
		ODM_delay_ms(1);
	else if (Addr == 0xfb)
		ODM_delay_us(50);
	else if (Addr == 0xfa)
		ODM_delay_us(5);
	else if (Addr == 0xf9)
		ODM_delay_us(1);
	else if (Addr == 0xa24)
		pDM_Odm->RFCalibrateInfo.RegA24 = Data;
	ODM_SetBBReg(pDM_Odm, Addr, Bitmask, Data);

	// Add 1us delay between BB/RF register setting.
	ODM_delay_us(1);
    ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_TRACE, ("===> ODM_ConfigBBWithHeaderFile: [PHY_REG] %08X %08X\n", Addr, Data));
}

void
odm_ConfigBB_TXPWR_LMT_8821A(
	PDM_ODM_T	pDM_Odm,
	const u8 *		Regulation,
	const u8 *		Band,
	const u8 *		Bandwidth,
	const u8 *		RateSection,
	const u8 *		RfPath,
	const u8 *		Channel,
	const u8 *		PowerLimit
    )
{
	PHY_SetPowerLimitTableValue(pDM_Odm, Regulation, Band,
		Bandwidth, RateSection, RfPath, Channel, PowerLimit);
}
