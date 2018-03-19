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

//============================================================
// include files
//============================================================


#include "odm_precomp.h"

#define READ_AND_CONFIG_MP(ic, txt) (ODM_ReadAndConfig_MP_##ic##txt(pDM_Odm))


#define READ_AND_CONFIG     READ_AND_CONFIG_MP


#define READ_FIRMWARE_MP(ic, txt)		(ODM_ReadFirmware_MP_##ic##txt(pDM_Odm, pFirmware, pSize))

#define READ_FIRMWARE     READ_FIRMWARE_MP

u8 odm_QueryRxPwrPercentage(s1Byte AntPower)
{
	if ((AntPower <= -100) || (AntPower >= 20))
		return	0;
	else if (AntPower >= 0)
		return	100;
	else
		return	100 + AntPower;

}

// 2012/01/12 MH MOve some signal strength smooth method to MP HAL layer.
// IF other SW team do not support the feature, remove this section.??
s4Byte odm_SignalScaleMapping_92CSeries_patch_RT_CID_819x_Lenovo(PDM_ODM_T pDM_Odm, s4Byte CurrSig)
{
	return 0;
}

s4Byte odm_SignalScaleMapping_92CSeries_patch_RT_CID_819x_Netcore(PDM_ODM_T pDM_Odm, s4Byte CurrSig)
{
	return 0;
}


s4Byte odm_SignalScaleMapping_92CSeries(PDM_ODM_T pDM_Odm, s4Byte CurrSig)
{
	s4Byte RetSig;

	if ((pDM_Odm->SupportInterface  == ODM_ITRF_USB) || (pDM_Odm->SupportInterface  == ODM_ITRF_SDIO))
	{
		if (CurrSig >= 51 && CurrSig <= 100)
			RetSig = 100;
		else if (CurrSig >= 41 && CurrSig <= 50)
			RetSig = 80 + ((CurrSig - 40)*2);
		else if (CurrSig >= 31 && CurrSig <= 40)
			RetSig = 66 + (CurrSig - 30);
		else if (CurrSig >= 21 && CurrSig <= 30)
			RetSig = 54 + (CurrSig - 20);
		else if (CurrSig >= 10 && CurrSig <= 20)
			RetSig = 42 + (((CurrSig - 10) * 2) / 3);
		else if (CurrSig >= 5 && CurrSig <= 9)
			RetSig = 22 + (((CurrSig - 5) * 3) / 2);
		else if (CurrSig >= 1 && CurrSig <= 4)
			RetSig = 6 + (((CurrSig - 1) * 3) / 2);
		else
			RetSig = CurrSig;
	}

	return RetSig;
}

s4Byte odm_SignalScaleMapping(PDM_ODM_T pDM_Odm, s4Byte CurrSig)
{
	if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
	    (pDM_Odm->SupportInterface  != ODM_ITRF_PCIE) && //USB & SDIO
	    (pDM_Odm->PatchID==10)) {//pMgntInfo->CustomerID == RT_CID_819x_Netcore
		return odm_SignalScaleMapping_92CSeries_patch_RT_CID_819x_Netcore(pDM_Odm,CurrSig);
	} else if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
		   (pDM_Odm->SupportInterface  == ODM_ITRF_PCIE) &&
		   (pDM_Odm->PatchID==19)) { //pMgntInfo->CustomerID == RT_CID_819x_Lenovo)
		return odm_SignalScaleMapping_92CSeries_patch_RT_CID_819x_Lenovo(pDM_Odm, CurrSig);
	} else {
		return odm_SignalScaleMapping_92CSeries(pDM_Odm,CurrSig);
	}

}

static u8 odm_SQ_process_patch_RT_CID_819x_Lenovo(PDM_ODM_T	pDM_Odm, u8 isCCKrate, u8 PWDB_ALL, u8 path, u8 RSSI)
{
	return 0;
}

static u8 odm_EVMdbToPercentage(s1Byte Value)
{
	//
	// -33dB~0dB to 0%~99%
	//
	s1Byte ret_val;

	ret_val = Value;

	if (ret_val >= 0)
		ret_val = 0;
	if (ret_val <= -33)
		ret_val = -33;

	ret_val = 0 - ret_val;
	ret_val*=3;

	if (ret_val == 99)
		ret_val = 100;
	return ret_val;
}

static u8 odm_EVMdbm_JaguarSeries(s1Byte Value)
{
	s1Byte ret_val = Value;

	// -33dB~0dB to 33dB ~ 0dB
	if (ret_val == -128)
		ret_val = 127;
	else if (ret_val < 0)
		ret_val = 0 - ret_val;
	ret_val  = ret_val >> 1;
	return ret_val;
}

static u16 odm_Cfo(s1Byte Value)
{
	s2Byte  ret_val;

	if (Value < 0)
	{
		ret_val = 0 - Value;
		ret_val = (ret_val << 1) + (ret_val >> 1) ;  //  *2.5~=312.5/2^7
		ret_val = ret_val | BIT12;  // set bit12 as 1 for negative cfo
	}
	else
	{
		ret_val = Value;
		ret_val = (ret_val << 1) + (ret_val>>1) ;  //  *2.5~=312.5/2^7
	}
	return ret_val;
}


void odm_RxPhyStatus92CSeries_Parsing(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, u8 * pPhyStatus, PODM_PACKET_INFO_T pPktinfo)
{
	SWAT_T				*pDM_SWAT_Table = &pDM_Odm->DM_SWAT_Table;
	u8				i, Max_spatial_stream;
	s1Byte				rx_pwr[4], rx_pwr_all=0;
	u8				EVM, PWDB_ALL = 0, PWDB_ALL_BT;
	u8				RSSI, total_rssi=0;
	bool				isCCKrate=false;
	u8				rf_rx_num = 0;
	u8				cck_highpwr = 0;
	u8				LNA_idx, VGA_idx;
	PPHY_STATUS_RPT_8192CD_T pPhyStaRpt = (PPHY_STATUS_RPT_8192CD_T)pPhyStatus;

	isCCKrate = (pPktinfo->DataRate <= DESC92C_RATE11M)?true :false;
	pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_A] = -1;
	pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_B] = -1;


	if (isCCKrate) {
		u8 report;
		u8 cck_agc_rpt;

		pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK++;
		// (1)Hardware does not provide RSSI for CCK
		// (2)PWDB, Average PWDB cacluated by hardware (for rate adaptive)

		cck_highpwr = pDM_Odm->bCckHighPower;

		//2011.11.28 LukeLee: 88E use different LNA & VGA gain table
		//The RSSI formula should be modified according to the gain table
		//In 88E, cck_highpwr is always set to 1
		if (pDM_Odm->SupportICType & (ODM_RTL8188E | ODM_RTL8192E | ODM_RTL8723B)) {
			LNA_idx = ((cck_agc_rpt & 0xE0) >>5);
			VGA_idx = (cck_agc_rpt & 0x1F);
			if (pDM_Odm->SupportICType & (ODM_RTL8188E|ODM_RTL8192E)) {
				switch(LNA_idx) {
				case 7:
					if (VGA_idx <= 27)
						rx_pwr_all = -100 + 2*(27-VGA_idx); //VGA_idx = 27~2
					else
						rx_pwr_all = -100;
					break;
				case 6:
					rx_pwr_all = -48 + 2*(2-VGA_idx); //VGA_idx = 2~0
					break;
				case 5:
					rx_pwr_all = -42 + 2*(7-VGA_idx); //VGA_idx = 7~5
					break;
				case 4:
					rx_pwr_all = -36 + 2*(7-VGA_idx); //VGA_idx = 7~4
					break;
				case 3:
					//rx_pwr_all = -28 + 2*(7-VGA_idx); //VGA_idx = 7~0
					rx_pwr_all = -24 + 2*(7-VGA_idx); //VGA_idx = 7~0
					break;
				case 2:
					if (cck_highpwr)
						rx_pwr_all = -12 + 2*(5-VGA_idx); //VGA_idx = 5~0
					else
						rx_pwr_all = -6+ 2*(5-VGA_idx);
					break;
				case 1:
					rx_pwr_all = 8-2*VGA_idx;
					break;
				case 0:
					rx_pwr_all = 14-2*VGA_idx;
					break;
				default:
					break;
				}
				rx_pwr_all += 6;

				//2012.10.08 LukeLee: Modify for 92E CCK RSSI
				if (pDM_Odm->SupportICType == ODM_RTL8192E)
					rx_pwr_all += 10;

				PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);
				if (cck_highpwr == false) {
					if (PWDB_ALL >= 80)
						PWDB_ALL = ((PWDB_ALL-80)<<1)+((PWDB_ALL-80)>>1)+80;
					else if ((PWDB_ALL <= 78) && (PWDB_ALL >= 20))
						PWDB_ALL += 3;
					if (PWDB_ALL>100)
						PWDB_ALL = 100;
				}
			}
		} else {
			if (!cck_highpwr) {
				report =( cck_agc_rpt & 0xc0 )>>6;
				switch(report) {
				// 03312009 modified by cosa
				// Modify the RF RNA gain value to -40, -20, -2, 14 by Jenyu's suggestion
				// Note: different RF with the different RNA gain.
				case 0x3:
					rx_pwr_all = -46 - (cck_agc_rpt & 0x3e);
					break;
				case 0x2:
					rx_pwr_all = -26 - (cck_agc_rpt & 0x3e);
					break;
				case 0x1:
					rx_pwr_all = -12 - (cck_agc_rpt & 0x3e);
					break;
				case 0x0:
					rx_pwr_all = 16 - (cck_agc_rpt & 0x3e);
					break;
				}
			} else {
				report = (cck_agc_rpt & 0x60)>>5;
				switch(report) {
				case 0x3:
					rx_pwr_all = -46 - ((cck_agc_rpt & 0x1f)<<1) ;
					break;
				case 0x2:
					rx_pwr_all = -26 - ((cck_agc_rpt & 0x1f)<<1);
					break;
				case 0x1:
					rx_pwr_all = -12 - ((cck_agc_rpt & 0x1f)<<1) ;
					break;
				case 0x0:
					rx_pwr_all = 16 - ((cck_agc_rpt & 0x1f)<<1) ;
					break;
				}
			}

			PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);

			//Modification for ext-LNA board
			if (pDM_Odm->BoardType & (ODM_BOARD_EXT_LNA | ODM_BOARD_EXT_PA)) {
				if ((cck_agc_rpt>>7) == 0) {
					PWDB_ALL = (PWDB_ALL>94)?100:(PWDB_ALL +6);
				} else {
					if (PWDB_ALL > 38)
						PWDB_ALL -= 16;
					else
						PWDB_ALL = (PWDB_ALL<=16)?(PWDB_ALL>>2):(PWDB_ALL -12);
				}

				//CCK modification
				if (PWDB_ALL > 25 && PWDB_ALL <= 60)
					PWDB_ALL += 6;
			} else { //Modification for int-LNA board
				if (PWDB_ALL > 99)
					PWDB_ALL -= 8;
				else if (PWDB_ALL > 50 && PWDB_ALL <= 68)
					PWDB_ALL += 4;
			}
		}

		pPhyInfo->RxPWDBAll = PWDB_ALL;
		pPhyInfo->BTRxRSSIPercentage = PWDB_ALL;
		pPhyInfo->RecvSignalPower = rx_pwr_all;
		// (3) Get Signal Quality (EVM)
		if (pPktinfo->bPacketMatchBSSID) {
			u8	SQ,SQ_rpt;

			if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
			     (pDM_Odm->PatchID==RT_CID_819x_Lenovo)) {
				SQ = odm_SQ_process_patch_RT_CID_819x_Lenovo(pDM_Odm,isCCKrate,PWDB_ALL,0,0);
			} else if (pPhyInfo->RxPWDBAll > 40 && !pDM_Odm->bInHctTest) {
				SQ = 100;
			} else {
				SQ_rpt = pPhyStaRpt->cck_sig_qual_ofdm_pwdb_all;

				if (SQ_rpt > 64)
					SQ = 0;
				else if (SQ_rpt < 20)
					SQ = 100;
				else
					SQ = ((64-SQ_rpt) * 100) / 44;

			}

			pPhyInfo->SignalQuality = SQ;
			pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_A] = SQ;
			pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_B] = -1;
		}
	} else { //is OFDM rate
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM++;

		// (1)Get RSSI for HT rate
		for(i = ODM_RF_PATH_A; i < ODM_RF_PATH_MAX; i++) {
			// 2008/01/30 MH we will judge RF RX path now.
			if (pDM_Odm->RFPathRxEnable & BIT(i))
				rf_rx_num++;

			rx_pwr[i] = ((pPhyStaRpt->path_agc[i].gain& 0x3F)*2) - 110;

			pPhyInfo->RxPwr[i] = rx_pwr[i];

			/* Translate DBM to percentage. */
			RSSI = odm_QueryRxPwrPercentage(rx_pwr[i]);
			total_rssi += RSSI;

			//Modification for ext-LNA board
			if (pDM_Odm->SupportICType&ODM_RTL8192C) {
				if (pDM_Odm->BoardType & (ODM_BOARD_EXT_LNA | ODM_BOARD_EXT_PA)) {
					if ((pPhyStaRpt->path_agc[i].trsw) == 1)
						RSSI = (RSSI>94)?100:(RSSI +6);
					else
						RSSI = (RSSI<=16)?(RSSI>>3):(RSSI -16);

					if ((RSSI <= 34) && (RSSI >=4))
						RSSI -= 4;
				}
			}

			pPhyInfo->RxMIMOSignalStrength[i] =(u8) RSSI;

			//Get Rx snr value in DB
			pPhyInfo->RxSNR[i] = pDM_Odm->PhyDbgInfo.RxSNRdB[i] = (s4Byte)(pPhyStaRpt->path_rxsnr[i]/2);

			/* Record Signal Strength for next packet */
			if (pPktinfo->bPacketMatchBSSID) {
				if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
					(pDM_Odm->PatchID==RT_CID_819x_Lenovo)) {
					if (i==ODM_RF_PATH_A)
						pPhyInfo->SignalQuality = odm_SQ_process_patch_RT_CID_819x_Lenovo(pDM_Odm,isCCKrate,PWDB_ALL,i,RSSI);
				}
			}
		}

		//
		// (2)PWDB, Average PWDB cacluated by hardware (for rate adaptive)
		//
		rx_pwr_all = (((pPhyStaRpt->cck_sig_qual_ofdm_pwdb_all) >> 1 )& 0x7f) -110;

		PWDB_ALL_BT = PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);

		pPhyInfo->RxPWDBAll = PWDB_ALL;
		pPhyInfo->BTRxRSSIPercentage = PWDB_ALL_BT;
		pPhyInfo->RxPower = rx_pwr_all;
		pPhyInfo->RecvSignalPower = rx_pwr_all;

		if ((pDM_Odm->SupportPlatform == ODM_WIN) && (pDM_Odm->PatchID==19)) {
			//do nothing
		} else {//pMgntInfo->CustomerID != RT_CID_819x_Lenovo
			//
			// (3)EVM of HT rate
			//
			if (pPktinfo->DataRate >=DESC92C_RATEMCS8 && pPktinfo->DataRate <=DESC92C_RATEMCS15)
				Max_spatial_stream = 2; //both spatial stream make sense
			else
				Max_spatial_stream = 1; //only spatial stream 1 makes sense

			for(i=0; i<Max_spatial_stream; i++) {
				// Do not use shift operation like "rx_evmX >>= 1" because the compilor of free build environment
				// fill most significant bit to "zero" when doing shifting operation which may change a negative
				// value to positive one, then the dbm value (which is supposed to be negative)  is not correct anymore.
				EVM = odm_EVMdbToPercentage( (pPhyStaRpt->stream_rxevm[i] ));	//dbm

				if (pPktinfo->bPacketMatchBSSID) {
					if (i==ODM_RF_PATH_A) // Fill value in RFD, Get the first spatial stream only
						pPhyInfo->SignalQuality = (u8)(EVM & 0xff);
					pPhyInfo->RxMIMOSignalQuality[i] = (u8)(EVM & 0xff);
				}
			}
		}

		//2 For dynamic ATC switch
		if (pDM_Odm->SupportAbility & ODM_BB_DYNAMIC_ATC) {
			if (pPktinfo->bPacketMatchBSSID) {
				//3 Update CFO report for path-A & path-B
				 for (i = ODM_RF_PATH_A; i < ODM_RF_PATH_MAX; i++)
					pDM_Odm->CFO_tail[i] = (int)pPhyStaRpt->path_cfotail[i];

				//3 Update packet counter
				if (pDM_Odm->packetCount == 0xffffffff)
					pDM_Odm->packetCount = 0;
				else
					pDM_Odm->packetCount++;

				//ODM_RT_TRACE(pDM_Odm, ODM_COMP_DYNAMIC_ATC, ODM_DBG_LOUD,
					//("pPhyStaRpt->path_cfotail[i] = 0x%x, pDM_Odm->CFO_tail[i] = 0x%x\n", pPhyStaRpt->path_cfotail[0], pDM_Odm->CFO_tail[1]));
			}
		}

	}
	//UI BSS List signal strength(in percentage), make it good looking, from 0~100.
	//It is assigned to the BSS List in GetValueFromBeaconOrProbeRsp().
	if (isCCKrate) {
		pPhyInfo->SignalStrength = (u8)(odm_SignalScaleMapping(pDM_Odm, PWDB_ALL));//PWDB_ALL;
	} else {
		if (rf_rx_num != 0)
			pPhyInfo->SignalStrength = (u8)(odm_SignalScaleMapping(pDM_Odm, total_rssi/=rf_rx_num));
	}

	//For 92C/92D HW (Hybrid) Antenna Diversity
#if (defined(CONFIG_HW_ANTENNA_DIVERSITY))
	pDM_SWAT_Table->antsel = pPhyStaRpt->ant_sel;
	//For 88E HW Antenna Diversity
	pDM_Odm->DM_FatTable.antsel_rx_keep_0 = pPhyStaRpt->ant_sel;
	pDM_Odm->DM_FatTable.antsel_rx_keep_1 = pPhyStaRpt->ant_sel_b;
	pDM_Odm->DM_FatTable.antsel_rx_keep_2 = pPhyStaRpt->antsel_rx_keep_2;
#endif
}

void odm_RxPhyStatusJaguarSeries_Parsing(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, u8 * pPhyStatus, PODM_PACKET_INFO_T pPktinfo)
{
	u8				i, Max_spatial_stream;
	s1Byte				rx_pwr[4], rx_pwr_all=0;
	u8				EVM, EVMdbm, PWDB_ALL = 0, PWDB_ALL_BT;
	u8				RSSI, total_rssi=0;
	u8				isCCKrate=0;
	u8				rf_rx_num = 0;
	u8				cck_highpwr = 0;
	u8				LNA_idx, VGA_idx;

	PPHY_STATUS_RPT_8812_T pPhyStaRpt = (PPHY_STATUS_RPT_8812_T)pPhyStatus;

	if (pPktinfo->DataRate <= DESC_RATE54M) {
		switch(pPhyStaRpt->r_RFMOD){
		case 1:
			if (pPhyStaRpt->sub_chnl == 0)
				pPhyInfo->BandWidth = 1;
			else
				pPhyInfo->BandWidth = 0;
			break;

		case 2:
			if (pPhyStaRpt->sub_chnl == 0)
				pPhyInfo->BandWidth = 2;
			else if (pPhyStaRpt->sub_chnl == 9 || pPhyStaRpt->sub_chnl == 10)
				pPhyInfo->BandWidth = 1;
			else
				pPhyInfo->BandWidth = 0;
			break;

		default:	case 0:
			pPhyInfo->BandWidth = 0;
			break;
		}
	}

	if (pPktinfo->DataRate <= DESC_RATE11M)
		isCCKrate = true;
	else
		isCCKrate = false;

	pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_A] = -1;
	pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_B] = -1;


	if (isCCKrate) {
		u8 cck_agc_rpt;

		pDM_Odm->PhyDbgInfo.NumQryPhyStatusCCK++;
		//
		// (1)Hardware does not provide RSSI for CCK
		// (2)PWDB, Average PWDB cacluated by hardware (for rate adaptive)
		//

		//if (pHalData->eRFPowerState == eRfOn)
			cck_highpwr = pDM_Odm->bCckHighPower;
		//else
		//	cck_highpwr = false;

		cck_agc_rpt =  pPhyStaRpt->cfosho[0] ;

		LNA_idx = ((cck_agc_rpt & 0xE0) >>5);
		VGA_idx = (cck_agc_rpt & 0x1F);
		if (pDM_Odm->SupportICType == ODM_RTL8812) {
			switch(LNA_idx) {
			case 7:
				if (VGA_idx <= 27)
					rx_pwr_all = -100 + 2*(27-VGA_idx); //VGA_idx = 27~2
				else
					rx_pwr_all = -100;
				break;
			case 6:
					rx_pwr_all = -48 + 2*(2-VGA_idx); //VGA_idx = 2~0
				break;
			case 5:
					rx_pwr_all = -42 + 2*(7-VGA_idx); //VGA_idx = 7~5
				break;
			case 4:
					rx_pwr_all = -36 + 2*(7-VGA_idx); //VGA_idx = 7~4
				break;
			case 3:
					rx_pwr_all = -24 + 2*(7-VGA_idx); //VGA_idx = 7~0
				break;
			case 2:
				if (cck_highpwr)
					rx_pwr_all = -12 + 2*(5-VGA_idx); //VGA_idx = 5~0
				else
					rx_pwr_all = -6+ 2*(5-VGA_idx);
				break;
			case 1:
					rx_pwr_all = 8-2*VGA_idx;
				break;
			case 0:
					rx_pwr_all = 14-2*VGA_idx;
				break;
			default:
				break;
			}
			rx_pwr_all += 6;
			PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);
			if (cck_highpwr == false) {
				if (PWDB_ALL >= 80)
					PWDB_ALL = ((PWDB_ALL-80)<<1)+((PWDB_ALL-80)>>1)+80;
				else if ((PWDB_ALL <= 78) && (PWDB_ALL >= 20))
					PWDB_ALL += 3;
				if (PWDB_ALL>100)
					PWDB_ALL = 100;
			}
		} else if (pDM_Odm->SupportICType == ODM_RTL8821) {
			s1Byte Pout = -6;

			switch(LNA_idx) {
			case 5:
				rx_pwr_all = Pout -32 -(2*VGA_idx);
					break;
			case 4:
				rx_pwr_all = Pout -24 -(2*VGA_idx);
					break;
			case 2:
				rx_pwr_all = Pout -11 -(2*VGA_idx);
					break;
			case 1:
				rx_pwr_all = Pout + 5 -(2*VGA_idx);
					break;
			case 0:
				rx_pwr_all = Pout + 21 -(2*VGA_idx);
					break;
			}
			PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);
		}

		pPhyInfo->RxPWDBAll = PWDB_ALL;
			pPhyInfo->BTRxRSSIPercentage = PWDB_ALL;
		pPhyInfo->RecvSignalPower = rx_pwr_all;
		//
		// (3) Get Signal Quality (EVM)
		//
		if (pPktinfo->bPacketMatchBSSID) {
			u8	SQ,SQ_rpt;

			if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
				(pDM_Odm->PatchID==RT_CID_819x_Lenovo)){
				SQ = odm_SQ_process_patch_RT_CID_819x_Lenovo(pDM_Odm,isCCKrate,PWDB_ALL,0,0);
			} else if (pPhyInfo->RxPWDBAll > 40 && !pDM_Odm->bInHctTest){
				SQ = 100;
			} else{
				SQ_rpt = pPhyStaRpt->pwdb_all;

				if (SQ_rpt > 64)
					SQ = 0;
				else if (SQ_rpt < 20)
					SQ = 100;
				else
					SQ = ((64-SQ_rpt) * 100) / 44;

			}

			pPhyInfo->SignalQuality = SQ;
			pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_A] = SQ;
			pPhyInfo->RxMIMOSignalQuality[ODM_RF_PATH_B] = -1;
		}
	} else { //is OFDM rate
		pDM_Odm->PhyDbgInfo.NumQryPhyStatusOFDM++;

		//
		// (1)Get RSSI for OFDM rate
		//

		for(i = ODM_RF_PATH_A; i < ODM_RF_PATH_MAX; i++) {
			// 2008/01/30 MH we will judge RF RX path now.
			if (pDM_Odm->RFPathRxEnable & BIT(i)) {
				rf_rx_num++;
			}
			rx_pwr[i] = (pPhyStaRpt->gain_trsw[i]&0x7F) - 110;

			pPhyInfo->RxPwr[i] = rx_pwr[i];

			/* Translate DBM to percentage. */
			RSSI = odm_QueryRxPwrPercentage(rx_pwr[i]);

			total_rssi += RSSI;

			pPhyInfo->RxMIMOSignalStrength[i] =(u8) RSSI;

			//Get Rx snr value in DB
			pPhyInfo->RxSNR[i] = pDM_Odm->PhyDbgInfo.RxSNRdB[i] = pPhyStaRpt->rxsnr[i]/2;

			//
			// (2) CFO_short  & CFO_tail
			//
			pPhyInfo->Cfo_short[i] = odm_Cfo( (pPhyStaRpt->cfosho[i]) );
			pPhyInfo->Cfo_tail[i] = odm_Cfo( (pPhyStaRpt->cfotail[i]) );

			/* Record Signal Strength for next packet */
			if (pPktinfo->bPacketMatchBSSID) {
				if ((pDM_Odm->SupportPlatform == ODM_WIN) &&
				    (pDM_Odm->PatchID==RT_CID_819x_Lenovo)) {
					if (i==ODM_RF_PATH_A)
						pPhyInfo->SignalQuality = odm_SQ_process_patch_RT_CID_819x_Lenovo(pDM_Odm,isCCKrate,PWDB_ALL,i,RSSI);

				}
			}
		}


		//
		// (3)PWDB, Average PWDB cacluated by hardware (for rate adaptive)
		//
		//2012.05.25 LukeLee: Testchip AGC report is wrong, it should be restored back to old formula in MP chip
		if ((pDM_Odm->SupportICType & (ODM_RTL8812|ODM_RTL8821)) && (!pDM_Odm->bIsMPChip))
			rx_pwr_all = (pPhyStaRpt->pwdb_all& 0x7f) -110;
		else
			rx_pwr_all = (((pPhyStaRpt->pwdb_all) >> 1 )& 0x7f) -110;	 //OLD FORMULA


		PWDB_ALL_BT = PWDB_ALL = odm_QueryRxPwrPercentage(rx_pwr_all);


		pPhyInfo->RxPWDBAll = PWDB_ALL;
		pPhyInfo->BTRxRSSIPercentage = PWDB_ALL_BT;
		pPhyInfo->RxPower = rx_pwr_all;
		pPhyInfo->RecvSignalPower = rx_pwr_all;

		if ((pDM_Odm->SupportPlatform == ODM_WIN) &&(pDM_Odm->PatchID==19)){
			//do nothing
		} else{//pMgntInfo->CustomerID != RT_CID_819x_Lenovo
			//
			// (4)EVM of OFDM rate
			//
			if ((pPktinfo->DataRate>=DESC_RATEMCS8) &&
			    (pPktinfo->DataRate <=DESC_RATEMCS15))
				Max_spatial_stream = 2;
			else if ((pPktinfo->DataRate>=DESC_RATEVHTSS2MCS0) &&
				 (pPktinfo->DataRate <=DESC_RATEVHTSS2MCS9))
				Max_spatial_stream = 2;
			else
				Max_spatial_stream = 1;

			for (i=0; i<Max_spatial_stream; i++) {
				// Do not use shift operation like "rx_evmX >>= 1" because the compilor of free build environment
				// fill most significant bit to "zero" when doing shifting operation which may change a negative
				// value to positive one, then the dbm value (which is supposed to be negative)  is not correct anymore.
				EVM = odm_EVMdbToPercentage( (pPhyStaRpt->rxevm[i] ));	//dbm
				EVMdbm = odm_EVMdbm_JaguarSeries(pPhyStaRpt->rxevm[i]);

				if (pPktinfo->bPacketMatchBSSID) {
					if (i==ODM_RF_PATH_A) // Fill value in RFD, Get the first spatial stream only
						pPhyInfo->SignalQuality = EVM;
					pPhyInfo->RxMIMOSignalQuality[i] = EVM;
					pPhyInfo->RxMIMOEVMdbm[i] = EVMdbm;
				}
			}
		}
		//2 For dynamic ATC switch
		if (pDM_Odm->SupportAbility & ODM_BB_DYNAMIC_ATC) {
			if (pPktinfo->bPacketMatchBSSID) {
				//3 Update CFO report for path-A & path-B
				 for(i = ODM_RF_PATH_A; i < ODM_RF_PATH_MAX; i++)
					pDM_Odm->CFO_tail[i] = (int)pPhyStaRpt->cfotail[i];

				//3 Update packet counter
				if (pDM_Odm->packetCount == 0xffffffff)
					pDM_Odm->packetCount = 0;
				else
					pDM_Odm->packetCount++;
			}
		}
	}

	//UI BSS List signal strength(in percentage), make it good looking, from 0~100.
	//It is assigned to the BSS List in GetValueFromBeaconOrProbeRsp().
	if (isCCKrate) {
		pPhyInfo->SignalStrength = (u8)(odm_SignalScaleMapping(pDM_Odm, PWDB_ALL));//PWDB_ALL;
	} else {
		if (rf_rx_num != 0)
			pPhyInfo->SignalStrength = (u8)(odm_SignalScaleMapping(pDM_Odm, total_rssi/=rf_rx_num));
	}
	pDM_Odm->RxPWDBAve = pDM_Odm->RxPWDBAve + pPhyInfo->RxPWDBAll;

	pDM_Odm->DM_FatTable.antsel_rx_keep_0 = pPhyStaRpt->antidx_anta;
}

void odm_Init_RSSIForDM(PDM_ODM_T pDM_Odm)
{
}

void odm_Process_RSSIForDM(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, PODM_PACKET_INFO_T pPktinfo)
{

	s4Byte			UndecoratedSmoothedPWDB, UndecoratedSmoothedCCK, UndecoratedSmoothedOFDM, RSSI_Ave;
	u8			isCCKrate=0;
	u8			RSSI_max, RSSI_min, i;
	u32			OFDM_pkt=0;
	u32			Weighting=0;

	PSTA_INFO_T	pEntry;

	if (pPktinfo->StationID == 0xFF)
		return;

	// 2012/05/30 MH/Luke.Lee Add some description
	// In windows driver: AP/IBSS mode STA
		pEntry = pDM_Odm->pODM_StaInfo[pPktinfo->StationID];

	if (!IS_STA_VALID(pEntry))
		return;
	if ((!pPktinfo->bPacketMatchBSSID) )
		return;

	if (pPktinfo->bPacketBeacon)
		pDM_Odm->PhyDbgInfo.NumQryBeaconPkt++;
	isCCKrate = (pPktinfo->DataRate <= DESC92C_RATE11M)?true :false;
	pDM_Odm->RxRate = pPktinfo->DataRate;
#if (defined(CONFIG_HW_ANTENNA_DIVERSITY))
	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		pFAT_T	pDM_FatTable = &pDM_Odm->DM_FatTable;
		if (pPktinfo->bPacketToSelf || pPktinfo->bPacketMatchBSSID) {
			if (pPktinfo->DataRate > DESC8812_RATE11M)
				ODM_AntselStatistics_8821A(pDM_Odm, pDM_FatTable->antsel_rx_keep_0, pPktinfo->StationID, pPhyInfo->RxPWDBAll);
		}
	}

#endif //#if (defined(CONFIG_HW_ANTENNA_DIVERSITY))

	//-----------------Smart Antenna Debug Message------------------//

	UndecoratedSmoothedCCK =  pEntry->rssi_stat.UndecoratedSmoothedCCK;
	UndecoratedSmoothedOFDM = pEntry->rssi_stat.UndecoratedSmoothedOFDM;
	UndecoratedSmoothedPWDB = pEntry->rssi_stat.UndecoratedSmoothedPWDB;

	if (pPktinfo->bPacketToSelf || pPktinfo->bPacketBeacon) {

		if (!isCCKrate) {//ofdm rate
			if (pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_B] == 0){
				RSSI_Ave = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A];
				pDM_Odm->RSSI_A = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A];
				pDM_Odm->RSSI_B = 0;
			} else {
				pDM_Odm->RSSI_A =  pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A];
				pDM_Odm->RSSI_B = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_B];

				if (pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A] > pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_B]) {
					RSSI_max = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A];
					RSSI_min = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_B];
				} else {
					RSSI_max = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_B];
					RSSI_min = pPhyInfo->RxMIMOSignalStrength[ODM_RF_PATH_A];
				}
				if ((RSSI_max -RSSI_min) < 3)
					RSSI_Ave = RSSI_max;
				else if ((RSSI_max -RSSI_min) < 6)
					RSSI_Ave = RSSI_max - 1;
				else if ((RSSI_max -RSSI_min) < 10)
					RSSI_Ave = RSSI_max - 2;
				else
					RSSI_Ave = RSSI_max - 3;
			}

			//1 Process OFDM RSSI
			if (UndecoratedSmoothedOFDM <= 0) {	// initialize
				UndecoratedSmoothedOFDM = pPhyInfo->RxPWDBAll;
			} else {
				if (pPhyInfo->RxPWDBAll > (u32)UndecoratedSmoothedOFDM) {
					UndecoratedSmoothedOFDM =
							( ((UndecoratedSmoothedOFDM)*(Rx_Smooth_Factor-1)) +
							(RSSI_Ave)) /(Rx_Smooth_Factor);
					UndecoratedSmoothedOFDM = UndecoratedSmoothedOFDM + 1;
				} else {
					UndecoratedSmoothedOFDM =
							( ((UndecoratedSmoothedOFDM)*(Rx_Smooth_Factor-1)) +
							(RSSI_Ave)) /(Rx_Smooth_Factor);
				}
			}
			pEntry->rssi_stat.PacketMap = (pEntry->rssi_stat.PacketMap<<1) | BIT0;
		} else {
			RSSI_Ave = pPhyInfo->RxPWDBAll;
			pDM_Odm->RSSI_A = (u8) pPhyInfo->RxPWDBAll;
			pDM_Odm->RSSI_B = 0xFF;

			//1 Process CCK RSSI
			if (UndecoratedSmoothedCCK <= 0) {	// initialize
				UndecoratedSmoothedCCK = pPhyInfo->RxPWDBAll;
			} else {
				if (pPhyInfo->RxPWDBAll > (u32)UndecoratedSmoothedCCK) {
					UndecoratedSmoothedCCK =
							(((UndecoratedSmoothedCCK)*(Rx_Smooth_Factor-1)) +
							(pPhyInfo->RxPWDBAll)) /(Rx_Smooth_Factor);
					UndecoratedSmoothedCCK = UndecoratedSmoothedCCK + 1;
				} else {
					UndecoratedSmoothedCCK =
							( ((UndecoratedSmoothedCCK)*(Rx_Smooth_Factor-1)) +
							(pPhyInfo->RxPWDBAll)) /(Rx_Smooth_Factor);
				}
			}
			pEntry->rssi_stat.PacketMap = pEntry->rssi_stat.PacketMap<<1;
		}

		//2011.07.28 LukeLee: modified to prevent unstable CCK RSSI
		if (pEntry->rssi_stat.ValidBit >= 64)
			pEntry->rssi_stat.ValidBit = 64;
		else
			pEntry->rssi_stat.ValidBit++;

		for(i=0; i<pEntry->rssi_stat.ValidBit; i++)
			OFDM_pkt += (u8)(pEntry->rssi_stat.PacketMap>>i)&BIT0;

		if (pEntry->rssi_stat.ValidBit == 64) {
			Weighting = ((OFDM_pkt<<4) > 64)?64:(OFDM_pkt<<4);
			UndecoratedSmoothedPWDB = (Weighting*UndecoratedSmoothedOFDM+(64-Weighting)*UndecoratedSmoothedCCK)>>6;
		} else {
			if (pEntry->rssi_stat.ValidBit != 0)
				UndecoratedSmoothedPWDB = (OFDM_pkt*UndecoratedSmoothedOFDM+(pEntry->rssi_stat.ValidBit-OFDM_pkt)*UndecoratedSmoothedCCK)/pEntry->rssi_stat.ValidBit;
			else
				UndecoratedSmoothedPWDB = 0;
		}
		pEntry->rssi_stat.UndecoratedSmoothedCCK = UndecoratedSmoothedCCK;
		pEntry->rssi_stat.UndecoratedSmoothedOFDM = UndecoratedSmoothedOFDM;
		pEntry->rssi_stat.UndecoratedSmoothedPWDB = UndecoratedSmoothedPWDB;
	}
}

void ODM_PhyStatusQuery_92CSeries(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, u8 * pPhyStatus, PODM_PACKET_INFO_T pPktinfo)
{

	odm_RxPhyStatus92CSeries_Parsing(pDM_Odm, pPhyInfo, pPhyStatus, pPktinfo);

	if (pDM_Odm->RSSI_test) {
		// Select the packets to do RSSI checking for antenna switching.
		if (pPktinfo->bPacketToSelf || pPktinfo->bPacketBeacon )
			ODM_SwAntDivChkPerPktRssi(pDM_Odm,pPktinfo->StationID,pPhyInfo);
	} else {
		odm_Process_RSSIForDM(pDM_Odm,pPhyInfo,pPktinfo);
	}
}

void ODM_PhyStatusQuery_JaguarSeries(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, u8 * pPhyStatus, PODM_PACKET_INFO_T pPktinfo)
{
	odm_RxPhyStatusJaguarSeries_Parsing(pDM_Odm, pPhyInfo, pPhyStatus, pPktinfo);

	odm_Process_RSSIForDM(pDM_Odm,pPhyInfo,pPktinfo);
}

void ODM_PhyStatusQuery(PDM_ODM_T pDM_Odm, PODM_PHY_INFO_T pPhyInfo, u8 * pPhyStatus, PODM_PACKET_INFO_T pPktinfo)
{

	if (pDM_Odm->SupportICType & ODM_IC_11AC_SERIES )
		ODM_PhyStatusQuery_JaguarSeries(pDM_Odm,pPhyInfo,pPhyStatus,pPktinfo);
	else
		ODM_PhyStatusQuery_92CSeries(pDM_Odm,pPhyInfo,pPhyStatus,pPktinfo);
}

// For future use.
void ODM_MacStatusQuery(PDM_ODM_T pDM_Odm, u8 * pMacStatus, u8 MacID, bool bPacketMatchBSSID, bool bPacketToSelf, bool bPacketBeacon)
{
	// 2011/10/19 Driver team will handle in the future.
}

//
// If you want to add a new IC, Please follow below template and generate a new one.
//
//

HAL_STATUS ODM_ConfigRFWithHeaderFile(PDM_ODM_T pDM_Odm, ODM_RF_Config_Type ConfigType, ODM_RF_RADIO_PATH_E eRFPath)
{
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("===>ODM_ConfigRFWithHeaderFile (%s)\n", (pDM_Odm->bIsMPChip) ? "MPChip" : "TestChip"));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("pDM_Odm->SupportPlatform: 0x%X, pDM_Odm->SupportInterface: 0x%X, pDM_Odm->BoardType: 0x%X\n",
		      pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface, pDM_Odm->BoardType));

	if (pDM_Odm->SupportICType == ODM_RTL8812) {
		if (ConfigType == CONFIG_RF_RADIO) {
			if (eRFPath == ODM_RF_PATH_A)
				READ_AND_CONFIG(8812A,_RadioA);
			else if (eRFPath == ODM_RF_PATH_B)
				READ_AND_CONFIG(8812A,_RadioB);
		} else if (ConfigType == CONFIG_RF_TXPWR_LMT) {
			  READ_AND_CONFIG(8812A,_TXPWR_LMT);
		}
	}

	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		if (ConfigType == CONFIG_RF_RADIO) {
			if (eRFPath == ODM_RF_PATH_A)
				READ_AND_CONFIG(8821A,_RadioA);
		} else if (ConfigType == CONFIG_RF_TXPWR_LMT) {
			READ_AND_CONFIG(8821A,_TXPWR_LMT);
		}
		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("<===8821_ODM_ConfigRFWithHeaderFile\n"));
	}
	return HAL_STATUS_SUCCESS;
}

HAL_STATUS ODM_ConfigRFWithTxPwrTrackHeaderFile(PDM_ODM_T pDM_Odm)
{
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("===>ODM_ConfigRFWithTxPwrTrackHeaderFile (%s)\n", (pDM_Odm->bIsMPChip) ? "MPChip" : "TestChip"));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("pDM_Odm->SupportPlatform: 0x%X, pDM_Odm->SupportInterface: 0x%X, pDM_Odm->BoardType: 0x%X\n",
		      pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface, pDM_Odm->BoardType));
	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		if (pDM_Odm->SupportInterface == ODM_ITRF_PCIE)
			READ_AND_CONFIG(8821A,_TxPowerTrack_PCIE);
		else if (pDM_Odm->SupportInterface == ODM_ITRF_USB)
			READ_AND_CONFIG(8821A,_TxPowerTrack_USB);
	}
	if (pDM_Odm->SupportICType == ODM_RTL8812) {
		if (pDM_Odm->SupportInterface == ODM_ITRF_PCIE)
			READ_AND_CONFIG(8812A,_TxPowerTrack_PCIE);
		else if (pDM_Odm->SupportInterface == ODM_ITRF_USB) {
			if (pDM_Odm->RFEType == 3 && pDM_Odm->bIsMPChip)
				READ_AND_CONFIG_MP(8812A,_TxPowerTrack_USB_RFE3);
			else
				READ_AND_CONFIG(8812A,_TxPowerTrack_USB);
		}

	}
	return HAL_STATUS_SUCCESS;
}

HAL_STATUS ODM_ConfigBBWithHeaderFile(PDM_ODM_T pDM_Odm, ODM_BB_Config_Type ConfigType)
{

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("===>ODM_ConfigBBWithHeaderFile (%s)\n", (pDM_Odm->bIsMPChip) ? "MPChip" : "TestChip"));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
		     ("pDM_Odm->SupportPlatform: 0x%X, pDM_Odm->SupportInterface: 0x%X, pDM_Odm->BoardType: 0x%X\n",
		      pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface, pDM_Odm->BoardType));
	if (pDM_Odm->SupportICType == ODM_RTL8812) {
		if (ConfigType == CONFIG_BB_PHY_REG) {
			READ_AND_CONFIG(8812A,_PHY_REG);
		} else if (ConfigType == CONFIG_BB_AGC_TAB) {
			READ_AND_CONFIG(8812A,_AGC_TAB);
		} else if (ConfigType == CONFIG_BB_PHY_REG_PG) {
			if (pDM_Odm->RFEType == 3 && pDM_Odm->bIsMPChip)
				READ_AND_CONFIG_MP(8812A,_PHY_REG_PG_ASUS);
			else
				READ_AND_CONFIG(8812A,_PHY_REG_PG);
		} else if (ConfigType == CONFIG_BB_PHY_REG_MP) {
			READ_AND_CONFIG_MP(8812A,_PHY_REG_MP);
		}
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_LOUD, (" ===> phy_ConfigBBWithHeaderFile() phy:Rtl8812AGCTABArray\n"));
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_LOUD, (" ===> phy_ConfigBBWithHeaderFile() agc:Rtl8812PHY_REGArray\n"));
	}

	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		if (ConfigType == CONFIG_BB_PHY_REG) {
			READ_AND_CONFIG(8821A,_PHY_REG);
		} else if (ConfigType == CONFIG_BB_AGC_TAB) {
			READ_AND_CONFIG(8821A,_AGC_TAB);
		} else if (ConfigType == CONFIG_BB_PHY_REG_PG) {
			READ_AND_CONFIG(8821A,_PHY_REG_PG);
		}
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_LOUD, (" ===> phy_ConfigBBWithHeaderFile() phy:Rtl8821AGCTABArray\n"));
		ODM_RT_TRACE(pDM_Odm,ODM_COMP_INIT, ODM_DBG_LOUD, (" ===> phy_ConfigBBWithHeaderFile() agc:Rtl8821PHY_REGArray\n"));
	}
	return HAL_STATUS_SUCCESS;
}

HAL_STATUS ODM_ConfigMACWithHeaderFile(PDM_ODM_T pDM_Odm)
{
	u8 result = HAL_STATUS_SUCCESS;

	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
				("===>ODM_ConfigMACWithHeaderFile (%s)\n", (pDM_Odm->bIsMPChip) ? "MPChip" : "TestChip"));
	ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD,
				("pDM_Odm->SupportPlatform: 0x%X, pDM_Odm->SupportInterface: 0x%X, pDM_Odm->BoardType: 0x%X\n",
				pDM_Odm->SupportPlatform, pDM_Odm->SupportInterface, pDM_Odm->BoardType));

	if (pDM_Odm->SupportICType == ODM_RTL8812) {
		READ_AND_CONFIG(8812A,_MAC_REG);
	}
	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		READ_AND_CONFIG(8821A,_MAC_REG);

		ODM_RT_TRACE(pDM_Odm, ODM_COMP_INIT, ODM_DBG_LOUD, ("<===8821_ODM_ConfigMACwithHeaderFile\n"));
	}
	return result;
}

HAL_STATUS ODM_ConfigFWWithHeaderFile(PDM_ODM_T pDM_Odm, ODM_FW_Config_Type ConfigType, u8 *pFirmware, u32 *pSize)
{
	if (pDM_Odm->SupportICType == ODM_RTL8812) {
		if (ConfigType == CONFIG_FW_NIC) {
			READ_FIRMWARE(8812A,_FW_NIC);
		} else if (ConfigType == CONFIG_FW_WoWLAN) {
			READ_FIRMWARE(8812A,_FW_WoWLAN);
		} else if (ConfigType == CONFIG_FW_BT) {
			READ_FIRMWARE(8812A,_FW_NIC_BT);
		}
	}
	if (pDM_Odm->SupportICType == ODM_RTL8821) {
		if (ConfigType == CONFIG_FW_NIC) {
			READ_FIRMWARE(8821A,_FW_NIC);
		} else if (ConfigType == CONFIG_FW_WoWLAN) {
			READ_FIRMWARE(8821A,_FW_WoWLAN);
		} else if (ConfigType == CONFIG_FW_BT) {
			READ_FIRMWARE(8821A,_FW_BT);
		}
	}
	return HAL_STATUS_SUCCESS;
}
