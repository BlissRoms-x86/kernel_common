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
#define _RTW_MP_C_

#include <drv_types.h>

#include "../hal/odm_precomp.h"

#ifdef CONFIG_MP_INCLUDED

u32 read_macreg(_adapter *padapter, u32 addr, u32 sz)
{
	u32 val = 0;

	switch(sz)
	{
		case 1:
			val = rtw_read8(padapter, addr);
			break;
		case 2:
			val = rtw_read16(padapter, addr);
			break;
		case 4:
			val = rtw_read32(padapter, addr);
			break;
		default:
			val = 0xffffffff;
			break;
	}

	return val;

}

void write_macreg(_adapter *padapter, u32 addr, u32 val, u32 sz)
{
	switch(sz)
	{
		case 1:
			rtw_write8(padapter, addr, (u8)val);
			break;
		case 2:
			rtw_write16(padapter, addr, (u16)val);
			break;
		case 4:
			rtw_write32(padapter, addr, val);
			break;
		default:
			break;
	}

}

u32 read_bbreg(_adapter *padapter, u32 addr, u32 bitmask)
{
	return rtw_hal_read_bbreg(padapter, addr, bitmask);
}

void write_bbreg(_adapter *padapter, u32 addr, u32 bitmask, u32 val)
{
	rtw_hal_write_bbreg(padapter, addr, bitmask, val);
}

u32 _read_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 bitmask)
{
	return rtw_hal_read_rfreg(padapter, rfpath, addr, bitmask);
}

void _write_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 bitmask, u32 val)
{
	rtw_hal_write_rfreg(padapter, rfpath, addr, bitmask, val);
}

u32 read_rfreg(PADAPTER padapter, u8 rfpath, u32 addr)
{
	return _read_rfreg(padapter, rfpath, addr, bRFRegOffsetMask);
}

void write_rfreg(PADAPTER padapter, u8 rfpath, u32 addr, u32 val)
{
	_write_rfreg(padapter, rfpath, addr, bRFRegOffsetMask, val);
}

static void _init_mp_priv_(struct mp_priv *pmp_priv)
{
	WLAN_BSSID_EX *pnetwork;

	_rtw_memset(pmp_priv, 0, sizeof(struct mp_priv));

	pmp_priv->mode = MP_OFF;

	pmp_priv->channel = 1;
	pmp_priv->bandwidth = CHANNEL_WIDTH_20;
	pmp_priv->prime_channel_offset = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
	pmp_priv->rateidx = MPT_RATE_1M;
	pmp_priv->txpoweridx = 0x2A;

	pmp_priv->antenna_tx = ANTENNA_A;
	pmp_priv->antenna_rx = ANTENNA_AB;

	pmp_priv->check_mp_pkt = 0;

	pmp_priv->tx_pktcount = 0;

	pmp_priv->rx_pktcount = 0;
	pmp_priv->rx_crcerrpktcount = 0;

	pmp_priv->network_macaddr[0] = 0x00;
	pmp_priv->network_macaddr[1] = 0xE0;
	pmp_priv->network_macaddr[2] = 0x4C;
	pmp_priv->network_macaddr[3] = 0x87;
	pmp_priv->network_macaddr[4] = 0x66;
	pmp_priv->network_macaddr[5] = 0x55;

	pnetwork = &pmp_priv->mp_network.network;
	_rtw_memcpy(pnetwork->MacAddress, pmp_priv->network_macaddr, ETH_ALEN);

	pnetwork->Ssid.SsidLength = 8;
	_rtw_memcpy(pnetwork->Ssid.Ssid, "mp_871x", pnetwork->Ssid.SsidLength);
}

static int init_mp_priv_by_os(struct mp_priv *pmp_priv)
{
	int i, res;
	struct mp_xmit_frame *pmp_xmitframe;

	if (pmp_priv == NULL) return _FAIL;

	_rtw_init_queue(&pmp_priv->free_mp_xmitqueue);

	pmp_priv->pallocated_mp_xmitframe_buf = NULL;
	pmp_priv->pallocated_mp_xmitframe_buf = rtw_zmalloc(NR_MP_XMITFRAME * sizeof(struct mp_xmit_frame) + 4);
	if (pmp_priv->pallocated_mp_xmitframe_buf == NULL) {
		res = _FAIL;
		goto _exit_init_mp_priv;
	}

	pmp_priv->pmp_xmtframe_buf = pmp_priv->pallocated_mp_xmitframe_buf + 4 - ((SIZE_PTR) (pmp_priv->pallocated_mp_xmitframe_buf) & 3);

	pmp_xmitframe = (struct mp_xmit_frame*)pmp_priv->pmp_xmtframe_buf;

	for (i = 0; i < NR_MP_XMITFRAME; i++)
	{
		_rtw_init_listhead(&pmp_xmitframe->list);
		rtw_list_insert_tail(&pmp_xmitframe->list, &pmp_priv->free_mp_xmitqueue.queue);

		pmp_xmitframe->pkt = NULL;
		pmp_xmitframe->frame_tag = MP_FRAMETAG;
		pmp_xmitframe->padapter = pmp_priv->papdater;

		pmp_xmitframe++;
	}

	pmp_priv->free_mp_xmitframe_cnt = NR_MP_XMITFRAME;

	res = _SUCCESS;

_exit_init_mp_priv:

	return res;
}

static void mp_init_xmit_attrib(struct mp_tx *pmptx, PADAPTER padapter)
{
	struct pkt_attrib *pattrib;
	struct tx_desc *desc;

	// init xmitframe attribute
	pattrib = &pmptx->attrib;
	_rtw_memset(pattrib, 0, sizeof(struct pkt_attrib));
	desc = &pmptx->desc;
	_rtw_memset(desc, 0, TXDESC_SIZE);

	pattrib->ether_type = 0x8712;
	//_rtw_memcpy(pattrib->src, padapter->eeprompriv.mac_addr, ETH_ALEN);
//	_rtw_memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
	_rtw_memset(pattrib->dst, 0xFF, ETH_ALEN);

//	pattrib->dhcp_pkt = 0;
//	pattrib->pktlen = 0;
	pattrib->ack_policy = 0;
//	pattrib->pkt_hdrlen = ETH_HLEN;
	pattrib->hdrlen = WLAN_HDR_A3_LEN;
	pattrib->subtype = WIFI_DATA;
	pattrib->priority = 0;
	pattrib->qsel = pattrib->priority;
//	do_queue_select(padapter, pattrib);
	pattrib->nr_frags = 1;
	pattrib->encrypt = 0;
	pattrib->bswenc = false;
	pattrib->qos_en = false;
}

s32 init_mp_priv(PADAPTER padapter)
{
	struct mp_priv *pmppriv = &padapter->mppriv;

	_init_mp_priv_(pmppriv);
	pmppriv->papdater = padapter;

	pmppriv->tx.stop = 1;
	mp_init_xmit_attrib(&pmppriv->tx, padapter);

	switch (padapter->registrypriv.rf_config) {
		case RF_1T1R:
			pmppriv->antenna_tx = ANTENNA_A;
			pmppriv->antenna_rx = ANTENNA_A;
			break;
		case RF_1T2R:
		default:
			pmppriv->antenna_tx = ANTENNA_A;
			pmppriv->antenna_rx = ANTENNA_AB;
			break;
		case RF_2T2R:
		case RF_2T2R_GREEN:
			pmppriv->antenna_tx = ANTENNA_AB;
			pmppriv->antenna_rx = ANTENNA_AB;
			break;
		case RF_2T4R:
			pmppriv->antenna_tx = ANTENNA_AB;
			pmppriv->antenna_rx = ANTENNA_ABCD;
			break;
	}

	return _SUCCESS;
}

void free_mp_priv(struct mp_priv *pmp_priv)
{
	if (pmp_priv->pallocated_mp_xmitframe_buf) {
		rtw_mfree(pmp_priv->pallocated_mp_xmitframe_buf, 0);
		pmp_priv->pallocated_mp_xmitframe_buf = NULL;
	}
	pmp_priv->pmp_xmtframe_buf = NULL;
}


static void PHY_IQCalibrate_default(
	PADAPTER	pAdapter,
	bool		bReCovery
	)
{
	DBG_871X("%s\n", __func__);
}

static void PHY_LCCalibrate_default(
	PADAPTER	pAdapter
	)
{
	DBG_871X("%s\n", __func__);
}

static void PHY_SetRFPathSwitch_default(
	PADAPTER	pAdapter,
	bool		bMain
	)
{
	DBG_871X("%s\n", __func__);
}

#define PHY_IQCalibrate(_Adapter, b)	\
		IS_HARDWARE_TYPE_8812(_Adapter) ? PHY_IQCalibrate_8812A(_Adapter, b) : \
		IS_HARDWARE_TYPE_8821(_Adapter) ? PHY_IQCalibrate_8821A(_Adapter, b) : \
		PHY_IQCalibrate_default(_Adapter, b)

#define PHY_LCCalibrate(_Adapter)	\
		IS_HARDWARE_TYPE_8812(_Adapter) ? PHY_LCCalibrate_8812A(&(GET_HAL_DATA(_Adapter)->odmpriv)) : \
		IS_HARDWARE_TYPE_8821(_Adapter) ? PHY_LCCalibrate_8821A(&(GET_HAL_DATA(_Adapter)->odmpriv)) : \
		PHY_LCCalibrate_default(_Adapter)

#define PHY_SetRFPathSwitch(_Adapter, b)	\
		(IS_HARDWARE_TYPE_JAGUAR(_Adapter)) ? PHY_SetRFPathSwitch_8812A(_Adapter, b) : \
		PHY_SetRFPathSwitch_default(_Adapter, b)

s32
MPT_InitializeAdapter(
	PADAPTER			pAdapter,
	u8				Channel
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	s32		rtStatus = _SUCCESS;
	PMPT_CONTEXT	pMptCtx = &pAdapter->mppriv.MptCtx;
	u32		ledsetting;
	struct mlme_priv *pmlmepriv = &pAdapter->mlmepriv;

	//-------------------------------------------------------------------------
	// HW Initialization for 8190 MPT.
	//-------------------------------------------------------------------------
	//-------------------------------------------------------------------------
	// SW Initialization for 8190 MP.
	//-------------------------------------------------------------------------
	pMptCtx->bMptDrvUnload = false;
	pMptCtx->bMassProdTest = false;
	pMptCtx->bMptIndexEven = true;	//default gain index is -6.0db
	pMptCtx->h2cReqNum = 0x0;
	/* Init mpt event. */
#if 0 // for Windows
	NdisInitializeEvent( &(pMptCtx->MptWorkItemEvent) );
	NdisAllocateSpinLock( &(pMptCtx->MptWorkItemSpinLock) );

	PlatformInitializeWorkItem(
		Adapter,
		&(pMptCtx->MptWorkItem),
		(RT_WORKITEM_CALL_BACK)MPT_WorkItemCallback,
		(void *)Adapter,
		"MptWorkItem");
#endif
	//init for BT MP

	pMptCtx->bMptWorkItemInProgress = false;
	pMptCtx->CurrMptAct = NULL;
	//-------------------------------------------------------------------------

	// Don't accept any packets
	rtw_write32(pAdapter, REG_RCR, 0);

	ledsetting = rtw_read32(pAdapter, REG_LEDCFG0);

	PHY_IQCalibrate(pAdapter, false);
	dm_CheckTXPowerTracking(&pHalData->odmpriv);	//trigger thermal meter
	PHY_LCCalibrate(pAdapter);

	pMptCtx->backup0xc50 = (u8)PHY_QueryBBReg(pAdapter, rOFDM0_XAAGCCore1, bMaskByte0);
	pMptCtx->backup0xc58 = (u8)PHY_QueryBBReg(pAdapter, rOFDM0_XBAGCCore1, bMaskByte0);
	pMptCtx->backup0xc30 = (u8)PHY_QueryBBReg(pAdapter, rOFDM0_RxDetector1, bMaskByte0);

	//set ant to wifi side in mp mode
	rtw_write16(pAdapter, 0x870, 0x300);
	rtw_write16(pAdapter, 0x860, 0x110);

	if (pAdapter->registrypriv.mp_mode == 1)
		pmlmepriv->fw_state = WIFI_MP_STATE;

	return	rtStatus;
}

/*-----------------------------------------------------------------------------
 * Function:	MPT_DeInitAdapter()
 *
 * Overview:	Extra DeInitialization for Mass Production Test.
 *
 * Input:		PADAPTER	pAdapter
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	05/08/2007	MHC		Create Version 0.
 *	05/18/2007	MHC		Add normal driver MPHalt code.
 *
 *---------------------------------------------------------------------------*/
void
MPT_DeInitAdapter(
	PADAPTER	pAdapter
	)
{
	PMPT_CONTEXT		pMptCtx = &pAdapter->mppriv.MptCtx;

	pMptCtx->bMptDrvUnload = true;
}

static u8 mpt_ProStartTest(PADAPTER padapter)
{
	PMPT_CONTEXT pMptCtx = &padapter->mppriv.MptCtx;

	pMptCtx->bMassProdTest = true;
	pMptCtx->bStartContTx = false;
	pMptCtx->bCckContTx = false;
	pMptCtx->bOfdmContTx = false;
	pMptCtx->bSingleCarrier = false;
	pMptCtx->bCarrierSuppression = false;
	pMptCtx->bSingleTone = false;

	return _SUCCESS;
}

/*
 * General use
 */
s32 SetPowerTracking(PADAPTER padapter, u8 enable)
{

	Hal_SetPowerTracking( padapter, enable );
	return 0;
}

void GetPowerTracking(PADAPTER padapter, u8 *enable)
{
	Hal_GetPowerTracking( padapter, enable );
}

static void disable_dm(PADAPTER padapter)
{
	u8 v8;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;


	//3 1. disable firmware dynamic mechanism
	// disable Power Training, Rate Adaptive
	v8 = rtw_read8(padapter, REG_BCN_CTRL);
	v8 &= ~EN_BCN_FUNCTION;
	rtw_write8(padapter, REG_BCN_CTRL, v8);

	//3 2. disable driver dynamic mechanism
	// disable Dynamic Initial Gain
	// disable High Power
	// disable Power Tracking
	Switch_DM_Func(padapter, DYNAMIC_FUNC_DISABLE, false);

	// enable APK, LCK and IQK but disable power tracking
	Switch_DM_Func(padapter, DYNAMIC_RF_CALIBRATION, true);
}

//This function initializes the DUT to the MP test mode
s32 mp_start_test(PADAPTER padapter)
{
	WLAN_BSSID_EX bssid;
	struct sta_info *psta;
	u32 length;
	u8 val8;

	_irqL irqL;
	s32 res = _SUCCESS;

	struct mp_priv *pmppriv = &padapter->mppriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;

	padapter->registrypriv.mp_mode = 1;
	pmppriv->bSetTxPower=0;		//for  manually set tx power

	//3 disable dynamic mechanism
	disable_dm(padapter);
	rtl8812_InitHalDm(padapter);
	//3 0. update mp_priv

	if (padapter->registrypriv.rf_config == RF_MAX_TYPE) {
//		switch (phal->rf_type) {
		switch (GET_RF_TYPE(padapter)) {
			case RF_1T1R:
				pmppriv->antenna_tx = ANTENNA_A;
				pmppriv->antenna_rx = ANTENNA_A;
				break;
			case RF_1T2R:
			default:
				pmppriv->antenna_tx = ANTENNA_A;
				pmppriv->antenna_rx = ANTENNA_AB;
				break;
			case RF_2T2R:
			case RF_2T2R_GREEN:
				pmppriv->antenna_tx = ANTENNA_AB;
				pmppriv->antenna_rx = ANTENNA_AB;
				break;
			case RF_2T4R:
				pmppriv->antenna_tx = ANTENNA_AB;
				pmppriv->antenna_rx = ANTENNA_ABCD;
				break;
		}
	}

	mpt_ProStartTest(padapter);

	//3 1. initialize a new WLAN_BSSID_EX
//	_rtw_memset(&bssid, 0, sizeof(WLAN_BSSID_EX));
	_rtw_memcpy(bssid.MacAddress, pmppriv->network_macaddr, ETH_ALEN);
	bssid.Ssid.SsidLength = strlen("mp_pseudo_adhoc");
	_rtw_memcpy(bssid.Ssid.Ssid, (u8*)"mp_pseudo_adhoc", bssid.Ssid.SsidLength);
	bssid.InfrastructureMode = Ndis802_11IBSS;
	bssid.NetworkTypeInUse = Ndis802_11DS;
	bssid.IELength = 0;

	length = get_WLAN_BSSID_EX_sz(&bssid);
	if (length % 4)
		bssid.Length = ((length >> 2) + 1) << 2; //round up to multiple of 4 bytes.
	else
		bssid.Length = length;

	_enter_critical_bh(&pmlmepriv->lock, &irqL);

	if (check_fwstate(pmlmepriv, WIFI_MP_STATE) == true)
		goto end_of_mp_start_test;

	//init mp_start_test status
	if (check_fwstate(pmlmepriv, _FW_LINKED) == true) {
		rtw_disassoc_cmd(padapter, 500, true);
		rtw_indicate_disconnect(padapter);
		rtw_free_assoc_resources(padapter, 1);
	}
	pmppriv->prev_fw_state = get_fwstate(pmlmepriv);
	if (padapter->registrypriv.mp_mode == 1)
		pmlmepriv->fw_state = WIFI_MP_STATE;
#if 0
	if (pmppriv->mode == _LOOPBOOK_MODE_) {
		set_fwstate(pmlmepriv, WIFI_MP_LPBK_STATE); //append txdesc
		RT_TRACE(_module_mp_, _drv_notice_, ("+start mp in Lookback mode\n"));
	} else {
		RT_TRACE(_module_mp_, _drv_notice_, ("+start mp in normal mode\n"));
	}
#endif
	set_fwstate(pmlmepriv, _FW_UNDER_LINKING);

	//3 2. create a new psta for mp driver
	//clear psta in the cur_network, if any
	psta = rtw_get_stainfo(&padapter->stapriv, tgt_network->network.MacAddress);
	if (psta) rtw_free_stainfo(padapter, psta);

	psta = rtw_alloc_stainfo(&padapter->stapriv, bssid.MacAddress);
	if (psta == NULL) {
		RT_TRACE(_module_mp_, _drv_err_, ("mp_start_test: Can't alloc sta_info!\n"));
		pmlmepriv->fw_state = pmppriv->prev_fw_state;
		res = _FAIL;
		goto end_of_mp_start_test;
	}

	//3 3. join psudo AdHoc
	tgt_network->join_res = 1;
	tgt_network->aid = psta->aid = 1;
	_rtw_memcpy(&tgt_network->network, &bssid, length);

	rtw_indicate_connect(padapter);
	_clr_fwstate_(pmlmepriv, _FW_UNDER_LINKING);

end_of_mp_start_test:

	_exit_critical_bh(&pmlmepriv->lock, &irqL);

	if (res == _SUCCESS) {
		// set MSR to WIFI_FW_ADHOC_STATE
		val8 = rtw_read8(padapter, MSR) & 0xFC; // 0x0102
		val8 |= WIFI_FW_ADHOC_STATE;
		rtw_write8(padapter, MSR, val8); // Link in ad hoc network
	}

	return res;
}
//------------------------------------------------------------------------------
//This function change the DUT from the MP test mode into normal mode
void mp_stop_test(PADAPTER padapter)
{
	struct mp_priv *pmppriv = &padapter->mppriv;
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct wlan_network *tgt_network = &pmlmepriv->cur_network;
	struct sta_info *psta;

	_irqL irqL;

	if(pmppriv->mode==MP_ON)
	{
	pmppriv->bSetTxPower=0;
	_enter_critical_bh(&pmlmepriv->lock, &irqL);
	if (check_fwstate(pmlmepriv, WIFI_MP_STATE) == false)
		goto end_of_mp_stop_test;

	//3 1. disconnect psudo AdHoc
	rtw_indicate_disconnect(padapter);

	//3 2. clear psta used in mp test mode.
//	rtw_free_assoc_resources(padapter, 1);
	psta = rtw_get_stainfo(&padapter->stapriv, tgt_network->network.MacAddress);
	if (psta) rtw_free_stainfo(padapter, psta);

	//3 3. return to normal state (default:station mode)
	pmlmepriv->fw_state = pmppriv->prev_fw_state; // WIFI_STATION_STATE;

	//flush the cur_network
	_rtw_memset(tgt_network, 0, sizeof(struct wlan_network));

	_clr_fwstate_(pmlmepriv, WIFI_MP_STATE);

end_of_mp_stop_test:

	_exit_critical_bh(&pmlmepriv->lock, &irqL);
	}
}

/*-----------------------------------------------------------------------------
 * Function:	mpt_SwitchRfSetting
 *
 * Overview:	Change RF Setting when we siwthc channel/rate/BW for MP.
 *
 * Input:       PADAPTER				pAdapter
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 * When			Who		Remark
 * 01/08/2009	MHC		Suggestion from SD3 Willis for 92S series.
 * 01/09/2009	MHC		Add CCK modification for 40MHZ. Suggestion from SD3.
 *
 *---------------------------------------------------------------------------*/
static void mpt_SwitchRfSetting(PADAPTER pAdapter)
{
	Hal_mpt_SwitchRfSetting(pAdapter);
    }

/*---------------------------hal\rtl8192c\MPT_Phy.c---------------------------*/
/*---------------------------hal\rtl8192c\MPT_HelperFunc.c---------------------------*/
static void MPT_CCKTxPowerAdjust(PADAPTER Adapter, bool bInCH14)
{
	Hal_MPT_CCKTxPowerAdjust(Adapter,bInCH14);
}

static void MPT_CCKTxPowerAdjustbyIndex(PADAPTER pAdapter, bool beven)
{
	Hal_MPT_CCKTxPowerAdjustbyIndex(pAdapter,beven);
	}

/*---------------------------hal\rtl8192c\MPT_HelperFunc.c---------------------------*/

/*
 * SetChannel
 * Description
 *	Use H2C command to change channel,
 *	not only modify rf register, but also other setting need to be done.
 */
void SetChannel(PADAPTER pAdapter)
{
	Hal_SetChannel(pAdapter);
}

/*
 * Notice
 *	Switch bandwitdth may change center frequency(channel)
 */
void SetBandwidth(PADAPTER pAdapter)
{
	Hal_SetBandwidth(pAdapter);

}

static void SetCCKTxPower(PADAPTER pAdapter, u8 *TxPower)
{
	Hal_SetCCKTxPower(pAdapter,TxPower);
}

static void SetOFDMTxPower(PADAPTER pAdapter, u8 *TxPower)
{
	Hal_SetOFDMTxPower(pAdapter,TxPower);
	}


void SetAntenna(PADAPTER pAdapter)
	{
	Hal_SetAntenna(pAdapter);
}

void	SetAntennaPathPower(PADAPTER pAdapter)
{
	Hal_SetAntennaPathPower(pAdapter);
}

int SetTxPower(PADAPTER pAdapter)
{
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(pAdapter);
	u8			CurrChannel;
	bool			bResult = true;
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
	u8			rf, TxPower[2];

	u8 u1TxPower = pAdapter->mppriv.txpoweridx;
	CurrChannel = pMptCtx->MptChannelToSw;

	if(HAL_IsLegalChannel(pAdapter, CurrChannel) == false)
	{
		DBG_871X("SetTxPower(): CurrentChannel:%d is not valid\n", CurrChannel);
		return false;
	}

	TxPower[ODM_RF_PATH_A] = (u8)(u1TxPower&0xff);
	TxPower[ODM_RF_PATH_B] = (u8)((u1TxPower&0xff00)>>8);
	DBG_871X("TxPower(A, B) = (0x%x, 0x%x)\n", TxPower[ODM_RF_PATH_A], TxPower[ODM_RF_PATH_B]);

	for(rf=0; rf<2; rf++)
	{
		if(TxPower[rf] > MAX_TX_PWR_INDEX_N_MODE) {
			DBG_871X("===> SetTxPower: The power index is too large.\n");
			return false;
		}
		pMptCtx->TxPwrLevel[rf] = TxPower[rf];
	}
	Hal_SetTxPower(pAdapter);
	return true;
}

static void SetTxAGCOffset(PADAPTER pAdapter, u32 ulTxAGCOffset)
{
	u32 TxAGCOffset_B, TxAGCOffset_C, TxAGCOffset_D,tmpAGC;

	TxAGCOffset_B = (ulTxAGCOffset&0x000000ff);
	TxAGCOffset_C = ((ulTxAGCOffset&0x0000ff00)>>8);
	TxAGCOffset_D = ((ulTxAGCOffset&0x00ff0000)>>16);

	tmpAGC = (TxAGCOffset_D<<8 | TxAGCOffset_C<<4 | TxAGCOffset_B);
	write_bbreg(pAdapter, rFPGA0_TxGainStage,
			(bXBTxAGC|bXCTxAGC|bXDTxAGC), tmpAGC);
}

void SetDataRate(PADAPTER pAdapter)
{
	Hal_SetDataRate(pAdapter);
}

void MP_PHY_SetRFPathSwitch(PADAPTER pAdapter ,bool bMain)
{

	PHY_SetRFPathSwitch(pAdapter,bMain);

}


s32 SetThermalMeter(PADAPTER pAdapter, u8 target_ther)
{
	return Hal_SetThermalMeter( pAdapter, target_ther);
}

static void TriggerRFThermalMeter(PADAPTER pAdapter)
{
	Hal_TriggerRFThermalMeter(pAdapter);
}

static u8 ReadRFThermalMeter(PADAPTER pAdapter)
{
	return Hal_ReadRFThermalMeter(pAdapter);
}

void GetThermalMeter(PADAPTER pAdapter, u8 *value)
{
	Hal_GetThermalMeter(pAdapter,value);
}

void SetSingleCarrierTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetSingleCarrierTx(pAdapter,bStart);
}

void SetSingleToneTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetSingleToneTx(pAdapter,bStart);
}

void SetCarrierSuppressionTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetCarrierSuppressionTx(pAdapter, bStart);
}

static void SetCCKContinuousTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetCCKContinuousTx(pAdapter,bStart);
}

static void SetOFDMContinuousTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetOFDMContinuousTx( pAdapter, bStart);
}/* mpt_StartOfdmContTx */

void SetContinuousTx(PADAPTER pAdapter, u8 bStart)
{
	PhySetTxPowerLevel(pAdapter);
	Hal_SetContinuousTx(pAdapter,bStart);
}


void PhySetTxPowerLevel(PADAPTER pAdapter)
{
	struct mp_priv *pmp_priv = &pAdapter->mppriv;

	if (pmp_priv->bSetTxPower==0) // for NO manually set power index
		PHY_SetTxPowerLevel8812(pAdapter,pmp_priv->channel);
}

//------------------------------------------------------------------------------
static void dump_mpframe(PADAPTER padapter, struct xmit_frame *pmpframe)
{
	rtw_hal_mgnt_xmit(padapter, pmpframe);
}

static struct xmit_frame *alloc_mp_xmitframe(struct xmit_priv *pxmitpriv)
{
	struct xmit_frame	*pmpframe;
	struct xmit_buf	*pxmitbuf;

	if ((pmpframe = rtw_alloc_xmitframe(pxmitpriv)) == NULL)
	{
		return NULL;
	}

	if ((pxmitbuf = rtw_alloc_xmitbuf(pxmitpriv)) == NULL)
	{
		rtw_free_xmitframe(pxmitpriv, pmpframe);
		return NULL;
	}

	pmpframe->frame_tag = MP_FRAMETAG;

	pmpframe->pxmitbuf = pxmitbuf;

	pmpframe->buf_addr = pxmitbuf->pbuf;

	pxmitbuf->priv_data = pmpframe;

	return pmpframe;

}

static thread_return mp_xmit_packet_thread(thread_context context)
{
	struct xmit_frame	*pxmitframe;
	struct mp_tx		*pmptx;
	struct mp_priv	*pmp_priv;
	struct xmit_priv	*pxmitpriv;
	PADAPTER padapter;

	pmp_priv = (struct mp_priv *)context;
	pmptx = &pmp_priv->tx;
	padapter = pmp_priv->papdater;
	pxmitpriv = &(padapter->xmitpriv);

	thread_enter("RTW_MP_THREAD");

	DBG_871X("%s:pkTx Start\n", __func__);
	while (1) {
		pxmitframe = alloc_mp_xmitframe(pxmitpriv);
		if (pxmitframe == NULL) {
			if (pmptx->stop ||
			    padapter->bSurpriseRemoved ||
			    padapter->bDriverStopped) {
				goto exit;
			}
			else {
				rtw_msleep_os(1);
				continue;
			}
		}
		_rtw_memcpy((u8 *)(pxmitframe->buf_addr+TXDESC_OFFSET), pmptx->buf, pmptx->write_size);
		_rtw_memcpy(&(pxmitframe->attrib), &(pmptx->attrib), sizeof(struct pkt_attrib));

		dump_mpframe(padapter, pxmitframe);
		pmptx->sended++;
		pmp_priv->tx_pktcount++;

		if (pmptx->stop ||
		    padapter->bSurpriseRemoved ||
		    padapter->bDriverStopped)
			goto exit;
		if ((pmptx->count != 0) &&
		    (pmptx->count == pmptx->sended))
			goto exit;

		flush_signals_thread();
	}

exit:
	//DBG_871X("%s:pkTx Exit\n", __func__);
	rtw_mfree(pmptx->pallocated_buf, pmptx->buf_size);
	pmptx->pallocated_buf = NULL;
	pmptx->stop = 1;

	thread_exit();
}

void fill_txdesc_for_mp(PADAPTER padapter, struct tx_desc *ptxdesc)
{
	struct mp_priv *pmp_priv = &padapter->mppriv;
	_rtw_memcpy(ptxdesc, &(pmp_priv->tx.desc), TXDESC_SIZE);
}

static void fill_tx_desc_8812a(PADAPTER padapter)
{
	struct mp_priv *pmp_priv = &padapter->mppriv;
	//struct tx_desc *pDesc   = &(pmp_priv->tx.desc);
	u8 *pDesc   = (u8 *)&(pmp_priv->tx.desc);
	struct pkt_attrib *pattrib = &(pmp_priv->tx.attrib);

	u32	pkt_size = pattrib->last_txcmdsz;
	s32 bmcast = IS_MCAST(pattrib->ra);
	u8 data_rate,pwr_status,offset;

	SET_TX_DESC_FIRST_SEG_8812(pDesc, 1);
	SET_TX_DESC_LAST_SEG_8812(pDesc, 1);
	SET_TX_DESC_OWN_8812(pDesc, 1);

	SET_TX_DESC_PKT_SIZE_8812(pDesc, pkt_size);

	offset = TXDESC_SIZE + OFFSET_SZ;

	SET_TX_DESC_OFFSET_8812(pDesc, offset);
	SET_TX_DESC_PKT_OFFSET_8812(pDesc, 1);

	if (bmcast) {
		SET_TX_DESC_BMC_8812(pDesc, 1);
	}

	SET_TX_DESC_MACID_8812(pDesc, pattrib->mac_id);
	SET_TX_DESC_RATE_ID_8812(pDesc, pattrib->raid);

	//SET_TX_DESC_RATE_ID_8812(pDesc, RATEID_IDX_G);
	SET_TX_DESC_QUEUE_SEL_8812(pDesc,  pattrib->qsel);
	//SET_TX_DESC_QUEUE_SEL_8812(pDesc,  QSLT_MGNT);

	if (!pattrib->qos_en) {
		SET_TX_DESC_HWSEQ_EN_8812(pDesc, 1); // Hw set sequence number
	} else {
		SET_TX_DESC_SEQ_8812(pDesc, pattrib->seqnum);
	}

	SET_TX_DESC_DISABLE_FB_8812(pDesc, 1);
	SET_TX_DESC_USE_RATE_8812(pDesc, 1);
	SET_TX_DESC_TX_RATE_8812(pDesc, pmp_priv->rateidx);

}

void SetPacketTx(PADAPTER padapter)
{
	u8 *ptr, *pkt_start, *pkt_end;
	u32 pkt_size,offset;
	struct tx_desc *desc;
	struct rtw_ieee80211_hdr *hdr;
	u8 payload;
	s32 bmcast;
	struct pkt_attrib *pattrib;
	struct mp_priv *pmp_priv;

	pmp_priv = &padapter->mppriv;

	if (pmp_priv->tx.stop) return;
	pmp_priv->tx.sended = 0;
	pmp_priv->tx.stop = 0;
	pmp_priv->tx_pktcount = 0;

	//3 1. update_attrib()
	pattrib = &pmp_priv->tx.attrib;
	_rtw_memcpy(pattrib->src, padapter->eeprompriv.mac_addr, ETH_ALEN);
	_rtw_memcpy(pattrib->ta, pattrib->src, ETH_ALEN);
	_rtw_memcpy(pattrib->ra, pattrib->dst, ETH_ALEN);
	bmcast = IS_MCAST(pattrib->ra);
	if (bmcast) {
		pattrib->mac_id = 1;
		pattrib->psta = rtw_get_bcmc_stainfo(padapter);
	} else {
		pattrib->mac_id = 0;
		pattrib->psta = rtw_get_stainfo(&padapter->stapriv, get_bssid(&padapter->mlmepriv));
	}

	pattrib->last_txcmdsz = pattrib->hdrlen + pattrib->pktlen;

	//3 2. allocate xmit buffer
	pkt_size = pattrib->last_txcmdsz;

	if (pmp_priv->tx.pallocated_buf)
		rtw_mfree(pmp_priv->tx.pallocated_buf, pmp_priv->tx.buf_size);
	pmp_priv->tx.write_size = pkt_size;
	pmp_priv->tx.buf_size = pkt_size + XMITBUF_ALIGN_SZ;
	pmp_priv->tx.pallocated_buf = rtw_zmalloc(pmp_priv->tx.buf_size);
	if (pmp_priv->tx.pallocated_buf == NULL) {
		DBG_871X("%s: malloc(%d) fail!!\n", __func__, pmp_priv->tx.buf_size);
		return;
	}
	pmp_priv->tx.buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(pmp_priv->tx.pallocated_buf), XMITBUF_ALIGN_SZ);
	ptr = pmp_priv->tx.buf;

	desc = &(pmp_priv->tx.desc);
	_rtw_memset(desc, 0, TXDESC_SIZE);
	pkt_start = ptr;
	pkt_end = pkt_start + pkt_size;

	//3 3. init TX descriptor
	if(IS_HARDWARE_TYPE_8812(padapter))
		fill_tx_desc_8812a(padapter);

	//3 4. make wlan header, make_wlanhdr()
	hdr = (struct rtw_ieee80211_hdr *)pkt_start;
	SetFrameSubType(&hdr->frame_ctl, pattrib->subtype);
	_rtw_memcpy(hdr->addr1, pattrib->dst, ETH_ALEN); // DA
	_rtw_memcpy(hdr->addr2, pattrib->src, ETH_ALEN); // SA
	_rtw_memcpy(hdr->addr3, get_bssid(&padapter->mlmepriv), ETH_ALEN); // RA, BSSID

	//3 5. make payload
	ptr = pkt_start + pattrib->hdrlen;

	switch (pmp_priv->tx.payload) {
		case 0:
			payload = 0x00;
			break;
		case 1:
			payload = 0x5a;
			break;
		case 2:
			payload = 0xa5;
			break;
		case 3:
			payload = 0xff;
			break;
		default:
			payload = 0x00;
			break;
	}

	_rtw_memset(ptr, payload, pkt_end - ptr);

	//3 6. start thread
	pmp_priv->tx.PktTxThread = kthread_run(mp_xmit_packet_thread, pmp_priv, "RTW_MP_THREAD");
	if (IS_ERR(pmp_priv->tx.PktTxThread))
		DBG_871X("Create PktTx Thread Fail !!!!!\n");
}

void SetPacketRx(PADAPTER pAdapter, u8 bStartRx)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	if(bStartRx)
	{
		// Accept CRC error and destination address
		pHalData->ReceiveConfig = AAP | APM | AM | AB | APP_ICV | ADF | AMF | HTC_LOC_CTRL | APP_MIC | APP_PHYSTS;

		pHalData->ReceiveConfig |= ACRC32;

		rtw_write32(pAdapter, REG_RCR, pHalData->ReceiveConfig);

		// Accept all data frames
		rtw_write16(pAdapter, REG_RXFLTMAP2, 0xFFFF);
	}
	else
	{
		rtw_write32(pAdapter, REG_RCR, 0);
	}
}

void ResetPhyRxPktCount(PADAPTER pAdapter)
{
	u32 i, phyrx_set = 0;

	for (i = 0; i <= 0xF; i++) {
		phyrx_set = 0;
		phyrx_set |= _RXERR_RPT_SEL(i);	//select
		phyrx_set |= RXERR_RPT_RST;	// set counter to zero
		rtw_write32(pAdapter, REG_RXERR_RPT, phyrx_set);
	}
}

static u32 GetPhyRxPktCounts(PADAPTER pAdapter, u32 selbit)
{
	//selection
	u32 phyrx_set = 0, count = 0;

	phyrx_set = _RXERR_RPT_SEL(selbit & 0xF);
	rtw_write32(pAdapter, REG_RXERR_RPT, phyrx_set);

	//Read packet count
	count = rtw_read32(pAdapter, REG_RXERR_RPT) & RXERR_COUNTER_MASK;

	return count;
}

u32 GetPhyRxPktReceived(PADAPTER pAdapter)
{
	u32 OFDM_cnt = 0, CCK_cnt = 0, HT_cnt = 0;

	OFDM_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_OFDM_MPDU_OK);
	CCK_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_CCK_MPDU_OK);
	HT_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_HT_MPDU_OK);

	return OFDM_cnt + CCK_cnt + HT_cnt;
}

u32 GetPhyRxPktCRC32Error(PADAPTER pAdapter)
{
	u32 OFDM_cnt = 0, CCK_cnt = 0, HT_cnt = 0;

	OFDM_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_OFDM_MPDU_FAIL);
	CCK_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_CCK_MPDU_FAIL);
	HT_cnt = GetPhyRxPktCounts(pAdapter, RXERR_TYPE_HT_MPDU_FAIL);

	return OFDM_cnt + CCK_cnt + HT_cnt;
}

//reg 0x808[9:0]: FFT data x
//reg 0x808[22]:  0  -->  1  to get 1 FFT data y
//reg 0x8B4[15:0]: FFT data y report
static u32 rtw_GetPSDData(PADAPTER pAdapter, u32 point)
{
	u32 psd_val=0;

	u16 psd_reg = 0x910;
	u16 psd_regL= 0xF44;

	psd_val = rtw_read32(pAdapter, psd_reg);

	psd_val &= 0xFFBFFC00;
	psd_val |= point;

	rtw_write32(pAdapter, psd_reg, psd_val);
	rtw_mdelay_os(1);
	psd_val |= 0x00400000;

	rtw_write32(pAdapter, psd_reg, psd_val);
	rtw_mdelay_os(1);

	psd_val = rtw_read32(pAdapter, psd_regL);
	psd_val &= 0x0000FFFF;

	return psd_val;
}

/*
 * pts	start_point_min		stop_point_max
 * 128	64			64 + 128 = 192
 * 256	128			128 + 256 = 384
 * 512	256			256 + 512 = 768
 * 1024	512			512 + 1024 = 1536
 *
 */
u32 mp_query_psd(PADAPTER pAdapter, u8 *data)
{
	u32 i, psd_pts=0, psd_start=0, psd_stop=0;
	u32 psd_data=0;


	if (!netif_running(pAdapter->pnetdev)) {
		RT_TRACE(_module_mp_, _drv_warning_, ("mp_query_psd: Fail! interface not opened!\n"));
		return 0;
	}

	if (check_fwstate(&pAdapter->mlmepriv, WIFI_MP_STATE) == false) {
		RT_TRACE(_module_mp_, _drv_warning_, ("mp_query_psd: Fail! not in MP mode!\n"));
		return 0;
	}

	if (strlen(data) == 0) { //default value
		psd_pts = 128;
		psd_start = 64;
		psd_stop = 128;
	} else {
		sscanf(data, "pts=%d,start=%d,stop=%d", &psd_pts, &psd_start, &psd_stop);
	}

	_rtw_memset(data, '\0', sizeof(data));

	i = psd_start;
	while (i < psd_stop)
	{
		if (i >= psd_pts) {
			psd_data = rtw_GetPSDData(pAdapter, i-psd_pts);
		} else {
			psd_data = rtw_GetPSDData(pAdapter, i);
		}
		sprintf(data, "%s%x ", data, psd_data);
		i++;
	}

	#ifdef CONFIG_LONG_DELAY_ISSUE
	rtw_msleep_os(100);
	#else
	rtw_mdelay_os(100);
	#endif

	return strlen(data)+1;
}



void _rtw_mp_xmit_priv (struct xmit_priv *pxmitpriv)
{
	int i,res;
	_adapter *padapter = pxmitpriv->adapter;
	struct xmit_frame	*pxmitframe = (struct xmit_frame*) pxmitpriv->pxmit_frame_buf;
	struct xmit_buf *pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmitbuf;

	u32 max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
	u32 num_xmit_extbuf = NR_XMIT_EXTBUFF;
	if(padapter->registrypriv.mp_mode ==0) {
		max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
		num_xmit_extbuf = NR_XMIT_EXTBUFF;
	} else {
		max_xmit_extbuf_size = 20000;
		num_xmit_extbuf = 1;
	}

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;
	for(i=0; i<num_xmit_extbuf; i++)
	{
		rtw_os_xmit_resource_free(padapter, pxmitbuf,(max_xmit_extbuf_size + XMITBUF_ALIGN_SZ), false);

		pxmitbuf++;
	}

	if(pxmitpriv->pallocated_xmit_extbuf) {
		rtw_vmfree(pxmitpriv->pallocated_xmit_extbuf, num_xmit_extbuf * sizeof(struct xmit_buf) + 4);
	}

	if(padapter->registrypriv.mp_mode ==0)
	{
		max_xmit_extbuf_size = 20000;
		num_xmit_extbuf = 1;
	}
	else
	{
		max_xmit_extbuf_size = MAX_XMIT_EXTBUF_SZ;
		num_xmit_extbuf = NR_XMIT_EXTBUFF;
	}

	// Init xmit extension buff
	_rtw_init_queue(&pxmitpriv->free_xmit_extbuf_queue);

	pxmitpriv->pallocated_xmit_extbuf = rtw_zvmalloc(num_xmit_extbuf * sizeof(struct xmit_buf) + 4);

	if (pxmitpriv->pallocated_xmit_extbuf  == NULL){
		RT_TRACE(_module_rtl871x_xmit_c_,_drv_err_,("alloc xmit_extbuf fail!\n"));
		res= _FAIL;
		goto exit;
	}

	pxmitpriv->pxmit_extbuf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(pxmitpriv->pallocated_xmit_extbuf), 4);

	pxmitbuf = (struct xmit_buf *)pxmitpriv->pxmit_extbuf;

	for (i = 0; i < num_xmit_extbuf; i++)
	{
		_rtw_init_listhead(&pxmitbuf->list);

		pxmitbuf->priv_data = NULL;
		pxmitbuf->padapter = padapter;
		pxmitbuf->buf_tag = XMITBUF_MGNT;

		if((res=rtw_os_xmit_resource_alloc(padapter, pxmitbuf,max_xmit_extbuf_size + XMITBUF_ALIGN_SZ, false)) == _FAIL) {
			res= _FAIL;
			goto exit;
		}

		rtw_list_insert_tail(&pxmitbuf->list, &(pxmitpriv->free_xmit_extbuf_queue.queue));
		#ifdef DBG_XMIT_BUF_EXT
		pxmitbuf->no=i;
		#endif
		pxmitbuf++;

	}

	pxmitpriv->free_xmit_extbuf_cnt = num_xmit_extbuf;

exit:
	;
}



static u32 getPowerDiffByRate8188E(
	PADAPTER	pAdapter,
	u8		CurrChannel,
	u32		RfPath
	)
{
	PMPT_CONTEXT			pMptCtx = &(pAdapter->mppriv.MptCtx);
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(pAdapter);
	u32	PwrGroup=0;
	u32	TxPower=0, Limit=0;
	u32	Pathmapping = (RfPath == ODM_RF_PATH_A?0:8);

	switch(pHalData->EEPROMRegulatory)
	{
		case 0: // driver-defined maximum power offset for longer communication range
				// refer to power by rate table
			PwrGroup = 0;
			Limit = 0xff;
			break;
		case 1: // Power-limit table-defined maximum power offset range
				// choosed by min(power by rate, power limit).
			{
				if(pHalData->pwrGroupCnt == 1)
					PwrGroup = 0;
				if(pHalData->pwrGroupCnt >= 3)
				{
					if(CurrChannel <= 3)
						PwrGroup = 0;
					else if(CurrChannel >= 4 && CurrChannel <= 9)
						PwrGroup = 1;
					else if(CurrChannel > 9)
						PwrGroup = 2;

					if(pHalData->CurrentChannelBW == CHANNEL_WIDTH_20)
						PwrGroup++;
					else
						PwrGroup+=4;
				}
				Limit = 0xff;
			}
			break;
		case 2: // not support power offset by rate.
				// don't increase any power diff
			PwrGroup = 0;
			Limit = 0;
			break;
		default:
			PwrGroup = 0;
			Limit = 0xff;
			break;
	}


	{
		switch(pMptCtx->MptRateIndex)
		{
			case MPT_RATE_1M:
			case MPT_RATE_2M:
			case MPT_RATE_55M:
			case MPT_RATE_11M:
				//CCK rates, don't add any tx power index.
				//RT_DISP(FPHY, PHY_TXPWR,("CCK rates!\n"));
				break;
			case MPT_RATE_6M:	//0xe00 [31:0] = 18M,12M,09M,06M
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][0] = 0x%x, OFDM 6M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0], TxPower));
				break;
			case MPT_RATE_9M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][0] = 0x%x, OFDM 9M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0], TxPower));
				break;
			case MPT_RATE_12M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][0] = 0x%x, OFDM 12M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0], TxPower));
				break;
			case MPT_RATE_18M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][0] = 0x%x, OFDM 24M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][0], TxPower));
				break;
			case MPT_RATE_24M:	//0xe04[31:0] = 54M,48M,36M,24M
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][1] = 0x%x, OFDM 24M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1], TxPower));
				break;
			case MPT_RATE_36M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][1] = 0x%x, OFDM 36M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1], TxPower));
				break;
			case MPT_RATE_48M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][1] = 0x%x, OFDM 48M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1], TxPower));
				break;
			case MPT_RATE_54M:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][1] = 0x%x, OFDM 54M, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][1], TxPower));
				break;
			case MPT_RATE_MCS0: //0xe10[31:0]= MCS=03,02,01,00
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][2] = 0x%x, MCS0, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2], TxPower));
				break;
			case MPT_RATE_MCS1:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][2] = 0x%x, MCS1, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2], TxPower));
				break;
			case MPT_RATE_MCS2:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][2] = 0x%x, MCS2, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2], TxPower));
				break;
			case MPT_RATE_MCS3:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][2] = 0x%x, MCS3, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][2], TxPower));
				break;
			case MPT_RATE_MCS4: //0xe14[31:0]= MCS=07,06,05,04
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][3] = 0x%x, MCS4, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3], TxPower));
				break;
			case MPT_RATE_MCS5:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][3] = 0x%x, MCS5, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3], TxPower));
				break;
			case MPT_RATE_MCS6:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][3] = 0x%x, MCS6, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3], TxPower));
				break;
			case MPT_RATE_MCS7:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][3] = 0x%x, MCS7, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][3], TxPower));
				break;

			case MPT_RATE_MCS8: //0xe18[31:0]= MCS=11,10,09,08
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][4] = 0x%x, MCS8, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4], TxPower));
				break;
			case MPT_RATE_MCS9:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][4] = 0x%x, MCS9, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4], TxPower));
				break;
			case MPT_RATE_MCS10:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][4] = 0x%x, MCS10, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4], TxPower));
				break;
			case MPT_RATE_MCS11:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][4] = 0x%x, MCS11, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][4], TxPower));
				break;
			case MPT_RATE_MCS12:	//0xe1c[31:0]= MCS=15,14,13,12
				TxPower += ((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5+Pathmapping])&0xff);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][5] = 0x%x, MCS12, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5], TxPower));
				break;
			case MPT_RATE_MCS13:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5+Pathmapping])&0xff00)>>8);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][5] = 0x%x, MCS13, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5], TxPower));
				break;
			case MPT_RATE_MCS14:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5+Pathmapping])&0xff0000)>>16);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][5] = 0x%x, MCS14, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5], TxPower));
				break;
			case MPT_RATE_MCS15:
				TxPower += (((pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5+Pathmapping])&0xff000000)>>24);
				//RT_DISP(FPHY, PHY_TXPWR,("MCSTxPowerLevelOriginalOffset[%d][5] = 0x%x, MCS15, TxPower = %d\n",
				//	PwrGroup, pHalData->MCSTxPowerLevelOriginalOffset[PwrGroup][5], TxPower));
				break;
			default:
				break;
		}
	}

	if(TxPower > Limit)
		TxPower = Limit;

	return TxPower;
}



static	u32
mpt_ProQueryCalTxPower_8188E(
	PADAPTER		pAdapter,
	u8			RfPath
	)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(pAdapter);
	u8				TxCount=TX_1S, i = 0;	//default set to 1S
	//PMGNT_INFO			pMgntInfo = &(pAdapter->MgntInfo);
	u32				TxPower = 1, PwrGroup=0, PowerDiffByRate=0;
	u32				TxPowerCCK = 1, TxPowerOFDM = 1, TxPowerBW20 = 1, TxPowerBW40 = 1 ;
	PMPT_CONTEXT		pMptCtx = &(pAdapter->mppriv.MptCtx);
	u8				CurrChannel = pHalData->CurrentChannel;
	u8				index = (CurrChannel -1);
	u8				rf_path=(RfPath), rfPath;
	u8				limit = 0, rate = 0;

	if(HAL_IsLegalChannel(pAdapter, CurrChannel) == false)
	{
		CurrChannel = 1;
	}

	if( pMptCtx->MptRateIndex >= MPT_RATE_1M &&
		pMptCtx->MptRateIndex <= MPT_RATE_11M )
	{
		TxPower = pHalData->Index24G_CCK_Base[rf_path][index];
	}
	else if(pMptCtx->MptRateIndex >= MPT_RATE_6M &&
		pMptCtx->MptRateIndex <= MPT_RATE_54M )
	{
		TxPower = pHalData->Index24G_BW40_Base[rf_path][index];
	}
	else if(pMptCtx->MptRateIndex >= MPT_RATE_MCS0 &&
		pMptCtx->MptRateIndex <= MPT_RATE_MCS7 )
	{
		TxPower = pHalData->Index24G_BW40_Base[rf_path][index];
	}

	//RT_DISP(FPHY, PHY_TXPWR, ("HT40 rate(%d) Tx power(RF-%c) = 0x%x\n", pMptCtx->MptRateIndex, ((rf_path==0)?'A':'B'), TxPower));


	if(pMptCtx->MptRateIndex >= MPT_RATE_6M &&
		pMptCtx->MptRateIndex <= MPT_RATE_54M )
	{
		TxPower += pHalData->OFDM_24G_Diff[rf_path][TxCount];
		///RT_DISP(FPHY, PHY_TXPWR, ("+OFDM_PowerDiff(RF-%c) = 0x%x\n", ((rf_path==0)?'A':'B'),
		//	pHalData->OFDM_24G_Diff[rf_path][TxCount]));
	}

	if(pMptCtx->MptRateIndex >= MPT_RATE_MCS0)
	{
		if (pHalData->CurrentChannelBW == CHANNEL_WIDTH_20)
		{
			TxPower += pHalData->BW20_24G_Diff[rf_path][TxCount];
		//	RT_DISP(FPHY, PHY_TXPWR, ("+HT20_PowerDiff(RF-%c) = 0x%x\n", ((rf_path==0)?'A':'B'),
		//		pHalData->BW20_24G_Diff[rf_path][TxCount]));
		}
	}


#ifdef ENABLE_POWER_BY_RATE
	PowerDiffByRate = getPowerDiffByRate8188E(pAdapter, CurrChannel, RfPath);
#else
	PowerDiffByRate = 0;
#endif


	//RT_DISP(FPHY, PHY_TXPWR, ("+PowerDiffByRate(RF-%c) = 0x%x\n", ((rf_path==0)?'A':'B'),
	//	PowerDiffByRate));
	TxPower += PowerDiffByRate;
//	RT_DISP(FPHY, PHY_TXPWR, ("Final TxPower(RF-%c) = %d(0x%x)\n", ((rf_path==0)?'A':'B'),
	//	TxPower, TxPower));

	/*
	if(TxPower > 0x3f)
		TxPower = 0x3f;
	*/

	// 2012/11/02 Awk: add power limit mechansim
	if( pMptCtx->MptRateIndex >= MPT_RATE_1M &&
		pMptCtx->MptRateIndex <= MPT_RATE_11M )
	{
		rate = MGN_1M;
	}
	else if(pMptCtx->MptRateIndex >= MPT_RATE_6M &&
		pMptCtx->MptRateIndex <= MPT_RATE_54M )
	{
		rate = MGN_54M;
	}
	else if(pMptCtx->MptRateIndex >= MPT_RATE_MCS0 &&
		pMptCtx->MptRateIndex <= MPT_RATE_MCS7 )
	{
		rate = MGN_MCS7;
	}
	#ifdef CONFIG_8192E
	limit = PHY_GetPowerLimitValue(pAdapter, pMptCtx->RegTxPwrLimit,
								   pHalData->CurrentBandType,
								   pHalData->CurrentChannelBW,RfPath,
								   rate, CurrChannel);
	#endif
	TxPower = TxPower > limit ? limit : TxPower;

	return TxPower;
}


u32 mpt_ProQueryCalTxPower(
	PADAPTER	pAdapter,
		u8		RfPath
	)
{

	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(pAdapter);
	u32			TxPower = 1, PwrGroup=0, PowerDiffByRate=0;
	PMPT_CONTEXT	pMptCtx = &(pAdapter->mppriv.MptCtx);
	u8			limit = 0, rate = 0;
	rate=pMptCtx->MptRateIndex;

	#ifdef CONFIG_8812A
	TxPower = PHY_GetTxPowerIndex_8812A(pAdapter, RfPath, rate,pHalData->CurrentChannelBW, pHalData->CurrentChannel);
	#endif
	return TxPower;
}


void Hal_ProSetCrystalCap (PADAPTER pAdapter , u32 CrystalCapVal)
{
	HAL_DATA_TYPE		*pHalData	= GET_HAL_DATA(pAdapter);

	CrystalCapVal = pHalData->CrystalCap & 0x3F;

	if(IS_HARDWARE_TYPE_8812(pAdapter))
	{
		// write 0x2C[30:25] = 0x2C[24:19] = CrystalCap
		PHY_SetBBReg(pAdapter, REG_MAC_PHY_CTRL, 0x7FF80000, (CrystalCapVal | (CrystalCapVal << 6)));
	}
	else
	{
		// write 0x2C[23:18] = 0x2C[17:12] = CrystalCap
		PHY_SetBBReg(pAdapter, REG_MAC_PHY_CTRL, 0xFFF000, (CrystalCapVal | (CrystalCapVal << 6)));
	}
}



#endif
