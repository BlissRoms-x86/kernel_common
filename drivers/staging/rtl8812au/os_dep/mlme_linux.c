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


#define _MLME_OSDEP_C_

#include <drv_types.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
void rtw_join_timeout_handler(void *FunctionContext)
#else
void rtw_join_timeout_handler(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *adapter = (_adapter *)FunctionContext;
#else
	_adapter *adapter = from_timer(adapter, t, mlmepriv.assoc_timer);
#endif

	_rtw_join_timeout_handler(adapter);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
void _rtw_scan_timeout_handler(void *FunctionContext)
#else
void _rtw_scan_timeout_handler(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *adapter = (_adapter *)FunctionContext;
#else
	_adapter *adapter = from_timer(adapter, t, mlmepriv.scan_to_timer);
#endif
	rtw_scan_timeout_handler(adapter);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void _dynamic_check_timer_handlder(void *FunctionContext)
#else
static void _dynamic_check_timer_handlder(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *adapter = (_adapter *)FunctionContext;
#else
	_adapter *adapter = from_timer(adapter, t, mlmepriv.dynamic_chk_timer);
#endif

#if (MP_DRIVER == 1)
	if (adapter->registrypriv.mp_mode == 1)
		return;
#endif
	rtw_dynamic_check_timer_handlder(adapter);

	_set_timer(&adapter->mlmepriv.dynamic_chk_timer, 2000);
}

#ifdef CONFIG_SET_SCAN_DENY_TIMER
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
void _rtw_set_scan_deny_timer_hdl(void *FunctionContext)
#else
void _rtw_set_scan_deny_timer_hdl(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *adapter = (_adapter *)FunctionContext;
#else
	_adapter *adapter = from_timer(adapter, t, mlmepriv.set_scan_deny_timer);
#endif

	rtw_set_scan_deny_timer_hdl(adapter);
}
#endif


void rtw_init_mlme_timer(_adapter *padapter)
{
	struct	mlme_priv *pmlmepriv = &padapter->mlmepriv;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_init_timer(&(pmlmepriv->assoc_timer), padapter->pnetdev, rtw_join_timeout_handler, padapter);
	_init_timer(&(pmlmepriv->scan_to_timer), padapter->pnetdev, _rtw_scan_timeout_handler, padapter);
	_init_timer(&(pmlmepriv->dynamic_chk_timer), padapter->pnetdev, _dynamic_check_timer_handlder, padapter);
	#ifdef CONFIG_SET_SCAN_DENY_TIMER
	_init_timer(&(pmlmepriv->set_scan_deny_timer), padapter->pnetdev, _rtw_set_scan_deny_timer_hdl, padapter);
	#endif
#else
	timer_setup(&pmlmepriv->assoc_timer, rtw_join_timeout_handler, 0);
	timer_setup(&pmlmepriv->scan_to_timer, _rtw_scan_timeout_handler, 0);
	timer_setup(&pmlmepriv->dynamic_chk_timer, _dynamic_check_timer_handlder, 0);
	#ifdef CONFIG_SET_SCAN_DENY_TIMER
	timer_setup(&pmlmepriv->set_scan_deny_timer, _rtw_set_scan_deny_timer_hdl, 0);
	#endif
#endif
}

void rtw_os_indicate_connect(_adapter *adapter)
{
#ifdef CONFIG_IOCTL_CFG80211
	rtw_cfg80211_indicate_connect(adapter);
#endif //CONFIG_IOCTL_CFG80211

	rtw_indicate_wx_assoc_event(adapter);
	netif_carrier_on(adapter->pnetdev);

	if(adapter->pid[2] !=0)
		rtw_signal_process(adapter->pid[2], SIGALRM);
}

extern void indicate_wx_scan_complete_event(_adapter *padapter);
void rtw_os_indicate_scan_done( _adapter *padapter, bool aborted)
{
#ifdef CONFIG_IOCTL_CFG80211
	rtw_cfg80211_indicate_scan_done(wdev_to_priv(padapter->rtw_wdev), aborted);
#endif
	indicate_wx_scan_complete_event(padapter);
}

static RT_PMKID_LIST   backupPMKIDList[ NUM_PMKID_CACHE ];
void rtw_reset_securitypriv( _adapter *adapter )
{
	u8	backupPMKIDIndex = 0;
	u8	backupTKIPCountermeasure = 0x00;
	u32	backupTKIPcountermeasure_time = 0;

	if(adapter->securitypriv.dot11AuthAlgrthm == dot11AuthAlgrthm_8021X)//802.1x
	{
		// Added by Albert 2009/02/18
		// We have to backup the PMK information for WiFi PMK Caching test item.
		//
		// Backup the btkip_countermeasure information.
		// When the countermeasure is trigger, the driver have to disconnect with AP for 60 seconds.

		_rtw_memset( &backupPMKIDList[ 0 ], 0x00, sizeof( RT_PMKID_LIST ) * NUM_PMKID_CACHE );

		_rtw_memcpy( &backupPMKIDList[ 0 ], &adapter->securitypriv.PMKIDList[ 0 ], sizeof( RT_PMKID_LIST ) * NUM_PMKID_CACHE );
		backupPMKIDIndex = adapter->securitypriv.PMKIDIndex;
		backupTKIPCountermeasure = adapter->securitypriv.btkip_countermeasure;
		backupTKIPcountermeasure_time = adapter->securitypriv.btkip_countermeasure_time;

		_rtw_memset((unsigned char *)&adapter->securitypriv, 0, sizeof (struct security_priv));
		//_init_timer(&(adapter->securitypriv.tkip_timer),adapter->pnetdev, rtw_use_tkipkey_handler, adapter);

		// Added by Albert 2009/02/18
		// Restore the PMK information to securitypriv structure for the following connection.
		_rtw_memcpy( &adapter->securitypriv.PMKIDList[ 0 ], &backupPMKIDList[ 0 ], sizeof( RT_PMKID_LIST ) * NUM_PMKID_CACHE );
		adapter->securitypriv.PMKIDIndex = backupPMKIDIndex;
		adapter->securitypriv.btkip_countermeasure = backupTKIPCountermeasure;
		adapter->securitypriv.btkip_countermeasure_time = backupTKIPcountermeasure_time;

		adapter->securitypriv.ndisauthtype = Ndis802_11AuthModeOpen;
		adapter->securitypriv.ndisencryptstatus = Ndis802_11WEPDisabled;

	}
	else //reset values in securitypriv
	{
		//if(adapter->mlmepriv.fw_state & WIFI_STATION_STATE)
		//{
		struct security_priv *psec_priv=&adapter->securitypriv;

		psec_priv->dot11AuthAlgrthm =dot11AuthAlgrthm_Open;  //open system
		psec_priv->dot11PrivacyAlgrthm = _NO_PRIVACY_;
		psec_priv->dot11PrivacyKeyIndex = 0;

		psec_priv->dot118021XGrpPrivacy = _NO_PRIVACY_;
		psec_priv->dot118021XGrpKeyid = 1;

		psec_priv->ndisauthtype = Ndis802_11AuthModeOpen;
		psec_priv->ndisencryptstatus = Ndis802_11WEPDisabled;
		//}
	}
}

void rtw_os_indicate_disconnect( _adapter *adapter )
{
   //RT_PMKID_LIST   backupPMKIDList[ NUM_PMKID_CACHE ];



	netif_carrier_off(adapter->pnetdev); // Do it first for tx broadcast pkt after disconnection issue!

#ifdef CONFIG_IOCTL_CFG80211
	rtw_cfg80211_indicate_disconnect(adapter);
#endif //CONFIG_IOCTL_CFG80211

	rtw_indicate_wx_disassoc_event(adapter);

	 rtw_reset_securitypriv( adapter );
}

void rtw_report_sec_ie(_adapter *adapter,u8 authmode,u8 *sec_ie)
{
	uint	len;
	u8	*buff,*p,i;
	union iwreq_data wrqu;



	RT_TRACE(_module_mlme_osdep_c_,_drv_info_,("+rtw_report_sec_ie, authmode=%d\n", authmode));

	buff = NULL;
	if(authmode==_WPA_IE_ID_)
	{
		RT_TRACE(_module_mlme_osdep_c_,_drv_info_,("rtw_report_sec_ie, authmode=%d\n", authmode));

		buff = rtw_malloc(IW_CUSTOM_MAX);

		_rtw_memset(buff,0,IW_CUSTOM_MAX);

		p=buff;

		p+=sprintf(p,"ASSOCINFO(ReqIEs=");

		len = sec_ie[1]+2;
		len =  (len < IW_CUSTOM_MAX) ? len:IW_CUSTOM_MAX;

		for(i=0;i<len;i++){
			p+=sprintf(p,"%02x",sec_ie[i]);
		}

		p+=sprintf(p,")");

		_rtw_memset(&wrqu,0,sizeof(wrqu));

		wrqu.data.length=p-buff;

		wrqu.data.length = (wrqu.data.length<IW_CUSTOM_MAX) ? wrqu.data.length:IW_CUSTOM_MAX;

#ifndef CONFIG_IOCTL_CFG80211
		wireless_send_event(adapter->pnetdev,IWEVCUSTOM,&wrqu,buff);
#endif

		if(buff)
		    rtw_mfree(buff, IW_CUSTOM_MAX);

	}



}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void _survey_timer_hdl(void *FunctionContext)
#else
static void _survey_timer_hdl(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *padapter = (_adapter *)FunctionContext;
#else
	_adapter *padapter = from_timer(padapter, t, mlmeextpriv.survey_timer);
#endif

	survey_timer_hdl(padapter);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void _link_timer_hdl (void *FunctionContext)
#else
static void _link_timer_hdl(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_adapter *padapter = (_adapter *)FunctionContext;
#else
	_adapter *padapter = from_timer(padapter, t, mlmeextpriv.link_timer);
#endif
	link_timer_hdl(padapter);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void _addba_timer_hdl(void *FunctionContext)
#else
static void _addba_timer_hdl(struct timer_list *t)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	struct sta_info *psta = (struct sta_info *)FunctionContext;
#else
	struct sta_info *psta = from_timer(psta, t, addba_retry_timer);
#endif
	addba_timer_hdl(psta);
}

void init_addba_retry_timer(_adapter *padapter, struct sta_info *psta)
{

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_init_timer(&psta->addba_retry_timer, padapter->pnetdev, _addba_timer_hdl, psta);
#else
	timer_setup(&psta->addba_retry_timer, _addba_timer_hdl, 0);
#endif
}

void init_mlme_ext_timer(_adapter *padapter)
{
	struct	mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	_init_timer(&pmlmeext->survey_timer, padapter->pnetdev, _survey_timer_hdl, padapter);
	_init_timer(&pmlmeext->link_timer, padapter->pnetdev, _link_timer_hdl, padapter);
#else
	timer_setup(&pmlmeext->survey_timer, _survey_timer_hdl, 0);
	timer_setup(&pmlmeext->link_timer, _link_timer_hdl, 0);
#endif
}

u8 rtw_handle_tkip_countermeasure(_adapter* padapter)
{
	u8 status = _SUCCESS;
	u32 cur_time = 0;

	if (padapter->securitypriv.btkip_countermeasure == true) {
		cur_time = rtw_get_current_time();

		if( (cur_time - padapter->securitypriv.btkip_countermeasure_time) > 60 * HZ )
		{
			padapter->securitypriv.btkip_countermeasure = false;
			padapter->securitypriv.btkip_countermeasure_time = 0;
		}
		else
		{
			status = _FAIL;
		}
	}

	return status;

}

#ifdef CONFIG_AP_MODE

void rtw_indicate_sta_assoc_event(_adapter *padapter, struct sta_info *psta)
{
	union iwreq_data wrqu;
	struct sta_priv *pstapriv = &padapter->stapriv;

	if(psta==NULL)
		return;

	if(psta->aid > NUM_STA)
		return;

	if(pstapriv->sta_aid[psta->aid - 1] != psta)
		return;


	wrqu.addr.sa_family = ARPHRD_ETHER;

	_rtw_memcpy(wrqu.addr.sa_data, psta->hwaddr, ETH_ALEN);

	DBG_871X("+rtw_indicate_sta_assoc_event\n");

#ifndef CONFIG_IOCTL_CFG80211
	wireless_send_event(padapter->pnetdev, IWEVREGISTERED, &wrqu, NULL);
#endif

}

void rtw_indicate_sta_disassoc_event(_adapter *padapter, struct sta_info *psta)
{
	union iwreq_data wrqu;
	struct sta_priv *pstapriv = &padapter->stapriv;

	if(psta==NULL)
		return;

	if(psta->aid > NUM_STA)
		return;

	if(pstapriv->sta_aid[psta->aid - 1] != psta)
		return;


	wrqu.addr.sa_family = ARPHRD_ETHER;

	_rtw_memcpy(wrqu.addr.sa_data, psta->hwaddr, ETH_ALEN);

	DBG_871X("+rtw_indicate_sta_disassoc_event\n");

#ifndef CONFIG_IOCTL_CFG80211
	wireless_send_event(padapter->pnetdev, IWEVEXPIRED, &wrqu, NULL);
#endif

}


#ifdef CONFIG_HOSTAPD_MLME

static int mgnt_xmit_entry(struct sk_buff *skb, struct net_device *pnetdev)
{
	struct hostapd_priv *phostapdpriv = rtw_netdev_priv(pnetdev);
	_adapter *padapter = (_adapter *)phostapdpriv->padapter;

	//DBG_871X("%s\n", __FUNCTION__);

	return rtw_hal_hostap_mgnt_xmit_entry(padapter, skb);
}

static int mgnt_netdev_open(struct net_device *pnetdev)
{
	struct hostapd_priv *phostapdpriv = rtw_netdev_priv(pnetdev);

	DBG_871X("mgnt_netdev_open: MAC Address:" MAC_FMT "\n", MAC_ARG(pnetdev->dev_addr));


	init_usb_anchor(&phostapdpriv->anchored);

	if(!rtw_netif_queue_stopped(pnetdev))
		rtw_netif_start_queue(pnetdev);
	else
		rtw_netif_wake_queue(pnetdev);


	netif_carrier_on(pnetdev);

	//rtw_write16(phostapdpriv->padapter, 0x0116, 0x0100);//only excluding beacon

	return 0;
}
static int mgnt_netdev_close(struct net_device *pnetdev)
{
	struct hostapd_priv *phostapdpriv = rtw_netdev_priv(pnetdev);

	DBG_871X("%s\n", __FUNCTION__);

	usb_kill_anchored_urbs(&phostapdpriv->anchored);

	netif_carrier_off(pnetdev);

	if (!rtw_netif_queue_stopped(pnetdev))
		rtw_netif_stop_queue(pnetdev);

	//rtw_write16(phostapdpriv->padapter, 0x0116, 0x3f3f);

	return 0;
}

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,29))
static const struct net_device_ops rtl871x_mgnt_netdev_ops = {
	.ndo_open = mgnt_netdev_open,
       .ndo_stop = mgnt_netdev_close,
       .ndo_start_xmit = mgnt_xmit_entry,
       //.ndo_set_mac_address = r871x_net_set_mac_address,
       //.ndo_get_stats = r871x_net_get_stats,
       //.ndo_do_ioctl = r871x_mp_ioctl,
};
#endif

int hostapd_mode_init(_adapter *padapter)
{
	unsigned char mac[ETH_ALEN];
	struct hostapd_priv *phostapdpriv;
	struct net_device *pnetdev;

	pnetdev = rtw_alloc_etherdev(sizeof(struct hostapd_priv));
	if (!pnetdev)
	   return -ENOMEM;

	//SET_MODULE_OWNER(pnetdev);
       ether_setup(pnetdev);

	//pnetdev->type = ARPHRD_IEEE80211;

	phostapdpriv = rtw_netdev_priv(pnetdev);
	phostapdpriv->pmgnt_netdev = pnetdev;
	phostapdpriv->padapter= padapter;
	padapter->phostapdpriv = phostapdpriv;

	//pnetdev->init = NULL;

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,29))

	DBG_871X("register rtl871x_mgnt_netdev_ops to netdev_ops\n");

	pnetdev->netdev_ops = &rtl871x_mgnt_netdev_ops;

#else

	pnetdev->open = mgnt_netdev_open;

	pnetdev->stop = mgnt_netdev_close;

	pnetdev->hard_start_xmit = mgnt_xmit_entry;

	//pnetdev->set_mac_address = r871x_net_set_mac_address;

	//pnetdev->get_stats = r871x_net_get_stats;

	//pnetdev->do_ioctl = r871x_mp_ioctl;

#endif

	pnetdev->watchdog_timeo = HZ; /* 1 second timeout */

	//pnetdev->wireless_handlers = NULL;

#ifdef CONFIG_TCP_CSUM_OFFLOAD_TX
	pnetdev->features |= NETIF_F_IP_CSUM;
#endif



	if(dev_alloc_name(pnetdev,"mgnt.wlan%d") < 0)
	{
		DBG_871X("hostapd_mode_init(): dev_alloc_name, fail! \n");
	}


	//SET_NETDEV_DEV(pnetdev, pintfpriv->udev);


	mac[0]=0x00;
	mac[1]=0xe0;
	mac[2]=0x4c;
	mac[3]=0x87;
	mac[4]=0x11;
	mac[5]=0x12;

	_rtw_memcpy(pnetdev->dev_addr, mac, ETH_ALEN);


	netif_carrier_off(pnetdev);


	/* Tell the network stack we exist */
	if (register_netdev(pnetdev) != 0)
	{
		DBG_871X("hostapd_mode_init(): register_netdev fail!\n");

		if(pnetdev)
		{
			rtw_free_netdev(pnetdev);
		}
	}

	return 0;

}

void hostapd_mode_unload(_adapter *padapter)
{
	struct hostapd_priv *phostapdpriv = padapter->phostapdpriv;
	struct net_device *pnetdev = phostapdpriv->pmgnt_netdev;

	unregister_netdev(pnetdev);
	rtw_free_netdev(pnetdev);

}

#endif
#endif
