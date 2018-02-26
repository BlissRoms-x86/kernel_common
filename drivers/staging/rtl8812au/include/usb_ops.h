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
#ifndef __USB_OPS_H_
#define __USB_OPS_H_


#define REALTEK_USB_VENQT_READ		0xC0
#define REALTEK_USB_VENQT_WRITE		0x40
#define REALTEK_USB_VENQT_CMD_REQ	0x05
#define REALTEK_USB_VENQT_CMD_IDX	0x00

enum{
	VENDOR_WRITE = 0x00,
	VENDOR_READ = 0x01,
};
#define ALIGNMENT_UNIT				16
#define MAX_VENDOR_REQ_CMD_SIZE	254		//8188cu SIE Support
#define MAX_USB_IO_CTL_SIZE		(MAX_VENDOR_REQ_CMD_SIZE +ALIGNMENT_UNIT)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12))
#define rtw_usb_control_msg(dev, pipe, request, requesttype, value, index, data, size, timeout_ms) \
	usb_control_msg((dev), (pipe), (request), (requesttype), (value), (index), (data), (size), (timeout_ms))
#define rtw_usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout_ms) \
	usb_bulk_msg((usb_dev), (pipe), (data), (len), (actual_length), (timeout_ms))
#else
#define rtw_usb_control_msg(dev, pipe, request, requesttype, value, index, data, size,timeout_ms) \
	usb_control_msg((dev), (pipe), (request), (requesttype), (value), (index), (data), (size), \
		((timeout_ms) == 0) ||((timeout_ms)*HZ/1000>0)?((timeout_ms)*HZ/1000):1)
#define rtw_usb_bulk_msg(usb_dev, pipe, data, len, actual_length, timeout_ms) \
	usb_bulk_msg((usb_dev), (pipe), (data), (len), (actual_length), \
		((timeout_ms) == 0) ||((timeout_ms)*HZ/1000>0)?((timeout_ms)*HZ/1000):1)
#endif
#include <usb_ops_linux.h>

void rtl8812au_set_hw_type(_adapter *padapter);
void rtl8812au_set_intf_ops(struct _io_ops *pops);

/*
* Increase and check if the continual_urb_error of this @param dvobjprive is larger than MAX_CONTINUAL_URB_ERR
* @return true:
* @return false:
*/
static inline int rtw_inc_and_chk_continual_urb_error(struct dvobj_priv *dvobj)
{
	int ret = false;
	int value;
	if( (value=ATOMIC_INC_RETURN(&dvobj->continual_urb_error)) > MAX_CONTINUAL_URB_ERR) {
		DBG_871X("[dvobj:%p][ERROR] continual_urb_error:%d > %d\n", dvobj, value, MAX_CONTINUAL_URB_ERR);
		ret = true;
	} else {
		//DBG_871X("[dvobj:%p] continual_urb_error:%d\n", dvobj, value);
	}
	return ret;
}

/*
* Set the continual_urb_error of this @param dvobjprive to 0
*/
static inline void rtw_reset_continual_urb_error(struct dvobj_priv *dvobj)
{
	ATOMIC_SET(&dvobj->continual_urb_error, 0);
}

enum RTW_USB_SPEED {
	RTW_USB_SPEED_UNKNOWN	= 0,
	RTW_USB_SPEED_1_1	= 1,
	RTW_USB_SPEED_2		= 2,
	RTW_USB_SPEED_3		= 3,
};

#define IS_FULL_SPEED_USB(Adapter)	(adapter_to_dvobj(Adapter)->usb_speed == RTW_USB_SPEED_1_1)
#define IS_HIGH_SPEED_USB(Adapter)	(adapter_to_dvobj(Adapter)->usb_speed == RTW_USB_SPEED_2)
#define IS_SUPER_SPEED_USB(Adapter)	(adapter_to_dvobj(Adapter)->usb_speed == RTW_USB_SPEED_3)

#define USB_SUPER_SPEED_BULK_SIZE	1024	// usb 3.0
#define USB_HIGH_SPEED_BULK_SIZE	512		// usb 2.0
#define USB_FULL_SPEED_BULK_SIZE	64		// usb 1.1

static inline u8 rtw_usb_bulk_size_boundary(_adapter * padapter,int buf_len)
{
	u8 rst = true;

	if (IS_SUPER_SPEED_USB(padapter))
		rst = (0 == (buf_len) % USB_SUPER_SPEED_BULK_SIZE)?true:false;
	if (IS_HIGH_SPEED_USB(padapter))
		rst = (0 == (buf_len) % USB_HIGH_SPEED_BULK_SIZE)?true:false;
	else
		rst = (0 == (buf_len) % USB_FULL_SPEED_BULK_SIZE)?true:false;
	return rst;
}


#endif //__USB_OPS_H_
