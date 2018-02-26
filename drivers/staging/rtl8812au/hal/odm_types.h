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
#ifndef __ODM_TYPES_H__
#define __ODM_TYPES_H__

//
// Define Different SW team support
//
#define		ODM_AP			0x01    //BIT0
#define		ODM_ADSL		0x02    //BIT1
#define		ODM_CE			0x04    //BIT2
#define		ODM_WIN			0x08    //BIT3

#define		RT_PCI_INTERFACE				1
#define		RT_USB_INTERFACE				2
#define		RT_SDIO_INTERFACE				3

typedef enum _HAL_STATUS{
	HAL_STATUS_SUCCESS,
	HAL_STATUS_FAILURE,
}HAL_STATUS,*PHAL_STATUS;

typedef enum _RT_SPINLOCK_TYPE{
	RT_TEMP =1,
}RT_SPINLOCK_TYPE;

	#include <drv_types.h>

	#define s1Byte		s8
	#define	ps1Byte		s8*

	#define s2Byte		s16
	#define	ps2Byte		s16*

	#define s4Byte		s32
	#define	ps4Byte		s32*

	#define s8Byte		s64
	#define	ps8Byte		s64*

	#define DEV_BUS_TYPE	RT_USB_INTERFACE

	typedef struct timer_list		RT_TIMER, *PRT_TIMER;
	typedef  void *				RT_TIMER_CALL_BACK;
	#define	STA_INFO_T			struct sta_info
	#define	PSTA_INFO_T		struct sta_info *

	#define SET_TX_DESC_ANTSEL_A_88E(__pTxDesc, __Value) SET_BITS_TO_LE_4BYTE(__pTxDesc+8, 24, 1, __Value)
	#define SET_TX_DESC_ANTSEL_B_88E(__pTxDesc, __Value) SET_BITS_TO_LE_4BYTE(__pTxDesc+8, 25, 1, __Value)
	#define SET_TX_DESC_ANTSEL_C_88E(__pTxDesc, __Value) SET_BITS_TO_LE_4BYTE(__pTxDesc+28, 29, 1, __Value)

	//define useless flag to avoid compile warning
	#define	USE_WORKITEM 0
	#define		FOR_BRAZIL_PRETEST 0
	#define	BT_30_SUPPORT			0
	#define   FPGA_TWO_MAC_VERIFICATION	0

#endif // __ODM_TYPES_H__
