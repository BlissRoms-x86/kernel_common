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
#define _HAL_COM_PHYCFG_C_

#include <drv_types.h>


#ifndef CONFIG_EMBEDDED_FWIMG
int
phy_ConfigMACWithParaFile(
	IN	PADAPTER	Adapter,
	IN	u8*			pFileName
)
{
	int	rtStatus = _FAIL;

	return rtStatus;
}

int
PHY_ConfigBBWithPowerLimitTableParaFile(
	IN	PADAPTER	Adapter,
	IN	s8*			pFileName
)
{
	int		rtStatus = _SUCCESS;

	return rtStatus;
}

int
phy_ConfigBBWithParaFile(
	IN	PADAPTER	Adapter,
	IN	u8*			pFileName
)
{
	int		rtStatus = _SUCCESS;

	return rtStatus;
}

int
phy_ConfigBBWithPgParaFile(
	IN	PADAPTER	Adapter,
	IN	u8*			pFileName)
{
	int		rtStatus = _SUCCESS;

	return rtStatus;
}

int
phy_ConfigBBWithMpParaFile(
	IN	PADAPTER	Adapter,
	IN	u8*			pFileName
)
{
	int		rtStatus = _SUCCESS;

	return rtStatus;
}

int
PHY_ConfigRFWithParaFile(
	IN	PADAPTER		Adapter,
	IN	u8*				pFileName,
	IN	u8				eRFPath
)
{
	int	rtStatus = _SUCCESS;

	return rtStatus;
}

int
PHY_ConfigRFWithTxPwrTrackParaFile(
	IN	PADAPTER		Adapter,
	IN	u8*				pFileName
)
{
	int	rtStatus = _SUCCESS;

	return rtStatus;
}
#endif
