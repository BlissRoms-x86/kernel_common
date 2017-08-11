/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <uapi/sound/sof-ipc.h>
#include "ops.h"
#include "sof-priv.h"

int snd_sof_pci_update_bits_unlocked(struct snd_sof_dev *sdev, u32 offset,
				u32 mask, u32 value)
{
	bool change;
	unsigned int old, new;
	u32 ret;

	pci_read_config_dword(sdev->pci, offset, &ret);
	dev_dbg(sdev->dev, "Debug PCIR: %8.8x at  %8.8x\n",\
		pci_read_config_dword(sdev->pci, offset, &ret), offset);


	old = ret;
	new = (old & (~mask)) | (value & mask);

	change = (old != new);
	if (change) {
		pci_write_config_dword(sdev->pci, offset, new);
		dev_dbg(sdev->dev, "Debug PCIW: %8.8x at  %8.8x\n",value,
offset);
	}

	return change;
}
EXPORT_SYMBOL(snd_sof_pci_update_bits_unlocked);

int snd_sof_pci_update_bits(struct snd_sof_dev *sdev, u32 offset,
				u32 mask, u32 value)
{
	unsigned long flags;
	bool change;

	spin_lock_irqsave(&sdev->spinlock, flags);
	change = snd_sof_pci_update_bits_unlocked(sdev, offset, mask, value);
	spin_unlock_irqrestore(&sdev->spinlock, flags);
	return change;
}
EXPORT_SYMBOL(snd_sof_pci_update_bits);

int snd_sof_dsp_update_bits_unlocked(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u32 mask, u32 value)
{
	bool change;
	unsigned int old, new;
	u32 ret;

	ret = snd_sof_dsp_read(sdev, bar, offset);

	old = ret;
	new = (old & (~mask)) | (value & mask);

	change = (old != new);
	if (change)
		snd_sof_dsp_write(sdev, bar, offset, new);

	return change;
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits_unlocked);

int snd_sof_dsp_update_bits64_unlocked(struct snd_sof_dev *sdev, u32 bar,
	u32 offset, u64 mask, u64 value)
{
	bool change;
	u64 old, new;

	old = snd_sof_dsp_read64(sdev, bar, offset);

	new = (old & (~mask)) | (value & mask);

	change = (old != new);
	if (change)
		snd_sof_dsp_write64(sdev, bar, offset, new);

	return change;
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits64_unlocked);

/* This is for registers bits with attribute RWC */
void snd_sof_dsp_update_bits_forced_unlocked(struct snd_sof_dev *sdev, u32 bar,
	u32 offset, u32 mask, u32 value)
{
	unsigned int old, new;
	u32 ret;

	ret = snd_sof_dsp_read(sdev, bar, offset);

	old = ret;
	new = (old & (~mask)) | (value & mask);

	snd_sof_dsp_write(sdev, bar, offset, new);
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits_forced_unlocked);

int snd_sof_dsp_update_bits(struct snd_sof_dev *sdev, u32 bar, u32 offset,
				u32 mask, u32 value)
{
	unsigned long flags;
	bool change;

	spin_lock_irqsave(&sdev->spinlock, flags);
	change = snd_sof_dsp_update_bits_unlocked(sdev, bar, offset, mask, value);
	spin_unlock_irqrestore(&sdev->spinlock, flags);
	return change;
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits);

int snd_sof_dsp_update_bits64(struct snd_sof_dev *sdev, u32 bar, u32 offset,
				u64 mask, u64 value)
{
	unsigned long flags;
	bool change;

	spin_lock_irqsave(&sdev->spinlock, flags);
	change = snd_sof_dsp_update_bits64_unlocked(sdev, bar, offset, mask, value);
	spin_unlock_irqrestore(&sdev->spinlock, flags);
	return change;
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits64);

/* This is for registers bits with attribute RWC */
void snd_sof_dsp_update_bits_forced(struct snd_sof_dev *sdev, u32 bar, 
	u32 offset, u32 mask, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&sdev->spinlock, flags);
	snd_sof_dsp_update_bits_forced_unlocked(sdev, bar, offset, mask, value);
	spin_unlock_irqrestore(&sdev->spinlock, flags);
}
EXPORT_SYMBOL(snd_sof_dsp_update_bits_forced);

int snd_sof_dsp_register_poll(struct snd_sof_dev *sdev, u32 bar, u32 offset,
	u32 mask, u32 target, u32 timeout)
{
	int time, ret;
	bool done = false;

	/*
	 * we will poll for couple of ms using mdelay, if not successful
	 * then go to longer sleep using usleep_range
	 */

	/* check if set state successful */
	for (time = 0; time < 5; time++) {
		if ((snd_sof_dsp_read(sdev, bar, offset) & mask) == target) {
			done = true;
			break;
		}
		msleep(1);
	}

	if (done ==  false) {
		/* sleeping in 10ms steps so adjust timeout value */
		timeout /= 10;

		for (time = 0; time < timeout; time++) {
			if ((snd_sof_dsp_read(sdev, bar, offset) & mask) == target)
				break;

			usleep_range(5000, 10000);
		}
	}

	//reg = snd_sof_dsp_read(sdev, offset);
	ret = time < timeout ? 0 : -ETIME;

	return ret;
}
EXPORT_SYMBOL(snd_sof_dsp_register_poll);

