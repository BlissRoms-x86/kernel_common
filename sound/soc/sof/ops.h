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

#ifndef __SOUND_SOC_SOF_IO_H
#define __SOUND_SOC_SOF_IO_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"

/* init */
static inline int snd_sof_probe(struct snd_sof_dev *sdev)
{
	if (sdev->ops->probe)
		return sdev->ops->probe(sdev);
	else
		return 0;
}

static inline int snd_sof_remove(struct snd_sof_dev *sdev)
{
	if (sdev->ops->remove)
		return sdev->ops->remove(sdev);
	else
		return 0;
}

/* control */
static inline int snd_sof_dsp_run(struct snd_sof_dev *sdev)
{
	if (sdev->ops->run)
		return sdev->ops->run(sdev);
	else
		return 0;
}

static inline int snd_sof_dsp_stall(struct snd_sof_dev *sdev)
{
	if (sdev->ops->stall)
		return sdev->ops->stall(sdev);
	else
		return 0;
}

static inline int snd_sof_dsp_reset(struct snd_sof_dev *sdev)
{
	if (sdev->ops->reset)
		return sdev->ops->reset(sdev);
	else
		return 0;
}

/* power management */
static inline int snd_sof_dsp_resume(struct snd_sof_dev *sdev)
{
	if (sdev->ops->resume)
		return sdev->ops->resume(sdev);
	else
		return 0;
}

static inline int snd_sof_dsp_suspend(struct snd_sof_dev *sdev, int state)
{
	if (sdev->ops->suspend)
		return sdev->ops->suspend(sdev, state);
	else
		return 0;
}

static inline int snd_sof_dsp_set_clk(struct snd_sof_dev *sdev, u32 freq)
{
	if (sdev->ops->set_clk)
		return sdev->ops->set_clk(sdev, freq);
	else
		return 0;
}

/* debug */
static inline void snd_sof_dsp_dbg_dump(struct snd_sof_dev *sdev, u32 flags)
{
	if (sdev->ops->dbg_dump)
		return sdev->ops->dbg_dump(sdev, flags);
}

/* register IO */
static inline void snd_sof_dsp_write(struct snd_sof_dev *sdev, u32 bar,
	u32 offset, u32 value)
{
	if (sdev->ops->write)
		sdev->ops->write(sdev, sdev->bar[bar] + offset, value);
}

static inline void snd_sof_dsp_write64(struct snd_sof_dev *sdev, u32 bar,
	u32 offset, u64 value)
{
	if (sdev->ops->write64)
		sdev->ops->write64(sdev, 
			sdev->bar[bar] + offset, value);
}

static inline u32 snd_sof_dsp_read(struct snd_sof_dev *sdev, u32 bar,
	u32 offset)
{
	if (sdev->ops->read)
		return sdev->ops->read(sdev, sdev->bar[bar] + offset);
	else
		return 0;
}

static inline u64 snd_sof_dsp_read64(struct snd_sof_dev *sdev, u32 bar,
	u32 offset)
{
	if (sdev->ops->read64)
		return sdev->ops->read64(sdev, sdev->bar[bar] + offset);
	else
		return 0;
}

/* block IO */
static inline void snd_sof_dsp_block_read(struct snd_sof_dev *sdev,
	u32 offset, void *dest, size_t bytes)
{
	if (sdev->ops->block_read)
		sdev->ops->block_read(sdev, offset, dest, bytes);
}

static inline void snd_sof_dsp_block_write(struct snd_sof_dev *sdev,
	u32 offset, void *src, size_t bytes)
{
	if (sdev->ops->block_write)
		sdev->ops->block_write(sdev, offset, src, bytes);
}

/* mailbox */
static inline void snd_sof_dsp_mailbox_read(struct snd_sof_dev *sdev,
	u32 offset, void *message, size_t bytes)
{
	if (sdev->ops->mailbox_read)
		sdev->ops->mailbox_read(sdev, offset, message, bytes);
}

static inline void snd_sof_dsp_mailbox_write(struct snd_sof_dev *sdev,
	u32 offset, void *message, size_t bytes)
{
	if (sdev->ops->mailbox_write)
		sdev->ops->mailbox_write(sdev, offset, message, bytes);
}

/* ipc */
static inline int snd_sof_dsp_tx_msg(struct snd_sof_dev *sdev,
	struct snd_sof_ipc_msg *msg)
{
	if (sdev->ops->tx_msg)
		return sdev->ops->tx_msg(sdev, msg);
	else
		return 0;
}

static inline int snd_sof_dsp_rx_msg(struct snd_sof_dev *sdev,
	struct snd_sof_ipc_msg *msg)
{
	if (sdev->ops->rx_msg)
		return sdev->ops->rx_msg(sdev, msg);
	else
		return 0;
}

int snd_sof_dsp_update_bits_unlocked(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u32 mask, u32 value);

int snd_sof_dsp_update_bits64_unlocked(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u64 mask, u64 value);

/* This is for registers bits with attribute RWC */
void snd_sof_dsp_update_bits_forced_unlocked(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u32 mask, u32 value);

int snd_sof_dsp_update_bits(struct snd_sof_dev *sdev, u32 bar, u32 offset,
		u32 mask, u32 value);

int snd_sof_dsp_update_bits64(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u64 mask, u64 value);

/* This is for registers bits with attribute RWC */
void snd_sof_dsp_update_bits_forced(struct snd_sof_dev *sdev, u32 bar,
		u32 offset, u32 mask, u32 value);

int snd_sof_pci_update_bits_unlocked(struct snd_sof_dev *sdev, u32 offset,
		u32 mask, u32 value);

int snd_sof_pci_update_bits(struct snd_sof_dev *sdev, u32 offset,
		u32 mask, u32 value);

int snd_sof_dsp_register_poll(struct snd_sof_dev *sdev, u32 bar, u32 offset,
	u32 mask, u32 target, u32 timeout);

#endif
