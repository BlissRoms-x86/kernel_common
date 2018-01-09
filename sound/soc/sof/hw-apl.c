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
 *
 * Authors: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 *	    Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 *	    Jeeja KP <jeeja.kp@intel.com>
 */

/*
 * Hardware interface for audio DSP on Apollolake.
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <sound/hdaudio_ext.h>
#include <sound/sof.h>
#include <linux/pm_runtime.h>

#include "sof-priv.h"
#include "ops.h"
#include "intel.h"

#define APL_HDA_BAR			0
#define APL_PP_BAR			1
#define APL_SPIB_BAR			2
#define APL_DRSM_BAR			3
#define APL_DSP_BAR			4

#define SRAM_WINDOW_OFFSET(x)		(0x80000 + x * 0x20000)

#define APL_MBOX_OFFSET 		SRAM_WINDOW_OFFSET(0)
#define APL_MBOX_DUMP_SIZE 		0x50

#define APL_MBOX_UPLINK_OFFSET		0x81000

#define APL_BDL_ARRAY_ADDR_L		0
#define APL_BDL_ARRAY_ADDR_U		1
#define APL_BDL_ARRAY_SIZE		2
#define APL_BDL_ARRAY_IOC		3
#define APL_BDL_SIZE               	4096
#define APL_MAX_BDL_ENTRIES		(APL_BDL_SIZE / 16)

#define APL_STREAM_RESET_TIMEOUT	300
#define APL_CL_TRIGGER_TIMEOUT		300

#define APL_SPIB_ENABLE			1
#define APL_SPIB_DISABLE		0


#define SOF_HDA_MAX_BUFFER_SIZE		(32 * PAGE_SIZE)

/* various timeout values */
#define APL_DSP_PU_TIMEOUT		50
#define APL_DSP_PD_TIMEOUT		50
#define APL_DSP_RESET_TIMEOUT		50
#define APL_BASEFW_TIMEOUT		3000
#define APL_INIT_TIMEOUT		500
#define APL_CTRL_RESET_TIMEOUT		100
#define APL_WAIT_TIMEOUT		500	/* 500 msec */

/* Firmware status window */
#define APL_ADSP_FW_STATUS		SRAM_WINDOW_OFFSET(0)
#define APL_ADSP_ERROR_CODE     	(APL_ADSP_FW_STATUS + 0x4)

/* ROM status IDs */
#define APL_ROM_STS_MASK		0xf
#define APL_ROM_INIT				0x1
#define APL_IPC_PURGE_FW		0x01004000
#define APL_FW_VERIFIED			0x5
#define APL_ROM_RFW_START		0xf

#define APL_ADSPIC_IPC			1
#define APL_ADSPIS_IPC			1

/* Intel HD Audio General DSP Registers */
#define APL_DSP_GEN_BASE		0x0
#define APL_DSP_REG_ADSPCS		(APL_DSP_GEN_BASE + 0x04)
#define APL_DSP_REG_ADSPIC		(APL_DSP_GEN_BASE + 0x08)
#define APL_DSP_REG_ADSPIS		(APL_DSP_GEN_BASE + 0x0C)
#define APL_DSP_REG_ADSPIC2		(APL_DSP_GEN_BASE + 0x10)
#define APL_DSP_REG_ADSPIS2		(APL_DSP_GEN_BASE + 0x14)

/* Intel HD Audio Inter-Processor Communication Registers */
#define APL_DSP_IPC_BASE		0x40
#define APL_DSP_REG_HIPCT		(APL_DSP_IPC_BASE + 0x00)
#define APL_DSP_REG_HIPCTE		(APL_DSP_IPC_BASE + 0x04)
#define APL_DSP_REG_HIPCI		(APL_DSP_IPC_BASE + 0x08)
#define APL_DSP_REG_HIPCIE		(APL_DSP_IPC_BASE + 0x0C)
#define APL_DSP_REG_HIPCCTL		(APL_DSP_IPC_BASE + 0x10)

/*  HIPCI */
#define APL_DSP_REG_HIPCI_BUSY		BIT(31)
#define APL_DSP_REG_HIPCI_MSG_MASK	0x7FFFFFFF

/* HIPCIE */
#define APL_DSP_REG_HIPCIE_DONE	BIT(30)
#define APL_DSP_REG_HIPCIE_MSG_MASK	0x3FFFFFFF

/* HIPCCTL */
#define APL_DSP_REG_HIPCCTL_DONE	BIT(1)
#define APL_DSP_REG_HIPCCTL_BUSY	BIT(0)

/* HIPCT */
#define APL_DSP_REG_HIPCT_BUSY		BIT(31)
#define APL_DSP_REG_HIPCT_MSG_MASK	0x7FFFFFFF

/* HIPCTE */
#define APL_DSP_REG_HIPCTE_MSG_MASK	0x3FFFFFFF

#define APL_ADSPIC_CL_DMA		0x2
#define APL_ADSPIS_CL_DMA		0x2

/* Delay before scheduling D0i3 entry */
#define BXT_D0I3_DELAY 5000


#define FW_CL_STREAM_NUMBER		0x1

/* ADSPCS - Audio DSP Control & Status */

/*
 * Core Reset - asserted high
 * CRST Mask for a given core mask pattern, cm
 */
#define APL_ADSPCS_CRST_SHIFT		0
#define APL_ADSPCS_CRST_MASK(cm)	((cm) << APL_ADSPCS_CRST_SHIFT)

/*
 * Core run/stall - when set to '1' core is stalled
 * CSTALL Mask for a given core mask pattern, cm
 */
#define APL_ADSPCS_CSTALL_SHIFT		8
#define APL_ADSPCS_CSTALL_MASK(cm)	((cm) << APL_ADSPCS_CSTALL_SHIFT)

/*
 * Set Power Active - when set to '1' turn cores on
 * SPA Mask for a given core mask pattern, cm
 */
#define APL_ADSPCS_SPA_SHIFT		16
#define APL_ADSPCS_SPA_MASK(cm)		((cm) << APL_ADSPCS_SPA_SHIFT)

/*
 * Current Power Active - power status of cores, set by hardware
 * CPA Mask for a given core mask pattern, cm
 */
#define APL_ADSPCS_CPA_SHIFT		24
#define APL_ADSPCS_CPA_MASK(cm)		((cm) << APL_ADSPCS_CPA_SHIFT)


#define APL_ADSPIC_CL_DMA		0x2
#define APL_ADSPIS_CL_DMA		0x2

/* Mask for a given core index, c = 0.. number of supported cores - 1 */
#define APL_DSP_CORE_MASK(c)		BIT(c)

/*
 * Mask for a given number of cores
 * nc = number of supported cores
 */
#define SOF_DSP_CORES_MASK(nc)	GENMASK((nc - 1), 0)


static bool is_apl_core_enable(struct snd_sof_dev *sdev,
	unsigned int core_mask);

/*
 * Debug
 */

static const struct snd_sof_debugfs_map apl_debugfs[] = {
	{"hda", APL_HDA_BAR, 0, 0x4000},
	{"pp", APL_PP_BAR,  0, 0x1000},
	{"dsp", APL_DSP_BAR,  0, 0x10000},
//	{"pci", APL_PCI_BAR, 0, 0x150},
};

static void apl_dump(struct snd_sof_dev *sdev, u32 flags)
{
	u32 reg;
	int i;

	if (flags & SOF_DBG_REGS) {
		for (i = 0; i < 0x120; i += 4 ) {
			dev_dbg(sdev->dev, "hda 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, APL_HDA_BAR, i));
		}
		for (i = 0; i < 0xc; i += 4 ) {
			dev_dbg(sdev->dev, "spib 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, APL_SPIB_BAR, i));
		}
		for (i = 0; i < 0x40; i += 4 ) {
			dev_dbg(sdev->dev, "dsp 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, APL_DSP_BAR, i));
		}
		for (i = 0; i < 0x40; i += 4 ) {
			dev_dbg(sdev->dev, "pp 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, APL_PP_BAR, i));
		}
	}

	if (flags & SOF_DBG_MBOX) {
		for (i = 0; i < APL_MBOX_DUMP_SIZE; i += 4) {
			dev_dbg(sdev->dev, "regs: 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, 
					APL_DSP_BAR, i + APL_MBOX_OFFSET));
		}
		for (i = 0; i < APL_MBOX_DUMP_SIZE; i += 4) {
			dev_dbg(sdev->dev, "inbox: 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, 
					APL_DSP_BAR, i + APL_MBOX_OFFSET + 1000));
		}
		for (i = 0; i < APL_MBOX_DUMP_SIZE; i += 4) {
			dev_dbg(sdev->dev, "outbox: 0x%2.2x value 0x%8.8x\n",
				i, snd_sof_dsp_read(sdev, 
					APL_DSP_BAR, i + SRAM_WINDOW_OFFSET(1)));
		}
	}

	if (flags & SOF_DBG_PCI) {
		for (i = 0; i < 0x150; i += 4) {
			pci_read_config_dword(sdev->pci, i, &reg);
			dev_dbg(sdev->dev, "pci: 0x%2.2x value 0x%8.8x\n",
				i, reg);
		}
	}
}

/*
 * Register IO
 */

static void apl_write(struct snd_sof_dev *sdev, void __iomem *addr,
	u32 value)
{
	writel(value, addr);
}

static u32 apl_read(struct snd_sof_dev *sdev, void __iomem *addr)
{
	return readl(addr);
}

static void apl_write64(struct snd_sof_dev *sdev, void __iomem *addr,
	u64 value)
{
	memcpy_toio(addr, &value, sizeof(value));
}

static u64 apl_read64(struct snd_sof_dev *sdev, void __iomem *addr)
{
	u64 val;

	memcpy_fromio(&val, addr, sizeof(val));
	return val;
}

/*
 * Memory copy.
 */

static void apl_block_write(struct snd_sof_dev *sdev, u32 offset, void *src,
	size_t size)
{
	volatile void __iomem *dest = sdev->bar[sdev->mmio_bar] + offset;
	u32 tmp = 0;
	int i, m, n;
	const u8 *src_byte = src;

	m = size / 4;
	n = size % 4;

	/* __iowrite32_copy use 32bit size values so divide by 4 */
	__iowrite32_copy((void *)dest, src, m);

	if (n) {
		for (i = 0; i < n; i++)
			tmp |= (u32)*(src_byte + m * 4 + i) << (i * 8);
		__iowrite32_copy((void *)(dest + m * 4), &tmp, 1);
	}
}

static void apl_block_read(struct snd_sof_dev *sdev, u32 offset, void *dest,
	size_t size)
{
	volatile void __iomem *src = sdev->bar[sdev->mmio_bar] + offset;
	memcpy_fromio(dest, src, size);
}

/*
 * IPC Mailbox IO
 */

static void apl_mailbox_write(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *dest = sdev->bar[sdev->mailbox_bar] + offset;

	memcpy_toio(dest, message, bytes);
}

static void apl_mailbox_read(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *src = sdev->bar[sdev->mailbox_bar] + offset;

	memcpy_fromio(message, src, bytes);
}

/*
 * Code loader
 */

static int apl_spib_config(struct snd_sof_dev *sdev, 
	struct snd_sof_hda_stream *stream, int enable, u32 size)
{
	u32 mask = 0;

	if (sdev->bar[APL_SPIB_BAR] == NULL) {
		dev_err(sdev->dev, "error: address of spb capability is NULL\n");
		return -EINVAL; 
	}
	
	mask |= (1 << stream->index);
	
	/* enable/disable SPIB for the stream */
	snd_sof_dsp_update_bits(sdev, APL_SPIB_BAR,
		SOF_HDA_ADSP_REG_CL_SPBFIFO_SPBFCCTL, mask, 
		enable << stream->index);
				
	/* set the SPIB value */			
	apl_write(sdev, stream->spib_addr, size);

	return 0;
}

static int apl_dsp_cleanup(struct snd_sof_dev *sdev,
	struct snd_dma_buffer *dmab, struct snd_sof_hda_stream *stream)
{
	int ret;

	ret = apl_spib_config(sdev, stream, APL_SPIB_DISABLE, 0);
	/* TODO: spin lock ?*/
	stream->open = 0;
	stream->running = 0;
	stream->substream = NULL;
	
	/* reset BDL address */
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPL, 0);
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPU, 0);

	snd_sof_dsp_write(sdev, APL_HDA_BAR, stream->sd_offset, 0);
	snd_dma_free_pages(dmab);
	dmab->area = NULL;
	stream->bufsize = 0;
	stream->format_val = 0;
	
	return ret;
}

static int apl_cl_trigger(struct snd_sof_dev *sdev, 
	struct snd_sof_hda_stream *stream)
{
	wait_event_timeout(sdev->waitq, !sdev->code_loading,
		APL_CL_TRIGGER_TIMEOUT);
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_INTCTL,
		1 << stream->index, 1 << stream->index);

	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
		SOF_HDA_SD_CTL_DMA_START | SOF_HDA_CL_DMA_SD_INT_MASK, 
		SOF_HDA_SD_CTL_DMA_START | SOF_HDA_CL_DMA_SD_INT_MASK);

	stream->running = true;
	return 0;
}
 
static int apl_trigger(struct snd_sof_dev *sdev, 
	struct snd_sof_hda_stream *stream, int cmd)
{	
	int ret = 0;

	/* code loader is special case that reuses stream ops */
	if (sdev->code_loading && cmd == SNDRV_PCM_TRIGGER_START)
		return apl_cl_trigger(sdev, stream);

	/* cmd must be for audio stream */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_START:
		snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_INTCTL,
			1 << stream->index, 1 << stream->index);

		snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
			SOF_HDA_SD_CTL_DMA_START | SOF_HDA_CL_DMA_SD_INT_MASK, 
			SOF_HDA_SD_CTL_DMA_START | SOF_HDA_CL_DMA_SD_INT_MASK);

		stream->running = true;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
			SOF_HDA_SD_CTL_DMA_START | SOF_HDA_CL_DMA_SD_INT_MASK, 0x0);

		snd_sof_dsp_write(sdev, APL_HDA_BAR, stream->sd_offset + 
			SOF_HDA_ADSP_REG_CL_SD_STS, SOF_HDA_CL_DMA_SD_INT_MASK); 

		stream->running = false; 
		snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_INTCTL,
			1 << stream->index, 0x0);
		break;
	default:
		dev_err(sdev->dev, "error: unknown command: %d\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int apl_transfer_fw(struct snd_sof_dev *sdev, int stream_tag)
{
	struct snd_sof_hda_stream *stream = NULL;
	struct snd_sof_hda_dev *hdev = &sdev->hda;
	int ret, status, i;

	/* get stream with stream_tag */
	for (i = 0; i < hdev->num_playback; i++) {
		if (hdev->pstream[i].stream_tag == stream_tag) {
			stream = &hdev->pstream[i];
			break;
		}
	}
	if (stream == NULL) {
		dev_err(sdev->dev, 
			"error: could not get stream with stream tag %d\n", 
			stream_tag);
		return -ENODEV;
	}

	ret = apl_trigger(sdev, stream, SNDRV_PCM_TRIGGER_START);
	if (ret < 0) {
		dev_err(sdev->dev, "error: DMA trigger start failed \n");
		return ret;
	}

	status = snd_sof_dsp_register_poll(sdev, APL_DSP_BAR, 
			APL_ADSP_FW_STATUS, APL_ROM_STS_MASK, APL_FW_VERIFIED,
			APL_BASEFW_TIMEOUT);

	ret = apl_trigger(sdev, stream, SNDRV_PCM_TRIGGER_STOP);
	if (ret < 0) {
		dev_err(sdev->dev, "error: DMA trigger stop failed \n");
		return ret;
	}

	ret = apl_dsp_cleanup(sdev, &sdev->dmab, stream);
	if (ret < 0) {
		dev_err(sdev->dev, "error: DSP cleanup failed \n");
		return ret;
	}

	return status;
}

/*
 * set up Buffer Descriptor List (BDL) for host memory transfer
 * BDL describes the location of the individual buffers and is little endian.
 */
static int apl_cl_setup_bdl(struct snd_sof_dev *sdev, 
	struct snd_dma_buffer *dmab,
	struct snd_sof_hda_stream *stream, __le32 **bdlp,
	int size)
{
	__le32 *bdl = *bdlp;
	int offset = 0;

	while (size > 0) {
		dma_addr_t addr;
		int chunk;

		if (stream->frags >= APL_MAX_BDL_ENTRIES) {
			dev_err(sdev->dev, "error: stream frags exceeded\n");
			return -EINVAL;
		}

		addr = snd_sgbuf_get_addr(dmab, offset);

		/* program BDL addr */
		bdl[APL_BDL_ARRAY_ADDR_L] = cpu_to_le32(lower_32_bits(addr)); 
		bdl[APL_BDL_ARRAY_ADDR_U] = cpu_to_le32(upper_32_bits(addr));

		/* program BDL size */
		chunk = snd_sgbuf_get_chunk_size(dmab, offset, size);	

		bdl[APL_BDL_ARRAY_SIZE] = cpu_to_le32(chunk);

		/* program the IOC to enable interrupt
		 * when the whole fragment is processed
		 */
		size -= chunk;
		if (size)
			bdl[APL_BDL_ARRAY_IOC] = 0;
		else
			bdl[APL_BDL_ARRAY_IOC] = cpu_to_le32(0x01);
		bdl += 4;
		stream->frags++;
		offset += chunk;
	}

	*bdlp = bdl;
	return offset;
}

static int apl_prepare(struct snd_sof_dev *sdev, unsigned int format,
	unsigned int size, struct snd_dma_buffer *dmab, int direction)
{
	struct snd_sof_hda_stream *stream = NULL;
	struct snd_sof_hda_dev *hdev = &sdev->hda;
	struct pci_dev *pci = sdev->pci;
	int ret, timeout = APL_STREAM_RESET_TIMEOUT, i;
	u32 val, mask;
	u32 *bdl;

	if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
		/* get an unused playback stream */
		for (i = 0; i < hdev->num_playback; i++) {
			if (!hdev->pstream[i].open) {
				hdev->pstream[i].open = true;
				stream = &hdev->pstream[i];
				break;
			}
		}
	} else {
		/* get an unused capture stream */
		for (i = 0; i < hdev->num_capture; i++) {
			if (!hdev->cstream[i].open) {
				hdev->cstream[i].open = true;
				stream = &hdev->cstream[i];
				break;
			}
		}
	}

	if (stream == NULL) {
		dev_err(sdev->dev, "error: no stream available\n");
		return -ENODEV;
	}
	
	/* decouple host and link DMA */
	mask = 0x1 << stream->index;
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL, mask, mask);

	/* allocate DMA buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_SG, &pci->dev, size, dmab);
	if (ret < 0) {
		dev_err(sdev->dev, "error: memory alloc failed: %x\n", ret);
		return ret;
	}

	stream->format_val = format;
	stream->bufsize = size;

	/* clear stream status */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
		SOF_HDA_CL_DMA_SD_INT_MASK | SOF_HDA_SD_CTL_DMA_START, 0);
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR,
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS,
	 	SOF_HDA_CL_DMA_SD_INT_MASK, SOF_HDA_CL_DMA_SD_INT_MASK);

	/* stream reset */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset, 0x1, 0x1);
	udelay(3);
	do {
		val = snd_sof_dsp_read(sdev, APL_HDA_BAR, stream->sd_offset);
		if (val & 0x1)
			break;
	} while (--timeout);
	if (timeout == 0) {
		dev_err(sdev->dev, "error: stream reset failed\n");
		return -EINVAL;
	}	

	timeout = APL_STREAM_RESET_TIMEOUT;
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset, 0x1, 0x0);

	/* wait for hardware to report that stream is out of reset */
	udelay(3);
	do {
		val = snd_sof_dsp_read(sdev, APL_HDA_BAR, stream->sd_offset);
		if ((val & 0x1) == 0) {
			break;
		}
	} while (--timeout);
	if (timeout == 0) {
		dev_err(sdev->dev, "error: timeout waiting for stream reset\n");
		return -EINVAL;
	}

	if (stream->posbuf)
		*stream->posbuf = 0;

	/* reset BDL address */
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPL, 0x0);
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPU, 0x0);
			
	/* clear stream status */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
	 	SOF_HDA_CL_DMA_SD_INT_MASK | SOF_HDA_SD_CTL_DMA_START, 0);
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS,
	 	SOF_HDA_CL_DMA_SD_INT_MASK, SOF_HDA_CL_DMA_SD_INT_MASK); 

	stream->frags = 0;		

	bdl = (u32 *)stream->bdl.area;
	ret = apl_cl_setup_bdl(sdev, dmab, stream, &bdl, size);
	if (ret < 0) {
		dev_dbg(sdev->dev, "error: set up bdl fail\n");
		goto error;
	}

	/* set up stream descriptor for DMA */	
	/* program stream tag */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset,
		SOF_HDA_CL_SD_CTL_STREAM_TAG_MASK, 
		stream->stream_tag << SOF_HDA_CL_SD_CTL_STREAM_TAG_SHIFT);
			
	/* program cyclic buffer length */
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_CBL, stream->bufsize);

	/* program stream format */ 
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_FORMAT, 
		0xffff, stream->format_val);
			
	/* program last valid index */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_LVI, 
		0xffff, (stream->frags - 1));
			
	/* program BDL address */
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPL, 
		(u32)stream->bdl.addr);
	snd_sof_dsp_write(sdev, APL_HDA_BAR, 
		stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_BDLPU,
		upper_32_bits(stream->bdl.addr));

	/* enable position buffer */
	if (!(snd_sof_dsp_read(sdev, APL_HDA_BAR, 
		SOF_HDA_ADSP_DPLBASE) & SOF_HDA_ADSP_DPLBASE_ENABLE))
		snd_sof_dsp_write(sdev, APL_HDA_BAR, SOF_HDA_ADSP_DPLBASE, 
			(u32)hdev->posbuffer.addr | SOF_HDA_ADSP_DPLBASE_ENABLE);
		
	/* set interrupt enable bits */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, stream->sd_offset, 
		SOF_HDA_CL_DMA_SD_INT_MASK, SOF_HDA_CL_DMA_SD_INT_MASK);

	/* read FIFO size */
	if (stream->direction == SNDRV_PCM_STREAM_PLAYBACK) {
		stream->fifo_size = snd_sof_dsp_read(sdev, APL_HDA_BAR, 
			stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_FIFOSIZE)
			& 0xffff;
		stream->fifo_size += 1;
	} else
		stream->fifo_size = 0;

	apl_spib_config(sdev, stream, APL_SPIB_ENABLE, size);

	return stream->stream_tag;

error:
	snd_dma_free_pages(dmab);
	return ret;
}

/*
 * IPC Firmware ready.
 */

static void apl_get_windows(struct snd_sof_dev *sdev)
{
	struct sof_ipc_window_elem *elem;
	int i;	
	u32 inbox_offset = 0, outbox_offset = 0;
	u32 inbox_size = 0, outbox_size = 0;

	if (sdev->info_window == NULL)
		return;

	for (i = 0; i < sdev->info_window->num_windows; i++) {

		elem = &sdev->info_window->window[i];

		switch (elem->type) {
		case SOF_IPC_REGION_UPBOX:
			inbox_offset =
				elem->offset + SRAM_WINDOW_OFFSET(elem->id);
			inbox_size = elem->size;
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + inbox_offset,
				elem->size, "inbox");
			break;
		case SOF_IPC_REGION_DOWNBOX:
			outbox_offset =
				elem->offset + SRAM_WINDOW_OFFSET(elem->id);
			outbox_size = elem->size;
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + outbox_offset,
				elem->size, "outbox");
			break;
		case SOF_IPC_REGION_TRACE:
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + elem->offset +
				SRAM_WINDOW_OFFSET(elem->id),
				elem->size, "trace");
			break;
		case SOF_IPC_REGION_DEBUG:
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + elem->offset +
				SRAM_WINDOW_OFFSET(elem->id),
				elem->size, "debug");
			break;
		case SOF_IPC_REGION_STREAM:
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + elem->offset +
				SRAM_WINDOW_OFFSET(elem->id),
				elem->size, "stream");
			break;
		case SOF_IPC_REGION_REGS:
			snd_sof_debugfs_create_item(sdev,
				sdev->bar[APL_DSP_BAR] + elem->offset +
				SRAM_WINDOW_OFFSET(elem->id),
				elem->size, "regs");
			break;	
		default:
			break;
		}
	}

	snd_sof_dsp_mailbox_init(sdev, inbox_offset, inbox_size, 
		outbox_offset, outbox_size);

	dev_dbg(sdev->dev, " mailbox upstream 0x%x - size 0x%x\n",
		inbox_offset, inbox_size);
	dev_dbg(sdev->dev, " mailbox downstream 0x%x - size 0x%x\n",
		outbox_offset, outbox_size);
}

static int apl_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct sof_ipc_fw_ready *fw_ready = &sdev->fw_ready;
	struct sof_ipc_fw_version *v = &fw_ready->version;
	u32 offset;

	/* mailbox must be on 4k boundary */
	offset = APL_MBOX_UPLINK_OFFSET;

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x offset 0x%x\n",
		msg_id, offset);

	/* copy data from the DSP FW ready offset */
	apl_block_read(sdev, offset, fw_ready,	sizeof(*fw_ready));
	dev_info(sdev->dev, " Firmware info: version %d:%d-%s build %d on %s:%s\n", 
		v->major, v->minor, v->tag, v->build, v->date, v->time);

	/* now check for extended data */
	snd_sof_fw_parse_ext_data(sdev,
		APL_MBOX_UPLINK_OFFSET + sizeof(struct sof_ipc_fw_ready));

	apl_get_windows(sdev);

	return 0;
}

/*
 * IPC Doorbell IRQ handler and thread.
 */

static irqreturn_t apl_irq_handler(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	int ret = IRQ_NONE;

	spin_lock(&sdev->spinlock);

	/* store status */
	sdev->irq_status = snd_sof_dsp_read(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPIS);

	/* invalid message ? */
	if (sdev->irq_status == 0xffffffff)
		goto out;

	/* IPC message ? */
	if (sdev->irq_status & APL_ADSPIS_IPC) {
		/* disable IPC interrupt */
		snd_sof_dsp_update_bits_unlocked(sdev, APL_DSP_BAR,
			APL_DSP_REG_ADSPIC, APL_ADSPIC_IPC, 0);
		ret = IRQ_WAKE_THREAD;
	}

out:
	spin_unlock(&sdev->spinlock);
	return ret;
}

static irqreturn_t apl_irq_thread(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	u32 hipci, hipcie, hipct, hipcte, msg = 0, msg_ext = 0;
	irqreturn_t ret = IRQ_NONE;

	/* here we handle IPC interrupts only */
	if (!(sdev->irq_status & APL_ADSPIS_IPC))
		return ret;

	hipcie = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCIE);
	hipct = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCT);

	/* reply message from DSP */
	if (hipcie & APL_DSP_REG_HIPCIE_DONE) {

		hipci = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCI);
		msg = hipci & APL_DSP_REG_HIPCI_MSG_MASK;
		msg_ext = hipcie & APL_DSP_REG_HIPCIE_MSG_MASK;
		dev_dbg(sdev->dev, "ipc: firmware response, msg:0x%x, msg_ext:0x%x\n",
			msg, msg_ext);

		/* mask Done interrupt */
		snd_sof_dsp_update_bits(sdev, APL_DSP_BAR,
			APL_DSP_REG_HIPCCTL, APL_DSP_REG_HIPCCTL_DONE, 0);

		/* handle immediate reply from DSP core */
		snd_sof_ipc_process_reply(sdev, msg);

		/* clear DONE bit - tell DSP we have completed the operation */
		snd_sof_dsp_update_bits_forced(sdev, APL_DSP_BAR, 
			APL_DSP_REG_HIPCIE, APL_DSP_REG_HIPCIE_DONE, 
			APL_DSP_REG_HIPCIE_DONE);

		/* unmask Done interrupt */
		snd_sof_dsp_update_bits(sdev, APL_DSP_BAR,
			APL_DSP_REG_HIPCCTL, APL_DSP_REG_HIPCCTL_DONE,
			APL_DSP_REG_HIPCCTL_DONE);

		ret = IRQ_HANDLED;
	}

	/* new message from DSP */
	if (hipct & APL_DSP_REG_HIPCT_BUSY) {

		hipcte = snd_sof_dsp_read(sdev, APL_DSP_BAR,
			APL_DSP_REG_HIPCTE);
		msg = hipct & APL_DSP_REG_HIPCT_MSG_MASK;
		msg_ext = hipcte & APL_DSP_REG_HIPCTE_MSG_MASK;

		dev_dbg(sdev->dev, "ipc: firmware initiated, msg:0x%x, msg_ext:0x%x\n",
			msg, msg_ext);

		snd_sof_ipc_process_notification(sdev, msg);

		/* clear busy interrupt */
		snd_sof_dsp_update_bits_forced(sdev, APL_DSP_BAR, 
			APL_DSP_REG_HIPCT, APL_DSP_REG_HIPCT_BUSY, 
			APL_DSP_REG_HIPCT_BUSY);

		ret = IRQ_HANDLED;
	}

	if (ret == IRQ_HANDLED) {
		/* reenable IPC interrupt */
		snd_sof_dsp_update_bits(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPIC,
			APL_ADSPIC_IPC, APL_ADSPIC_IPC);
		/* continue to send any remaining messages... */
		snd_sof_ipc_process_msgs(sdev);
	}

	if (sdev->code_loading)	{
		sdev->code_loading = 0;
		wake_up(&sdev->waitq);
	}

	return ret;
}

static irqreturn_t skl_interrupt(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	u32 status;

	if (!pm_runtime_active(sdev->dev))
		return IRQ_NONE;

	status = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_INTSTS);

	if (status == 0 || status == 0xffffffff) {
		return IRQ_NONE;
	}

	return status ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t skl_threaded_handler(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	struct snd_sof_hda_dev *hdev = &sdev->hda;
	u32 status = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_INTSTS);
	u32 sd_status;
	int i;

	/* check playback streams */
	for (i = 0; i < hdev->num_playback; i++) {

		/* is IRQ for this stream ? */
		if (status & (1 << hdev->pstream[i].index)) {

			sd_status = snd_sof_dsp_read(sdev, APL_HDA_BAR,
				hdev->pstream[i].sd_offset +
				SOF_HDA_ADSP_REG_CL_SD_STS) & 0xff;

			dev_dbg(sdev->dev, "pstream %d status 0x%x\n",
				i, sd_status);

			snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, 
				hdev->pstream[i].sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS, 
				SOF_HDA_CL_DMA_SD_INT_MASK,
				SOF_HDA_CL_DMA_SD_INT_MASK);

			if (hdev->pstream[i].substream == NULL ||
				hdev->pstream[i].running == false || 
				(sd_status & SOF_HDA_CL_DMA_SD_INT_MASK) == 0)
				continue;

			/* update buffer position to ALSA */
			snd_pcm_period_elapsed(hdev->pstream[i].substream);
		}
	
	}

	/* check capture streams */
	for (i = 0; i < hdev->num_capture; i++) {

		/* is IRQ for this stream ? */
		if (status & (1 << hdev->cstream[i].index)) {

			sd_status = snd_sof_dsp_read(sdev, APL_HDA_BAR,
				hdev->cstream[i].sd_offset +
				SOF_HDA_ADSP_REG_CL_SD_STS) & 0xff;

			dev_dbg(sdev->dev, "cstream %d status 0x%x\n",
				i, sd_status);

			snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, 
				hdev->cstream[i].sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS, 
				SOF_HDA_CL_DMA_SD_INT_MASK,
				SOF_HDA_CL_DMA_SD_INT_MASK);

			if (hdev->cstream[i].substream == NULL ||
				hdev->cstream[i].running == false || 
				(sd_status & SOF_HDA_CL_DMA_SD_INT_MASK) == 0)
				continue;

			/* update buffer position to ALSA */
			snd_pcm_period_elapsed(hdev->cstream[i].substream);
		}
	
	}

	return IRQ_HANDLED;
}

/*
 * DSP control.
 */

static int
apl_dsp_core_reset_enter(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	u32 adspcs;
	int ret;

	/* set reset bits for cores */
	snd_sof_dsp_update_bits_unlocked(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_CRST_MASK(core_mask),
		APL_ADSPCS_CRST_MASK(core_mask));

	/* poll with timeout to check if operation successful */
	ret = snd_sof_dsp_register_poll(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS,
		APL_ADSPCS_CRST_MASK(core_mask),
		APL_ADSPCS_CRST_MASK(core_mask), APL_DSP_RESET_TIMEOUT);

	adspcs = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS);
	if ((adspcs & APL_ADSPCS_CRST_MASK(core_mask)) !=
		APL_ADSPCS_CRST_MASK(core_mask)) {
		dev_err(sdev->dev, 
			"error: reset enter failed: core_mask %x adspcs 0x%x\n", 
			core_mask, adspcs);
		ret = -EIO;
	}

	return ret;
}

static int apl_dsp_core_reset_leave(struct snd_sof_dev *sdev,
	unsigned int core_mask)
{
	u32 adspcs;
	int ret;

	/* clear reset bits for cores */
	snd_sof_dsp_update_bits_unlocked(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_CRST_MASK(core_mask), 0);

	/* poll with timeout to check if operation successful */
	ret = snd_sof_dsp_register_poll(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS,
		APL_ADSPCS_CRST_MASK(core_mask), 0, APL_DSP_RESET_TIMEOUT);

	adspcs = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS);
	if ((adspcs & APL_ADSPCS_CRST_MASK(core_mask)) != 0) {
		dev_err(sdev->dev, 
			"error: reset leave failed: core_mask %x adspcs 0x%x\n",
			core_mask, adspcs);
		ret = -EIO;
	}

	return ret;
}

static int apl_reset_core(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	/* stall core */
	snd_sof_dsp_update_bits_unlocked(sdev, APL_HDA_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_CSTALL_MASK(core_mask),
		APL_ADSPCS_CSTALL_MASK(core_mask));

	/* set reset state */
	return apl_dsp_core_reset_enter(sdev, core_mask);
}

static int apl_run_core(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	int ret;

	/* leave reset state */
	ret = apl_dsp_core_reset_leave(sdev, core_mask);
	if (ret < 0)
		return ret;

	/* run core */
	dev_dbg(sdev->dev, "unstall/run core: core_mask = %x\n", core_mask);
	snd_sof_dsp_update_bits_unlocked(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_CSTALL_MASK(core_mask), 0);

	if (!is_apl_core_enable(sdev, core_mask)) {
		apl_reset_core(sdev, core_mask);
		dev_err(sdev->dev, "error: DSP start core failed: core_mask %x\n",
			core_mask);
		ret = -EIO;
	}

	return ret;
}

/*
 * Power Management.
 */

static int apl_core_power_up(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	u32 adspcs;
	int ret;

	/* update bits */
	snd_sof_dsp_update_bits(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS,
		APL_ADSPCS_SPA_MASK(core_mask),
		APL_ADSPCS_SPA_MASK(core_mask));

	/* poll with timeout to check if operation successful */
	ret = snd_sof_dsp_register_poll(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS,
		APL_ADSPCS_CPA_MASK(core_mask), APL_ADSPCS_CPA_MASK(core_mask),
		APL_DSP_PU_TIMEOUT);
	if (ret < 0)
		dev_err(sdev->dev, "error: timout on core powerup\n");

	/* did core power up ? */
	adspcs = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS);
	if ((adspcs & APL_ADSPCS_CPA_MASK(core_mask)) !=
		APL_ADSPCS_CPA_MASK(core_mask)) {
		dev_err(sdev->dev, 
			"error: power up core failed core_mask %xadspcs 0x%x\n",
			core_mask, adspcs);
		ret = -EIO;
	}

	return ret;
}

static int apl_core_power_down(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	/* update bits */
	snd_sof_dsp_update_bits_unlocked(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_SPA_MASK(core_mask), 0);

	/* poll with timeout to check if operation successful */
	return snd_sof_dsp_register_poll(sdev, APL_DSP_BAR,
		APL_DSP_REG_ADSPCS, APL_ADSPCS_CPA_MASK(core_mask), 0,
		APL_DSP_PD_TIMEOUT);
}

#if 0
static int apl_set_dsp_D0(struct snd_sof_dev *sdev, unsigned int core_id)
{
	unsigned int core_mask = APL_DSP_CORE_MASK(core_id);
	int ret = 0;

	/* core 1 also has to be powered on if core 0 is selected */
	if (core_id == 0)
		core_mask |= APL_DSP_CORE_MASK(1);

	/* power up the cores */
	ret = apl_core_power_up(sdev, core_mask);
	if (ret < 0)
		goto err;

	if (core_id == 0) {
		/* Enable interrupts after SPA is set and before unstall */
		apl_ipc_int_enable(sdev);
		apl_ipc_op_int_enable(sdev);

	}
err:
	return ret;
}
#endif

static bool is_apl_core_enable(struct snd_sof_dev *sdev,
	unsigned int core_mask)
{
	int val;
	bool is_enable;

	val = snd_sof_dsp_read(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPCS);

	is_enable = ((val & APL_ADSPCS_CPA_MASK(core_mask)) &&
			(val & APL_ADSPCS_SPA_MASK(core_mask)) &&
			!(val & APL_ADSPCS_CRST_MASK(core_mask)) &&
			!(val & APL_ADSPCS_CSTALL_MASK(core_mask)));

	dev_dbg(sdev->dev, "DSP core(s) enabled? %d : core_mask %x\n",
		is_enable, core_mask);

	return is_enable;
}

#if 0
static int apl_enable_core(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	int ret;

	/* power up */
	ret = apl_core_power_up(sdev, core_mask);
	if (ret < 0) {
		dev_err(sdev->dev, "dsp core power up failed: core_mask %x\n",
			core_mask);
		return ret;
	}

	return apl_run_core(sdev, core_mask);
}
#endif

static int apl_disable_core(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	int ret;

	/* place core in reset prior to power doown */
	ret = apl_reset_core(sdev, core_mask);
	if (ret < 0) {
		dev_err(sdev->dev, "error: dsp core reset failed: core_mask %x\n",
			core_mask);
		return ret;
	}

	/* power down core*/
	ret = apl_core_power_down(sdev, core_mask);
	if (ret < 0) {
		dev_err(sdev->dev, "error: dsp core power down fail mask %x: %d\n",
			core_mask, ret);
		return ret;
	}

	/* make sure we are in OFF state */
	if (is_apl_core_enable(sdev, core_mask)) {
		dev_err(sdev->dev, "error: dsp core disable fail mask %x: %d\n",
			core_mask, ret);
		ret = -EIO;
	}

	return ret;
}

#if 0
static int apl_set_dsp_D3(struct snd_sof_dev *sdev, unsigned int core_id)
{
	int ret;
	unsigned int core_mask = APL_DSP_CORE_MASK(core_id);

	ret = apl_disable_core(sdev, core_mask);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to enter D3 core %d\n", ret);
		return ret;
	}

	return 0;
}

static void apl_notify(struct snd_sof_dev *dsp)
{
	snd_sof_dsp_update_bits64(dsp, SHIM_IPCD,
		SHIM_BYT_IPCD_BUSY | SHIM_BYT_IPCD_DONE,
		SHIM_BYT_IPCD_DONE);
}

static bool apl_is_dsp_busy(struct snd_sof_dev *sdev)
{
	return 0;
}
#endif

static int apl_tx_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	u32 cmd = msg->header;

	/* send the message */
	apl_mailbox_write(sdev, sdev->outbox.offset, msg->msg_data,
		 msg->msg_size);
	snd_sof_dsp_write(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCI,
		cmd | APL_DSP_REG_HIPCI_BUSY);

	return 0;
}

/*
 * HDA Operations.
 */

static int apl_link_reset(struct snd_sof_dev *sdev)
{
	unsigned long timeout;
	u32 gctl = 0;

	/* reset the HDA controller */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_GCTL, SOF_HDA_GCTL_RESET, 0);

	/* wait for reset */
	timeout = jiffies + msecs_to_jiffies(APL_CTRL_RESET_TIMEOUT);
	while (time_before(jiffies, timeout)) {

		usleep_range(500, 1000);
		gctl = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_GCTL);
		if ((gctl & SOF_HDA_GCTL_RESET) == 0)
			goto clear;

	}

	/* reset failed */
	dev_err(sdev->dev, "error: failed to reset HDA controller gctl 0x%x\n",
		gctl);
	return -EIO;

clear:
	/* delay for >= 100us for codec PLL to settle per spec
	 * Rev 0.9 section 5.5.1
	 */		
	usleep_range(500, 1000);

	/* now take controller out of reset */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_GCTL, SOF_HDA_GCTL_RESET,
		SOF_HDA_GCTL_RESET);

	/* wait for controller to be ready */
	timeout = jiffies + msecs_to_jiffies(APL_CTRL_RESET_TIMEOUT);
	while (time_before(jiffies, timeout)) {
		gctl = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_GCTL);
		if ((gctl & SOF_HDA_GCTL_RESET) == 1)
			return 0;
		usleep_range(500, 1000);
	}

	/* reset failed */
	dev_err(sdev->dev, "error: failed to ready HDA controller gctl 0x%x\n",
		gctl);
	return -EIO;
}

static int apl_get_caps(struct snd_sof_dev *sdev)
{
	u32 cap, offset, feature;
	int ret = -ENODEV, count = 0;

	offset = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_LLCH);

	do {
		cap = snd_sof_dsp_read(sdev, APL_HDA_BAR, offset);

		dev_dbg(sdev->dev, "checking for capabilities at offset 0x%x\n",
			offset & SOF_HDA_CAP_NEXT_MASK);

		feature = (cap & SOF_HDA_CAP_ID_MASK) >> SOF_HDA_CAP_ID_OFF;

		switch (feature) {
		case SOF_HDA_PP_CAP_ID:
			dev_dbg(sdev->dev, "found DSP capability at 0x%x\n",
				offset);
			sdev->bar[APL_PP_BAR] = sdev->bar[APL_HDA_BAR] +
				offset;
			ret = 0;
			break;
		case SOF_HDA_SPIB_CAP_ID:
			dev_dbg(sdev->dev, "found SPIB capability at 0x%x\n",
				offset);
			sdev->bar[APL_SPIB_BAR] = sdev->bar[APL_HDA_BAR] +
				offset;
			break;
		case SOF_HDA_DRSM_CAP_ID:
			dev_dbg(sdev->dev, "found DRSM capability at 0x%x\n",
				offset);
			sdev->bar[APL_DRSM_BAR] = sdev->bar[APL_HDA_BAR] +
				offset;
			break;
		default:
			dev_vdbg(sdev->dev, "found capability %d at 0x%x\n",
				feature, offset);
			break;
		}		

		offset = cap & SOF_HDA_CAP_NEXT_MASK;
	} while (count++ <= SOF_HDA_MAX_CAPS && offset);

	return ret;
}

static int apl_stream_init(struct snd_sof_dev *sdev)
{
	struct snd_sof_hda_dev *hdev = &sdev->hda;
	struct snd_sof_hda_stream *stream;
	struct pci_dev *pci = sdev->pci;
	int i, num_playback, num_capture, num_total, ret;
	u32 gcap;

	gcap = snd_sof_dsp_read(sdev, APL_HDA_BAR, SOF_HDA_GCAP);
	dev_dbg(sdev->dev, "hda global caps = 0x%x\n", gcap);

	/* get stream count from GCAP */
	num_capture = (gcap >> 8) & 0x0f;
	num_playback = (gcap >> 12) & 0x0f;
	num_total = num_playback + num_capture;

	hdev->num_capture = num_capture;
	hdev->num_playback = num_playback;

	dev_dbg(sdev->dev, "detected %d playback and %d capture streams\n",
		num_playback, num_capture);

	if (num_playback >= SOF_HDA_PLAYBACK_STREAMS) {
		dev_err(sdev->dev, "error: too many playback streams %d\n",
			num_playback);
		return -EINVAL;
	}

	if (num_capture >= SOF_HDA_CAPTURE_STREAMS) {
		dev_err(sdev->dev, "error: too many capture streams %d\n",
			num_playback);
		return -EINVAL;
	}

	/* mem alloc for the position buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, &pci->dev, 8, 
		&hdev->posbuffer);
	if (ret < 0) {
		dev_err(sdev->dev, "error: posbuffer dma alloc failed\n");
		return -ENOMEM;
	}

	/* mem alloc for ring buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, &pci->dev, 
		PAGE_SIZE, &hdev->ringbuffer);
	if (ret < 0) {
		dev_err(sdev->dev, "error: ringbuffer dma alloc failed\n");
		return -ENOMEM;
	}

	/* create capture streams */
	for (i = 0; i < num_capture; i++) {
	
		stream = &hdev->cstream[i];

		stream->pphc_addr = sdev->bar[APL_PP_BAR] + SOF_HDA_PPHC_BASE +
			SOF_HDA_PPHC_INTERVAL * i;

		stream->pplc_addr = sdev->bar[APL_PP_BAR] + SOF_HDA_PPLC_BASE +
			SOF_HDA_PPLC_MULTI * num_total + SOF_HDA_PPLC_INTERVAL * i;

		/* do we support SPIB */
		if (sdev->bar[APL_SPIB_BAR]) {
			stream->spib_addr = sdev->bar[APL_SPIB_BAR] +
				SOF_HDA_SPIB_BASE + SOF_HDA_SPIB_INTERVAL * i +
				SOF_HDA_SPIB_SPIB;

			stream->fifo_addr = sdev->bar[APL_SPIB_BAR] +
				SOF_HDA_SPIB_BASE + SOF_HDA_SPIB_INTERVAL * i +
				SOF_HDA_SPIB_MAXFIFO;
		}

		/* do we support DRSM */
		if (sdev->bar[APL_DRSM_BAR])
			stream->drsm_addr = sdev->bar[APL_DRSM_BAR] +
				SOF_HDA_DRSM_BASE + SOF_HDA_DRSM_INTERVAL * i;

		stream->sd_offset = 0x20 * i + SOF_HDA_ADSP_LOADER_BASE;
		stream->sd_addr = sdev->bar[APL_HDA_BAR] + 
					stream->sd_offset;

		stream->stream_tag = i + 1;
		stream->open = false;
		stream->running = false;
		stream->direction = SNDRV_PCM_STREAM_CAPTURE;
		stream->index = i;

		/* memory alloc for stream BDL */ 
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, &pci->dev, 
			APL_BDL_SIZE, &stream->bdl);
		if (ret < 0) {
			dev_err(sdev->dev, "error: stream bdl dma alloc failed\n");
			return -ENOMEM;
		}
		stream->posbuf = (__le32 *)(hdev->posbuffer.area +
			(stream->index) * 8);

	}

	/* create playback streams */
	for (i = num_capture; i < num_total; i++) {

		stream = &hdev->pstream[i - num_capture];

		/* we always have DSP support */
		stream->pphc_addr = sdev->bar[APL_PP_BAR] + SOF_HDA_PPHC_BASE +
			SOF_HDA_PPHC_INTERVAL * i;

		stream->pplc_addr = sdev->bar[APL_PP_BAR] + SOF_HDA_PPLC_BASE +
			SOF_HDA_PPLC_MULTI * num_total + SOF_HDA_PPLC_INTERVAL * i;

		/* do we support SPIB */
		if (sdev->bar[APL_SPIB_BAR]) {
			stream->spib_addr = sdev->bar[APL_SPIB_BAR] +
				SOF_HDA_SPIB_BASE + SOF_HDA_SPIB_INTERVAL * i +
				SOF_HDA_SPIB_SPIB;

			stream->fifo_addr = sdev->bar[APL_SPIB_BAR] +
				SOF_HDA_SPIB_BASE + SOF_HDA_SPIB_INTERVAL * i +
				SOF_HDA_SPIB_MAXFIFO;
		}

		/* do we support DRSM */
		if (sdev->bar[APL_DRSM_BAR])
			stream->drsm_addr = sdev->bar[APL_DRSM_BAR] +
				SOF_HDA_DRSM_BASE + SOF_HDA_DRSM_INTERVAL * i;
			
		stream->sd_offset = 0x20 * i + SOF_HDA_ADSP_LOADER_BASE;
		stream->sd_addr = sdev->bar[APL_HDA_BAR] + 
					stream->sd_offset;
		stream->stream_tag = i - num_capture + 1;
		stream->open = false;
		stream->running = false;
		stream->direction = SNDRV_PCM_STREAM_PLAYBACK;
		stream->index = i;
	
		/* mem alloc for stream BDL */ 
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, &pci->dev,
			APL_BDL_SIZE, &stream->bdl);
		if (ret < 0) {
			dev_err(sdev->dev, "error: stream bdl dma alloc failed\n");
			return -ENOMEM;
		}				
	
		stream->posbuf = (__le32 *)(hdev->posbuffer.area +
			(stream->index) * 8);
		
	}

	return 0;
}

/*
 * first boot sequence has some extra steps. core 0 waits for power
 * status on core 1, so power up core 1 also momentarily, keep it in
 * reset/stall and then turn it off
 */
static int apl_init(struct snd_sof_dev *sdev,
	const void *fwdata, u32 fwsize)
{
	int stream_tag, ret, i;
	u32 hipcie;

	// prepare DMA for code loader use
	stream_tag = apl_prepare(sdev, 0x40, fwsize, &sdev->dmab, 
			SNDRV_PCM_STREAM_PLAYBACK);

	if (stream_tag <= 0) {
		dev_err(sdev->dev, "error: dma prepare for fw loading err: %x\n",
			stream_tag);
		return stream_tag;
	}

	memcpy(sdev->dmab.area, fwdata, fwsize);

	/* step 1: power up core 0 and core 1 */
	ret = apl_core_power_up(sdev, APL_DSP_CORE_MASK(0) |
		APL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(sdev->dev, "error: dsp core 0/1 power up failed\n");
		goto err;
	}

	/* step 2: purge FW request */
	snd_sof_dsp_write(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCI,
		APL_DSP_REG_HIPCI_BUSY | (APL_IPC_PURGE_FW | 
		((stream_tag - 1) << 9)));

	/* step 3: unset core 0 reset state & unstall/run core 0 */
	ret = apl_run_core(sdev, APL_DSP_CORE_MASK(0));
	if (ret < 0) {
		dev_err(sdev->dev, "error: dsp core start failed %d\n", ret);
		ret = -EIO;
		goto err;
	}

	/* step 4: wait for IPC DONE bit from ROM */
	for (i = APL_INIT_TIMEOUT; i > 0; i--) {

		hipcie = snd_sof_dsp_read(sdev, APL_DSP_BAR,
			APL_DSP_REG_HIPCIE);

		if (hipcie & APL_DSP_REG_HIPCIE_DONE) {
			snd_sof_dsp_update_bits(sdev, APL_DSP_BAR,
				APL_DSP_REG_HIPCIE,
				APL_DSP_REG_HIPCIE_DONE,
				APL_DSP_REG_HIPCIE_DONE);
			goto step5;
		}
		mdelay(1);
	}

	dev_err(sdev->dev, "error: waiting for HIPCIE done, reg: 0x%x\n",
		hipcie);
	goto err;

step5:
	/* step 5: power down core1 */
	ret = apl_core_power_down(sdev, APL_DSP_CORE_MASK(1));
	if (ret < 0) {
		dev_err(sdev->dev, "error: dsp core 1 power down failed\n");
		goto err;
	}

	/* step 6: enable interrupt */
	snd_sof_dsp_update_bits(sdev, APL_DSP_BAR, APL_DSP_REG_ADSPIC,
		APL_ADSPIC_IPC, APL_ADSPIC_IPC);
	
	/* enable IPC DONE interrupt */
	snd_sof_dsp_update_bits(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCCTL,
		APL_DSP_REG_HIPCCTL_DONE, APL_DSP_REG_HIPCCTL_DONE);

	/* enable IPC BUSY interrupt */
	snd_sof_dsp_update_bits(sdev, APL_DSP_BAR, APL_DSP_REG_HIPCCTL,
		APL_DSP_REG_HIPCCTL_BUSY, APL_DSP_REG_HIPCCTL_BUSY);

	/* step 7: wait for ROM init */
	ret = snd_sof_dsp_register_poll(sdev, APL_DSP_BAR, APL_ADSP_FW_STATUS, 
		APL_ROM_STS_MASK, APL_ROM_INIT, APL_INIT_TIMEOUT) ;
	if (ret >= 0)
		goto out;
	
	ret = -EIO;

err:
	apl_dump(sdev, SOF_DBG_REGS | SOF_DBG_PCI | SOF_DBG_MBOX);
	//sdev->dsp_ops.cleanup(sdev->dev, &sdev->dmab, stream_tag);
	apl_disable_core(sdev, APL_DSP_CORE_MASK(0) | APL_DSP_CORE_MASK(1));
	return ret;

out:
	return stream_tag;
}

/* 
 *DMA Code Loader for BXT/APL 
 */

int apl_load_firmware(struct snd_sof_dev *sdev, 
	const struct firmware *fw)
{
	struct snd_sof_pdata *plat_data = dev_get_platdata(sdev->dev);
	int ret;

	/* set code loading condition to true */
	sdev->code_loading = 1;
		
	ret = request_firmware(&plat_data->fw, 
		plat_data->machine->fw_filename, sdev->dev);

	if (ret < 0) {
		dev_err(sdev->dev, "error: request firmware failed err: %d\n", ret);
		return -EINVAL;
	}

	if (plat_data->fw == NULL)
		return -EINVAL;

	return ret;
}

/*
 *firmware download and run for BXT/APL
 */

int apl_run_firmware(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *plat_data = dev_get_platdata(sdev->dev);
	struct firmware stripped_firmware;
	int ret, stream_tag;

	stripped_firmware.data = plat_data->fw->data;
	stripped_firmware.size = plat_data->fw->size;

	stream_tag = apl_init(sdev, stripped_firmware.data, 
		stripped_firmware.size);

	/* retry enabling core and ROM load. seemed to help */
	if (stream_tag < 0) {
		stream_tag = apl_init(sdev, stripped_firmware.data,
				stripped_firmware.size);
		if (stream_tag <= 0) {
			dev_err(sdev->dev, "Error code=0x%x: FW status=0x%x\n",
				snd_sof_dsp_read(sdev, APL_DSP_BAR, 
					APL_ADSP_ERROR_CODE),
				snd_sof_dsp_read(sdev, APL_DSP_BAR, 
					APL_ADSP_FW_STATUS));
			dev_err(sdev->dev, "Core En/ROM load fail:%d\n", 
				stream_tag);
			ret = stream_tag;
			goto irq_err;
		}
	}

	/* init for booting wait */
	init_waitqueue_head(&sdev->boot_wait);
	sdev->boot_complete = false;

	/* at this point DSP ROM has been initialized and should be ready for 
	 * code loading and firmware boot 
	 */
	ret = apl_transfer_fw(sdev, stream_tag);
	if (ret < 0) {
		dev_err(sdev->dev, "error: load fw failed err: %d\n", ret);
		goto irq_err;
	} 

	dev_dbg(sdev->dev, "Firmware download successful, booting...\n");

	return ret;

irq_err:
	apl_dump(sdev, SOF_DBG_REGS | SOF_DBG_PCI | SOF_DBG_MBOX);
	free_irq(sdev->ipc_irq, sdev);
	/* disable DSP */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL,
		SOF_HDA_PPCTL_GPROCEN, 0);
	dev_err(sdev->dev, "error: load fw failed err: %d\n", ret);
	return ret;
}

/*
 * Probe and remove.
 */
 
/*
 * We dont need to do a full HDA codec probe as external HDA codec mode is
 * considered legacy and will not be supported under SOF. HDMI/DP HDA will
 * be supported in the DSP.
 */ 
static int apl_probe(struct snd_sof_dev *sdev)
{
	struct pci_dev *pci = sdev->pci;
	int ret = 0, i;
	struct snd_sof_hda_dev *hdev = &sdev->hda;
	struct snd_sof_hda_stream *stream;

	/* HDA base */
	sdev->bar[APL_HDA_BAR] = pci_ioremap_bar(pci, APL_HDA_BAR);
	if (sdev->bar[APL_HDA_BAR] == NULL) {
		dev_err(&pci->dev, "error: ioremap error\n");
		return -ENXIO;
	}

	/* DSP base */
	sdev->bar[APL_DSP_BAR] = pci_ioremap_bar(pci, APL_DSP_BAR);
	if (sdev->bar[APL_DSP_BAR] == NULL) {
		dev_err(&pci->dev, "error: ioremap error\n");
		ret = -ENXIO;
		goto err;
	}

	/* TODO: add base offsets for each SRAM window */ 
	sdev->mmio_bar = APL_DSP_BAR;
	sdev->mailbox_bar = APL_DSP_BAR;

	pci_set_master(pci);
	synchronize_irq(pci->irq);

	/* allow 64bit DMA address if supported by H/W */
	if (!dma_set_mask(&pci->dev, DMA_BIT_MASK(64))) {
		dev_dbg(&pci->dev, "64 bit\n");
		dma_set_coherent_mask(&pci->dev, DMA_BIT_MASK(64));
	} else {
		dev_dbg(&pci->dev, "32 bit\n");
		dma_set_mask(&pci->dev, DMA_BIT_MASK(32));
		dma_set_coherent_mask(&pci->dev, DMA_BIT_MASK(32));
	}

	/* get controller capabilities */
	ret = apl_get_caps(sdev);
	if (ret < 0) {
		dev_err(&pci->dev, "error: failed to find DSP capability\n");
		goto irq_err;
	}

	/* init streams */
	ret = apl_stream_init(sdev);
	if (ret < 0) {
		dev_err(&pci->dev, "error: failed to init streams\n");
		goto irq_err;
	}
	
	/*
	 * clear bits 0-2 of PCI register TCSEL (at offset 0x44)
	 * TCSEL == Traffic Class Select Register, which sets PCI express QOS
	 * Ensuring these bits are 0 clears playback static on some HD Audio
	 * codecs. PCI register TCSEL is defined in the Intel manuals.
	 */
	snd_sof_pci_update_bits(sdev, PCI_TCSEL, 0x07, 0);

	/*
	 * while performing reset, controller may not come back properly causing
	 * issues, so recommendation is to set CGCTL.MISCBDCGE to 0 then do 
	 * reset (init chip) and then again set CGCTL.MISCBDCGE to 1
	 */
	snd_sof_pci_update_bits(sdev, PCI_CGCTL,
		PCI_CGCTL_MISCBDCGE_MASK, 0);

	/* clear WAKESTS */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_WAKESTS, 
		SOF_HDA_WAKESTS_INT_MASK, SOF_HDA_WAKESTS_INT_MASK);

	/* reset HDA controller */
	ret = apl_link_reset(sdev);
	if (ret < 0) {
		dev_err(&pci->dev, "error: failed to reset HDA controller\n");
		goto err;
	}

	/* clear stream status */
	for (i = 0 ; i < hdev->num_capture ; i++ ) {
		stream = &hdev->cstream[i];
		if (stream)
			snd_sof_dsp_update_bits(sdev, APL_HDA_BAR,
				stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS,
				SOF_HDA_CL_DMA_SD_INT_MASK, SOF_HDA_CL_DMA_SD_INT_MASK);
	}

	for (i = 0 ; i < hdev->num_playback ; i++ ) {
		stream = &hdev->pstream[i];
		if (stream)
			snd_sof_dsp_update_bits(sdev, APL_HDA_BAR,
				stream->sd_offset + SOF_HDA_ADSP_REG_CL_SD_STS,
				SOF_HDA_CL_DMA_SD_INT_MASK, SOF_HDA_CL_DMA_SD_INT_MASK);
	} 

	/* clear WAKESTS */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_WAKESTS, 
		SOF_HDA_WAKESTS_INT_MASK, SOF_HDA_WAKESTS_INT_MASK);
#if 0
	/* clear RIRBSTS */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, RIRBSTS, RIRB_INT_MASK, 
		RIRB_INT_MASK);
#endif
	/* clear interrupt status register */
	snd_sof_dsp_write(sdev, APL_HDA_BAR, SOF_HDA_INTSTS, 
		SOF_HDA_INT_CTRL_EN | SOF_HDA_INT_ALL_STREAM);	

	/* enable CIE and GIE interrupts */
	snd_sof_dsp_update_bits(sdev, APL_HDA_BAR, SOF_HDA_INTCTL,
		SOF_HDA_INT_CTRL_EN | SOF_HDA_INT_GLOBAL_EN,
		SOF_HDA_INT_CTRL_EN | SOF_HDA_INT_GLOBAL_EN);

	dev_dbg(sdev->dev, "using PCI IRQ %d\n", sdev->pci->irq);

	/* register our IRQ */
	ret = request_threaded_irq(sdev->pci->irq, skl_interrupt,
		skl_threaded_handler, IRQF_SHARED, "AudioHDA", sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register HDA IRQ %d\n",
			sdev->ipc_irq);
		goto err;
	}
	sdev->hda.irq = pci->irq;
	
	sdev->ipc_irq = pci->irq;
	dev_dbg(sdev->dev, "using IPC IRQ %d\n", sdev->ipc_irq);
	ret = request_threaded_irq(sdev->ipc_irq, apl_irq_handler,
		apl_irq_thread, IRQF_SHARED, "AudioDSP", sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register PCI IRQ %d\n",
			sdev->ipc_irq);
		goto err;
	}

	/* re-enable CGCTL.MISCBDCGE after rest */
	snd_sof_pci_update_bits(sdev, PCI_CGCTL,
		PCI_CGCTL_MISCBDCGE_MASK, PCI_CGCTL_MISCBDCGE_MASK);

	device_disable_async_suspend(&pci->dev);

	/* enable DSP features */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL,
		SOF_HDA_PPCTL_GPROCEN, SOF_HDA_PPCTL_GPROCEN);

	/* enable DSP IRQ */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL, 
		SOF_HDA_PPCTL_PIE, SOF_HDA_PPCTL_PIE);
	
	/* inititalize waitq for code loading */
	init_waitqueue_head(&sdev->waitq);

	return 0;		

irq_err:
	free_irq(sdev->ipc_irq, sdev);
err:
	/* disable DSP */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL,
		SOF_HDA_PPCTL_GPROCEN, 0);
	return ret;
}

static int apl_remove(struct snd_sof_dev *sdev)
{
	/* disable cores */
	apl_disable_core(sdev, APL_DSP_CORE_MASK(0) | APL_DSP_CORE_MASK(1));

	/* disable DSP IRQ */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL,
		SOF_HDA_PPCTL_PIE, 0);

	/* disable DSP */
	snd_sof_dsp_update_bits(sdev, APL_PP_BAR, SOF_HDA_REG_PP_PPCTL,
		SOF_HDA_PPCTL_GPROCEN, 0);

	free_irq(sdev->ipc_irq, sdev);

	return 0;
}

/* broxton ops */
struct snd_sof_dsp_ops snd_sof_bxt_ops = {

	/* probe and remove */
	.probe		= apl_probe,
	.remove		= apl_remove,

	/* DSP core boot / reset */
//	.run		= apl_run,
//	.reset		= apl_reset,

	/* Register IO */
	.write		= apl_write,
	.read		= apl_read,
	.write64	= apl_write64,
	.read64		= apl_read64,

	/* Block IO */
	.block_read	= apl_block_read,
	.block_write	= apl_block_write,

	/* doorbell */
	.irq_handler	= apl_irq_handler,
	.irq_thread	= apl_irq_thread,

	/* mailbox */
	.mailbox_read	= apl_mailbox_read,
	.mailbox_write	= apl_mailbox_write,

	/* ipc */
	.tx_msg		= apl_tx_msg,
	.fw_ready	= apl_fw_ready,
	//int (*rx_msg)(struct snd_sof_dev *sof_dev, struct sof_ipc_msg *msg);

	/* debug */
	.debug_map	= apl_debugfs,
	.debug_map_count	= ARRAY_SIZE(apl_debugfs),
	.dbg_dump	= apl_dump,
	
	/* firmware loading */
	.load_firmware = apl_load_firmware,

	/* firmware run */
	.run = apl_run_firmware,

};
EXPORT_SYMBOL(snd_sof_bxt_ops);

/* appololake ops */
struct snd_sof_dsp_ops snd_sof_apl_ops = {

	/* probe and remove */
	.probe		= apl_probe,
	.remove		= apl_remove,

	/* DSP core boot / reset */
//	.run		= apl_run,
//	.reset		= apl_reset,

	/* Register IO */
	.write		= apl_write,
	.read		= apl_read,
	.write64	= apl_write64,
	.read64		= apl_read64,

	/* Block IO */
	.block_read	= apl_block_read,
	.block_write	= apl_block_write,

	/* doorbell */
	.irq_handler	= apl_irq_handler,
	.irq_thread	= apl_irq_thread,

	/* mailbox */
	.mailbox_read	= apl_mailbox_read,
	.mailbox_write	= apl_mailbox_write,

	/* ipc */
	.tx_msg		= apl_tx_msg,
	.fw_ready	= apl_fw_ready,
	//int (*rx_msg)(struct snd_sof_dev *sof_dev, struct sof_ipc_msg *msg);

	/* debug */
	.debug_map	= apl_debugfs,
	.debug_map_count	= ARRAY_SIZE(apl_debugfs),
	.dbg_dump	= apl_dump,

	/* firmware loading */
	.load_firmware = apl_load_firmware,

};
EXPORT_SYMBOL(snd_sof_apl_ops);

MODULE_LICENSE("Dual BSD/GPL");
