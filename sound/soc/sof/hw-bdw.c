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

#define DEBUG

/* 
 * Hardwre interface for audio DSP on Haswell and Broadwell
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>

#include <trace/events/hswadsp.h>
#include <sound/sof.h>
#include "sof-priv.h"
#include "ops.h"
#include "intel.h"

/* BARs */
#define BDW_DSP_BAR 0
#define BDW_PCI_BAR 1

/*
 * Debug
 */

/* DSP memories for BDW */
#define IRAM_OFFSET     0xA0000
#define BDW_IRAM_SIZE       (10 * 32 * 1024) 
#define DRAM_OFFSET     0x00000
#define BDW_DRAM_SIZE       (20 * 32 * 1024) 
#define SHIM_OFFSET     0xFB000
#define SHIM_SIZE       0x100
#define MBOX_OFFSET     0x9E000
#define MBOX_SIZE       0x1000
#define MBOX_DUMP_SIZE 0x30

/* DSP peripherals */
#define DMAC0_OFFSET    0xFE000
#define DMAC1_OFFSET    0xFF000
#define DMAC_SIZE       0x420
#define SSP0_OFFSET     0xFC000
#define SSP1_OFFSET     0xFD000
#define SSP_SIZE	0x100

static const struct snd_sof_debugfs_map bdw_debugfs[] = {
	{"dmac0", BDW_DSP_BAR, DMAC0_OFFSET, DMAC_SIZE},
	{"dmac1", BDW_DSP_BAR, DMAC1_OFFSET, DMAC_SIZE},
	{"ssp0", BDW_DSP_BAR, SSP0_OFFSET, SSP_SIZE},
	{"ssp1", BDW_DSP_BAR, SSP1_OFFSET, SSP_SIZE},
	{"iram", BDW_DSP_BAR, IRAM_OFFSET, BDW_IRAM_SIZE},
	{"dram", BDW_DSP_BAR, DRAM_OFFSET, BDW_DRAM_SIZE},
	{"shim", BDW_DSP_BAR, SHIM_OFFSET, SHIM_SIZE},
	{"mbox", BDW_DSP_BAR, MBOX_OFFSET, MBOX_SIZE},
};

/*
 * Memory copy.
 */

/* write has to deal with copying non 32 bit sized data */
static void bdw_block_write(struct snd_sof_dev *sdev, u32 offset, void *src,
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

static void bdw_block_read(struct snd_sof_dev *sdev, u32 offset, void *dest,
	size_t size)
{
	volatile void __iomem *src = sdev->bar[sdev->mmio_bar] + offset;
	memcpy_fromio(dest, src, size);
}
/*
 * Register IO
 */

static void bdw_write(struct snd_sof_dev *sdev, void __iomem *addr,
	u32 value)
{
	writel(value, addr);
}

static u32 bdw_read(struct snd_sof_dev *sdev, void __iomem *addr)
{
	return readl(addr);
}

static void bdw_write64(struct snd_sof_dev *sdev, void __iomem *addr,
	u64 value)
{
	memcpy_toio(addr, &value, sizeof(value));
}

static u64 bdw_read64(struct snd_sof_dev *sdev, void __iomem *addr)
{
	u64 val;

	memcpy_fromio(&val, addr, sizeof(val));
	return val;
}

/* 
 * DSP Control.
 */

static int bdw_run(struct snd_sof_dev *sdev)
{
	/* set oportunistic mode on engine 0,1 for all channels */
	snd_sof_dsp_update_bits(sdev, BDW_DSP_BAR, SHIM_HMDC,
		SHIM_HMDC_HDDA_E0_ALLCH | SHIM_HMDC_HDDA_E1_ALLCH, 0);

	/* set DSP to RUN */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR, SHIM_CSR,
		SHIM_CSR_STALL, 0x0);

	return 0; //TODO: Fix return value
}

static int bdw_reset(struct snd_sof_dev *sdev)
{
	/* put DSP into reset and stall */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR, SHIM_CSR,
		SHIM_CSR_RST | SHIM_CSR_STALL, SHIM_CSR_RST | SHIM_CSR_STALL);

	/* keep in reset for 10ms */
	mdelay(10);

	/* take DSP out of reset and keep stalled for FW loading */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR, SHIM_CSR,
		SHIM_CSR_RST | SHIM_CSR_STALL, SHIM_CSR_STALL);

	return 0; //TODO: Fix return value
}

static int bdw_set_dsp_D0(struct snd_sof_dev *sdev)
{
	int tries = 10;
	u32 reg;

	/* Disable core clock gating (VDRTCTL2.DCLCGE = 0) */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_VDRTCTL2,
		PCI_VDRTCL2_DCLCGE | PCI_VDRTCL2_DTCGE, 
		~(PCI_VDRTCL2_DCLCGE | PCI_VDRTCL2_DTCGE));

	/* Disable D3PG (VDRTCTL0.D3PGD = 1) */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_VDRTCTL0,
		PCI_VDRTCL0_D3PGD, PCI_VDRTCL0_D3PGD);

	/* Set D0 state */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_PMCS,
		PCI_PMCS_PS_MASK, ~PCI_PMCS_PS_MASK);

	/* check that ADSP shim is enabled */
	while (tries--) {
		reg = readl(sdev->bar[BDW_PCI_BAR] + PCI_PMCS)
			& PCI_PMCS_PS_MASK;
		if (reg == 0)
			goto finish;

		msleep(1);
	}

	return -ENODEV;

finish:
	/* select SSP1 19.2MHz base clock, SSP clock 0, turn off Low Power Clock */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR, SHIM_CSR,
		SHIM_CSR_S1IOCS | SHIM_CSR_SBCS1 | SHIM_CSR_LPCS, 0x0);

	/* stall DSP core, set clk to 192/96Mhz */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR,
		SHIM_CSR, SHIM_CSR_STALL | SHIM_CSR_DCS_MASK,
		SHIM_CSR_STALL | SHIM_CSR_DCS(4));

	/* Set 24MHz MCLK, prevent local clock gating, enable SSP0 clock */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR, SHIM_CLKCTL,
		SHIM_CLKCTL_MASK | SHIM_CLKCTL_DCPLCG | SHIM_CLKCTL_SCOE0,
		SHIM_CLKCTL_MASK | SHIM_CLKCTL_DCPLCG | SHIM_CLKCTL_SCOE0);

	/* Stall and reset core, set CSR */
	bdw_reset(sdev);

	/* Enable core clock gating (VDRTCTL2.DCLCGE = 1), delay 50 us */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_VDRTCTL2,
		PCI_VDRTCL2_DCLCGE | PCI_VDRTCL2_DTCGE,
		PCI_VDRTCL2_DCLCGE | PCI_VDRTCL2_DTCGE);

	udelay(50);

	/* switch on audio PLL */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_VDRTCTL2,
		PCI_VDRTCL2_APLLSE_MASK, ~PCI_VDRTCL2_APLLSE_MASK);

	/* set default power gating control, enable power gating control for 
        all blocks. that is, can't be accessed, please enable each block
        before accessing. */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_PCI_BAR, PCI_VDRTCTL0,
		0xfffffffC,0x0);

	/* disable DMA finish function for SSP0 & SSP1 */
	snd_sof_dsp_update_bits_unlocked(sdev, BDW_DSP_BAR,  SHIM_CSR2,
		SHIM_CSR2_SDFD_SSP1, SHIM_CSR2_SDFD_SSP1);

	/* set on-demond mode on engine 0,1 for all channels */
	snd_sof_dsp_update_bits(sdev, BDW_DSP_BAR, SHIM_HMDC,
		SHIM_HMDC_HDDA_E0_ALLCH | SHIM_HMDC_HDDA_E1_ALLCH,
		SHIM_HMDC_HDDA_E0_ALLCH | SHIM_HMDC_HDDA_E1_ALLCH);

	/* Enable Interrupt from both sides */
	snd_sof_dsp_update_bits(sdev, BDW_DSP_BAR, SHIM_IMRX,
		(SHIM_IMRX_BUSY | SHIM_IMRX_DONE), 0x0);
	snd_sof_dsp_update_bits(sdev, BDW_DSP_BAR, SHIM_IMRD,
		(SHIM_IMRD_DONE |SHIM_IMRD_BUSY | SHIM_IMRD_SSP0
		| SHIM_IMRD_DMAC), 0x0);

	/* clear IPC registers */
	snd_sof_dsp_write(sdev, BDW_DSP_BAR, SHIM_IPCX, 0x0);
	snd_sof_dsp_write(sdev, BDW_DSP_BAR, SHIM_IPCD, 0x0);
	snd_sof_dsp_write(sdev, BDW_DSP_BAR, 0x80, 0x6);
	snd_sof_dsp_write(sdev, BDW_DSP_BAR, 0xe0, 0x300a);

	return 0;
}

static void bdw_dump(struct snd_sof_dev *sdev, u32 flags)
{
	int i;

	if (flags & SOF_DBG_REGS) {
		for (i = SHIM_OFFSET; i < SHIM_OFFSET  + SHIM_SIZE; i += 8 ) {
			dev_dbg(sdev->dev, "shim 0x%2.2x value 0x%16.16llx\n",
			i - SHIM_OFFSET,
			snd_sof_dsp_read64(sdev, BDW_DSP_BAR, i));
		}
	}

	if (flags & SOF_DBG_MBOX) {
		for (i = MBOX_OFFSET; i < MBOX_OFFSET + MBOX_DUMP_SIZE; i += 4)
		{
			dev_dbg(sdev->dev, "mbox: 0x%2.2x value 0x%8.8x\n",
			i - MBOX_OFFSET,
			readl(sdev->bar[BDW_DSP_BAR] + i));
		}
	}

	if (flags & SOF_DBG_TEXT) {
		for (i = IRAM_OFFSET; i < IRAM_OFFSET + MBOX_DUMP_SIZE; i += 4)
		{
			dev_dbg(sdev->dev, "iram: 0x%2.2x value 0x%8.8x\n",
			i - IRAM_OFFSET,
			readl(sdev->bar[BDW_DSP_BAR] + i));
		}
	}

	if (flags & SOF_DBG_PCI) {
		for (i = 0; i < 0xff; i += 4) {
			dev_dbg(sdev->dev, "pci: 0x%2.2x value 0x%8.8x\n",
			i, readl(sdev->bar[BDW_PCI_BAR] + i));
		}
	}
}

#if 0

static void bdw_notify(struct sst_dsp *dsp)
{
	sst_dsp_shim_update_bits(dsp, SST_IPCD,
		SST_IPCD_BUSY | SST_IPCD_DONE, SST_IPCD_DONE);
}


static bool bdw_is_dsp_busy(struct snd_sof_dev *sdev)
{
	u32 ipcx;

	ipcx = snd_sof_dsp_read64(sdev, BDW_DSP_BAR, SHIM_IPCX);
	return (ipcx & (SHIM_IPCX_BUSY | SHIM_IPCX_DONE));
}
#endif

/*
 * IPC Doorbell IRQ handler and thread.
 */

static irqreturn_t bdw_irq_handler(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	u64 isr;
	int ret = IRQ_NONE;

	spin_lock(&sdev->spinlock);

	/* Interrupt arrived, check src */
	isr = snd_sof_dsp_read64(sdev, BDW_DSP_BAR, SHIM_ISRX);
	if (isr & SHIM_ISRX_DONE) {

		/* Mask Done interrupt before return */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR,
			SHIM_IMRX, SHIM_IMRX_DONE, SHIM_IMRX_DONE);
		ret = IRQ_WAKE_THREAD;
	}

	if (isr & SHIM_ISRX_BUSY) {

		/* Mask Busy interrupt before return */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR,
			SHIM_IMRX, SHIM_IMRX_BUSY, SHIM_IMRX_BUSY);
		ret = IRQ_WAKE_THREAD;
	}

	spin_unlock(&sdev->spinlock);
	return ret;
}

static irqreturn_t bdw_irq_thread(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	u64 ipcx, ipcd;
	unsigned long flags;

	spin_lock_irqsave(&sdev->spinlock, flags);

	ipcx = snd_sof_dsp_read64(sdev, BDW_DSP_BAR, SHIM_IPCX);
	ipcd = snd_sof_dsp_read64(sdev, BDW_DSP_BAR, SHIM_IPCD);

	/* reply message from DSP */
	if (ipcx & SHIM_IPCX_DONE) {

		/* Handle Immediate reply from DSP Core */
		snd_sof_ipc_process_reply(sdev, ipcx);

		/* clear DONE bit - tell DSP we have completed */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR, SHIM_IPCX,
			SHIM_IPCX_DONE, 0);

		/* unmask Done interrupt */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR, SHIM_IMRX,
			SHIM_IMRX_DONE, 0);
	}

	/* new message from DSP */
	if (ipcd & SHIM_IPCD_BUSY) {

		/* Handle Notification and Delayed reply from DSP Core */
		snd_sof_ipc_process_notification(sdev, ipcd);

		/* clear BUSY bit and set DONE bit - accept new messages */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR, SHIM_IPCD,
			SHIM_IPCD_BUSY | SHIM_IPCD_DONE,
			SHIM_IPCD_DONE);

		/* unmask busy interrupt */
		snd_sof_dsp_update_bits64_unlocked(sdev, BDW_DSP_BAR, SHIM_IMRX,
			SHIM_IMRX_BUSY, 0);
	}

	spin_unlock_irqrestore(&sdev->spinlock, flags);

	/* continue to send any remaining messages... */
	snd_sof_ipc_process_msgs(sdev);

	return IRQ_HANDLED;
}

/*
 * IPC Firmware ready.
 */
static int bdw_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct sof_ipc_fw_ready *fw_ready = &sdev->fw_ready;
	struct sof_ipc_fw_version *v = &fw_ready->version;
	u32 offset;

	/* mailbox must be on 4k boundary */
	offset = MBOX_OFFSET;

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x offset %d\n",
		msg_id, offset);

	/* copy data from the DSP FW ready offset */
	bdw_block_read(sdev, offset, fw_ready, sizeof(*fw_ready));

	snd_sof_dsp_mailbox_init(sdev, fw_ready->inbox_offset,
		fw_ready->inbox_size, fw_ready->outbox_offset,
		fw_ready->outbox_size);

	dev_dbg(sdev->dev, " mailbox upstream 0x%x - size 0x%x\n",
		fw_ready->inbox_offset, fw_ready->inbox_size);
	dev_dbg(sdev->dev, " mailbox downstream 0x%x - size 0x%x\n",
		fw_ready->outbox_offset, fw_ready->outbox_size);
	
	dev_info(sdev->dev, " Firmware info: version %d:%d-%s build %d on %s:%s\n", 
		v->major, v->minor, v->tag, v->build, v->date, v->time);

	return 0;
}

/*
 * IPC Mailbox IO
 */

static void bdw_mailbox_write(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *dest = sdev->bar[sdev->mailbox_bar] + offset;

	memcpy_toio(dest, message, bytes);
}

static void bdw_mailbox_read(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *src = sdev->bar[sdev->mailbox_bar] + offset;

	memcpy_fromio(message, src, bytes);
}

static int bdw_tx_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	u64 cmd = msg->header;

	/* send the message */
	bdw_mailbox_write(sdev, sdev->outbox.offset, msg->msg_data, 
		msg->msg_size);
	snd_sof_dsp_write64(sdev, BDW_DSP_BAR, SHIM_IPCX, cmd | SHIM_IPCX_BUSY);

	return 0;
}

/*
 * Probe and remove.
 */
static int bdw_probe(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *pdata = sdev->pdata;
	const struct sof_dev_desc *desc = pdata->desc;
	struct platform_device *pdev =
		container_of(sdev->parent, struct platform_device, dev);
	struct resource *mmio;
	u32 base, size;
	int ret = 0;

	/* LPE base */
	mmio = platform_get_resource(pdev, IORESOURCE_MEM,
		desc->resindex_lpe_base);
	if (mmio) {
		base = mmio->start;
		size = resource_size(mmio);
	} else {
		dev_err(sdev->dev, "error: failed to get LPE base at idx %d\n",
			desc->resindex_lpe_base);
		return -EINVAL;
	}

	dev_dbg(sdev->dev, "LPE PHY base at 0x%x size 0x%x", base, size);
	sdev->bar[BDW_DSP_BAR] = ioremap(base, size);
	if (sdev->bar[BDW_DSP_BAR] == NULL) {
		dev_err(sdev->dev, 
			"error: failed to ioremap LPE base 0x%x size 0x%x\n",
			base, size);
		return -ENODEV;
	}
	dev_dbg(sdev->dev, "LPE VADDR %p\n", sdev->bar[BDW_DSP_BAR]);

	/* TODO: add offsets */
	sdev->mmio_bar = BDW_DSP_BAR;
	sdev->mailbox_bar = BDW_DSP_BAR;

	/* PCI base */
	mmio = platform_get_resource(pdev, IORESOURCE_MEM,
		desc->resindex_pcicfg_base);
	if (mmio) {
		base = mmio->start;
		size = resource_size(mmio);
	} else {
		dev_err(sdev->dev, "error: failed to get PCI base at idx %d\n",
			desc->resindex_pcicfg_base);
		ret = -ENODEV;
		goto pci_err;
	}

	dev_dbg(sdev->dev, "PCI base at 0x%x size 0x%x", base, size);
	sdev->bar[BDW_PCI_BAR] = ioremap(base, size);
	if (sdev->bar[BDW_PCI_BAR] == NULL) {
		dev_err(sdev->dev, 
			"error: failed to ioremap PCI base 0x%x size 0x%x\n",
			base, size);
		ret = -ENODEV;
		goto pci_err;
	}
	dev_dbg(sdev->dev, "PCI VADDR %p\n", sdev->bar[BDW_PCI_BAR]);

	/* register our IRQ */
	sdev->ipc_irq = platform_get_irq(pdev, desc->irqindex_host_ipc);
	if (sdev->ipc_irq < 0) {
		dev_err(sdev->dev, "error: failed to get IRQ at index %d\n",
			desc->irqindex_host_ipc);
		ret = sdev->ipc_irq;
		goto irq_err;
	}

	dev_dbg(sdev->dev, "using IRQ %d\n", sdev->ipc_irq);
	ret = request_threaded_irq(sdev->ipc_irq, bdw_irq_handler,
		bdw_irq_thread, 0, "AudioDSP", sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register IRQ %d\n",
			sdev->ipc_irq);
		goto irq_err;		
	}

	/* enable the DSP SHIM */
	ret = bdw_set_dsp_D0(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to set DSP D0 \n");
		return ret;
	}

	/* DSP DMA can only access low 31 bits of host memory */
	ret = dma_coerce_mask_and_coherent(sdev->dev, DMA_BIT_MASK(31));
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to set DMA mask %d\n", ret);
		return ret;
	}

	/* set BARS */
	sdev->cl_bar = BDW_DSP_BAR;

	return ret;

irq_err:
	iounmap(sdev->bar[BDW_DSP_BAR]);
pci_err:
	iounmap(sdev->bar[BDW_PCI_BAR]);
	return ret;
}

static int bdw_remove(struct snd_sof_dev *sdev)
{
	iounmap(sdev->bar[BDW_DSP_BAR]);
	iounmap(sdev->bar[BDW_PCI_BAR]);
	free_irq(sdev->ipc_irq, sdev);
	return 0;
}

/* broadwell ops */
struct snd_sof_dsp_ops snd_sof_bdw_ops = {

	/*Device init */
	.probe          = bdw_probe,
	.remove         = bdw_remove,
	
	/* DSP Core Control */
	.run            = bdw_run,
	.reset          = bdw_reset,

	/* Register IO */
	.read           = bdw_read,
	.write          = bdw_write,
	.read64         = bdw_read64,
	.write64        = bdw_write64,

	/* Block IO */
	.block_read     = bdw_block_read,
	.block_write    = bdw_block_write,

	/* mailbox */
	.mailbox_read   = bdw_mailbox_read,
	.mailbox_write  = bdw_mailbox_write,

	/* ipc */
	.tx_msg     	= bdw_tx_msg,
	.fw_ready	= bdw_fw_ready,
	//int (*rx_msg)(struct snd_sof_dev *sof_dev, struct sof_ipc_msg *msg);

	/* debug */
	.debug_map  = bdw_debugfs,
	.debug_map_count    = ARRAY_SIZE(bdw_debugfs),
	.dbg_dump   = bdw_dump,

	/* Module loading */
	.load_module    = snd_sof_parse_module_memcpy,

	/*Firmware loading */
	.load_firmware	= snd_sof_load_firmware_memcpy,
};
EXPORT_SYMBOL(snd_sof_bdw_ops);

MODULE_LICENSE("Dual BSD/GPL");
