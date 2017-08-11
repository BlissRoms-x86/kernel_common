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

#ifndef __SOF_INTEL_H
#define __SOF_INTEL_H


/*
 * SHIM registers for BYT, BSW, CHT HSW, BDW
 */

#define SHIM_CSR		(SHIM_OFFSET + 0x00)
#define SHIM_PISR		(SHIM_OFFSET + 0x08)
#define SHIM_PIMR		(SHIM_OFFSET + 0x10)
#define SHIM_ISRX		(SHIM_OFFSET + 0x18)
#define SHIM_ISRD		(SHIM_OFFSET + 0x20)
#define SHIM_IMRX		(SHIM_OFFSET + 0x28)
#define SHIM_IMRD		(SHIM_OFFSET + 0x30)
#define SHIM_IPCX		(SHIM_OFFSET + 0x38)
#define SHIM_IPCD		(SHIM_OFFSET + 0x40)
#define SHIM_ISRSC		(SHIM_OFFSET + 0x48)
#define SHIM_ISRLPESC		(SHIM_OFFSET + 0x50)
#define SHIM_IMRSC		(SHIM_OFFSET + 0x58)
#define SHIM_IMRLPESC		(SHIM_OFFSET + 0x60)
#define SHIM_IPCSC		(SHIM_OFFSET + 0x68)
#define SHIM_IPCLPESC		(SHIM_OFFSET + 0x70)
#define SHIM_CLKCTL		(SHIM_OFFSET + 0x78)
#define SHIM_CSR2		(SHIM_OFFSET + 0x80)
#define SHIM_LTRC		(SHIM_OFFSET + 0xE0)
#define SHIM_HMDC		(SHIM_OFFSET + 0xE8)


#define SHIM_PWMCTRL		0x1000


/*
 * SST SHIM register bits for BYT, BSW, CHT HSW, BDW
 * Register bit naming and functionaility can differ between devices.
 */

/* CSR / CS */
#define SHIM_CSR_RST		(0x1 << 1)
#define SHIM_CSR_SBCS0		(0x1 << 2)
#define SHIM_CSR_SBCS1		(0x1 << 3)
#define SHIM_CSR_DCS(x)		(x << 4)
#define SHIM_CSR_DCS_MASK	(0x7 << 4)
#define SHIM_CSR_STALL		(0x1 << 10)
#define SHIM_CSR_S0IOCS		(0x1 << 21)
#define SHIM_CSR_S1IOCS		(0x1 << 23)
#define SHIM_CSR_LPCS		(0x1 << 31)
#define SHIM_CSR_24MHZ_LPCS	(SHIM_CSR_SBCS0 | SHIM_CSR_SBCS1 | SHIM_CSR_LPCS)
#define SHIM_CSR_24MHZ_NO_LPCS	(SHIM_CSR_SBCS0 | SHIM_CSR_SBCS1)
#define SHIM_BYT_CSR_RST	(0x1 << 0)
#define SHIM_BYT_CSR_VECTOR_SEL	(0x1 << 1)
#define SHIM_BYT_CSR_STALL	(0x1 << 2)
#define SHIM_BYT_CSR_PWAITMODE	(0x1 << 3)

/*  ISRX / ISC */
#define SHIM_ISRX_BUSY		(0x1 << 1)
#define SHIM_ISRX_DONE		(0x1 << 0)
#define SHIM_BYT_ISRX_REQUEST	(0x1 << 1)

/*  ISRD / ISD */
#define SHIM_ISRD_BUSY		(0x1 << 1)
#define SHIM_ISRD_DONE		(0x1 << 0)

/* IMRX / IMC */
#define SHIM_IMRX_BUSY		(0x1 << 1)
#define SHIM_IMRX_DONE		(0x1 << 0)
#define SHIM_BYT_IMRX_REQUEST	(0x1 << 1)

/* IMRD / IMD */
#define SHIM_IMRD_DONE		(0x1 << 0)
#define SHIM_IMRD_BUSY		(0x1 << 1)
#define SHIM_IMRD_SSP0		(0x1 << 16)
#define SHIM_IMRD_DMAC0		(0x1 << 21)
#define SHIM_IMRD_DMAC1		(0x1 << 22)
#define SHIM_IMRD_DMAC		(SHIM_IMRD_DMAC0 | SHIM_IMRD_DMAC1)

/*  IPCX / IPCC */
#define	SHIM_IPCX_DONE		(0x1 << 30)
#define	SHIM_IPCX_BUSY		(0x1 << 31)
#define SHIM_BYT_IPCX_DONE	((u64)0x1 << 62)
#define SHIM_BYT_IPCX_BUSY	((u64)0x1 << 63)

/*  IPCD */
#define	SHIM_IPCD_DONE		(0x1 << 30)
#define	SHIM_IPCD_BUSY		(0x1 << 31)
#define SHIM_BYT_IPCD_DONE	((u64)0x1 << 62)
#define SHIM_BYT_IPCD_BUSY	((u64)0x1 << 63)

/* CLKCTL */
#define SHIM_CLKCTL_SMOS(x)	(x << 24)
#define SHIM_CLKCTL_MASK	(3 << 24)
#define SHIM_CLKCTL_DCPLCG	(1 << 18)
#define SHIM_CLKCTL_SCOE1	(1 << 17)
#define SHIM_CLKCTL_SCOE0	(1 << 16)

/* CSR2 / CS2 */
#define SHIM_CSR2_SDFD_SSP0	(1 << 1)
#define SHIM_CSR2_SDFD_SSP1	(1 << 2)

/* LTRC */
#define SHIM_LTRC_VAL(x)	(x << 0)

/* HMDC */
#define SHIM_HMDC_HDDA0(x)	(x << 0)
#define SHIM_HMDC_HDDA1(x)	(x << 7)
#define SHIM_HMDC_HDDA_E0_CH0	1
#define SHIM_HMDC_HDDA_E0_CH1	2
#define SHIM_HMDC_HDDA_E0_CH2	4
#define SHIM_HMDC_HDDA_E0_CH3	8
#define SHIM_HMDC_HDDA_E1_CH0	SHIM_HMDC_HDDA1(SHIM_HMDC_HDDA_E0_CH0)
#define SHIM_HMDC_HDDA_E1_CH1	SHIM_HMDC_HDDA1(SHIM_HMDC_HDDA_E0_CH1)
#define SHIM_HMDC_HDDA_E1_CH2	SHIM_HMDC_HDDA1(SHIM_HMDC_HDDA_E0_CH2)
#define SHIM_HMDC_HDDA_E1_CH3	SHIM_HMDC_HDDA1(SHIM_HMDC_HDDA_E0_CH3)
#define SHIM_HMDC_HDDA_E0_ALLCH	(SHIM_HMDC_HDDA_E0_CH0 | SHIM_HMDC_HDDA_E0_CH1 | \
				 SHIM_HMDC_HDDA_E0_CH2 | SHIM_HMDC_HDDA_E0_CH3)
#define SHIM_HMDC_HDDA_E1_ALLCH	(SHIM_HMDC_HDDA_E1_CH0 | SHIM_HMDC_HDDA_E1_CH1 | \
				 SHIM_HMDC_HDDA_E1_CH2 | SHIM_HMDC_HDDA_E1_CH3)




/* Audio DSP PCI registers */
#define PCI_VDRTCTL0		0xa0
#define PCI_VDRTCTL1		0xa4
#define PCI_VDRTCTL2		0xa8
#define PCI_VDRTCTL3		0xaC

/* VDRTCTL0 */
#define PCI_VDRTCL0_D3PGD		(1 << 0)
#define PCI_VDRTCL0_D3SRAMPGD		(1 << 1)
#define PCI_VDRTCL0_DSRAMPGE_SHIFT	12
#define PCI_VDRTCL0_DSRAMPGE_MASK	(0xfffff << PCI_VDRTCL0_DSRAMPGE_SHIFT)
#define PCI_VDRTCL0_ISRAMPGE_SHIFT	2
#define PCI_VDRTCL0_ISRAMPGE_MASK	(0x3ff << PCI_VDRTCL0_ISRAMPGE_SHIFT)

/* VDRTCTL2 */
#define PCI_VDRTCL2_DCLCGE		(1 << 1)
#define PCI_VDRTCL2_DTCGE		(1 << 10)
#define PCI_VDRTCL2_APLLSE_MASK		(1 << 31)

/* PMCS */
#define PCI_PMCS		0x84
#define PCI_PMCS_PS_MASK	0x3


/* controller reset timrout in ms */
#define APL_CTRL_RESET_TIMEOUT		100


/* PCI registers */
#define PCI_TCSEL			0x44
#define PCI_CGCTL			0x48

/* PCI_CGCTL bits */
#define PCI_CGCTL_MISCBDCGE_MASK	(1 << 6)

/* Legacy HDA registers and bits used - widths are variable - TODO: check*/
#define HDA_GCAP			0x0
#define HDA_GCTL			0x8
#define HDA_GCTL_UNSOL			(1 << 8)   /* accept unsol. response enable */
#define HDA_LLCH			0x14
#define HDA_INTCTL			0x20
#define HDA_INTSTS			0x24
#define HDA_WAKESTS			0x0E
#define HDA_WAKESTS_INT_MASK		((1 << HDA_MAX_CODECS) - 1)

/* HDA_GCTL register bist */
#define HDA_GCTL_RESET			(1 << 0)	

/* HDA_INCTL and HDA_INTSTS regs */
#define HDA_INT_GLOBAL_EN		(1 << 31)
#define HDA_INT_CTRL_EN			(1 << 30)
#define HDA_INT_ALL_STREAM		0xff

#define HDA_MAX_CAPS			10
#define HDA_CAP_ID_OFF			16
#define HDA_CAP_ID_MASK			(0xFFF << HDA_CAP_ID_OFF)
#define HDA_CAP_NEXT_MASK		0xFFFF

#define HDA_PP_CAP_ID			0x3
#define HDA_REG_PP_PPCH			0x10
#define HDA_REG_PP_PPCTL		0x04
#define HDA_PPCTL_PIE			(1<<31)
#define HDA_PPCTL_GPROCEN		(1<<30)

#define HDA_SPIB_CAP_ID			0x4
#define HDA_DRSM_CAP_ID			0x5

#define HDA_SPIB_BASE			0x08
#define HDA_SPIB_INTERVAL		0x08
#define HDA_SPIB_SPIB			0x00
#define HDA_SPIB_MAXFIFO		0x04

#define HDA_PPHC_BASE			0x10
#define HDA_PPHC_INTERVAL		0x10

#define HDA_PPLC_BASE			0x10
#define HDA_PPLC_MULTI			0x10
#define HDA_PPLC_INTERVAL		0x10

#define HDA_DRSM_BASE			0x08
#define HDA_DRSM_INTERVAL		0x08


/* Intel HD Audio General DSP Registers */
#define SKL_ADSP_GEN_BASE		0x0
#define SKL_ADSP_REG_ADSPCS		(SKL_ADSP_GEN_BASE + 0x04)
#define SKL_ADSP_REG_ADSPIC		(SKL_ADSP_GEN_BASE + 0x08)
#define SKL_ADSP_REG_ADSPIS		(SKL_ADSP_GEN_BASE + 0x0C)
#define SKL_ADSP_REG_ADSPIC2		(SKL_ADSP_GEN_BASE + 0x10)
#define SKL_ADSP_REG_ADSPIS2		(SKL_ADSP_GEN_BASE + 0x14)

/* Intel HD Audio Inter-Processor Communication Registers */
#define SKL_ADSP_IPC_BASE		0x40
#define SKL_ADSP_REG_HIPCT		(SKL_ADSP_IPC_BASE + 0x00)
#define SKL_ADSP_REG_HIPCTE		(SKL_ADSP_IPC_BASE + 0x04)
#define SKL_ADSP_REG_HIPCI		(SKL_ADSP_IPC_BASE + 0x08)
#define SKL_ADSP_REG_HIPCIE		(SKL_ADSP_IPC_BASE + 0x0C)
#define SKL_ADSP_REG_HIPCCTL		(SKL_ADSP_IPC_BASE + 0x10)

/*  HIPCI */
#define SKL_ADSP_REG_HIPCI_BUSY		BIT(31)

/* HIPCIE */
#define SKL_ADSP_REG_HIPCIE_DONE	BIT(30)

/* HIPCCTL */
#define SKL_ADSP_REG_HIPCCTL_DONE	BIT(1)
#define SKL_ADSP_REG_HIPCCTL_BUSY	BIT(0)

/* HIPCT */
#define SKL_ADSP_REG_HIPCT_BUSY		BIT(31)

/* FW base IDs */
#define SKL_INSTANCE_ID			0
#define SKL_BASE_FW_MODULE_ID		0

/* Intel HD Audio SRAM Window 1 */
#define SKL_ADSP_SRAM1_BASE		0xA000

#define SKL_ADSP_MMIO_LEN		0x10000

#define SKL_ADSP_W0_STAT_SZ		0x1000

#define SKL_ADSP_W0_UP_SZ		0x1000

#define SKL_ADSP_W1_SZ			0x1000

#define SKL_FW_STS_MASK			0xf

#define SKL_FW_INIT			0x1
#define SKL_FW_RFW_START		0xf

#define SKL_ADSPIC_IPC			1
#define SKL_ADSPIS_IPC			1

/* Core ID of core0 */
#define SKL_DSP_CORE0_ID		0

/* Mask for a given core index, c = 0.. number of supported cores - 1 */
#define SKL_DSP_CORE_MASK(c)		BIT(c)

/*
 * Mask for a given number of cores
 * nc = number of supported cores
 */
#define SKL_DSP_CORES_MASK(nc)	GENMASK((nc - 1), 0)

/* ADSPCS - Audio DSP Control & Status */

/*
 * Core Reset - asserted high
 * CRST Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CRST_SHIFT		0
#define SKL_ADSPCS_CRST_MASK(cm)	((cm) << SKL_ADSPCS_CRST_SHIFT)

/*
 * Core run/stall - when set to '1' core is stalled
 * CSTALL Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CSTALL_SHIFT		8
#define SKL_ADSPCS_CSTALL_MASK(cm)	((cm) << SKL_ADSPCS_CSTALL_SHIFT)

/*
 * Set Power Active - when set to '1' turn cores on
 * SPA Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_SPA_SHIFT		16
#define SKL_ADSPCS_SPA_MASK(cm)		((cm) << SKL_ADSPCS_SPA_SHIFT)

/*
 * Current Power Active - power status of cores, set by hardware
 * CPA Mask for a given core mask pattern, cm
 */
#define SKL_ADSPCS_CPA_SHIFT		24
#define SKL_ADSPCS_CPA_MASK(cm)		((cm) << SKL_ADSPCS_CPA_SHIFT)


/* various timeout values */
#define SKL_DSP_PU_TO		50
#define SKL_DSP_PD_TO		50
#define SKL_DSP_RESET_TO	50

#define BXT_BASEFW_TIMEOUT	3000
#define BXT_INIT_TIMEOUT	500
#define BXT_IPC_PURGE_FW	0x01004000

#define BXT_ROM_INIT		0x5
#define BXT_ADSP_SRAM0_BASE	0x80000

/* Firmware status window */
#define BXT_ADSP_FW_STATUS	BXT_ADSP_SRAM0_BASE
#define BXT_ADSP_ERROR_CODE     (BXT_ADSP_FW_STATUS + 0x4)

#define BXT_ADSP_SRAM1_BASE	0xA0000

#define BXT_INSTANCE_ID 0
#define BXT_BASE_FW_MODULE_ID 0

#define BXT_ADSP_FW_BIN_HDR_OFFSET 0x2000

/* Delay before scheduling D0i3 entry */
#define BXT_D0I3_DELAY 5000


#define FW_CL_STREAM_NUMBER		0x1

#define DMA_ADDRESS_128_BITS_ALIGNMENT	7
#define BDL_ALIGN(x)			(x >> DMA_ADDRESS_128_BITS_ALIGNMENT)

#define SKL_ADSPIC_CL_DMA			0x2
#define SKL_ADSPIS_CL_DMA			0x2
#define HDA_CL_DMA_SD_INT_DESC_ERR		0x10 /* Descriptor error interrupt */
#define HDA_CL_DMA_SD_INT_FIFO_ERR		0x08 /* FIFO error interrupt */
#define HDA_CL_DMA_SD_INT_COMPLETE		0x04 /* Buffer completion interrupt */
#define HDA_CL_DMA_SD_INT_MASK		(HDA_CL_DMA_SD_INT_DESC_ERR|\
					 HDA_CL_DMA_SD_INT_FIFO_ERR|\
				         HDA_CL_DMA_SD_INT_COMPLETE)
#define HDA_SD_CTL_DMA_START			0x02 /* Stream DMA start bit */

/* Intel HD Audio Code Loader DMA Registers */

#define HDA_ADSP_LOADER_BASE		0x80
#define HDA_ADSP_DPLBASE		0x70
#define HDA_ADSP_DPUBASE		0x74
#define HDA_ADSP_DPLBASE_ENABLE		0x1
#define CORBLBASE		0x40
#define CORBUBASE		0x44
#define CORBWP			0x48
#define CORBRP			0x4a
#define   CORBRP_RST	(1 << 15)  /* read pointer reset */
#define CORBCTL			0x4c
#define   CORBCTL_RUN	(1 << 1)   /* enable DMA */
#define   CORBCTL_CMEIE	(1 << 0)   /* enable memory error irq */
#define CORBSTS			0x4d
#define   CORBSTS_CMEI	(1 << 0)   /* memory error indication */
#define CORBSIZE		0x4e

#define RIRBLBASE		0x50
#define RIRBUBASE		0x54
#define RIRBWP			0x58
#define   RIRBWP_RST	(1 << 15)  /* write pointer reset */
#define RINTCNT			0x5a
#define RIRBCTL			0x5c
#define   RBCTL_IRQ_EN	(1 << 0)   /* enable IRQ */
#define   RBCTL_DMA_EN	(1 << 1)   /* enable DMA */
#define   RBCTL_OVERRUN_EN	(1 << 2)   /* enable overrun irq */
#define RIRBSTS			0x5d
#define   RBSTS_IRQ		(1 << 0)   /* response irq */
#define   RBSTS_OVERRUN	(1 << 2)   /* overrun irq */
#define RIRBSIZE		0x5e
#define RIRB_INT_MASK		0x05

/* Stream Registers */
#define HDA_ADSP_REG_CL_SD_CTL			0x00
#define HDA_ADSP_REG_CL_SD_STS			0x03
#define HDA_ADSP_REG_CL_SD_LPIB			0x04
#define HDA_ADSP_REG_CL_SD_CBL			0x08
#define HDA_ADSP_REG_CL_SD_LVI			0x0C
#define HDA_ADSP_REG_CL_SD_FIFOW		0x0E
#define HDA_ADSP_REG_CL_SD_FIFOSIZE		0x10
#define HDA_ADSP_REG_CL_SD_FORMAT		0x12
#define HDA_ADSP_REG_CL_SD_FIFOL		0x14
#define HDA_ADSP_REG_CL_SD_BDLPL		0x18
#define HDA_ADSP_REG_CL_SD_BDLPU		0x1C

/* CL: Software Position Based FIFO Capability Registers */
#define SKL_ADSP_REG_CL_SPBFIFO			(HDA_ADSP_LOADER_BASE + 0x20)
#define HDA_ADSP_REG_CL_SPBFIFO_SPBFCH		0x0
#define HDA_ADSP_REG_CL_SPBFIFO_SPBFCCTL	0x4
#define HDA_ADSP_REG_CL_SPBFIFO_SPIB		0x8
#define HDA_ADSP_REG_CL_SPBFIFO_MAXFIFOS	0xc

/* CL: Stream Descriptor x Control */

/* Stream Reset */
#define CL_SD_CTL_SRST_SHIFT		0
#define CL_SD_CTL_SRST_MASK		(1 << CL_SD_CTL_SRST_SHIFT)
#define CL_SD_CTL_SRST(x)		\
			((x << CL_SD_CTL_SRST_SHIFT) & CL_SD_CTL_SRST_MASK)

/* Stream Run */
#define CL_SD_CTL_RUN_SHIFT		1
#define CL_SD_CTL_RUN_MASK		(1 << CL_SD_CTL_RUN_SHIFT)
#define CL_SD_CTL_RUN(x)		\
			((x << CL_SD_CTL_RUN_SHIFT) & CL_SD_CTL_RUN_MASK)

/* Interrupt On Completion Enable */
#define CL_SD_CTL_IOCE_SHIFT		2
#define CL_SD_CTL_IOCE_MASK		(1 << CL_SD_CTL_IOCE_SHIFT)
#define CL_SD_CTL_IOCE(x)		\
			((x << CL_SD_CTL_IOCE_SHIFT) & CL_SD_CTL_IOCE_MASK)

/* FIFO Error Interrupt Enable */
#define CL_SD_CTL_FEIE_SHIFT		3
#define CL_SD_CTL_FEIE_MASK		(1 << CL_SD_CTL_FEIE_SHIFT)
#define CL_SD_CTL_FEIE(x)		\
			((x << CL_SD_CTL_FEIE_SHIFT) & CL_SD_CTL_FEIE_MASK)

/* Descriptor Error Interrupt Enable */
#define CL_SD_CTL_DEIE_SHIFT		4
#define CL_SD_CTL_DEIE_MASK		(1 << CL_SD_CTL_DEIE_SHIFT)
#define CL_SD_CTL_DEIE(x)		\
			((x << CL_SD_CTL_DEIE_SHIFT) & CL_SD_CTL_DEIE_MASK)

/* FIFO Limit Change */
#define CL_SD_CTL_FIFOLC_SHIFT		5
#define CL_SD_CTL_FIFOLC_MASK		(1 << CL_SD_CTL_FIFOLC_SHIFT)
#define CL_SD_CTL_FIFOLC(x)		\
			((x << CL_SD_CTL_FIFOLC_SHIFT) & CL_SD_CTL_FIFOLC_MASK)

/* Stripe Control */
#define CL_SD_CTL_STRIPE_SHIFT		16
#define CL_SD_CTL_STRIPE_MASK		(0x3 << CL_SD_CTL_STRIPE_SHIFT)
#define CL_SD_CTL_STRIPE(x)		\
			((x << CL_SD_CTL_STRIPE_SHIFT) & CL_SD_CTL_STRIPE_MASK)

/* Traffic Priority */
#define CL_SD_CTL_TP_SHIFT		18
#define CL_SD_CTL_TP_MASK		(1 << CL_SD_CTL_TP_SHIFT)
#define CL_SD_CTL_TP(x)			\
			((x << CL_SD_CTL_TP_SHIFT) & CL_SD_CTL_TP_MASK)

/* Bidirectional Direction Control */
#define CL_SD_CTL_DIR_SHIFT		19
#define CL_SD_CTL_DIR_MASK		(1 << CL_SD_CTL_DIR_SHIFT)
#define CL_SD_CTL_DIR(x)		\
			((x << CL_SD_CTL_DIR_SHIFT) & CL_SD_CTL_DIR_MASK)

/* Stream Number */
#define HDA_CL_SD_CTL_STREAM_TAG_SHIFT		20
#define HDA_CL_SD_CTL_STREAM_TAG_MASK		(0xf << HDA_CL_SD_CTL_STREAM_TAG_SHIFT)

/* CL: Stream Descriptor x Status */

/* Buffer Completion Interrupt Status */
#define CL_SD_STS_BCIS(x)		CL_SD_CTL_IOCE(x)

/* FIFO Error */
#define CL_SD_STS_FIFOE(x)		CL_SD_CTL_FEIE(x)

/* Descriptor Error */
#define CL_SD_STS_DESE(x)		CL_SD_CTL_DEIE(x)

/* FIFO Ready */
#define CL_SD_STS_FIFORDY(x)	CL_SD_CTL_FIFOLC(x)


/* CL: Stream Descriptor x Last Valid Index */
#define CL_SD_LVI_SHIFT			0
#define CL_SD_LVI_MASK			(0xff << CL_SD_LVI_SHIFT)
#define CL_SD_LVI(x)			((x << CL_SD_LVI_SHIFT) & CL_SD_LVI_MASK)

/* CL: Stream Descriptor x FIFO Eviction Watermark */
#define CL_SD_FIFOW_SHIFT		0
#define CL_SD_FIFOW_MASK		(0x7 << CL_SD_FIFOW_SHIFT)
#define CL_SD_FIFOW(x)			\
			((x << CL_SD_FIFOW_SHIFT) & CL_SD_FIFOW_MASK)

/* CL: Stream Descriptor x Buffer Descriptor List Pointer Lower Base Address */

/* Protect Bits */
#define CL_SD_BDLPLBA_PROT_SHIFT	0
#define CL_SD_BDLPLBA_PROT_MASK		(1 << CL_SD_BDLPLBA_PROT_SHIFT)
#define CL_SD_BDLPLBA_PROT(x)		\
		((x << CL_SD_BDLPLBA_PROT_SHIFT) & CL_SD_BDLPLBA_PROT_MASK)

/* Buffer Descriptor List Lower Base Address */
#define CL_SD_BDLPLBA_SHIFT		7
#define CL_SD_BDLPLBA_MASK		(0x1ffffff << CL_SD_BDLPLBA_SHIFT)
#define CL_SD_BDLPLBA(x)		\
	((BDL_ALIGN(lower_32_bits(x)) << CL_SD_BDLPLBA_SHIFT) & CL_SD_BDLPLBA_MASK)

/* Buffer Descriptor List Upper Base Address */
#define CL_SD_BDLPUBA_SHIFT		0
#define CL_SD_BDLPUBA_MASK		(0xffffffff << CL_SD_BDLPUBA_SHIFT)
#define CL_SD_BDLPUBA(x)		\
		((upper_32_bits(x) << CL_SD_BDLPUBA_SHIFT) & CL_SD_BDLPUBA_MASK)

/*
 * Code Loader - Software Position Based FIFO
 * Capability Registers x Software Position Based FIFO Header
 */

/* Next Capability Pointer */
#define CL_SPBFIFO_SPBFCH_PTR_SHIFT	0
#define CL_SPBFIFO_SPBFCH_PTR_MASK	(0xff << CL_SPBFIFO_SPBFCH_PTR_SHIFT)
#define CL_SPBFIFO_SPBFCH_PTR(x)	\
		((x << CL_SPBFIFO_SPBFCH_PTR_SHIFT) & CL_SPBFIFO_SPBFCH_PTR_MASK)

/* Capability Identifier */
#define CL_SPBFIFO_SPBFCH_ID_SHIFT	16
#define CL_SPBFIFO_SPBFCH_ID_MASK	(0xfff << CL_SPBFIFO_SPBFCH_ID_SHIFT)
#define CL_SPBFIFO_SPBFCH_ID(x)		\
		((x << CL_SPBFIFO_SPBFCH_ID_SHIFT) & CL_SPBFIFO_SPBFCH_ID_MASK)

/* Capability Version */
#define CL_SPBFIFO_SPBFCH_VER_SHIFT	28
#define CL_SPBFIFO_SPBFCH_VER_MASK	(0xf << CL_SPBFIFO_SPBFCH_VER_SHIFT)
#define CL_SPBFIFO_SPBFCH_VER(x)	\
	((x << CL_SPBFIFO_SPBFCH_VER_SHIFT) & CL_SPBFIFO_SPBFCH_VER_MASK)

/* Software Position in Buffer Enable */
#define CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT	0
#define CL_SPBFIFO_SPBFCCTL_SPIBE_MASK	(1 << CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT)
#define CL_SPBFIFO_SPBFCCTL_SPIBE(x)	\
	((x << CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT) & CL_SPBFIFO_SPBFCCTL_SPIBE_MASK)

/* SST IPC SKL defines */
#define SKL_WAIT_TIMEOUT		500	/* 500 msec */
#define HDA_MAX_BUFFER_SIZE		(32 * PAGE_SIZE)

enum skl_cl_dma_wake_states {
	SKL_CL_DMA_STATUS_NONE = 0,
	SKL_CL_DMA_BUF_COMPLETE,
	SKL_CL_DMA_ERR,	/* TODO: Expand the error states */
};

#endif
