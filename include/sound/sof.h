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

#ifndef __INCLUDE_SOUND_SOF_H
#define __INCLUDE_SOUND_SOF_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <uapi/sound/sof-ipc.h>

struct snd_sof_dsp_ops;

/*
 * SOF Platform data.
 */
struct snd_sof_pdata {
	u32 id;		/* PCI/ACPI ID */
	const struct firmware *fw;
	const char *drv_name;
	const char *name;

	/* parent devices */
	struct device *dev;
	struct pci_dev *pci;
	struct platform_device *pdev;

	/* descriptor */
	const struct sof_dev_desc *desc;

	/* machine */
	struct platform_device *pdev_mach;
	const struct snd_sof_machine *machine;
};


/* 
 * Descriptor for ASoC machine driver.
 * This data is used to determine the correct machine driver to use depending
 * on DSP ID and codec ID. TODO: also include DMI name for matching
 */
struct snd_sof_machine {
	/* ACPI ID for the codec */
	const u8 codec_id[ACPI_ID_LEN];
	/* machine driver name */
	const char *drv_name;
	/* firmware file name */
	const char *fw_filename;
	/* default topology */
	const char *tplg_filename;
	/* ASoC platform name - used for binding machine drivers if non NULL */
	const char *asoc_plat_name;
	/* machine specific ops */
	const struct snd_sof_dsp_ops *ops;
	/* machine driver private data fixup */
	struct platform_device * (*new_mach_data)
		(struct snd_sof_pdata *sof_pdata);
	/* machine detection helper */
	int (*confirm_mach)(struct device *dev,
		const struct sof_dev_desc **desc);
};

/* 
 * Descriptor used for setting up SOF platform data. This is used when
 * ACPI/PCI data is missing or mapped differently.
 */
struct sof_dev_desc {
	/* list of machines using this configuration */
	const struct snd_sof_machine *machines;

	/* Platform resource indexes in BAR / ACPI resources. */ 
	/* Must set to -1 if not used - add new items to end */
	int resindex_lpe_base;
	int resindex_pcicfg_base;
	int resindex_imr_base;
	int irqindex_host_ipc;
	int resindex_dma_base;

	/* DMA only valid when resindex_dma_base != -1*/
	int dma_engine;
	int dma_size;

	/* IPC timeouts in ms */
	int ipc_timeout;
	int boot_timeout;

	/* defaults for no codec mode */
	const char *nocodec_fw_filename;
	const char *nocodec_tplg_filename;
};

#endif
