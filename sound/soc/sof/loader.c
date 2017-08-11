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
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <uapi/sound/sof-fw.h>
#include "sof-priv.h"
#include "ops.h"

/* generic module parser for mmaped DSPs */
int snd_sof_parse_module_memcpy(struct snd_sof_dev *sdev,
	struct snd_sof_mod_hdr *module)
{
	struct snd_sof_blk_hdr *block;
	int count;
	u32 offset;

	dev_dbg(sdev->dev, "new module size 0x%x blocks 0x%x type 0x%x\n",
		module->size, module->num_blocks, module->type);

	block = (void *)module + sizeof(*module);

	for (count = 0; count < module->num_blocks; count++) {

		if (block->size == 0) {
			dev_err(sdev->dev,
				"error: block %d size invalid\n", count);
			return -EINVAL;
		}

		switch (block->type) {
		case SOF_BLK_IMAGE:
		case SOF_BLK_CACHE:
		case SOF_BLK_REGS:
		case SOF_BLK_SIG:
		case SOF_BLK_ROM:
			continue;	/* not handled atm */
		case SOF_BLK_TEXT:
		case SOF_BLK_DATA:
			offset = block->offset;
			break;
		default:
			dev_err(sdev->dev, "error: bad type 0x%x for block 0x%x\n",
				block->type, count);
			return -EINVAL;
		}

		dev_dbg(sdev->dev, "block %d type 0x%x "
			"size 0x%x ==>  offset 0x%x\n",
			count, block->type, block->size, offset);

		snd_sof_dsp_block_write(sdev, offset, 
			(void*)block + sizeof(*block), block->size);

		/* next block */
		block = (void *)block + sizeof(*block) + block->size;
	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_parse_module_memcpy);

static int check_header(struct snd_sof_dev *sdev, const struct firmware *fw)
{
	struct snd_sof_fw_header *header;

	/* Read the header information from the data pointer */
	header = (struct snd_sof_fw_header *)fw->data;

	/* verify FW sig */
	if (strncmp(header->sig, SND_SOF_FW_SIG, SND_SOF_FW_SIG_SIZE) != 0) {
		dev_err(sdev->dev, "error: invalid firmware signature\n");
		return -EINVAL;
	}

	/* check size is valid */
	if (fw->size != header->file_size + sizeof(*header)) {
		dev_err(sdev->dev, "error: invalid filesize mismatch got 0x%lx expected 0x%lx\n",
			fw->size, header->file_size + sizeof(*header));
		return -EINVAL;
	}

	dev_dbg(sdev->dev, "header size=0x%x modules=0x%x abi=0x%x size=%zu\n",
		header->file_size, header->num_modules,
		header->abi, sizeof(*header));

	return 0;
}

static int load_modules(struct snd_sof_dev *sdev, const struct firmware *fw)
{
	struct snd_sof_fw_header *header;
	struct snd_sof_mod_hdr *module;
	int (*load_module)(struct snd_sof_dev *sof_dev,
		struct snd_sof_mod_hdr *hdr);
	int ret, count;

	header = (struct snd_sof_fw_header *)fw->data;
	load_module = sdev->ops->load_module;
	if (load_module == NULL)
		return -EINVAL;

	/* parse each module */
	module = (void *)fw->data + sizeof(*header);
	for (count = 0; count < header->num_modules; count++) {

		/* module */
		ret = load_module(sdev, module);
		if (ret < 0) {
			dev_err(sdev->dev, "error: invalid module %d\n", count);
			return ret;
		}
		module = (void *)module + sizeof(*module) + module->size;
	}

	return 0;
}

int snd_sof_load_firmware_memcpy(struct snd_sof_dev *sdev,
	const struct firmware *fw)
{
	int ret;
	
	/* make sure the FW header and file is valid */
	ret = check_header(sdev, fw);
	if (ret < 0) {
		dev_err(sdev->dev, "error: invalid FW header\n");
		return ret;
	}

	/* prepare the DSP for FW loading */
	ret = snd_sof_dsp_reset(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to reset DSP\n");
		return ret;
	}

	/* parse and load firmware modules to DSP */
	ret = load_modules(sdev, fw);
	if (ret < 0) {
		dev_err(sdev->dev, "error: invalid FW modules\n");
		return ret;
	}
	
	return ret;
}
EXPORT_SYMBOL(snd_sof_load_firmware_memcpy);

int snd_sof_load_firmware(struct snd_sof_dev *sdev,
	const struct firmware *fw)
{	
	dev_dbg(sdev->dev, "loading firmware\n");

	if (sdev->ops->load_firmware)
		return sdev->ops->load_firmware(sdev, fw);
	return 0;
}
EXPORT_SYMBOL(snd_sof_load_firmware);

int snd_sof_run_firmware(struct snd_sof_dev *sdev)
{
	int ret;

	init_waitqueue_head(&sdev->boot_wait);
	sdev->boot_complete = false;

	dev_dbg(sdev->dev, "booting DSP firmware\n");

	/* boot the firmware on the DSP */
	ret = snd_sof_dsp_run(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to reset DSP\n");
		return ret;
	}

	/* now wait for the DSP to boot */
	ret = wait_event_timeout(sdev->boot_wait, sdev->boot_complete,
		msecs_to_jiffies(sdev->boot_timeout));
	if (ret == 0) {
		dev_err(sdev->dev, "error: firmware boot timeout\n");
		snd_sof_dsp_dbg_dump(sdev, SOF_DBG_REGS | SOF_DBG_MBOX |
			SOF_DBG_TEXT | SOF_DBG_PCI);
		return -EIO;
	} else
		dev_info(sdev->dev, "firmware boot complete\n");

	return 0;
}
EXPORT_SYMBOL(snd_sof_run_firmware);

void snd_sof_fw_unload(struct snd_sof_dev *sdev)
{

}
EXPORT_SYMBOL(snd_sof_fw_unload);

