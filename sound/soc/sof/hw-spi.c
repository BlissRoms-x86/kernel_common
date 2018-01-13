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

/*
 * Hardware interface for audio DSPs via SPI
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
#include <linux/spi/spi.h>
#include <linux/of_device.h>

#include <linux/device.h>
#include <sound/sof.h>
#include <uapi/sound/sof-fw.h>

#include "sof-priv.h"
#include "ops.h"
#include "intel.h"


/*
 * Memory copy.
 */

static void spi_block_write(struct snd_sof_dev *sdev, u32 offset, void *src,
	size_t size)
{
	// use spi_write() to copy data to DSP
}

static void spi_block_read(struct snd_sof_dev *sdev, u32 offset, void *dest,
	size_t size)
{
	// use spi_read() to copy data from DSP
}

/*
 * IPC Firmware ready.
 */
static int spi_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct sof_ipc_fw_ready *fw_ready = &sdev->fw_ready;
	struct sof_ipc_fw_version *v = &fw_ready->version;

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x\n", msg_id);

	// read local buffer with SPI data

#if 0
	/* copy data from the DSP FW ready offset */
	spi_block_read(sdev, offset, fw_ready, sizeof(*fw_ready));

	snd_sof_dsp_mailbox_init(sdev, fw_ready->dspbox_offset,
		fw_ready->dspbox_size, fw_ready->hostbox_offset,
		fw_ready->hostbox_size);

	dev_dbg(sdev->dev, " mailbox DSP initiated 0x%x - size 0x%x\n",
		fw_ready->dspbox_offset, fw_ready->dspbox_size);
	dev_dbg(sdev->dev, " mailbox Host initiated 0x%x - size 0x%x\n",
		fw_ready->hostbox_offset, fw_ready->hostbox_size);

#endif
	dev_info(sdev->dev, " Firmware info: version %d:%d-%s build %d on %s:%s\n", 
		v->major, v->minor, v->tag, v->build, v->date, v->time);

	return 0;
}

/*
 * IPC Mailbox IO
 */

static void spi_mailbox_write(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *dest = sdev->bar[sdev->mailbox_bar] + offset;

	//memcpy_toio(dest, message, bytes);

	// this will copy to a local memory buffer that will be sent to DSP via SPI at next IPC

}

static void spi_mailbox_read(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	void __iomem *src = sdev->bar[sdev->mailbox_bar] + offset;

	//memcpy_fromio(message, src, bytes);
	// this will copy from a local memory buffer that will be received from DSP via SPI at last IPC
}

/*
 * IPC Doorbell IRQ handler and thread.
 */

static irqreturn_t spi_irq_handler(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	int ret = IRQ_NONE;

	// on SPI based devices this will likely come via a SoC GPIO IRQ

	// check if GPIO is assetred and if so run thread.

	return ret;
}

static irqreturn_t spi_irq_thread(int irq, void *context)
{
	struct snd_sof_dev *sdev = (struct snd_sof_dev *) context;
	
	// read SPI data into local buffer and determine IPC cmd or reply
		
	/* if reply. Handle Immediate reply from DSP Core  and set DSP state to ready */
	//snd_sof_ipc_reply(sdev, ipcx);

		
	/* if cmd, Handle messages from DSP Core */
	//snd_sof_ipc_msgs_rx(sdev);

	/* continue to send any remaining messages... */
	snd_sof_ipc_msgs_tx(sdev);

	return IRQ_HANDLED;
}

static int spi_is_ready(struct snd_sof_dev *sdev)
{
	// use local variable to store DSP comand state. either DSP is ready
	// for new cmd or still processing current cmd.

	return 1;
}

static int spi_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	u64 cmd = msg->header;

	/* send the message */
	spi_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data, msg->msg_size);

	return 0;
}

static int spi_get_reply(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct sof_ipc_reply reply;
	int ret = 0;
	u32 size;

	/* get reply */
	spi_mailbox_read(sdev, sdev->host_box.offset, &reply, sizeof(reply));
	if (reply.error < 0) {
		size = sizeof(reply);
		ret = reply.error;
	} else {
		/* reply correct size ? */
		if (reply.hdr.size != msg->reply_size) {
			dev_err(sdev->dev, "error: reply expected 0x%lx got 0x%x bytes\n",
				msg->reply_size, reply.hdr.size);
			size = msg->reply_size;
			ret = -EINVAL;
		} else {
			size = reply.hdr.size;
		}
	}

	/* read the message */
	if (msg->msg_data && size > 0)
		spi_mailbox_read(sdev, sdev->host_box.offset, msg->reply_data, size);

	return ret;
}


/*
 * Probe and remove.
 */

static int spi_sof_probe(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *pdata = sdev->pdata;
	const struct sof_dev_desc *desc = pdata->desc;
	struct platform_device *pdev =
		container_of(sdev->parent, struct platform_device, dev);
	int ret = 0;

	/* get IRQ from Device tree or ACPI - register our IRQ */
#if 0
	sdev->ipc_irq = platform_get_irq(pdev, desc->irqindex_host_ipc);
	if (sdev->ipc_irq < 0) {
		dev_err(sdev->dev, "error: failed to get IRQ at index %d\n",
			desc->irqindex_host_ipc);
		ret = sdev->ipc_irq;
		goto irq_err;
	}
#endif
	dev_dbg(sdev->dev, "using IRQ %d\n", sdev->ipc_irq);
	ret = request_threaded_irq(sdev->ipc_irq, spi_irq_handler,
		spi_irq_thread, IRQF_SHARED, "AudioDSP", sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to register IRQ %d\n",
			sdev->ipc_irq);
		goto irq_err;		
	}

	return ret;
}

static int spi_sof_remove(struct snd_sof_dev *sdev)
{
	free_irq(sdev->ipc_irq, sdev);
	return 0;
}


/* baytrail ops */
struct snd_sof_dsp_ops snd_sof_spi_ops = {

	/* device init */
	.probe		= spi_sof_probe,
	.remove		= spi_sof_remove,

	/* Block IO */
	.block_read	= spi_block_read,
	.block_write	= spi_block_write,

	/* doorbell */
	.irq_handler	= spi_irq_handler,
	.irq_thread	= spi_irq_thread,

	/* mailbox */
	.mailbox_read	= spi_mailbox_read,
	.mailbox_write	= spi_mailbox_write,

	/* ipc */
	.send_msg	= spi_send_msg,
	.get_reply	= spi_get_reply,
	.fw_ready	= spi_fw_ready,
	.is_ready	= spi_is_ready,
	.cmd_done	= spi_cmd_done,

	/* debug */
	.debug_map	= spi_debugfs,
	.debug_map_count	= ARRAY_SIZE(spi_debugfs),
	.dbg_dump	= spi_dump,

	/* module loading */
	.load_module	= snd_sof_parse_module_memcpy,

	/*Firmware loading */
 	.load_firmware	= snd_sof_load_firmware_memcpy,
};
EXPORT_SYMBOL(snd_sof_spi_ops);

MODULE_LICENSE("Dual BSD/GPL");
