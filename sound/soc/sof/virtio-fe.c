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
 * Author: Luo Xionghu <xionghu.luo@intel.com>
 *         Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

/*
 * virt IO FE driver
 *
 * The SOF driver thinks this driver is another audio DSP, however the calls
 * made by the SOF driver core do not directly go to HW, but over a virtIO
 * message Q to the virtIO BE driver.
 *
 * The virtIO message Q will use the *exact* same IPC structures as we currently
 * use in the mailbox.
 *
 * Guest OS SOF core -> SOF FE -> virtIO Q -> SOF BE -> System OS SOF core -> DSP
 *
 * The mailbox IO and TX/RX msg functions below will do IO on the virt IO Q.
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
//#include <trace/events/sofadsp.h>
#include <linux/device.h>
#include <linux/virtio.h>
#include <sound/sof.h>
#include <uapi/sound/sof-fw.h>

#include "sof-priv.h"
#include "ops.h"
#include "intel.h"


/*
 * IPC Firmware ready.
 */
static int virtio_fe_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	/* not needed for FE ? */
	return 0;
}

/*
 * IPC Mailbox IO
 */

static void virtio_fe_mailbox_write(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	/* write data to message Q buffer before sending message */
}

static void virtio_fe_mailbox_read(struct snd_sof_dev *sdev, u32 offset,
	void *message, size_t bytes)
{
	/* read data from message Q buffer after receiving message */
}


static int virtio_fe_tx_busy(struct snd_sof_dev *sdev)
{
	/* return 1 if tx message Q is busy */
}

static int virtio_fe_tx_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	/* write msg to the virtio queue message for BE */

	return 0;
}

static int virtio_fe_rx_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{

	/* read the virtio queue message from BE and copy to msg */
	return 0;
}


/*
 * Probe and remove.
 */

static int virtio_fe_probe(struct snd_sof_dev *sdev)
{
	/* register virtio device */

	/* conenct virt queues to BE */
}

static int virtio_fe_remove(struct snd_sof_dev *sdev)
{
	/* free virtio resurces and unregister device */
}

/* baytrail ops */
struct snd_sof_dsp_ops snd_sof_virtio_fe_ops = {

	/* device init */
	.probe		= virtio_fe_probe,
	.remove		= virtio_fe_remove,

	/* mailbox */
	.mailbox_read	= virtio_fe_mailbox_read,
	.mailbox_write	= virtio_fe_mailbox_write,

	/* ipc */
	.tx_msg		= virtio_fe_tx_msg,
	.rx_msg		= virtio_fe_rx_msg,
	.fw_ready	= virtio_fe_fw_ready,
	.tx_busy	= virtio_fe_tx_busy,

	/* module loading */
//	.load_module	= snd_sof_parse_module_memcpy,

	/*Firmware loading */
 	.load_firmware	= snd_sof_load_firmware_memcpy,
};
EXPORT_SYMBOL(snd_sof_virtio_fe_ops);

MODULE_LICENSE("Dual BSD/GPL");
