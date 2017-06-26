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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/pm_runtime.h>
#include <sound/asound.h>
#include <sound/sof.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"
#include "ops.h"

/* IPC message timeout (msecs) */
#define IPC_TIMEOUT_MSECS	300

#define IPC_EMPTY_LIST_SIZE	8

/* SOF generic IPC data */
struct snd_sof_ipc {
	struct snd_sof_dev *sdev;

	/* message work and status */
	wait_queue_head_t wait_txq;
	struct task_struct *tx_thread;
	struct work_struct kwork;
	bool msg_pending;

	/* lists */
	struct list_head tx_list;
	struct list_head rx_list;
	struct list_head empty_list;
};

/* locks held by caller */
static struct snd_sof_ipc_msg *msg_get_empty(struct snd_sof_ipc *ipc)
{
	struct snd_sof_ipc_msg *msg = NULL;

	if (!list_empty(&ipc->empty_list)) {
		msg = list_first_entry(&ipc->empty_list, struct snd_sof_ipc_msg,
			list);
		list_del(&msg->list);
	}

	return msg;
}

static int tx_wait_done(struct snd_sof_ipc *ipc, struct snd_sof_ipc_msg *msg,
	void *reply_data)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct sof_ipc_hdr *hdr = (struct sof_ipc_hdr *)msg->msg_data;
	unsigned long flags;
	int ret;

	/* wait for DSP IPC completion */
	ret = wait_event_timeout(msg->waitq, msg->complete,
		msecs_to_jiffies(IPC_TIMEOUT_MSECS));

	spin_lock_irqsave(&sdev->spinlock, flags);
	if (ret == 0) {
		dev_err(sdev->dev, "error: ipc timed out for 0x%x size 0x%x\n",
			hdr->cmd, hdr->size);
		list_del(&msg->list);
		snd_sof_dsp_dbg_dump(ipc->sdev, SOF_DBG_REGS | SOF_DBG_MBOX);
		ret = -ETIMEDOUT;
	} else {
		/* copy the data returned from DSP */
		if (msg->reply_size)
			memcpy(reply_data, msg->reply_data, msg->reply_size);
		ret = 0;
	}

	/* return message body to empty list */
	list_add_tail(&msg->list, &ipc->empty_list);

	spin_unlock_irqrestore(&sdev->spinlock, flags);
	return ret;
}

static int ipc_tx_message(struct snd_sof_ipc *ipc, u64 header,
	void *msg_data, size_t msg_bytes, void *reply_data, 
	size_t reply_bytes, int wait)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct snd_sof_ipc_msg *msg;
	unsigned long flags;

	spin_lock_irqsave(&sdev->spinlock, flags);

	msg = msg_get_empty(ipc);
	if (msg == NULL) {
		spin_unlock_irqrestore(&sdev->spinlock, flags);
		return -EBUSY;
	}

	msg->header = header;
	msg->msg_size = msg_bytes;
	msg->reply_size = reply_bytes;
	msg->wait = wait;
	msg->complete = false;

	if (msg_bytes)
		memcpy(msg->msg_data, msg_data, msg_bytes);

	list_add_tail(&msg->list, &ipc->tx_list);
	schedule_work(&ipc->kwork);
	spin_unlock_irqrestore(&sdev->spinlock, flags);

	if (wait)
		return tx_wait_done(ipc, msg, reply_data);
	else
		return 0;
}

static void ipc_tx_next_msg(struct work_struct *work)
{
	struct snd_sof_ipc *ipc =
		container_of(work, struct snd_sof_ipc, kwork);
	struct snd_sof_dev *sdev = ipc->sdev;
	struct snd_sof_ipc_msg *msg;

	spin_lock_irq(&sdev->spinlock);

	if (list_empty(&ipc->tx_list))
		goto out;

	msg = list_first_entry(&ipc->tx_list, struct snd_sof_ipc_msg, list);
	list_move(&msg->list, &ipc->rx_list);

	snd_sof_dsp_tx_msg(sdev, msg);

out:
	spin_unlock_irq(&sdev->spinlock);
}


struct snd_sof_ipc_msg *sof_ipc_reply_find_msg(struct snd_sof_ipc *ipc, u32 header)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct snd_sof_ipc_msg *msg;

	header = SOF_IPC_MESSAGE_ID(header);

	if (list_empty(&ipc->rx_list))
		goto err;

	list_for_each_entry(msg, &ipc->rx_list, list) {
		if (SOF_IPC_MESSAGE_ID(msg->header) == header)
			return msg;
	}

err:
	dev_err(sdev->dev, "error: rx list empty but received 0x%x\n",
			header);
	return NULL;
}
EXPORT_SYMBOL(sof_ipc_reply_find_msg);

/* locks held by caller */
void sof_ipc_tx_msg_reply_complete(struct snd_sof_ipc *ipc,
	struct snd_sof_ipc_msg *msg)
{
	if (!msg->wait)
		list_add_tail(&msg->list, &ipc->empty_list);
	else {
		msg->complete = true;
		wake_up(&msg->waitq);
	}
}

void sof_ipc_drop_all(struct snd_sof_ipc *ipc)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct snd_sof_ipc_msg *msg, *tmp;
	unsigned long flags;

	/* drop all TX and Rx messages before we stall + reset DSP */
	spin_lock_irqsave(&sdev->spinlock, flags);

	list_for_each_entry_safe(msg, tmp, &ipc->tx_list, list) {
		list_move(&msg->list, &ipc->empty_list);
		dev_err(sdev->dev, "error: dropped msg %d\n", msg->header);
	}

	list_for_each_entry_safe(msg, tmp, &ipc->rx_list, list) {
		list_move(&msg->list, &ipc->empty_list);
		dev_err(sdev->dev, "error: dropped reply %d\n", msg->header);
	}

	spin_unlock_irqrestore(&sdev->spinlock, flags);
}
EXPORT_SYMBOL(sof_ipc_drop_all);

int sof_ipc_tx_message_wait(struct snd_sof_ipc *ipc, u32 header,
	void *tx_data, size_t tx_bytes, void *rx_data, size_t rx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes,
		rx_data, rx_bytes, 1);
}
EXPORT_SYMBOL(sof_ipc_tx_message_wait);

int sof_ipc_tx_message_nowait(struct snd_sof_ipc *ipc, u32 header,
	void *tx_data, size_t tx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes, NULL, 0, 0);
}
EXPORT_SYMBOL(sof_ipc_tx_message_nowait);

void snd_sof_ipc_process_reply(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct snd_sof_ipc_msg *msg;
	uint32_t reply = msg_id & SOF_CMD_TYPE_MASK;

	msg = sof_ipc_reply_find_msg(sdev->ipc, msg_id);
	if (msg == NULL) {
		dev_err(sdev->dev, "error: can't find message header 0x%x",
			msg_id);
		return;
	}

	switch (reply) {
	case SOF_IPC_REPLY_SUCCESS:
		break;
	case SOF_IPC_REPLY_ERROR:
	default:
		break;
	}

	/* wake up and return the error if we have waiters on this message ? */
	list_del(&msg->list);
	sof_ipc_tx_msg_reply_complete(sdev->ipc, msg);
}
EXPORT_SYMBOL(snd_sof_ipc_process_reply);

int snd_sof_dsp_mailbox_init(struct snd_sof_dev *sdev, u32 inbox,
		size_t inbox_size, u32 outbox, size_t outbox_size)
{
	sdev->inbox.offset = inbox;
	sdev->inbox.size = inbox_size;
	sdev->outbox.offset = outbox;
	sdev->outbox.size = outbox_size;
	return 0;
}
EXPORT_SYMBOL(snd_sof_dsp_mailbox_init);

static void sof_ipc_notify_reply(struct snd_sof_dev *sdev, u32 msg_id)
{
	uint32_t reply = msg_id & SOF_CMD_TYPE_MASK;

	switch (reply) {
	case SOF_IPC_REPLY_SUCCESS:
		break;
	case SOF_IPC_REPLY_ERROR:
	default:
		break;
	}

}

void snd_sof_ipc_process_notification(struct snd_sof_dev *sdev, u32 msg_id)
{
	uint32_t cmd;
	int err = -EINVAL;

	cmd = msg_id & SOF_GLB_TYPE_MASK;
	switch (cmd) {
	case SOF_IPC_GLB_REPLY:
		sof_ipc_notify_reply(sdev, msg_id);
		break;
	case SOF_IPC_FW_READY:
		/* check for FW boot completion */
		if (!sdev->boot_complete) {
			if (sdev->ops->fw_ready)
				err = sdev->ops->fw_ready(sdev, msg_id);
			if (err < 0) {
				dev_err(sdev->dev, "DSP firmware boot timeout %d\n",
					err);
				return;
			}

			/* firware boot completed OK */
			sdev->boot_complete = true;
			dev_dbg(sdev->dev, "booting DSP firmware completed\n");
			wake_up(&sdev->boot_wait);
			return;
		}
		break;
	case SOF_IPC_GLB_COMPOUND:
	case SOF_IPC_GLB_TPLG_MSG:
	case SOF_IPC_GLB_PM_MSG:
	case SOF_IPC_GLB_COMP_MSG:
	case SOF_IPC_GLB_STREAM_MSG:
	default:
		dev_err(sdev->dev, "unknown DSP notification 0x%x\n", cmd);
		break;
	}
}
EXPORT_SYMBOL(snd_sof_ipc_process_notification);

void snd_sof_ipc_process_msgs(struct snd_sof_dev *sdev)
{
	schedule_work(&sdev->ipc->kwork);
}
EXPORT_SYMBOL(snd_sof_ipc_process_msgs);

struct snd_sof_ipc *snd_sof_ipc_init(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc *ipc;
	struct snd_sof_ipc_msg *msg;
	int i;

	ipc = devm_kzalloc(sdev->dev, sizeof(*ipc), GFP_KERNEL);
	if (ipc == NULL)
		return NULL; 

	INIT_LIST_HEAD(&ipc->tx_list);
	INIT_LIST_HEAD(&ipc->rx_list);
	INIT_LIST_HEAD(&ipc->empty_list);
	init_waitqueue_head(&ipc->wait_txq);
	INIT_WORK(&ipc->kwork, ipc_tx_next_msg);
	ipc->sdev = sdev;

	/* pre-allocate messages */
	dev_dbg(sdev->dev, "pre-allocate %d IPC messages\n",
		IPC_EMPTY_LIST_SIZE);
	msg = devm_kzalloc(sdev->dev, sizeof(struct snd_sof_ipc_msg) *
		IPC_EMPTY_LIST_SIZE, GFP_KERNEL);
	if (msg == NULL)
		return NULL;
	
	/* pre-allocate message data */	
	for (i = 0; i < IPC_EMPTY_LIST_SIZE; i++) {

		msg->msg_data = devm_kzalloc(sdev->dev, PAGE_SIZE, GFP_KERNEL);
		if (msg->msg_data == NULL)
			return NULL;

		msg->reply_data = devm_kzalloc(sdev->dev, PAGE_SIZE, GFP_KERNEL);
		if (msg->reply_data == NULL)
			return NULL;

		init_waitqueue_head(&msg->waitq);
		list_add(&msg->list, &ipc->empty_list);
		msg++;
	}

	return ipc;
}
EXPORT_SYMBOL(snd_sof_ipc_init);

void snd_sof_ipc_free(struct snd_sof_dev *sdev)
{
	/* TODO: send IPC to prepare DSP for shutdown */

	cancel_work_sync(&sdev->ipc->kwork);
}
EXPORT_SYMBOL(snd_sof_ipc_free);

void snd_sof_ipc_stream_posn(struct snd_sof_dev *sdev,
	struct snd_sof_pcm *spcm, int direction,
	snd_pcm_uframes_t *host, snd_pcm_uframes_t *dai)
{
	struct sof_ipc_stream_posn posn;
	struct sof_ipc_stream stream;
	int err;

	/* read firmware byte counters */
	if (spcm->posn_offset[direction] != 0) {

		/* we can read position via mmaped region */
		snd_sof_dsp_block_read(sdev, spcm->posn_offset[direction],
			&posn, sizeof(posn));

	} else {
		/* read position via slower IPC */
		stream.hdr.size = sizeof(stream);
		stream.hdr.cmd =
			SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_POSITION;
		stream.comp_id = spcm->comp_id;

		/* send IPC to the DSP */
 		err = sof_ipc_tx_message_wait(sdev->ipc, 
			stream.hdr.cmd, &stream, sizeof(stream), 
			&posn, sizeof(posn));
		if (err < 0) {
			dev_err(sdev->dev, "error: failed to get stream %d position\n",
				stream.comp_id);
			return;
		}

	}

	*host = posn.host_posn;
	*dai = posn.dai_posn;
}
EXPORT_SYMBOL(snd_sof_ipc_stream_posn);

int snd_sof_ipc_put_mixer(struct snd_sof_ipc *ipc,
	struct snd_sof_control *scontrol)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct sof_ipc_ctrl_values values;
	int err;

	/* write firmware byte counters */
	if (scontrol->readback_offset != 0) {

		/* we can read value header via mmaped region */
		snd_sof_dsp_block_write(sdev, scontrol->readback_offset,
			scontrol->values, sizeof(scontrol->values));

	} else {
		/* read position via slower IPC */
		values.hdr.size = sizeof(values);
		values.hdr.cmd = SOF_IPC_GLB_COMP_MSG | SOF_IPC_COMP_SET_VOLUME;
		values.comp_id = scontrol->comp_id;
		values.num_values = scontrol->num_channels;

		/* now copy the values */
		memcpy(values.values, scontrol->values,
			sizeof(scontrol->values));

		/* send IPC to the DSP */
 		err = sof_ipc_tx_message_wait(sdev->ipc, 
			values.hdr.cmd, &values, sizeof(values), NULL, 0);
		if (err < 0) {
			dev_err(sdev->dev, "error: failed to set control %d values\n",
				values.comp_id);
			return err;
		}

	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_ipc_put_mixer);

int snd_sof_ipc_get_mixer(struct snd_sof_ipc *ipc,
	struct snd_sof_control *scontrol)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct sof_ipc_ctrl_values values;
	struct sof_ipc_ctrl_get_values get;
	int err;

	/* read firmware byte counters */
	if (scontrol->readback_offset != 0) {

		/* we can read values via mmaped region */
		snd_sof_dsp_block_read(sdev, scontrol->readback_offset,
			scontrol->values, sizeof(scontrol->values));

	} else {
		/* read position via slower IPC */
		get.hdr.size = sizeof(values);
		get.hdr.cmd = SOF_IPC_GLB_COMP_MSG | SOF_IPC_COMP_GET_VOLUME;
		get.comp_id = scontrol->comp_id;

		/* send IPC to the DSP */
 		err = sof_ipc_tx_message_wait(sdev->ipc, 
			get.hdr.cmd, &get, sizeof(get), 
			&values, sizeof(values));
		if (err < 0) {
			dev_err(sdev->dev, "error: faild to get control %d values\n",
				values.comp_id);
			return err;
		}

		/* copy to local values */
		memcpy(scontrol->values, values.values,
			sizeof(scontrol->values));
	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_ipc_get_mixer);

int snd_sof_ipc_put_mixer_chan(struct snd_sof_ipc *ipc,
	struct snd_sof_control *scontrol, int chan, long value)
{
	if (chan >= SOF_IPC_MAX_CHANNELS)
		return -EINVAL;

	scontrol->values[chan].value = value;
	return 0;
}
EXPORT_SYMBOL(snd_sof_ipc_put_mixer_chan);

long snd_sof_ipc_get_mixer_chan(struct snd_sof_ipc *ipc,
	struct snd_sof_control *scontrol, int chan)
{
	if (chan >= SOF_IPC_MAX_CHANNELS)
		return 0;
	return scontrol->values[chan].value;
}
EXPORT_SYMBOL(snd_sof_ipc_get_mixer_chan);


#if 0

static void hsw_tx_data_copy(struct snd_sof_ipc_msg *msg, char *tx_data,
	size_t tx_size)
{
	memcpy(msg->tx_data, tx_data, tx_size);
}

static u64 hsw_reply_msg_match(u64 header, u64 *mask)
{
	/* clear reply bits & status bits */
	header &= ~(IPC_STATUS_MASK | IPC_GLB_REPLY_MASK);
	*mask = (u64)-1;

	return header;
}

#endif
