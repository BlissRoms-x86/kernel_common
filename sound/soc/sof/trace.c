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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched/signal.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <uapi/sound/sof-ipc.h>
#include <uapi/sound/sof-fw.h>
#include "sof-priv.h"
#include "ops.h"

static int sof_wait_trace_avail(struct snd_sof_dev *sdev, size_t *count, loff_t pos)
{
	size_t avail;
	wait_queue_entry_t wait;

	/* if pos is invalid for DMA trace host buffer*/
	/* return error code */
	if (sdev->host_offset < pos)
		return -EINVAL;

	/* if there is available trace data now */
	/* it is unecessary to wait */
	if (sdev->host_offset > pos)
		goto _endcheck;

	/* wait for available trace data from FW */
	init_waitqueue_entry(&wait, current);
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&sdev->trace_sleep, &wait);

	if (signal_pending(current)) {
		remove_wait_queue(&sdev->trace_sleep, &wait);
		goto _endcheck;
	}

	/* set timeout to max value, no error code */
	schedule_timeout(MAX_SCHEDULE_TIMEOUT);
	remove_wait_queue(&sdev->trace_sleep, &wait);

_endcheck:
	/* calculate the available count */
	avail = sdev->host_offset - pos;

	/* return min value between available and request count */
	*count = avail < *count ? avail : *count;

	return 0;
}

static ssize_t sof_dfsentry_trace_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	struct snd_sof_dfsentry *dfse = file->private_data;
	struct snd_sof_dev *sdev = dfse->sdev;
	int err;
	loff_t pos = *ppos;
	size_t ret, size;

	size = dfse->size;

	/* check pos and count */
	if (pos < 0)
		return -EINVAL;
	if (pos >= size || !count)
		return 0;
	if (count > size - pos)
		count = size - pos;

	/* get available count based on current host offset */
	err = sof_wait_trace_avail(sdev, &count, pos);
	if (err < 0 || count == 0) {
		dev_err(sdev->dev,
			"error: cant get more trace %d\n", err);
		return 0;
	}

	/* copy available trace data to debugfs */
	ret = copy_to_user(buffer, dfse->buf + pos, count);

	if (ret == count)
		return -EFAULT;
	count -= ret;

	/* move debugfs reading position */
	*ppos = pos + count;

	return count;
}

static const struct file_operations sof_dfs_trace_fops = {
	.open = simple_open,
	.read = sof_dfsentry_trace_read,
	.llseek = default_llseek,
};

static int trace_debugfs_create(struct snd_sof_dev *sdev)
{
	struct snd_sof_dfsentry *dfse;

	if (!sdev)
		return -EINVAL;

	dfse = kzalloc(sizeof(*dfse), GFP_KERNEL);
	if (!dfse)
		return -ENOMEM;

	dfse->buf = sdev->dmatb.area;
	dfse->size = sdev->dmatb.bytes;
	dfse->sdev = sdev;

	dfse->dfsentry = debugfs_create_file("trace", 0444, sdev->debugfs_root,
					     dfse, &sof_dfs_trace_fops);
	if (!dfse->dfsentry) {
		dev_err(sdev->dev,
			"error: cannot create debugfs entry for trace\n");
		kfree(dfse);
		return -ENODEV;
	}

	return 0;
}

int snd_sof_init_trace(struct snd_sof_dev *sdev)
{
	struct sof_ipc_dma_trace_params params;
	struct sof_ipc_reply ipc_reply;
	int ret;

	/* allocate trace page table buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, sdev->parent,
		PAGE_SIZE, &sdev->dmatp);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: cant alloc page table for trace %d\n", ret);
		return ret;
	}

	/* allocate trace data buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_SG, sdev->parent,
		DMA_BUF_SIZE_FOR_TRACE, &sdev->dmatb);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: cant alloc buffer for trace%d\n", ret);
		goto page_err;
	}

	/* craete compressed page table for audio firmware */
	ret = snd_sof_create_page_table(sdev, &sdev->dmatb, sdev->dmatp.area,
		sdev->dmatb.bytes);
	if (ret < 0)
		goto table_err;

	sdev->dma_trace_pages = ret;
	dev_dbg(sdev->dev, "dma_trace_pages: %d\n", sdev->dma_trace_pages);

	ret = trace_debugfs_create(sdev);
	if (ret < 0)
		goto table_err;

	/* set IPC parameters */
	params.hdr.size = sizeof(params);
	params.hdr.cmd = SOF_IPC_GLB_TRACE_MSG | SOF_IPC_TRACE_DMA_PARAMS;
	params.buffer.phy_addr = sdev->dmatp.addr;
	params.buffer.size = sdev->dmatb.bytes;
	params.buffer.offset = 0;
	params.buffer.pages = sdev->dma_trace_pages;

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc,
		params.hdr.cmd, &params, sizeof(params),
		&ipc_reply, sizeof(ipc_reply));
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: cant set params for DMA for Trace%d\n", ret);
		goto table_err;
	}

	init_waitqueue_head(&sdev->trace_sleep);
	sdev->host_offset = 0;
	return 0;

table_err:
	snd_dma_free_pages(&sdev->dmatb);
page_err:
	snd_dma_free_pages(&sdev->dmatp);
	return ret;
}

int snd_sof_trace_update_pos(struct snd_sof_dev *sdev,
	struct sof_ipc_dma_trace_posn *posn)
{
	if (sdev->host_offset != posn->host_offset) {
		sdev->host_offset = posn->host_offset;
		wake_up(&sdev->trace_sleep);
	}

	return 0;
}

void snd_sof_trace_notify_for_error(struct snd_sof_dev *sdev)
{
	dev_err(sdev->dev, "ASoC: trace wakes up for error!\n");
	wake_up(&sdev->trace_sleep);
}

EXPORT_SYMBOL(snd_sof_init_trace);

void snd_sof_release_trace(struct snd_sof_dev *sdev)
{
	snd_dma_free_pages(&sdev->dmatb);
	snd_dma_free_pages(&sdev->dmatp);
}
EXPORT_SYMBOL(snd_sof_release_trace);
