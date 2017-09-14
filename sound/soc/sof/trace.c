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

#if 0
struct dma_trace_buffer {
	u32 size; /* total sg elems size */
	int read_offset; /* the read ptr in rcurrent */
	int write_offset; /* the write ptr in wcurrent */
	int pending_size; /* the data size pending to copy to userspace */
	struct list_head elem_list;
	struct list_head *rcurrent;
	struct list_head *wcurrent;
	u32 current_end;
};

static int sst_dma_trace_open(struct inode *inode, struct file *file)
{
	struct sst_hsw *hsw = inode->i_private;
	struct sst_hsw_ipc_debug_log_enable_req *dtrace_req =
		&hsw->dt_enable_request;
	struct sst_hsw_ipc_debug_log_reply reply;
	struct dma_trace_buffer *hbuf = &hsw->host_buffer;
	u32 header;
	int ret;

	file->private_data = hsw;

	reply.log_buffer_size = 0;
	hbuf->read_offset = hbuf->write_offset = 0;
	hbuf->pending_size = 0;
	hbuf->rcurrent = hbuf->wcurrent = hbuf->elem_list.next;

	header = IPC_GLB_TYPE(IPC_GLB_DEBUG_LOG_MESSAGE);

	dtrace_req->config[0] = IPC_DEBUG_ENABLE_LOG;
	dtrace_req->config[1] = PAGE_SIZE/2;
	ret = sst_ipc_tx_message_wait(&hsw->ipc, header, dtrace_req,
			  sizeof(*dtrace_req), &reply, sizeof(reply));
	if (ret < 0) {
		dev_err(hsw->dev, "error: stream comint failed\n");
		return ret;
	}

	return 0;
}

static int sst_dma_trace_release(struct inode *inode, struct file *file)
{
	struct sst_hsw *hsw = inode->i_private;
	struct sst_hsw_ipc_debug_log_enable_req *dtrace_req =
		&hsw->dt_enable_request;
	struct sst_hsw_ipc_debug_log_reply reply;
	u32 header;
	int ret;

	header = IPC_GLB_TYPE(IPC_GLB_DEBUG_LOG_MESSAGE);

	dtrace_req->config[0] = IPC_DEBUG_DISABLE_LOG;
	dtrace_req->config[1] = PAGE_SIZE/2;
	ret = sst_ipc_tx_message_wait(&hsw->ipc, header, dtrace_req,
			  sizeof(*dtrace_req), &reply, sizeof(reply));
	if (ret < 0) {
		dev_err(hsw->dev, "error: stream comint failed\n");
		return ret;
	}

	return 0;
}

static inline struct list_head *next_dma_trace_sg(struct sst_hsw *hsw,
						  struct list_head *list)
{
	if (list_is_last(list, &hsw->host_buffer.elem_list))
		return hsw->host_buffer.elem_list.next;
	else
		return list->next;
}

static ssize_t sst_dma_trace_read(struct file *file, char __user *buffer,
					  size_t count, loff_t *ppos)
{
	struct sst_hsw *hsw = file->private_data;
	struct dma_trace_buffer *hbuf = &hsw->host_buffer;
	struct dma_trace_sg *sg_elem;
	int sg_size, msg_size, cnt = 0, ret;

	memset(buffer, 0, count);
	while (hbuf->pending_size > 0) {
		sg_elem = list_entry(hbuf->rcurrent, struct dma_trace_sg, list);
		sg_size = sg_elem->size;
		/* the dma trace buffer is a ring sg buffer
		 * msg_size should be:
		 * 1. if rcurrent != wcurrent, msg_size is sg_size - read_offset
		 * 2. if rcurrent == wcurrent
		 *  1) if pending_size > (sg_size - read_offset),
		 *     the ring is rolled over
		 *     (sg_size - read_offset) data should be transferred
		 *  2) if pending_size <= (sg_size - read_offset),
		 *     transfer all the pending_size data
		 */
		msg_size = min(hbuf->pending_size, sg_size - hbuf->read_offset);
		ret = copy_to_user(buffer, sg_elem->buf + hbuf->read_offset,
				   msg_size);
		ret = msg_size - ret;
		cnt += ret;
		hbuf->read_offset += ret;
		if (hbuf->read_offset == sg_size) {
			hbuf->rcurrent = next_dma_trace_sg(hsw, hbuf->rcurrent);
			hbuf->read_offset = 0;
		}
		hbuf->pending_size -= ret;
	}

	msleep(100);
	return cnt + 1;
}

static const struct file_operations sst_dma_trace_fops = {
	.open = sst_dma_trace_open,
	.read = sst_dma_trace_read,
	.llseek = default_llseek,
	.release = sst_dma_trace_release,
};
#endif

static int sof_dfsentry_trace_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t sof_dfsentry_trace_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	struct snd_sof_dfsentry *dfse = file->private_data;
	int size;
	loff_t pos = *ppos;
	size_t ret;

	size = dfse->size;

	if (pos < 0)
		return -EINVAL;
	if (pos >= size || !count)
		return 0;
	if (count > size - pos)
		count = size - pos;

	size = (count + 3) & ~3;

	ret = copy_to_user(buffer, dfse->buf + pos, count);

	if (ret == count)
		return -EFAULT;
	count -= ret;
	*ppos = pos + count;

	return count;
}

static const struct file_operations sof_dfs_trace_fops = {
	.open = sof_dfsentry_trace_open,
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
	int ret = 0;
	struct sof_ipc_dma_trace_params params;
	struct sof_ipc_reply ipc_reply;
	struct sof_ipc_hdr hdr;
	dev_dbg(sdev->dev, "snd_sof_init_trace() start!!!\n");

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
		return ret;
	}

	/* craete compressed page table for audio firmware */
	ret = snd_sof_create_page_table(sdev, &sdev->dmatb, sdev->dmatp.area,
		sdev->dmatb.bytes);
	if (ret < 0)
		return ret;

	sdev->dma_trace_pages = ret;
	dev_dbg(sdev->dev, "dma_trace_pages: %d\n", sdev->dma_trace_pages);

	ret = trace_debugfs_create(sdev);
	if (ret < 0)
		return ret;

	/* set IPC parameters */
	hdr.size = sizeof(hdr);
	hdr.cmd = SOF_IPC_GLB_TRACE_MSG | SOF_IPC_TRACE_DMA_INIT;

	params.hdr.size = sizeof(params);
	params.hdr.cmd = SOF_IPC_GLB_TRACE_MSG | SOF_IPC_TRACE_DMA_PARAMS;
	params.buffer.phy_addr = sdev->dmatp.addr;
	params.buffer.size = sdev->dmatb.bytes;
	params.buffer.offset = 0;
	params.buffer.pages = sdev->dma_trace_pages;

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message_wait(sdev->ipc,
		hdr.cmd, &hdr, sizeof(hdr),
		&ipc_reply, sizeof(ipc_reply));
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: cant initialize DMA for Trace%d\n", ret);
		return ret;
	}

	ret = sof_ipc_tx_message_wait(sdev->ipc,
		params.hdr.cmd, &params, sizeof(params),
		&ipc_reply, sizeof(ipc_reply));
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: cant set params for DMA for Trace%d\n", ret);
		return ret;
	}

	dev_dbg(sdev->dev, "snd_sof_init_trace() end!!!\n");
	return 0;
}
EXPORT_SYMBOL(snd_sof_init_trace);
