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
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"

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

int snd_sof_init_trace(struct snd_sof_dev *sof_dev)
{
	return 0;
}
EXPORT_SYMBOL(snd_sof_init_trace);

