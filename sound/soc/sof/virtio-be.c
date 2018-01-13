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
 *         
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
#include <virtio.h>
#include <uapi/sound/sof-fw.h>
#include "sof-priv.h"
#include "ops.h"

/* BE driver
 *
 * This driver will create IO Queues for communition from FE drivers.
 * The FE driver will send real IPC structures over the queue and then 
 * the BE driver will send the structures directlt to the DSP. The BE will
 * get the IPC reply from the DSP and send it back to the FE over the queue.
 *
 * The virt IO message Q handlers in this file will :-
 *
 * 1) Check that the message is valid and not for any componenets that dont
 *    belong to the guest.
 *
 * 2) Call snd_sof_dsp_tx_msg(struct snd_sof_dev *sdev,
 *	struct snd_sof_ipc_msg *msg) to send the message to the DSP.
 *
 * Replies will be sent back using a similar method.
 */

static int sof_virtio_validate(struct virtio_device *dev)
{
	/* do we need this func ?? */
	return 0;
}

static int sof_virtio_probe(struct virtio_device *dev)
{
	/* register fe device with sof core */
	//snd_sof_virtio_register_fe(dev);

	/* create our virtqueues */s

	/* send topology data to fe via virtq */

	return 0;
}

static void sof_virtio_remove(struct virtio_device *dev)
{
	/* remove topology from fe via virtqueue */

	/* destroy virtqueue */
}

#ifdef CONFIG_PM
static int sof_virtio_freeze(struct virtio_device *dev)
{
	/* pause and suspend any streams for this FE */
	return 0;
}

static int sof_virtio_restore(struct virtio_device *dev)
{
	/* restore and unpause any streams for this FE */
	return 0;
}
#endif


/* IDs of FEs */
static const struct virtio_device_id *fe_id_table[] + {
};

static struct virtio_driver sof_be_virtio_driver = {
	.driver = {
		.name = "sof-virtio-be",
		.owner = THIS_MODULE,
	},

	.id_table = fe_id_table,

	//const unsigned int *feature_table;
	//unsigned int feature_table_size;
	//const unsigned int *feature_table_legacy;
	//unsigned int feature_table_size_legacy;
	
	validate = sof_virtio_validate,
	probe = sof_virtio_probe,
	remove = sof_virtio_remove,

#ifdef CONFIG_PM
	freeze = sof_virtio_freeze,
	restore = sof_virtio_restore,
#endif
};

/* this will be called by sof core when core is ready */
int sof_virtio_register(struct snd_sof_dev *sdev)
{
	int ret;

	ret = register_virtio_driver(&sof_be_virtio_driver);
	/* do we need to do anythig else here */
	return ret;
}

/* called by sof core when driver is removed */
void sof_virtio_unregister(struct snd_sof_dev *sdev)
{
	unregister_virtio_driver(&sof_be_virtio_driver);
	/* do we need to do anythig else here */
}
