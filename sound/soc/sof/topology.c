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
#include <linux/string.h>
#include <sound/soc-topology.h>
#include <sound/soc.h>
#include <uapi/sound/sof-ipc.h>
#include <uapi/sound/sof-topology.h>
#include "sof-priv.h"


/* 
 * Extended Tokens.
 *
 * Extended tokens are optional attributes for firmware objects and are parsed
 * after the firmware object has been created. The tokens are then classified
 * and then sent to the firmware object using the IPC ABI for that data.  
 */

static void sof_process_ext_tokens(struct snd_soc_component *scomp, u32 id,
	struct snd_soc_tplg_private *private)
{
	/* process all extended token types */
	// TODO: support any extended tokens.
}

/*
 * Standard Kcontrols.
 */
static int sof_control_load_volume(struct snd_soc_component *scomp,
	struct snd_sof_control *scontrol, struct snd_kcontrol_new *kc,
	struct snd_soc_tplg_ctl_hdr *hdr, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_mixer_control *mc =
		(struct snd_soc_tplg_mixer_control *)hdr;
	struct sof_ipc_comp_volume v;
	int i;

	/* validate topology data */
	if (mc->num_channels >= SND_SOC_TPLG_MAX_CHAN)
		return -EINVAL;

	/* init the volume control IPC */
	memset(&v, 0, sizeof(v));
	v.comp.hdr.size = sizeof(v);
	v.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_NEW;
	v.comp.id = scontrol->comp_id = sdev->next_comp_id++;
	v.comp.type = SOF_COMP_VOLUME;
	v.pcm.format = 0;
	v.pcm.frames = 0;
	v.pcm.channels = 0;
	v.channels = scontrol->num_channels = mc->num_channels;
	v.min_value = mc->min;
	v.max_value = mc->max;
	// TODO: TLV
	//v.step_size = 

	dev_dbg(sdev->dev, "tplg: load kcontrol index %d\n", scontrol->comp_id);

	/* configure channel IDs */
	for (i = 0; i < mc->num_channels; i++) {
		v.pcm.chmap[i] = mc->channel[i].id;
	}

	/* send IPC to the DSP */
 	return sof_ipc_tx_message_wait(sdev->ipc, 
		v.comp.hdr.cmd, &v, sizeof(v), r, sizeof(*r));
}


/* external kcontrol init - used for any driver specific init */
static int sof_control_load(struct snd_soc_component *scomp, int index,
	struct snd_kcontrol_new *kc, struct snd_soc_tplg_ctl_hdr *hdr)
{
	struct soc_mixer_control *sm;
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_dobj *dobj = NULL;
	struct snd_sof_control *scontrol;
	struct sof_ipc_comp_reply r;
	int ret = -EINVAL;

	dev_dbg(sdev->dev, "tplg: load control type %d name : %s\n", 
		hdr->type, hdr->name);

	scontrol = kzalloc(sizeof(*scontrol), GFP_KERNEL);
	if (scontrol == NULL)
		return -ENOMEM;

	scontrol->sdev = sdev;
	mutex_init(&scontrol->mutex);

	switch (hdr->ops.info) {
	case SND_SOC_TPLG_CTL_VOLSW:
	case SND_SOC_TPLG_CTL_VOLSW_SX:
	case SND_SOC_TPLG_CTL_VOLSW_XR_SX:
		sm = (struct soc_mixer_control *)kc->private_value;
		dobj = &sm->dobj;
		ret = sof_control_load_volume(scomp, scontrol, kc, hdr, &r);
		break;
	case SND_SOC_TPLG_CTL_ENUM:
	case SND_SOC_TPLG_CTL_BYTES:
	case SND_SOC_TPLG_CTL_ENUM_VALUE:
	case SND_SOC_TPLG_CTL_RANGE:
	case SND_SOC_TPLG_CTL_STROBE:
	case SND_SOC_TPLG_DAPM_CTL_VOLSW:
	case SND_SOC_TPLG_DAPM_CTL_ENUM_DOUBLE:
	case SND_SOC_TPLG_DAPM_CTL_ENUM_VIRT:
	case SND_SOC_TPLG_DAPM_CTL_ENUM_VALUE:
	case SND_SOC_TPLG_DAPM_CTL_PIN:
	default:
		dev_warn(sdev->dev, "control type not supported %d:%d:%d\n",
			hdr->ops.get, hdr->ops.put, hdr->ops.info);
		return 0;
	}

	if (ret < 0) {
		kfree(scontrol);
		return ret;
	}

	dobj->private = scontrol;
	scontrol->readback_offset = r.offset;
	list_add(&scontrol->list, &sdev->kcontrol_list);
	return ret;
}

static int sof_control_unload(struct snd_soc_component *scomp,
	struct snd_soc_dobj *dobj)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_free fcomp;
	struct snd_sof_control *scontrol = dobj->private;

	dev_dbg(sdev->dev, "tplg: unload control name : %s\n", scomp->name);

	fcomp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_FREE;
	fcomp.hdr.size = sizeof(fcomp);
	fcomp.id = scontrol->comp_id;

	/* send IPC to the DSP */
 	return sof_ipc_tx_message_wait(sdev->ipc, 
		fcomp.hdr.cmd, &fcomp, sizeof(fcomp), NULL, 0);
}

static int sof_connect_dai_widget(struct snd_soc_component *scomp,
	struct snd_soc_dapm_widget *w,
	struct snd_soc_tplg_dapm_widget *tw)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_card *card = scomp->card;
	struct snd_soc_pcm_runtime *rtd;

	list_for_each_entry(rtd, &card->rtd_list, list) {

		if (!strcmp(rtd->dai_link->stream_name, w->sname)) {
			switch (w->id) {
			case snd_soc_dapm_dai_out:
				rtd->cpu_dai->capture_widget = w;
				dev_dbg(sdev->dev, "tplg: connected widget %s -> DAI link %s\n",
					w->name, rtd->dai_link->name);
				break;
			case snd_soc_dapm_dai_in:
				rtd->cpu_dai->playback_widget = w;
				dev_dbg(sdev->dev, "tplg: connected widget %s -> DAI link %s\n",
					w->name, rtd->dai_link->name);
				break;
			default:
				break;
			}
		}
	}

	return 0;
}

static void sof_dai_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_dai *dai, struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		case SOF_TKN_DAI_DMAC:
			dai->dmac_id = elem->value;
			break;
		case SOF_TKN_DAI_DMAC_CHAN:
			dai->dmac_chan = elem->value;
			break;
		case SOF_TKN_DAI_DMAC_CONFIG:
			dai->dmac_config = elem->value;
			break;
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected DAI token %d\n",
				elem->token);
			break;
		}

	}
}

static int sof_widget_dai_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_dai *dai)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array = private->array;
	int size = private->size, asize;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid DAI size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_dai_get_words(scomp, swidget, tw, dai, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in DAI data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}



static int sof_widget_load_dai(struct snd_soc_component *scomp, int index,
	struct snd_sof_widget *swidget,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_comp_dai dai;
	int ret = -EINVAL, ext_tokens;

	/* configure dai IPC message */
	memset(&dai, 0, sizeof(dai));
	dai.comp.hdr.size = sizeof(dai);
	dai.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_NEW;
	dai.comp.id = swidget->comp_id;
	dai.comp.type = SOF_COMP_DAI;
	dai.comp.pipeline_id = index;

	/* get the rest from private data i.e. tuples */
	ext_tokens = sof_widget_dai_get_data(scomp, swidget, tw, &dai);
	if (ext_tokens < 0) {
		dev_err(sdev->dev, "error: failed to get DAI private data\n");
		return ret;
	}

	ret = sof_ipc_tx_message_wait(sdev->ipc, 
		dai.comp.hdr.cmd, &dai, sizeof(dai), r, sizeof(*r));
	if (ret < 0) {

	}

	if (ext_tokens == 0)
		return ret;

	/* process any optional extended data for this component */
	sof_process_ext_tokens(scomp, swidget->comp_id, &tw->priv);
	return ret;
}

static void sof_buffer_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_buffer *buffer,
	struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		case SOF_TKN_BUF_SIZE:
			buffer->size = elem->value;
			break;
		case SOF_TKN_BUF_PRELOAD:
			buffer->preload_count = elem->value;
			break;
		// TODO buffer type, i.e. LP/HP etc
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected buffer token %d\n",
				elem->token);
			break;
		}
	}
}

static int sof_widget_buffer_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_buffer *buffer)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array = private->array;
	int size = private->size, asize;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid buffer structure size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_buffer_get_words(scomp, swidget, tw, buffer, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in buffer data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}

static int sof_widget_load_buffer(struct snd_soc_component *scomp, int index,
	struct snd_sof_widget *swidget,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_buffer buffer;
	int ret;

	/* configure dai IPC message */
	memset(&buffer, 0, sizeof(buffer));
	buffer.comp.hdr.size = sizeof(buffer);
	buffer.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_BUFFER_NEW;
	buffer.comp.id = swidget->comp_id ;
	buffer.comp.type = SOF_COMP_BUFFER;
	buffer.comp.pipeline_id = index;

	/* get the rest from private data i.e. tuples */
	ret = sof_widget_buffer_get_data(scomp, swidget, tw, &buffer);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to get buffer private data\n");
		return ret;
	}

	return sof_ipc_tx_message_wait(sdev->ipc, 
		buffer.comp.hdr.cmd, &buffer, sizeof(buffer), r, sizeof(*r));
}

static void sof_pcm_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_host *host,
	struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected host token %d\n",
				elem->token);
			break;
		}

	}
}

static int sof_widget_pcm_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_host *host)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array = private->array;
	int size = private->size, asize;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid host size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_pcm_get_words(scomp, swidget, tw, host, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in host data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}

static int sof_widget_load_pcm(struct snd_soc_component *scomp, int index,
	struct snd_sof_widget *swidget, enum sof_ipc_stream_direction dir,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_comp_host host;
	int ret;

	/* configure mixer IPC message */
	memset(&host, 0, sizeof(host));
	host.comp.hdr.size = sizeof(host);
	host.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_NEW;
	host.comp.id = swidget->comp_id;
	host.comp.type = SOF_COMP_HOST;
	host.comp.pipeline_id = index;
	host.direction = dir;

	/* get the rest from private data i.e. tuples */
	ret = sof_widget_pcm_get_data(scomp, swidget, tw, &host);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to get host private data\n");
		return ret;
	}

	return sof_ipc_tx_message_wait(sdev->ipc, 
		host.comp.hdr.cmd, &host, sizeof(host), r, sizeof(*r));
}


static void sof_pipeline_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_pipe_new *pipeline,
	struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		case SOF_TKN_SCHED_DEADLINE:
			pipeline->deadline = elem->value;
			break;
		case SOF_TKN_SCHED_PRIORITY:
			pipeline->priority = elem->value;
			break;
		case SOF_TKN_SCHED_MIPS:
			pipeline->mips = elem->value;
			break;
		case SOF_TKN_SCHED_CORE:
			pipeline->core = elem->value;
			break;
		case SOF_TKN_SCHED_FRAMES:
			pipeline->frames_per_sched = elem->value;
			break;
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected pipeline token %d\n",
				elem->token);
			break;
		}

	}
	dev_dbg(sdev->dev, "pipeline %s: deadline %d pri %d mips %d core %d frames %d\n",
		swidget->widget->name, pipeline->deadline, pipeline->priority,
		pipeline->mips, pipeline->core, pipeline->frames_per_sched);
}

static int sof_widget_pipeline_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_pipe_new *pipeline)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array = private->array;
	int size = private->size, asize;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid buffer size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_pipeline_get_words(scomp, swidget, tw, pipeline, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in pipeline data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}

static int sof_widget_load_pipeline(struct snd_soc_component *scomp,
	int index, struct snd_sof_widget *swidget,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_pipe_new pipeline;
	struct snd_sof_widget *comp_swidget;
	int ret;

	/* configure dai IPC message */
	memset(&pipeline, 0, sizeof(pipeline));
	pipeline.hdr.size = sizeof(pipeline);
	pipeline.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_PIPE_NEW;
	pipeline.pipeline_id = index;
	pipeline.comp_id = swidget->comp_id;

	/* component at start of pipeline is our stream id */
	comp_swidget = snd_sof_find_swidget(sdev, tw->sname);
	if (comp_swidget == NULL) {
		dev_err(sdev->dev, "error: widget %s refers to non existant widget %s\n",
			tw->name, tw->sname);
		//return -EINVAL;
	} else
		pipeline.sched_id = comp_swidget->comp_id;

	dev_dbg(sdev->dev, "tplg: pipeline id %d comp id %d\n",
		pipeline.pipeline_id, pipeline.comp_id);

	/* get the rest from private data i.e. tuples */
	ret = sof_widget_pipeline_get_data(scomp, swidget, tw, &pipeline);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to get pipeline private data\n");
		return ret;
	}

	return sof_ipc_tx_message_wait(sdev->ipc, 
		pipeline.hdr.cmd, &pipeline, sizeof(pipeline), r, sizeof(*r));
}

static void sof_mixer_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_mixer *mixer,
	struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected buffer token %d\n",
				elem->token);
			break;
		}

	}
}

static int sof_widget_mixer_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_mixer *mixer)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array = private->array;
	int size = private->size, asize;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid buffer size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_mixer_get_words(scomp, swidget, tw, mixer, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in buffer data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}

static int sof_widget_load_mixer(struct snd_soc_component *scomp, int index,
	struct snd_sof_widget *swidget,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_comp_mixer mixer;
	int ret;

	/* configure mixer IPC message */
	memset(&mixer, 0, sizeof(mixer));
	mixer.comp.hdr.size = sizeof(mixer);
	mixer.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_NEW;
	mixer.comp.id = swidget->comp_id;
	mixer.comp.type = SOF_COMP_MIXER;
	mixer.comp.pipeline_id = index;

	/* get the rest from private data i.e. tuples */
	ret = sof_widget_mixer_get_data(scomp, swidget, tw, &mixer);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to get mixer private data\n");
		return ret;
	}

	return sof_ipc_tx_message_wait(sdev->ipc, 
		mixer.comp.hdr.cmd, &mixer, sizeof(mixer), r, sizeof(*r));
}

static void sof_pga_get_words(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_volume *volume,
	struct snd_soc_tplg_vendor_array *array)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_vendor_value_elem *elem;
	int i;

	for (i = 0; i < array->num_elems; i++) {

		elem = &array->value[i];

		switch (elem->token) {
		case SOF_TKN_VOLUME_RAMP_STEP_TYPE:
			volume->ramp = elem->value;
			break;
		case SOF_TKN_VOLUME_RAMP_STEP_MS:
			volume->initial_ramp = elem->value;
			break;
		default:
			/* non fatal */
			dev_info(sdev->dev, "info: unexpected volume token %d\n",
				elem->token);
			break;
		}

	}
}

static int sof_widget_pga_get_data(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget, struct snd_soc_tplg_dapm_widget *tw,
	struct sof_ipc_comp_volume *volume)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_soc_tplg_private *private = &tw->priv;
	struct snd_soc_tplg_vendor_array *array;
	int size, asize;

	/* skip widget private data as its not used for volume */
	private = (void*)private + private->size;
	array = private->array;
	size = private->size;

	/* private data can be made up of multiple arrays */
	while (size) {

		asize = array->size;

		/* validate size */
		size -= asize;
		if (size < 0) {
			dev_err(sdev->dev, "error: invalid volume size 0x%x\n",
				asize);
			return -EINVAL;
		} 

		switch (array->type) {
		case SND_SOC_TPLG_TUPLE_TYPE_WORD:
			sof_pga_get_words(scomp, swidget, tw, volume, array);
			break;
		case SND_SOC_TPLG_TUPLE_TYPE_STRING:
		case SND_SOC_TPLG_TUPLE_TYPE_BOOL:
		case SND_SOC_TPLG_TUPLE_TYPE_BYTE:
		case SND_SOC_TPLG_TUPLE_TYPE_SHORT:
		case SND_SOC_TPLG_TUPLE_TYPE_UUID:
		default:
			/* non fatal - can be skipped */
			dev_info(sdev->dev, "info: unsupported type %d found in volume data\n",
				array->type);
			break;
		}

		/* next array */
		array = (void*)array + asize;
	}

	return 0;
}

static int sof_widget_load_pga(struct snd_soc_component *scomp, int index,
	struct snd_sof_widget *swidget,
	struct snd_soc_tplg_dapm_widget *tw, struct sof_ipc_comp_reply *r)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_comp_volume volume;
	int ret;

	if (tw->num_kcontrols != 1) {
		dev_err(sdev->dev, "error: invalid kcontrol count %d for volume\n",
			tw->num_kcontrols);
		//return -EINVAL;
	}

	/* configure dai IPC message */
	memset(&volume, 0, sizeof(volume));
	volume.comp.hdr.size = sizeof(volume);
	volume.comp.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_NEW;
	volume.comp.id = swidget->comp_id;
	volume.comp.type = SOF_COMP_VOLUME;
	volume.comp.pipeline_id = index;

	/* get the rest from kcontrol (not widget) private data i.e. tuples */
	ret = sof_widget_pga_get_data(scomp, swidget, tw, &volume);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to get volume private data\n");
		return ret;
	}

	return sof_ipc_tx_message_wait(sdev->ipc, 
		volume.comp.hdr.cmd, &volume, sizeof(volume), r, sizeof(*r));
}


static int sof_widget_load(struct snd_soc_component *scomp, int index,
	struct snd_soc_dapm_widget *w,
	struct snd_soc_tplg_dapm_widget *tw)
{
	return 0;
}

/* external widget init - used for any driver specific init */
static int sof_widget_ready(struct snd_soc_component *scomp, int index,
	struct snd_soc_dapm_widget *w,
	struct snd_soc_tplg_dapm_widget *tw)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_sof_widget *swidget;
	struct sof_ipc_comp_reply reply;
	int ret = 0;

	swidget = kzalloc(sizeof(*swidget), GFP_KERNEL);
	if (swidget == NULL)
		return -ENOMEM;

	swidget->sdev = sdev;
	swidget->widget = w;
	swidget->comp_id = sdev->next_comp_id++;
	swidget->complete = 0;
	swidget->id = w->id;
	swidget->pipeline_id = index;
	memset(&reply, 0, sizeof(reply));

	dev_dbg(sdev->dev, "tplg: ready widget id %d type %d name : %s stream %s\n",
		swidget->comp_id, tw->id, tw->name,
		tw->sname ? tw->sname : "none");

	/* handle any special case widgets */
	switch (w->id) {
	case snd_soc_dapm_dai_in:
	case snd_soc_dapm_dai_out:
		ret = sof_widget_load_dai(scomp, index, swidget, tw, &reply);
		if (ret == 0)
			sof_connect_dai_widget(scomp, w, tw);
		break;
	case snd_soc_dapm_mixer:
		ret = sof_widget_load_mixer(scomp, index, swidget, tw, &reply);
		break;
	case snd_soc_dapm_pga:
		ret = sof_widget_load_pga(scomp, index, swidget, tw, &reply);
		break;
	case snd_soc_dapm_buffer:
		ret = sof_widget_load_buffer(scomp, index, swidget, tw, &reply);
		break;
	case snd_soc_dapm_scheduler:
		ret = sof_widget_load_pipeline(scomp, index, swidget, tw,
			&reply);
		break;
	case snd_soc_dapm_aif_in:
		ret = sof_widget_load_pcm(scomp, index, swidget,
			SOF_IPC_STREAM_CAPTURE, tw, &reply);
		break;
	case snd_soc_dapm_aif_out:
		ret = sof_widget_load_pcm(scomp, index, swidget,
			SOF_IPC_STREAM_PLAYBACK, tw, &reply);
		break;
	case snd_soc_dapm_mux:
	case snd_soc_dapm_demux:
	case snd_soc_dapm_switch:
	case snd_soc_dapm_siggen:
	case snd_soc_dapm_dai_link:
	case snd_soc_dapm_kcontrol:
	case snd_soc_dapm_effect:
	default:
		dev_warn(sdev->dev, "warning: widget type %d name %s not handled\n",
			tw->id, tw->name);
		break;
	}

	/* check IPC return value */
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to add widget id %d type %d name : %s stream %s\n",
			tw->shift, tw->id, tw->name,
			tw->sname ? tw->sname : "none");
		//return ret;
	}

	/* check IPC reply */
	if (reply.hdr.cmd & SOF_IPC_REPLY_ERROR) {
		dev_err(sdev->dev, "error: DSP failed to add widget id %d type %d name : %s stream %s reply 0x%x\n",
			tw->shift, tw->id, tw->name,
			tw->sname ? tw->sname : "none", reply.hdr.cmd);
		//return ret; // TODO:
	}

	w->dobj.private = swidget;
	mutex_init(&swidget->mutex);
	list_add(&swidget->list, &sdev->widget_list);
	return ret;
}

static int sof_widget_unload(struct snd_soc_component *scomp,
	struct snd_soc_dobj *dobj)
{
	return 0;
}


/* FE DAI - used for any driver specific init */
static int sof_dai_load(struct snd_soc_component *scomp, int index,
	struct snd_soc_dai_driver *dai_drv,
	struct snd_soc_tplg_pcm *pcm, struct snd_soc_dai *dai)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_sof_pcm *spcm;

	/* dont need to do anything for BEs atm */
	if (pcm == NULL)
		return 0;

	spcm = kzalloc(sizeof(*spcm), GFP_KERNEL);
	if (spcm == NULL)
		return -ENOMEM;

	spcm->sdev = sdev;
	spcm->pcm = *pcm;
	dai_drv->dobj.private = spcm;
	mutex_init(&spcm->mutex);
	list_add(&spcm->list, &sdev->pcm_list);

	dev_dbg(sdev->dev, "tplg: load pcm %d index %d %s to dai %d %s\n", 
		pcm->pcm_id, spcm->comp_id, pcm->pcm_name, pcm->dai_id,
		pcm->dai_name);
		
	return 0;
}

static int sof_dai_unload(struct snd_soc_component *scomp,
	struct snd_soc_dobj *dobj)
{
	struct snd_sof_pcm *spcm = dobj->private;

	list_del(&spcm->list);
	kfree(&spcm);

	return 0;
}

/* DAI link - used for any driver specific init */
static int sof_link_load(struct snd_soc_component *scomp, int index,
	struct snd_soc_dai_link *link)
{
	link->platform_name = "sof-audio";
	return 0;
}

static int sof_link_unload(struct snd_soc_component *scomp,
	struct snd_soc_dobj *dobj)
{
	return 0;
}

/* bind PCM ID to host component ID */
static int spcm_bind(struct snd_soc_component *scomp, struct snd_sof_pcm *spcm,
	const char *host)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_sof_widget *host_widget;

	host_widget = snd_sof_find_swidget(sdev, (char*)host);
	if (host_widget == NULL) {
		dev_err(sdev->dev, "error: cant find host component %s\n",
			host);
		return -ENODEV;
	}

	switch (host_widget->id) {
	case snd_soc_dapm_aif_in:
	case snd_soc_dapm_aif_out:
		spcm->comp_id = host_widget->comp_id;
		return 0;
	default:
		dev_err(sdev->dev, "error: host is wrong type %d\n",
			host_widget->id);
		return -EINVAL;
	}
}

/* DAI link - used for any driver specific init */
static int sof_route_load(struct snd_soc_component *scomp, int index,
	struct snd_soc_dapm_route *route)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_pipe_comp_connect connect;
	struct snd_sof_widget *source_swidget, *sink_swidget;
	struct snd_sof_pcm *spcm;
	struct sof_ipc_comp_reply reply;
	int ret;

	memset(&connect, 0, sizeof(connect));
	connect.hdr.size = sizeof(connect);
	connect.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_COMP_CONNECT;

	dev_dbg(sdev->dev, "sink %s control %s source %s\n",
		route->sink, route->control ? route->control : "none",
		route->source);

	/* source component */
	source_swidget = snd_sof_find_swidget(sdev, (char*)route->source);
	if (source_swidget == NULL) {

		/* dont send any routes to DSP that include a driver PCM */
		spcm = snd_sof_find_spcm_name(sdev, (char*)route->source);
		if (spcm)
			return spcm_bind(scomp, spcm, route->sink);

		dev_err(sdev->dev, "error: source %s not found\n",
			route->source);
		//return -EINVAL;
	} else
		connect.source_id = source_swidget->comp_id;

	/* sink component */
	sink_swidget = snd_sof_find_swidget(sdev, (char*)route->sink);
	if (sink_swidget == NULL) {

		/* dont send any routes to DSP that include a driver PCM */
		spcm = snd_sof_find_spcm_name(sdev, (char*)route->sink);
		if (spcm)
			return spcm_bind(scomp, spcm, route->source);

		dev_err(sdev->dev, "error: sink %s not found\n",
			route->sink);
		//return -EINVAL;
	} else
		connect.sink_id = sink_swidget->comp_id;

	ret = sof_ipc_tx_message_wait(sdev->ipc, 
		connect.hdr.cmd, &connect, sizeof(connect), &reply,
		sizeof(reply));

	/* check IPC return value */
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to add route sink %s control %s source %s\n",
			route->sink, route->control ? route->control : "none",
			route->source);
		return ret;
	}

	/* check IPC reply */
	if (reply.hdr.cmd & SOF_IPC_REPLY_ERROR) {
		dev_err(sdev->dev, "error: DSP failed to add route sink %s control %s source %s result 0x%x\n",
			route->sink, route->control ? route->control : "none",
			route->source, reply.hdr.cmd);
		//return ret; // TODO:
	}

	return ret;
}

static int sof_route_unload(struct snd_soc_component *scomp,
	struct snd_soc_dobj *dobj)
{
	return 0;
}

static int sof_complete_pipeline(struct snd_soc_component *scomp,
	struct snd_sof_widget *swidget)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct sof_ipc_pipe_ready ready;
	int ret;

	dev_dbg(sdev->dev, "tplg: complete pipeline %d\n", swidget->comp_id);

	memset(&ready, 0, sizeof(ready));
	ready.hdr.size = sizeof(ready);
	ready.hdr.cmd = SOF_IPC_GLB_TPLG_MSG | SOF_IPC_TPLG_PIPE_COMPLETE;
	ready.comp_id = swidget->comp_id;

	ret = sof_ipc_tx_message_wait(sdev->ipc, 
		ready.hdr.cmd, &ready, sizeof(ready), NULL, 0);
	if (ret < 0)
		return ret;
	return 1;
}

/* completion - called at completion of firmware loading */
static void sof_complete(struct snd_soc_component *scomp)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(scomp);
	struct snd_sof_widget *swidget;

	/* some widget types require completion notificattion */
	list_for_each_entry(swidget, &sdev->widget_list, list) {

		if (swidget->complete)
			continue;

		switch (swidget->id) {
		case snd_soc_dapm_scheduler:
			swidget->complete = sof_complete_pipeline(scomp,
				swidget);
			break;
		default:
			break;
		}
	}	
}

/* manifest - optional to inform component of manifest */
static int sof_manifest(struct snd_soc_component *scomp, int index,
	struct snd_soc_tplg_manifest *man)
{
	return 0;
}

/* vendor specific kcontrol handlers available for binding */
static const struct snd_soc_tplg_kcontrol_ops sof_io_ops[] = {
	{SOF_TPLG_KCTL_VOL_ID, snd_sof_volume_get, snd_sof_volume_put},
};

/* vendor specific bytes ext handlers available for binding */
static const struct snd_soc_tplg_bytes_ext_ops sof_bytes_ext_ops[] = {
{},
};

static struct snd_soc_tplg_ops sof_tplg_ops = {

	/* external kcontrol init - used for any driver specific init */
	.control_load	= sof_control_load,
	.control_unload	= sof_control_unload,

	/* external kcontrol init - used for any driver specific init */
	.dapm_route_load	= sof_route_load,
	.dapm_route_unload	= sof_route_unload,

	/* external widget init - used for any driver specific init */
	.widget_load	= sof_widget_load,
	.widget_ready	= sof_widget_ready,
	.widget_unload	= sof_widget_unload,

	/* FE DAI - used for any driver specific init */
	.dai_load	= sof_dai_load,
	.dai_unload	= sof_dai_unload,

	/* DAI link - used for any driver specific init */
	.link_load	= sof_link_load,
	.link_unload	= sof_link_unload,

	/* completion - called at completion of firmware loading */
	.complete	= sof_complete,

	/* manifest - optional to inform component of manifest */
	.manifest	= sof_manifest,

	/* vendor specific kcontrol handlers available for binding */
	.io_ops 	= sof_io_ops,
	.io_ops_count	= ARRAY_SIZE(sof_io_ops),

	/* vendor specific bytes ext handlers available for binding */
	.bytes_ext_ops	= sof_bytes_ext_ops,
	.bytes_ext_ops_count	= ARRAY_SIZE(sof_bytes_ext_ops),
};

int snd_sof_init_topology(struct snd_sof_dev *sdev,
	struct snd_soc_tplg_ops *ops)
{
	/* TODO: support linked list of topologies */
	sdev->tplg_ops = ops;
	return 0;
}
EXPORT_SYMBOL(snd_sof_init_topology);

int snd_sof_load_topology(struct snd_sof_dev *sdev, const char *file)
{
	const struct firmware *fw;
	struct snd_soc_tplg_hdr *hdr;
	int ret;

	dev_dbg(sdev->dev, "loading topology\n");

	ret = request_firmware(&fw, file, sdev->dev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: tplg %s load failed with %d\n",
				file, ret);
		return ret;
	}

	hdr = (struct snd_soc_tplg_hdr *)fw->data;
	ret = snd_soc_tplg_component_load(sdev->component,
			&sof_tplg_ops, fw, SND_SOC_TPLG_INDEX_ALL);
	if (ret < 0) {
		dev_err(sdev->dev, "error: tplg component load failed %d\n", ret);
		release_firmware(fw);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_load_topology);

void snd_sof_free_topology(struct snd_sof_dev *sdev)
{

}
EXPORT_SYMBOL(snd_sof_free_topology);
