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
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/compress_driver.h>
#include <sound/sof.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"


/* Create DMA buffer page table for DSP */
static int create_page_table(struct snd_pcm_substream *substream,
	unsigned char *dma_area, size_t size)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	struct snd_dma_buffer *dmab = snd_pcm_get_dma_buf(substream);
	int stream = substream->stream;

	return snd_sof_create_page_table(sdev, dmab,
		spcm->stream[stream].page_table.area, size);
}

/* this may get called several times by oss emulation */
static int sof_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	const struct snd_sof_dsp_ops *ops = sdev->ops;
	struct snd_sof_pcm *spcm = rtd->sof;
	struct sof_ipc_pcm_params pcm;
	struct sof_ipc_pcm_params_reply ipc_params_reply;
	int ret;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	dev_dbg(sdev->dev, "pcm: hw params stream %d dir %d\n",
		spcm->pcm.pcm_id, substream->stream);

	memset(&pcm, 0, sizeof(pcm));

	/* allocate audio buffer pages */
	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0) {
		dev_err(sdev->dev, "error: could not allocate %d bytes for PCM %d\n",
			params_buffer_bytes(params), ret);
		return ret;
	}

	/* craete compressed page table for audio firmware */
	ret = create_page_table(substream, runtime->dma_area,
		runtime->dma_bytes);
	if (ret < 0)
		return ret;

	/* number of pages should be rounded up */
	if (runtime->dma_bytes % PAGE_SIZE)
		pcm.params.buffer.pages = (runtime->dma_bytes / PAGE_SIZE) + 1;
	else
		pcm.params.buffer.pages = runtime->dma_bytes / PAGE_SIZE;

	/* set IPC PCM parameters */
	pcm.hdr.size = sizeof(pcm);
	pcm.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_PARAMS;
	pcm.comp_id = spcm->stream[substream->stream].comp_id;
	pcm.params.buffer.phy_addr = spcm->stream[substream->stream].page_table.addr;
	pcm.params.buffer.size = runtime->dma_bytes;
	pcm.params.buffer.offset = 0;
	pcm.params.direction = substream->stream;
	pcm.params.sample_valid_bytes = params_width(params) >> 3;
	pcm.params.buffer_fmt = SOF_IPC_BUFFER_INTERLEAVED;
	pcm.params.rate = params_rate(params);
	pcm.params.channels = params_channels(params);
	pcm.params.host_period_bytes = params_period_bytes(params);

	/* container size */
	switch (params_width(params)) {
	case 16:
		pcm.params.sample_container_bytes = 2;
		break;
	case 24:
		pcm.params.sample_container_bytes = 4;
		break;
	case 32:
		pcm.params.sample_container_bytes = 4;
		break;
	default:
		return -EINVAL;
	}

	/* format */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S16_LE;
		break;
	case SNDRV_PCM_FORMAT_S24:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S24_4LE;
		break;
	case SNDRV_PCM_FORMAT_S32:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S32_LE;
		break;
	case SNDRV_PCM_FORMAT_FLOAT:
		pcm.params.frame_fmt = SOF_IPC_FRAME_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc,
		pcm.hdr.cmd, &pcm, sizeof(pcm), 
		&ipc_params_reply, sizeof(ipc_params_reply));

	/* copy offset */
	//spcm->posn_offset[substream->stream] = ipc_params_reply.posn_offset;

	/* firmware already configured host stream */
	if (ops && ops->host_stream_prepare) {
		pcm.params.stream_tag =
			ops->host_stream_prepare(sdev, substream, params);
		dev_dbg(sdev->dev, "stream_tag %d", pcm.params.stream_tag);
	}

	return ret;
}

static int sof_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	int ret;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	dev_dbg(sdev->dev, "pcm: free stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_FREE;
	stream.comp_id = spcm->stream[substream->stream].comp_id;

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd, &stream,
		sizeof(stream), &reply, sizeof(reply));

	snd_pcm_lib_free_pages(substream);
	return ret;
}

static int sof_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	const struct snd_sof_dsp_ops *ops = sdev->ops;
	int ret = 0;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	dev_dbg(sdev->dev, "pcm: trigger stream %d dir %d cmd %d\n",
		spcm->pcm.pcm_id, substream->stream, cmd);

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG;
	stream.comp_id = spcm->stream[substream->stream].comp_id;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_RELEASE;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_STOP;
		break;		
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_PAUSE;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	default:
		dev_err(sdev->dev, "error: unhandled trigger cmd %d\n", cmd);
		return -EINVAL;
	}

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd, &stream,
		sizeof(stream), &reply, sizeof(reply));

	if (ops && ops->host_stream_trigger)
		ret = ops->host_stream_trigger(sdev, substream, cmd);

	return ret;
}

static snd_pcm_uframes_t sof_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	snd_pcm_uframes_t host = 0, dai = 0;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	/* TODO: call HW position callback */
	host = bytes_to_frames(substream->runtime,
		spcm->stream[substream->stream].posn.host_posn);
	dai = bytes_to_frames(substream->runtime,
		spcm->stream[substream->stream].posn.dai_posn);

	dev_dbg(sdev->dev, "PCM: stream %d dir %d DMA position %lu DAI position %lu\n",
		spcm->pcm.pcm_id, substream->stream, host, dai);

	return host;
}


static int sof_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	struct snd_soc_tplg_stream_caps *caps = 
		&spcm->pcm.caps[substream->stream];
	const struct snd_sof_dsp_ops *ops = sdev->ops;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	dev_dbg(sdev->dev, "pcm: open stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	mutex_lock(&spcm->mutex);

	pm_runtime_get_sync(sdev->dev);

	/* set any runtime constraints based on topology */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_SIZE, caps->period_size_min);
	snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_SIZE, caps->period_size_min);

	/* set runtime config */
	runtime->hw.info = SNDRV_PCM_INFO_MMAP |
			  SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_PAUSE |
			  SNDRV_PCM_INFO_RESUME |
			  SNDRV_PCM_INFO_NO_PERIOD_WAKEUP;
	runtime->hw.formats = caps->formats;
	runtime->hw.period_bytes_min = caps->period_size_min;
	runtime->hw.period_bytes_max = caps->period_size_max;
	runtime->hw.periods_min = caps->periods_min;
	runtime->hw.periods_max = caps->periods_max;
	runtime->hw.buffer_bytes_max = caps->buffer_size_max;

	dev_dbg(sdev->dev, "period min %ld max %ld bytes\n",
		runtime->hw.period_bytes_min,
		runtime->hw.period_bytes_max);
	dev_dbg(sdev->dev, "period count %d max %d\n",
		runtime->hw.periods_min,
		runtime->hw.periods_max);
	dev_dbg(sdev->dev, "buffer max %ld bytes\n",
		runtime->hw.buffer_bytes_max);

	// TODO: create IPC to get this from DSP pipeline
	//runtime->hw.fifo_size = hw->fifo_size;

	/* set wait time - TODO: come from topology */
	snd_pcm_wait_time(substream, 500);

	spcm->stream[substream->stream].posn.host_posn = 0;
	spcm->stream[substream->stream].posn.dai_posn = 0;
	spcm->stream[substream->stream].substream = substream;

	if (ops && ops->host_stream_open)
		ops->host_stream_open(sdev, substream);

	mutex_unlock(&spcm->mutex);
	return 0;
}

static int sof_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	const struct snd_sof_dsp_ops *ops = sdev->ops;

	/* nothing todo for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	dev_dbg(sdev->dev, "pcm: close stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	if (ops && ops->host_stream_close)
		ops->host_stream_close(sdev, substream);

	mutex_lock(&spcm->mutex);
	pm_runtime_mark_last_busy(sdev->dev);
	pm_runtime_put_autosuspend(sdev->dev);
	mutex_unlock(&spcm->mutex);
	return 0;
}

static struct snd_pcm_ops sof_pcm_ops = {
	.open		= sof_pcm_open,
	.close		= sof_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sof_pcm_hw_params,
	.hw_free	= sof_pcm_hw_free,
	.trigger	= sof_pcm_trigger,
	.pointer	= sof_pcm_pointer,
	.page		= snd_pcm_sgbuf_ops_page,
};


static int sof_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_soc_tplg_stream_caps *caps;
	int ret = 0, stream = SNDRV_PCM_STREAM_PLAYBACK;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);

	if (spcm == NULL) {
		dev_warn(sdev->dev, "warn: cant find PCM with DAI ID %d\n",
			rtd->dai_link->id);
		return 0;
	}
	rtd->sof = spcm;

	dev_dbg(sdev->dev, "creating new PCM %s\n", spcm->pcm.pcm_name);

	/* do we need to allocate playback PCM DMA pages */
	if (!spcm->pcm.playback)
		goto capture;

	caps = &spcm->pcm.caps[stream];

	/* pre-allocate playback audio buffer pages */
	dev_dbg(sdev->dev, "spcm: allocate %s playback DMA buffer size 0x%x max 0x%x\n",
		caps->name, caps->buffer_size_min, caps->buffer_size_max);

	ret = snd_pcm_lib_preallocate_pages(pcm->streams[stream].substream,
		SNDRV_DMA_TYPE_DEV_SG, sdev->parent,
		caps->buffer_size_min, caps->buffer_size_max);
	if (ret) {
		dev_err(sdev->dev, "error: cant alloc DMA buffer size 0x%x/0x%x for %s %d\n",
			caps->buffer_size_min, caps->buffer_size_max,
			caps->name, ret);
		return ret;
	}

	/* allocate playback page table buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, sdev->parent,
		PAGE_SIZE, &spcm->stream[stream].page_table);
	if (ret < 0) {
		dev_err(sdev->dev, "error: cant alloc page table for %s %d\n",
			caps->name, ret);
		return ret;
	}

capture:
	stream = SNDRV_PCM_STREAM_CAPTURE;

	/* do we need to allocate capture PCM DMA pages */
	if (!spcm->pcm.capture)
		return ret;

	caps = &spcm->pcm.caps[stream];

	/* pre-allocate capture audio buffer pages */
	dev_dbg(sdev->dev, "spcm: allocate %s capture DMA buffer size 0x%x max 0x%x\n",
		caps->name, caps->buffer_size_min, caps->buffer_size_max);

	ret = snd_pcm_lib_preallocate_pages(pcm->streams[stream].substream,
		SNDRV_DMA_TYPE_DEV_SG, sdev->parent,
		caps->buffer_size_min, caps->buffer_size_max);
	if (ret) {
		dev_err(sdev->dev, "error: cant alloc DMA buffer size 0x%x/0x%x for %s %d\n",
			caps->buffer_size_min, caps->buffer_size_max,
			caps->name, ret);
		snd_dma_free_pages(&spcm->stream[stream].page_table);
		return ret;
	}

	/* allocate capture page table buffer */
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, sdev->parent,
		PAGE_SIZE, &spcm->stream[stream].page_table);
	if (ret < 0) {
		dev_err(sdev->dev, "error: cant alloc page table for %s %d\n",
			caps->name, ret);
		snd_dma_free_pages(&spcm->stream[stream].page_table);
		return ret;
	}

	/* TODO: assign channel maps from topology */
#if 0
	int snd_pcm_add_chmap_ctls(struct snd_pcm *pcm, int stream,
			   const struct snd_pcm_chmap_elem *chmap,
			   int max_channels,
			   unsigned long private_value,
			   struct snd_pcm_chmap **info_ret);
#endif

	return ret;
}

static void sof_pcm_free(struct snd_pcm *pcm)
{
	struct snd_sof_pcm *spcm;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (spcm == NULL) {
		dev_warn(sdev->dev, "warn: cant find PCM with DAI ID %d\n",
			rtd->dai_link->id);
		return;
	}

	if (spcm->pcm.playback)
		snd_dma_free_pages(&spcm->stream[SNDRV_PCM_STREAM_PLAYBACK].page_table);

	if (spcm->pcm.capture)
		snd_dma_free_pages(&spcm->stream[SNDRV_PCM_STREAM_CAPTURE].page_table);

}

static int sof_pcm_dai_link_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	/*  tmp hard code to 48k, stereo, 24bits. TODO: read this from topology */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static int sof_pcm_probe(struct snd_soc_platform *platform)
{
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(platform);
	struct snd_sof_pdata *plat_data = dev_get_platdata(platform->dev);
	int ret;

	/* load the default topology */
	sdev->component = &platform->component;
	ret = snd_sof_load_topology(sdev, plat_data->machine->sof_tplg_filename);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to load DSP topology %d\n",
			ret);
		goto err;
	}

	/* enable runtime PM with auto suspend */
	pm_runtime_set_autosuspend_delay(platform->dev,
		SND_SOF_SUSPEND_DELAY);
	pm_runtime_use_autosuspend(platform->dev);
	pm_runtime_enable(platform->dev);
	pm_runtime_idle(platform->dev);

err:
	return ret;
}

static int sof_pcm_remove(struct snd_soc_platform *platform)
{
	pm_runtime_disable(platform->dev);

	return 0;
}

void snd_sof_new_platform_drv(struct snd_sof_dev *sdev)
{
	struct snd_soc_platform_driver *pd = &sdev->plat_drv;
	struct snd_sof_pdata *plat_data = sdev->pdata;

	dev_dbg(sdev->dev, "using platform alias %s\n", 
		plat_data->machine->asoc_plat_name);	

	pd->probe = sof_pcm_probe;
	pd->remove = sof_pcm_remove;
	pd->ops	= &sof_pcm_ops;
	pd->compr_ops = &sof_compressed_ops;
	pd->pcm_new = sof_pcm_new;
	pd->pcm_free = sof_pcm_free;
	pd->ignore_machine = plat_data->machine->drv_name;
	pd->be_hw_params_fixup = sof_pcm_dai_link_fixup;
	pd->be_pcm_base = SOF_BE_PCM_BASE;
	pd->use_dai_pcm_id = true;
}

static const struct snd_soc_dai_ops sof_dai_ops = {
#if 0
	.startup	= sof_dai_startup,
	.shutdown	= sof_dai_shutdown,
	.trigger	= sof_dai_trigger,
	.hw_params	= sof_dai_hw_params,
	.set_sysclk	= sof_dai_set_dai_sysclk,
	.set_clkdiv	= sof_dai_set_dai_clkdiv,
	.set_pll	= sof_dai_set_dai_pll,
	.set_fmt	= sof_dai_set_dai_fmt,
	.set_tdm_slot	= sof_dai_set_dai_tdm_slot,
	.set_tristate	= sof_dai_set_dai_tristate,
#endif
};


static const struct snd_soc_component_driver sof_dai_component = {
	.name		= "sof-dai",
};

#define SOF_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
	SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_FLOAT)

void snd_sof_new_dai_drv(struct snd_sof_dev *sdev)
{
	struct snd_soc_dai_driver *dd = &sdev->dai_drv;
	//struct snd_sof_pdata *plat_data = sdev->pdata;

	sdev->cmpnt_drv = &sof_dai_component;
	dd->playback.channels_min = 1;
	dd->playback.channels_max = 16;
	dd->playback.rates = SNDRV_PCM_RATE_8000_192000;
	dd->playback.formats = SOF_FORMATS;
	dd->capture.channels_min = 1;
	dd->capture.channels_max = 16;
	dd->capture.rates = SNDRV_PCM_RATE_8000_192000;
	dd->capture.formats = SOF_FORMATS;
	dd->ops = &sof_dai_ops;
	sdev->num_dai = 1;
}


