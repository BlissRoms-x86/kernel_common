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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/sof.h>
#include <sound/compress_driver.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"

#if 0
/* compress stream operations */
static void sof_period_elapsed(void *arg)
{
	struct snd_compr_stream *cstream = (struct snd_compr_stream *)arg;

	snd_compr_fragment_elapsed(cstream);
}

static void sof_drain_notify(void *arg)
{
	struct snd_compr_stream *cstream = (struct snd_compr_stream *)arg;

	snd_compr_drain_notify(cstream);
}
#endif

static int sof_compressed_open(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;

	mutex_lock(&spcm->mutex);
	pm_runtime_get_sync(sdev->dev);
	mutex_unlock(&spcm->mutex);
	return 0;
}

static int sof_compressed_free(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;

	mutex_lock(&spcm->mutex);
	pm_runtime_put(sdev->dev);
	mutex_unlock(&spcm->mutex);
	return 0;
}


static int sof_vorbis_set_params(struct snd_compr_stream *cstream,
					struct snd_compr_params *params)
{
#if 0
//	struct snd_compr_runtime *runtime = cstream->runtime;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
//	struct snd_soc_tplg_stream_caps *caps = 
//		&spcm->pcm.caps[cstream->direction];
#endif
	return 0;
}

static int sof_mp3_set_params(struct snd_compr_stream *cstream,
					struct snd_compr_params *params)
{
#if 0
//	struct snd_compr_runtime *runtime = cstream->runtime;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
//	struct snd_soc_tplg_stream_caps *caps = 
//		&spcm->pcm.caps[cstream->direction];
#endif
	return 0;
}

static int sof_compressed_set_params(struct snd_compr_stream *cstream,
					struct snd_compr_params *params)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);

	switch (params->codec.id) {
	case SND_AUDIOCODEC_VORBIS:
		return sof_vorbis_set_params(cstream, params);
	case SND_AUDIOCODEC_MP3:
		return sof_mp3_set_params(cstream, params);
	default:
		dev_err(sdev->dev, "error: codec id %d not supported\n",
			params->codec.id);
		return -EINVAL;
	}
}

static int sof_compressed_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	struct sof_ipc_stream stream;

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG;
	stream.comp_id = spcm->comp_id;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		//stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_RELEASE;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_STOP;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		//stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;		
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_PAUSE;
		break;
	default:
		break;
	}

	/* send IPC to the DSP */
 	return sof_ipc_tx_message_nowait(sdev->ipc, stream.hdr.cmd, &stream,
		sizeof(stream));
}

static int sof_compressed_pointer(struct snd_compr_stream *cstream,
					struct snd_compr_tstamp *tstamp)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_dev *sdev =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct snd_sof_pcm *spcm = rtd->sof;
	snd_pcm_uframes_t host, dai;

	snd_sof_ipc_stream_posn(sdev, spcm, cstream->direction, &host, &dai);

	dev_vdbg(sdev->dev, "CPCM: DMA position %lu DAI position %lu\n",
		host, dai);

	return 0;
}

static int sof_compressed_ack(struct snd_compr_stream *cstream,
					size_t bytes)
{
	return 0;
}

static int sof_compressed_get_caps(struct snd_compr_stream *cstream,
					struct snd_compr_caps *caps)
{
	return 0;
}

static int sof_compressed_get_codec_caps(struct snd_compr_stream *cstream,
					struct snd_compr_codec_caps *codec)
{
	return 0;
}

static int sof_compressed_set_metadata(struct snd_compr_stream *cstream,
					struct snd_compr_metadata *metadata)
{
	return 0;
}

struct snd_compr_ops sof_compressed_ops = {

	.open = sof_compressed_open,
	.free = sof_compressed_free,
	.set_params = sof_compressed_set_params,
	.set_metadata = sof_compressed_set_metadata,
	.trigger = sof_compressed_trigger,
	.pointer = sof_compressed_pointer,
	.ack = sof_compressed_ack,
	.get_caps = sof_compressed_get_caps,
	.get_codec_caps = sof_compressed_get_codec_caps,
};
