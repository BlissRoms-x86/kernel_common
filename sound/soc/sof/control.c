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

/* Mixer Controls */


#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/soc-topology.h>
#include <sound/soc.h>
#include <sound/control.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"

int snd_sof_volume_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	unsigned int i, channels = scontrol->num_channels;

	pm_runtime_get_sync(sdev->dev);

	/* get all the mixer data from DSP */
	snd_sof_ipc_get_mixer(sdev->ipc, scontrol);

	/* read back each channel */
	for (i = 0; i < channels; i++)
		ucontrol->value.integer.value[i] =
			snd_sof_ipc_get_mixer_chan(sdev->ipc, scontrol, i);

	pm_runtime_mark_last_busy(sdev->dev);
	pm_runtime_put_autosuspend(sdev->dev);
	return 0;
}

int snd_sof_volume_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	unsigned int i, channels = scontrol->num_channels;

	pm_runtime_get_sync(sdev->dev);

	/* update each channel */
	for (i = 0; i < channels; i++)
		snd_sof_ipc_put_mixer_chan(sdev->ipc, scontrol, i,
			ucontrol->value.integer.value[i]);

	/* notify DSP of mixer updates */
	snd_sof_ipc_put_mixer(sdev->ipc, scontrol);

	pm_runtime_mark_last_busy(sdev->dev);
	pm_runtime_put_autosuspend(sdev->dev);
	return 0;
}

