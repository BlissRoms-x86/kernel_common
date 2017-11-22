/*
 *   intel_hdmi_audio.c - Intel HDMI audio driver
 *
 *  Copyright (C) 2016 Intel Corp
 *  Authors:	Sailaja Bandarupalli <sailaja.bandarupalli@intel.com>
 *		Ramesh Babu K V	<ramesh.babu@intel.com>
 *		Vaibhav Agarwal <vaibhav.agarwal@intel.com>
 *		Jerome Anand <jerome.anand@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ALSA driver for Intel HDMI audio
 */

#define pr_fmt(fmt)	"had: " fmt

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <asm/cacheflush.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/initval.h>
#include "intel_hdmi_audio.h"

/*standard module options for ALSA. This module supports only one card*/
static int hdmi_card_index = SNDRV_DEFAULT_IDX1;
static char *hdmi_card_id = SNDRV_DEFAULT_STR1;

module_param_named(index, hdmi_card_index, int, 0444);
MODULE_PARM_DESC(index,
		"Index value for INTEL Intel HDMI Audio controller.");
module_param_named(id, hdmi_card_id, charp, 0444);
MODULE_PARM_DESC(id,
		"ID string for INTEL Intel HDMI Audio controller.");

/*
 * ELD SA bits in the CEA Speaker Allocation data block
 */
static int eld_speaker_allocation_bits[] = {
	[0] = FL | FR,
	[1] = LFE,
	[2] = FC,
	[3] = RL | RR,
	[4] = RC,
	[5] = FLC | FRC,
	[6] = RLC | RRC,
	/* the following are not defined in ELD yet */
	[7] = 0,
};

/*
 * This is an ordered list!
 *
 * The preceding ones have better chances to be selected by
 * hdmi_channel_allocation().
 */
static struct cea_channel_speaker_allocation channel_allocations[] = {
/*                        channel:   7     6    5    4    3     2    1    0  */
{ .ca_index = 0x00,  .speakers = {   0,    0,   0,   0,   0,    0,  FR,  FL } },
				/* 2.1 */
{ .ca_index = 0x01,  .speakers = {   0,    0,   0,   0,   0,  LFE,  FR,  FL } },
				/* Dolby Surround */
{ .ca_index = 0x02,  .speakers = {   0,    0,   0,   0,  FC,    0,  FR,  FL } },
				/* surround40 */
{ .ca_index = 0x08,  .speakers = {   0,    0,  RR,  RL,   0,    0,  FR,  FL } },
				/* surround41 */
{ .ca_index = 0x09,  .speakers = {   0,    0,  RR,  RL,   0,  LFE,  FR,  FL } },
				/* surround50 */
{ .ca_index = 0x0a,  .speakers = {   0,    0,  RR,  RL,  FC,    0,  FR,  FL } },
				/* surround51 */
{ .ca_index = 0x0b,  .speakers = {   0,    0,  RR,  RL,  FC,  LFE,  FR,  FL } },
				/* 6.1 */
{ .ca_index = 0x0f,  .speakers = {   0,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
				/* surround71 */
{ .ca_index = 0x13,  .speakers = { RRC,  RLC,  RR,  RL,  FC,  LFE,  FR,  FL } },

{ .ca_index = 0x03,  .speakers = {   0,    0,   0,   0,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x04,  .speakers = {   0,    0,   0,  RC,   0,    0,  FR,  FL } },
{ .ca_index = 0x05,  .speakers = {   0,    0,   0,  RC,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x06,  .speakers = {   0,    0,   0,  RC,  FC,    0,  FR,  FL } },
{ .ca_index = 0x07,  .speakers = {   0,    0,   0,  RC,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x0c,  .speakers = {   0,   RC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x0d,  .speakers = {   0,   RC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x0e,  .speakers = {   0,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x10,  .speakers = { RRC,  RLC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x11,  .speakers = { RRC,  RLC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x12,  .speakers = { RRC,  RLC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x14,  .speakers = { FRC,  FLC,   0,   0,   0,    0,  FR,  FL } },
{ .ca_index = 0x15,  .speakers = { FRC,  FLC,   0,   0,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x16,  .speakers = { FRC,  FLC,   0,   0,  FC,    0,  FR,  FL } },
{ .ca_index = 0x17,  .speakers = { FRC,  FLC,   0,   0,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x18,  .speakers = { FRC,  FLC,   0,  RC,   0,    0,  FR,  FL } },
{ .ca_index = 0x19,  .speakers = { FRC,  FLC,   0,  RC,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x1a,  .speakers = { FRC,  FLC,   0,  RC,  FC,    0,  FR,  FL } },
{ .ca_index = 0x1b,  .speakers = { FRC,  FLC,   0,  RC,  FC,  LFE,  FR,  FL } },
{ .ca_index = 0x1c,  .speakers = { FRC,  FLC,  RR,  RL,   0,    0,  FR,  FL } },
{ .ca_index = 0x1d,  .speakers = { FRC,  FLC,  RR,  RL,   0,  LFE,  FR,  FL } },
{ .ca_index = 0x1e,  .speakers = { FRC,  FLC,  RR,  RL,  FC,    0,  FR,  FL } },
{ .ca_index = 0x1f,  .speakers = { FRC,  FLC,  RR,  RL,  FC,  LFE,  FR,  FL } },
};

static struct channel_map_table map_tables[] = {
	{ SNDRV_CHMAP_FL,       0x00,   FL },
	{ SNDRV_CHMAP_FR,       0x01,   FR },
	{ SNDRV_CHMAP_RL,       0x04,   RL },
	{ SNDRV_CHMAP_RR,       0x05,   RR },
	{ SNDRV_CHMAP_LFE,      0x02,   LFE },
	{ SNDRV_CHMAP_FC,       0x03,   FC },
	{ SNDRV_CHMAP_RLC,      0x06,   RLC },
	{ SNDRV_CHMAP_RRC,      0x07,   RRC },
	{} /* terminator */
};

/* hardware capability structure */
static const struct snd_pcm_hardware snd_intel_hadstream = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_DOUBLE |
		SNDRV_PCM_INFO_MMAP|
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH),
	.formats = (SNDRV_PCM_FMTBIT_S24 |
		SNDRV_PCM_FMTBIT_U24),
	.rates = SNDRV_PCM_RATE_32000 |
		SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000 |
		SNDRV_PCM_RATE_88200 |
		SNDRV_PCM_RATE_96000 |
		SNDRV_PCM_RATE_176400 |
		SNDRV_PCM_RATE_192000,
	.rate_min = HAD_MIN_RATE,
	.rate_max = HAD_MAX_RATE,
	.channels_min = HAD_MIN_CHANNEL,
	.channels_max = HAD_MAX_CHANNEL,
	.buffer_bytes_max = HAD_MAX_BUFFER,
	.period_bytes_min = HAD_MIN_PERIOD_BYTES,
	.period_bytes_max = HAD_MAX_PERIOD_BYTES,
	.periods_min = HAD_MIN_PERIODS,
	.periods_max = HAD_MAX_PERIODS,
	.fifo_size = HAD_FIFO_SIZE,
};

/* Register access functions */

int had_get_hwstate(struct snd_intelhad *intelhaddata)
{
	/* Check for device presence -SW state */
	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED) {
		pr_debug("%s:Device not connected:%d\n", __func__,
				intelhaddata->drv_status);
		return -ENODEV;
	}

	return 0;
}

int had_get_caps(struct snd_intelhad *intelhaddata,
		 enum had_caps_list query, void *caps)
{
	struct platform_device *pdev = to_platform_device(intelhaddata->dev);
	int retval;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = mid_hdmi_audio_get_caps(pdev, query, caps);

	return retval;
}

int had_set_caps(struct snd_intelhad *intelhaddata,
		 enum had_caps_list set_element, void *caps)
{
	struct platform_device *pdev = to_platform_device(intelhaddata->dev);
	int retval;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = mid_hdmi_audio_set_caps(pdev, set_element, caps);

	return retval;
}

int had_read_register(struct snd_intelhad *intelhaddata, u32 offset, u32 *data)
{
	struct platform_device *pdev = to_platform_device(intelhaddata->dev);
	int retval;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = mid_hdmi_audio_read(pdev, offset, data);

	return retval;
}

int had_write_register(struct snd_intelhad *intelhaddata, u32 offset, u32 data)
{
	struct platform_device *pdev = to_platform_device(intelhaddata->dev);
	int retval;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = mid_hdmi_audio_write(pdev, offset, data);

	return retval;
}

int had_read_modify(struct snd_intelhad *intelhaddata, u32 offset,
		    u32 data, u32 mask)
{
	struct platform_device *pdev = to_platform_device(intelhaddata->dev);
	int retval;

	retval = had_get_hwstate(intelhaddata);
	if (!retval)
		retval = mid_hdmi_audio_rmw(pdev, offset, data, mask);

	return retval;
}
/**
 * function to read-modify
 * AUD_CONFIG register on VLV2.The had_read_modify() function should not
 * directly be used on VLV2 for updating AUD_CONFIG register.
 * This is because:
 * Bit6 of AUD_CONFIG register is writeonly due to a silicon bug on VLV2
 * HDMI IP. As a result a read-modify of AUD_CONFIG regiter will always
 * clear bit6. AUD_CONFIG[6:4] represents the "channels" field of the
 * register. This field should be 1xy binary for configuration with 6 or
 * more channels. Read-modify of AUD_CONFIG (Eg. for enabling audio)
 * causes the "channels" field to be updated as 0xy binary resulting in
 * bad audio. The fix is to always write the AUD_CONFIG[6:4] with
 * appropriate value when doing read-modify of AUD_CONFIG register.
 *
 * @substream: the current substream or NULL if no active substream
 * @data : data to be written
 * @mask : mask
 *
 */
static int had_read_modify_aud_config_v2(struct snd_pcm_substream *substream,
					u32 data, u32 mask)
{
	struct snd_intelhad *intelhaddata = snd_pcm_substream_chip(substream);
	union aud_cfg cfg_val = {.cfg_regval = 0};
	u8 channels;

	/*
	 * If substream is NULL, there is no active stream.
	 * In this case just set channels to 2
	 */
	if (substream)
		channels = substream->runtime->channels;
	else
		channels = 2;
	cfg_val.cfg_regx_v2.num_ch = channels - 2;

	data = data | cfg_val.cfg_regval;
	mask = mask | AUD_CONFIG_CH_MASK_V2;

	pr_debug("%s : data = %x, mask =%x\n", __func__, data, mask);

	return had_read_modify(intelhaddata, AUD_CONFIG, data, mask);
}

void snd_intelhad_enable_audio(struct snd_pcm_substream *substream, u8 enable)
{
	had_read_modify_aud_config_v2(substream, enable, BIT(0));
}

static void snd_intelhad_reset_audio(struct snd_intelhad *intelhaddata,
				     u8 reset)
{
	had_write_register(intelhaddata, AUD_HDMI_STATUS_v2, reset);
}

/**
 * initialize audio channel status registers
 * This function is called in the prepare callback
 */
static int had_prog_status_reg(struct snd_pcm_substream *substream,
			struct snd_intelhad *intelhaddata)
{
	union aud_cfg cfg_val = {.cfg_regval = 0};
	union aud_ch_status_0 ch_stat0 = {.status_0_regval = 0};
	union aud_ch_status_1 ch_stat1 = {.status_1_regval = 0};
	int format;

	pr_debug("Entry %s\n", __func__);

	ch_stat0.status_0_regx.lpcm_id = (intelhaddata->aes_bits &
						IEC958_AES0_NONAUDIO)>>1;
	ch_stat0.status_0_regx.clk_acc = (intelhaddata->aes_bits &
						IEC958_AES3_CON_CLOCK)>>4;
	cfg_val.cfg_regx_v2.val_bit = ch_stat0.status_0_regx.lpcm_id;

	switch (substream->runtime->rate) {
	case AUD_SAMPLE_RATE_32:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_32KHZ;
		break;

	case AUD_SAMPLE_RATE_44_1:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_44KHZ;
		break;
	case AUD_SAMPLE_RATE_48:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_48KHZ;
		break;
	case AUD_SAMPLE_RATE_88_2:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_88KHZ;
		break;
	case AUD_SAMPLE_RATE_96:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_96KHZ;
		break;
	case AUD_SAMPLE_RATE_176_4:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_176KHZ;
		break;
	case AUD_SAMPLE_RATE_192:
		ch_stat0.status_0_regx.samp_freq = CH_STATUS_MAP_192KHZ;
		break;

	default:
		/* control should never come here */
		return -EINVAL;
	break;

	}
	had_write_register(intelhaddata,
			   AUD_CH_STATUS_0, ch_stat0.status_0_regval);

	format = substream->runtime->format;

	if (format == SNDRV_PCM_FORMAT_S16_LE) {
		ch_stat1.status_1_regx.max_wrd_len = MAX_SMPL_WIDTH_20;
		ch_stat1.status_1_regx.wrd_len = SMPL_WIDTH_16BITS;
	} else if (format == SNDRV_PCM_FORMAT_S24_LE) {
		ch_stat1.status_1_regx.max_wrd_len = MAX_SMPL_WIDTH_24;
		ch_stat1.status_1_regx.wrd_len = SMPL_WIDTH_24BITS;
	} else {
		ch_stat1.status_1_regx.max_wrd_len = 0;
		ch_stat1.status_1_regx.wrd_len = 0;
	}
	had_write_register(intelhaddata,
			   AUD_CH_STATUS_1, ch_stat1.status_1_regval);
	return 0;
}

/*
 * function to initialize audio
 * registers and buffer confgiuration registers
 * This function is called in the prepare callback
 */
static int snd_intelhad_audio_ctrl(struct snd_pcm_substream *substream,
				   struct snd_intelhad *intelhaddata)
{
	union aud_cfg cfg_val = {.cfg_regval = 0};
	union aud_buf_config buf_cfg = {.buf_cfgval = 0};
	u8 channels;

	had_prog_status_reg(substream, intelhaddata);

	buf_cfg.buf_cfg_regx_v2.audio_fifo_watermark = FIFO_THRESHOLD;
	buf_cfg.buf_cfg_regx_v2.dma_fifo_watermark = DMA_FIFO_THRESHOLD;
	buf_cfg.buf_cfg_regx_v2.aud_delay = 0;
	had_write_register(intelhaddata, AUD_BUF_CONFIG, buf_cfg.buf_cfgval);

	channels = substream->runtime->channels;
	cfg_val.cfg_regx_v2.num_ch = channels - 2;
	if (channels <= 2)
		cfg_val.cfg_regx_v2.layout = LAYOUT0;
	else
		cfg_val.cfg_regx_v2.layout = LAYOUT1;

	cfg_val.cfg_regx_v2.val_bit = 1;
	had_write_register(intelhaddata, AUD_CONFIG, cfg_val.cfg_regval);
	return 0;
}

/*
 * Compute derived values in channel_allocations[].
 */
static void init_channel_allocations(void)
{
	int i, j;
	struct cea_channel_speaker_allocation *p;

	pr_debug("%s: Enter\n", __func__);

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		p = channel_allocations + i;
		p->channels = 0;
		p->spk_mask = 0;
		for (j = 0; j < ARRAY_SIZE(p->speakers); j++)
			if (p->speakers[j]) {
				p->channels++;
				p->spk_mask |= p->speakers[j];
			}
	}
}

/*
 * The transformation takes two steps:
 *
 *      eld->spk_alloc => (eld_speaker_allocation_bits[]) => spk_mask
 *            spk_mask => (channel_allocations[])         => ai->CA
 *
 * TODO: it could select the wrong CA from multiple candidates.
 */
static int snd_intelhad_channel_allocation(struct snd_intelhad *intelhaddata,
					int channels)
{
	int i;
	int ca = 0;
	int spk_mask = 0;

	/*
	 * CA defaults to 0 for basic stereo audio
	 */
	if (channels <= 2)
		return 0;

	/*
	 * expand ELD's speaker allocation mask
	 *
	 * ELD tells the speaker mask in a compact(paired) form,
	 * expand ELD's notions to match the ones used by Audio InfoFrame.
	 */

	for (i = 0; i < ARRAY_SIZE(eld_speaker_allocation_bits); i++) {
		if (intelhaddata->eeld.speaker_allocation_block & (1 << i))
			spk_mask |= eld_speaker_allocation_bits[i];
	}

	/* search for the first working match in the CA table */
	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (channels == channel_allocations[i].channels &&
		(spk_mask & channel_allocations[i].spk_mask) ==
				channel_allocations[i].spk_mask) {
			ca = channel_allocations[i].ca_index;
			break;
		}
	}

	pr_debug("HDMI: select CA 0x%x for %d\n", ca, channels);

	return ca;
}

/* from speaker bit mask to ALSA API channel position */
static int spk_to_chmap(int spk)
{
	struct channel_map_table *t = map_tables;

	for (; t->map; t++) {
		if (t->spk_mask == spk)
			return t->map;
	}
	return 0;
}

void had_build_channel_allocation_map(struct snd_intelhad *intelhaddata)
{
	int i = 0, c = 0;
	int spk_mask = 0;
	struct snd_pcm_chmap_elem *chmap;
	u8 eld_high, eld_high_mask = 0xF0;
	u8 high_msb;

	chmap = kzalloc(sizeof(*chmap), GFP_KERNEL);
	if (chmap == NULL) {
		intelhaddata->chmap->chmap = NULL;
		return;
	}

	had_get_caps(intelhaddata, HAD_GET_ELD, &intelhaddata->eeld);
	had_get_caps(intelhaddata, HAD_GET_DP_OUTPUT, &intelhaddata->dp_output);

	pr_debug("eeld.speaker_allocation_block = %x\n",
			intelhaddata->eeld.speaker_allocation_block);

	/* WA: Fix the max channel supported to 8 */

	/*
	 * Sink may support more than 8 channels, if eld_high has more than
	 * one bit set. SOC supports max 8 channels.
	 * Refer eld_speaker_allocation_bits, for sink speaker allocation
	 */

	/* if 0x2F < eld < 0x4F fall back to 0x2f, else fall back to 0x4F */
	eld_high = intelhaddata->eeld.speaker_allocation_block & eld_high_mask;
	if ((eld_high & (eld_high-1)) && (eld_high > 0x1F)) {
		/* eld_high & (eld_high-1): if more than 1 bit set */
		/* 0x1F: 7 channels */
		for (i = 1; i < 4; i++) {
			high_msb = eld_high & (0x80 >> i);
			if (high_msb) {
				intelhaddata->eeld.speaker_allocation_block &=
					high_msb | 0xF;
				break;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(eld_speaker_allocation_bits); i++) {
		if (intelhaddata->eeld.speaker_allocation_block & (1 << i))
			spk_mask |= eld_speaker_allocation_bits[i];
	}

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (spk_mask == channel_allocations[i].spk_mask) {
			for (c = 0; c < channel_allocations[i].channels; c++) {
				chmap->map[c] = spk_to_chmap(
					channel_allocations[i].speakers[
						(MAX_SPEAKERS - 1)-c]);
			}
			chmap->channels = channel_allocations[i].channels;
			intelhaddata->chmap->chmap = chmap;
			break;
		}
	}
	if (i >= ARRAY_SIZE(channel_allocations)) {
		intelhaddata->chmap->chmap = NULL;
		kfree(chmap);
	}
}

/*
 * ALSA API channel-map control callbacks
 */
static int had_chmap_ctl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct snd_intelhad *intelhaddata = info->private_data;

	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED)
		return -ENODEV;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = HAD_MAX_CHANNEL;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SNDRV_CHMAP_LAST;
	return 0;
}

static int had_chmap_ctl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct snd_intelhad *intelhaddata = info->private_data;
	int i = 0;
	const struct snd_pcm_chmap_elem *chmap;

	if (intelhaddata->drv_status == HAD_DRV_DISCONNECTED)
		return -ENODEV;
	if (intelhaddata->chmap->chmap ==  NULL)
		return -ENODATA;
	chmap = intelhaddata->chmap->chmap;
	for (i = 0; i < chmap->channels; i++) {
		ucontrol->value.integer.value[i] = chmap->map[i];
		pr_debug("chmap->map[%d] = %d\n", i, chmap->map[i]);
	}

	return 0;
}

static int had_register_chmap_ctls(struct snd_intelhad *intelhaddata,
						struct snd_pcm *pcm)
{
	int err = 0;

	err = snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			NULL, 0, (unsigned long)intelhaddata,
			&intelhaddata->chmap);
	if (err < 0)
		return err;

	intelhaddata->chmap->private_data = intelhaddata;
	intelhaddata->kctl = intelhaddata->chmap->kctl;
	intelhaddata->kctl->info = had_chmap_ctl_info;
	intelhaddata->kctl->get = had_chmap_ctl_get;
	intelhaddata->chmap->chmap = NULL;
	return 0;
}

/*
 * snd_intelhad_prog_dip - to initialize Data Island Packets registers
 *
 * @substream:substream for which the prepare function is called
 * @intelhaddata:substream private data
 *
 * This function is called in the prepare callback
 */
static void snd_intelhad_prog_dip(struct snd_pcm_substream *substream,
				  struct snd_intelhad *intelhaddata)
{
	int i;
	union aud_ctrl_st ctrl_state = {.ctrl_val = 0};
	union aud_info_frame2 frame2 = {.fr2_val = 0};
	union aud_info_frame3 frame3 = {.fr3_val = 0};
	u8 checksum = 0;
	u32 info_frame;
	int channels;

	channels = substream->runtime->channels;

	had_write_register(intelhaddata, AUD_CNTL_ST, ctrl_state.ctrl_val);

	if (intelhaddata->dp_output) {
		info_frame = DP_INFO_FRAME_WORD1;
		frame2.fr2_val = 1;
	} else {
		info_frame = HDMI_INFO_FRAME_WORD1;
		frame2.fr2_regx.chnl_cnt = substream->runtime->channels - 1;

		frame3.fr3_regx.chnl_alloc = snd_intelhad_channel_allocation(
			intelhaddata, channels);

		/*Calculte the byte wide checksum for all valid DIP words*/
		for (i = 0; i < BYTES_PER_WORD; i++)
			checksum += (info_frame >> i*BITS_PER_BYTE) & MASK_BYTE0;
		for (i = 0; i < BYTES_PER_WORD; i++)
			checksum += (frame2.fr2_val >> i*BITS_PER_BYTE) & MASK_BYTE0;
		for (i = 0; i < BYTES_PER_WORD; i++)
			checksum += (frame3.fr3_val >> i*BITS_PER_BYTE) & MASK_BYTE0;

		frame2.fr2_regx.chksum = -(checksum);
	}

	had_write_register(intelhaddata, AUD_HDMIW_INFOFR_v2, info_frame);
	had_write_register(intelhaddata, AUD_HDMIW_INFOFR_v2, frame2.fr2_val);
	had_write_register(intelhaddata, AUD_HDMIW_INFOFR_v2, frame3.fr3_val);

	/* program remaining DIP words with zero */
	for (i = 0; i < HAD_MAX_DIP_WORDS-VALID_DIP_WORDS; i++)
		had_write_register(intelhaddata, AUD_HDMIW_INFOFR_v2, 0x0);

	ctrl_state.ctrl_regx.dip_freq = 1;
	ctrl_state.ctrl_regx.dip_en_sta = 1;
	had_write_register(intelhaddata, AUD_CNTL_ST, ctrl_state.ctrl_val);
}

/**
 * snd_intelhad_prog_buffer - programs buffer
 * address and length registers
 *
 * @substream:substream for which the prepare function is called
 * @intelhaddata:substream private data
 *
 * This function programs ring buffer address and length into registers.
 */
int snd_intelhad_prog_buffer(struct snd_intelhad *intelhaddata,
					int start, int end)
{
	u32 ring_buf_addr, ring_buf_size, period_bytes;
	u8 i, num_periods;
	struct snd_pcm_substream *substream;

	substream = intelhaddata->stream_info.had_substream;
	if (!substream) {
		pr_err("substream is NULL\n");
		dump_stack();
		return 0;
	}

	ring_buf_addr = substream->runtime->dma_addr;
	ring_buf_size = snd_pcm_lib_buffer_bytes(substream);
	intelhaddata->stream_info.ring_buf_size = ring_buf_size;
	period_bytes = frames_to_bytes(substream->runtime,
				substream->runtime->period_size);
	num_periods = substream->runtime->periods;

	/*
	 * buffer addr should  be 64 byte aligned, period bytes
	 * will be used to calculate addr offset
	 */
	period_bytes &= ~0x3F;

	/* Hardware supports MAX_PERIODS buffers */
	if (end >= HAD_MAX_PERIODS)
		return -EINVAL;

	for (i = start; i <= end; i++) {
		/* Program the buf registers with addr and len */
		intelhaddata->buf_info[i].buf_addr = ring_buf_addr +
							 (i * period_bytes);
		if (i < num_periods-1)
			intelhaddata->buf_info[i].buf_size = period_bytes;
		else
			intelhaddata->buf_info[i].buf_size = ring_buf_size -
							(period_bytes*i);

		had_write_register(intelhaddata,
				   AUD_BUF_A_ADDR + (i * HAD_REG_WIDTH),
					intelhaddata->buf_info[i].buf_addr |
					BIT(0) | BIT(1));
		had_write_register(intelhaddata,
				   AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
					period_bytes);
		intelhaddata->buf_info[i].is_valid = true;
	}
	pr_debug("%s:buf[%d-%d] addr=%#x  and size=%d\n", __func__, start, end,
			intelhaddata->buf_info[start].buf_addr,
			intelhaddata->buf_info[start].buf_size);
	intelhaddata->valid_buf_cnt = num_periods;
	return 0;
}

int snd_intelhad_read_len(struct snd_intelhad *intelhaddata)
{
	int i, retval = 0;
	u32 len[4];

	for (i = 0; i < 4 ; i++) {
		had_read_register(intelhaddata,
				  AUD_BUF_A_LENGTH + (i * HAD_REG_WIDTH),
				  &len[i]);
		if (!len[i])
			retval++;
	}
	if (retval != 1) {
		for (i = 0; i < 4 ; i++)
			pr_debug("buf[%d] size=%d\n", i, len[i]);
	}

	return retval;
}

static int had_calculate_maud_value(u32 aud_samp_freq, u32 link_rate)
{
	u32 maud_val;

	/* Select maud according to DP 1.2 spec*/
	if (link_rate == DP_2_7_GHZ) {
		switch (aud_samp_freq) {
		case AUD_SAMPLE_RATE_32:
			maud_val = AUD_SAMPLE_RATE_32_DP_2_7_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_44_1:
			maud_val = AUD_SAMPLE_RATE_44_1_DP_2_7_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_48:
			maud_val = AUD_SAMPLE_RATE_48_DP_2_7_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_88_2:
			maud_val = AUD_SAMPLE_RATE_88_2_DP_2_7_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_96:
			maud_val = AUD_SAMPLE_RATE_96_DP_2_7_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_176_4:
			maud_val = AUD_SAMPLE_RATE_176_4_DP_2_7_MAUD_VAL;
			break;

		case HAD_MAX_RATE:
			maud_val = HAD_MAX_RATE_DP_2_7_MAUD_VAL;
			break;

		default:
			maud_val = -EINVAL;
			break;
		}
	} else if (link_rate == DP_1_62_GHZ) {
		switch (aud_samp_freq) {
		case AUD_SAMPLE_RATE_32:
			maud_val = AUD_SAMPLE_RATE_32_DP_1_62_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_44_1:
			maud_val = AUD_SAMPLE_RATE_44_1_DP_1_62_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_48:
			maud_val = AUD_SAMPLE_RATE_48_DP_1_62_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_88_2:
			maud_val = AUD_SAMPLE_RATE_88_2_DP_1_62_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_96:
			maud_val = AUD_SAMPLE_RATE_96_DP_1_62_MAUD_VAL;
			break;

		case AUD_SAMPLE_RATE_176_4:
			maud_val = AUD_SAMPLE_RATE_176_4_DP_1_62_MAUD_VAL;
			break;

		case HAD_MAX_RATE:
			maud_val = HAD_MAX_RATE_DP_1_62_MAUD_VAL;
			break;

		default:
			maud_val = -EINVAL;
			break;
		}
	} else
		maud_val = -EINVAL;

	return maud_val;
}

/*
 * snd_intelhad_prog_cts - Program HDMI audio CTS value
 *
 * @aud_samp_freq: sampling frequency of audio data
 * @tmds: sampling frequency of the display data
 * @n_param: N value, depends on aud_samp_freq
 * @intelhaddata:substream private data
 *
 * Program CTS register based on the audio and display sampling frequency
 */
static void snd_intelhad_prog_cts(u32 aud_samp_freq, u32 tmds,
				  u32 link_rate, u32 n_param,
				  struct snd_intelhad *intelhaddata)
{
	u32 cts_val;
	u64 dividend, divisor;

	if (intelhaddata->dp_output) {
		/* Substitute cts_val with Maud according to DP 1.2 spec*/
		cts_val = had_calculate_maud_value(aud_samp_freq, link_rate);
	} else {
		/* Calculate CTS according to HDMI 1.3a spec*/
		dividend = (u64)tmds * n_param*1000;
		divisor = 128 * aud_samp_freq;
		cts_val = div64_u64(dividend, divisor);
	}
	pr_debug("TMDS value=%d, N value=%d, CTS Value=%d\n",
		 tmds, n_param, cts_val);
	had_write_register(intelhaddata, AUD_HDMI_CTS, (BIT(24) | cts_val));
}

static int had_calculate_n_value(u32 aud_samp_freq)
{
	s32 n_val;

	/* Select N according to HDMI 1.3a spec*/
	switch (aud_samp_freq) {
	case AUD_SAMPLE_RATE_32:
		n_val = 4096;
	break;

	case AUD_SAMPLE_RATE_44_1:
		n_val = 6272;
	break;

	case AUD_SAMPLE_RATE_48:
		n_val = 6144;
	break;

	case AUD_SAMPLE_RATE_88_2:
		n_val = 12544;
	break;

	case AUD_SAMPLE_RATE_96:
		n_val = 12288;
	break;

	case AUD_SAMPLE_RATE_176_4:
		n_val = 25088;
	break;

	case HAD_MAX_RATE:
		n_val = 24576;
	break;

	default:
		n_val = -EINVAL;
	break;
	}
	return n_val;
}

/*
 * snd_intelhad_prog_n - Program HDMI audio N value
 *
 * @aud_samp_freq: sampling frequency of audio data
 * @n_param: N value, depends on aud_samp_freq
 * @intelhaddata:substream private data
 *
 * This function is called in the prepare callback.
 * It programs based on the audio and display sampling frequency
 */
static int snd_intelhad_prog_n(u32 aud_samp_freq, u32 *n_param,
			       struct snd_intelhad *intelhaddata)
{
	s32 n_val;

	if (intelhaddata->dp_output) {
		/*
		 * According to DP specs, Maud and Naud values hold
		 * a relationship, which is stated as:
		 * Maud/Naud = 512 * fs / f_LS_Clk
		 * where, fs is the sampling frequency of the audio stream
		 * and Naud is 32768 for Async clock.
		 */

		n_val = DP_NAUD_VAL;
	} else
		n_val =	had_calculate_n_value(aud_samp_freq);

	if (n_val < 0)
		return n_val;

	had_write_register(intelhaddata, AUD_N_ENABLE, (BIT(24) | n_val));
	*n_param = n_val;
	return 0;
}

void snd_intelhad_handle_underrun(struct snd_intelhad *intelhaddata)
{
	u32 hdmi_status, i = 0;

	/* Handle Underrun interrupt within Audio Unit */
	had_write_register(intelhaddata, AUD_CONFIG, 0);
	/* Reset buffer pointers */
	had_write_register(intelhaddata, AUD_HDMI_STATUS_v2, 1);
	had_write_register(intelhaddata, AUD_HDMI_STATUS_v2, 0);
	/**
	 * The interrupt status 'sticky' bits might not be cleared by
	 * setting '1' to that bit once...
	 */
	do { /* clear bit30, 31 AUD_HDMI_STATUS */
		had_read_register(intelhaddata, AUD_HDMI_STATUS_v2,
				  &hdmi_status);
		pr_debug("HDMI status =0x%x\n", hdmi_status);
		if (hdmi_status & AUD_CONFIG_MASK_UNDERRUN) {
			i++;
			had_write_register(intelhaddata,
					   AUD_HDMI_STATUS_v2, hdmi_status);
		} else
			break;
	} while (i < MAX_CNT);
	if (i >= MAX_CNT)
		pr_err("Unable to clear UNDERRUN bits\n");
}

/**
 * snd_intelhad_open - stream initializations are done here
 * @substream:substream for which the stream function is called
 *
 * This function is called whenever a PCM stream is opened
 */
static int snd_intelhad_open(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;
	struct snd_pcm_runtime *runtime;
	struct had_stream_pvt *stream;
	struct had_pvt_data *had_stream;
	int retval;

	pr_debug("snd_intelhad_open called\n");
	intelhaddata = snd_pcm_substream_chip(substream);
	had_stream = intelhaddata->private_data;
	runtime = substream->runtime;
	intelhaddata->underrun_count = 0;

	pm_runtime_get(intelhaddata->dev);

	if (had_get_hwstate(intelhaddata)) {
		pr_err("%s: HDMI cable plugged-out\n", __func__);
		retval = -ENODEV;
		goto exit_put_handle;
	}

	/* Check, if device already in use */
	if (runtime->private_data) {
		pr_err("Device already in use\n");
		retval = -EBUSY;
		goto exit_put_handle;
	}

	/* set the runtime hw parameter with local snd_pcm_hardware struct */
	runtime->hw = snd_intel_hadstream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream) {
		retval = -ENOMEM;
		goto exit_put_handle;
	}
	stream->stream_status = STREAM_INIT;
	runtime->private_data = stream;

	retval = snd_pcm_hw_constraint_integer(runtime,
			 SNDRV_PCM_HW_PARAM_PERIODS);
	if (retval < 0)
		goto exit_err;

	/* Make sure, that the period size is always aligned
	 * 64byte boundary
	 */
	retval = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 64);
	if (retval < 0) {
		pr_err("%s:step_size=64 failed,err=%d\n", __func__, retval);
		goto exit_err;
	}

	return retval;
exit_err:
	kfree(stream);
exit_put_handle:
	pm_runtime_put(intelhaddata->dev);
	runtime->private_data = NULL;
	return retval;
}

/**
 * had_period_elapsed - updates the hardware pointer status
 * @had_substream:substream for which the stream function is called
 *
 */
static void had_period_elapsed(void *had_substream)
{
	struct snd_pcm_substream *substream = had_substream;
	struct had_stream_pvt *stream;

	/* pr_debug("had_period_elapsed called\n"); */

	if (!substream || !substream->runtime)
		return;
	stream = substream->runtime->private_data;
	if (!stream)
		return;

	if (stream->stream_status != STREAM_RUNNING)
		return;
	snd_pcm_period_elapsed(substream);
}

/**
 * snd_intelhad_init_stream - internal function to initialize stream info
 * @substream:substream for which the stream function is called
 *
 */
static int snd_intelhad_init_stream(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata = snd_pcm_substream_chip(substream);

	pr_debug("snd_intelhad_init_stream called\n");

	pr_debug("setting buffer ptr param\n");
	intelhaddata->stream_info.period_elapsed = had_period_elapsed;
	intelhaddata->stream_info.had_substream = substream;
	intelhaddata->stream_info.buffer_ptr = 0;
	intelhaddata->stream_info.buffer_rendered = 0;
	intelhaddata->stream_info.sfreq = substream->runtime->rate;
	return 0;
}

/**
 * snd_intelhad_close- to free parameteres when stream is stopped
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework when stream is stopped
 */
static int snd_intelhad_close(struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;
	struct snd_pcm_runtime *runtime;

	pr_debug("snd_intelhad_close called\n");

	intelhaddata = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;

	if (!runtime->private_data) {
		pr_debug("close() might have called after failed open");
		return 0;
	}

	intelhaddata->stream_info.buffer_rendered = 0;
	intelhaddata->stream_info.buffer_ptr = 0;
	intelhaddata->stream_info.str_id = 0;
	intelhaddata->stream_info.had_substream = NULL;

	/* Check if following drv_status modification is required - VA */
	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED) {
		intelhaddata->drv_status = HAD_DRV_CONNECTED;
		pr_debug("%s @ %d:DEBUG PLUG/UNPLUG : HAD_DRV_CONNECTED\n",
			__func__, __LINE__);
	}
	kfree(runtime->private_data);
	runtime->private_data = NULL;
	pm_runtime_put(intelhaddata->dev);
	return 0;
}

/**
 * snd_intelhad_hw_params- to setup the hardware parameters
 * like allocating the buffers
 *
 * @substream:  substream for which the function is called
 * @hw_params: hardware parameters
 *
 * This function is called by ALSA framework when hardware params are set
 */
static int snd_intelhad_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *hw_params)
{
	unsigned long addr;
	int pages, buf_size, retval;

	pr_debug("snd_intelhad_hw_params called\n");

	if (!hw_params)
		return -EINVAL;

	buf_size = params_buffer_bytes(hw_params);
	retval = snd_pcm_lib_malloc_pages(substream, buf_size);
	if (retval < 0)
		return retval;
	pr_debug("%s:allocated memory = %d\n", __func__, buf_size);
	/* mark the pages as uncached region */
	addr = (unsigned long) substream->runtime->dma_area;
	pages = (substream->runtime->dma_bytes + PAGE_SIZE - 1) / PAGE_SIZE;
	retval = set_memory_uc(addr, pages);
	if (retval) {
		pr_err("set_memory_uc failed.Error:%d\n", retval);
		return retval;
	}
	memset(substream->runtime->dma_area, 0, buf_size);

	return retval;
}

/**
 * snd_intelhad_hw_free- to release the resources allocated during
 * hardware params setup
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework before close callback.
 *
 */
static int snd_intelhad_hw_free(struct snd_pcm_substream *substream)
{
	unsigned long addr;
	u32 pages;

	pr_debug("snd_intelhad_hw_free called\n");

	/* mark back the pages as cached/writeback region before the free */
	if (substream->runtime->dma_area != NULL) {
		addr = (unsigned long) substream->runtime->dma_area;
		pages = (substream->runtime->dma_bytes + PAGE_SIZE - 1) /
								PAGE_SIZE;
		set_memory_wb(addr, pages);
		return snd_pcm_lib_free_pages(substream);
	}
	return 0;
}

/**
 * snd_intelhad_pcm_trigger - stream activities are handled here
 * @substream:substream for which the stream function is called
 * @cmd:the stream commamd thats requested from upper layer
 * This function is called whenever an a stream activity is invoked
 */
static int snd_intelhad_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	int caps, retval = 0;
	unsigned long flag_irq;
	struct snd_intelhad *intelhaddata;
	struct had_stream_pvt *stream;
	struct had_pvt_data *had_stream;

	pr_debug("snd_intelhad_pcm_trigger called\n");

	intelhaddata = snd_pcm_substream_chip(substream);
	stream = substream->runtime->private_data;
	had_stream = intelhaddata->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("Trigger Start\n");

		/* Disable local INTRs till register prgmng is done */
		if (had_get_hwstate(intelhaddata)) {
			pr_err("_START: HDMI cable plugged-out\n");
			retval = -ENODEV;
			break;
		}
		stream->stream_status = STREAM_RUNNING;

		had_stream->stream_type = HAD_RUNNING_STREAM;

		/* Enable Audio */
		/*
		 * ToDo: Need to enable UNDERRUN interrupts as well
		 *   caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		 */
		caps = HDMI_AUDIO_BUFFER_DONE;
		retval = had_set_caps(intelhaddata, HAD_SET_ENABLE_AUDIO_INT,
				      &caps);
		retval = had_set_caps(intelhaddata, HAD_SET_ENABLE_AUDIO, NULL);
		snd_intelhad_enable_audio(substream, 1);

		pr_debug("Processed _Start\n");

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("Trigger Stop\n");
		spin_lock_irqsave(&intelhaddata->had_spinlock, flag_irq);
		intelhaddata->stream_info.str_id = 0;
		intelhaddata->curr_buf = 0;

		/* Stop reporting BUFFER_DONE/UNDERRUN to above layers*/

		had_stream->stream_type = HAD_INIT;
		spin_unlock_irqrestore(&intelhaddata->had_spinlock, flag_irq);
		/* Disable Audio */
		/*
		 * ToDo: Need to disable UNDERRUN interrupts as well
		 *   caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		 */
		caps = HDMI_AUDIO_BUFFER_DONE;
		had_set_caps(intelhaddata, HAD_SET_DISABLE_AUDIO_INT, &caps);
		snd_intelhad_enable_audio(substream, 0);
		/* Reset buffer pointers */
		snd_intelhad_reset_audio(intelhaddata, 1);
		snd_intelhad_reset_audio(intelhaddata, 0);
		stream->stream_status = STREAM_DROPPED;
		had_set_caps(intelhaddata, HAD_SET_DISABLE_AUDIO, NULL);
		break;

	default:
		retval = -EINVAL;
	}
	return retval;
}

/**
 * snd_intelhad_pcm_prepare- internal preparation before starting a stream
 *
 * @substream:  substream for which the function is called
 *
 * This function is called when a stream is started for internal preparation.
 */
static int snd_intelhad_pcm_prepare(struct snd_pcm_substream *substream)
{
	int retval;
	u32 disp_samp_freq, n_param;
	u32 link_rate = 0;
	struct snd_intelhad *intelhaddata;
	struct snd_pcm_runtime *runtime;
	struct had_pvt_data *had_stream;

	pr_debug("snd_intelhad_pcm_prepare called\n");

	intelhaddata = snd_pcm_substream_chip(substream);
	runtime = substream->runtime;
	had_stream = intelhaddata->private_data;

	if (had_get_hwstate(intelhaddata)) {
		pr_err("%s: HDMI cable plugged-out\n", __func__);
		retval = -ENODEV;
		goto prep_end;
	}

	pr_debug("period_size=%d\n",
		(int)frames_to_bytes(runtime, runtime->period_size));
	pr_debug("periods=%d\n", runtime->periods);
	pr_debug("buffer_size=%d\n", (int)snd_pcm_lib_buffer_bytes(substream));
	pr_debug("rate=%d\n", runtime->rate);
	pr_debug("channels=%d\n", runtime->channels);

	if (intelhaddata->stream_info.str_id) {
		pr_debug("_prepare is called for existing str_id#%d\n",
					intelhaddata->stream_info.str_id);
		retval = snd_intelhad_pcm_trigger(substream,
						SNDRV_PCM_TRIGGER_STOP);
		return retval;
	}

	retval = snd_intelhad_init_stream(substream);
	if (retval)
		goto prep_end;


	/* Get N value in KHz */
	retval = had_get_caps(intelhaddata, HAD_GET_DISPLAY_RATE,
			      &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto prep_end;
	}

	had_get_caps(intelhaddata, HAD_GET_ELD, &intelhaddata->eeld);
	had_get_caps(intelhaddata, HAD_GET_DP_OUTPUT, &intelhaddata->dp_output);

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
				     intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto prep_end;
	}

	if (intelhaddata->dp_output)
		had_get_caps(intelhaddata, HAD_GET_LINK_RATE, &link_rate);


	snd_intelhad_prog_cts(substream->runtime->rate,
			      disp_samp_freq, link_rate,
			      n_param, intelhaddata);

	snd_intelhad_prog_dip(substream, intelhaddata);

	retval = snd_intelhad_audio_ctrl(substream, intelhaddata);

	/* Prog buffer address */
	retval = snd_intelhad_prog_buffer(intelhaddata,
			HAD_BUF_TYPE_A, HAD_BUF_TYPE_D);

	/*
	 * Program channel mapping in following order:
	 * FL, FR, C, LFE, RL, RR
	 */

	had_write_register(intelhaddata, AUD_BUF_CH_SWAP, SWAP_LFE_CENTER);

prep_end:
	return retval;
}

/**
 * snd_intelhad_pcm_pointer- to send the current buffer pointerprocessed by hw
 *
 * @substream:  substream for which the function is called
 *
 * This function is called by ALSA framework to get the current hw buffer ptr
 * when a period is elapsed
 */
static snd_pcm_uframes_t snd_intelhad_pcm_pointer(
					struct snd_pcm_substream *substream)
{
	struct snd_intelhad *intelhaddata;
	u32 bytes_rendered = 0;
	u32 t;
	int buf_id;

	/* pr_debug("snd_intelhad_pcm_pointer called\n"); */

	intelhaddata = snd_pcm_substream_chip(substream);

	if (intelhaddata->flag_underrun) {
		intelhaddata->flag_underrun = 0;
		return SNDRV_PCM_POS_XRUN;
	}

	/* Use a hw register to calculate sub-period position reports.
	 * This makes PulseAudio happier.
	 */

	buf_id = intelhaddata->curr_buf % 4;
	had_read_register(intelhaddata,
			  AUD_BUF_A_LENGTH + (buf_id * HAD_REG_WIDTH), &t);

	if ((t == 0) || (t == ((u32)-1L))) {
		intelhaddata->underrun_count++;
		pr_debug("discovered buffer done for buf %d, count = %d\n",
			 buf_id, intelhaddata->underrun_count);

		if (intelhaddata->underrun_count > (HAD_MIN_PERIODS/2)) {
			pr_debug("assume audio_codec_reset, underrun = %d - do xrun\n",
				 intelhaddata->underrun_count);
			intelhaddata->underrun_count = 0;
			return SNDRV_PCM_POS_XRUN;
		}
	} else {
		/* Reset Counter */
		intelhaddata->underrun_count = 0;
	}

	t = intelhaddata->buf_info[buf_id].buf_size - t;

	if (intelhaddata->stream_info.buffer_rendered)
		div_u64_rem(intelhaddata->stream_info.buffer_rendered,
			intelhaddata->stream_info.ring_buf_size,
			&(bytes_rendered));

	intelhaddata->stream_info.buffer_ptr = bytes_to_frames(
						substream->runtime,
						bytes_rendered + t);
	return intelhaddata->stream_info.buffer_ptr;
}

/**
 * snd_intelhad_pcm_mmap- mmaps a kernel buffer to user space for copying data
 *
 * @substream:  substream for which the function is called
 * @vma:		struct instance of memory VMM memory area
 *
 * This function is called by OS when a user space component
 * tries to get mmap memory from driver
 */
static int snd_intelhad_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{

	pr_debug("snd_intelhad_pcm_mmap called\n");

	pr_debug("entry with prot:%s\n", __func__);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			substream->dma_buffer.addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

int hdmi_audio_mode_change(struct snd_pcm_substream *substream)
{
	int retval = 0;
	u32 disp_samp_freq, n_param;
	u32 link_rate = 0;
	struct snd_intelhad *intelhaddata;

	intelhaddata = snd_pcm_substream_chip(substream);

	/* Disable Audio */
	snd_intelhad_enable_audio(substream, 0);

	/* Update CTS value */
	retval = had_get_caps(intelhaddata, HAD_GET_DISPLAY_RATE,
			      &disp_samp_freq);
	if (retval) {
		pr_err("querying display sampling freq failed %#x\n", retval);
		goto out;
	}

	retval = snd_intelhad_prog_n(substream->runtime->rate, &n_param,
				     intelhaddata);
	if (retval) {
		pr_err("programming N value failed %#x\n", retval);
		goto out;
	}

	if (intelhaddata->dp_output)
		had_get_caps(intelhaddata, HAD_GET_LINK_RATE, &link_rate);

	snd_intelhad_prog_cts(substream->runtime->rate,
			      disp_samp_freq, link_rate,
			      n_param, intelhaddata);

	/* Enable Audio */
	snd_intelhad_enable_audio(substream, 1);

out:
	return retval;
}

/*PCM operations structure and the calls back for the same */
struct snd_pcm_ops snd_intelhad_playback_ops = {
	.open =		snd_intelhad_open,
	.close =	snd_intelhad_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_intelhad_hw_params,
	.hw_free =	snd_intelhad_hw_free,
	.prepare =	snd_intelhad_pcm_prepare,
	.trigger =	snd_intelhad_pcm_trigger,
	.pointer =	snd_intelhad_pcm_pointer,
	.mmap =	snd_intelhad_pcm_mmap,
};

/**
 * snd_intelhad_create - to crete alsa card instance
 *
 * @intelhaddata: pointer to internal context
 * @card: pointer to card
 *
 * This function is called when the hdmi cable is plugged in
 */
static int snd_intelhad_create(
		struct snd_intelhad *intelhaddata,
		struct snd_card *card)
{
	int retval;
	static struct snd_device_ops ops = {
	};

	pr_debug("snd_intelhad_create called\n");

	if (!intelhaddata)
		return -EINVAL;

	/* ALSA api to register the device */
	retval = snd_device_new(card, SNDRV_DEV_LOWLEVEL, intelhaddata, &ops);
	return retval;
}
/**
 * snd_intelhad_pcm_free - to free the memory allocated
 *
 * @pcm: pointer to pcm instance
 * This function is called when the device is removed
 */
static void snd_intelhad_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("Freeing PCM preallocated pages\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int had_iec958_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int had_iec958_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_intelhad *intelhaddata = snd_kcontrol_chip(kcontrol);

	ucontrol->value.iec958.status[0] = (intelhaddata->aes_bits >> 0) & 0xff;
	ucontrol->value.iec958.status[1] = (intelhaddata->aes_bits >> 8) & 0xff;
	ucontrol->value.iec958.status[2] =
					(intelhaddata->aes_bits >> 16) & 0xff;
	ucontrol->value.iec958.status[3] =
					(intelhaddata->aes_bits >> 24) & 0xff;
	return 0;
}
static int had_iec958_mask_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.iec958.status[0] = 0xff;
	ucontrol->value.iec958.status[1] = 0xff;
	ucontrol->value.iec958.status[2] = 0xff;
	ucontrol->value.iec958.status[3] = 0xff;
	return 0;
}
static int had_iec958_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	unsigned int val;
	struct snd_intelhad *intelhaddata = snd_kcontrol_chip(kcontrol);

	pr_debug("entered had_iec958_put\n");
	val = (ucontrol->value.iec958.status[0] << 0) |
		(ucontrol->value.iec958.status[1] << 8) |
		(ucontrol->value.iec958.status[2] << 16) |
		(ucontrol->value.iec958.status[3] << 24);
	if (intelhaddata->aes_bits != val) {
		intelhaddata->aes_bits = val;
		return 1;
	}
	return 1;
}

static struct snd_kcontrol_new had_control_iec958_mask = {
	.access =   SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =    SNDRV_CTL_ELEM_IFACE_PCM,
	.name =     SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
	.info =     had_iec958_info, /* shared */
	.get =      had_iec958_mask_get,
};

static struct snd_kcontrol_new had_control_iec958 = {
	.iface =    SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	.info =         had_iec958_info,
	.get =          had_iec958_get,
	.put =          had_iec958_put
};

/*
 * hdmi_audio_probe - to create sound card instance for HDMI audio playabck
 *
 * @devptr: platform device
 * @had_ret: pointer to store the created snd_intelhad object
 *
 * This function is called when the platform device is probed. This function
 * creates and registers the sound card with ALSA
 */
int hdmi_audio_probe(struct platform_device *devptr,
		     struct snd_intelhad **had_ret)
{
	int retval;
	struct snd_pcm *pcm;
	struct snd_card *card;
	struct snd_intelhad *intelhaddata;
	struct had_pvt_data *had_stream;

	pr_debug("Enter %s\n", __func__);

	pr_debug("hdmi_audio_probe dma_mask: %p\n", devptr->dev.dma_mask);

	/* allocate memory for saving internal context and working */
	intelhaddata = kzalloc(sizeof(*intelhaddata), GFP_KERNEL);
	if (!intelhaddata)
		return -ENOMEM;

	had_stream = kzalloc(sizeof(*had_stream), GFP_KERNEL);
	if (!had_stream) {
		retval = -ENOMEM;
		goto free_haddata;
	}

	spin_lock_init(&intelhaddata->had_spinlock);
	intelhaddata->drv_status = HAD_DRV_DISCONNECTED;
	pr_debug("%s @ %d:DEBUG PLUG/UNPLUG : HAD_DRV_DISCONNECTED\n",
			__func__, __LINE__);

	/* create a card instance with ALSA framework */
	retval = snd_card_new(&devptr->dev, hdmi_card_index, hdmi_card_id,
				THIS_MODULE, 0, &card);

	if (retval)
		goto free_hadstream;
	intelhaddata->card = card;
	intelhaddata->card_id = hdmi_card_id;
	intelhaddata->card_index = card->number;
	intelhaddata->private_data = had_stream;
	intelhaddata->flag_underrun = 0;
	intelhaddata->aes_bits = SNDRV_PCM_DEFAULT_CON_SPDIF;
	strncpy(card->driver, INTEL_HAD, strlen(INTEL_HAD));
	strncpy(card->shortname, INTEL_HAD, strlen(INTEL_HAD));

	retval = snd_pcm_new(card, INTEL_HAD, PCM_INDEX, MAX_PB_STREAMS,
						MAX_CAP_STREAMS, &pcm);
	if (retval)
		goto err;

	/* setup private data which can be retrieved when required */
	pcm->private_data = intelhaddata;
	pcm->private_free = snd_intelhad_pcm_free;
	pcm->info_flags = 0;
	strncpy(pcm->name, card->shortname, strlen(card->shortname));
	/* setup the ops for palyabck */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			    &snd_intelhad_playback_ops);
	/* allocate dma pages for ALSA stream operations
	 * memory allocated is based on size, not max value
	 * thus using same argument for max & size
	 */
	retval = snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_DEV, NULL,
			HAD_MAX_BUFFER, HAD_MAX_BUFFER);

	if (card->dev == NULL)
		pr_debug("card->dev is NULL!!!!! Should not be this case\n");
	else if (card->dev->dma_mask == NULL)
		pr_debug("hdmi_audio_probe dma_mask is NULL!!!!!\n");
	else
		pr_debug("hdmi_audio_probe dma_mask is : %p\n",
				card->dev->dma_mask);

	if (retval)
		goto err;

	/* internal function call to register device with ALSA */
	retval = snd_intelhad_create(intelhaddata, card);
	if (retval)
		goto err;

	card->private_data = &intelhaddata;
	retval = snd_card_register(card);
	if (retval)
		goto err;

	/* IEC958 controls */
	retval = snd_ctl_add(card, snd_ctl_new1(&had_control_iec958_mask,
						intelhaddata));
	if (retval < 0)
		goto err;
	retval = snd_ctl_add(card, snd_ctl_new1(&had_control_iec958,
						intelhaddata));
	if (retval < 0)
		goto err;

	init_channel_allocations();

	/* Register channel map controls */
	retval = had_register_chmap_ctls(intelhaddata, pcm);
	if (retval < 0)
		goto err;

	intelhaddata->dev = &devptr->dev;
	pm_runtime_set_active(intelhaddata->dev);
	pm_runtime_enable(intelhaddata->dev);

	*had_ret = intelhaddata;

	return 0;
err:
	snd_card_free(card);
free_hadstream:
	kfree(had_stream);
	pm_runtime_disable(intelhaddata->dev);
	intelhaddata->dev = NULL;
free_haddata:
	kfree(intelhaddata);
	intelhaddata = NULL;
	pr_err("Error returned from %s api %#x\n", __func__, retval);
	return retval;
}

/*
 * hdmi_audio_remove - removes the alsa card
 *
 *@haddata: pointer to HAD private data
 *
 * This function is called when the hdmi cable is un-plugged. This function
 * free the sound card.
 */
int hdmi_audio_remove(struct snd_intelhad *intelhaddata)
{
	int caps;

	pr_debug("Enter %s\n", __func__);

	if (!intelhaddata)
		return 0;

	if (intelhaddata->drv_status != HAD_DRV_DISCONNECTED) {
		caps = HDMI_AUDIO_UNDERRUN | HDMI_AUDIO_BUFFER_DONE;
		had_set_caps(intelhaddata, HAD_SET_DISABLE_AUDIO_INT, &caps);
		had_set_caps(intelhaddata, HAD_SET_DISABLE_AUDIO, NULL);
	}
	snd_card_free(intelhaddata->card);
	kfree(intelhaddata->private_data);
	kfree(intelhaddata);
	return 0;
}

MODULE_AUTHOR("Sailaja Bandarupalli <sailaja.bandarupalli@intel.com>");
MODULE_AUTHOR("Ramesh Babu K V <ramesh.babu@intel.com>");
MODULE_AUTHOR("Vaibhav Agarwal <vaibhav.agarwal@intel.com>");
MODULE_AUTHOR("Jerome Anand <jerome.anand@intel.com>");
MODULE_DESCRIPTION("Intel HDMI Audio driver");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{Intel,Intel_HAD}");
