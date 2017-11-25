/*
 *  icl_rt274.c - ASOC Machine driver for ICL
 *
 *  Copyright (C) 2016 Intel Corp
 *  Author: Dronamraju Santosh Pavan Kumar <santosh.pavan.kumarx.dronamraju@intel.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>

#include "../../codecs/rt274.h"

#define ICL_FREQ_OUT		19200000
#define ICL_BE_FIXUP_RATE	48000
#define RT274_CODEC_DAI		"rt274-aif1"

static struct snd_soc_dai *icl_get_codec_dai(struct snd_soc_card *card,
						     const char *dai_name)
{
	struct snd_soc_pcm_runtime *rtd;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (!strcmp(rtd->codec_dai->name, dai_name))
			return rtd->codec_dai;
	}

	return NULL;
}

static int icl_rt274_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	int ret = 0, ratio = 100;
	struct snd_soc_dai *codec_dai = icl_get_codec_dai(card,
							  RT274_CODEC_DAI);

	/* Codec needs clock for Jack detection and button press */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT274_SCLK_S_PLL2,
				     ICL_FREQ_OUT, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "set codec sysclk failed: %d\n", ret);
		return ret;
	}

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_dai_set_bclk_ratio(codec_dai, ratio);

		ret = snd_soc_dai_set_pll(codec_dai, 0, RT274_PLL2_S_BCLK,
					  ICL_BE_FIXUP_RATE * ratio,
					  ICL_FREQ_OUT);
		if (ret) {
			dev_err(codec_dai->dev,
				"failed to enable PLL2: %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static struct snd_soc_jack icl_headset;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin icl_headset_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct snd_kcontrol_new icl_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

static const struct snd_soc_dapm_widget icl_rt274_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			icl_rt274_clock_control, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_pcm_stream dai_params_codec = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int icl_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->min = channels->max = 2;

	return 0;
}

static const struct snd_soc_dapm_route icl_map[] = {
	{"Headphone Jack", NULL, "HPO Pin"},
	{"MIC", NULL, "Mic Jack"},
	{"DMic", NULL, "SoC DMIC"},
	{"DMIC01 Rx", NULL, "Capture"},
	{"dmic01_hifi", NULL, "DMIC01 Rx"},

	/* ssp2 path */
	{"Dummy Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "ssp2_out"},

	{"ssp2 Rx", NULL, "Dummy Capture"},
	{"ssp2_in", NULL, "ssp2 Rx"},

	/* ssp1 path */
	{"Dummy Playback", NULL, "ssp1 Tx"},
	{"ssp1 Tx", NULL, "ssp1_out"},

	{"AIF1 Playback", NULL, "ssp0 Tx"},
	{"ssp0 Tx", NULL, "codec1_out"},
	{"ssp0 Tx", NULL, "codec0_out"},

	{"ssp0 Rx", NULL, "AIF1 Capture"},
	{"codec0_in", NULL, "ssp0 Rx"},

	{"Headphone Jack", NULL, "Platform Clock"},
	{"MIC", NULL, "Platform Clock"},
};

static int icl_rt274_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *codec_dai = runtime->codec_dai;

	ret = snd_soc_card_jack_new(runtime->card, "Headset",
		SND_JACK_HEADSET, &icl_headset,
		icl_headset_pins, ARRAY_SIZE(icl_headset_pins));

	if (ret)
		return ret;

	rt274_mic_detect(codec, &icl_headset);

	/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4, 24);
	if (ret < 0) {
		dev_err(runtime->dev, "can't set codec pcm format %d\n", ret);
		return ret;
	}

	card->dapm.idle_bias_off = true;

	return 0;
}

static int icl_be_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = ICL_BE_FIXUP_RATE;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

#if IS_ENABLED(CONFIG_SND_SOC_INTEL_CNL_FPGA)
static const char pname[] = "0000:02:18.0";
static const char cname[] = "rt274.0-001c";
#else
static const char pname[] = "0000:00:1f.3";
static const char cname[] = "i2c-INT34C2:00";
#endif

struct snd_soc_dai_link icl_rt274_msic_dailink[] = {
	/* Trace Buffer DAI links */
	{
		.name = "ICL Trace Buffer0",
		.stream_name = "Core 0 Trace Buffer",
		.cpu_dai_name = "TraceBuffer0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.capture_only = true,
		.ignore_suspend = 1,
	},
	{
		.name = "ICL Trace Buffer1",
		.stream_name = "Core 1 Trace Buffer",
		.cpu_dai_name = "TraceBuffer1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.capture_only = true,
		.ignore_suspend = 1,
	},
	{
		.name = "ICL Trace Buffer2",
		.stream_name = "Core 2 Trace Buffer",
		.cpu_dai_name = "TraceBuffer2 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.capture_only = true,
		.ignore_suspend = 1,
	},
	{
		.name = "ICL Trace Buffer3",
		.stream_name = "Core 3 Trace Buffer",
		.cpu_dai_name = "TraceBuffer3 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.capture_only = true,
		.ignore_suspend = 1,
	},
	/* Probe DAI-links */
	{
		.name = "ICL Compress Probe playback",
		.stream_name = "Probe Playback",
		.cpu_dai_name = "Compress Probe0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
	},
	{
		.name = "ICL Compress Probe capture",
		.stream_name = "Probe Capture",
		.cpu_dai_name = "Compress Probe1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = pname,
		.init = NULL,
		.ignore_suspend = 1,
		.nonatomic = 1,
	},
	/* back ends */
	{
		.name = "SSP0-Codec",
		.id = 1,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = cname,
		.codec_dai_name = "rt274-aif1",
		.platform_name = pname,
		.be_hw_params_fixup = icl_be_fixup,
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.init = icl_rt274_init,
	},
	{
		.name = "dmic01",
		.id = 2,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = pname,
		.ignore_suspend = 1,
		.no_pcm = 1,
		.dpcm_capture = 1,
		.be_hw_params_fixup = icl_dmic_fixup,
	},
	/* codec-codec link */
	{
		.name = "ICL SSP0-Loop Port",
		.stream_name = "ICL SSP0-Loop",
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = pname,
		.codec_name = cname,
		.codec_dai_name = "rt274-aif1",
		.params = &dai_params_codec,
		.dsp_loopback = true,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
};

static int
icl_add_dai_link(struct snd_soc_card *card, struct snd_soc_dai_link *link)
{
       link->platform_name = pname;
       link->nonatomic = 1;

       return 0;
}

/* SoC card */
static struct snd_soc_card snd_soc_card_icl = {
	.name = "icl-audio",
	.dai_link = icl_rt274_msic_dailink,
	.num_links = ARRAY_SIZE(icl_rt274_msic_dailink),
	.dapm_widgets = icl_rt274_widgets,
	.num_dapm_widgets = ARRAY_SIZE(icl_rt274_widgets),
	.dapm_routes = icl_map,
	.num_dapm_routes = ARRAY_SIZE(icl_map),
	.controls = icl_controls,
	.num_controls = ARRAY_SIZE(icl_controls),
	.add_dai_link = icl_add_dai_link,
};

static int snd_icl_rt274_mc_probe(struct platform_device *pdev)
{
	snd_soc_card_icl.dev = &pdev->dev;
	return snd_soc_register_card(&snd_soc_card_icl);
}

static const struct platform_device_id icl_board_ids[] = {
	{ .name = "icl_rt274" },
	{ }
};

static struct platform_driver snd_icl_rt274_driver = {
	.driver = {
		.name = "icl_rt274",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_icl_rt274_mc_probe,
	.id_table = icl_board_ids,
};

module_platform_driver(snd_icl_rt274_driver);

MODULE_AUTHOR("Dronamraju Santosh pavan Kumar <santosh.pavan.kumarx.dronamrajus@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:icl_rt274");
