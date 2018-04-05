/*
 *  cht-bsw-nau8824.c - ASoc Machine driver for Intel Cherryview-based
 *  platforms Cherrytrail and Braswell, with nau8824 codec.
 *
 *  Copyright (C) 2015 Bitquant Research Laboratories (Asia) Limited
 *  Author: Wang, Joseph C <joequant@gmail.com>
 *  This file is modified from cht_bsw_max98090_ti.c
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
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <uapi/linux/input-event-codes.h>
#include "../../codecs/nau8824.h"
#include "../atom/sst-atom-controls.h"
#include "../common/sst-acpi.h"

#define CHT_PLAT_CLK_3_HZ	19200000
#define CHT_CODEC_DAI	"nau8824-hifi"

struct cht_acpi_card {
  char *codec_id;
  int codec_type;
  struct snd_soc_card *soc_card;
};

static struct snd_soc_jack_pin cht_bsw_jack_pins[] = {
  {
    .pin    = "Headphone",
    .mask   = SND_JACK_HEADPHONE,
  },
  {
    .pin    = "Headset Mic",
    .mask   = SND_JACK_MICROPHONE,
  },
};

struct cht_mc_private {
  struct snd_soc_jack jack;
  struct cht_acpi_card *acpi_card;
};

static inline struct snd_soc_dai *cht_get_codec_dai(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (!strncmp(rtd->codec_dai->name, CHT_CODEC_DAI,
			     strlen(CHT_CODEC_DAI)))
			return rtd->codec_dai;
	}
	return NULL;
}

static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route cht_audio_map[] = {
	/* External Speakers: SPKOUTL, SPKOUTR */
	{"Speaker", NULL, "SPKOUTL"},
	{"Speaker", NULL, "SPKOUTR"},
	/* Headset Stereophone(Headphone): HPOL, HPOR */
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},

	{"DMIC1", NULL, "Internal Mic"},
	{"DMIC2", NULL, "Internal Mic"},
	{"DMIC3", NULL, "Internal Mic"},
	{"DMIC4", NULL, "Internal Mic"},
	/* Headset Mic: Headset Mic with bias */
	{"HSMIC1", NULL, "Headset Mic"},
	{"HSMIC2", NULL, "Headset Mic"},

	{"Playback", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "codec_out0"},
	{"ssp2 Tx", NULL, "codec_out1"},
	{"codec_in0", NULL, "ssp2 Rx" },
	{"codec_in1", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "Capture"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8824_CLK_FLL_FS, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(codec_dai->dev, "can't set FS clock %d\n", ret);

	ret = snd_soc_dai_set_pll(codec_dai, 0, 0, params_rate(params),
		params_rate(params) * 256);
	if (ret < 0)
		dev_err(codec_dai->dev, "can't set FLL: %d\n", ret);

	return 0;
}

static int cht_codec_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	int jack_type;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	struct snd_soc_jack *jack = &ctx->jack;
	struct snd_soc_codec *codec = runtime->codec;

	/**
	* NAU88L24 supports 4 butons headset detection
	* KEY_MEDIA
	* KEY_VOICECOMMAND
	* KEY_VOLUMEUP
	* KEY_VOLUMEDOWN
	*/
	jack_type = SND_JACK_HEADPHONE | SND_JACK_BTN_0 | SND_JACK_BTN_1 |
	  SND_JACK_BTN_2 | SND_JACK_BTN_3;
	ret = snd_soc_card_jack_new(runtime->card, "Headset",
				    jack_type, jack,
				    cht_bsw_jack_pins,
				    ARRAY_SIZE(cht_bsw_jack_pins));
	if (ret) {
		dev_err(runtime->dev, "Headset Jack creation failed %d\n",
			ret);
		return ret;
	}
	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	nau8824_enable_jack_detect(codec, jack);

	return ret;
}

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	int ret = 0;
	unsigned int fmt = 0;

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 24);
	if (ret < 0) {
	  dev_err(rtd->dev, "can't set I2S config, err %d\n", ret);
	  return ret;
	}
	
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_IB_NF
	  | SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(rtd->cpu_dai, fmt);
	if (ret < 0) {
	  dev_err(rtd->dev, "can't set cpu_dai set fmt: %d\n", ret);
	  return ret;
	}

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	params_set_format(params, SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int cht_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_single(substream->runtime,
			SNDRV_PCM_HW_PARAM_RATE, 48000);
}

static struct snd_soc_ops cht_aif1_ops = {
	.startup = cht_aif1_startup,
};

static struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_dai_link cht_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "media-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_aif1_ops,
	},
	[MERR_DPCM_DEEP_BUFFER] = {
		.name = "Deep-Buffer Audio Port",
		.stream_name = "Deep-Buffer Audio",
		.cpu_dai_name = "deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
		.nonatomic = true,
		.dynamic = 1,
		.dpcm_playback = 1,
		.ops = &cht_aif1_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Compressed Port",
		.stream_name = "Compress",
		.cpu_dai_name = "compress-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-mfld-platform",
	},
	/* back ends */
	{
		.name = "SSP2-Codec",
		.id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-mfld-platform",
		.no_pcm = 1,
		.codec_dai_name = "nau8824-hifi",
		.codec_name = "i2c-10508824:00",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_IB_NF
					| SND_SOC_DAIFMT_CBS_CFS,
		.init = cht_codec_init,
		.be_hw_params_fixup = cht_codec_fixup,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cht_be_ssp2_ops,
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_cht = {
	.name = "chtnau8824",
	.owner = THIS_MODULE,
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static struct cht_acpi_card snd_soc_cards[] = {
  {"10508824", 0, &snd_soc_card_cht}
};

static char cht_rt5640_codec_name[16];

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct cht_mc_private *drv;
	struct snd_soc_card *card = snd_soc_cards[0].soc_card;
	char codec_name[16];
	struct sst_acpi_mach *mach;
	const char *i2c_name = NULL;
	int dai_index = 0;
	int i;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(snd_soc_cards); i++) {
	  if (acpi_dev_found(snd_soc_cards[i].codec_id)) {
	    dev_dbg(&pdev->dev,
		    "found codec %s\n", snd_soc_cards[i].codec_id);
	    card = snd_soc_cards[i].soc_card;
	    drv->acpi_card = &snd_soc_cards[i];
	    break;
	  }
	}
	card->dev = &pdev->dev;
	mach = card->dev->platform_data;
	sprintf(codec_name, "i2c-%s:00", drv->acpi_card->codec_id);

	/* set correct codec name */
	for (i = 0; i < ARRAY_SIZE(cht_dailink); i++)
	  if (!strcmp(card->dai_link[i].codec_name, "i2c-10508824:00")) {
	    card->dai_link[i].codec_name = kstrdup(codec_name, GFP_KERNEL);
	    dai_index = i;
	  }

	/* fixup codec name based on HID */
	i2c_name = sst_acpi_find_name_from_hid(mach->id);
	if (i2c_name != NULL) {
	  snprintf(cht_rt5640_codec_name, sizeof(cht_rt5640_codec_name),
		   "%s%s", "i2c-", i2c_name);
	  cht_dailink[dai_index].codec_name = cht_rt5640_codec_name;
	}

	/* register the soc card */
	snd_soc_card_cht.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_cht, drv);
	ret_val = devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_cht);
	if (ret_val) {
		dev_err(&pdev->dev,
			"snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_cht);
	return ret_val;
}

static struct platform_driver snd_cht_mc_driver = {
	.driver = {
		.name = "cht-bsw-nau8824",
	},
	.probe = snd_cht_mc_probe,
};

module_platform_driver(snd_cht_mc_driver)

MODULE_DESCRIPTION("ASoC Intel(R) Braswell Machine driver");
MODULE_AUTHOR("Wang, Joseph C <joequant@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht-bsw-nau8824");
