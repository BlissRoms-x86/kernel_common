/*
 *  bxt_florida.c - ASoC Machine driver for Intel Broxton platform
 *
 *  Copyright (C) 2014 Wolfson Micro
 *  Copyright (C) 2014-16 Intel Corp
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
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

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

#include <linux/mfd/arizona/registers.h>
#include "../../codecs/wm5110.h"
#include "../../codecs/wm8998.h"


/* Codec PLL output clk rate */
#define CODEC_SYSCLK_RATE			49152000
/* Input clock to codec at MCLK1 PIN */
#define CODEC_IN_MCLK1_RATE			19200000
/* Input clock to codec at MCLK2 PIN */
#define CODEC_IN_MCLK2_RATE			32768
/* Input bit clock to codec */
#define CODEC_IN_BCLK_RATE			4800000

/*  define to select between MCLK1 and MCLK2 input to codec as its clock */
#define CODEC_IN_MCLK1				1
#define CODEC_IN_MCLK2				2
#define CODEC_IN_BCLK				3

#define SLOT_MASK(x) ((1 << x) - 1)
bool is_codec8998;
struct mrgfld_mc_private {
	u8		pmic_id;
	void __iomem    *osc_clk0_reg;
	int bt_mode;
};

static inline struct snd_soc_codec *mrgfld_florida_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;
	char *codec_name;

	if (is_codec8998)
		codec_name = "wm8998-codec";
	else
		codec_name = "wm5110-codec";

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->component.name, codec_name)) {
			pr_debug("codec was %s", codec->component.name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

/* Function to switch the input clock for codec,  When audio is in
 * progress input clock to codec will be through MCLK1 which is 19.2MHz
 * while in off state input clock to codec will be through 32KHz through
 * MCLK2
 * card	: Sound card structure
 * src	: Input clock source to codec
 */

static int mrgfld_florida_set_codec_clk(struct snd_soc_codec *florida_codec, int src)
{
	int ret;

	pr_debug("mrgfld_florida_set_codec_clk: source %d\n", src);

	/*reset FLL1*/
	snd_soc_codec_set_pll(florida_codec, WM5110_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE, 0, 0);
	snd_soc_codec_set_pll(florida_codec, WM5110_FLL1,
				ARIZONA_FLL_SRC_NONE, 0, 0);

	switch (src) {
	case CODEC_IN_MCLK1:
		/* Turn ON the PLL to generate required sysclk rate
		 * from MCLK1 */
		ret = snd_soc_codec_set_pll(florida_codec, WM5110_FLL1,
				ARIZONA_CLK_SRC_MCLK1, CODEC_IN_MCLK1_RATE,
				CODEC_SYSCLK_RATE);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to enable FLL1 with Ref(MCLK) Clock Loop: %d\n", ret);
			return ret;
		}
		break;
	case CODEC_IN_BCLK:
		/* Turn ON the PLL to generate required sysclk rate
		 * from BCLK */
		ret = snd_soc_codec_set_pll(florida_codec, WM5110_FLL1,
				ARIZONA_CLK_SRC_AIF1BCLK, CODEC_IN_BCLK_RATE,
				CODEC_SYSCLK_RATE);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);
			return ret;
		}

		break;
	default:
		return -EINVAL;
	}

	/*Switch to PLL*/
	ret = snd_soc_codec_set_sysclk(florida_codec,
			ARIZONA_CLK_SYSCLK, ARIZONA_CLK_SRC_FLL1,
			CODEC_SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(florida_codec->dev, "Failed to set SYSCLK to FLL1: %d\n", ret);
		return ret;
	}

	return 0;
}


static int mrgfld_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{

	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *florida_codec = mrgfld_florida_get_codec(card);
	int ret = 0;

	if (!florida_codec) {
		pr_err("%s: florida codec not found\n", __func__);
		return -EINVAL;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("%s %d Event On\n", __func__, __LINE__);
		/* TODO: Ideally MCLK should be used to drive codec PLL
		 * currently we are using BCLK
		 */
		ret = mrgfld_florida_set_codec_clk(florida_codec, CODEC_IN_BCLK);
	} else {
		pr_info("%s %d Event Off\n", __func__, __LINE__);
		/* TODO: Switch to 32K clock for saving power. */
		pr_info("Currently we are not switching to 32K PMIC clock\n");
	}
	return ret;

}
static const struct snd_soc_dapm_widget mrgfld_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SPK("EP", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
	SND_SOC_DAPM_MIC("HDMI", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			mrgfld_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),

};

static int bxt_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->min = channels->max = 2;
	return 0;
}

static unsigned int rates[] = {
	48000,
};
static unsigned int channels_dmic[] = {
	2, 4,
};

static const struct snd_soc_dapm_route mrgfld_wm5110_map[] = {
	/*Headphones*/
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/*Speakers*/
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},

	{ "AMIC", NULL, "MICBIAS2" },
	{ "AMIC", NULL, "MICBIAS1" },
	{ "IN1L", NULL, "AMIC" },
	{ "IN1R", NULL, "AMIC" },

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1 Playback", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "codec1_out"},
	{ "ssp0 Tx", NULL, "codec0_out"},

	{ "ssp0 Rx", NULL, "AIF1 Capture" },
	{ "codec0_in", NULL, "ssp0 Rx" },
	{ "codec1_in", NULL, "ssp0 Rx" },

	{ "DMic", NULL, "SoC DMIC"},

	{ "DMIC01 Rx", NULL, "Capture" },
	{ "DMIC23 Rx", NULL, "Capture" },
	{ "dmic01_hifi", NULL, "DMIC01 Rx" },
	{ "dmic23_hifi", NULL, "DMIC23 Rx" },


	{"Headphones", NULL, "Platform Clock"},
	{"AMIC", NULL, "Platform Clock"},
	{"DMIC", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
	{"EP", NULL, "Platform Clock"},
	{"Tone Generator 1", NULL, "Platform Clock" },
	{"Tone Generator 2", NULL, "Platform Clock" },
};


static const struct snd_soc_dapm_route mrgfld_wm8998_map[] = {
	/*Headphones*/
	{ "Headphones", NULL, "HPOUTL" },
	{ "Headphones", NULL, "HPOUTR" },

	/*Speakers*/
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},

	/*Earpiece*/
	{ "EP", NULL, "EPOUTN" },
	{ "EP", NULL, "EPOUTP" },

	{"IN3L", NULL, "DMIC"},
	{"IN3R", NULL, "DMIC"},
	{"DMIC", NULL, "MICVDD"},

	{ "AMIC", NULL, "MICBIAS2" },
	{ "AMIC", NULL, "MICBIAS1" },
	{ "IN2B", NULL, "AMIC" },
	{ "DMic", NULL, "SoC DMIC"},

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1 Playback", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "codec1_out"},
	{ "ssp0 Tx", NULL, "codec0_out"},

	{ "ssp0 Rx", NULL, "AIF1 Capture" },
	{ "codec0_in", NULL, "ssp0 Rx" },
	{ "codec1_in", NULL, "ssp0 Rx" },

	{ "DMIC01 Rx", NULL, "Capture" },
	{ "DMIC23 Rx", NULL, "Capture" },
	{ "dmic01_hifi", NULL, "DMIC01 Rx" },
	{ "dmic23_hifi", NULL, "DMIC23 Rx" },


	/* TODO: map for rest of the ports */

	{"Headphones", NULL, "Platform Clock"},
	{"AMIC", NULL, "Platform Clock"},
	{"DMIC", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
	{"EP", NULL, "Platform Clock"},
	{"Tone Generator 1", NULL, "Platform Clock" },
	{"Tone Generator 2", NULL, "Platform Clock" },
};

static const struct snd_kcontrol_new mrgfld_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("EP"),
	SOC_DAPM_PIN_SWITCH("AMIC"),
	SOC_DAPM_PIN_SWITCH("DMIC"),
};

static int mrgfld_florida_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_component *component = runtime->cpu_dai->component;


	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, mrgfld_controls,
					ARRAY_SIZE(mrgfld_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

	dapm = snd_soc_component_get_dapm(component);
	snd_soc_dapm_ignore_suspend(dapm, "Reference Capture");
	snd_soc_dapm_ignore_suspend(&card->dapm, "SoC DMIC");

	return 0;
}

static unsigned int rates_48000[] = {
	48000,
	16000,
	8000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrgfld_florida_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrgfld_florida_ops = {
	.startup = mrgfld_florida_startup,
};

static int mrgfld_florida_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);

	pr_debug("param width set to:0x%x\n",
			snd_pcm_format_width(params_format(params)));

	return 0;
}

static int mrgfld_ssp_florida_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *florida_dai= rtd->codec_dai;
	unsigned int fmt;
	int ret;

	ret = snd_soc_dai_set_tdm_slot(florida_dai, 0, 0, 4, 24);
	/* slot width is set as 25, SNDRV_PCM_FORMAT_S32_LE */
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* bit clock inverse not required */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(florida_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops mrgfld_ssp_florida_ops = {
	.hw_params =  mrgfld_ssp_florida_hw_params,
};

struct snd_soc_dai_link mrgfld_florida_msic_dailink[] = {
	{
		.name = "Bxtn Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = mrgfld_florida_init,
		.nonatomic = 1,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &mrgfld_florida_ops,
	},

	/* back ends */
	{
		.name = "SSP0-Codec",
		.id = 1,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = "wm5110-codec",
		.codec_dai_name = "wm5110-aif1",
		.platform_name = "0000:00:0e.0",
		.be_hw_params_fixup = mrgfld_florida_codec_fixup,
		.ops = &mrgfld_ssp_florida_ops,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},

	{
		.name = "dmic01",
		.id = 2,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = bxt_dmic_fixup,
	},

	{
		.name = "dmic23",
		.id = 3,
		.cpu_dai_name = "DMIC23 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.be_hw_params_fixup = bxt_dmic_fixup,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},

};

struct snd_soc_dai_link mrgfld_wm8998_msic_dailink[] = {
	{
		.name = "Bxtn Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = mrgfld_florida_init,
		.nonatomic = 1,
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &mrgfld_florida_ops,
	},

	/* back ends */
	{
		.name = "SSP0-Codec",
		.id = 1,
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:00:0e.0",
		.codec_name = "wm8998-codec",
		.codec_dai_name = "wm8998-aif1",
		.be_hw_params_fixup = mrgfld_florida_codec_fixup,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},

	{
		.name = "dmic01",
		.id = 2,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = bxt_dmic_fixup,
	},

	{
		.name = "dmic23",
		.id = 3,
		.cpu_dai_name = "DMIC23 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.dpcm_capture = 1,
		.no_pcm = 1,
		.be_hw_params_fixup = bxt_dmic_fixup,
        },

};

#ifdef CONFIG_PM_SLEEP
static int snd_mrgfld_florida_prepare(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_mrgfld_florida_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_mrgfld_florida_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_mrgfld_florida_prepare NULL
#define snd_mrgfld_florida_complete NULL
#define snd_mrgfld_florida_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_wm8998_mrgfld = {
	.name = "wm8998-audio",
	.dai_link =  mrgfld_wm8998_msic_dailink,
	.num_links = ARRAY_SIZE(mrgfld_wm8998_msic_dailink),
	.dapm_widgets = mrgfld_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mrgfld_widgets),
	.dapm_routes = mrgfld_wm8998_map,
	.num_dapm_routes = ARRAY_SIZE(mrgfld_wm8998_map),
	.fully_routed = false,
};

static int snd_mrgfld_florida_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrgfld_mc_private *drv;
	struct snd_soc_card *card_mrgfld = NULL;

	is_codec8998 = true;
	card_mrgfld = &snd_soc_card_wm8998_mrgfld;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv)
		return -ENOMEM;

	card_mrgfld->dev = &pdev->dev;
	snd_soc_card_set_drvdata(card_mrgfld, drv);
	/* Register the card */
	ret_val = snd_soc_register_card(card_mrgfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, card_mrgfld);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	devm_kfree(&pdev->dev, drv);
	return ret_val;
}

static int snd_mrgfld_florida_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrgfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);

	devm_kfree(&pdev->dev, drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

const struct dev_pm_ops snd_mrgfld_florida_mc_pm_ops = {
	.prepare = snd_mrgfld_florida_prepare,
	.complete = snd_mrgfld_florida_complete,
	.poweroff = snd_mrgfld_florida_poweroff,
};

static struct platform_driver snd_mrgfld_florida_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrgfld_florida",
	},
	.probe = snd_mrgfld_florida_mc_probe,
	.remove = snd_mrgfld_florida_mc_remove,
};

static int snd_mrgfld_florida_driver_init(void)
{
	pr_info("Morganfield Machine Driver mrgfld_florida registered\n");
	return platform_driver_register(&snd_mrgfld_florida_mc_driver);
}
module_init(snd_mrgfld_florida_driver_init);

static void snd_mrgfld_florida_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_mrgfld_florida_mc_driver);
}
module_exit(snd_mrgfld_florida_driver_exit)

MODULE_DESCRIPTION("ASoC Morganfield Machine driver");
MODULE_AUTHOR("Samreen Nilofer <samreen.nilofer@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrgfld_florida");
