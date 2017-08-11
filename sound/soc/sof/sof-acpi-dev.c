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
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <sound/pcm.h>
#include <sound/sof.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <asm/cpu_device_id.h>
#include <asm/iosf_mbi.h>
#include "sof-priv.h"

/* machine driver reuse - platform data */
#include "../intel/common/sst-acpi.h"


static struct platform_device * 
	mfld_new_mach_data(struct snd_sof_pdata *sof_pdata)
{
	struct sst_acpi_mach pmach;
	struct device *dev = &sof_pdata->pdev->dev;
	const struct snd_sof_machine *mach = sof_pdata->machine;
	struct platform_device *pdev = NULL;

	memset(&pmach, 0, sizeof(pmach));
	memcpy((void*)pmach.id, mach->codec_id, ACPI_ID_LEN);
	pmach.drv_name = mach->drv_name;
	//pmach.board;

	pdev = platform_device_register_data(dev, mach->drv_name, -1,
		&pmach, sizeof(pmach));
	return pdev;
}

struct sof_acpi_priv {
	struct snd_sof_pdata *sof_pdata;
	struct platform_device *pdev_pcm;
};

static acpi_status mach_match(acpi_handle handle, u32 level,
				       void *context, void **ret)
{
	unsigned long long sta;
	acpi_status status;

	*(bool *)context = true;
	status = acpi_evaluate_integer(handle, "_STA", NULL, &sta);

	if (ACPI_FAILURE(status) || !(sta & ACPI_STA_DEVICE_PRESENT))
		*(bool *)context = false;

	return AE_OK;
}

static const struct snd_sof_machine *find_machine(struct device *dev,
		const struct snd_sof_machine *machines,
		const struct sof_dev_desc **desc)
{
	const struct snd_sof_machine *mach;
	bool found = false;
	int confirm;

	for (mach = machines; mach->codec_id[0]; mach++) {
		if (ACPI_SUCCESS(acpi_get_devices(mach->codec_id,
						  mach_match,
						  &found, NULL)) && found) {

			/* is help needed to confirm the machine */
			if (mach->confirm_mach) {
				confirm = mach->confirm_mach(dev, desc);
				if (confirm <= 0)
					continue;
			}

			return mach;
		}
	}

	return NULL;
}

static void sof_acpi_fw_cb(const struct firmware *fw, void *context)
{
	struct sof_acpi_priv *priv = context;
	struct snd_sof_pdata *sof_pdata = priv->sof_pdata;
	const struct snd_sof_machine *mach = sof_pdata->machine;
	struct device *dev = &sof_pdata->pdev->dev;

	sof_pdata->fw = fw;
	if (!fw) {
		dev_err(dev, "Cannot load firmware %s\n", mach->fw_filename);
		return;
	}

	/* register PCM and DAI driver */
	priv->pdev_pcm =
		platform_device_register_data(dev, "sof-audio", -1,
					      sof_pdata, sizeof(*sof_pdata));
	if (IS_ERR(priv->pdev_pcm)) {
		dev_err(dev, "Cannot register device sof-audio. Error %d\n",
			(int)PTR_ERR(priv->pdev_pcm));
	}

	return;
}

static const struct dev_pm_ops sof_acpi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(snd_sof_suspend, snd_sof_resume)
	SET_RUNTIME_PM_OPS(snd_sof_runtime_suspend, snd_sof_runtime_resume, NULL)
	.suspend_late = snd_sof_suspend_late,
};

static int sof_acpi_probe(struct platform_device *pdev)
{
	const struct acpi_device_id *id;
	struct device *dev = &pdev->dev;
	const struct sof_dev_desc *desc;
	const struct snd_sof_machine *mach;
	struct snd_sof_pdata *sof_pdata;
	struct sof_acpi_priv *priv;
	int ret = 0;

	dev_dbg(&pdev->dev, "ACPI DSP detected");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	sof_pdata = devm_kzalloc(dev, sizeof(*sof_pdata), GFP_KERNEL);
	if (sof_pdata == NULL)
		return -ENOMEM;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return -ENODEV;
	desc = (const struct sof_dev_desc*)id->driver_data;

	/* find machine */
	mach = find_machine(dev, desc->machines, &desc);
	if (mach == NULL) {
		struct snd_sof_machine *m;
		/* dont bind to any particular codec, just initialse the DSP */
		dev_err(dev, "No matching ASoC machine driver found - using nocodec\n");
		sof_pdata->drv_name = "sof-nocodec";
		m = devm_kzalloc(dev, sizeof(*mach), GFP_KERNEL);
		if (m == NULL)
			return -ENOMEM;

		m->drv_name = "sof-nocodec";
		m->fw_filename = desc->nocodec_fw_filename;
		m->tplg_filename = desc->nocodec_tplg_filename;
		m->ops = desc->machines[0].ops;
		m->asoc_plat_name = "sof-platform";/// used ???
		mach = m;
	}

	//sof_pdata->id = acpi_id->device;
	//sof_pdata->name = acpi_name(pci);
	sof_pdata->machine = mach;
	sof_pdata->desc = (struct sof_dev_desc*) id->driver_data;
	priv->sof_pdata = sof_pdata;
	sof_pdata->pdev = pdev;

	/* do we need to generate any machine plat data ? */ 
	if (mach->new_mach_data)
		sof_pdata->pdev_mach = mach->new_mach_data(sof_pdata);
	else
		/* register machine driver without plat data*/
		sof_pdata->pdev_mach =
			platform_device_register_data(dev, mach->drv_name, -1,
				NULL, 0);
	if (IS_ERR(sof_pdata->pdev_mach))
		return PTR_ERR(sof_pdata->pdev_mach);
	dev_dbg(dev, "created machine %s\n",
		dev_name(&sof_pdata->pdev_mach->dev));

	/* continue SST probing after firmware is loaded */
	ret = request_firmware_nowait(THIS_MODULE, true, mach->fw_filename,
				      dev, GFP_KERNEL, priv, sof_acpi_fw_cb);
	if (ret)
		platform_device_unregister(sof_pdata->pdev_mach);

	return ret;
}

static void sof_acpi_shutdown(struct platform_device *pdev)
{
	snd_sof_shutdown(&pdev->dev);
}

static int sof_acpi_remove(struct platform_device *pdev)
{
	struct sof_acpi_priv *priv = dev_get_drvdata(&pdev->dev);
	struct snd_sof_pdata *sof_pdata = priv->sof_pdata;

	platform_device_unregister(sof_pdata->pdev_mach);
	if (!IS_ERR_OR_NULL(priv->pdev_pcm))
		platform_device_unregister(priv->pdev_pcm);
	release_firmware(sof_pdata->fw);

	return 0;
}

#if IS_ENABLED(CONFIG_SND_SOC_SOF_HASWELL)
static struct snd_sof_machine haswell_machines[] = {
	{ "INT33CA", "haswell-audio", "intel/reef-hsw.ri",
		"intel/reef-hsw.tplg", "haswell-pcm-audio",
		&snd_sof_hsw_ops },
	{}
};

static struct sof_dev_desc sof_acpi_haswell_desc = {
	.machines = haswell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = -1,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-hsw.ri",
	.nocodec_tplg_filename = "intel/reef-hsw-nocodec.tplg"
};
#endif

#if IS_ENABLED(CONFIG_SND_SOC_SOF_BROADWELL)
static struct snd_sof_machine broadwell_machines[] = {
	{ "INT343A", "broadwell-audio", "intel/reef-bdw.ri",
		"intel/reef-bdw-rt286.tplg", "haswell-pcm-audio",
		&snd_sof_bdw_ops },
	{ "INT33CA", "haswell-audio", "intel/reef-bdw.ri",
		"intel/reef-bdw-rt5640.tplg", "haswell-pcm-audio",
		&snd_sof_bdw_ops },
	{ "RT5677CE", "bdw-rt5677", "intel/reef-bdw.ri",
		"intel/reef-bdw.tplg", "haswell-pcm-audio",
		&snd_sof_bdw_ops },
	{}
};

static struct sof_dev_desc sof_acpi_broadwell_desc = {
	.machines = broadwell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = -1,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-bdw.ri",
	.nocodec_tplg_filename = "intel/reef-bdw-nocodec.tplg"
};
#endif

#if IS_ENABLED(CONFIG_SND_SOC_SOF_BAYTRAIL)

static int is_byt_cr(struct device *dev, const struct sof_dev_desc **desc);

static struct snd_sof_machine baytrail_machines[] = {
	{ "10EC5640", "bytcr-rt5640", "intel/reef-byt.ri",
		"intel/reef-byt-rt5640.tplg", "sst-mfld-platform",
		&snd_sof_byt_ops, mfld_new_mach_data, is_byt_cr},
	{ "10EC5640", "byt-rt5640", "intel/reef-byt.ri",
		"intel/reef-byt-rt5640.tplg", "baytrail-pcm-audio",
		&snd_sof_byt_ops, },
	{ "10EC5651", "bytcr_rt5651", "intel/reef-byt.ri",
		"intel/reef-byt-rt5651.tplg", "sst-mfld-platform",
		&snd_sof_byt_ops, mfld_new_mach_data, is_byt_cr},
	{ "10EC5651", "byt-rt5651", "intel/reef-byt.ri",
		"intel/reef-byt-rt5651.tplg", "baytrail-pcm-audio",
		&snd_sof_byt_ops, },
	{ "193C9890", "byt-max98090", "intel/reef-byt.ri",
		"intel/reef-byt.tplg", "baytrail-pcm-audio",
		&snd_sof_byt_ops },
	{}
};

/* BYTCR uses different IRQ index */
static struct sof_dev_desc sof_acpi_baytrailcr_desc = {
	.machines = baytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-byt.ri",
	.nocodec_tplg_filename = "intel/reef-byt-nocodec.tplg"
};

static struct sof_dev_desc sof_acpi_baytrail_desc = {
	.machines = baytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 5,
	.nocodec_fw_filename = "intel/reef-byt.ri",
	.nocodec_tplg_filename = "intel/reef-byt-nocodec.tplg"
};

static int is_byt_cr(struct device *dev, const struct sof_dev_desc **desc)
{
	u32 bios_status;
	int status;

	if (!IS_ENABLED(CONFIG_IOSF_MBI) || !iosf_mbi_available()) {
		dev_info(dev, "IOSF_MBI not enabled - cant determine CPU variant\n");
		return -EIO;
	}

	status = iosf_mbi_read(BT_MBI_UNIT_PMC, /* 0x04 PUNIT */
			       MBI_REG_READ, /* 0x10 */
			       0x006, /* BIOS_CONFIG */
			       &bios_status);

	if (status) {
		dev_err(dev, "error: could not read PUNIT BIOS_CONFIG\n");
		return -EIO;
	} else {
		/* bits 26:27 mirror PMIC options */
		bios_status = (bios_status >> 26) & 3;

		if ((bios_status == 1) || (bios_status == 3)) {
			dev_info(dev, "BYT-CR detected\n");
			*desc = &sof_acpi_baytrailcr_desc;
			return 1;
		} else {
			dev_info(dev, "BYT-CR not detected\n");
			return 0;
		}
	}
}

static struct snd_sof_machine cherrytrail_machines[] = {

	{"10EC5670", "cht-bsw-rt5672", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	{"10EC5672", "cht-bsw-rt5672", "intel/reef-cht.ri",
		"intel/reef-cht.tplg","sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	{"10EC5645", "cht-bsw-rt5645", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	{"10EC5650", "cht-bsw-rt5645", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	{"10EC5670", "cht-bsw-rt5672", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	{"193C9890", "cht-bsw-max98090", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "sst-mfld-platform", &snd_sof_byt_ops,
		mfld_new_mach_data },
	/* some CHT-T platforms rely on RT5640, use Baytrail machine driver */
	{"10EC5640", "bytcr_rt5640", "intel/reef-cht.ri",
		"intel/reef-cht.tplg", "baytrail-pcm-audio", &snd_sof_byt_ops },
	{},
};

static struct sof_dev_desc sof_acpi_cherrytrail_desc = {
	.machines = cherrytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 5,
	.nocodec_fw_filename = "intel/reef-cht.ri",
	.nocodec_tplg_filename = "intel/reef-cht-nocodec.tplg"
};
#endif

static const struct acpi_device_id sof_acpi_match[] = {
#if IS_ENABLED(CONFIG_SND_SOC_SOF_HASWELL)
	{ "INT33C8", (unsigned long)&sof_acpi_haswell_desc },
#endif
#if IS_ENABLED(CONFIG_SND_SOC_SOF_BROADWELL)
	{ "INT3438", (unsigned long)&sof_acpi_broadwell_desc },
#endif
#if IS_ENABLED(CONFIG_SND_SOC_SOF_BAYTRAIL)
	{ "80860F28", (unsigned long)&sof_acpi_baytrail_desc },
	{ "808622A8", (unsigned long)&sof_acpi_cherrytrail_desc },
#endif
	{ }
};
MODULE_DEVICE_TABLE(acpi, sof_acpi_match);

/* acpi_driver definition */
static struct platform_driver snd_sof_acpi_driver = {
	.probe = sof_acpi_probe,
	.remove = sof_acpi_remove,
	.shutdown = sof_acpi_shutdown,
	.driver = {
		.name = "sof-audio-acpi",
		.pm = &sof_acpi_pm,
		.acpi_match_table = ACPI_PTR(sof_acpi_match),
	},
};
module_platform_driver(snd_sof_acpi_driver);

MODULE_LICENSE("Dual BSD/GPL");
