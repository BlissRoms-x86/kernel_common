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
#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include <sound/sof.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <asm/cpu_device_id.h>
#include <asm/iosf_mbi.h>
#include "sof-priv.h"

#if IS_ENABLED(CONFIG_SND_SOC_SOF_HASWELL)
static struct sof_dev_desc sof_acpi_haswell_desc = {
	.machines = snd_soc_acpi_intel_haswell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = -1,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-hsw.ri",
	.nocodec_tplg_filename = "intel/reef-hsw-nocodec.tplg"
};
#endif

#if IS_ENABLED(CONFIG_SND_SOC_SOF_BROADWELL)
static struct sof_dev_desc sof_acpi_broadwell_desc = {
	.machines = snd_soc_acpi_intel_broadwell_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = -1,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-bdw.ri",
	.nocodec_tplg_filename = "intel/reef-bdw-nocodec.tplg"
};
#endif

#if IS_ENABLED(CONFIG_SND_SOC_SOF_BAYTRAIL)

/* BYTCR uses different IRQ index */
static struct sof_dev_desc sof_acpi_baytrailcr_desc = {
	.machines = snd_soc_acpi_intel_baytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 0,
	.nocodec_fw_filename = "intel/reef-byt.ri",
	.nocodec_tplg_filename = "intel/reef-byt-nocodec.tplg"
};

static struct sof_dev_desc sof_acpi_baytrail_desc = {
	.machines = snd_soc_acpi_intel_baytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 5,
	.nocodec_fw_filename = "intel/reef-byt.ri",
	.nocodec_tplg_filename = "intel/reef-byt-nocodec.tplg"
};

static int is_byt_cr(struct device *dev)
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
			return 1;
		} else {
			dev_info(dev, "BYT-CR not detected\n");
			return 0;
		}
	}
}

static struct sof_dev_desc sof_acpi_cherrytrail_desc = {
	.machines = snd_soc_acpi_intel_cherrytrail_machines,
	.resindex_lpe_base = 0,
	.resindex_pcicfg_base = 1,
	.resindex_imr_base = 2,
	.irqindex_host_ipc = 5,
	.nocodec_fw_filename = "intel/reef-cht.ri",
	.nocodec_tplg_filename = "intel/reef-cht-nocodec.tplg"
};
#endif

static struct platform_device * 
	mfld_new_mach_data(struct snd_sof_pdata *sof_pdata)
{
	struct snd_soc_acpi_mach pmach;
	struct device *dev = &sof_pdata->pdev->dev;
	const struct snd_soc_acpi_mach *mach = sof_pdata->machine;
	struct platform_device *pdev = NULL;

	memset(&pmach, 0, sizeof(pmach));
	memcpy((void*)pmach.id, mach->id, ACPI_ID_LEN);
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

static void sof_acpi_fw_cb(const struct firmware *fw, void *context)
{
	struct sof_acpi_priv *priv = context;
	struct snd_sof_pdata *sof_pdata = priv->sof_pdata;
	const struct snd_soc_acpi_mach *mach = sof_pdata->machine;
	struct device *dev = &sof_pdata->pdev->dev;

	sof_pdata->fw = fw;
	if (!fw) {
		dev_err(dev, "Cannot load firmware %s\n", mach->sof_fw_filename);
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
	struct snd_soc_acpi_mach *mach;
	struct snd_sof_pdata *sof_pdata;
	struct sof_acpi_priv *priv;
	struct snd_sof_dsp_ops *ops;
	void *new_mach_data;
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

	/* FIXME: is there a better way to write this */
	if (0) ;
#if IS_ENABLED(CONFIG_SND_SOC_SOF_HASWELL)
	else if (desc == &sof_acpi_haswell_desc) {
		ops = &snd_sof_hsw_ops;
	}
#endif
#if IS_ENABLED(CONFIG_SND_SOC_SOF_BROADWELL)
	else if (desc == &sof_acpi_broadwell_desc) {
		ops = &snd_sof_bdw_ops;
	}
#endif
#if IS_ENABLED(CONFIG_SND_SOC_SOF_BAYTRAIL)
	else if (desc == &sof_acpi_baytrail_desc) {
		if (is_byt_cr(dev))
			desc = &sof_acpi_baytrailcr_desc;
		ops = &snd_sof_byt_ops;
		new_mach_data = mfld_new_mach_data;
	}
	else if (desc == &sof_acpi_cherrytrail_desc) {
		ops = &snd_sof_cht_ops;
		new_mach_data = mfld_new_mach_data;
	}
#endif
	else return -ENODEV;

	/* find machine */
	mach = snd_soc_acpi_find_machine(desc->machines);
	if (mach == NULL) {
		struct snd_soc_acpi_mach *m;
		/* dont bind to any particular codec, just initialse the DSP */
		dev_err(dev, "No matching ASoC machine driver found - using nocodec\n");
		sof_pdata->drv_name = "sof-nocodec";
		m = devm_kzalloc(dev, sizeof(*mach), GFP_KERNEL);
		if (m == NULL)
			return -ENOMEM;

		m->drv_name = "sof-nocodec";
		m->sof_fw_filename = desc->nocodec_fw_filename;
		m->sof_tplg_filename = desc->nocodec_tplg_filename;
		m->asoc_plat_name = "sof-platform";/// used ???
		mach = m;
	}

	mach->pdata = ops;
	mach->new_mach_data =  (struct platform_device *
				(*)(void *pdata)) new_mach_data;

	//sof_pdata->id = acpi_id->device;
	//sof_pdata->name = acpi_name(pci);
	sof_pdata->machine = mach;
	// FIXME, this can't work for baytrail cr: sof_pdata->desc = (struct sof_dev_desc*) id->driver_data;
	sof_pdata->desc = desc;
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
	ret = request_firmware_nowait(THIS_MODULE, true, mach->sof_fw_filename,
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
