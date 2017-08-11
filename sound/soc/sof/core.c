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
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/sof.h>
#include "sof-priv.h"
#include "ops.h"

#define TIMEOUT_IPC	5
#define TIMEOUT_BOOT	100

static int sof_probe(struct platform_device *pdev)
{
	struct snd_sof_pdata *plat_data = dev_get_platdata(&pdev->dev);
	struct snd_sof_dev *sdev;
	int ret;

	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (sdev == NULL)
		return -ENOMEM;

	dev_dbg(&pdev->dev, "probing SOF DSP device....\n");

	/* intialise sof device */
	sdev->dev = &pdev->dev;
	if (plat_data->pci) {
		sdev->pci = plat_data->pci;
		sdev->parent = &plat_data->pci->dev;
	} else if (plat_data->pdev) {
		sdev->parent = &plat_data->pdev->dev;
	} else
		sdev->parent = plat_data->dev;
	sdev->ops = plat_data->machine->ops;

	sdev->pdata = plat_data;
	INIT_LIST_HEAD(&sdev->pcm_list);
	INIT_LIST_HEAD(&sdev->kcontrol_list);
	INIT_LIST_HEAD(&sdev->widget_list);
	dev_set_drvdata(&pdev->dev, sdev);

	/* set up platform and component drivers */
	snd_sof_new_platform_drv(sdev);
	snd_sof_new_dai_drv(sdev);

	/* set default timeouts if none provided */
	if (plat_data->desc->ipc_timeout == 0)
		sdev->ipc_timeout = TIMEOUT_IPC;
	else
		sdev->ipc_timeout = plat_data->desc->ipc_timeout;
	if (plat_data->desc->boot_timeout == 0)
		sdev->boot_timeout = TIMEOUT_BOOT;
	else
		sdev->boot_timeout = plat_data->desc->boot_timeout;

	/* probe the DSP hardware */
	ret = snd_sof_probe(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to probe DSP %d\n", ret);
		return ret;
	}

	/* register any debug/trace capabilities */
	ret = snd_sof_dbg_init(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to init DSP trace/debug %d\n", ret);
		return ret;
	}

	/* init the IPC */
	sdev->ipc = snd_sof_ipc_init(sdev);
	if (sdev->ipc == NULL) {
		dev_err(sdev->dev, "error: failed to init DSP IPC %d\n", ret);
		goto err;
	}

	/* load the firmware */
	ret = snd_sof_load_firmware(sdev, plat_data->fw);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to load DSP firmware %d\n", ret);
		goto err;
	}

	/* boot the firmware */
	ret = snd_sof_run_firmware(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to boot DSP firmware %d\n", ret);
		goto err;
	}

	/* now register audio DSP platform driver */
	ret = snd_soc_register_platform(&pdev->dev, &sdev->plat_drv);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to register DSP platform driver %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_component(&pdev->dev,  sdev->cmpnt_drv,
			 &sdev->dai_drv, sdev->num_dai);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to register DSP DAI driver %d\n", ret);
		goto err;
	}

	/* we return 0 on error if debug is defined as this allows DSP
	 * memories to and peripherals to be inspected. */
err:
#if defined DEBUG
	return 0;
#else
	return ret;
#endif
}

static int sof_remove(struct platform_device *pdev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	snd_sof_free_topology(sdev);
	snd_sof_fw_unload(sdev);
	snd_sof_ipc_free(sdev);
	snd_sof_free_debug(sdev);
	snd_sof_remove(sdev);
	return 0;
}


void snd_sof_shutdown(struct device *dev)
{
}
EXPORT_SYMBOL(snd_sof_shutdown);

static struct platform_driver sof_driver = {
	.driver = {
		.name = "sof-audio",
	},

	.probe = sof_probe,
	.remove = sof_remove,
};
module_platform_driver(sof_driver);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("Sound Open Firmware (Reef) Core");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:sof-audio");
