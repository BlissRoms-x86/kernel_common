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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <uapi/sound/sof-ipc.h>
#include "sof-priv.h"
#include "ops.h"


static int sof_dfsentry_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t sof_dfsentry_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	struct snd_sof_dfsentry *dfse = file->private_data;
	struct snd_sof_dev *sdev = dfse->sdev;
	int size;
	u32 *buf;
	loff_t pos = *ppos;
	size_t ret;

	size = dfse->size;

	if (pos < 0)
		return -EINVAL;
	if (pos >= size || !count)
		return 0;
	if (count > size - pos)
		count = size - pos;

	size = (count + 3) & ~3;
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pm_runtime_get(sdev->dev);
	memcpy_fromio(buf,  dfse->buf + pos, size);
	pm_runtime_put(sdev->dev);

	ret = copy_to_user(buffer, buf, count);
	kfree(buf);

	if (ret == count)
		return -EFAULT;
	count -= ret;
	*ppos = pos + count;

	return count;
}

static const struct file_operations sof_dfs_fops = {
	.open = sof_dfsentry_open,
	.read = sof_dfsentry_read,
	.llseek = default_llseek,
};

static int sof_debugfs_create_item(struct snd_sof_dev *sdev,
	void __iomem *base, size_t size, const char *name)
{
	struct snd_sof_dfsentry *dfse;

	if (!sdev)
		return -EINVAL;

	dfse = kzalloc(sizeof(*dfse), GFP_KERNEL);
	if (!dfse)
		return -ENOMEM;

	dfse->buf = base;
	dfse->size = size;
	dfse->sdev = sdev;

	dfse->dfsentry = debugfs_create_file(name, 0444, sdev->debugfs_root,
					     dfse, &sof_dfs_fops);
	if (!dfse->dfsentry) {
		dev_err(sdev->dev, "cannot create debugfs entry.\n");
		kfree(dfse);
		return -ENODEV;
	}

	return 0;
}


int snd_sof_dbg_init(struct snd_sof_dev *sdev)
{
	const struct snd_sof_dsp_ops *ops = sdev->ops;
	const struct snd_sof_debugfs_map *map;
	int err = 0, i;

	sdev->debugfs_root = debugfs_create_dir("sof", NULL);
	if (IS_ERR(sdev->debugfs_root) || !sdev->debugfs_root) {
		dev_err(sdev->dev, "error: failed to create debugfs directory\n");
		return -EINVAL;
	}

	for (i = 0; i < ops->debug_map_count; i++) {

		map = &ops->debug_map[i];

		err = sof_debugfs_create_item(sdev,
			sdev->bar[map->bar] + map->offset, map->size, map->name);
		if (err < 0)
			dev_err(sdev->dev, "cannot create debugfs for %s\n",
				map->name);
	}

	return err;
}
EXPORT_SYMBOL(snd_sof_dbg_init);

void snd_sof_free_debug(struct snd_sof_dev *sdev)
{

}
EXPORT_SYMBOL(snd_sof_free_debug);
