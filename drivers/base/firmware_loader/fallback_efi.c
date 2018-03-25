// SPDX-License-Identifier: GPL-2.0

#include <linux/efi_embedded_fw.h>
#include <linux/property.h>
#include <linux/security.h>
#include <linux/vmalloc.h>

#include "fallback.h"
#include "firmware.h"

int fw_get_efi_embedded_fw(struct device *dev, struct fw_priv *fw_priv,
			   enum fw_opt *opt_flags, int ret)
{
	enum kernel_read_file_id id = READING_FIRMWARE;
	size_t size, max = INT_MAX;
	int rc;

	if (!dev)
		return ret;

	if (!device_property_read_bool(dev, "efi-embedded-firmware"))
		return ret;

	*opt_flags |= FW_OPT_NO_WARN | FW_OPT_NOCACHE | FW_OPT_NOFALLBACK;

	/* Already populated data member means we're loading into a buffer */
	if (fw_priv->data) {
		id = READING_FIRMWARE_PREALLOC_BUFFER;
		max = fw_priv->allocated_size;
	}

	rc = efi_get_embedded_fw(fw_priv->fw_name, &fw_priv->data, &size, max);
	if (rc) {
		dev_warn(dev, "Firmware %s not in EFI\n", fw_priv->fw_name);
		return ret;
	}

	rc = security_kernel_post_read_file(NULL, fw_priv->data, size, id);
	if (rc) {
		if (id != READING_FIRMWARE_PREALLOC_BUFFER) {
			vfree(fw_priv->data);
			fw_priv->data = NULL;
		}
		return rc;
	}

	dev_dbg(dev, "using efi-embedded fw %s\n", fw_priv->fw_name);
	fw_priv->size = size;
	fw_state_done(fw_priv);
	return 0;
}
