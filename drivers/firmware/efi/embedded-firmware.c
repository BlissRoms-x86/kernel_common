// SPDX-License-Identifier: GPL-2.0
/*
 * Support for extracting embedded firmware for peripherals from EFI code,
 *
 * Copyright (c) 2018 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/crc32.h>
#include <linux/dmi.h>
#include <linux/efi.h>
#include <linux/efi_embedded_fw.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

struct embedded_fw {
	struct list_head list;
	const char *name;
	void *data;
	size_t length;
};

static LIST_HEAD(found_fw_list);

static const struct dmi_system_id * const embedded_fw_table[] = {
	NULL
};

/*
 * Note the efi_check_for_embedded_firmwares() code currently makes the
 * following 2 assumptions. This may needs to be revisited if embedded firmware
 * is found where this is not true:
 * 1) The firmware is only found in EFI_BOOT_SERVICES_CODE memory segments
 * 2) The firmware always starts at an offset which is a multiple of 8 bytes
 */
static int __init efi_check_md_for_embedded_firmware(
	efi_memory_desc_t *md, const struct efi_embedded_fw_desc *desc)
{
	struct embedded_fw *fw;
	u64 i, size;
	u32 crc;
	u8 *mem;

	size = md->num_pages << EFI_PAGE_SHIFT;
	mem = memremap(md->phys_addr, size, MEMREMAP_WB);
	if (!mem) {
		pr_err("Error mapping EFI mem at %#llx\n", md->phys_addr);
		return -ENOMEM;
	}

	size -= desc->length;
	for (i = 0; i < size; i += 8) {
		if (*((u64 *)(mem + i)) != *((u64 *)desc->prefix))
			continue;

		/* Seed with ~0, invert to match crc32 userspace utility */
		crc = ~crc32(~0, mem + i, desc->length);
		if (crc == desc->crc)
			break;
	}

	memunmap(mem);

	if (i >= size)
		return -ENOENT;

	pr_info("Found EFI embedded fw '%s' crc %08x\n", desc->name, desc->crc);

	fw = kmalloc(sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	mem = memremap(md->phys_addr + i, desc->length, MEMREMAP_WB);
	if (!mem) {
		pr_err("Error mapping embedded firmware\n");
		goto error_free_fw;
	}
	fw->data = kmemdup(mem, desc->length, GFP_KERNEL);
	memunmap(mem);
	if (!fw->data)
		goto error_free_fw;

	fw->name = desc->name;
	fw->length = desc->length;
	list_add(&fw->list, &found_fw_list);

	return 0;

error_free_fw:
	kfree(fw);
	return -ENOMEM;
}

void __init efi_check_for_embedded_firmwares(void)
{
	const struct efi_embedded_fw_desc *fw_desc;
	const struct dmi_system_id *dmi_id;
	efi_memory_desc_t *md;
	int i, r;

	for (i = 0; embedded_fw_table[i]; i++) {
		dmi_id = dmi_first_match(embedded_fw_table[i]);
		if (!dmi_id)
			continue;

		fw_desc = dmi_id->driver_data;
		for_each_efi_memory_desc(md) {
			if (md->type != EFI_BOOT_SERVICES_CODE)
				continue;

			r = efi_check_md_for_embedded_firmware(md, fw_desc);
			if (r == 0)
				break;
		}
	}
}

int efi_get_embedded_fw(const char *name, void **data, size_t *size,
			size_t msize)
{
	struct embedded_fw *iter, *fw = NULL;
	void *buf = *data;

	list_for_each_entry(iter, &found_fw_list, list) {
		if (strcmp(name, iter->name) == 0) {
			fw = iter;
			break;
		}
	}

	if (!fw)
		return -ENOENT;

	if (msize && msize < fw->length)
		return -EFBIG;

	if (!buf) {
		buf = vmalloc(fw->length);
		if (!buf)
			return -ENOMEM;
	}

	memcpy(buf, fw->data, fw->length);
	*size = fw->length;
	*data = buf;

	return 0;
}
EXPORT_SYMBOL_GPL(efi_get_embedded_fw);
