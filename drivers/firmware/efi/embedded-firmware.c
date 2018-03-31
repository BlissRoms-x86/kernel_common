// SPDX-License-Identifier: GPL-2.0
/*
 * Support for extracting embedded firmware for peripherals from EFI code,
 *
 * Copyright (c) 2018 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/crc32.h>
#include <linux/dmi.h>
#include <linux/efi.h>
#include <linux/types.h>

/* Sofar there are no machines with more then 1 interesting embedded firmware */
#define MAX_EMBEDDED_FIRMWARES	1

struct embedded_fw_desc {
	const char *name;
	u8 prefix[8];
	u32 length;
	u32 crc;
};

struct embedded_fw {
	const char *name;
	void *data;
	size_t length;
};

static struct embedded_fw found_fw[MAX_EMBEDDED_FIRMWARES];
static int found_fw_count;

static struct embedded_fw_desc cube_iwork8_air_fw[] __initdata = {
	{
		.name	= "silead/gsl3670-cube-iwork8-air.fw",
		.prefix = { 0xf0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00 },
		.length	= 38808,
		.crc	= 0xfecde51f,
	},
	{}
};

static struct embedded_fw_desc pipo_w2s_fw[] __initdata = {
	{
		.name	= "silead/gsl1680-pipo-w2s.fw",
		.prefix = { 0xf0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00 },
		.length	= 39072,
		.crc	= 0x28d5dc6c,
	},
	{}
};

static struct dmi_system_id embedded_fw_table[] __initdata = {
	{
		.driver_data = (void *)cube_iwork8_air_fw,
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "cube"),
			DMI_MATCH(DMI_PRODUCT_NAME, "i1-TF"),
			DMI_MATCH(DMI_BOARD_NAME, "Cherry Trail CR"),
		},
	},
	{
		.driver_data = (void *)pipo_w2s_fw,
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "PIPO"),
			DMI_MATCH(DMI_PRODUCT_NAME, "W2S"),
		},
	},
	{}
};

/*
 * Note the efi_check_for_embedded_firmwares() code currently makes the
 * following 2 assumptions. This may needs to be revisited if embedded firmware
 * is found where this is not true:
 * 1) The firmware is only found in EFI_BOOT_SERVICES_CODE memory segments
 * 2) The firmware always starts at an offset which is a multiple of 16 bytes
 */
static int __init efi_check_md_for_embedded_firmware(
	efi_memory_desc_t *md, const struct embedded_fw_desc *desc)
{
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
	for (i = 0; i < size; i += 16) {
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

	if (found_fw_count >= MAX_EMBEDDED_FIRMWARES) {
		pr_err("Error already have %d embedded firmwares\n",
		       MAX_EMBEDDED_FIRMWARES);
		return -ENOSPC;
	}

	found_fw[found_fw_count].data =
		memremap(md->phys_addr + i, desc->length, MEMREMAP_WB);
	if (!found_fw[found_fw_count].data) {
		pr_err("Error mapping embedded firmware\n");
		return -ENOMEM;
	}

	found_fw[found_fw_count].name = desc->name;
	found_fw[found_fw_count].length = desc->length;
	found_fw_count++;

	/* Note md points to *unmapped* memory after this!!! */
	efi_mem_reserve(md->phys_addr + i, desc->length);
	return 0;
}

void __init efi_check_for_embedded_firmwares(void)
{
	const struct embedded_fw_desc *fw_desc;
	const struct dmi_system_id *dmi_id;
	efi_memory_desc_t *md;
	int i, r;

	dmi_id = dmi_first_match(embedded_fw_table);
	if (!dmi_id)
		return;

	fw_desc = dmi_id->driver_data;
	for (i = 0; fw_desc[i].length; i++) {
		for_each_efi_memory_desc(md) {
			if (md->type != EFI_BOOT_SERVICES_CODE)
				continue;

			r = efi_check_md_for_embedded_firmware(md, &fw_desc[i]);
			if (r == 0) {
				/*
				 * On success efi_mem_reserve() has been called
				 * installing a new memmap, so our pointers
				 * are invalid now and we MUST break the loop.
				 */
				break;
			}
		}
	}
}

int efi_get_embedded_fw(const char *name, void **data, size_t *size,
			size_t msize)
{
	struct embedded_fw *fw = NULL;
	void *buf = *data;
	int i;

	for (i = 0; i < found_fw_count; i++) {
		if (strcmp(name, found_fw[i].name) == 0) {
			fw = &found_fw[i];
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
