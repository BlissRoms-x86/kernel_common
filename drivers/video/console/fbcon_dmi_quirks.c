/*
 *  fbcon_dmi_quirks.c -- DMI based quirk detection for fbcon
 *
 *	Copyright (C) 2017 Hans de Goede <hdegoede@redhat.com>
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/dmi.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include "fbcon.h"

/*
 * Some x86 clamshell design devices use portrait tablet screens and a display
 * engine which cannot rotate in hardware, so we need to rotate the fbcon to
 * compensate. Unfortunately these (cheap) devices also typically have quite
 * generic DMI data, so we match on a combination of DMI data, screen resolution
 * and a list of known BIOS dates to avoid false positives.
 */

struct fbcon_dmi_rotate_data {
	struct dmi_system_id dmi_id;
	int width;
	int height;
	const char * const *bios_dates;
	int rotate;
};

static const struct fbcon_dmi_rotate_data rotate_data[] = {
	{	/*
		 * GPD Win, note that the the DMI data is less generic then it
		 * seems, devices with a board_vendor of "AMI Corporation" are
		 * quite rare, as are devices which have both board- *and*
		 * product-id set to "Default String"
		 */
		.dmi_id = { .matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AMI Corporation"),
			DMI_MATCH(DMI_BOARD_NAME, "Default string"),
			DMI_MATCH(DMI_BOARD_SERIAL, "Default string"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Default string"),
		} },
		.width = 720,
		.height = 1280,
		.bios_dates = (const char * const []){
			"10/25/2016", "11/18/2016", "02/21/2017",
			"03/20/2017", NULL },
		.rotate = FB_ROTATE_CW
	}, {	/* GPD Pocket (same note on DMI match as GPD Win) */
		.dmi_id = { .matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AMI Corporation"),
			DMI_MATCH(DMI_BOARD_NAME, "Default string"),
			DMI_MATCH(DMI_BOARD_SERIAL, "Default string"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Default string"),
		} },
		.width = 1200,
		.height = 1920,
		.bios_dates = (const char * const []){ "05/26/2017", NULL },
		.rotate = FB_ROTATE_CW,
	}, {	/* I.T.Works TW891 */
		.dmi_id = { .matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "To be filled by O.E.M."),
			DMI_MATCH(DMI_PRODUCT_NAME, "TW891"),
			DMI_MATCH(DMI_BOARD_VENDOR, "To be filled by O.E.M."),
			DMI_MATCH(DMI_BOARD_NAME, "TW891"),
		} },
		.width = 800,
		.height = 1280,
		.bios_dates = (const char * const []){ "10/16/2015", NULL },
		.rotate = FB_ROTATE_CW,
	}
};

int fbcon_platform_get_rotate(struct fb_info *info)
{
	const char *bios_date;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(rotate_data); i++) {
		if (!dmi_matches(&rotate_data[i].dmi_id))
			continue;

		if (rotate_data[i].width != info->var.xres ||
		    rotate_data[i].height != info->var.yres)
			continue;

		if (!rotate_data[i].bios_dates)
			return rotate_data->rotate;

		bios_date = dmi_get_system_info(DMI_BIOS_DATE);
		if (!bios_date)
			continue;

		for (j = 0; rotate_data[i].bios_dates[j]; j++) {
			if (!strcmp(rotate_data[i].bios_dates[j], bios_date))
				return rotate_data->rotate;
		}
	}

	return FB_ROTATE_UR;
}
