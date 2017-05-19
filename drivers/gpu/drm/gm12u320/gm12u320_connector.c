/*
 * Copyright (C) 2012-2016 Red Hat Inc.
 *
 * Based in parts on the udl code. Based in parts on the gm12u320 fb driver:
 * Copyright (C) 2013 Viacheslav Nurmekhamitov <slavrn@yandex.ru>
 * Copyright (C) 2009 Roberto De Ioris <roberto@unbit.it>
 * Copyright (C) 2009 Jaya Kumar <jayakumar.lkml@gmail.com>
 * Copyright (C) 2009 Bernie Thompson <bernie@plugable.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_crtc_helper.h>
#include "gm12u320_drv.h"

/*
 * Note this assumes this driver is only ever used with the Acer C120, if we
 * add support for other devices the vendor and model should be parameterized.
 */
static struct edid gm12u320_edid = {
	.header		= { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 },
	.mfg_id		= { 0x04, 0x72 },	/* "ACR" */
	.prod_code	= { 0x20, 0xc1 },	/* C120h */
	.mfg_week	= 1,
	.mfg_year	= 1,
	.version	= 1,			/* EDID 1.3 */
	.revision	= 3,			/* EDID 1.3 */
	.input		= 0x80,			/* Digital input */
	.features	= 0x02,			/* Pref timing in DTD 1 */
	.standard_timings = { { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },
			      { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 } },
	.detailed_timings = { {
		.pixel_clock = 3383,
		/* hactive = 852, hblank = 256 */
		.data.pixel_data.hactive_lo = 0x54,
		.data.pixel_data.hblank_lo = 0x00,
		.data.pixel_data.hactive_hblank_hi = 0x31,
		/* vactive = 480, vblank = 28 */
		.data.pixel_data.vactive_lo = 0xe0,
		.data.pixel_data.vblank_lo = 0x1c,
		.data.pixel_data.vactive_vblank_hi = 0x10,
		/* hsync offset 40 pw 128, vsync offset 1 pw 4 */
		.data.pixel_data.hsync_offset_lo = 0x28,
		.data.pixel_data.hsync_pulse_width_lo = 0x80,
		.data.pixel_data.vsync_offset_pulse_width_lo = 0x14,
		.data.pixel_data.hsync_vsync_offset_pulse_width_hi = 0x00,
		/* Digital separate syncs, hsync+, vsync+ */
		.data.pixel_data.misc = 0x1e,
	}, {
		.pixel_clock = 0,
		.data.other_data.type = 0xfd, /* Monitor ranges */
		.data.other_data.data.range.min_vfreq = 59,
		.data.other_data.data.range.max_vfreq = 61,
		.data.other_data.data.range.min_hfreq_khz = 29,
		.data.other_data.data.range.max_hfreq_khz = 32,
		.data.other_data.data.range.pixel_clock_mhz = 4, /* 40 MHz */
		.data.other_data.data.range.flags = 0,
		.data.other_data.data.range.formula.cvt = {
			0xa0, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 },
	}, {
		.pixel_clock = 0,
		.data.other_data.type = 0xfc, /* Model string */
		.data.other_data.data.str.str = {
			'C', '1', '2', '0', 'P', 'r', 'o', 'j', 'e', 'c',
			't', 'o', 'r' },
	}, {
		.pixel_clock = 0,
		.data.other_data.type = 0xfe, /* Unspecified text / padding */
		.data.other_data.data.str.str = {
			'\n', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
			' ', ' ',  ' ' },
	} },
	.checksum = 0x40,
};

static int gm12u320_get_modes(struct drm_connector *connector)
{
	drm_mode_connector_update_edid_property(connector, &gm12u320_edid);
	return drm_add_edid_modes(connector, &gm12u320_edid);
}

static enum drm_connector_status
gm12u320_detect(struct drm_connector *connector, bool force)
{
	if (drm_device_is_unplugged(connector->dev))
		return connector_status_disconnected;

	return connector_status_connected;
}

static struct drm_encoder*
gm12u320_best_single_encoder(struct drm_connector *connector)
{
	int enc_id = connector->encoder_ids[0];

	return drm_encoder_find(connector->dev, enc_id);
}

static int gm12u320_connector_set_property(struct drm_connector *connector,
					   struct drm_property *property,
					   uint64_t val)
{
	return 0;
}

static void gm12u320_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static const struct drm_connector_helper_funcs gm12u320_helper_funcs = {
	.get_modes = gm12u320_get_modes,
	.best_encoder = gm12u320_best_single_encoder,
};

static const struct drm_connector_funcs gm12u320_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = gm12u320_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = gm12u320_connector_destroy,
	.set_property = gm12u320_connector_set_property,
};

int gm12u320_connector_init(struct drm_device *dev,
			    struct drm_encoder *encoder)
{
	struct drm_connector *connector;

	connector = kzalloc(sizeof(struct drm_connector), GFP_KERNEL);
	if (!connector)
		return -ENOMEM;

	drm_connector_init(dev, connector, &gm12u320_connector_funcs,
			   DRM_MODE_CONNECTOR_Unknown);
	drm_connector_helper_add(connector, &gm12u320_helper_funcs);

	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}
