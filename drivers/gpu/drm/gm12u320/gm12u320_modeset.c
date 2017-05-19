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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include "gm12u320_drv.h"

static void gm12u320_crtc_dpms(struct drm_crtc *crtc, int mode)
{
}

static int gm12u320_crtc_mode_set(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode,
				  int x, int y,
				  struct drm_framebuffer *old_fb)

{
	struct gm12u320_framebuffer *fb = to_gm12u320_fb(crtc->primary->fb);

	gm12u320_fb_mark_dirty(fb, 0, GM12U320_USER_WIDTH, 0, GM12U320_HEIGHT);
	return 0;
}


static void gm12u320_crtc_disable(struct drm_crtc *crtc)
{
}

static void gm12u320_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
	kfree(crtc);
}

static int gm12u320_crtc_page_flip(struct drm_crtc *crtc,
				   struct drm_framebuffer *drm_fb,
				   struct drm_pending_vblank_event *event,
				   uint32_t page_flip_flags)
{
	struct gm12u320_framebuffer *fb = to_gm12u320_fb(drm_fb);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	gm12u320_fb_mark_dirty(fb, 0, GM12U320_USER_WIDTH, 0, GM12U320_HEIGHT);

	spin_lock_irqsave(&dev->event_lock, flags);
	if (event)
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	crtc->primary->fb = drm_fb;

	return 0;
}

static void gm12u320_crtc_prepare(struct drm_crtc *crtc)
{
}

static void gm12u320_crtc_commit(struct drm_crtc *crtc)
{
	gm12u320_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static const struct drm_crtc_helper_funcs gm12u320_helper_funcs = {
	.dpms = gm12u320_crtc_dpms,
	.mode_set = gm12u320_crtc_mode_set,
	.prepare = gm12u320_crtc_prepare,
	.commit = gm12u320_crtc_commit,
	.disable = gm12u320_crtc_disable,
};

static const struct drm_crtc_funcs gm12u320_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.destroy = gm12u320_crtc_destroy,
	.page_flip = gm12u320_crtc_page_flip,
};

static int gm12u320_crtc_init(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	crtc = kzalloc(sizeof(struct drm_crtc) +
		       sizeof(struct drm_connector *), GFP_KERNEL);
	if (crtc == NULL)
		return -ENOMEM;

	drm_crtc_init(dev, crtc, &gm12u320_crtc_funcs);
	drm_crtc_helper_add(crtc, &gm12u320_helper_funcs);

	return 0;
}

static const struct drm_mode_config_funcs gm12u320_mode_funcs = {
	.fb_create = gm12u320_fb_user_fb_create,
	.output_poll_changed = NULL,
};

int gm12u320_modeset_init(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = GM12U320_USER_WIDTH;
	dev->mode_config.min_height = GM12U320_HEIGHT;

	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.preferred_depth = 24;

	dev->mode_config.funcs = &gm12u320_mode_funcs;

	gm12u320_crtc_init(dev);

	encoder = gm12u320_encoder_init(dev);

	gm12u320_connector_init(dev, encoder);

	return 0;
}

void gm12u320_modeset_cleanup(struct drm_device *dev)
{
	drm_mode_config_cleanup(dev);
}
