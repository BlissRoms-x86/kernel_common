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

#ifndef GM12U320_DRV_H
#define GM12U320_DRV_H

#include <linux/usb.h>
#include <linux/spinlock.h>
#include <drm/drm_gem.h>

#define DRIVER_NAME		"gm12u320"
#define DRIVER_DESC		"Grain Media GM12U320 USB projector display"
#define DRIVER_DATE		"20150107"

#define DRIVER_MAJOR		0
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	1

#define GM12U320_BO_CACHEABLE	(1 << 0)
#define GM12U320_BO_WC		(1 << 1)

/*
 * The DLP has an actual width of 854 pixels, but that is not a multiple
 * of 4, breaking things left and right, so we export a width of 852.
 */
#define GM12U320_USER_WIDTH	852
#define GM12U320_REAL_WIDTH	854
#define GM12U320_HEIGHT		480

#define GM12U320_BLOCK_COUNT	20

struct gm12u320_device;

struct gm12u320_fbdev;

struct gm12u320_device {
	struct device *dev;
	struct usb_device *udev;
	struct drm_device *ddev;
	struct gm12u320_fbdev *fbdev;
	unsigned char *cmd_buf;
	unsigned char *data_buf[GM12U320_BLOCK_COUNT];
	struct {
		bool run;
		struct workqueue_struct *workq;
		struct work_struct work;
		wait_queue_head_t waitq;
		struct mutex lock;
		struct gm12u320_framebuffer *fb;
		int x1;
		int x2;
		int y1;
		int y2;
	} fb_update;
};

struct gm12u320_gem_object {
	struct drm_gem_object base;
	struct page **pages;
	void *vmapping;
	struct sg_table *sg;
	unsigned int flags;
};

#define to_gm12u320_bo(x) container_of(x, struct gm12u320_gem_object, base)

struct gm12u320_framebuffer {
	struct drm_framebuffer base;
	struct gm12u320_gem_object *obj;
};

#define to_gm12u320_fb(x) container_of(x, struct gm12u320_framebuffer, base)

/* modeset */
int gm12u320_modeset_init(struct drm_device *dev);
void gm12u320_modeset_cleanup(struct drm_device *dev);
int gm12u320_connector_init(struct drm_device *dev,
			    struct drm_encoder *encoder);

struct drm_encoder *gm12u320_encoder_init(struct drm_device *dev);

int gm12u320_driver_load(struct drm_device *dev, unsigned long flags);
void gm12u320_driver_unload(struct drm_device *dev);

int gm12u320_fbdev_init(struct drm_device *dev);
void gm12u320_fbdev_cleanup(struct drm_device *dev);
void gm12u320_fbdev_unplug(struct drm_device *dev);
struct drm_framebuffer *
gm12u320_fb_user_fb_create(struct drm_device *dev, struct drm_file *file,
			   const struct drm_mode_fb_cmd2 *mode_cmd);

int gm12u320_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
			 struct drm_mode_create_dumb *args);
int gm12u320_gem_mmap(struct drm_file *file_priv, struct drm_device *dev,
		      uint32_t handle, uint64_t *offset);

struct gm12u320_gem_object *
gm12u320_gem_alloc_object(struct drm_device *dev, size_t size);
void gm12u320_gem_free_object(struct drm_gem_object *gem_obj);
struct dma_buf *gm12u320_gem_prime_export(struct drm_device *dev,
				      struct drm_gem_object *obj, int flags);
struct drm_gem_object *gm12u320_gem_prime_import(struct drm_device *dev,
						 struct dma_buf *dma_buf);

int gm12u320_gem_get_pages(struct gm12u320_gem_object *obj);
void gm12u320_gem_put_pages(struct gm12u320_gem_object *obj);
int gm12u320_gem_vmap(struct gm12u320_gem_object *obj);
void gm12u320_gem_vunmap(struct gm12u320_gem_object *obj);
int gm12u320_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);
int gm12u320_gem_fault(struct vm_fault *vmf);

void gm12u320_fb_mark_dirty(struct gm12u320_framebuffer *fb,
			    int x1, int x2, int y1, int y2);
void gm12u320_start_fb_update(struct drm_device *dev);
void gm12u320_stop_fb_update(struct drm_device *dev);
int gm12u320_set_ecomode(struct drm_device *dev);

#endif
