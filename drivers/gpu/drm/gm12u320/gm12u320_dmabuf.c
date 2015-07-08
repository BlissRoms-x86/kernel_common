/*
 * Based on the udl dmabuf code:
 *
 * Copyright (c) 2014 The Chromium OS Authors
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drmP.h>
#include "gm12u320_drv.h"
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>

struct gm12u320_drm_dmabuf_attachment {
	struct sg_table sgt;
	enum dma_data_direction dir;
	bool is_mapped;
};

static int gm12u320_attach_dma_buf(struct dma_buf *dmabuf,
				   struct device *dev,
				   struct dma_buf_attachment *attach)
{
	struct gm12u320_drm_dmabuf_attachment *gm12u320_attach;

	DRM_DEBUG_PRIME("[DEV:%s] size:%zd\n", dev_name(attach->dev),
			attach->dmabuf->size);

	gm12u320_attach = kzalloc(sizeof(*gm12u320_attach), GFP_KERNEL);
	if (!gm12u320_attach)
		return -ENOMEM;

	gm12u320_attach->dir = DMA_NONE;
	attach->priv = gm12u320_attach;

	return 0;
}

static void gm12u320_detach_dma_buf(struct dma_buf *dmabuf,
				    struct dma_buf_attachment *attach)
{
	struct gm12u320_drm_dmabuf_attachment *gm12u320_attach = attach->priv;
	struct sg_table *sgt;

	if (!gm12u320_attach)
		return;

	DRM_DEBUG_PRIME("[DEV:%s] size:%zd\n", dev_name(attach->dev),
			attach->dmabuf->size);

	sgt = &gm12u320_attach->sgt;

	if (gm12u320_attach->dir != DMA_NONE)
		dma_unmap_sg(attach->dev, sgt->sgl, sgt->nents,
				gm12u320_attach->dir);

	sg_free_table(sgt);
	kfree(gm12u320_attach);
	attach->priv = NULL;
}

static struct sg_table *gm12u320_map_dma_buf(struct dma_buf_attachment *attach,
					     enum dma_data_direction dir)
{
	struct gm12u320_drm_dmabuf_attachment *gm12u320_attach = attach->priv;
	struct gm12u320_gem_object *obj = to_gm12u320_bo(attach->dmabuf->priv);
	struct drm_device *dev = obj->base.dev;
	struct scatterlist *rd, *wr;
	struct sg_table *sgt = NULL;
	unsigned int i;
	int page_count;
	int nents, ret;

	DRM_DEBUG_PRIME("[DEV:%s] size:%zd dir=%d\n", dev_name(attach->dev),
			attach->dmabuf->size, dir);

	/* just return current sgt if already requested. */
	if (gm12u320_attach->dir == dir && gm12u320_attach->is_mapped)
		return &gm12u320_attach->sgt;

	if (!obj->pages) {
		ret = gm12u320_gem_get_pages(obj);
		if (ret) {
			DRM_ERROR("failed to map pages.\n");
			return ERR_PTR(ret);
		}
	}

	page_count = obj->base.size / PAGE_SIZE;
	obj->sg = drm_prime_pages_to_sg(obj->pages, page_count);
	if (IS_ERR(obj->sg)) {
		DRM_ERROR("failed to allocate sgt.\n");
		return ERR_CAST(obj->sg);
	}

	sgt = &gm12u320_attach->sgt;

	ret = sg_alloc_table(sgt, obj->sg->orig_nents, GFP_KERNEL);
	if (ret) {
		DRM_ERROR("failed to alloc sgt.\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&dev->struct_mutex);

	rd = obj->sg->sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	if (dir != DMA_NONE) {
		nents = dma_map_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
		if (!nents) {
			DRM_ERROR("failed to map sgl with iommu.\n");
			sg_free_table(sgt);
			sgt = ERR_PTR(-EIO);
			goto err_unlock;
		}
	}

	gm12u320_attach->is_mapped = true;
	gm12u320_attach->dir = dir;
	attach->priv = gm12u320_attach;

err_unlock:
	mutex_unlock(&dev->struct_mutex);
	return sgt;
}

static void gm12u320_unmap_dma_buf(struct dma_buf_attachment *attach,
				   struct sg_table *sgt,
				   enum dma_data_direction dir)
{
	/* Nothing to do. */
	DRM_DEBUG_PRIME("[DEV:%s] size:%zd dir:%d\n", dev_name(attach->dev),
			attach->dmabuf->size, dir);
}

static void *gm12u320_dmabuf_kmap(struct dma_buf *dma_buf,
				  unsigned long page_num)
{
	/* TODO */

	return NULL;
}

static void *gm12u320_dmabuf_kmap_atomic(struct dma_buf *dma_buf,
					 unsigned long page_num)
{
	/* TODO */

	return NULL;
}

static void gm12u320_dmabuf_kunmap(struct dma_buf *dma_buf,
				   unsigned long page_num, void *addr)
{
	/* TODO */
}

static void gm12u320_dmabuf_kunmap_atomic(struct dma_buf *dma_buf,
					  unsigned long page_num,
					  void *addr)
{
	/* TODO */
}

static int gm12u320_dmabuf_mmap(struct dma_buf *dma_buf,
				struct vm_area_struct *vma)
{
	/* TODO */

	return -EINVAL;
}

static struct dma_buf_ops gm12u320_dmabuf_ops = {
	.attach			= gm12u320_attach_dma_buf,
	.detach			= gm12u320_detach_dma_buf,
	.map_dma_buf		= gm12u320_map_dma_buf,
	.unmap_dma_buf		= gm12u320_unmap_dma_buf,
	.kmap			= gm12u320_dmabuf_kmap,
	.kmap_atomic		= gm12u320_dmabuf_kmap_atomic,
	.kunmap			= gm12u320_dmabuf_kunmap,
	.kunmap_atomic		= gm12u320_dmabuf_kunmap_atomic,
	.mmap			= gm12u320_dmabuf_mmap,
	.release		= drm_gem_dmabuf_release,
};

struct dma_buf *gm12u320_gem_prime_export(struct drm_device *dev,
				     struct drm_gem_object *obj, int flags)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &gm12u320_dmabuf_ops;
	exp_info.size = obj->size;
	exp_info.flags = flags;
	exp_info.priv = obj;

	return drm_gem_dmabuf_export(dev, &exp_info);
}

static int gm12u320_prime_create(struct drm_device *dev,
				 size_t size, struct sg_table *sg,
				 struct gm12u320_gem_object **obj_p)
{
	struct gm12u320_gem_object *obj;
	int npages;

	npages = size / PAGE_SIZE;

	*obj_p = NULL;
	obj = gm12u320_gem_alloc_object(dev, npages * PAGE_SIZE);
	if (!obj)
		return -ENOMEM;

	obj->sg = sg;
	obj->pages = drm_malloc_ab(npages, sizeof(struct page *));
	if (obj->pages == NULL) {
		DRM_ERROR("obj pages is NULL %d\n", npages);
		return -ENOMEM;
	}

	drm_prime_sg_to_page_addr_arrays(sg, obj->pages, NULL, npages);

	*obj_p = obj;
	return 0;
}

struct drm_gem_object *gm12u320_gem_prime_import(struct drm_device *dev,
						 struct dma_buf *dma_buf)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sg;
	struct gm12u320_gem_object *uobj;
	int ret;

	/* need to attach */
	get_device(dev->dev);
	attach = dma_buf_attach(dma_buf, dev->dev);
	if (IS_ERR(attach)) {
		put_device(dev->dev);
		return ERR_CAST(attach);
	}

	get_dma_buf(dma_buf);

	sg = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sg)) {
		ret = PTR_ERR(sg);
		goto fail_detach;
	}

	ret = gm12u320_prime_create(dev, dma_buf->size, sg, &uobj);
	if (ret)
		goto fail_unmap;

	uobj->base.import_attach = attach;
	uobj->flags = GM12U320_BO_WC;

	return &uobj->base;

fail_unmap:
	dma_buf_unmap_attachment(attach, sg, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);
	put_device(dev->dev);
	return ERR_PTR(ret);
}
