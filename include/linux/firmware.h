/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_FIRMWARE_H
#define _LINUX_FIRMWARE_H

#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/gfp.h>

#define FW_ACTION_NOHOTPLUG 0
#define FW_ACTION_HOTPLUG 1

struct firmware {
	size_t size;
	const u8 *data;
	struct page **pages;

	/* firmware loader private fields */
	void *priv;
};

struct module;
struct device;

struct builtin_fw {
	char *name;
	void *data;
	unsigned long size;
};

/* We have to play tricks here much like stringify() to get the
   __COUNTER__ macro to be expanded as we want it */
#define __fw_concat1(x, y) x##y
#define __fw_concat(x, y) __fw_concat1(x, y)

#define DECLARE_BUILTIN_FIRMWARE(name, blob)				     \
	DECLARE_BUILTIN_FIRMWARE_SIZE(name, &(blob), sizeof(blob))

#define DECLARE_BUILTIN_FIRMWARE_SIZE(name, blob, size)			     \
	static const struct builtin_fw __fw_concat(__builtin_fw,__COUNTER__) \
	__used __section(.builtin_fw) = { name, blob, size }

#if defined(CONFIG_FW_LOADER) || (defined(CONFIG_FW_LOADER_MODULE) && defined(MODULE))
int firmware_request(const struct firmware **fw, const char *name,
		     struct device *device);
int firmware_request_nowarn(const struct firmware **fw, const char *name,
			    struct device *device);
int firmware_request_nowait(
	struct module *module, bool uevent,
	const char *name, struct device *device, gfp_t gfp, void *context,
	void (*cont)(const struct firmware *fw, void *context));
int firmware_request_direct(const struct firmware **fw, const char *name,
			    struct device *device);
int firmware_request_into_buf(const struct firmware **firmware_p,
	const char *name, struct device *device, void *buf, size_t size);

void firmware_release(const struct firmware *fw);
#else
static inline int firmware_request(const struct firmware **fw,
				   const char *name,
				   struct device *device)
{
	return -EINVAL;
}
static inline int firmware_request_nowait(
	struct module *module, bool uevent,
	const char *name, struct device *device, gfp_t gfp, void *context,
	void (*cont)(const struct firmware *fw, void *context))
{
	return -EINVAL;
}

static inline void firmware_release(const struct firmware *fw)
{
}

static inline int firmware_request_direct(const struct firmware **fw,
					  const char *name,
					  struct device *device)
{
	return -EINVAL;
}

static inline int firmware_request_into_buf(const struct firmware **firmware_p,
	const char *name, struct device *device, void *buf, size_t size)
{
	return -EINVAL;
}

#endif

/*
 * Mapping to the old scheme. This mapping will eventually be removed, once we
 * move all subsystems to the new naming scheme. To convert a subsystem:
 *
 * make coccicheck COCCI=scripts/coccinelle/cross-tree/firmware-api-rename.cocci \
 * 	MODE=patch \
 * 	M=path-to-subsys/ \
 * 	SPFLAGS="--in-place"
 */
#define request_firmware		firmware_request
#define request_firmware_nowait		firmware_request_nowait
#define request_firmware_direct		firmware_request_direct
#define request_firmware_into_buf	firmware_request_into_buf
#define release_firmware		firmware_release

int firmware_request_cache(struct device *device, const char *name);

#endif
