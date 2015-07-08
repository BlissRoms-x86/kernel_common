/*
 * Copyright (C) 2012-2016 Red Hat Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include "gm12u320_drv.h"

static int gm12u320_driver_set_busid(struct drm_device *d, struct drm_master *m)
{
	return 0;
}

static const struct vm_operations_struct gm12u320_gem_vm_ops = {
	.fault = gm12u320_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations gm12u320_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = gm12u320_drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.release = drm_release,
	.compat_ioctl = drm_compat_ioctl,
	.llseek = noop_llseek,
};

static struct drm_driver driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load = gm12u320_driver_load,
	.unload = gm12u320_driver_unload,
	.set_busid = gm12u320_driver_set_busid,

	/* gem hooks */
	.gem_free_object = gm12u320_gem_free_object,
	.gem_vm_ops = &gm12u320_gem_vm_ops,

	.dumb_create = gm12u320_dumb_create,
	.dumb_map_offset = gm12u320_gem_mmap,
	.dumb_destroy = drm_gem_dumb_destroy,
	.fops = &gm12u320_driver_fops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = gm12u320_gem_prime_export,
	.gem_prime_import = gm12u320_gem_prime_import,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int gm12u320_usb_probe(struct usb_interface *interface,
			      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct drm_device *dev;
	int r;

	/*
	 * The gm12u320 presents itself to the system as 2 usb mass-storage
	 * interfaces, for the second one we proceed successully with binding,
	 * but otherwise ignore it.
	 */
	if (interface->cur_altsetting->desc.bInterfaceNumber != 0)
		return 0;

	dev = drm_dev_alloc(&driver, &interface->dev);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	r = drm_dev_register(dev, (unsigned long)udev);
	if (r)
		goto err_free;

	usb_set_intfdata(interface, dev);
	DRM_INFO("Initialized gm12u320 on minor %d\n", dev->primary->index);

	return 0;

err_free:
	drm_dev_unref(dev);
	return r;
}

static void gm12u320_usb_disconnect(struct usb_interface *interface)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	if (!dev)
		return;

	drm_kms_helper_poll_disable(dev);
	gm12u320_fbdev_unplug(dev);
	gm12u320_stop_fb_update(dev);
	drm_unplug_dev(dev);
}

#ifdef CONFIG_PM

int gm12u320_suspend(struct usb_interface *interface, pm_message_t message)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	if (!dev)
		return 0;

	gm12u320_stop_fb_update(dev);
	return 0;
}

int gm12u320_resume(struct usb_interface *interface)
{
	struct drm_device *dev = usb_get_intfdata(interface);

	if (!dev)
		return 0;

	gm12u320_set_ecomode(dev);
	gm12u320_start_fb_update(dev);
	return 0;
}
#endif

static struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x1de1, 0xc102) },
	{},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver gm12u320_driver = {
	.name = "gm12u320",
	.probe = gm12u320_usb_probe,
	.disconnect = gm12u320_usb_disconnect,
	.id_table = id_table,
#ifdef CONFIG_PM
	.suspend = gm12u320_suspend,
	.resume = gm12u320_resume,
	.reset_resume = gm12u320_resume,
#endif
};

module_usb_driver(gm12u320_driver);
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
