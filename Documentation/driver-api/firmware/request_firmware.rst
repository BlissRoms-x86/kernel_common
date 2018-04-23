====================
firmware_request API
====================

You would typically load firmware and then load it into your device somehow.
The typical firmware work flow is reflected below::

	 if(firmware_request(&fw_entry, $FIRMWARE, device) == 0)
                copy_fw_to_device(fw_entry->data, fw_entry->size);
	 firmware_release(fw_entry);

Synchronous firmware requests
=============================

Synchronous firmware requests will wait until the firmware is found or until
an error is returned.

firmware_request
----------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request

firmware_request_nowarn
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_nowarn

firmware_request_direct
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_direct

firmware_request_into_buf
-------------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_into_buf

Asynchronous firmware requests
==============================

Asynchronous firmware requests allow driver code to not have to wait
until the firmware or an error is returned. Function callbacks are
provided so that when the firmware or an error is found the driver is
informed through the callback. firmware_request_nowait() cannot be called
in atomic contexts.

firmware_request_nowait
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_nowait

Special optimizations on reboot
===============================

Some devices have an optimization in place to enable the firmware to be
retained during system reboot. When such optimizations are used the driver
author must ensure the firmware is still available on resume from suspend,
this can be done with firmware_request_cache() instead of requesting for the
firmware to be loaded.

firmware_request_cache()
------------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_cache

request firmware API expected driver use
========================================

Once an API call returns you process the firmware and then release the
firmware. For example if you used firmware_request() and it returns,
the driver has the firmware image accessible in fw_entry->{data,size}.
If something went wrong firmware_request() returns non-zero and fw_entry
is set to NULL. Once your driver is done with processing the firmware it
can call call firmware_release(fw_entry) to release the firmware image
and any related resource.
