/// Cross-tree firmware API rename
///
// Confidence: High
// Copyright: (C) 2018 Luis R. Rodriguez. GPLv2
// URL: http://coccinelle.lip6.fr/
// Comments:
// Options: --no-includes --include-headers
// We can remove this when the API conversion is fully done.

virtual patch

@ firmware_request depends on patch @
expression fw, name, dev;
@@

-request_firmware(
+firmware_request(
fw, name, dev)

@ firmware_request_direct depends on patch @
expression fw, name, dev;
@@

-request_firmware_direct(
+firmware_request_direct(
fw, name, dev)

@ firmware_request_into_buf depends on patch @
expression fw, name, dev, buf, size;
@@

-request_firmware_into_buf(
+firmware_request_into_buf(
fw, name, dev, buf, size)

@ firmware_request_nowait depends on patch @
expression module, uevent, name, dev, gfp, conext, cb;
@@

-request_firmware_nowait(
+firmware_request_nowait(
module, uevent, name, dev, gfp, conext, cb)

@ firmware_release depends on patch @
expression fw;
@@

-release_firmware(
+firmware_release(
fw)
