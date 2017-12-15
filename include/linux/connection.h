// SPDX-License-Identifier: GPL-2.0

#ifndef _LINUX_CONNECTION_H_
#define _LINUX_CONNECTION_H_

#include <linux/list.h>

struct device;

/**
 * struct devcon - Device Connection Descriptor
 * @endpoint: The names of the two devices connected together
 * @id: Unique identifier for the connection
 * @list: List head, private for devcon internal use only
 */
struct devcon {
	const char		*endpoint[2];
	const char		*id;
	struct list_head	list;
};

void *__device_find_connection(struct device *dev, const char *con_id,
			       void *data,
			       void *(*match)(struct devcon *con, int ep,
					      void *data));

struct device *device_find_connection(struct device *dev, const char *con_id);

#define DEVCON(_ep0, _ep1, _id)    (struct devcon) { { _ep0, _ep1 }, _id, }

void add_device_connection(struct devcon *con);
void remove_device_connection(struct devcon *con);

#endif /* _LINUX_CONNECTION_H_ */
