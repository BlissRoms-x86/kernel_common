// SPDX-License-Identifier: GPL-2.0
/**
 * Device connections
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/connection.h>
#include <linux/device.h>

static DEFINE_MUTEX(devcon_lock);
static LIST_HEAD(devcon_list);

/**
 * __device_find_connection - Find physical connection to a device
 * @dev: Device with the connection
 * @con_id: Identifier for the connection
 * @data: Data for the match function
 * @match: Function to check and convert the connection description
 *
 * Find a connection with unique identifier @con_id between @dev and another
 * device. @match will be used to convert the connection description to data the
 * caller is expecting to be returned.
 */
void *__device_find_connection(struct device *dev, const char *con_id,
			       void *data,
			       void *(*match)(struct devcon *con, int ep,
					      void *data))
{
	const char *devname = dev_name(dev);
	struct devcon *con;
	void *ret = NULL;
	int ep;

	if (!match)
		return NULL;

	rcu_read_lock();

	list_for_each_entry_rcu(con, &devcon_list, list) {
		ep = match_string(con->endpoint, 2, devname);
		if (ep < 0)
			continue;

		if (con_id && strcmp(con->id, con_id))
			continue;

		ret = match(con, !ep, data);
		if (ret)
			break;
	}

	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(__device_find_connection);

#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/pci.h>

static struct bus_type *generic_match_buses[] = {
	&platform_bus_type,
#ifdef CONFIG_PCI
	&pci_bus_type,
#endif
#ifdef CONFIG_I2C
	&i2c_bus_type,
#endif
#ifdef CONFIG_SPI_MASTER
	&spi_bus_type,
#endif
	NULL,
};

/* This tries to find the device from the most common bus types by name. */
static void *generic_match(struct devcon *con, int ep, void *data)
{
	struct bus_type *bus;
	struct device *dev;

	for (bus = generic_match_buses[0]; bus; bus++) {
		dev = bus_find_device_by_name(bus, NULL, con->endpoint[ep]);
		if (dev)
			return dev;
	}

	/*
	 * We only get called if a connection was found, tell the caller to
	 * wait for the other device to show up.
	 */
	return ERR_PTR(-EPROBE_DEFER);
}

/**
 * device_find_connection - Find two devices connected together
 * @dev: Device with the connection
 * @con_id: Identifier for the connection
 *
 * Find a connection with unique identifier @con_id between @dev and
 * another device. On success returns handle to the device that is connected
 * to @dev, with the reference count for the found device incremented. Returns
 * NULL if no matching connection was found, or ERR_PTR(-EPROBE_DEFER) when a
 * connection was found but the other device has not been enumerated yet.
 */
struct device *device_find_connection(struct device *dev, const char *con_id)
{
	return __device_find_connection(dev, con_id, generic_match, NULL);
}
EXPORT_SYMBOL_GPL(device_find_connection);

/**
 * add_device_connection - Register a collection of connection descriptions
 * @con: Collection of connection descriptions to be registered
 */
void add_device_connection(struct devcon *con)
{
	mutex_lock(&devcon_lock);
	list_add_tail_rcu(&con->list, &devcon_list);
	mutex_unlock(&devcon_lock);
}
EXPORT_SYMBOL_GPL(add_device_connection);

/**
 * remove_device_connections - Unregister collection of connection descriptions
 * @con: Collection of connection descriptions to be unregistered
 */
void remove_device_connection(struct devcon *con)
{
	mutex_lock(&devcon_lock);
	list_del_rcu(&con->list);
	mutex_unlock(&devcon_lock);
	/* The caller may free the devcon struct immediately afterwards. */
	synchronize_rcu();
}
EXPORT_SYMBOL_GPL(remove_device_connection);
