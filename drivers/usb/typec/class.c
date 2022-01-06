// SPDX-License-Identifier: GPL-2.0
/*
 * USB Type-C Connector Class
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include "class.h"

static const char * const typec_orientations[] = {
	[TYPEC_ORIENTATION_NONE]	= "unknown",
	[TYPEC_ORIENTATION_NORMAL]	= "normal",
	[TYPEC_ORIENTATION_REVERSE]	= "reverse",
};

static const char * const typec_roles[] = {
	[TYPEC_SINK]	= "sink",
	[TYPEC_SOURCE]	= "source",
};

static const char * const typec_data_roles[] = {
	[TYPEC_DEVICE]	= "device",
	[TYPEC_HOST]	= "host",
};

static const char * const typec_port_power_roles[] = {
	[TYPEC_PORT_SRC] = "source",
	[TYPEC_PORT_SNK] = "sink",
	[TYPEC_PORT_DRP] = "dual",
};

static const char * const typec_port_data_roles[] = {
	[TYPEC_PORT_DFP] = "host",
	[TYPEC_PORT_UFP] = "device",
	[TYPEC_PORT_DRD] = "dual",
};

static const char * const typec_pwr_opmodes[] = {
	[TYPEC_PWR_MODE_USB]	= "default",
	[TYPEC_PWR_MODE_1_5A]	= "1.5A",
	[TYPEC_PWR_MODE_3_0A]	= "3.0A",
	[TYPEC_PWR_MODE_PD]	= "usb_power_delivery",
};

/**
 * typec_find_pwr_opmode - Get the typec power operation mode capability
 * @name: power operation mode string
 *
 * This routine is used to find the typec_pwr_opmode by its string @name.
 *
 * Returns typec_pwr_opmode if success, otherwise negative error code.
 */
int typec_find_pwr_opmode(const char *name)
{
	return match_string(typec_pwr_opmodes,
			    ARRAY_SIZE(typec_pwr_opmodes), name);
}
EXPORT_SYMBOL_GPL(typec_find_pwr_opmode);

/**
 * typec_find_orientation - Convert orientation string to enum typec_orientation
 * @name: Orientation string
 *
 * This routine is used to find the typec_orientation by its string name @name.
 *
 * Returns the orientation value on success, otherwise negative error code.
 */
int typec_find_orientation(const char *name)
{
	return match_string(typec_orientations, ARRAY_SIZE(typec_orientations),
			    name);
}
EXPORT_SYMBOL_GPL(typec_find_orientation);

/**
 * typec_find_port_power_role - Get the typec port power capability
 * @name: port power capability string
 *
 * This routine is used to find the typec_port_type by its string name.
 *
 * Returns typec_port_type if success, otherwise negative error code.
 */
int typec_find_port_power_role(const char *name)
{
	return match_string(typec_port_power_roles,
			    ARRAY_SIZE(typec_port_power_roles), name);
}
EXPORT_SYMBOL_GPL(typec_find_port_power_role);

/**
 * typec_find_power_role - Find the typec one specific power role
 * @name: power role string
 *
 * This routine is used to find the typec_role by its string name.
 *
 * Returns typec_role if success, otherwise negative error code.
 */
int typec_find_power_role(const char *name)
{
	return match_string(typec_roles, ARRAY_SIZE(typec_roles), name);
}
EXPORT_SYMBOL_GPL(typec_find_power_role);

/**
 * typec_find_port_data_role - Get the typec port data capability
 * @name: port data capability string
 *
 * This routine is used to find the typec_port_data by its string name.
 *
 * Returns typec_port_data if success, otherwise negative error code.
 */
int typec_find_port_data_role(const char *name)
{
	return match_string(typec_port_data_roles,
			    ARRAY_SIZE(typec_port_data_roles), name);
}
EXPORT_SYMBOL_GPL(typec_find_port_data_role);

