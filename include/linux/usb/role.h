// SPDX-License-Identifier: GPL-2.0

#ifndef __LINUX_USB_ROLE_H
#define __LINUX_USB_ROLE_H

#include <linux/device.h>

struct usb_role_switch;

enum usb_role {
	USB_ROLE_NONE,
	USB_ROLE_HOST,
	USB_ROLE_DEVICE,
};

#endif /* __LINUX_USB_ROLE_H */

