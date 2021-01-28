/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRM_I2C_ADV7511_H__
#define __DRM_I2C_ADV7511_H__

#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

#include <video/of_videomode.h>
#include <video/videomode.h>
#include <drm/drm_dp_helper.h>

struct sn65dsi86_data {
	struct backlight_device *backlight;
	struct i2c_client *client;
	struct device *dev;

	unsigned int dsi_lanes;
	unsigned int width_mm;
	unsigned int height_mm;
	bool test_pattern_en;
	bool enabled;

	bool powered;

	struct gpio_desc *edp_vdd_en_gpio;
	struct gpio_desc *sn65dsi86_en_gpio;
	struct gpio_desc *dsi86_vbl_en_gpio;
	struct gpio_desc *pwr_source_gpio;

	enum drm_connector_status status;
	struct drm_connector connector;
	struct device_node *host_node;
	struct drm_bridge bridge;

	struct mipi_dsi_device *dsi;
	struct videomode  vm;

	struct drm_display_mode mode;
	unsigned int bus_format;
	unsigned int bpc;
	unsigned int format;
	unsigned int mode_flags;
	unsigned int t1;
	unsigned int t2;
	unsigned int t3;
	unsigned int t4;
	unsigned int t5;
	unsigned int t6;
	unsigned int t7;
	unsigned int t8;
	unsigned int t12;
	unsigned int t14;
	unsigned int t15;
	unsigned int t16;
	unsigned int t17;

	struct workqueue_struct *wq;
	struct work_struct work;
	struct gpio_desc *dsi86_irq_gpio;
	unsigned int	dsi86_irq;

	///////////////////////// ti
	struct drm_dp_aux		aux;
	struct dentry			*debugfs;
	struct clk			*refclk;

	int				dp_lanes;
	u8				ln_assign;
	u8				ln_polrs;
	/////////////////////////
};
#endif /* __DRM_I2C_ADV7511_H__ */
