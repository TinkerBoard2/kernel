// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX477 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define IMX477_REG_VALUE_08BIT		1
#define IMX477_REG_VALUE_16BIT		2

#define SENSOR_NAME		"imx477"

/* Chip ID */
#define IMX477_REG_CHIP_ID		0x0016
#define IMX477_CHIP_ID			0x0477

#define IMX477_REG_MODE_SELECT		0x0100
#define IMX477_MODE_STANDBY		0x00
#define IMX477_MODE_STREAMING		0x01

#define IMX477_REG_ORIENTATION		0x101

#define IMX477_XCLK_FREQ		24000000

#define IMX477_DEFAULT_LINK_FREQ	450000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX477_PIXEL_RATE		840000000

/* V_TIMING internal */
#define IMX477_REG_FRAME_LENGTH		0x0340
#define IMX477_FRAME_LENGTH_MAX		0xffdc

/* Exposure control */
#define IMX477_REG_EXPOSURE		0x0202
#define IMX477_EXPOSURE_OFFSET		22
#define IMX477_EXPOSURE_MIN		20
#define IMX477_EXPOSURE_STEP		1
#define IMX477_EXPOSURE_DEFAULT		0x640
#define IMX477_EXPOSURE_MAX		(IMX477_FRAME_LENGTH_MAX - \
					 IMX477_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX477_REG_ANALOG_GAIN		0x0204
#define IMX477_ANA_GAIN_MIN		0
#define IMX477_ANA_GAIN_MAX		978
#define IMX477_ANA_GAIN_STEP		1
#define IMX477_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX477_REG_DIGITAL_GAIN		0x020e
#define IMX477_DGTL_GAIN_MIN		0x0100
#define IMX477_DGTL_GAIN_MAX		0xffff
#define IMX477_DGTL_GAIN_DEFAULT	0x0100
#define IMX477_DGTL_GAIN_STEP		1

/* Test Pattern Control */
#define IMX477_REG_TEST_PATTERN		0x0600
#define IMX477_TEST_PATTERN_DISABLE	0
#define IMX477_TEST_PATTERN_SOLID_COLOR	1
#define IMX477_TEST_PATTERN_COLOR_BARS	2
#define IMX477_TEST_PATTERN_GREY_COLOR	3
#define IMX477_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX477_REG_TEST_PATTERN_R	0x0602
#define IMX477_REG_TEST_PATTERN_GR	0x0604
#define IMX477_REG_TEST_PATTERN_B	0x0606
#define IMX477_REG_TEST_PATTERN_GB	0x0608
#define IMX477_TEST_PATTERN_COLOUR_MIN	0
#define IMX477_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX477_TEST_PATTERN_COLOUR_STEP	1
#define IMX477_TEST_PATTERN_R_DEFAULT	IMX477_TEST_PATTERN_COLOUR_MAX
#define IMX477_TEST_PATTERN_GR_DEFAULT	0
#define IMX477_TEST_PATTERN_B_DEFAULT	0
#define IMX477_TEST_PATTERN_GB_DEFAULT	0

/* Embedded metadata stream structure */
#define IMX477_EMBEDDED_LINE_WIDTH 16384
#define IMX477_NUM_EMBEDDED_LINES 1

/* IMX477 native and active pixel array size. */
#define IMX477_NATIVE_WIDTH		4072U
#define IMX477_NATIVE_HEIGHT		3176U
#define IMX477_PIXEL_ARRAY_LEFT		8U
#define IMX477_PIXEL_ARRAY_TOP		16U
#define IMX477_PIXEL_ARRAY_WIDTH	4056U
#define IMX477_PIXEL_ARRAY_HEIGHT	3040U

static const s64 link_freq_menu_items[] = {
	IMX477_DEFAULT_LINK_FREQ,
};

struct imx477_reg {
	u16 address;
	u8 val;
};

struct imx477_reg_list {
	unsigned int num_of_regs;
	const struct imx477_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx477_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int hts_def;
	/* V-timing in pixels */
	unsigned int vts_def;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default framerate. */
	struct v4l2_fract timeperframe;

	/* Default register values */
	struct imx477_reg_list reg_list;
};

static const struct imx477_reg mode_common_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02},
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0112, 0x0c},
	{0x0113, 0x0c},
	{0x0114, 0x01},
	{0x0350, 0x00},
	{0xbcf1, 0x02},
	{0x3ff9, 0x01},
};

/* 12 mpix 10fps */
static const struct imx477_reg mode_4056x3040_regs[] = {
	{0x0342, 0x5d},
	{0x0343, 0xc0},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x0f},
	{0x034d, 0xd8},
	{0x034e, 0x0b},
	{0x034f, 0xe0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x02},
	{0x3f57, 0xae},
};

/* 2x2 binned. 40fps */
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x05},
	{0x034f, 0xf0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 1080p cropped mode */
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0342, 0x5d},
	{0x0343, 0xc0},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* Mode configs */
static const struct imx477_mode supported_modes_12bit[] = {
	{
		/* 12MPix 10fps mode */
		.width = 4056,
		.height = 3040,
		.hts_def = 0x5dc0,
		.vts_def = 0x0dac,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 4056,
			.height = 3040,
		},
		.timeperframe = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs),
			.regs = mode_4056x3040_regs,
		},
	},
	{
		/* 2x2 binned 40fps mode */
		.width = 2028,
		.height = 1520,
		.hts_def = 0x31c4,
		.vts_def = 0x0670,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 4056,
			.height = 3040,
		},
		.timeperframe = {
			.numerator = 100,
			.denominator = 4000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1520_regs),
			.regs = mode_2028x1520_regs,
		},
	},
	{
		/* 1080p 50fps cropped mode */
		.width = 2028,
		.height = 1080,
		.hts_def = 0x5dc0,
		.vts_def = 0x048e,
		.crop = {
			.left = 0,
			.top = 440,
			.width = 4056,
			.height = 2600,
		},
		.timeperframe = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2028x1080_regs),
			.regs = mode_2028x1080_regs,
		},
	}
};

static const char * const imx477_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx477_test_pattern_val[] = {
	IMX477_TEST_PATTERN_DISABLE,
	IMX477_TEST_PATTERN_COLOR_BARS,
	IMX477_TEST_PATTERN_SOLID_COLOR,
	IMX477_TEST_PATTERN_GREY_COLOR,
	IMX477_TEST_PATTERN_PN9,
};

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX477_XCLR_MIN_DELAY_US	8000
#define IMX477_XCLR_DELAY_RANGE_US	1000

struct imx477 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk;
	u32 xclk_freq;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx477_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	struct gpio_desc *enable_gpio;
};

static inline struct imx477 *to_imx477(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx477, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx477_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx477_read_reg(struct imx477 *imx477, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx477_write_reg(struct imx477 *imx477, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx477_write_regs(struct imx477 *imx477,
			     const struct imx477_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx477_write_reg(imx477, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static void imx477_set_default_format(struct imx477 *imx477)
{
	struct v4l2_mbus_framefmt *fmt = &imx477->fmt;

	/* Set default mode to max resolution */
	imx477->mode = &supported_modes_12bit[0];

	fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = imx477->mode->width;
	fmt->height = imx477->mode->height;
	fmt->field = V4L2_FIELD_NONE;
}

static int imx477_set_exposure(struct imx477 *imx477, unsigned int val)
{
	int ret;

	ret = imx477_write_reg(imx477, IMX477_REG_EXPOSURE,
			       IMX477_REG_VALUE_16BIT, val);

	return ret;
}

static void imx477_adjust_exposure_range(struct imx477 *imx477,
					 struct v4l2_ctrl *ctrl)
{
	int exposure_max, exposure_def;

	if (ctrl->id == V4L2_CID_VBLANK || !ctrl->val) {
		/*
		 * Either VBLANK has been changed or auto framerate
		 * adjusting has been disabled. Honour the VBLANK limits
		 * when setting exposure.
		 */
		exposure_max = imx477->mode->height + imx477->vblank->val -
						      IMX477_EXPOSURE_OFFSET;
	} else {
		/*
		 * Auto framerate adjusting has been enabled. VBLANK
		 * ctrl has been disabled and exposure can ramp up
		 * to the maximum allowable value.
		 */
		exposure_max = IMX477_EXPOSURE_MAX;
	}

	exposure_def = min(exposure_max, imx477->exposure->val);
	__v4l2_ctrl_modify_range(imx477->exposure, imx477->exposure->minimum,
				 exposure_max, imx477->exposure->step,
				 exposure_def);
}

static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret = 0;
	dev_info(&client->dev, "imx477_set_ctrl(id:0x%x,val:0x%x)\n",
			ctrl->id, ctrl->val);
	if (ctrl->id == V4L2_CID_VBLANK) {
		/*
		 * These controls may change the limits of usable exposure,
		 * so check and adjust if necessary.
		 */
		imx477_adjust_exposure_range(imx477, ctrl);
	}

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_ANALOG_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx477_set_exposure(imx477, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN,
				       IMX477_REG_VALUE_16BIT,
				       imx477_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_R,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GR,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_B,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GB,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx477_write_reg(imx477, IMX477_REG_ORIENTATION, 1,
				       imx477->hflip->val |
				       imx477->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx477_write_reg(imx477, IMX477_REG_FRAME_LENGTH,
				       IMX477_REG_VALUE_16BIT,
				       imx477->mode->height + ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
};

static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB12_1X12;

	return 0;
}

static int imx477_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct imx477_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(fie->code, &mode_list, &num_modes);

	if (fie->index >= num_modes)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SRGGB12_1X12)
		return -EINVAL;

	fie->width = mode_list[fie->index].width;
	fie->height = mode_list[fie->index].height;
	fie->interval = mode_list[fie->index].timeperframe;
	return 0;
}

static void imx477_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx477_update_image_pad_format(struct imx477 *imx477,
					   const struct imx477_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx477_reset_colorspace(&fmt->format);
}

static int imx477_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	mutex_lock(&imx477->mutex);

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY) {
		imx477_update_image_pad_format(imx477, imx477->mode, fmt);
		fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
	}

	mutex_unlock(&imx477->mutex);
	return 0;
}

static void imx477_set_framing_limits(struct imx477 *imx477)
{
	unsigned int vblank, hblank;
	const struct imx477_mode *mode = imx477->mode;

	/* Update limits and set FPS to default */
	vblank = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(imx477->vblank, vblank, vblank, 1, vblank);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx477->vblank, vblank);

	/*
	 * Currently PPL is fixed to the mode specified value, so hblank
	 * depends on mode->width only, and is not changeable in any
	 * way other than changing the mode.
	 */
	hblank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(imx477->hblank, hblank, hblank, 1, hblank);
}

static int imx477_get_reso_dist(const struct imx477_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx477_mode *imx477_find_best_fit(
					struct v4l2_subdev *sd, struct v4l2_subdev_format *fmt)
{
	//struct imx477 *imx477 = to_imx477(sd);
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;
	const struct imx477_mode *mode_list;
	unsigned int num_modes;

	/* Bayer order varies with flips */
	fmt->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;

	get_mode_table(fmt->format.code, &mode_list, &num_modes);

	for (i = 0; i < num_modes; i++) {
		dist = imx477_get_reso_dist(&mode_list[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &mode_list[cur_best_fit];
}

static int imx477_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	const struct imx477_mode *mode;
	struct imx477 *imx477 = to_imx477(sd);

	mutex_lock(&imx477->mutex);

	mode = imx477_find_best_fit(sd, fmt);
	imx477_update_image_pad_format(imx477, mode, fmt);
	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY) {
		imx477->mode = mode;
		imx477_set_framing_limits(imx477);
	}
	mutex_unlock(&imx477->mutex);

	return 0;
}

/* Start streaming */
static int imx477_start_streaming(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	const struct imx477_reg_list *reg_list;
	int ret;

	if (!imx477->common_regs_written) {
		ret = imx477_write_regs(imx477, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx477->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx477->mode->reg_list;
	ret = imx477_write_regs(imx477, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* set stream on register */
	return imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

/* Stop streaming */
static void imx477_stop_streaming(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;

	/* set stream off register */
	ret = imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx477_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->mode;
	fi->interval = mode->timeperframe;

	return 0;
}

static int imx477_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx477 *imx477 = to_imx477(sd);
	int ret = 0;

	mutex_lock(&imx477->mutex);
	if (imx477->streaming == enable) {
		mutex_unlock(&imx477->mutex);
		return 0;
	}
	pr_info("imx477_set_stream() enable %d\n", enable);
	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx477_start_streaming(imx477);
		if (ret)
			goto err_unlock;
	} else {
		imx477_stop_streaming(imx477);
	}

	imx477->streaming = enable;

	mutex_unlock(&imx477->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx477->mutex);

	return ret;
}

/* V4L2 subdev core operations */
static int imx477_s_power(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	dev_info(&client->dev, "imx477_s_power() enable %d\n", enable);
	if(enable) {
		ret = clk_prepare_enable(imx477->xclk);
		if (ret) {
			dev_err(&client->dev, "%s: failed to enable clock\n",
				__func__);
		}
	} else {
		clk_disable_unprepare(imx477->xclk);
		/* Force reprogramming of the common registers when powered up again. */
		imx477->common_regs_written = false;
	}


	return 0;
}

/* Verify chip ID */
static int imx477_identify_module(struct imx477 *imx477)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;
	u32 val;

	ret = imx477_read_reg(imx477, IMX477_REG_CHIP_ID,
			      IMX477_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			IMX477_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX477_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX477_CHIP_ID, val);
		return -EIO;
	}

	dev_info(&client->dev, "chip id %x\n",val);

	return 0;
}

static void imx477_get_module_inf(struct imx477 *imx477,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SENSOR_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx477->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx477->len_name, sizeof(inf->base.lens));
}

static long imx477_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx477 *imx477 = to_imx477(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx477_get_module_inf(imx477, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx477_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx477_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx477_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops imx477_core_ops = {
	.s_power = imx477_s_power,
	.ioctl = imx477_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx477_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_set_stream,
	.g_frame_interval = imx477_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.enum_frame_interval = imx477_enum_frame_interval,
	.get_fmt = imx477_get_pad_format,
	.set_fmt = imx477_set_pad_format,
};

static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.core = &imx477_core_ops,
	.video = &imx477_video_ops,
	.pad = &imx477_pad_ops,
};

/* Initialize control handlers */
static int imx477_init_controls(struct imx477 *imx477)
{
	const struct imx477_mode *mode = imx477->mode;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	u32 h_blank, v_blank;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx477->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx477->mutex);
	ctrl_hdlr->lock = &imx477->mutex;

	imx477->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);

	/* By default, PIXEL_RATE is read only */
	imx477->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX477_PIXEL_RATE,
					       IMX477_PIXEL_RATE, 1,
					       IMX477_PIXEL_RATE);
	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx477_set_framing_limits() call below.
	 */
	h_blank = mode->hts_def - mode->width;
	imx477->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_HBLANK, h_blank, h_blank, 1, h_blank);

	v_blank = mode->vts_def - mode->height;
	imx477->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, v_blank, v_blank,
					   1, v_blank);

	/* HBLANK is read-only for now, but does change with mode. */
	if (imx477->hblank)
		imx477->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx477->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     IMX477_EXPOSURE_MAX,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANA_GAIN_MIN, IMX477_ANA_GAIN_MAX,
			  IMX477_ANA_GAIN_STEP, IMX477_ANA_GAIN_DEFAULT);

	imx477->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx477->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				     0, 0, imx477_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX477_TEST_PATTERN_COLOUR_MIN,
				  IMX477_TEST_PATTERN_COLOUR_MAX,
				  IMX477_TEST_PATTERN_COLOUR_STEP,
				  IMX477_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	imx477->sd.ctrl_handler = ctrl_hdlr;
	/* Setup exposure and frame/line length limits. */
	imx477_set_framing_limits(imx477);

	ret = v4l2_ctrl_handler_setup(ctrl_hdlr);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n", ret);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx477->mutex);

	return ret;
}

static void imx477_free_controls(struct imx477 *imx477)
{
	v4l2_ctrl_handler_free(imx477->sd.ctrl_handler);
	mutex_destroy(&imx477->mutex);
}

static int imx477_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx477 *imx477;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "imx477_probe() enter.\n");
	imx477 = devm_kzalloc(&client->dev, sizeof(*imx477), GFP_KERNEL);
	if (!imx477)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx477->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx477->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx477->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx477->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	imx477->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(imx477->enable_gpio))
		dev_warn(dev, "Failed to get enable_gpios\n");

	/* Get system clock (xclk) */
	imx477->xclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx477->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx477->xclk);
	}

	ret = clk_set_rate(imx477->xclk, IMX477_XCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");

	imx477->xclk_freq = clk_get_rate(imx477->xclk);
	if (imx477->xclk_freq != IMX477_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx477->xclk_freq);
	}

	v4l2_i2c_subdev_init(&imx477->sd, client, &imx477_subdev_ops);
	sd = &imx477->sd;

	gpiod_set_value_cansleep(imx477->enable_gpio, 1);
	usleep_range(IMX477_XCLR_MIN_DELAY_US,
			IMX477_XCLR_MIN_DELAY_US + IMX477_XCLR_DELAY_RANGE_US);
	ret = imx477_s_power(sd, 1);
	if (ret)
		return ret;

	ret = imx477_identify_module(imx477);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx477_set_default_format(imx477);

	/* This needs the pm runtime to be registered. */
	ret = imx477_init_controls(imx477);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx477->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx477->pad.flags = MEDIA_PAD_FL_SOURCE;
	imx477->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&imx477->sd.entity, 1, &imx477->pad, 0);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}
	memset(facing, 0, sizeof(facing));
	if (strcmp(imx477->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx477->module_index, facing,
		 SENSOR_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(&imx477->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	imx477_s_power(sd, 0);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx477->sd.entity);

error_handler_free:
	imx477_free_controls(imx477);

error_power_off:
	imx477_s_power(sd, 0);

	return ret;
}

static int imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx477_free_controls(imx477);
	imx477_s_power(sd, 0);

	return 0;
}

static const struct i2c_device_id imx477_id[] = {
	{"imx477", 0},
	{}
};

static const struct of_device_id imx477_dt_ids[] = {
	{ .compatible = "sony,imx477" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx477_dt_ids);

MODULE_DEVICE_TABLE(i2c, imx477_id);
static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.of_match_table	= imx477_dt_ids,
	},
	.probe = imx477_probe,
	.remove = imx477_remove,
	.id_table = imx477_id,
};

module_i2c_driver(imx477_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX477 sensor driver");
MODULE_LICENSE("GPL v2");
