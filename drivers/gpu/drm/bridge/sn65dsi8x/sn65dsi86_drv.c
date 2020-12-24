/*
 * Analog Devices sn65dsi86 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>

#include "sn65dsi86.h"
#include <linux/i2c.h>

#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>

#define DRIVER_NAME "sn65dsi86"

#define SN_DEVICE_REV_REG			0x08
#define SN_DPPLL_SRC_REG			0x0A
#define  DPPLL_CLK_SRC_DSICLK			BIT(0)
#define  REFCLK_FREQ_MASK			GENMASK(3, 1)
#define  REFCLK_FREQ_27M			0x3
#define  REFCLK_FREQ(x)				((x) << 1)
#define  DPPLL_SRC_DP_PLL_LOCK			BIT(7)
#define SN_PLL_ENABLE_REG			0x0D
#define SN_DSI_LANES_REG			0x10
#define  CHA_DSI_LANES_MASK			GENMASK(4, 3)
#define  CHA_DSI_LANES(x)			((x) << 3)
#define SN_DSIA_CLK_FREQ_REG			0x12
#define SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG	0x20
#define SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG	0x24
#define SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG	0x2C
#define SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG	0x2D
#define  CHA_HSYNC_POLARITY			BIT(7)
#define SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG	0x30
#define SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG	0x31
#define  CHA_VSYNC_POLARITY			BIT(7)
#define SN_CHA_HORIZONTAL_BACK_PORCH_REG	0x34
#define SN_CHA_VERTICAL_BACK_PORCH_REG		0x36
#define SN_CHA_HORIZONTAL_FRONT_PORCH_REG	0x38
#define SN_CHA_VERTICAL_FRONT_PORCH_REG		0x3A
#define SN_LN_ASSIGN_REG			0x59
#define  LN_ASSIGN_WIDTH			2
#define SN_ENH_FRAME_REG			0x5A
#define  AUTHEN_METHOD_MASK			GENMASK(0, 1)
#define  SCRAMBLER_SEED_RESET			BIT(0)
#define  ENH_FRAME_ENABLE				BIT(2)
#define  VSTREAM_ENABLE				BIT(3)
#define  LN_POLRS_OFFSET			4
#define  LN_POLRS_MASK				0xf0
#define SN_DATA_FORMAT_REG			0x5B
#define  BPP_18_RGB				BIT(0)
#define SN_HPD_DISABLE_REG			0x5C
#define  HPD_DISABLE				BIT(0)
#define SN_GPIO_IO_REG				0x5E
#define  SN_GPIO_INPUT_SHIFT			4
#define  SN_GPIO_OUTPUT_SHIFT			0
#define SN_GPIO_CTRL_REG			0x5F
#define  SN_GPIO_MUX_INPUT			0
#define  SN_GPIO_MUX_OUTPUT			1
#define  SN_GPIO_MUX_SPECIAL			2
#define  SN_GPIO_MUX_MASK			0x3
#define SN_I2C_ADDR_CLAIM1			0x60
#define I2C_CLAM1_EN_MASK			0x1
#define I2C_CLAM1_EN				BIT(0)
#define SN_AUX_WDATA_REG(x)			(0x64 + (x))
#define SN_AUX_ADDR_19_16_REG			0x74
#define SN_AUX_ADDR_15_8_REG			0x75
#define SN_AUX_ADDR_7_0_REG			0x76
#define SN_AUX_LENGTH_REG			0x77
#define SN_AUX_CMD_REG				0x78
#define  AUX_CMD_SEND				BIT(0)
#define  AUX_CMD_REQ(x)				((x) << 4)
#define SN_AUX_RDATA_REG(x)			(0x79 + (x))
#define SN_SSC_CONFIG_REG			0x93
#define  DP_NUM_LANES_MASK			GENMASK(5, 4)
#define  DP_NUM_LANES(x)			((x) << 4)
#define SN_DATARATE_CONFIG_REG			0x94
#define  DP_DATARATE_MASK			GENMASK(7, 5)
#define  DP_DATARATE(x)				((x) << 5)
#define SN_ML_TX_MODE_REG			0x96
#define  ML_TX_MAIN_LINK_OFF			0
#define  ML_TX_NORMAL_MODE			BIT(0)
#define SN_AUX_CMD_STATUS_REG			0xF4
#define  AUX_IRQ_STATUS_AUX_RPLY_TOUT		BIT(3)
#define  AUX_IRQ_STATUS_AUX_SHORT		BIT(5)
#define  AUX_IRQ_STATUS_NAT_I2C_FAIL		BIT(6)

#define MIN_DSI_CLK_FREQ_MHZ   40
#define DSI_CLK_FREQ_INCREMENT 5

/* fudge factor required to account for 8b/10b encoding */
#define DP_CLK_FUDGE_NUM	10
#define DP_CLK_FUDGE_DEN	8

/* Matches DP_AUX_MAX_PAYLOAD_BYTES (for now) */
#define SN_AUX_MAX_PAYLOAD_BYTES	16

#define SN_REGULATOR_SUPPLY_NUM		4

#define SN_MAX_DP_LANES			4
#define SN_NUM_GPIOS			4
#define SN_GPIO_PHYSICAL_OFFSET		1


#define SN_IRQ_EN 0xE0
#define SN_IRQ_EN_1 0xE1
#define SN_IRQ_EN_2 0xE2
#define SN_IRQ_EN_3 0xE3
#define SN_IRQ_EN_4 0xE4
#define SN_IRQ_EN_5 0xE5
#define SN_IRQ_EN_6 0xE6
#define SN_IRQ_EN_7 0xE7
#define SN_IRQ_EN_8 0xE8
#define SN_IRQ_EN_9 0xE9
#define SN_IRQ_STATUS0 0xF0
#define SN_IRQ_STATUS1 0xF1
#define SN_IRQ_STATUS2 0xF2
#define SN_IRQ_STATUS3 0xF3
#define SN_IRQ_STATUS4 0xF4
#define SN_IRQ_STATUS5 0xF5
#define SN_IRQ_STATUS6 0xF6
#define SN_IRQ_STATUS7 0xF7
#define SN_IRQ_STATUS8 0xF8
#define  SN_PWM_PRE_DIV			0xA0
#define  SN_BACKLIGHT_SCAL_LOW 	0xA1
#define  SN_BACKLIGHT_SCAL_HIGH 	0xA2
#define  SN_BACKLIGHT_LOW 			0xA3
#define  SN_BACKLIGHT_HIGH			0xA4
#define  SN_PWM_EN 					0xA5
#define  PWM_EN_OFFSET		1
#define  PWM_EN_MASK		0x1
#define  SN_GPIO4_CTL_OFFSET		6
#define  SN_GPIO4_MUX_PWM			0x2


#define MAX_BRIGHENESS 		(255)
#define DDC_ADDR		0x50
#define EDID_SIZE		128

static bool sn65dsi86_exist = false;
struct sn65dsi86_data *gdata;

#ifndef DIRECT_EDID_METHOD
static ssize_t sn65dsi86_read_edid(struct sn65dsi86_data *pdata);
#endif
void sn65dsi86_enable_irq(struct sn65dsi86_data *sn65dsi86, bool enable);
static void sn65dsi86_dump_status_register(struct sn65dsi86_data *sn65dsi86);
static void sn65dsi86_dump_status_register(struct sn65dsi86_data *sn65dsi86);
#ifdef PWM_FROM_SN65DSI86
static void sn65dsi86_gpio4_to_pwm(struct sn65dsi86_data *pdata, bool enable);
static void sn65dsi86_set_pwm_freq(struct sn65dsi86_data *pdata);
static void sn65dsi86_pwm_enable(struct sn65dsi86_data *pdata, bool enable);
#endif

bool sn65dsi86_is_connected(void)
{
	return sn65dsi86_exist;
}
EXPORT_SYMBOL_GPL(sn65dsi86_is_connected);

static void edp_power_on(struct sn65dsi86_data *sn65dsi86)
{
	printk(KERN_INFO "%s \n", __func__);
	if (sn65dsi86->edp_vdd_en_gpio) {
		gpiod_set_value_cansleep(sn65dsi86->edp_vdd_en_gpio, 1);
		//msleep(20);//T2: 0.01ms ~50ms
	}
}

static void edp_power_off(struct sn65dsi86_data *sn65dsi86)
{
	printk(KERN_INFO "%s \n", __func__);
	if (sn65dsi86->edp_vdd_en_gpio) {
		//msleep(10);//T5: 0.01ms ~50ms
		gpiod_set_value_cansleep(sn65dsi86->edp_vdd_en_gpio, 0);
		//msleep(1000);//T7: 1000ms
	}
}

int sn65dsi86_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at reg=0x%02x\n", reg);
		*val = 0;
		return ret;
	}

	*val = ret;
	return 0;
}

int sn65dsi86_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	//printk("sn65dsi86_write reg=0x%02x, val=0x%02x\n", reg, val);
	if (ret)
		dev_err(&client->dev, "failed to write at reg=0x%02x, val=0x%02x\n", reg, val);

	return ret;
}

int sn65dsi86_update_bit(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}

	tmp = (u8)ret;
	tmp &= ~mask;
	tmp |= data & mask;

	return sn65dsi86_write(client, reg, tmp);
}

#define sn65dsi86_read_poll_timeout(client, addr, val, cond, sleep_us, timeout_us) \
({ \
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us); \
	int ret; \
	might_sleep_if(sleep_us); \
	for (;;) { \
		ret = sn65dsi86_read((client), (addr), &(val)); \
		if (ret) \
			break; \
		if (cond) \
			break; \
		if (timeout_us && ktime_compare(ktime_get(), timeout) > 0) { \
			ret = sn65dsi86_read((client), (addr), &(val)); \
			break; \
		} \
		if (sleep_us) \
			usleep_range((sleep_us >> 2) + 1, sleep_us); \
	} \
	ret ?: ((cond) ? 0 : -ETIMEDOUT); \
})

static void sn65dsi86_chip_enable(struct sn65dsi86_data *sn65dsi86)
{
	printk(KERN_INFO "%s \n", __func__);
	if (sn65dsi86->sn65dsi86_en_gpio) {
		gpiod_set_value_cansleep(sn65dsi86->sn65dsi86_en_gpio, 1);
		msleep(10);
	}

	sn65dsi86->powered = true;
}

static void sn65dsi86_chip_shutdown(struct sn65dsi86_data *sn65dsi86)
{
	printk(KERN_INFO "%s \n", __func__);
	if (sn65dsi86->sn65dsi86_en_gpio) {
		gpiod_set_value_cansleep(sn65dsi86->sn65dsi86_en_gpio, 0);
		msleep(10);
	}

	sn65dsi86->powered = false;
}

static int status_show(struct seq_file *s, void *data)
{
	struct sn65dsi86_data *pdata = s->private;
	unsigned int reg;
	uint8_t val;

	seq_puts(s, "STATUS REGISTERS:\n");

	pm_runtime_get_sync(pdata->dev);

	/* IRQ Status Registers, see Table 31 in datasheet */
	for (reg = 0xf0; reg <= 0xf8; reg++) {
		sn65dsi86_read(pdata->client, reg, &val);
		seq_printf(s, "[0x%02x] = 0x%08x\n", reg, val);
	}

	pm_runtime_put(pdata->dev);

	return 0;
}

#define DEFINE_SHOW_ATTRIBUTE(__name)					\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name ## _show, inode->i_private);	\
}									\
									\
static const struct file_operations __name ## _fops = {			\
	.owner		= THIS_MODULE,					\
	.open		= __name ## _open,				\
	.read		= seq_read,					\
	.llseek		= seq_lseek,					\
	.release	= single_release,				\
}


DEFINE_SHOW_ATTRIBUTE(status);

static void sn65dsi86_debugfs_init(struct sn65dsi86_data *sn65dsi86)
{
	sn65dsi86->debugfs = debugfs_create_dir(dev_name(sn65dsi86->dev), NULL);

	debugfs_create_file("status", 0600, sn65dsi86->debugfs, sn65dsi86,
			&status_fops);
}

static void sn65dsi86_debugfs_remove(struct sn65dsi86_data *sn65dsi86)
{
	debugfs_remove_recursive(sn65dsi86->debugfs);
	sn65dsi86->debugfs = NULL;
}

/* Connector funcs */
static struct sn65dsi86_data *
connector_to_sn65dsi86(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi86_data, connector);
}

static int sn65dsi86_get_modes(struct sn65dsi86_data *sn65dsi86,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	u32 bus_format = sn65dsi86->bus_format;//MEDIA_BUS_FMT_RGB666_1X18;
	int ret;

	printk(KERN_INFO "%s +\n", __func__);
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		printk(KERN_INFO "%s :Failed to create display mode!\n", __func__);
		return 0;
	}

	drm_display_mode_from_videomode(&sn65dsi86->vm, mode);
	mode->width_mm = sn65dsi86->width_mm;
	mode->height_mm = sn65dsi86->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_probed_add(connector, mode);
   	drm_mode_connector_list_update(connector, true);
	connector->display_info.width_mm = sn65dsi86->width_mm;
	connector->display_info.height_mm = sn65dsi86->height_mm;
	connector->display_info.bpc = sn65dsi86->bpc;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret) {
		printk(KERN_ERR "%s return ret=%d\n",  __func__, ret);
		return ret;
	}

	//drm_mode_probed_add(connector, mode);
 	printk(KERN_INFO "%s -\n", __func__);
	return 1;
}

static int sn65dsi86_bridge_connector_get_modes(struct drm_connector *connector)
{
	struct sn65dsi86_data *pdata = connector_to_sn65dsi86(connector);

	//return drm_panel_get_modes(pdata->panel, connector);
	return  sn65dsi86_get_modes(pdata, connector);
}

static enum drm_mode_status
sn65dsi86_bridge_connector_mode_valid(struct drm_connector *connector,
				  struct drm_display_mode *mode)
{
	/* maximum supported resolution is 4K at 60 fps */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static struct drm_connector_helper_funcs sn65dsi86_bridge_connector_helper_funcs = {
	.get_modes = sn65dsi86_bridge_connector_get_modes,
	.mode_valid = sn65dsi86_bridge_connector_mode_valid,
};

static enum drm_connector_status
sn65dsi86_detect(struct sn65dsi86_data *sn65dsi86)
{
	#define ID_REGISTERS_SZIE (8)
	enum drm_connector_status status = sn65dsi86->status;
	uint8_t id[ID_REGISTERS_SZIE] = {0x36, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20};
	uint8_t return_id[ID_REGISTERS_SZIE] = {0};
	int i;

	printk(KERN_INFO "%s \n", __func__);

	if (status == connector_status_connected)
		return status;

	for (i = 0; i < sizeof(id) /sizeof(uint8_t); i++) {
		sn65dsi86_read(sn65dsi86->client, i, &return_id[i]);
	}

	if (!memcmp(id, return_id, sizeof(id) /sizeof(uint8_t))) {
		printk(KERN_INFO "sn65dsi86_detect successful\n");
		status = connector_status_connected;
	} else {
		printk(KERN_ERR "sn65dsi86_detect fail, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			return_id[0], return_id[1], return_id[2], return_id[3], return_id[4], return_id[5], return_id[6], return_id[7]);
	}

	sn65dsi86->status = status;

	printk(KERN_INFO "%s sn65dsi86->status=%d\n", __func__, sn65dsi86->status);

	return status;
}


static enum drm_connector_status
sn65dsi86_connector_detect(struct drm_connector *connector, bool force)
{
	struct sn65dsi86_data *sn65dsi86 = connector_to_sn65dsi86(connector);

	return sn65dsi86_detect(sn65dsi86);
}

static const struct drm_connector_funcs sn65dsi86_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi86_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct sn65dsi86_data *bridge_to_sn65dsi86(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi86_data, bridge);
}

static int sn65dsi86_attach_dsi(struct sn65dsi86_data *sn65dsi86)
{
	const struct mipi_dsi_device_info info = { .type = "sn65dsi86",
						   .channel = 0,
						   //.node = NULL;
						   .node = sn65dsi86->bridge.of_node,
						 };
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	//uint8_t val;
	int ret;
	/*
	 * TODO: ideally finding host resource and dsi dev registration needs
	 * to be done in bridge probe. But some existing DSI host drivers will
	 * wait for any of the drm_bridge/drm_panel to get added to the global
	 * bridge/panel list, before completing their probe. So if we do the
	 * dsi dev registration part in bridge probe, before populating in
	 * the global bridge list, then it will cause deadlock as dsi host probe
	 * will never complete, neither our bridge probe. So keeping it here
	 * will satisfy most of the existing host drivers. Once the host driver
	 * is fixed we can move the below code to bridge probe safely.
	 */
	host = of_find_mipi_dsi_host_by_node(sn65dsi86->host_node);
	if (!host) {
		printk("sn65dsi86_bridge_attach: failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		printk("sn65dsi86_bridge_attach: failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	/* TODO: setting to 4 MIPI lanes always for now */
	dsi->lanes = sn65dsi86->dsi_lanes;
	dsi->format = sn65dsi86->format;
	dsi->mode_flags = sn65dsi86->mode_flags;

	/* check if continuous dsi clock is required or not */
	//pm_runtime_get_sync(sn65dsi86->dev);
	//sn65dsi86_read(sn65dsi86->client, SN_DPPLL_SRC_REG, &val);
	//pm_runtime_put(sn65dsi86->dev);
	//if (!(val & DPPLL_CLK_SRC_DSICLK))
	//	dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS;

	printk("sn65dsi86_bridge_attach: lanes=%d format=%d mode_flags=0x%lx\n", dsi->lanes, dsi->format, dsi->mode_flags);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		printk("sn65dsi86_bridge_attach: failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	sn65dsi86->dsi = dsi;

	/* attach panel to bridge */
	//drm_panel_attach(pdata->panel, &pdata->connector);

	printk(KERN_INFO "%s -\n", __func__);
	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_host:
	//drm_connector_cleanup(&sn65dsi86->connector);
	return ret;
}

static int sn65dsi86_bridge_attach(struct drm_bridge *bridge)
{
	struct sn65dsi86_data *pdata = bridge_to_sn65dsi86(bridge);
	int ret;

	printk(KERN_INFO "%s +\n", __func__);

	//if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR) {
	//	DRM_ERROR("Fix bridge driver to make connector optional!");
	//	return -EINVAL;
	//}

	ret = drm_connector_init(bridge->dev, &pdata->connector,
				 &sn65dsi86_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);//DRM_MODE_CONNECTOR_eDP
	if (ret) {
		printk("sn65dsi86_bridge_attach: Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&pdata->connector,
				 &sn65dsi86_bridge_connector_helper_funcs);
	//drm_connector_attach_encoder(&pdata->connector, bridge->encoder);
	drm_mode_connector_attach_encoder(&pdata->connector, bridge->encoder);

	//ret = sn65dsi86_attach_dsi(pdata);

	printk(KERN_INFO "%s -\n", __func__);

	return ret;
}

static void sn65dsi86_bridge_disable(struct drm_bridge *bridge)
{
	struct sn65dsi86_data *pdata = bridge_to_sn65dsi86(bridge);

	printk(KERN_INFO "%s pdata->enabled =%d +\n", __func__, pdata->enabled);
	if (!pdata->enabled)
		return;

	//drm_panel_disable(pdata->panel);
	if (pdata->backlight) {
		pdata->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(pdata->backlight);
	}

	disable_irq(pdata->dsi86_irq);
	sn65dsi86_enable_irq(pdata, false);

	/* disable video stream */
	sn65dsi86_update_bit(pdata->client, SN_ENH_FRAME_REG, VSTREAM_ENABLE, 0);
	/* semi auto link training mode OFF */
	sn65dsi86_write(pdata->client, SN_ML_TX_MODE_REG, 0);
	/* disable DP PLL */
	sn65dsi86_write(pdata->client, SN_PLL_ENABLE_REG, 0);

	sn65dsi86_chip_shutdown(pdata);

	//drm_panel_unprepare(pdata->panel);
	edp_power_off(pdata);
	msleep(pdata->t12);
	pdata->enabled = false;

	printk(KERN_INFO "%s -\n", __func__);
}

static u32 sn65dsi86_bridge_get_dsi_freq(struct sn65dsi86_data *pdata)
{
	u32 bit_rate_khz, clk_freq_khz;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	bit_rate_khz = mode->clock *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	clk_freq_khz = bit_rate_khz / (pdata->dsi->lanes * 2);

	printk("sn65dsi86_bridge_get_dsi_freq clock=%d bit_rate_khz=%u  clk_freq_khz=%u \n", mode->clock, bit_rate_khz, clk_freq_khz);
	return clk_freq_khz;
}

/* clk frequencies supported by bridge in Hz in case derived from REFCLK pin */
static const u32 sn65dsi86_bridge_refclk_lut[] = {
	12000000,
	19200000,
	26000000,
	27000000,
	38400000,
};

/* clk frequencies supported by bridge in Hz in case derived from DACP/N pin */
static const u32 sn65dsi86_bridge_dsiclk_lut[] = {
	468000000,
	384000000,
	416000000,
	486000000,
	460800000,
};

static void sn65dsi86_bridge_set_refclk_freq(struct sn65dsi86_data *pdata)
{
	int i;
	u32 refclk_rate;
	const u32 *refclk_lut;
	size_t refclk_lut_size;

	if (pdata->refclk) {
		refclk_rate = clk_get_rate(pdata->refclk);
		refclk_lut = sn65dsi86_bridge_refclk_lut;
		refclk_lut_size = ARRAY_SIZE(sn65dsi86_bridge_refclk_lut);
		clk_prepare_enable(pdata->refclk);
	} else {
		refclk_rate = sn65dsi86_bridge_get_dsi_freq(pdata) * 1000;
		refclk_lut = sn65dsi86_bridge_dsiclk_lut;
		refclk_lut_size = ARRAY_SIZE(sn65dsi86_bridge_dsiclk_lut);
	}

	/* for i equals to refclk_lut_size means default frequency */
	for (i = 0; i < refclk_lut_size; i++)
		if (refclk_lut[i] == refclk_rate)
			break;
	printk("sn65dsi86_bridge_set_refclk_freq REFCLK_FREQ(REFCLK_FREQ_27M)=%x \n", REFCLK_FREQ(REFCLK_FREQ_27M));
	sn65dsi86_update_bit(pdata->client, SN_DPPLL_SRC_REG, REFCLK_FREQ_MASK,
			   REFCLK_FREQ(REFCLK_FREQ_27M)/*REFCLK_FREQ(i)*/);
}

static void sn65dsi86_bridge_set_dsi_rate(struct sn65dsi86_data *pdata)
{
	unsigned int bit_rate_mhz, clk_freq_mhz;
	unsigned int val;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/* set DSIA clk frequency */
	bit_rate_mhz = (mode->clock / 1000) *
			mipi_dsi_pixel_format_to_bpp(pdata->dsi->format);
	clk_freq_mhz = bit_rate_mhz / (pdata->dsi->lanes * 2);

	/* for each increment in val, frequency increases by 5MHz */
	if (clk_freq_mhz  >= MIN_DSI_CLK_FREQ_MHZ)
		val = clk_freq_mhz / DSI_CLK_FREQ_INCREMENT;
	else
		val = 0;

	printk("sn65dsi86_bridge_set_dsi_rate clock=%d bit_rate_mhz=%u  clk_freq_mhz=%u val=%x \n", mode->clock, bit_rate_mhz, clk_freq_mhz, val);

	sn65dsi86_write(pdata->client, SN_DSIA_CLK_FREQ_REG, val);
}


static unsigned int sn65dsi86_bridge_get_bpp(struct sn65dsi86_data *pdata)
{
	printk("sn65dsi86_bridge_get_bpp = %u \n", pdata->connector.display_info.bpc);

	if (pdata->connector.display_info.bpc <= 6)
		return 18;
	else
		return 24;
}

/*
 * LUT index corresponds to register value and
 * LUT values corresponds to dp data rate supported
 * by the bridge in Mbps unit.
 */
static const unsigned int sn65dsi86_bridge_dp_rate_lut[] = {
	0, 1620, 2160, 2430, 2700, 3240, 4320, 5400
};

static int sn65dsi86_bridge_calc_min_dp_rate_idx(struct sn65dsi86_data *pdata)
{
	unsigned int bit_rate_khz, dp_rate_mhz;
	unsigned int i;
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;

	/* Calculate minimum bit rate based on our pixel clock. */
	bit_rate_khz = mode->clock * sn65dsi86_bridge_get_bpp(pdata);

	/* Calculate minimum DP data rate, taking 80% as per DP spec */
	dp_rate_mhz = DIV_ROUND_UP(bit_rate_khz * DP_CLK_FUDGE_NUM,
				   1000 * pdata->dp_lanes * DP_CLK_FUDGE_DEN);

	for (i = 1; i < ARRAY_SIZE(sn65dsi86_bridge_dp_rate_lut) - 1; i++)
		if (sn65dsi86_bridge_dp_rate_lut[i] >= dp_rate_mhz)
			break;
	printk("sn65dsi86_bridge_calc_min_dp_rate_idx bit_rate_khz=%u dp_rate_mhz=%u  pdata->dp_lanes =%d i=%u\n", bit_rate_khz, dp_rate_mhz,pdata->dp_lanes , i);
	return i;
}

static void sn65dsi86_bridge_read_valid_rates(struct sn65dsi86_data *pdata,
					  bool rate_valid[])
{
	unsigned int rate_per_200khz;
	unsigned int rate_mhz;
	u8 dpcd_val;
	int ret;
	int i, j;

	ret = drm_dp_dpcd_readb(&pdata->aux, DP_EDP_DPCD_REV, &dpcd_val);
	if (ret != 1) {
		printk("sn65dsi86_bridge_read_valid_rates:Can't read eDP rev (%d), assuming 1.1\n", ret);
		dpcd_val = DP_EDP_11;
	}
	printk("sn65dsi86_bridge_read_valid_rates: eDP rev (%u)\n",dpcd_val);

	if (dpcd_val >= DP_EDP_14) {
		/* eDP 1.4 devices must provide a custom table */
		__le16 sink_rates[DP_MAX_SUPPORTED_RATES];

		ret = drm_dp_dpcd_read(&pdata->aux, DP_SUPPORTED_LINK_RATES,
				       sink_rates, sizeof(sink_rates));

		if (ret != sizeof(sink_rates)) {
			printk("sn65dsi86_bridge_read_valid_rates: Can't read supported rate table (%d)\n", ret);

			/* By zeroing we'll fall back to DP_MAX_LINK_RATE. */
			memset(sink_rates, 0, sizeof(sink_rates));
		}

		for (i = 0; i < ARRAY_SIZE(sink_rates); i++) {
			rate_per_200khz = le16_to_cpu(sink_rates[i]);

			if (!rate_per_200khz)
				break;

			rate_mhz = rate_per_200khz * 200 / 1000;
			for (j = 0;
			     j < ARRAY_SIZE(sn65dsi86_bridge_dp_rate_lut);
			     j++) {
				if (sn65dsi86_bridge_dp_rate_lut[j] == rate_mhz)
					rate_valid[j] = true;
			}
		}

		for (i = 0; i < ARRAY_SIZE(sn65dsi86_bridge_dp_rate_lut); i++) {
			if (rate_valid[i])
				return;
		}
		printk("sn65dsi86_bridge_read_valid_rates: No matching eDP rates in table; falling back\n");
	}

	/* On older versions best we can do is use DP_MAX_LINK_RATE */
	ret = drm_dp_dpcd_readb(&pdata->aux, DP_MAX_LINK_RATE, &dpcd_val);
	if (ret != 1) {
		printk("sn65dsi86_bridge_read_valid_rates: Can't read max rate (%d); assuming 5.4 GHz\n",
			      ret);
		dpcd_val = DP_LINK_BW_5_4;
	}

	printk("sn65dsi86_bridge_read_valid_rates: dpcd_val=0x%x\n", dpcd_val);

	switch (dpcd_val) {
	default:
		printk("sn65dsi86_bridge_read_valid_rates: Unexpected max rate (%#x); assuming 5.4 GHz\n",
			      (int)dpcd_val);
		//fallthrough;
	case DP_LINK_BW_5_4:
		rate_valid[7] = 1;
		//fallthrough;
	case DP_LINK_BW_2_7:
		rate_valid[4] = 1;
		//fallthrough;
	case DP_LINK_BW_1_62:
		rate_valid[1] = 1;
		break;
	}
}

static void sn65dsi86_bridge_set_video_timings(struct sn65dsi86_data *pdata)
{
	struct drm_display_mode *mode =
		&pdata->bridge.encoder->crtc->state->adjusted_mode;
	u8 hsync_polarity = 0, vsync_polarity = 0;

	printk("sn65dsi86_bridge_set_video_timings mode->flags=%x\n", mode->flags);
	printk("sn65dsi86_bridge_set_video_timings hdisplay=%d hsync_start=%d hsync_end=%d htotal=%d \n", mode->hdisplay, mode->hsync_start, mode->hsync_end, mode->htotal);
	printk("sn65dsi86_bridge_set_video_timings vdisplay=%d vsync_start=%d vsync_end=%d vtotal=%d \n", mode->vdisplay, mode->vsync_start, mode->vsync_end, mode->vtotal);
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		hsync_polarity = CHA_HSYNC_POLARITY;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vsync_polarity = CHA_VSYNC_POLARITY;

	sn65dsi86_write(pdata->client, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG,
			       mode->hdisplay & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_ACTIVE_LINE_LENGTH_LOW_REG + 1,
			       mode->hdisplay >> 8);
	sn65dsi86_write(pdata->client, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG,
			       mode->vdisplay & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_VERTICAL_DISPLAY_SIZE_LOW_REG + 1,
			       mode->vdisplay >> 8);
	sn65dsi86_write(pdata->client, SN_CHA_HSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->hsync_end - mode->hsync_start) & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_HSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->hsync_end - mode->hsync_start) >> 8) & 0x7F) |
		     hsync_polarity);
	sn65dsi86_write(pdata->client, SN_CHA_VSYNC_PULSE_WIDTH_LOW_REG,
		     (mode->vsync_end - mode->vsync_start) & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_VSYNC_PULSE_WIDTH_HIGH_REG,
		     (((mode->vsync_end - mode->vsync_start) >> 8) & 0x7F) |
		     vsync_polarity);

	sn65dsi86_write(pdata->client, SN_CHA_HORIZONTAL_BACK_PORCH_REG,
		     (mode->htotal - mode->hsync_end) & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_VERTICAL_BACK_PORCH_REG,
		     (mode->vtotal - mode->vsync_end) & 0xFF);

	sn65dsi86_write(pdata->client, SN_CHA_HORIZONTAL_FRONT_PORCH_REG,
		     (mode->hsync_start - mode->hdisplay) & 0xFF);
	sn65dsi86_write(pdata->client, SN_CHA_VERTICAL_FRONT_PORCH_REG,
		     (mode->vsync_start - mode->vdisplay) & 0xFF);

	usleep_range(10000, 10500); /* 10ms delay recommended by spec */
}

static unsigned int sn65dsi86_get_max_lanes(struct sn65dsi86_data *pdata)
{
	u8 data;
	int ret;

	ret = drm_dp_dpcd_readb(&pdata->aux, DP_MAX_LANE_COUNT, &data);
	if (ret != 1) {
		printk("error: sn65dsi86_get_max_lanes: Can't read lane count (%d); assuming 4\n", ret);
		return 4;
	}
	printk("sn65dsi86_get_max_lanes: ret=%d lane=%x\n", ret, data & DP_LANE_COUNT_MASK);
	return data & DP_LANE_COUNT_MASK;
}

static int sn65dsi86_link_training(struct sn65dsi86_data *pdata, int dp_rate_idx,
			       const char **last_err_str)
{
	uint8_t PLL_result = 0;
	uint8_t val;
	int ret;

	*last_err_str = "Link training successful";

	/* set dp clk frequency value */
	sn65dsi86_update_bit(pdata->client, SN_DATARATE_CONFIG_REG, DP_DATARATE_MASK, DP_DATARATE(dp_rate_idx));

	/* enable DP PLL */
	sn65dsi86_write(pdata->client, SN_PLL_ENABLE_REG, 1);
	msleep(10);
	sn65dsi86_read(pdata->client, 0x0A, &PLL_result);
	printk("sn65dsi86_link_training: PLL_result=0x%x (%s)\n", PLL_result, PLL_result & BIT(7) ? "DP PLL locked" : "DP PLL not locked");

	ret = sn65dsi86_read_poll_timeout(pdata->client, SN_DPPLL_SRC_REG, val,
				       val & DPPLL_SRC_DP_PLL_LOCK, 1000,
				       50 * 1000);
	if (ret) {
		*last_err_str = "DP_PLL_LOCK polling failed";
		goto exit;
	}

	/* Semi auto link training mode */
	sn65dsi86_write(pdata->client, SN_ML_TX_MODE_REG, 0x0A);
	ret = sn65dsi86_read_poll_timeout(pdata->client, SN_ML_TX_MODE_REG, val,
				       val == ML_TX_MAIN_LINK_OFF ||
				       val == ML_TX_NORMAL_MODE, 1000,
				       500 * 1000);
	if (ret) {
		*last_err_str = "Training complete polling failed";
	} else if (val == ML_TX_MAIN_LINK_OFF) {
		*last_err_str = "Link training failed, link is off";
		ret = -EIO;
	}

exit:
	/* Disable the PLL if we failed */
	if (ret)
		sn65dsi86_write(pdata->client, SN_PLL_ENABLE_REG, 0);

	return ret;
}

static void sn65dsi86_detailed_pixel_timing(unsigned char *buf, struct sn65dsi86_data *pdata, struct drm_display_mode *mode)
{
	unsigned hactive, vactive, hblank, vblank, hsync_offset, hsync_pulse_width, vsync_offset, vsync_pulse_width;
	unsigned int temp_flags = 0;
	struct detailed_pixel_timing {
		u8 hactive_lo;
		u8 hblank_lo;
		u8 hactive_hblank_hi;
		u8 vactive_lo;
		u8 vblank_lo;
		u8 vactive_vblank_hi;
		u8 hsync_offset_lo;
		u8 hsync_pulse_width_lo;
		u8 vsync_offset_pulse_width_lo;
		u8 hsync_vsync_offset_pulse_width_hi;
		u8 width_mm_lo;
		u8 height_mm_lo;
		u8 width_height_mm_hi;
		u8 hborder;
		u8 vborder;
		u8 misc;
	} __attribute__((packed));
	struct detailed_pixel_timing pt ;
	#define DETAILED_MODE_ADDR  (0x36)

	memcpy(&pt, buf + DETAILED_MODE_ADDR + 2, 16);
	hactive = (pt.hactive_hblank_hi & 0xf0) << 4 | pt.hactive_lo;
	vactive = (pt.vactive_vblank_hi & 0xf0) << 4 | pt.vactive_lo;
	hblank = (pt.hactive_hblank_hi & 0xf) << 8 | pt.hblank_lo;
	vblank = (pt.vactive_vblank_hi & 0xf) << 8 | pt.vblank_lo;
	hsync_offset = (pt.hsync_vsync_offset_pulse_width_hi & 0xc0) << 2 | pt.hsync_offset_lo;
	hsync_pulse_width = (pt.hsync_vsync_offset_pulse_width_hi & 0x30) << 4 | pt.hsync_pulse_width_lo;
	vsync_offset = (pt.hsync_vsync_offset_pulse_width_hi & 0xc) << 2 | pt.vsync_offset_pulse_width_lo >> 4;
	vsync_pulse_width = (pt.hsync_vsync_offset_pulse_width_hi & 0x3) << 4 | (pt.vsync_offset_pulse_width_lo & 0xf);

	mode->clock = (buf[DETAILED_MODE_ADDR] + (buf[DETAILED_MODE_ADDR+1]<<8)) * 10;

	mode->hdisplay = hactive;
	mode->hsync_start = mode->hdisplay + hsync_offset;
	mode->hsync_end = mode->hsync_start + hsync_pulse_width;
	mode->htotal = mode->hdisplay + hblank;

	mode->vdisplay = vactive;
	mode->vsync_start = mode->vdisplay + vsync_offset;
	mode->vsync_end = mode->vsync_start + vsync_pulse_width;
	mode->vtotal = mode->vdisplay + vblank;
	mode->width_mm = pt.width_mm_lo | (pt.width_height_mm_hi & 0xf0) << 4;
	mode->height_mm = pt.height_mm_lo | (pt.width_height_mm_hi & 0xf) << 8;


	mode->flags |= (pt.misc & DRM_EDID_PT_HSYNC_POSITIVE) ?
		DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
	mode->flags |= (pt.misc & DRM_EDID_PT_VSYNC_POSITIVE) ?
		DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;

	mode->width_mm = pt.width_mm_lo | (pt.width_height_mm_hi & 0xf0) << 4;
	mode->height_mm = pt.height_mm_lo | (pt.width_height_mm_hi & 0xf) << 8;
	mode->vscan = 0;
	mode->vrefresh = drm_mode_vrefresh(mode);
	mode->flags |= (pt.misc & DRM_EDID_PT_HSYNC_POSITIVE) ?
		DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
	mode->flags |= (pt.misc & DRM_EDID_PT_VSYNC_POSITIVE) ?
		DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;

	printk("Pixel Clock %dKHz\n", mode->clock);
	printk("Resolution: %dx%d\n", mode->hdisplay, mode->vdisplay);
	printk("Refresh rate = %d\n", mode->vrefresh);
	printk("hsync_start =%d hsync_end = %d htotal = %d\n", mode->hsync_start, mode->hsync_end, mode->htotal);
	printk("vsync_start =%d vsync_end = %d vtotal = %d\n", mode->vsync_start, mode->vsync_end, mode->vtotal);
	printk("HFP =%d HSYNC = %d HBP= %d\n", mode->hsync_start - mode->hdisplay, mode->hsync_end - mode->hsync_start, mode->htotal - mode->hsync_end);
	printk("VFP =%d VSYNC = %d VBP= %d\n", mode->vsync_start - mode->vdisplay, mode->vsync_end - mode->vsync_start, mode->vtotal - mode->vsync_end);
	printk("hsync  Polarity = %s\n", (pt.misc & DRM_EDID_PT_HSYNC_POSITIVE) ? "Positive" : "Negative");
	printk("vsync  Polarity = %s\n", (pt.misc & DRM_EDID_PT_VSYNC_POSITIVE) ? "Positive" : "Negative");
	printk("Active Area: %dx%d mm\n", mode->width_mm, mode->height_mm);

	pdata->vm.pixelclock = mode->clock*1000;
	pdata->vm.hactive = mode->hdisplay;
	pdata->vm.vactive = mode->vdisplay;

	temp_flags |= (pt.misc & DRM_EDID_PT_HSYNC_POSITIVE) ?
		DISPLAY_FLAGS_HSYNC_HIGH : DISPLAY_FLAGS_HSYNC_LOW;
	temp_flags |= (pt.misc & DRM_EDID_PT_VSYNC_POSITIVE) ?
		DISPLAY_FLAGS_VSYNC_HIGH : DISPLAY_FLAGS_VSYNC_LOW;
	temp_flags |= (pdata->vm.flags & 0xF0);
	pdata->vm.flags = temp_flags;
	pdata->vm.hfront_porch = mode->hsync_start - mode->hdisplay;
	pdata->vm.hsync_len = mode->hsync_end - mode->hsync_start;
	pdata->vm.hback_porch = mode->htotal - mode->hsync_end;
	pdata->vm.vfront_porch = mode->vsync_start - mode->vdisplay;
	pdata->vm.vsync_len = mode->vsync_end - mode->vsync_start;
	pdata->vm.vback_porch = mode->vtotal - mode->vsync_end;
}

static void showDPCDInfo(struct sn65dsi86_data *pdata)
{
	#define ASSR_SUPPORT				(1<<0)
	#define ENHANCE_FRAMING			(1<<1)
	#define DPCD_DISPLAY_CONTORL_CAP   (1<<3)
	uint8_t buf[16];

	drm_dp_dpcd_read(&pdata->aux, 0, buf, sizeof(buf));

	printk("DPCD: REV:%d.%d, MAX_LINK_RATE:", (buf[0] >> 4), (buf[0]&0xF));
	if (buf[1] == 0x06) {
		printk("1.62Gbps");
	} else if (buf[1] == 0x0A) {
		printk("2.7Gbps");
	}
	printk(" MAX_LINK_LANE:%d\n", buf[2] & DP_LANE_COUNT_MASK);
	if (buf[0x0D] & ASSR_SUPPORT) {
		printk(" support ASSR");
	} else {
		printk(" not support ASSR");
	}
	if (buf[0x0D] & ENHANCE_FRAMING) {
		printk(" support Enhance framing");
	} else {
		printk(" not support Enhance framing");
	}
	printk("\n");
}

#ifdef DIRECT_EDID_METHOD
static int sn65dsi86_direct_read_edid(struct sn65dsi86_data *pdata)
{
	unsigned char buf[EDID_SIZE];
	unsigned char offset = 0;
	int r, retries;
	printk(KERN_INFO "%s\n", __func__);
	sn65dsi86_write(pdata->client, SN_I2C_ADDR_CLAIM1, DDC_ADDR << 1);
	sn65dsi86_update_bit(pdata->client, SN_I2C_ADDR_CLAIM1, I2C_CLAM1_EN_MASK, I2C_CLAM1_EN);

	for (retries = 3; retries > 0; retries--) {
		struct i2c_msg msgs[] = {
			{
				.addr   = DDC_ADDR,
				.flags  = 0,
				.len    = 1,
				.buf    = &offset,
			}, {
				.addr   =DDC_ADDR,
				.flags  = I2C_M_RD,
				.len    = EDID_SIZE,
				.buf    = buf,
			}
		};

		r = i2c_transfer(pdata->client->adapter, msgs, 2);
		if (r == 2)
			goto end;

		if (r != -EAGAIN)
			break;
	}
end:
	sn65dsi86_update_bit(pdata->client, SN_I2C_ADDR_CLAIM1, I2C_CLAM1_EN_MASK, 0);

	if (r == 2){
		int i = 0,  j = 0;
		u8 csum = 0;

		for (j = 0; j < (EDID_SIZE/16); j++) {
			printk("sn65dsi86_direct_read_edid edid 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", buf[i+0], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7], buf[i+8], buf[i+9], buf[i+10], buf[i+11], buf[i+12], buf[i+13], buf[i+14], buf[i+15]);
			i += 16;
		}

		for (i = 0;  i < EDID_SIZE;  i++)
			csum += buf[i];

		printk("sn65dsi86_direct_read_edid checksum =%x\n", csum);
		if (csum == 0)
			sn65dsi86_detailed_pixel_timing(buf, pdata, &pdata->mode);
	}

	return r < 0 ? r : -EIO;
}
#endif

void sn65dsi86_bridge_enable(struct drm_bridge *bridge)
{
	struct sn65dsi86_data *pdata = bridge_to_sn65dsi86(bridge);
	bool rate_valid[ARRAY_SIZE(sn65dsi86_bridge_dp_rate_lut)] = { };
	const char *last_err_str = "No supported DP rate";
	int dp_rate_idx;
	unsigned int val;
	int ret = -EINVAL;
	int max_dp_lanes;
	uint8_t training_result = 0, PLL_result = 0;

	printk(KERN_INFO "%s  pdata->enabled = %d+\n", __func__, pdata->enabled);

	if (pdata->enabled)
		return;

	pm_runtime_get_sync(pdata->dev);

if (0) {
	edp_power_on(pdata);
	msleep(pdata->t3);
	sn65dsi86_chip_enable(pdata);

#ifdef PWM_FROM_SN65DSI86
	sn65dsi86_gpio4_to_pwm(pdata, true);
	sn65dsi86_set_pwm_freq(pdata);
#endif

	//======REFCLK Frequency  ======
	sn65dsi86_write(pdata->client, 0x0A,0x6);

	sn65dsi86_update_bit(pdata->client, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	showDPCDInfo(pdata);
	//======DSI Mode  ======
	sn65dsi86_write(pdata->client, 0x10, 0x26);

	//======DSIA Clock  ======
	sn65dsi86_write(pdata->client, 0x12, 0x55);

	//======DSIB Clock  ======
	sn65dsi86_write(pdata->client, 0x13, 0x55);

	//======DP Datarate  ======
	sn65dsi86_write(pdata->client, 0x94, 0x80);

	//======Enable PLL  ======
	sn65dsi86_write(pdata->client, 0x0D, 0x01);
	msleep(10);
	sn65dsi86_read(pdata->client, 0x0A, &PLL_result);
	printk("sn65dsi86_bridge_enable: PLL_result=0x%x (%s)\n", PLL_result, PLL_result & BIT(7) ? "DP PLL locked" : "DP PLL not locked");

	//======Enable ASSR in Panel  ======
	drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
			   DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
	msleep(10);

	//======Enable enhanced frame and ASSR in DSI86  ======
	sn65dsi86_write(pdata->client, 0x5A, 0x05);

	//======Number of DP lanes  ======
	sn65dsi86_write(pdata->client, 0x93, 0x24);

      //======Start Semi-Auto Link Training  =====
	msleep(pdata->t4);
	sn65dsi86_write(pdata->client, 0x96, 0x0A);
	msleep(20);
	sn65dsi86_read(pdata->client, 0x96, &training_result);
	printk("sn65dsi86_bridge_enable: sn65dsi86_link_training: training_result=%d(%s)\n",
		training_result, training_result == 0x1 ? "normal mode" : (training_result == 0x0 ? "main link off" : "unknow error"));

	 sn65dsi86_read(pdata->client, 0xF8, &training_result);
	 printk("sn65dsi86_bridge_enable: sn65dsi86_link_training: training status =0x%x(%s)\n", training_result, training_result == 0x1 ? "LT_PASS" : "LT_FAIL");

	//======CHA Active Line Length  ======
	sn65dsi86_write(pdata->client, 0x20, 0x80);
	sn65dsi86_write(pdata->client, 0x21, 0x07);

	//======CHB Active Line Length  ======
	sn65dsi86_write(pdata->client, 0x22, 0x0);
	sn65dsi86_write(pdata->client, 0x23, 0x0);

	//======Vertical Active Size   ======
	sn65dsi86_write(pdata->client, 0x24, 0x38);
	sn65dsi86_write(pdata->client, 0x25, 0x04);

	//======Horizontal Pulse Width   ======
	sn65dsi86_write(pdata->client, 0x2C, 48);
	sn65dsi86_write(pdata->client, 0x2D, 0x00);//polarity

	//======Vertical Pulse Width   ======
	sn65dsi86_write(pdata->client, 0x30, 10);
	sn65dsi86_write(pdata->client, 0x31, 0x00);//polarity

	//======HBP   ======
	sn65dsi86_write(pdata->client, 0x34, 24);

	//======VBP   ======
	sn65dsi86_write(pdata->client, 0x36, 26);

	//===== HFP  ======
	sn65dsi86_write(pdata->client, 0x38, 108);

	//===== VFP  ======
	sn65dsi86_write(pdata->client, 0x3A, 10);

	//===== DP-18BPP Disable  ======
	sn65dsi86_write(pdata->client, 0x5B, 0x0);

	//===== Color Bar Enable  ======
	sn65dsi86_write(pdata->client, 0x3C, 0/*0x12*/);

	//===== Enhanced Frame, ASSR, and Vstream Enable  ======
	sn65dsi86_write(pdata->client, 0x5A, 0x0d);

	if (pdata->backlight) {
		msleep(pdata->t8);
		pdata->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(pdata->backlight);
	}

	sn65dsi86_dump_status_register(pdata);
	pdata->enabled = true;
	sn65dsi86_enable_irq(pdata, true);
	enable_irq(pdata->dsi86_irq);
} else {
	edp_power_on(pdata);
	msleep(pdata->t3);
	sn65dsi86_chip_enable(pdata);

	/* configure bridge ref_clk */
	sn65dsi86_bridge_set_refclk_freq(pdata);

	/*
	 * HPD on this bridge chip is a bit useless.  This is an eDP bridge
	 * so the HPD is an internal signal that's only there to signal that
	 * the panel is done powering up.  ...but the bridge chip debounces
	 * this signal by between 100 ms and 400 ms (depending on process,
	 * voltage, and temperate--I measured it at about 200 ms).  One
	 * particular panel asserted HPD 84 ms after it was powered on meaning
	 * that we saw HPD 284 ms after power on.  ...but the same panel said
	 * that instead of looking at HPD you could just hardcode a delay of
	 * 200 ms.  We'll assume that the panel driver will have the hardcoded
	 * delay in its prepare and always disable HPD.
	 *
	 * If HPD somehow makes sense on some future panel we'll have to
	 * change this to be conditional on someone specifying that HPD should
	 * be used.
	 */
	sn65dsi86_update_bit(pdata->client, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);
	showDPCDInfo(pdata);

	max_dp_lanes = sn65dsi86_get_max_lanes(pdata);
	pdata->dp_lanes = min(pdata->dp_lanes, max_dp_lanes);
	printk(KERN_INFO "%s pdata->dp_lanes=%d\n", __func__, pdata->dp_lanes);
	/* DSI_A lane config */
	val = CHA_DSI_LANES(SN_MAX_DP_LANES - pdata->dsi->lanes);
	sn65dsi86_update_bit(pdata->client, SN_DSI_LANES_REG,
			   CHA_DSI_LANES_MASK, val);

	sn65dsi86_write(pdata->client, SN_LN_ASSIGN_REG, pdata->ln_assign);
	sn65dsi86_update_bit(pdata->client, SN_ENH_FRAME_REG, LN_POLRS_MASK,
			   pdata->ln_polrs << LN_POLRS_OFFSET);

	/* set dsi clk frequency value */
	sn65dsi86_bridge_set_dsi_rate(pdata);

	/**
	 * The SN65DSI86 only supports ASSR Display Authentication method and
	 * this method is enabled by default. An eDP panel must support this
	 * authentication method. We need to enable this method in the eDP panel
	 * at DisplayPort address 0x0010A prior to link training.
	 */
	drm_dp_dpcd_writeb(&pdata->aux, DP_EDP_CONFIGURATION_SET,
			   DP_ALTERNATE_SCRAMBLER_RESET_ENABLE);
	msleep(10);

	//======Enable enhanced frame and ASSR in DSI86  ======
	sn65dsi86_update_bit(pdata->client, SN_ENH_FRAME_REG, AUTHEN_METHOD_MASK,
			   SCRAMBLER_SEED_RESET);
	sn65dsi86_update_bit(pdata->client, SN_ENH_FRAME_REG, ENH_FRAME_ENABLE,
			   ENH_FRAME_ENABLE);


	/* Set the DP output format (18 bpp or 24 bpp) */
	val = (sn65dsi86_bridge_get_bpp(pdata) == 18) ? BPP_18_RGB : 0;
	sn65dsi86_update_bit(pdata->client, SN_DATA_FORMAT_REG, BPP_18_RGB, val);

	/* DP lane config */
	val = DP_NUM_LANES(min(pdata->dp_lanes, 3));
	sn65dsi86_update_bit(pdata->client, SN_SSC_CONFIG_REG, DP_NUM_LANES_MASK,
			   val);

	sn65dsi86_bridge_read_valid_rates(pdata, rate_valid);

	/* Train until we run out of rates */
	msleep(pdata->t4);
	for (dp_rate_idx = sn65dsi86_bridge_calc_min_dp_rate_idx(pdata);
	     dp_rate_idx < ARRAY_SIZE(sn65dsi86_bridge_dp_rate_lut);
	     dp_rate_idx++) {
		if (!rate_valid[dp_rate_idx])
			continue;

		ret = sn65dsi86_link_training(pdata, dp_rate_idx, &last_err_str);
		if (!ret)
			break;
	}
	if (ret) {
		printk("sn65dsi86_bridge_enable: %s (%d)\n", last_err_str, ret);
		//return;
	} else {
		printk("sn65dsi86_bridge_enable dp_rate_idx=%d\n", dp_rate_idx);
		msleep(20);
		sn65dsi86_read(pdata->client, SN_ML_TX_MODE_REG, &training_result);
		printk("sn65dsi86_bridge_enable: sn65dsi86_link_training: training_result=%d(%s)\n",
			training_result, training_result == 0x1 ? "normal mode" : (training_result == 0x0 ? "main link off" : "unknow error"));

		sn65dsi86_read(pdata->client, SN_IRQ_STATUS8, &training_result);
		printk("sn65dsi86_bridge_enable: sn65dsi86_link_training: training status =0x%x(%s)\n", training_result, training_result == 0x1 ? "LT_PASS" : "LT_FAIL");
	}

	/* config video parameters */
	sn65dsi86_bridge_set_video_timings(pdata);

	//edp_power_on(pdata);
	//msleep(pdata->t2);//T1+T2 >0.5ms

	/* enable video stream */
	sn65dsi86_update_bit(pdata->client, SN_ENH_FRAME_REG, VSTREAM_ENABLE,
			   VSTREAM_ENABLE);

	if (pdata->backlight) {
		pdata->backlight->props.power = FB_BLANK_UNBLANK;
		msleep(pdata->t8);
		backlight_update_status(pdata->backlight);
	}

	//drm_panel_enable(pdata->panel);
	pdata->enabled = true;

	sn65dsi86_dump_status_register(pdata);
	sn65dsi86_enable_irq(pdata, true);
	enable_irq(pdata->dsi86_irq);
}

	printk(KERN_INFO "%s -\n", __func__);
}

static void sn65dsi86_bridge_pre_enable(struct drm_bridge *bridge)
{
#if 0
	struct sn65dsi86_data *pdata = bridge_to_sn65dsi86(bridge);

	printk(KERN_INFO "%s +\n", __func__);


	pm_runtime_get_sync(pdata->dev);

	/* configure bridge ref_clk */
	sn65dsi86_bridge_set_refclk_freq(pdata);

	/*
	 * HPD on this bridge chip is a bit useless.  This is an eDP bridge
	 * so the HPD is an internal signal that's only there to signal that
	 * the panel is done powering up.  ...but the bridge chip debounces
	 * this signal by between 100 ms and 400 ms (depending on process,
	 * voltage, and temperate--I measured it at about 200 ms).  One
	 * particular panel asserted HPD 84 ms after it was powered on meaning
	 * that we saw HPD 284 ms after power on.  ...but the same panel said
	 * that instead of looking at HPD you could just hardcode a delay of
	 * 200 ms.  We'll assume that the panel driver will have the hardcoded
	 * delay in its prepare and always disable HPD.
	 *
	 * If HPD somehow makes sense on some future panel we'll have to
	 * change this to be conditional on someone specifying that HPD should
	 * be used.
	 */
	sn65dsi86_update_bit(pdata->client, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);

	edp_power_on(pdata);
	msleep(200);//edp t3
#else
	printk(KERN_INFO "%s +\n", __func__);
#endif
	//drm_panel_prepare(pdata->panel);
	printk(KERN_INFO "%s -\n", __func__);
}

static void sn65dsi86_bridge_post_disable(struct drm_bridge *bridge)
{
	struct sn65dsi86_data *pdata = bridge_to_sn65dsi86(bridge);

	printk(KERN_INFO "%s +\n", __func__);

	if (pdata->refclk)
		clk_disable_unprepare(pdata->refclk);

	pm_runtime_put_sync(pdata->dev);

	printk(KERN_INFO "%s -\n", __func__);
}

static const struct drm_bridge_funcs sn65dsi86_bridge_funcs = {
	.attach = sn65dsi86_bridge_attach,
	.pre_enable = sn65dsi86_bridge_pre_enable,
	.enable = sn65dsi86_bridge_enable,
	.disable = sn65dsi86_bridge_disable,
	.post_disable = sn65dsi86_bridge_post_disable,
};

static struct sn65dsi86_data *aux_to_sn65dsi86_bridge(struct drm_dp_aux *aux)
{
	return container_of(aux, struct sn65dsi86_data, aux);
}

static ssize_t sn65dsi86_aux_transfer(struct drm_dp_aux *aux,
				  struct drm_dp_aux_msg *msg)
{
	struct sn65dsi86_data *pdata = aux_to_sn65dsi86_bridge(aux);
	u32 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 request_val = AUX_CMD_REQ(msg->request);
	u8 *buf = (u8 *)msg->buffer;
	int retry = 0, ret, i;
	uint8_t val;

	if (msg->size > SN_AUX_MAX_PAYLOAD_BYTES)
		return -EINVAL;

	switch (request) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
		break;
	default:
		return -EINVAL;
	}

	sn65dsi86_write(pdata->client, SN_AUX_ADDR_19_16_REG,
		     (msg->address >> 16) & 0xF);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_15_8_REG,
		     (msg->address >> 8) & 0xFF);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, msg->address & 0xFF);

	sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, msg->size);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE) {
		for (i = 0; i < msg->size; i++)
			sn65dsi86_write(pdata->client, SN_AUX_WDATA_REG(i),
				     buf[i]);
	}

	/* Clear old status bits before start so we don't get confused */
	sn65dsi86_write(pdata->client, SN_AUX_CMD_STATUS_REG,
			ML_TX_NORMAL_MODE |
			AUX_IRQ_STATUS_NAT_I2C_FAIL |
			AUX_IRQ_STATUS_AUX_RPLY_TOUT |
			AUX_IRQ_STATUS_AUX_SHORT);

	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);

	ret = sn65dsi86_read_poll_timeout(pdata->client, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 200,
				       50 * 1000);
	if (ret)
		return ret;

	do {
		ret = sn65dsi86_read(pdata->client, SN_AUX_CMD_STATUS_REG, &val);
		if (ret)
			return ret;
		else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
			|| (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
			|| (val & AUX_IRQ_STATUS_AUX_SHORT)) {
			printk("sn65dsi86_aux_transfer: return -ENXIO val=%x\n", val);
			return -ENXIO;
		}
		msleep(1);
	}while(!(val & ML_TX_NORMAL_MODE) && retry++ < 20);

	if (request == DP_AUX_NATIVE_WRITE || request == DP_AUX_I2C_WRITE)
		return msg->size;

	for (i = 0; i < msg->size; i++) {
		uint8_t val;
		ret = sn65dsi86_read(pdata->client, SN_AUX_RDATA_REG(i), &val);
		if (ret)
			return ret;

		buf[i] = (u8)(val & 0xFF);
	}

	return msg->size;
}

#ifndef DIRECT_EDID_METHOD
static ssize_t sn65dsi86_send_aux_cmd(struct sn65dsi86_data *pdata, u32 request_val)
{
	int retry = 0, ret;
	uint8_t val;

	sn65dsi86_write(pdata->client, SN_AUX_CMD_STATUS_REG,
			ML_TX_NORMAL_MODE |
			AUX_IRQ_STATUS_NAT_I2C_FAIL |
			AUX_IRQ_STATUS_AUX_RPLY_TOUT |
			AUX_IRQ_STATUS_AUX_SHORT);
	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val | AUX_CMD_SEND);
	ret = sn65dsi86_read_poll_timeout(pdata->client, SN_AUX_CMD_REG, val,
				       !(val & AUX_CMD_SEND), 200,
				       50 * 1000);
	if (ret) {
		printk("sn65dsi86_send_aux_cmd: send cmd fail, val=%x\n", val);
	}

	do {
		ret = sn65dsi86_read(pdata->client, SN_AUX_CMD_STATUS_REG, &val);
		if (ret)
			return ret;
		else if ((val & AUX_IRQ_STATUS_NAT_I2C_FAIL)
			|| (val & AUX_IRQ_STATUS_AUX_RPLY_TOUT)
			|| (val & AUX_IRQ_STATUS_AUX_SHORT)) {
			printk("sn65dsi86_send_aux_cmd: return -ENXIO val=%x\n", val);
			return -ENXIO;
		}
		msleep(1);
	}while(!(val & ML_TX_NORMAL_MODE) && retry++ < 20);

	return 0;
}

static ssize_t sn65dsi86_read_edid(struct sn65dsi86_data *pdata)
{
	#define AUX_CMD(x)  (x<<4)
	u8 buf[EDID_SIZE] = {0}, csum = 0;
	u32 request_val;
	int ret, i = 0, j = 0, index = 0;

	printk("sn65dsi86_read_edid+\n");
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_19_16_REG, 0);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_15_8_REG, 0);

	//step 1
	request_val = AUX_CMD(0x04);
	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);


	//step 5
	request_val = AUX_CMD(0x04);
	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, 1);
	sn65dsi86_write(pdata->client, SN_AUX_WDATA_REG(0), 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);

	//step 9
	request_val = AUX_CMD(0x05);
	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);


	//step 13
	do {
		request_val = AUX_CMD(0x05);
		sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
		sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, 0x50);
		sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, 0x10);
		sn65dsi86_send_aux_cmd(pdata, request_val);
		for (i = 0; i < 16; i++) {
			uint8_t val;
			ret = sn65dsi86_read(pdata->client, SN_AUX_RDATA_REG(i), &val);
			//if (ret)
			//	return ret;

			buf[index++] = (u8)(val & 0xFF);
		}
	}while(index < EDID_SIZE);

	//step 18
	request_val = AUX_CMD(0x01);
	sn65dsi86_write(pdata->client, SN_AUX_CMD_REG, request_val);
	sn65dsi86_write(pdata->client, SN_AUX_ADDR_7_0_REG, 0x50);
	sn65dsi86_write(pdata->client, SN_AUX_LENGTH_REG, 0);
	sn65dsi86_send_aux_cmd(pdata, request_val);

	i = 0;
	for (j = 0; j < (EDID_SIZE/16); j++) {
		printk("sn65dsi86_read_edid 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", buf[i+0], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7], buf[i+8], buf[i+9], buf[i+10], buf[i+11], buf[i+12], buf[i+13], buf[i+14], buf[i+15]);
		i += 16;
	}

	for (i = 0;  i < EDID_SIZE;  i++)
		csum += buf[i];

	printk("sn65dsi86_read_edid checksum =%x -\n", csum);

	if (csum == 0)
		sn65dsi86_detailed_pixel_timing(buf, pdata, &pdata->mode);

	return index;
}
#endif

static int sn65dsi86_bridge_parse_dsi_host(struct sn65dsi86_data *pdata)
{
	struct device_node *np = pdata->dev->of_node;
	struct device_node *endpoint;

#if 0
	pdata->host_node = of_graph_get_remote_node(np, 0, 0);
#else
	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint) {
		printk("error: sn65dsi86_bridge_parse_dsi_hostcan not get next_endpoint\n");
		return -ENODEV;
	} else {
		printk("sn65dsi86_bridge_parse_dsi_host  endpoint->name=%s type=%s full_name=%s\n", endpoint->name, endpoint->type, endpoint->full_name);
	}
	pdata->host_node = of_graph_get_remote_port_parent(endpoint);
	if (!pdata->host_node) {
		of_node_put(endpoint);
		return -ENODEV;
	}else {
		printk("sn65dsi86_bridge_parse_dsi_host endpoint->name=%s type=%s full_name=%s\n", pdata->host_node->name, pdata->host_node->type, pdata->host_node->full_name);
	}
#endif
	if (!pdata->host_node) {
		printk("sn65dsi86_bridge_parse_dsi_host: remote dsi host node not found\n");
		return -ENODEV;
	}

	of_node_put(endpoint);
	of_node_put(pdata->host_node);

	return 0;
}

static void sn65dsi86_bridge_parse_lanes(struct sn65dsi86_data *pdata,
				     struct device_node *np)
{
	u32 lane_assignments[SN_MAX_DP_LANES] = { 0, 1, 2, 3 };
	u32 lane_polarities[SN_MAX_DP_LANES] = { };
	u8 ln_assign = 0;
	u8 ln_polrs = 0;
	int dp_lanes;
	int i;

	/*
	 * Read config from the device tree about lane remapping and lane
	 * polarities.  These are optional and we assume identity map and
	 * normal polarity if nothing is specified.  It's OK to specify just
	 * data-lanes but not lane-polarities but not vice versa.
	 *
	 * Error checking is light (we just make sure we don't crash or
	 * buffer overrun) and we assume dts is well formed and specifying
	 * mappings that the hardware supports.
	 */
	dp_lanes = of_property_count_u32_elems(np, "data-lanes");
	if (dp_lanes > 0 && dp_lanes <= SN_MAX_DP_LANES) {
		of_property_read_u32_array(np, "data-lanes",
					   lane_assignments, dp_lanes);
		of_property_read_u32_array(np, "lane-polarities",
					   lane_polarities, dp_lanes);
	} else {
		dp_lanes = SN_MAX_DP_LANES;
	}
	of_node_put(np);

	/*
	 * Convert into register format.  Loop over all lanes even if
	 * data-lanes had fewer elements so that we nicely initialize
	 * the LN_ASSIGN register.
	 */
	for (i = SN_MAX_DP_LANES - 1; i >= 0; i--) {
		ln_assign = ln_assign << LN_ASSIGN_WIDTH | lane_assignments[i];
		ln_polrs = ln_polrs << 1 | lane_polarities[i];
	}

	/* Stash in our struct for when we power on */
	pdata->dp_lanes = dp_lanes;
	pdata->ln_assign = ln_assign;
	pdata->ln_polrs = ln_polrs;

	printk("sn65dsi86_bridge_parse_lanes dp_lanes=%d ln_assign=0x%x ln_polrs=0x%x\n", dp_lanes, ln_assign, ln_polrs);
}

#ifdef PWM_FROM_SN65DSI86
static void sn65dsi86_gpio4_to_pwm(struct sn65dsi86_data *pdata, bool enable)
{
	int val;

	val = enable ? SN_GPIO4_MUX_PWM : SN_GPIO_MUX_INPUT;
	sn65dsi86_update_bit(pdata->client, SN_GPIO_CTRL_REG,
				 SN_GPIO_MUX_MASK << SN_GPIO4_CTL_OFFSET,
				 val << SN_GPIO4_CTL_OFFSET);
}

static void sn65dsi86_set_pwm_freq(struct sn65dsi86_data *pdata)
{
	/*REFCLK=27MHZ, 27MHZ/(1* 25500) =  1058.2HZ, 25500=0x639c*/
	sn65dsi86_write(pdata->client, SN_PWM_PRE_DIV,1);
	sn65dsi86_write(pdata->client, SN_BACKLIGHT_SCAL_LOW , 0x639c & 0xFF);
	sn65dsi86_write(pdata->client, SN_BACKLIGHT_SCAL_HIGH , (0x639c >> 8) & 0xFF);
}

static void sn65dsi86_pwm_enable(struct sn65dsi86_data *pdata, bool enable)
{
	int val;

	val = enable ? 1 : 0;
	printk("sn65dsi86_pwm_enable enable=%d\n", enable);
	if (enable) {
		if (pdata->dsi86_vbl_en_gpio) {
			gpiod_set_value_cansleep(pdata->dsi86_vbl_en_gpio, 1);
			msleep(pdata->t14);
		}
		sn65dsi86_update_bit(pdata->client, SN_PWM_EN,
			PWM_EN_MASK << PWM_EN_OFFSET,
			val << PWM_EN_OFFSET);
		msleep(pdata->t15);
	} else {
		msleep(pdata->t16);
		sn65dsi86_update_bit(pdata->client, SN_PWM_EN,
			PWM_EN_MASK << PWM_EN_OFFSET,
			val << PWM_EN_OFFSET);
		if (pdata->dsi86_vbl_en_gpio) {
			gpiod_set_value_cansleep(pdata->dsi86_vbl_en_gpio, 0);
			msleep(pdata->t17);
		}
	}
}

static void sn65dsi86_pwm_level(struct sn65dsi86_data *pdata, int brightness)
{
	brightness = brightness * 100;
	printk("sn65dsi86_pwm_level  brightness=%d\n", brightness);
	sn65dsi86_write(pdata->client, SN_BACKLIGHT_LOW , brightness & 0xFF);
	sn65dsi86_write(pdata->client, SN_BACKLIGHT_HIGH , (brightness >> 8) & 0xFF);
}

static unsigned int sn65dsi86_get_pwm_level(struct sn65dsi86_data *pdata)
{
	uint8_t low, high;

	sn65dsi86_read(pdata->client, SN_BACKLIGHT_LOW , &low);
	sn65dsi86_read(pdata->client, SN_BACKLIGHT_HIGH , &high);
	return ((high << 8) | low) /100;
}

 int sn65dsi86_bl_update_status(struct backlight_device * bd)
 {
	int brightness = bd->props.brightness;

	printk("sn65dsi86_bl_update_status brightness=%d power=%d fb_blank=%d\n", brightness, bd->props.power, bd->props.fb_blank);
	if (brightness > MAX_BRIGHENESS)
		brightness = MAX_BRIGHENESS;

	/*minimal PWM duty cyclc of auo is 5%, 255*0.05 is about 13*/
	if (brightness < 13)
		brightness = 13;

	if (bd->props.power== FB_BLANK_POWERDOWN)
		brightness = 0;

	if (bd->props.fb_blank == FB_BLANK_POWERDOWN)
		brightness = 0;

	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	if (brightness > 0) {
		sn65dsi86_pwm_level(gdata, brightness);
		sn65dsi86_pwm_enable(gdata, true);
	} else {
		sn65dsi86_pwm_enable(gdata, false);
	}

	return 0;
 }

static int sn65dsi86_bl_get_brightness(struct backlight_device *bd)
{
	return sn65dsi86_get_pwm_level(gdata);
}

static const struct backlight_ops sn65dsi86_bl_ops = {
	.get_brightness	= sn65dsi86_bl_get_brightness,//actual_brightness_show
	.update_status	= sn65dsi86_bl_update_status,
	.options 			= BL_CORE_SUSPENDRESUME,
};
#endif

static const struct display_timing asus_sn65dsi86_default_mode = {
	.pixelclock = { 141000000, 141000000, 141000000 },
	.hactive = { 1920, 1920, 1920},
	.hfront_porch = { 60, 60, 60 },
	.hsync_len = { 42, 42, 42 },
	.hback_porch = {60, 60, 60 },
	.vactive = { 1080, 1080, 1080 },
	.vfront_porch = { 20, 20, 20 },
	.vsync_len = { 8, 8, 8 },
	.vback_porch = { 20, 20, 20 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
			DISPLAY_FLAGS_VSYNC_LOW |
			DISPLAY_FLAGS_DE_HIGH |
			DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int sn65dsi86_parse_dt(struct device_node *np,
			   struct sn65dsi86_data *data)
{
	#ifndef PWM_FROM_SN65DSI86
	struct device_node *backlight;
	#endif
	struct device_node *endpoint;
	struct device *dev = data->dev;
	int ret;

	printk(KERN_INFO "%s +\n", __func__);

	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (!endpoint) {
		dev_err(dev, "error:sn65dsi86_parse_dt can not get next_endpoint\n");
		return -ENODEV;
	} else {
		printk("sn65dsi86_parse_dt  endpoint->name=%s type=%s full_name=%s\n", endpoint->name, endpoint->type, endpoint->full_name);
	}
	data->host_node = of_graph_get_remote_port_parent(endpoint);
	if (!data->host_node) {
		of_node_put(endpoint);
		return -ENODEV;
	}else {
		printk("sn65dsi86_parse_dt  endpoint->name=%s type=%s full_name=%s\n", data->host_node->name, data->host_node->type, data->host_node->full_name);
	}

	ret = of_property_read_u32(np, "dsi-lanes", &data->dsi_lanes);
	if (data->dsi_lanes < 1 || data->dsi_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", data->dsi_lanes);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "edp-width-mm", &data->width_mm);
	ret = of_property_read_u32(np, "edp-height-mm", &data->height_mm);
	data->test_pattern_en = of_property_read_bool(np, "test-pattern");

	ret = of_property_read_u32(dev->of_node, "bus-format", &data->bus_format);
	ret = of_property_read_u32(dev->of_node, "bpc", &data->bpc);
	ret = of_property_read_u32(dev->of_node, "dsi,flags", &data->mode_flags);
	ret = of_property_read_u32(dev->of_node, "dsi,format", &data->format);
	ret = of_property_read_u32(dev->of_node, "dsi,lanes", &data->dsi_lanes);

	ret = of_property_read_u32(dev->of_node,"t1", &data->t1);
	ret = of_property_read_u32(dev->of_node,"t2", &data->t2);
	ret = of_property_read_u32(dev->of_node,"t3", &data->t3);
	ret = of_property_read_u32(dev->of_node,"t4", &data->t4);
	ret = of_property_read_u32(dev->of_node,"t5", &data->t5);
	ret = of_property_read_u32(dev->of_node,"t6", &data->t6);
	ret = of_property_read_u32(dev->of_node,"t7", &data->t7);
	ret = of_property_read_u32(dev->of_node,"t8", &data->t8);
	ret = of_property_read_u32(dev->of_node,"t12", &data->t12);
	ret = of_property_read_u32(dev->of_node,"t14", &data->t14);
	ret = of_property_read_u32(dev->of_node,"t15", &data->t15);
	ret = of_property_read_u32(dev->of_node,"t16", &data->t16);
	ret = of_property_read_u32(dev->of_node,"t17", &data->t17);

	ret = of_get_videomode(np, &data->vm, 0);
	printk(KERN_INFO "sn65dsi86_parse_dt pixelclock=%lu hactive=%u vactive=%u vm.flags=%x \n",
		data->vm.pixelclock, data->vm.hactive, data->vm.vactive, data->vm.flags);
	printk(KERN_INFO "sn65dsi86_parse_dt hfront_porch=%u .hsync_len=%u hback_porch=%u \n",
		data->vm.hfront_porch, data->vm.hsync_len, data->vm.hback_porch);
	printk(KERN_INFO "sn65dsi86_parse_dt vfront_porch=%u vsync_len=%u vback_porch=%u \n",
		data->vm.vfront_porch, data->vm.vsync_len, data->vm.vback_porch);
	printk(KERN_INFO "sn65dsi86_parse_dt bus_format=%x data->bpc=%u format =%u mode_flags=%u\n",
		data->bus_format, data->bpc, data->format, data->mode_flags);
	printk(KERN_INFO "sn65dsi86_parse_dt t1=%u t2=%u t3=%u t4=%u t5=%u t6=%u t7=%u t8=%u t12=%u\n",
		data->t1, data->t2, data->t3, data->t4, data->t5, data->t6, data->t7, data->t8, data->t12);

	if (ret < 0) {
		videomode_from_timing(&asus_sn65dsi86_default_mode, &data->vm);
	}

	data->sn65dsi86_en_gpio = devm_gpiod_get_optional(dev, "EN",  GPIOD_OUT_LOW);
	if (IS_ERR(data->sn65dsi86_en_gpio)) {
		printk(KERN_INFO "sn65dsi86_parse_dt :failed to get EN GPIO \n");
	}

	data->edp_vdd_en_gpio = devm_gpiod_get_optional(dev, "edp_vdd_en", GPIOD_OUT_LOW);
	if (IS_ERR(data->edp_vdd_en_gpio)) {
		printk(KERN_INFO "sn65dsi86_parse_dt :data->edp_vdd_en_gpio\n");
	}

#ifdef PWM_FROM_SN65DSI86
	data->dsi86_vbl_en_gpio = devm_gpiod_get_optional(dev, "dsi86_vbl_en", GPIOD_OUT_LOW);
	if (IS_ERR(data->dsi86_vbl_en_gpio)) {
		printk(KERN_INFO "sn65dsi86_parse_dt :data->dsi86_vbl_en_gpio\n");
	}
#endif

	//data->dsi86_irq_gpio = of_get_named_gpio_flags(np, "dsi86_irq", 0, (enum of_gpio_flags *)&irq_flags);
	//printk(KERN_INFO "sn65dsi86_parse_dt  dsi86_irq_gpio=%u\n", data->dsi86_irq_gpio);
	data->dsi86_irq_gpio = devm_gpiod_get_optional(dev, "dsi86_irq", GPIOD_IN);
	if (IS_ERR(data->dsi86_irq_gpio)) {
		printk(KERN_INFO "sn65dsi86_parse_dt :data->dsi86_irq_gpio\n");
	}

#ifndef PWM_FROM_SN65DSI86
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		data->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!data->backlight) {
			printk(KERN_ERR "sn65dsi86_parse_dt: failed to find backlight\n");
		}
	} else {
		printk(KERN_ERR "sn65dsi86_parse_dt: failed to parse backlight handle\n");
	}
#endif

	printk(KERN_INFO "sn65dsi86_parse_dt dsi_lanes=%u width_mm=%u height_mm=%u\n",
		   data->dsi_lanes, data->width_mm, data->height_mm);
	printk(KERN_INFO "sn65dsi86_parse_dt  test_pattern_en=%u\n", data->test_pattern_en);
	printk(KERN_INFO "%s -\n", __func__);

	of_node_put(endpoint);
	of_node_put(data->host_node);
	return ret;
}

static irqreturn_t sn65dsi86_irq(int irq, void *dev)
{
	struct sn65dsi86_data *sn65dsi86 = (struct sn65dsi86_data *)dev;\

	disable_irq_nosync(sn65dsi86->dsi86_irq);// Disable ts interrupt

	if (!work_pending(&sn65dsi86->work))
		queue_work(sn65dsi86->wq, &sn65dsi86->work);

	return IRQ_HANDLED;
}

static void sn65dsi86_dump_status_register(struct sn65dsi86_data *sn65dsi86)
{
	uint8_t val = 0xFF;

	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS0, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS0 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS1, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS1 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS2, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS2 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS3, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS3 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS4, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS4 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS5, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS5 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS6, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS6 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS7, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS7 = %x\n", val);
	sn65dsi86_read(sn65dsi86->client, SN_IRQ_STATUS8, &val);
	printk(KERN_ERR "sn65dsi86_dump_status_register: SN_IRQ_STATUS8 = %x\n", val);

	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS0, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS1, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS2, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS3, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS4, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS5, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS6, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS7, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_STATUS8, 0xFF);
}

static void sn65dsi86_irq_worker(struct work_struct *work)
{
	struct sn65dsi86_data *sn65dsi86 = container_of(work, struct sn65dsi86_data, work);

	printk(KERN_ERR "sn65dsi86_irq_worker\n");
	sn65dsi86_dump_status_register(sn65dsi86);
	enable_irq(sn65dsi86->dsi86_irq);
}

void sn65dsi86_enable_irq(struct sn65dsi86_data *sn65dsi86, bool enable)
{
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_1, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_2, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_3, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_4, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_5, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_6, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_7, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_8, 0xFF);
	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN_9, 0xFF);

	sn65dsi86_write(sn65dsi86->client, SN_IRQ_EN, enable ? 1 : 0);
}

static ssize_t sn65dsi86_reg_show(struct device *dev, struct device_attribute *attr, char *buf2)
{
	struct sn65dsi86_data *sn65dsi86 = dev_get_drvdata(dev);

	sn65dsi86_dump_status_register(sn65dsi86);

	return 0;
}

static ssize_t sn65dsi86_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sn65dsi86_data *sn65dsi86 = dev_get_drvdata(dev);
	unsigned val;
	int ret;
	char *endp;
	unsigned reg = simple_strtol(buf, &endp, 16);

	if (reg > 0xe5)
		return count;

	if (!endp)
		return count;

	printk("%s: %c\n", __func__, *endp);
	val = simple_strtol(endp+1, &endp, 16);
	if (val >= 0x100)
		return count;

	printk("%s:reg=0x%x, val=0x%x\n", __func__, reg, val);
	ret = sn65dsi86_write(sn65dsi86->client, reg, val);
	if (ret < 0)
		return ret;

	return strnlen(buf, count);
}

static DEVICE_ATTR(sn65dsi86_reg, S_IRUGO | S_IWUSR, sn65dsi86_reg_show, sn65dsi86_reg_store);

static struct attribute *sn65dsi86_attributes[] = {
	&dev_attr_sn65dsi86_reg.attr,
	NULL
};

static const struct attribute_group sn65dsi86_attr_group = {
	.attrs = sn65dsi86_attributes,
};

static void sn65dsi86_proch_from_edid(struct sn65dsi86_data *sn65dsi86)
{
	edp_power_on(sn65dsi86);
	msleep(sn65dsi86->t3);
	sn65dsi86_chip_enable(sn65dsi86);

	//======REFCLK Frequency  ======
	sn65dsi86_write(sn65dsi86->client, 0x0A,0x6);
	sn65dsi86_update_bit(sn65dsi86->client, SN_HPD_DISABLE_REG, HPD_DISABLE,
			   HPD_DISABLE);

	#ifdef DIRECT_EDID_METHOD
	sn65dsi86_direct_read_edid(sn65dsi86);
	#else
	sn65dsi86_read_edid(sn65dsi86);
	#endif

	edp_power_off(sn65dsi86);
	sn65dsi86_chip_shutdown(sn65dsi86);
}

static int sn65dsi86_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct sn65dsi86_data *sn65dsi86;
	#ifdef PWM_FROM_SN65DSI86
	struct backlight_properties props;
	#endif
	int ret;

	printk(KERN_INFO "%s +\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("sn65dsi86_probet: device doesn't support I2C\n");
		return -ENODEV;
	}

	sn65dsi86 = devm_kzalloc(&client->dev, sizeof(struct sn65dsi86_data), GFP_KERNEL);
	if (!sn65dsi86)
		return -ENOMEM;

	client->dev.driver_data = sn65dsi86;
	sn65dsi86->dev = &client->dev;
	sn65dsi86->client = client;
	sn65dsi86->enabled = false;
	sn65dsi86->powered = false;
	sn65dsi86->status = connector_status_disconnected;
	gdata = sn65dsi86;

	ret = sn65dsi86_parse_dt(client->dev.of_node, sn65dsi86);
	if (ret)
		return ret;
	//ret = drm_of_find_panel_or_bridge(sn65dsi86->dev->of_node, 1, 0,
	//				  &sn65dsi86->panel, NULL);
	//if (ret) {
	//	DRM_ERROR("could not find any panel node\n");
	//	return ret;
	//}

	dev_set_drvdata(&client->dev, sn65dsi86);

	sn65dsi86_chip_shutdown(sn65dsi86);
	sn65dsi86_chip_enable(sn65dsi86);
	sn65dsi86_detect(sn65dsi86);
	if (sn65dsi86->status == connector_status_connected) {
		printk(KERN_INFO "%s : sn65dsi86 is %s !\n", __func__, sn65dsi86->status ? "connected" : "disconnected");
		sn65dsi86_exist  = true;
	}

	sn65dsi86_bridge_parse_lanes(sn65dsi86, client->dev.of_node);

	sn65dsi86->refclk = devm_clk_get(sn65dsi86->dev, "refclk");
	if (IS_ERR(sn65dsi86->refclk)) {
		ret = PTR_ERR(sn65dsi86->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		printk("sn65dsi86_probet: refclk not found\n");
		sn65dsi86->refclk = NULL;
	}

	ret = sn65dsi86_bridge_parse_dsi_host(sn65dsi86);
	if (ret)
		return ret;

	pm_runtime_enable(sn65dsi86->dev);

	i2c_set_clientdata(client, sn65dsi86);

	sn65dsi86->aux.name = "ti-sn65dsi86-aux";
	sn65dsi86->aux.dev = sn65dsi86->dev;
	sn65dsi86->aux.transfer = sn65dsi86_aux_transfer;
	drm_dp_aux_register(&sn65dsi86->aux);

	sn65dsi86->bridge.funcs = &sn65dsi86_bridge_funcs;
	sn65dsi86->bridge.of_node = client->dev.of_node;

	drm_bridge_add(&sn65dsi86->bridge);

	sn65dsi86_attach_dsi(sn65dsi86);

	sn65dsi86_debugfs_init(sn65dsi86);

#ifdef PWM_FROM_SN65DSI86
	sn65dsi86_gpio4_to_pwm(sn65dsi86, true);
	sn65dsi86_set_pwm_freq(sn65dsi86);
	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHENESS;
	sn65dsi86->backlight = backlight_device_register("edp_backlight", NULL, NULL,
					   &sn65dsi86_bl_ops, &props);
#endif

	INIT_WORK(&sn65dsi86->work, sn65dsi86_irq_worker);
	sn65dsi86->wq = create_singlethread_workqueue("sn65dsi86_irq_wq");
	if (!sn65dsi86->wq) {
		printk(KERN_ERR  "error: sn65dsi86_probe could not create workqueue\n");
	}

	/* IRQ config*/
	//sn65dsi86->dsi86_irq = gpio_to_irq(sn65dsi86->dsi84_irq_gpio);
	sn65dsi86->dsi86_irq = gpiod_to_irq(sn65dsi86->dsi86_irq_gpio);
	/* Init irq */
	ret = request_irq(sn65dsi86->dsi86_irq, sn65dsi86_irq, IRQF_TRIGGER_RISING, DRIVER_NAME, sn65dsi86);
	if ( ret ) {
		printk(KERN_ERR  "error: sn65dsi86_probe, unable to request irq for device %s.\n", DRIVER_NAME);
	}

	disable_irq(sn65dsi86->dsi86_irq);
	sn65dsi86_enable_irq(sn65dsi86, false);
	sn65dsi86_proch_from_edid(sn65dsi86);
	sn65dsi86_chip_shutdown(sn65dsi86);

	 ret = sysfs_create_group(&client->dev.kobj, &sn65dsi86_attr_group);

	printk(KERN_INFO "%s -\n", __func__);
	return 0;
}

static int sn65dsi86_remove(struct i2c_client *client)
{
	struct sn65dsi86_data *sn65dsi86 = i2c_get_clientdata(client);

	if (!sn65dsi86)
		return -EINVAL;

	sn65dsi86_debugfs_remove(sn65dsi86);

	of_node_put(sn65dsi86->host_node);

	pm_runtime_disable(sn65dsi86->dev);

	if (sn65dsi86->dsi) {
		mipi_dsi_detach(sn65dsi86->dsi);
		mipi_dsi_device_unregister(sn65dsi86->dsi);
	}

	drm_bridge_remove(&sn65dsi86->bridge);

	return 0;
}

static void  sn65dsi86_shutdown(struct i2c_client *i2c)
{
	//struct sn65dsi86_data *sn65dsi86 = i2c_get_clientdata(i2c);

	//sn65dsi86_bridge_disable(&sn65dsi86->bridge);

	return;
}

static const struct i2c_device_id sn65dsi86_i2c_ids[] = {
	{ "sn65dsi86"},
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi86_i2c_ids);

static const struct of_device_id sn65dsi86_of_ids[] = {
	{ .compatible = "asus,sn65dsi86"},
	{ }
};
MODULE_DEVICE_TABLE(of, sn65dsi86_of_ids);

static struct mipi_dsi_driver sn65dsi86_dsi_driver = {
	.driver.name = "sn65dsi86",
};

static struct i2c_driver sn65dsi86_driver = {
	.driver = {
		.name = "sn65dsi86",
		.of_match_table = sn65dsi86_of_ids,
	},
	.id_table = sn65dsi86_i2c_ids,
	.probe = sn65dsi86_probe,
	.remove = sn65dsi86_remove,
	.shutdown = sn65dsi86_shutdown,
};

static int __init sn65dsi86_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_register(&sn65dsi86_dsi_driver);

	return i2c_add_driver(&sn65dsi86_driver);
}
module_init(sn65dsi86_init);

static void __exit sn65dsi86_exit(void)
{
	i2c_del_driver(&sn65dsi86_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&sn65dsi86_dsi_driver);
}
module_exit(sn65dsi86_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("sn65dsi86 mipi to edp driver");
MODULE_LICENSE("GPL");
