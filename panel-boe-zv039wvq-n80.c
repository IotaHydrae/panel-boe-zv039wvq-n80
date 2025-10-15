// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for panels based on BOE ZV039WVQ-N80, such as:
 *
 * - GoldenMorning T397B5-C24-02 3.97" MIPI-DSI panel
 *
 * Copyright (C) 2025, embeddedboys
 * Author: Wooden Chair <hua.zheng@embeddedboys.com>
 *
 * Based on drivers/gpu/drm/panel/panel-sitronix-st7703.c
 * Copyright (C) Purism SPC 2019
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#define DRV_NAME "panel-sitronix-st7701sn"

/* Command2 BKx selection command */
#define ST7701_CMD2BKX_SEL 0xFF
#define ST7701_CMD1 0
#define ST7701_CMD2 BIT(4)
#define ST7701_CMD2BK_MASK GENMASK(3, 0)

/* Command2, BK0 commands */
#define ST7701_CMD2_BK0_PVGAMCTRL 0xB0 /* Positive Voltage Gamma Control */
#define ST7701_CMD2_BK0_NVGAMCTRL 0xB1 /* Negative Voltage Gamma Control */
#define ST7701_CMD2_BK0_LNESET 0xC0 /* Display Line setting */
#define ST7701_CMD2_BK0_PORCTRL 0xC1 /* Porch control */
#define ST7701_CMD2_BK0_INVSEL \
	0xC2 /* Inversion selection, Frame Rate Control */

/* Command2, BK1 commands */
#define ST7701_CMD2_BK1_VRHS 0xB0 /* Vop amplitude setting */
#define ST7701_CMD2_BK1_VCOM 0xB1 /* VCOM amplitude setting */
#define ST7701_CMD2_BK1_VGHSS 0xB2 /* VGH Voltage setting */
#define ST7701_CMD2_BK1_TESTCMD 0xB3 /* TEST Command Setting */
#define ST7701_CMD2_BK1_VGLS 0xB5 /* VGL Voltage setting */
#define ST7701_CMD2_BK1_PWCTLR1 0xB7 /* Power Control 1 */
#define ST7701_CMD2_BK1_PWCTLR2 0xB8 /* Power Control 2 */
#define ST7701_CMD2_BK1_SPD1 0xC1 /* Source pre_drive timing set1 */
#define ST7701_CMD2_BK1_SPD2 0xC2 /* Source EQ2 Setting */
#define ST7701_CMD2_BK1_MIPISET1 0xD0 /* MIPI Setting 1 */

/* Command2, BK0 bytes */
#define ST7701_CMD2_BK0_GAMCTRL_AJ_MASK GENMASK(7, 6)
#define ST7701_CMD2_BK0_GAMCTRL_VC0_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC4_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC8_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC16_MASK GENMASK(4, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC24_MASK GENMASK(4, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC52_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC80_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC108_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC147_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC175_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC203_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC231_MASK GENMASK(4, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC239_MASK GENMASK(4, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC247_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC251_MASK GENMASK(5, 0)
#define ST7701_CMD2_BK0_GAMCTRL_VC255_MASK GENMASK(4, 0)
#define ST7701_CMD2_BK0_LNESET_LINE_MASK GENMASK(6, 0)
#define ST7701_CMD2_BK0_LNESET_LDE_EN BIT(7)
#define ST7701_CMD2_BK0_LNESET_LINEDELTA GENMASK(1, 0)
#define ST7701_CMD2_BK0_PORCTRL_VBP_MASK GENMASK(7, 0)
#define ST7701_CMD2_BK0_PORCTRL_VFP_MASK GENMASK(7, 0)
#define ST7701_CMD2_BK0_INVSEL_ONES_MASK GENMASK(5, 4)
#define ST7701_CMD2_BK0_INVSEL_NLINV_MASK GENMASK(2, 0)
#define ST7701_CMD2_BK0_INVSEL_RTNI_MASK GENMASK(4, 0)

/* Command2, BK1 bytes */
#define ST7701_CMD2_BK1_VRHA_MASK GENMASK(7, 0)
#define ST7701_CMD2_BK1_VCOM_MASK GENMASK(7, 0)
#define ST7701_CMD2_BK1_VGHSS_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK1_TESTCMD_VAL BIT(7)
#define ST7701_CMD2_BK1_VGLS_ONES BIT(6)
#define ST7701_CMD2_BK1_VGLS_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK1_PWRCTRL1_AP_MASK GENMASK(7, 6)
#define ST7701_CMD2_BK1_PWRCTRL1_APIS_MASK GENMASK(3, 2)
#define ST7701_CMD2_BK1_PWRCTRL1_APOS_MASK GENMASK(1, 0)
#define ST7701_CMD2_BK1_PWRCTRL2_AVDD_MASK GENMASK(5, 4)
#define ST7701_CMD2_BK1_PWRCTRL2_AVCL_MASK GENMASK(1, 0)
#define ST7701_CMD2_BK1_SPD1_ONES_MASK GENMASK(6, 4)
#define ST7701_CMD2_BK1_SPD1_T2D_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK1_SPD2_ONES_MASK GENMASK(6, 4)
#define ST7701_CMD2_BK1_SPD2_T3D_MASK GENMASK(3, 0)
#define ST7701_CMD2_BK1_MIPISET1_ONES BIT(7)
#define ST7701_CMD2_BK1_MIPISET1_EOT_EN BIT(3)

#define CMD_TB1 false
#define CMD_TB2 true

#define CFIELD_PREP(_mask, _val) \
	(((typeof(_mask))(_val) << (__builtin_ffsll(_mask) - 1)) & (_mask))

enum op_bias { OP_BIAS_OFF = 0, OP_BIAS_MIN, OP_BIAS_MIDDLE, OP_BIAS_MAX };

struct st7701sn {
	struct device *dev;
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator *vci;
	struct regulator *iovcc;
	enum drm_panel_orientation orientation;

	const struct st7701sn_panel_desc *desc;
};

struct st7701sn_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;

	/* TFT matrix driver configuration, panel specific. */
	const u8 pv_gamma[16]; /* Positive voltage gamma control */
	const u8 nv_gamma[16]; /* Negative voltage gamma control */
	const u8 nlinv; /* Inversion selection */
	const u32 vop_uv; /* Vop in uV */
	const u32 vcom_uv; /* Vcom in uV */
	const u16 vgh_mv; /* Vgh in mV */
	const s16 vgl_mv; /* Vgl in mV */
	const u16 avdd_mv; /* Avdd in mV */
	const s16 avcl_mv; /* Avcl in mV */
	const enum op_bias gamma_op_bias;
	const enum op_bias input_op_bias;
	const enum op_bias output_op_bias;
	const u16 t2d_ns; /* T2D in ns */
	const u16 t3d_ns; /* T3D in ns */
	const bool eot_en;

	int (*init_seq)(struct st7701sn *ctx);
};

static inline struct st7701sn *panel_to_st7701sn(struct drm_panel *panel)
{
	return container_of(panel, struct st7701sn, panel);
}

#define ST7701SN_WRITE(st7701sn, cmd, seq...)                             \
	{                                                                 \
		const u8 d[] = { seq };                                   \
		mipi_dsi_dcs_write(st7701sn->dsi, cmd, d, ARRAY_SIZE(d)); \
	}

static void st7701sn_switch_cmd_bkx(struct st7701sn *ctx, bool cmd2, u8 bkx)
{
	u8 val;

	if (cmd2)
		val = ST7701_CMD2 | FIELD_PREP(ST7701_CMD2BK_MASK, bkx);
	else
		val = ST7701_CMD1;

	ST7701SN_WRITE(ctx, ST7701_CMD2BKX_SEL, 0x77, 0x01, 0x00, 0x00, val);
}

static int t397b5_c24_02_init_seq(struct st7701sn *ctx)
{
	const struct st7701sn_panel_desc *desc = ctx->desc;
	const struct drm_display_mode *mode = desc->mode;
	const u8 linecount8 = mode->vdisplay / 8;
	const u8 linecountrem2 = (mode->vdisplay % 8) / 2;

	ST7701SN_WRITE(ctx, MIPI_DCS_SOFT_RESET, 0x00);
	msleep(5);

	ST7701SN_WRITE(ctx, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);
	msleep(120);

	/* Command 2, BK0 */
	st7701sn_switch_cmd_bkx(ctx, true, 0);

	ST7701SN_WRITE(ctx, 0xB0, 0x40, 0x0E, 0x51, 0x0F, 0x11, 0x07, 0x00,
		       0x09, 0x06, 0x1E, 0x04, 0x12, 0x11, 0x64, 0x29, 0xDF);
	ST7701SN_WRITE(ctx, 0xB1, 0x40, 0x07, 0x4C, 0x0A, 0x0E, 0x04, 0x00,
		       0x08, 0x09, 0x1D, 0x01, 0x0E, 0x0C, 0x6A, 0x34, 0xDF);

	ST7701SN_WRITE(
		ctx, ST7701_CMD2_BK0_LNESET,
		FIELD_PREP(ST7701_CMD2_BK0_LNESET_LINE_MASK, linecount8 - 1) |
			(linecountrem2 ? ST7701_CMD2_BK0_LNESET_LDE_EN : 0),
		FIELD_PREP(ST7701_CMD2_BK0_LNESET_LINEDELTA, linecountrem2));

	/* Porch Control,VBP,VFP */
	ST7701SN_WRITE(ctx, ST7701_CMD2_BK0_PORCTRL,
		       FIELD_PREP(ST7701_CMD2_BK0_PORCTRL_VBP_MASK,
				  mode->vtotal - mode->vsync_end),
		       FIELD_PREP(ST7701_CMD2_BK0_PORCTRL_VFP_MASK,
				  mode->vsync_start - mode->vdisplay));

	/* Inversion selection & Frame Rate Control */
	/*
	 * Horizontal pixel count configuration:
	 * PCLK = 512 + (RTNI[4:0] * 16)
	 * The PCLK is number of pixel clock per line, which matches
	 * mode htotal. The minimum is 512 PCLK.
	 */
	ST7701SN_WRITE(
		ctx, ST7701_CMD2_BK0_INVSEL,
		ST7701_CMD2_BK0_INVSEL_ONES_MASK |
			FIELD_PREP(ST7701_CMD2_BK0_INVSEL_NLINV_MASK,
				   desc->nlinv),
		FIELD_PREP(ST7701_CMD2_BK0_INVSEL_RTNI_MASK,
			   (clamp((u32)mode->htotal, 512U, 1008U) - 512) / 16));

	/* Command2, BK1 */
	st7701sn_switch_cmd_bkx(ctx, CMD_TB2, 1);

	ST7701SN_WRITE(ctx, 0xB0, 0x30); // Vop amplitude setting
	ST7701SN_WRITE(ctx, 0xB1, 0x48); // VCOM amplitude setting
	ST7701SN_WRITE(ctx, 0xB2, 0x80); // VGH voltage setting

	ST7701SN_WRITE(ctx, ST7701_CMD2_BK1_TESTCMD,
		       ST7701_CMD2_BK1_TESTCMD_VAL);

	ST7701SN_WRITE(ctx, 0xB5, 0x4F); // VGL Voltage setting
	ST7701SN_WRITE(ctx, 0xB7, 0x85); // Power Control 1
	ST7701SN_WRITE(ctx, 0xB8, 0x23); // Power Control 2
	ST7701SN_WRITE(ctx, 0xB9, 0x22, 0x13); // Power Control 3
	ST7701SN_WRITE(ctx, 0xBB,
		       0x03); // PCLKS2 (BK1): Power pumping clk selection 2
	ST7701SN_WRITE(ctx, 0xBC,
		       0x10); // PCLKS3 (BK1): Power pumping clk selection 3

	ST7701SN_WRITE(ctx, 0xC0, 0x89);
	ST7701SN_WRITE(ctx, 0xC1,
		       0x78); // SPD1(BK1): Source pre_drive timing set1
	ST7701SN_WRITE(ctx, 0xC2, 0x78); // SPD2(BK1): Source EQ2 Setting

	ST7701SN_WRITE(ctx, ST7701_CMD2_BK1_MIPISET1,
		       ST7701_CMD2_BK1_MIPISET1_ONES |
			       (desc->eot_en ? ST7701_CMD2_BK1_MIPISET1_EOT_EN :
					       0));
	/* End of generic settings */

	st7701sn_switch_cmd_bkx(ctx, true, 3);
	ST7701SN_WRITE(ctx, 0xEF, 0x08);

	st7701sn_switch_cmd_bkx(ctx, true, 0);
	ST7701SN_WRITE(ctx, 0xCC, 0x18);

	// ST7701SN_WRITE(ctx, 0xEF, 0x08, 0x08, 0x08, 0x4C, 0x3F, 0x54);

	/* MIPI lane delay setting */
	// ST7701SN_WRITE(ctx, 0xD3, 0x11, 0x00);
	st7701sn_switch_cmd_bkx(ctx, true, 1);
	ST7701SN_WRITE(ctx, 0xE0, 0x00, 0x00, 0x02);
	ST7701SN_WRITE(ctx, 0xE1, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00,
		       0x00, 0x00, 0x10, 0x10);
	ST7701SN_WRITE(ctx, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		       0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701SN_WRITE(ctx, 0xE3, 0x00, 0x00, 0x33, 0x00);
	ST7701SN_WRITE(ctx, 0xE4, 0x22, 0x00);
	ST7701SN_WRITE(ctx, 0xE5, 0x03, 0x34, 0xAF, 0xB3, 0x05, 0x34, 0xAF,
		       0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701SN_WRITE(ctx, 0xE6, 0x00, 0x00, 0x33, 0x00);
	ST7701SN_WRITE(ctx, 0xE7, 0x22, 0x00);
	ST7701SN_WRITE(ctx, 0xE8, 0x04, 0x34, 0xAF, 0xB3, 0x06, 0x34, 0xAF,
		       0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	ST7701SN_WRITE(ctx, 0xEB, 0x02, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00);
	ST7701SN_WRITE(ctx, 0xEC, 0x00, 0x00);
	ST7701SN_WRITE(ctx, 0xED, 0xFA, 0x45, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF,
		       0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB0, 0x54, 0xAF);
	ST7701SN_WRITE(ctx, 0xEF, 0x08, 0x08, 0x08, 0x45, 0x3F, 0x54);

	st7701sn_switch_cmd_bkx(ctx, CMD_TB1, 0);

	st7701sn_switch_cmd_bkx(ctx, CMD_TB2, 3);
	// ST7701SN_WRITE(ctx, 0xE6, 0x16, 0x7C);
	ST7701SN_WRITE(ctx, 0xE6, 0x16);
	ST7701SN_WRITE(ctx, 0xE8, 0x00, 0x0E);

	st7701sn_switch_cmd_bkx(ctx, CMD_TB1, 0);
	ST7701SN_WRITE(ctx, 0x34); // Tearing effect OFF
	// ST7701SN_WRITE(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
	ST7701SN_WRITE(ctx, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);
	msleep(120);

	st7701sn_switch_cmd_bkx(ctx, CMD_TB2, 3);
	ST7701SN_WRITE(ctx, 0xE8, 0x00, 0x0C);
	msleep(10);
	ST7701SN_WRITE(ctx, 0xE8, 0x00, 0x00);
	st7701sn_switch_cmd_bkx(ctx, CMD_TB1, 0);

	return 0;
}

static const struct drm_display_mode t397b5_c24_02_mode = {
	.hdisplay = 480,
	.hsync_start = 480 + 80,
	.hsync_end = 480 + 80 + 60,
	.htotal = 480 + 80 + 60 + 8,

	// .vdisplay = 800,
	// .vsync_start = 800 + 7,
	// .vsync_end = 800 + 7 + 12,
	// .vtotal = 800 + 7 + 12 + 5,

	.vdisplay = 800,
	.vsync_start = 800 + 7,
	.vsync_end = 800 + 7 + 12,
	.vtotal = 800 + 7 + 12 + 10,

	.width_mm = 52,
	.height_mm = 86,

	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,

	.clock = 31000,
};

static const struct st7701sn_panel_desc t397b5_c24_02_desc = {
	.mode = &t397b5_c24_02_mode,
	.lanes = 2,
	.nlinv = 0,
	.eot_en = 1,

	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS,

	.format = MIPI_DSI_FMT_RGB888,
	.init_seq = t397b5_c24_02_init_seq,
};

static int st7701sn_prepare(struct drm_panel *panel)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(20);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(150);

	ctx->desc->init_seq(ctx);

	return 0;
}

static int st7701sn_enable(struct drm_panel *panel)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);

	ST7701SN_WRITE(ctx, MIPI_DCS_SET_DISPLAY_ON, 0x00);
	return 0;
}

static int st7701sn_disable(struct drm_panel *panel)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);

	ST7701SN_WRITE(ctx, MIPI_DCS_SET_DISPLAY_OFF, 0x00);
	return 0;
}

static int st7701sn_unprepare(struct drm_panel *panel)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(120);

	return 0;
}

static int st7701sn_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		// dev_err(ctx->dev, "Failed to add mode %ux%u@%u\n",
		// 	ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
		// 	drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static enum drm_panel_orientation
st7701sn_get_orientation(struct drm_panel *panel)
{
	struct st7701sn *ctx = panel_to_st7701sn(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs st7701sn_funcs = {
	.prepare = st7701sn_prepare,
	.enable = st7701sn_enable,
	.disable = st7701sn_disable,
	.unprepare = st7701sn_unprepare,
	.get_modes = st7701sn_get_modes,
	.get_orientation = st7701sn_get_orientation,
};

static void st7701sn_cleanup(void *data)
{
	struct st7701sn *ctx = data;

	drm_panel_remove(&ctx->panel);
	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
}

static int st7701sn_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct st7701sn *ctx;
	int ret;

	// printk("%s\n", __func__);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	// if (IS_ERR(ctx->reset_gpio))
	// 	return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
	// 			     "Failed to get reset GPIO");

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0) {
		// dev_err(dev, "%pOF: failed to get orientation %d\n",
		// 	dev->of_node, ret);
		return ret;
	}

	devm_add_action_or_reset(dev, st7701sn_cleanup, ctx);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->dsi = dsi;
	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->format = ctx->desc->format;
	dsi->lanes = ctx->desc->lanes;

	/* TODO: Regulator initialization */

	drm_panel_init(&ctx->panel, dev, &st7701sn_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		// dev_err_probe(dev, ret, "mipi_dsi_attach failed\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dev_dbg(dev, "%ux%u@%u %ubpp dsi %udl - ready\n",
		ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
		drm_mode_vrefresh(ctx->desc->mode),
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	return 0;
}

static void st7701sn_remove(struct mipi_dsi_device *dsi)
{
	mipi_dsi_detach(dsi);
}

static const struct of_device_id st7701sn_of_match[] = {
	{ .compatible = "goldenmorning,t397b5", .data = &t397b5_c24_02_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, st7701sn_of_match);

static struct mipi_dsi_driver st7701sn_driver = {
	.probe = st7701sn_probe,
	.remove = st7701sn_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = st7701sn_of_match,
	},
};
module_mipi_dsi_driver(st7701sn_driver);

MODULE_AUTHOR("Wooden Chair <hua.zheng@embeddedboys.com>");
MODULE_DESCRIPTION("DRM Driver for BOE ZV039WVQ-N80 panels");
MODULE_LICENSE("GPL");
