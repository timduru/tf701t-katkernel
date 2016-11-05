/*
 * arch/arm/mach-tegra/panel-s-wqxga-10-1.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mach/dc.h>
#include <mach/iomap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/max8831_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include <generated/mach-types.h>
#include "board.h"
#include "board-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define TEGRA_DSI_GANGED_MODE	1

#define DSI_PANEL_RESET		0
#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH3
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1

#define CONTINUOUS_MODE 1

#if CONTINUOUS_MODE
#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE
#else
#define DC_CTRL_MODE	(TEGRA_DC_OUT_ONE_SHOT_MODE |\
			TEGRA_DC_OUT_ONE_SHOT_LP_MODE)
#endif

#define en_vdd_bl	TEGRA_GPIO_PG0
#define lvds_en		TEGRA_GPIO_PG3


static bool reg_requested;
static bool gpio_requested;
static struct platform_device *disp_device;
static struct regulator *avdd_lcd_3v3;
static struct regulator *dvdd_lcd_3v3;
static struct regulator *vdd_lcd_bl;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *dvdd_lcd_1v8;
static struct regulator *vdd_ds_1v8;

#define en_vdd_bl	TEGRA_GPIO_PG0
#define lvds_en		TEGRA_GPIO_PG3

static tegra_dc_bl_output dsi_s_wqxga_10_1_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 11, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 21, 22,
	23, 24, 25, 26, 27, 28, 29, 30,
	31, 32, 32, 33, 34, 35, 36, 37,
	38, 39, 40, 41, 42, 43, 43, 44,
	45, 46, 47, 48, 49, 50, 51, 52,
	53, 54, 54, 55, 56, 57, 58, 59,
	60, 61, 62, 63, 63, 64, 65, 66,
	67, 68, 69, 70, 71, 72, 73, 74,
	75, 76, 77, 78, 79, 80, 80, 81,
	82, 83, 84, 85, 86, 87, 88, 89,
	90, 91, 92, 93, 94, 95, 96, 97,
	98, 99, 100, 101, 102, 103, 104, 105,
	106, 107, 108, 109, 110, 111, 112, 113,
	114, 115, 116, 117, 118, 119, 120, 121,
	122, 123, 124, 125, 126, 127, 128, 129,
	130, 131, 132, 133, 134, 135, 136, 137,
	138, 140, 141, 142, 143, 144, 145, 146,
	147, 148, 149, 150, 151, 152, 153, 154,
	155, 156, 157, 158, 159, 160, 161, 162,
	163, 164, 165, 166, 167, 168, 169, 170,
	171, 172, 173, 174, 175, 177, 178, 179,
	180, 181, 182, 183, 184, 185, 186, 187,
	188, 189, 190, 191, 192, 193, 194, 195,
	196, 197, 198, 200, 201, 202, 203, 204,
	205, 206, 207, 208, 209, 210, 211, 212,
	213, 214, 215, 217, 218, 219, 220, 221,
	222, 223, 224, 225, 226, 227, 228, 229,
	230, 231, 232, 234, 235, 236, 237, 238,
	239, 240, 241, 242, 243, 244, 245, 246,
	248, 249, 250, 251, 252, 253, 254, 255,
};

u8 fbuf_mode_sel[] = {0x10, 0x00, 0x2A}; /* left-right */
u8 mipi_if_sel[] = {0x10, 0x01, 0x01}; /* cmd mode */
static struct tegra_dsi_cmd dsi_s_wqxga_10_1_init_cmd[] = {
#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, fbuf_mode_sel),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_DLY_MS(20),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, mipi_if_sel),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM,
				DSI_DCS_SET_TEARING_EFFECT_ON, 0x0),
	DSI_DLY_MS(20),
#endif
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(40),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(20),
};

static struct tegra_dsi_cmd dsi_s_wqxga_10_1_suspend_cmd[] = {
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_OFF, 0x0),
	DSI_DLY_MS(50),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_ENTER_SLEEP_MODE, 0x0),
	DSI_DLY_MS(200),
#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM,
			DSI_DCS_SET_TEARING_EFFECT_OFF, 0x0),
	DSI_DLY_MS(20),
#endif
};

static struct tegra_dsi_out dsi_s_wqxga_10_1_pdata = {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.n_data_lanes = 2,
	.controller_vs = DSI_VS_0,
#else
	.controller_vs = DSI_VS_1,
#endif

	.n_data_lanes = 8,

#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.ganged_type = TEGRA_DSI_GANGED_SYMMETRIC_LEFT_RIGHT,
	.suspend_aggr = DSI_HOST_SUSPEND_LV2,
	.refresh_rate = 62,
	.rated_refresh_rate = 60,
	.te_polarity_low = true,
#else
	.ganged_type = TEGRA_DSI_GANGED_SYMMETRIC_EVEN_ODD,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE,
	.refresh_rate = 60,
#endif

	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.dsi_init_cmd = dsi_s_wqxga_10_1_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_s_wqxga_10_1_init_cmd),
	.dsi_suspend_cmd = dsi_s_wqxga_10_1_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_s_wqxga_10_1_suspend_cmd),
	.bl_name = "pwm-backlight",
	.lp00_pre_panel_wakeup = true,
	.ulpm_not_supported = true,
};

static int dalmore_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	dvdd_lcd_1v8 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(dvdd_lcd_1v8)) {
		pr_err("dvdd_lcd regulator get failed\n");
		err = PTR_ERR(dvdd_lcd_1v8);
		dvdd_lcd_1v8 = NULL;
		goto fail;
	}

	avdd_lcd_3v3 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v3)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v3);
		avdd_lcd_3v3 = NULL;
		goto fail;
	}

	vdd_lcd_bl = regulator_get(dev, "vdd_lcd_bl");
	if (IS_ERR_OR_NULL(vdd_lcd_bl)) {
		pr_err("vdd_lcd_bl regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl);
		vdd_lcd_bl = NULL;
		goto fail;
	}

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}
	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dalmore_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	err = gpio_request(DSI_PANEL_RST_GPIO, "panel rst");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}

	/* free pwm GPIO */
	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	gpio_free(DSI_PANEL_BL_PWM);
	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int macallan_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	dvdd_lcd_3v3 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(dvdd_lcd_3v3)) {
		pr_err("dvdd_lcd regulator get failed\n");
		err = PTR_ERR(dvdd_lcd_3v3);
		dvdd_lcd_3v3 = NULL;
		goto fail;
	}

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}
	reg_requested = true;
	return 0;
fail:
	return err;
}

static int macallan_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	err = gpio_request(DSI_PANEL_RST_GPIO, "panel rst");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}

	/* free pwm GPIO */
	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	gpio_free(DSI_PANEL_BL_PWM);
	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_s_wqxga_10_1_postpoweron(struct device *dev)
{
	int err = 0;

	if (machine_is_dalmore())
		err = dalmore_dsi_regulator_get(dev);
	else if (machine_is_macallan() || machine_is_mozart() || machine_is_haydn())
		err = macallan_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	if (machine_is_dalmore())
		err = dalmore_dsi_gpio_get();
	else if (machine_is_macallan() || machine_is_mozart())
		err = macallan_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if (vdd_ds_1v8) {
		err = regulator_enable(vdd_ds_1v8);
		if (err < 0) {
			pr_err("vdd_ds_1v8 regulator enable failed\n");
			goto fail;
		}
	}

	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	if (dvdd_lcd_3v3) {
		err = regulator_enable(dvdd_lcd_3v3);
		if (err < 0) {
			pr_err("dvdd_lcd_3v3 regulator enable failed\n");
			goto fail;
		}
	}

	/* panel ic requirement after vcc enable */
	msleep(230);

	if (vdd_lcd_bl) {
		err = regulator_enable(vdd_lcd_bl);
		if (err < 0) {
			pr_err("vdd_lcd_bl regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}


#if DSI_PANEL_RESET
	gpio_direction_output(DSI_PANEL_RST_GPIO, 1);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 0);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
	msleep(20);
#endif

	return 0;
fail:
	return err;
}

static int dsi_s_wqxga_10_1_enable(struct device *dev)
{
	return 0;
}

static int dsi_s_wqxga_10_1_disable(void)
{
	if (dvdd_lcd_3v3)
		regulator_disable(dvdd_lcd_3v3);

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);

	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);

	/* Make sure we have at least 1 second delay before turning on panel next time */
	usleep_range(1000000, 1000010);
	return 0;
}

static int dsi_s_wqxga_10_1_postsuspend(void)
{
	return 0;
}
static void bl_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(bl_work, bl_work_handler);
static int dsi_s_wqxga_10_1_prepoweroff(void)
{
	cancel_delayed_work_sync(&bl_work);

	if (vdd_lcd_bl_en && regulator_is_enabled(vdd_lcd_bl_en))
		regulator_disable(vdd_lcd_bl_en);

	if (vdd_lcd_bl && regulator_is_enabled(vdd_lcd_bl))
		regulator_disable(vdd_lcd_bl);

	msleep(20);
	return 0;
}
static void bl_work_handler(struct work_struct *work)
{
	int err;

	if (vdd_lcd_bl) {
		err = regulator_enable(vdd_lcd_bl);
			if (err < 0) {
				pr_err("vdd_lcd_bl regulator enable failed %d\n", err);
			}
	}

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed %d\n", err);
		}
	}
}
static struct tegra_dc_mode dsi_s_wqxga_10_1_modes[] = {
	{
#if DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE
		.pclk = 306156000, /* @62Hz */
#else
		.pclk = 268500000, /* @60Hz */
#endif
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 32,
		.v_sync_width = 6,
		.h_back_porch = 80,
		.v_back_porch = 37,
		.h_active = 2560,
		.v_active = 1600,
		.h_front_porch = 48,
		.v_front_porch = 3,
	},
};

#ifdef CONFIG_TEGRA_DC_CMU
static struct tegra_dc_cmu dsi_s_wqxga_10_1_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0x000, 0x010, 0x020, 0x030, 0x040, 0x050, 0x060, 0x070,
		0x080, 0x090, 0x0A0, 0x0B0, 0x0C0, 0x0D0, 0x0E0, 0x0F0,
		0x100, 0x110, 0x120, 0x130, 0x140, 0x150, 0x160, 0x170,
		0x180, 0x190, 0x1A0, 0x1B0, 0x1C0, 0x1D0, 0x1E0, 0x1F0,
		0x200, 0x210, 0x220, 0x230, 0x240, 0x250, 0x260, 0x270,
		0x280, 0x290, 0x2A0, 0x2B0, 0x2C0, 0x2D0, 0x2E0, 0x2F0,
		0x300, 0x310, 0x320, 0x330, 0x340, 0x350, 0x360, 0x370,
		0x380, 0x390, 0x3A0, 0x3B0, 0x3C0, 0x3D0, 0x3E0, 0x3F0,
		0x400, 0x410, 0x420, 0x430, 0x440, 0x450, 0x460, 0x470,
		0x480, 0x490, 0x4A0, 0x4B0, 0x4C0, 0x4D0, 0x4E0, 0x4F0,
		0x500, 0x510, 0x520, 0x530, 0x540, 0x550, 0x560, 0x570,
		0x580, 0x590, 0x5A0, 0x5B0, 0x5C0, 0x5D0, 0x5E0, 0x5F0,
		0x600, 0x610, 0x620, 0x630, 0x640, 0x650, 0x660, 0x670,
		0x680, 0x690, 0x6A0, 0x6B0, 0x6C0, 0x6D0, 0x6E0, 0x6F0,
		0x700, 0x710, 0x720, 0x730, 0x740, 0x750, 0x760, 0x770,
		0x780, 0x790, 0x7A0, 0x7B0, 0x7C0, 0x7D0, 0x7E0, 0x7F0,
		0x800, 0x810, 0x820, 0x830, 0x840, 0x850, 0x860, 0x870,
		0x880, 0x890, 0x8A0, 0x8B0, 0x8C0, 0x8D0, 0x8E0, 0x8F0,
		0x900, 0x910, 0x920, 0x930, 0x940, 0x950, 0x960, 0x970,
		0x980, 0x990, 0x9A0, 0x9B0, 0x9C0, 0x9D0, 0x9E0, 0x9F0,
		0xA00, 0xA10, 0xA20, 0xA30, 0xA40, 0xA50, 0xA60, 0xA70,
		0xA80, 0xA90, 0xAA0, 0xAB0, 0xAC0, 0xAD0, 0xAE0, 0xAF0,
		0xB00, 0xB10, 0xB20, 0xB30, 0xB40, 0xB50, 0xB60, 0xB70,
		0xB80, 0xB90, 0xBA0, 0xBB0, 0xBC0, 0xBD0, 0xBE0, 0xBF0,
		0xC00, 0xC10, 0xC20, 0xC30, 0xC40, 0xC50, 0xC60, 0xC70,
		0xC80, 0xC90, 0xCA0, 0xCB0, 0xCC0, 0xCD0, 0xCE0, 0xCF0,
		0xD00, 0xD10, 0xD20, 0xD30, 0xD40, 0xD50, 0xD60, 0xD70,
		0xD80, 0xD90, 0xDA0, 0xDB0, 0xDC0, 0xDD0, 0xDE0, 0xDF0,
		0xE00, 0xE10, 0xE20, 0xE30, 0xE40, 0xE50, 0xE60, 0xE70,
		0xE80, 0xE90, 0xEA0, 0xEB0, 0xEC0, 0xED0, 0xEE0, 0xEF0,
		0xF00, 0xF10, 0xF20, 0xF30, 0xF40, 0xF50, 0xF60, 0xF70,
		0xF80, 0xF90, 0xFA0, 0xFB0, 0xFC0, 0xFD0, 0xFE0, 0xFF0,
	},
	/* csc */
	{
		0x100, 0x000, 0x000, /* 1.0 0.0 0.0 */
		0x000, 0x100, 0x000, /* 0.0 1.0 0.0 */
		0x000, 0x000, 0x100, /* 0.0 0.0 1.0 */
	},
	/* lut2 maps linear space to sRGB */
	{
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2,
		3, 3, 3, 3, 3, 3, 3, 3,
		3, 3, 3, 3, 3, 3, 3, 3,
		4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4,
		5, 5, 5, 5, 5, 5, 5, 5,
		5, 5, 5, 5, 5, 5, 5, 5,
		6, 6, 6, 6, 6, 6, 6, 6,
		6, 6, 6, 6, 6, 6, 6, 6,
		7, 7, 7, 7, 7, 7, 7, 7,
		7, 7, 7, 7, 7, 7, 7, 7,
		8, 8, 8, 8, 8, 8, 8, 8,
		8, 8, 8, 8, 8, 8, 8, 8,
		9, 9, 9, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9,
		10, 10, 10, 10, 10, 10, 10, 10,
		10, 10, 10, 10, 10, 10, 10, 10,
		11, 11, 11, 11, 11, 11, 11, 11,
		11, 11, 11, 11, 11, 11, 11, 11,
		12, 12, 12, 12, 12, 12, 12, 12,
		12, 12, 12, 12, 12, 12, 12, 12,
		13, 13, 13, 13, 13, 13, 13, 13,
		13, 13, 13, 13, 13, 13, 13, 13,
		14, 14, 14, 14, 14, 14, 14, 14,
		14, 14, 14, 14, 14, 14, 14, 14,
		15, 15, 15, 15, 15, 15, 15, 15,
		15, 15, 15, 15, 15, 15, 15, 15,
		16, 16, 16, 16, 16, 16, 16, 16,
		16, 16, 16, 16, 16, 16, 16, 16,
		17, 17, 17, 17, 17, 17, 17, 17,
		17, 17, 17, 17, 17, 17, 17, 17,
		18, 18, 18, 18, 18, 18, 18, 18,
		18, 18, 18, 18, 18, 18, 18, 18,
		19, 19, 19, 19, 19, 19, 19, 19,
		19, 19, 19, 19, 19, 19, 19, 19,
		20, 20, 20, 20, 20, 20, 20, 20,
		20, 20, 20, 20, 20, 20, 20, 20,
		21, 21, 21, 21, 21, 21, 21, 21,
		21, 21, 21, 21, 21, 21, 21, 21,
		22, 22, 22, 22, 22, 22, 22, 22,
		22, 22, 22, 22, 22, 22, 22, 22,
		23, 23, 23, 23, 23, 23, 23, 23,
		23, 23, 23, 23, 23, 23, 23, 23,
		24, 24, 24, 24, 24, 24, 24, 24,
		24, 24, 24, 24, 24, 24, 24, 24,
		25, 25, 25, 25, 25, 25, 25, 25,
		25, 25, 25, 25, 25, 25, 25, 25,
		26, 26, 26, 26, 26, 26, 26, 26,
		26, 26, 26, 26, 26, 26, 26, 26,
		27, 27, 27, 27, 27, 27, 27, 27,
		27, 27, 27, 27, 27, 27, 27, 27,
		28, 28, 28, 28, 28, 28, 28, 28,
		28, 28, 28, 28, 28, 28, 28, 28,
		29, 29, 29, 29, 29, 29, 29, 29,
		29, 29, 29, 29, 29, 29, 29, 29,
		30, 30, 30, 30, 30, 30, 30, 30,
		30, 30, 30, 30, 30, 30, 30, 30,
		31, 31, 31, 31, 31, 31, 31, 31,
		31, 31, 31, 31, 31, 31, 31, 31,
		32, 32, 32, 32, 32, 32, 32, 32,
		32, 33, 33, 34, 34, 35, 35, 36,
		36, 37, 37, 38, 38, 39, 39, 40,
		40, 41, 41, 42, 42, 43, 43, 44,
		44, 45, 45, 46, 46, 47, 47, 48,
		48, 49, 49, 50, 50, 51, 51, 52,
		52, 53, 53, 54, 54, 55, 55, 56,
		56, 57, 57, 58, 58, 59, 59, 60,
		60, 61, 61, 62, 62, 63, 63, 64,
		64, 65, 65, 66, 66, 67, 67, 68,
		68, 69, 69, 70, 70, 71, 71, 72,
		72, 73, 73, 74, 74, 75, 75, 76,
		76, 77, 77, 78, 78, 79, 79, 80,
		80, 81, 81, 82, 82, 83, 83, 84,
		84, 85, 85, 86, 86, 87, 87, 88,
		88, 89, 89, 90, 90, 91, 91, 92,
		92, 93, 93, 94, 94, 95, 95, 96,
		96, 97, 97, 98, 98, 99, 99, 100,
		100, 101, 101, 102, 102, 103, 103, 104,
		104, 105, 105, 106, 106, 107, 107, 108,
		108, 109, 109, 110, 110, 111, 111, 112,
		112, 113, 113, 114, 114, 115, 115, 116,
		116, 117, 117, 118, 118, 119, 119, 120,
		120, 121, 121, 122, 122, 123, 123, 124,
		124, 125, 125, 126, 126, 127, 127, 128,
		128, 129, 129, 130, 130, 131, 131, 132,
		132, 133, 133, 134, 134, 135, 135, 136,
		136, 137, 137, 138, 138, 139, 139, 140,
		140, 141, 141, 142, 142, 143, 143, 144,
		144, 145, 145, 146, 146, 147, 147, 148,
		148, 149, 149, 150, 150, 151, 151, 152,
		152, 153, 153, 154, 154, 155, 155, 156,
		156, 157, 157, 158, 158, 159, 159, 160,
		160, 161, 161, 162, 162, 163, 163, 164,
		164, 165, 165, 166, 166, 167, 167, 168,
		168, 169, 169, 170, 170, 171, 171, 172,
		172, 173, 173, 174, 174, 175, 175, 176,
		176, 177, 177, 178, 178, 179, 179, 180,
		180, 181, 181, 182, 182, 183, 183, 184,
		184, 185, 185, 186, 186, 187, 187, 188,
		188, 189, 189, 190, 190, 191, 191, 192,
		192, 193, 193, 194, 194, 195, 195, 196,
		196, 197, 197, 198, 198, 199, 199, 200,
		200, 201, 201, 202, 202, 203, 203, 204,
		204, 205, 205, 206, 206, 207, 207, 208,
		208, 209, 209, 210, 210, 211, 211, 212,
		212, 213, 213, 214, 214, 215, 215, 216,
		216, 217, 217, 218, 218, 219, 219, 220,
		220, 221, 221, 222, 222, 223, 223, 224,
		224, 225, 225, 226, 226, 227, 227, 228,
		228, 229, 229, 230, 230, 231, 231, 232,
		232, 233, 233, 234, 234, 235, 235, 236,
		236, 237, 237, 238, 238, 239, 239, 240,
		240, 241, 241, 242, 242, 243, 243, 244,
		244, 245, 245, 246, 246, 247, 247, 248,
		248, 249, 249, 250, 250, 251, 251, 252,
		252, 253, 253, 254, 254, 255, 255, 255,
	},
};
#endif

static int dsi_s_wqxga_10_1_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_s_wqxga_10_1_bl_output_measured[brightness];

	return brightness;
}

static int dsi_s_wqxga_10_1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static struct platform_pwm_backlight_data dsi_s_wqxga_10_1_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_s_wqxga_10_1_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_s_wqxga_10_1_check_fb,
};

static struct platform_device __maybe_unused
		dsi_s_wqxga_10_1_bl_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_s_wqxga_10_1_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_s_wqxga_10_1_bl_devices[] __initdata = {
	&tegra_pwfm1_device,
	&dsi_s_wqxga_10_1_bl_device,
};

static int __init dsi_s_wqxga_10_1_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(dsi_s_wqxga_10_1_bl_devices,
				ARRAY_SIZE(dsi_s_wqxga_10_1_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_s_wqxga_10_1_set_disp_device(
	struct platform_device *dalmore_display_device)
{
	disp_device = dalmore_display_device;
}
/* Sharp is configured in Ganged mode */
static void dsi_s_wqxga_10_1_resources_init(struct resource *
resources, int n_resources)
{
	int i;
	for (i = 0; i < n_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "ganged_dsia_regs")) {
			r->start = TEGRA_DSI_BASE;
			r->end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1;
		}
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "ganged_dsib_regs")) {
			r->start = TEGRA_DSIB_BASE;
			r->end = TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1;
		}
	}
}

static void dsi_s_wqxga_10_1_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_s_wqxga_10_1_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_s_wqxga_10_1_modes;
	dc->n_modes = ARRAY_SIZE(dsi_s_wqxga_10_1_modes);
	dc->enable = dsi_s_wqxga_10_1_enable;
	dc->postpoweron = dsi_s_wqxga_10_1_postpoweron;
	dc->disable = dsi_s_wqxga_10_1_disable;
	dc->postsuspend	= dsi_s_wqxga_10_1_postsuspend,
	dc->prepoweroff = dsi_s_wqxga_10_1_prepoweroff,
	dc->width = 216;
	dc->height = 135;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_s_wqxga_10_1_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_s_wqxga_10_1_modes[0].h_active;
	fb->yres = dsi_s_wqxga_10_1_modes[0].v_active;
}

static void
dsi_s_wqxga_10_1_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}

static void dsi_s_wqxga_10_1_cmu_init(struct tegra_dc_platform_data *pdata)
{
	pdata->cmu = &dsi_s_wqxga_10_1_cmu;
}

struct tegra_panel __initdata dsi_s_wqxga_10_1 = {
	.init_sd_settings = dsi_s_wqxga_10_1_sd_settings_init,
	.init_dc_out = dsi_s_wqxga_10_1_dc_out_init,
	.init_fb_data = dsi_s_wqxga_10_1_fb_data_init,
	.init_resources = dsi_s_wqxga_10_1_resources_init,
	.register_bl_dev = dsi_s_wqxga_10_1_register_bl_dev,
	.init_cmu_data = dsi_s_wqxga_10_1_cmu_init,
	.set_disp_device = dsi_s_wqxga_10_1_set_disp_device,
};
EXPORT_SYMBOL(dsi_s_wqxga_10_1);

