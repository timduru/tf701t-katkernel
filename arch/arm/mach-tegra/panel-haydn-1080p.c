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

static struct platform_device *disp_device;

static struct regulator *hdmi_reg;
static struct regulator *hdmi_pll;

/* haydn scalar power enable */
#define scalar_enable           TEGRA_GPIO_PH4

#ifdef CONFIG_TEGRA_DC_CMU
static struct tegra_dc_cmu haydn_cmu = {
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

static struct tegra_dc_mode haydn_panel_modes[] = {
	{
		.pclk = 148500000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 44,
		.v_sync_width = 5,
		.h_back_porch = 148,
		.v_back_porch = 36,
		.h_active = 1920,
		.v_active = 1080,
		.h_front_porch = 88,
		.v_front_porch = 4,
	},
};

static struct resource haydn_disp1_resources[] = {
	{
		.name   = "irq",
		.start  = INT_DISPLAY_GENERAL,
		.end    = INT_DISPLAY_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "regs",
		.start  = TEGRA_DISPLAY_BASE,
		.end    = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "fbmem",
		.start  = 0, /* Filled in by macallan_panel_init() */
		.end    = 0, /* Filled in by macallan_panel_init() */
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "hdmi_regs",
		.start  = TEGRA_HDMI_BASE,
		.end    = TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct i2c_board_info haydn_scalar_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("scalar", 0x49),
	},
};

static int hdmi_enable(struct device *dev)
{
	int ret;
	if (!hdmi_reg) {
		hdmi_reg = regulator_get(dev, "avdd_hdmi");
		if (IS_ERR_OR_NULL(hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			hdmi_reg = NULL;
			return PTR_ERR(hdmi_reg);
		}
	}
	ret = regulator_enable(hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!hdmi_pll) {
		hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			hdmi_pll = NULL;
			regulator_put(hdmi_reg);
			hdmi_reg = NULL;
			return PTR_ERR(hdmi_pll);
		}
	}
	ret = regulator_enable(hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}

	gpio_set_value(scalar_enable, 1);

	return 0;
}

extern int scalar_update_status;

static int hdmi_disable(void)
{
	if (hdmi_reg) {
		regulator_disable(hdmi_reg);
		regulator_put(hdmi_reg);
		hdmi_reg = NULL;
	}

	if (hdmi_pll) {
		regulator_disable(hdmi_pll);
		regulator_put(hdmi_pll);
		hdmi_pll = NULL;
	}

	//do not power off scalar when scalar firmware is updating
	if(scalar_update_status!=2 &&
			scalar_update_status!=-1) {
		gpio_set_value(scalar_enable, 0);
	}

	return 0;
}

static void hdmi_haydn_1080p_dc_out_init(struct tegra_dc_out *dc)
{
	/* pass in macallan_disp1_out */
	dc->type = TEGRA_DC_OUT_HDMI;
	dc->flags = TEGRA_DC_OUT_CONTINUOUS_MODE;
	dc->parent_clk = "pll_d_out0";
	dc->max_pixclock = KHZ2PICOS(148500);
	dc->modes = haydn_panel_modes;
	dc->n_modes = ARRAY_SIZE(haydn_panel_modes);
	dc->depth = 24;
	dc->width = 408;
	dc->height = 230;
	dc->hotplug_init = NULL;

	dc->enable = hdmi_enable;
	dc->disable = hdmi_disable;
}

	static void
hdmi_haydn_1080p_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
	// haydn(dc0 with hdmi) does not have vpulse2 irq
	settings->use_vpulse2 = false;
	// default disable sd
	settings->enable = 0;

	//we use software phase in, not hardware phase in
	settings->phase_in_adjustments = true;
	settings->smooth_k_enable = false;
	//set min brightness for sd software phase in
	settings->panel_min_brightness = 77;
}

static void hdmi_haydn_1080p_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = haydn_panel_modes[0].h_active;
	fb->yres = haydn_panel_modes[0].v_active;
}

static void hdmi_haydn_1080p_set_disp_device(
		struct platform_device *display_device)
{
	int err;
	/*pass in macallan_disp1_device*/
	display_device->resource = haydn_disp1_resources;
	display_device->num_resources = ARRAY_SIZE(haydn_disp1_resources);

	disp_device = display_device;

	err = gpio_request(scalar_enable, "EN_VDD_BL");
	if (err < 0)
		printk("gpio scalar_enable request failed \n");
	gpio_direction_output(scalar_enable, 1);

}

static void hdmi_haydn_1080p_cmu_init(struct tegra_dc_platform_data *pdata)
{
	pdata->cmu = &haydn_cmu;
}

static int haydn_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static struct platform_pwm_backlight_data haydn_bl_data = {
	.max_brightness = 255,
	.dft_brightness = 224,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb       = haydn_check_fb,
};

static struct platform_device __maybe_unused
haydn_bl_device = {
	.name   = "scalar-backlight",
	.id     = -1,
	.dev    = {
		.platform_data = &haydn_bl_data,
	},
};

static struct platform_device __maybe_unused
*haydn_bl_devices[] __initdata = {
	&haydn_bl_device,
};

static int __init hdmi_haydn_1080p_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(haydn_bl_devices,
			ARRAY_SIZE(haydn_bl_devices));
	if (err) {
		pr_err("haydn bl device registration failed");
		return err;
	}

	i2c_register_board_info(2, haydn_scalar_i2c3_board_info,
			ARRAY_SIZE(haydn_scalar_i2c3_board_info));

	return err;
}

struct tegra_panel __initdata hdmi_haydn_1080p = {
	.init_sd_settings = hdmi_haydn_1080p_sd_settings_init,
	.init_dc_out = hdmi_haydn_1080p_dc_out_init,
	.init_fb_data = hdmi_haydn_1080p_fb_data_init,
	.register_bl_dev = hdmi_haydn_1080p_register_bl_dev,
	.init_cmu_data = hdmi_haydn_1080p_cmu_init,
	.set_disp_device = hdmi_haydn_1080p_set_disp_device,
};
EXPORT_SYMBOL(hdmi_haydn_1080p);
