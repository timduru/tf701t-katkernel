/*
 * arch/arm/mach-tegra/board-macallan-sensors.c
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <linux/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/imx091.h>
#include <media/ov9772.h>
#include <media/as364x.h>
#include <media/ad5816.h>
#include <media/ov5693.h>
#include <media/ad5823.h>
#include <media/mi1040.h>
#include <linux/akm09911.h>
#include <generated/mach-types.h>
#include <linux/power/sbs-battery.h>
#include <linux/kionix_accel.h>
#include <asm/mach-types.h>
#include <mach/board_asustek.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-macallan.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

static struct board_info board_info;

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*    CPU,   C2BUS,   C3BUS,   SCLK,    EMC */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init macallan_throttle_init(void)
{
	if (machine_is_macallan() || machine_is_mozart() || machine_is_haydn())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(macallan_throttle_init);

static struct nct1008_platform_data macallan_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 120, /* C */

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

static struct i2c_board_info macallan_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &macallan_nct1008_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
}

static struct tegra_pingroup_config mclk_disable =
	VI_PINMUX(CAM_MCLK, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_disable =
	VI_PINMUX(GPIO_PBB0, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

/*
 * As a workaround, macallan_vcmvdd need to be allocated to activate the
 * sensor devices. This is due to the focuser device(AD5816) will hook up
 * the i2c bus if it is not powered up.
*/
static struct regulator *macallan_vcmvdd;

static int macallan_get_vcmvdd(void)
{
	if (!macallan_vcmvdd) {
		macallan_vcmvdd = regulator_get(NULL, "avdd_2v8_cam_af");
		if (unlikely(WARN_ON(IS_ERR(macallan_vcmvdd)))) {
			pr_err("%s: can't get regulator vcmvdd: %ld\n",
				__func__, PTR_ERR(macallan_vcmvdd));
			macallan_vcmvdd = NULL;
			return -ENODEV;
		}
	}
	return 0;
}

static int macallan_ov5693_power_on(struct ov5693_power_rail *pw)
{
	int err;

	pr_info("%s()+\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->dovdd)) {
		pr_info("%s()- EFAULT\n", __func__);
		return -EFAULT;
	}
	if (macallan_get_vcmvdd())
		goto ov5693_poweron_fail;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_iovdd_fail;

	err = regulator_enable(pw->avdd);

	if (err)
		goto ov5693_avdd_fail;

	msleep(8);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	regulator_set_voltage(macallan_vcmvdd, 2800000, 2800000);
	err = regulator_enable(macallan_vcmvdd);
	if (unlikely(err))
		goto ov5693_vcmvdd_fail;

	udelay(1000);

	tegra_pinmux_config_table(&mclk_enable, 1);
	usleep_range(300, 310);
	pr_info("%s()-\n", __func__);
	return 0;

ov5693_vcmvdd_fail:

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	regulator_disable(pw->dovdd);

ov5693_iovdd_fail:
ov5693_poweron_fail:
	pr_info("%s()- ENODEV\n", __func__);
	return -ENODEV;
}

static int macallan_ov5693_power_off(struct ov5693_power_rail *pw)
{
	pr_info("%s()+\n", __func__);
	if (unlikely(!pw || !macallan_vcmvdd || !pw->avdd || !pw->dovdd)) {
		pr_info("%s()-EFAULT\n", __func__);
		return -EFAULT;
	}
	usleep_range(21, 25);

	tegra_pinmux_config_table(&mclk_disable, 1);

	msleep(3);

	regulator_disable(macallan_vcmvdd);

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

	msleep(6);

	regulator_disable(pw->avdd);

	regulator_disable(pw->dovdd);

	pr_info("%s()-\n", __func__);
	return 0;
}

static struct nvc_gpio_pdata ov5693_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM1_POWER_DWN_GPIO, true, 0, },
};

#define ENABLE_OV5693_EDP 0
#if ENABLE_OV5693_EDP
static unsigned ov5693_estates[] = { 1010, 764, 256, 0 };
#endif

static struct ov5693_platform_data macallan_ov5693_pdata = {
	.num		= 0,
	.dev_name	= "ov5693",
	.gpio_count	= ARRAY_SIZE(ov5693_gpio_pdata),
	.gpio		= ov5693_gpio_pdata,
#if ENABLE_OV5693_EDP
	.edpc_config    = {
		.states = ov5693_estates,
                .num_states = ARRAY_SIZE(ov5693_estates),
                .e0_index = ARRAY_SIZE(ov5693_estates) - 1,
                .priority = EDP_MAX_PRIO + 1,
        },
#endif
	.power_on	= macallan_ov5693_power_on,
	.power_off	= macallan_ov5693_power_off,
};

static int macallan_ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);

	gpio_set_value_cansleep(pdata->gpio, 1);

	return err;
}

static int macallan_ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);
	return 0;
}

static struct ad5823_platform_data macallan_ad5823_pdata = {
	.gpio = CAM_AF_PWDN,
	.power_on	= macallan_ad5823_power_on,
	.power_off	= macallan_ad5823_power_off,
};


static struct i2c_board_info macallan_i2c_board_info_e1599[] = {
	{
		I2C_BOARD_INFO("ov5693", 0x10),
		.platform_data = &macallan_ov5693_pdata,
	},
	{
		I2C_BOARD_INFO("ad5823", 0x0c),
		.platform_data = &macallan_ad5823_pdata,
	},
};

static int yuv_mi1040_power_on(struct mi1040_power_rail *pw)
{
	int err;
	pr_info("%s+\n", __func__);

	err = regulator_enable(pw->vddio_1v8);
	if (err)
		goto mi1040_iovdd_fail;

	err = regulator_enable(pw->avdd_2v8);
	if (err)
		goto mi1040_avdd_fail;

	mdelay(10);

	tegra_pinmux_config_table(&pbb0_enable, 1);

	pr_info("%s-\n", __func__);
	return 0;

mi1040_avdd_fail:
	regulator_disable(pw->vddio_1v8);

mi1040_iovdd_fail:
	pr_info("%s()- ENODEV\n", __func__);
	return -ENODEV;
}

static int yuv_mi1040_power_off(struct mi1040_power_rail *pw)
{
	pr_info("%s()+\n", __func__);
	if (unlikely(!pw || !pw->avdd_2v8 || !pw->vddio_1v8)) {
		pr_info("%s()-EFAULT\n", __func__);
		return -EFAULT;
	}

	tegra_pinmux_config_table(&pbb0_disable, 1);
	mdelay(6);

	regulator_disable(pw->avdd_2v8);

	regulator_disable(pw->vddio_1v8);

	pr_info("%s()-\n", __func__);
	return 0;
}

struct mi1040_platform_data yuv_mi1040_pdata = {
	.power_on = yuv_mi1040_power_on,
	.power_off = yuv_mi1040_power_off,
};

static struct i2c_board_info front_sensor_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("mi1040", 0x48),
		.platform_data = &yuv_mi1040_pdata,
	},
};

static struct i2c_board_info macallan_i2c0_battery_by_EC_board_info[] = {
        {
                I2C_BOARD_INFO("pad_battery_by_EC", 0x0b),
        },
};

static int macallan_camera_init(void)
{
	int ret = 0;

	pr_info("%s ", __func__);

	ret = gpio_request(SUB_CAM_RST_GPIO, "SUB_CAM_RST");
	if (ret < 0) {
		pr_err("%s: SUB_CAM_RST_GPIO gpio_request failed %d\n",
			__func__, ret);
		return ret;
	}

	ret = gpio_direction_input(SUB_CAM_RST_GPIO);
	if (ret < 0) {
		pr_err("%s: SUB_CAM_RST_GPIO gpio_direction_input failed %d\n",
			__func__, ret);
		gpio_free(SUB_CAM_RST_GPIO);
		return ret;
	}

	tegra_pinmux_config_table(&mclk_disable, 1);
	tegra_pinmux_config_table(&pbb0_disable, 1);

	/* Rear Camera ov5693 */
	i2c_register_board_info(2, macallan_i2c_board_info_e1599,
		ARRAY_SIZE(macallan_i2c_board_info_e1599));

	/* Front Camera mi1040 */
	pr_info("mi1040 i2c_register_board_info");
	i2c_register_board_info(2, front_sensor_i2c2_board_info,
		ARRAY_SIZE(front_sensor_i2c2_board_info));

	return 0;
}

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

/* MPU board file definition	*/
static struct mpu_platform_data mpu6500_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION, /* overwrite with rewrite_orientation() */
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct akm09911_platform_data platform_data_akm09911 = {
	.gpio_RSTN = 0,
	.layout = MPU_COMPASS_LAYOUT, /* overwrite with rewrite_orientation() */
};

static struct kionix_accel_platform_data kionix_accel_pdata = {
	.min_interval = 5,
	.poll_interval = 200,
	.accel_direction = KIONIX_ACCEL_DIRECTION,
	.accel_irq_use_drdy = 1,
	.accel_res = KIONIX_ACCEL_RES_12BIT,
	.accel_g_range = KIONIX_ACCEL_G_2G,
};

static struct i2c_board_info __initdata kionix_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(KIONIX_ACCEL_NAME, KIONIX_ACCEL_ADDR),
		.platform_data = &kionix_accel_pdata,
	},
};

static struct i2c_board_info __initdata inv_mpu6500_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6500_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &platform_data_akm09911,
	},
};

struct mpu_orientation_def {
	__s8 gyro_orientation[9];
	char compass_layout;
};

/* Rewrite orientation & layout by project/hw_rev */
static void rewrite_orientation(void)
{
	hw_rev rev = asustek_get_hw_rev();
	if (HW_REV_INVALID == rev) {
		pr_err("%s: asustek_get_hw_rev invalid %d\n", __func__, rev);
		return;
	}
	if(machine_is_haydn()) {
		switch (rev) {
			case HW_REV_A: /* Haydn 2_0 */
			default: /* Haydn non-2_0 */
				kionix_accel_pdata.accel_direction = HAYDN_ACCEL_DIRECTION;
			break;
		}
	} else { // now only machine_is_mozart() goes here
		struct mpu_orientation_def * ori;
		/* switch by valid hw_rev, MUST assign ori */
		switch (rev) {
			case HW_REV_A: /* MOZART SR1 */ {
				static struct mpu_orientation_def MOZART_SR1 = {
					MOZART_SR1_MPU_GYRO_ORIENTATION,
					MOZART_SR1_MPU_COMPASS_LAYOUT
				};
				ori = &MOZART_SR1;
				pr_info("initialise mpu with MOZART SR1 config\n");
			}
			break;
			case HW_REV_E:
			case HW_REV_C: /* MOZART SR2/ER1 */ {
				static struct mpu_orientation_def MOZART_SR2_ER1 = {
					MOZART_MPU_GYRO_ORIENTATION,
					MOZART_SR2_ER1_MPU_COMPASS_LAYOUT
				};
				ori = &MOZART_SR2_ER1;
				pr_info("initialise mpu with MOZART SR2/ER1 config\n");
			}
			break;
			case HW_REV_G:
			case HW_REV_B:
			default: /* MOZART ER2/PR and unknow situations */ {
				static struct mpu_orientation_def MOZART = {
					MOZART_MPU_GYRO_ORIENTATION,
					MOZART_MPU_COMPASS_LAYOUT
				};
				ori = &MOZART;
				if(HW_REV_G == rev || HW_REV_B == rev)
					pr_info("initialise mpu with MOZART ER2/PR config\n");
				else
					pr_err("unknown hw_rev, try initialise mpu with the latest MOZART config\n");
			}
			break;
		}
		/* rewrite orientation & layout with ori*/
		memcpy(mpu6500_gyro_data.orientation, ori->gyro_orientation,
			   sizeof(mpu6500_gyro_data.orientation));
		platform_data_akm09911.layout = ori->compass_layout;
	}
}

static const struct i2c_board_info macallan_cam_i2c_board_info_als_al3320a[] = {
    {
        I2C_BOARD_INFO("al3320a",0x1c),
    },
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu6500_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	/* rewrite layout orientation by project/hw_rev */
	rewrite_orientation();
	i2c_register_board_info(gyro_bus_num, inv_mpu6500_i2c0_board_info,
		ARRAY_SIZE(inv_mpu6500_i2c0_board_info));
}

static void kionix_init(void)
{
	int ret = 0;
	pr_info("*** kionix_init START *** \n");
	ret = gpio_request(KIONIX_ACCEL_IRQ_GPIO, KIONIX_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}
	ret = gpio_direction_input(KIONIX_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(KIONIX_ACCEL_IRQ_GPIO);
		return;
	}

	kionix_i2c0_board_info[0].irq = gpio_to_irq(KIONIX_ACCEL_IRQ_GPIO);
	rewrite_orientation();
	i2c_register_board_info(KIONIX_ACCEL_BUS_NUM, kionix_i2c0_board_info,
			ARRAY_SIZE(kionix_i2c0_board_info));
	pr_info("*** kionix_init END *** \n");
}

static int macallan_nct1008_init(void)
{
	int nct1008_port;
	int ret = 0;

	nct1008_port = TEGRA_GPIO_PO4;

	tegra_add_cdev_trips(macallan_nct1008_pdata.trips,
				&macallan_nct1008_pdata.num_trips);

	macallan_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: macallan nct1008 irq %d",
			__func__, macallan_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* macallan has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, macallan_i2c4_nct1008_board_info,
		ARRAY_SIZE(macallan_i2c4_nct1008_board_info));

	return ret;
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 41000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			1, 0, 0, 0,
			0, 0, -1, -1,
			-1, -1, -1, -1,
			-1, -1, -1, -1,
			-2, -4, -5, -7
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			18, 16, 15, 13,
			11, 9, 8, 7,
			6, 6, 5, 4,
			3, 3, 1, 0,
			-2, -4, -5, -7
		},
	},
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.toffset = 5857,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*    CPU,   C2BUS,   C3BUS,   SCLK,    EMC    */
		{ { 1700000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1600000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1500000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 624000 } },
	{ {  739500, 528000, 468000, 372000, 624000 } },
	{ {  714000, 528000, 468000, 336000, 624000 } },
	{ {  688500, 528000, 420000, 336000, 624000 } },
	{ {  663000, 492000, 420000, 336000, 624000 } },
	{ {  637500, 492000, 420000, 336000, 624000 } },
	{ {  612000, 492000, 420000, 300000, 528000 } },
	{ {  586500, 492000, 360000, 336000, 528000 } },
	{ {  561000, 420000, 420000, 300000, 528000 } },
	{ {  535500, 420000, 360000, 228000, 528000 } },
	{ {  510000, 420000, 288000, 228000, 528000 } },
	{ {  484500, 324000, 288000, 228000, 528000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle skin_throttle = {
        .throt_tab_size = ARRAY_SIZE(skin_throttle_table),
        .throt_tab = skin_throttle_table,
};

static int __init macallan_skin_init(void)
{
	if (machine_is_macallan() || machine_is_mozart()) {
		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(macallan_skin_init);
#endif

int __init macallan_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = macallan_nct1008_init();
	if (err)
	{
		pr_err("%s: nct1008 register failed.\n", __func__);
		return err;
	}

	if(!machine_is_haydn()) {
		macallan_camera_init();
	}

	if(machine_is_haydn()) {
		kionix_init();
	} else { // now only machine_is_mozart() goes here
		mpuirq_init();
	}


	if (machine_is_mozart()){
		i2c_register_board_info(2, macallan_cam_i2c_board_info_als_al3320a,
				ARRAY_SIZE(macallan_cam_i2c_board_info_als_al3320a));
	}

	i2c_register_board_info(0, macallan_i2c0_battery_by_EC_board_info,
			ARRAY_SIZE(macallan_i2c0_battery_by_EC_board_info));
	return 0;
}
