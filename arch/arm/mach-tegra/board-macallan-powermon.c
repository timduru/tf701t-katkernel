/*
 * arch/arm/mach-tegra/board-macallan-powermon.c
 *
 * Copyright (c) 2013, NVIDIA Corporation. All Rights Reserved.
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

#include <linux/i2c.h>
#include <linux/ina219.h>
#include <linux/platform_data/ina230.h>

#include "board.h"
#include "board-macallan.h"

#define PRECISION_MULTIPLIER_MACALLAN 1000

enum {
	VD_CPU,
	VD_SOC,
	VS_DDR0,
	VS_DDR1,
	VS_LCD_BL,
	VD_LCD_HV,
	VS_SYS_1V8,
	VD_AP_1V8,
	VD_AP_RTC,
	VS_AUD_SYS,
	VD_DDR0,
	VD_DDR1,
	VD_AP_VBUS,
	VS_SYS_2V9,
	VA_PLLX,
	VA_AP_1V2,
};

enum {
	VDD_CELL
};

static struct ina230_platform_data power_mon_ina230_info[] = {
	[VDD_CELL] = {
		.calibration_data  = 0x20c4,
		.power_lsb = 3.051757813 * PRECISION_MULTIPLIER_MACALLAN,
		.rail_name = "VDD_CELL",
		.resistor = 5,
		.min_cores_online = 2,
		.divisor = 25,
		.precision_multiplier = PRECISION_MULTIPLIER_MACALLAN,
		/* set to 5A, wait for syseng tuning */
		.current_threshold = 5000,
		.shunt_polarity_inverted = 1,
	}
};

enum {
	INA_I2C_ADDR_44,
};

static struct i2c_board_info macallan_i2c1_ina_board_info[] = {
	[INA_I2C_ADDR_44] = {
		I2C_BOARD_INFO("ina230", 0x44),
		.platform_data = &power_mon_ina230_info[VDD_CELL],
		.irq = -1,
	},
};

int __init macallan_pmon_init(void)
{
	i2c_register_board_info(1, macallan_i2c1_ina_board_info,
		ARRAY_SIZE(macallan_i2c1_ina_board_info));

	return 0;
}


