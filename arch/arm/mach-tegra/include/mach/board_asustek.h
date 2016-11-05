/* arch/arm/mach-tegra/include/mach/board_asustek.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 * Copyright (c) 2012-2013, ASUSTek Computer Inc.
 * Author: Hank Lee <hank_lee@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_ASUSTEK_H
#define __ASM_ARCH_MSM_BOARD_ASUSTEK_H

typedef enum {
	WIFI_BT_TYPE_INVALID = -1,
	WIFI_BT_TYPE_A = 0,
	WIFI_BT_TYPE_B = 1,
	WIFI_BT_TYPE_C = 2,
	WIFI_BT_TYPE_D = 3,
	WIFI_BT_TYPE_MAX
} wifi_bt_type;

typedef enum {
	PMU_TYPE_INVALID = -1,
	PMU_TYPE_A = 0,
	PMU_TYPE_B = 1,
	PMU_TYPE_MAX
} pmu_type;

typedef enum {
	GPS_TYPE_INVALID = -1,
	GPS_TYPE_A = 0,
	GPS_TYPE_B = 1,
} gps_type;

typedef enum {
	HW_REV_INVALID = -1,
	HW_REV_A = 0,
	HW_REV_B = 1,
	HW_REV_C = 2,
	HW_REV_D = 3,
	HW_REV_E = 4,
	HW_REV_F = 5,
	HW_REV_G = 6,
	HW_REV_H = 7,
	HW_REV_MAX
} hw_rev;

struct asustek_pcbid_platform_data {
	const char *UUID;
};

#ifdef CONFIG_ASUSTEK_PCBID
void __init asustek_add_pcbid_devices(void);

hw_rev asustek_get_hw_rev(void);

#endif

#endif // __ASM_ARCH_MSM_BOARD_ASUSTEK_H
