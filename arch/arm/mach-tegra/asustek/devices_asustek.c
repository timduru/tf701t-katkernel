/* Copyright (c) 2013, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "../gpio-names.h"

#include <mach/board_asustek.h>


#ifdef CONFIG_ASUSTEK_PCBID
static char serialno[32] = {0,};
int __init asustek_androidboot_serialno(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(serialno, sizeof(serialno), "%s", s);
	serialno[n] = '\0';

	return 1;
}
__setup("androidboot.serialno", asustek_androidboot_serialno);

struct asustek_pcbid_platform_data asustek_pcbid_pdata = {
	.UUID = serialno,
};

static struct resource resources_asustek_pcbid[] = {
	{
		.start	= TEGRA_GPIO_PH7,
		.end	= TEGRA_GPIO_PH7,
		.name	= "PCB_ID0",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PH6,
		.end	= TEGRA_GPIO_PH6,
		.name	= "PCB_ID1",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PJ3,
		.end	= TEGRA_GPIO_PJ3,
		.name	= "PCB_ID2",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PB1,
		.end	= TEGRA_GPIO_PB1,
		.name	= "PCB_ID3",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PH5,
		.end	= TEGRA_GPIO_PH5,
		.name	= "PCB_ID4",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PK3,
		.end	= TEGRA_GPIO_PK3,
		.name	= "PCB_ID5",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PC7,
		.end	= TEGRA_GPIO_PC7,
		.name	= "PCB_ID6",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PO2,
		.end	= TEGRA_GPIO_PO2,
		.name	= "PCB_ID7",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PO6,
		.end	= TEGRA_GPIO_PO6,
		.name	= "PCB_ID8",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PP0,
		.end	= TEGRA_GPIO_PP0,
		.name	= "PCB_ID9",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= TEGRA_GPIO_PP3,
		.end	= TEGRA_GPIO_PP3,
		.name	= "PCB_ID10",
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device asustek_pcbid_device = {
	.name		= "asustek_pcbid",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_asustek_pcbid),
	.resource = resources_asustek_pcbid,
	.dev = {
		.platform_data = &asustek_pcbid_pdata,
	}
};

void __init asustek_add_pcbid_devices(void)
{
	printk("asustek_add_pcbid_devices+\n");
	platform_device_register(&asustek_pcbid_device);
	printk("asustek_add_pcbid_devices-\n");
}

#endif
