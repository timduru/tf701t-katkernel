/*
 * drivers/power/pad_battery_by_ec.c
 **
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/board_asustek.h>
#include "../../arch/arm/mach-tegra/cpu-tegra.h"
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/wakeups-t11x.h"
#include "../usb/gadget/tegra_udc.h"
//#include "tegra_udc.h"
//#include <mach/board-cardhu-misc.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#define GPIO_PIN_LOW_BATTERY_DETECT TEGRA_GPIO_PR4
#define SMBUS_RETRY (0)
#define KELVIN_BASE 2730
#define INITIAL_CAPACITY_VALUE	50
#define INITIAL_TEMPERATURE_VALUE	250
#define PROTECT_TEMPERATURE_IN_CELSIUS_HIGH 1200
#define PROTECT_TEMPERATURE_IN_CELSIUS_LOW (-200)
#define PROPERTY_ERROR_RESET_VALUE_CAPACITY	0
#define BATTERY_POLLING_RATE 60
#define RETRY_TIMES_IF_EC_FEEDBACK_ERROR	(3)
#define EC_FEEDBACK_STATUS_DISCHARGING		0x0040
#define EC_FEEDBACK_STATUS_FULLY_CHARGED	0x0020
#define EC_FEEDBACK_STATUS_FULLY_DISCHARGED	0x0010
#define HAYDN_GPIO_AC_OK TEGRA_GPIO_PV1
#define _PROPERTY_AND_REG_SPEC(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

bool low_battery_flag = 0;

#ifdef REMOVE_USB_POWER_SUPPLY
static bool can_power_supply_by_usb = false;
#else
static bool can_power_supply_by_usb = true;
#endif
static int test_temperature = 2730;
static bool pad_battery_by_EC_bq27520_FW_updating_enable = false;
static bool pad_battery_by_EC_driver_ready = false;
#ifdef USER_IMAGE
static bool pad_gaugeIC_firmware_is_updated = true;
static bool image_is_user = true;
#else
static bool pad_gaugeIC_firmware_is_updated = false;
static bool image_is_user = false;
#endif
static bool dock_gaugeIC_firmware_is_updated = true;
static bool project_is_haydn = false;
static struct pad_battery_struct *pad_battery = NULL;
static struct workqueue_struct *pad_battery_by_EC_workqueue = NULL;
char gaugeIC_firmware_message[100];
static int delay_time = 0x07;
static int pad_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
extern int asuspec_battery_monitor(char *cmd, bool pad);
extern int cable_status_register_client(struct notifier_block *nb);
extern unsigned int query_cable_status(void);
extern int asuspec_gauge_firmware_update(u8 cmd, u8 buff[], u8 length, bool pad);
extern int asuspec_start_gauge_firmware_update(bool pad);
extern int asuspec_stop_gauge_firmware_update(bool pad);
extern u16 asuspec_gauge_ic_monitor(char *cmd, bool pad);
extern int asuspec_get_gauge_mode(bool pad);

enum which_battery_enum{
	dock_device_battery = 0,
	pad_device_battery,
};

enum pad_battery_cable_type {
	non_cable =0,
	usb_cable,
	unknow_cable,
	ac_cable,
};

enum gaugeIC_update_ret_value{
	EC_error_unknow = -4,
	gaugeIC_check_error = -3,
	EC_i2c_router_error = -2,
	EC_i2c_busy = -1,
	update_success = 0,
};

static enum pad_battery_cable_type this_time_cable_type = non_cable;

enum property_REG{
       REG_MANUFACTURER_DATA = 0,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_CURRENT_AVG,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY_IN_PERCENT,
	REG_SERIAL_NUMBER,
	REG_CAPACITY_IN_MAH,
	REG_MAX,
	REC_FULL_CHARGE_CAPACITY,
};

struct low_low_battery_struct{
	bool	low_low_battery_present;
	unsigned	GPIO_pin;
	unsigned int irq;
	struct wake_lock low_low_battery_wake_lock;
};

struct cable_type_struct{
	enum pad_battery_cable_type this_time_cable_type;
	enum pad_battery_cable_type last_time_cable_type;
	struct wake_lock cable_type_change_event_wake_lock;
};

struct haydn_AC_ok_struct{
	unsigned	GPIO_pin;
	unsigned int irq;
	struct wake_lock AC_ok_wake_lock;
};

struct pad_battery_struct{
	struct mutex			mutex_lock;
	struct i2c_client		*client;
	struct power_supply	ac;
	struct power_supply	usb;
	struct power_supply	battery;
	struct power_supply	dock_battery;
	struct power_supply	dock_ac;
	struct delayed_work	battery_status_polling_work;
	struct delayed_work	low_low_battery_work;
	struct delayed_work	check_pad_gaugeIC_firmware_is_updated;
	struct delayed_work	check_dock_gaugeIC_firmware_is_updated;
	struct delayed_work	battery_status_reupdate_at_booting;
	struct delayed_work	haydn_AC_ok_work;
	struct low_low_battery_struct low_low_battery;
	struct cable_type_struct cable_type;
	struct timer_list dock_battery_status_detect_timer;
	struct haydn_AC_ok_struct haydn_AC_ok;
	struct wake_lock gaugeIC_updating;
	int capacity_in_percent_this_time;
	int capacity_in_percent_last_time;
	int dock_capacity_in_percent_this_time;
	int dock_capacity_in_percent_last_time;
	int temperature_this_time;
	int temperature_last_time;
	int capacity_get_error_times;
	int dock_capacity_get_error_times;
	bool temperature_get_error;
	bool chargerIC_is_work;
	bool dock_in;
	bool haydn_AC_in;
};

static enum power_supply_property pad_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
};

static enum power_supply_property ac_and_usb_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property dock_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static char *ac_and_usb_supply_list[] = {"battery",};

#define CONFIG_POWER_SUPPLY_BATTERY(ps_battery)					\
do{																	\
	ps_battery.name		= "battery";									\
	ps_battery.type		= POWER_SUPPLY_TYPE_BATTERY;				\
	ps_battery.properties	= pad_battery_properties;				\
	ps_battery.num_properties = ARRAY_SIZE(pad_battery_properties); \
	ps_battery.get_property	= pad_battery_get_property;					\
}while(0)

#define CONFIG_POWER_SUPPLY_AC(ps_ac)							\
do{																	\
	ps_ac.name		= "ac";												\
	ps_ac.type		= POWER_SUPPLY_TYPE_MAINS;							\
	ps_ac.supplied_to	= ac_and_usb_supply_list;											\
	ps_ac.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	ps_ac.properties = ac_and_usb_power_properties;							\
	ps_ac.num_properties = ARRAY_SIZE(ac_and_usb_power_properties);			\
	ps_ac.get_property	= power_get_property;								\
}while(0)

#define CONFIG_POWER_SUPPLY_USB(ps_USB)							\
do{																	\
	ps_USB.name		= "usb";												\
	ps_USB.type		= POWER_SUPPLY_TYPE_USB;							\
	ps_USB.supplied_to	= ac_and_usb_supply_list;											\
	ps_USB.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	ps_USB.properties = ac_and_usb_power_properties;							\
	ps_USB.num_properties = ARRAY_SIZE(ac_and_usb_power_properties);			\
	ps_USB.get_property = power_get_property;								\
}while(0)

#define CONFIG_POWER_SUPPLY_AC_FROM_DOCK(ps_from_dock_ac)							\
do{																	\
	ps_from_dock_ac.name		= "docking_ac";												\
	ps_from_dock_ac.type		= POWER_SUPPLY_TYPE_DOCK_AC;							\
	ps_from_dock_ac.supplied_to	= ac_and_usb_supply_list;											\
	ps_from_dock_ac.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	ps_from_dock_ac.properties = ac_and_usb_power_properties;							\
	ps_from_dock_ac.num_properties = ARRAY_SIZE(ac_and_usb_power_properties);			\
	ps_from_dock_ac.get_property = power_get_property;								\
}while(0)

#define CONFIG_POWER_SUPPLY_DOCK_BATTERY(dock_battery)							\
do{																	\
	dock_battery.name		= "dock_battery";												\
	dock_battery.type		= POWER_SUPPLY_TYPE_DOCK_BATTERY;							\
	dock_battery.supplied_to	= ac_and_usb_supply_list;											\
	dock_battery.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	dock_battery.properties = dock_battery_properties;							\
	dock_battery.num_properties = ARRAY_SIZE(dock_battery_properties);			\
	dock_battery.get_property = pad_battery_get_property;								\
}while(0)



static struct property_and_reg_spec {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27520_property_and_reg_spec[] = {
       [REG_MANUFACTURER_DATA]	= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_PRESENT, 0, 0, 65535),
       [REG_STATE_OF_HEALTH]		= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]			= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_TEMP, 0x06, 0, 65535),
	[REG_VOLTAGE]				= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x08, 0, 6000),
	[REG_CURRENT]				= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_CURRENT_NOW, 0x14, -32768, 32767),
	[REG_CURRENT_AVG]			= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_CURRENT_AVG, 0x14, -32768, 32767),
	[REG_TIME_TO_EMPTY]			= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0x16, 0, 65535),
	[REG_TIME_TO_FULL]			= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0x18, 0, 65535),
	[REG_STATUS]				= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_STATUS, 0x0a, 0, 65535),
	[REG_CAPACITY_IN_PERCENT]	= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_CAPACITY, 0x2c, 0, 100),
	[REG_CAPACITY_IN_MAH]		= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_ENERGY_NOW, 0x10, 0, 65535), //RM(mAh)
	[REC_FULL_CHARGE_CAPACITY]	= _PROPERTY_AND_REG_SPEC(POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN, 0x12, 0, 65535),
};


static int _pad_battery_read_i2c(u8 reg, int *rt_value, int b_single)
{
	struct i2c_client *client = pad_battery->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static int pad_battery_smbus_read_data(int reg_offset,int is_byte,int *rt_value)
{
     s32 ret=-EINVAL;
     int count=0;

	do {
		ret = _pad_battery_read_i2c(bq27520_property_and_reg_spec[reg_offset].addr, rt_value, is_byte);
	} while((ret<0)&&(++count<=SMBUS_RETRY));

	return ret;
}

static int pad_battery_smbus_write_data(int reg_offset,int byte, unsigned int value)
{
     s32 ret = -EINVAL;
     int count=0;

	do{
		if(byte){
			ret = i2c_smbus_write_byte_data(pad_battery->client,bq27520_property_and_reg_spec[reg_offset].addr,value&0xFF);
		}
		else{
			ret = i2c_smbus_write_word_data(pad_battery->client,bq27520_property_and_reg_spec[reg_offset].addr,value&0xFFFF);
		}
	}while((ret<0) && (++count<=SMBUS_RETRY));
	return ret;
}

static enum pad_battery_cable_type get_this_time_cable_type()
{
	enum pad_battery_cable_type cable_type_now;

	if(!project_is_haydn)
		cable_type_now = this_time_cable_type;
	else{
		if(pad_battery->haydn_AC_in)
			cable_type_now = 	ac_cable;
		else
			cable_type_now = 	this_time_cable_type;
	}

	return cable_type_now;
}

static enum pad_battery_cable_type change_tegra_conntect_type_to_pad_battery_cable_type(
	unsigned long usb_cable_state)
{
	switch(usb_cable_state){
		case CONNECT_TYPE_NONE:
			return non_cable;
			break;

		case CONNECT_TYPE_SDP:
			return usb_cable;
			break;

		case CONNECT_TYPE_DCP:
		case CONNECT_TYPE_CDP:
			return ac_cable;

		default:
			return unknow_cable;
	}
}

void dock_battery_status_timer_func(void)
{
	power_supply_changed(&pad_battery->dock_battery);
}

int for_asuspec_call_me_back_if_dock_in_status_change(bool dock_in)
{
	pad_battery->dock_in = dock_in;

	printk("piter :Battery: call back from EC, dock in:%d\n", dock_in);
	if(pad_battery_by_EC_driver_ready){
		del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
		mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
		power_supply_changed(&pad_battery->battery);
	}

	return 0;
}
EXPORT_SYMBOL(for_asuspec_call_me_back_if_dock_in_status_change);

int for_udc_call_me_back_if_cable_type_change(struct notifier_block *self,
	unsigned long cable_type, void *dev)
{
	enum pad_battery_cable_type old_cable_status;
/*
	if(!pad_battery_by_EC_driver_ready) {
		printk("pad_battery: battery driver not ready\n");
		return 1;
	}
*/
	wake_lock_timeout(&pad_battery->cable_type.cable_type_change_event_wake_lock, 5*HZ);

	cancel_delayed_work(&pad_battery->battery_status_polling_work);

	old_cable_status = this_time_cable_type;
	this_time_cable_type = change_tegra_conntect_type_to_pad_battery_cable_type(cable_type);

       printk("========================================================\n");
	printk("pad_battery:_callback from udc notify, usb_cable_state = %x\n", this_time_cable_type) ;
       printk("========================================================\n");

	if(this_time_cable_type== unknow_cable) {
		if (old_cable_status == ac_cable){
			power_supply_changed(&pad_battery->ac);
			del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
			mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
		}else if (old_cable_status == usb_cable && can_power_supply_by_usb){
			power_supply_changed(&pad_battery->usb);
			del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
			mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
		}
	}else if (this_time_cable_type == usb_cable && can_power_supply_by_usb){
		power_supply_changed(&pad_battery->usb);
		del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
		mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
	}else if (this_time_cable_type == ac_cable){
		power_supply_changed(&pad_battery->ac);
		del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
		mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
	}else if (this_time_cable_type == non_cable){
		power_supply_changed(&pad_battery->battery);
		del_timer_sync(&pad_battery->dock_battery_status_detect_timer);
		mod_timer(&pad_battery->dock_battery_status_detect_timer,jiffies +(5*HZ));
	}

	queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 2*HZ);

	return 0;
}
EXPORT_SYMBOL(for_udc_call_me_back_if_cable_type_change);

static struct notifier_block usb_cable_changed_callback_notifier = {
	.notifier_call = for_udc_call_me_back_if_cable_type_change,
};

static int check_property_value_is_within_spec(enum property_REG property_reg, int property_value){
	if(property_value < bq27520_property_and_reg_spec[property_reg].min_value ||
		property_value > bq27520_property_and_reg_spec[property_reg].max_value)
		return -EINVAL;

	return 1;
}

static int get_battery_capacity_in_mAh(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int EC_return_value ;
	int EC_feedback_error_times = 0;

	do{
		EC_return_value = asuspec_battery_monitor("remaining_capacity", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		return -EINVAL;

	val->intval = EC_return_value;

	return 0;
}

static void modify_real_capacity_for_user_read(int *capacity_value)
{
	int temp_capacity;
	temp_capacity = ((*capacity_value >= 100) ? 100 : *capacity_value);

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

	*capacity_value = temp_capacity;
}

static int get_battery_capacity_in_percent(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int EC_return_value;
	int EC_feedback_error_times = 0;
	bool error = false;
	int tempcapacity = 50;
	bool gaugeIC_firmware_is_updated;

	if(machine_is_haydn())
		pad_gaugeIC_firmware_is_updated = true;

	if(which_battery == pad_device_battery){
		gaugeIC_firmware_is_updated = pad_gaugeIC_firmware_is_updated;
		pad_battery->capacity_in_percent_last_time = pad_battery->capacity_in_percent_this_time;
	}else{
		gaugeIC_firmware_is_updated = dock_gaugeIC_firmware_is_updated;
		pad_battery->dock_capacity_in_percent_last_time = pad_battery->dock_capacity_in_percent_this_time;
	}

	if(gaugeIC_firmware_is_updated){
		do{
			EC_return_value = asuspec_battery_monitor("capacity", which_battery);
		}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

		if(EC_return_value < 0)
			error = true;

		if(which_battery == pad_device_battery){
			if(error == true){
				printk("battery: read pad capacity fail times = %d\n", pad_battery->capacity_get_error_times);
				pad_battery->capacity_get_error_times++;
				if(pad_battery->capacity_get_error_times > 5){
					val->intval = PROPERTY_ERROR_RESET_VALUE_CAPACITY;
					pad_battery->capacity_in_percent_this_time = val->intval;
					return 0;
				}

				EC_return_value = pad_battery->capacity_in_percent_last_time;
			}else
				pad_battery->capacity_get_error_times = 0;
		}else{
			if(error == true){
				printk("battery: read dock capacity fail times = %d\n", pad_battery->dock_capacity_get_error_times);
				pad_battery->dock_capacity_get_error_times++;
				if(pad_battery->dock_capacity_get_error_times > 5){
					val->intval = PROPERTY_ERROR_RESET_VALUE_CAPACITY;
					pad_battery->dock_capacity_in_percent_this_time = val->intval;
					return 0;
				}

				EC_return_value = pad_battery->dock_capacity_in_percent_last_time;
			}else
				pad_battery->dock_capacity_get_error_times = 0;
		}

		modify_real_capacity_for_user_read(&EC_return_value);

		val->intval = EC_return_value;

		if(val->intval < 16 && which_battery == pad_device_battery)
		{
			low_battery_flag = 1;
		}
		else if(val->intval >= 16 && which_battery == pad_device_battery)
		{
			low_battery_flag = 0;
		}

	}else{
		do{
			EC_return_value = asuspec_battery_monitor("voltage", which_battery);
		}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

		pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

		if(EC_return_value >= 4250)
			val->intval = 100;
		else if(EC_return_value >= 4100 && EC_return_value < 4250)
			val->intval = 70;
		else if(EC_return_value >= 4000 && EC_return_value < 4100)
			val->intval = 50;
		else if(EC_return_value >= 3900 && EC_return_value < 4000)
			val->intval = 30;
		else if(EC_return_value >= 3800 && EC_return_value < 3900)
			val->intval = 20;
		else if(EC_return_value < 3800)
			val->intval = 6;

		if(val->intval > 100)
			val->intval = 100;

		if(val->intval < 6)
			val->intval = 6;
	}

	if(val->intval <= 20 && which_battery == pad_device_battery)
	{
		low_battery_flag = 1;
	}
	else if(val->intval > 20 && which_battery == pad_device_battery)
	{
		low_battery_flag = 0;
	}

	if(which_battery == pad_device_battery)
		pad_battery->capacity_in_percent_this_time = val->intval;
	else
		pad_battery->dock_capacity_in_percent_this_time = val->intval;


	return 0;
}

static int get_battery_time_to_full(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int EC_return_value ;
	int EC_feedback_error_times = 0;

	do{
		EC_return_value = asuspec_battery_monitor("avg_time_to_full", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		return -EINVAL;

	val->intval = EC_return_value;

	return 0;
}

static int get_battery_time_to_empty(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int EC_return_value ;
	int EC_feedback_error_times = 0;

	do{
		EC_return_value = asuspec_battery_monitor("avg_time_to_empty", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		return -EINVAL;

	val->intval = EC_return_value;

	return 0;
}

static int get_battery_current(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int EC_return_value ;
	int EC_feedback_error_times = 0;

	do{
		EC_return_value = asuspec_battery_monitor("current", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		return -EINVAL;


	val->intval = (int)(s16)EC_return_value;

	return 0;
}

static int get_battery_voltage(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int EC_return_value ;
	int EC_feedback_error_times = 0;

	do{
		EC_return_value = asuspec_battery_monitor("voltage", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		return -EINVAL;

	val->intval = EC_return_value * 1000;

	return 0;
}

static int get_battery_temperature(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int EC_return_value;
	int EC_feedback_error_times = 0;
	bool error = false;

	pad_battery->temperature_last_time= pad_battery->temperature_this_time;

	do{
		EC_return_value = asuspec_battery_monitor("temperature", which_battery);
	}while(EC_return_value < 0 && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value < 0 ? false : true;

	if(EC_return_value < 0)
		error = true;

	if(EC_return_value > (PROTECT_TEMPERATURE_IN_CELSIUS_HIGH+ KELVIN_BASE) ||
		EC_return_value < PROTECT_TEMPERATURE_IN_CELSIUS_LOW + KELVIN_BASE )
		error = true;

	if(error == true){
		pad_battery->temperature_get_error = true;
		val->intval = pad_battery->temperature_this_time = pad_battery->temperature_last_time;

		printk("bq27520: get temperature error\n");
		return 0;
	}else
		pad_battery->temperature_get_error = false;

	val->intval= EC_return_value -KELVIN_BASE;

	pad_battery->temperature_this_time = val->intval;

	return 0;
}

static int get_battery_health_or_present(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	val->intval = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int get_battery_status(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int EC_return_value_status;
	//int EC_return_value_capacity;
	int EC_feedback_error_times = 0;
	enum pad_battery_cable_type cable_type_now;
	union power_supply_propval ps_temp_value;

	union power_supply_propval ps_temp_current_value;
	enum which_battery_enum temp_get_current_which_battery = pad_device_battery;

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

	if(which_battery == dock_device_battery){
		if(!pad_battery->dock_in){
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
	}

	do{
		EC_return_value_status = asuspec_battery_monitor("status", which_battery);
	}while((EC_return_value_status < 0) && ++EC_feedback_error_times < RETRY_TIMES_IF_EC_FEEDBACK_ERROR);

	pad_battery->chargerIC_is_work = EC_return_value_status < 0 ? false : true;

	if(EC_return_value_status < 0){
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return -EINVAL;
	}

	cable_type_now = get_this_time_cable_type();
	EC_feedback_error_times = 0;

	if(EC_return_value_status & EC_FEEDBACK_STATUS_DISCHARGING ||
		EC_return_value_status & EC_FEEDBACK_STATUS_FULLY_DISCHARGED){
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	 }

	if(EC_return_value_status & EC_FEEDBACK_STATUS_FULLY_CHARGED){
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if(!(EC_return_value_status & EC_FEEDBACK_STATUS_DISCHARGING)){
		if(which_battery == pad_device_battery){
			if(cable_type_now == ac_cable || pad_battery->dock_in == 1){
				if(cable_type_now == ac_cable)
					val->intval = pad_battery->capacity_in_percent_this_time == 100 ?
					POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
				else{
					if(pad_battery->capacity_in_percent_this_time == 100)
						val->intval = POWER_SUPPLY_STATUS_FULL;
					else{
						get_battery_current(&ps_temp_current_value, temp_get_current_which_battery);
						val->intval = ps_temp_current_value.intval < 5 ?
						POWER_SUPPLY_STATUS_NOT_CHARGING : POWER_SUPPLY_STATUS_CHARGING;
					}
				}
			}
			else{
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			return 0;
		}else{	// dock_device_battery
			if(cable_type_now == ac_cable)
				val->intval = pad_battery->dock_capacity_in_percent_this_time == 100 ?
				POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return 0;
		}
	}

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int pad_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery;
	static char *which_battery_text[] = {"Dock", "Pad"};

	if (!strcmp(psy->name, "dock_battery"))
		which_battery = dock_device_battery;
	else
		which_battery = pad_device_battery;


	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (get_battery_health_or_present(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			ps_temp_value.intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (get_battery_capacity_in_percent(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery capacity[%s] = %d (%)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_ENERGY_NOW:
			if(get_battery_capacity_in_mAh(&ps_temp_value, which_battery) < 0 )
				return -EINVAL;
			break;

		case POWER_SUPPLY_PROP_STATUS:
			if(get_battery_status(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			static char *charging_status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
			printk("battery charging status[%s] = %s\n", which_battery_text[which_battery],
				charging_status_text[ps_temp_value.intval]);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			if(get_battery_voltage(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery voltage[%s] = %d (uV)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if(get_battery_current(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery current[%s] = %d (mA)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_CURRENT_AVG:
			if(get_battery_current(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery current[%s] = %d (mA)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			if(get_battery_temperature(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery temperature[%s] = %d (¢XC)\n", which_battery_text[which_battery],
				(ps_temp_value.intval / 10));
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			if(get_battery_time_to_empty(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery time to empty[%s] = %d (min)\n", which_battery_text[which_battery],
				ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			if(get_battery_time_to_full(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery time to full[%s] = %d (min)\n", which_battery_text[which_battery],
				ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
			ps_temp_value.intval = 8180;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			ps_temp_value.intval = 4350;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			ps_temp_value.intval = 3400;
			break;

	       case POWER_SUPPLY_PROP_CURRENT_MAX:
			ps_temp_value.intval = 1800;;
			break;

		default:
			dev_err(&pad_battery->client->dev,
				"%s: INVALID property psp[%s] = %u\n", __func__, which_battery_text[which_battery], psp);
			return -EINVAL;
	}

	val->intval = ps_temp_value.intval;
	return 0;
}

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	union power_supply_propval dock_ps_value;
	enum which_battery_enum which_battery = dock_device_battery;

	get_battery_status(&dock_ps_value, which_battery);

	if(!project_is_haydn){
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  this_time_cable_type == ac_cable)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_USB && this_time_cable_type == usb_cable)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_DOCK_AC &&  this_time_cable_type == ac_cable &&
				dock_ps_value.intval == POWER_SUPPLY_STATUS_CHARGING)
				val->intval =  1;
			else
				val->intval = 0;
			break;

		default:
			return -EINVAL;
		}

		return 0;
	}else{
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  pad_battery->haydn_AC_in == true)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_USB && this_time_cable_type == usb_cable)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_DOCK_AC &&  pad_battery->haydn_AC_in == true &&
				dock_ps_value.intval == POWER_SUPPLY_STATUS_CHARGING)
				val->intval =  1;
			else
				val->intval = 0;
			break;

		default:
			return -EINVAL;
		}

		return 0;
	}

}

static void battery_status_reupdate_at_booting_work_func(void)
{
	// since the power consumption of T40 is too large, so re-update the status.
	// ex: when with dock, the net current may negative at booting, even the dock is charging
	// the pad, so re-update the battery status,
	// or it may show the wrong status at the first glance.

	printk("battery status update at booting\n");
	power_supply_changed(&pad_battery->battery);
}

static bool check_if_dock_gaugeIC_firmware_is_updated_work_func(void)
{
	u16 gaugeIC_firmware_info;
	enum which_battery_enum which_battery = dock_device_battery;

	printk("Battery: check if dock gaugeIC's firmware is updated...\n");

	dock_gaugeIC_firmware_is_updated = true;

	asuspec_start_gauge_firmware_update(which_battery);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DEVICE_TYPE", which_battery);
	if(gaugeIC_firmware_info != 0x0520)
		dock_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("FW_VERSION", which_battery);
	if(gaugeIC_firmware_info != 0x0329)
		dock_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
	if(!(gaugeIC_firmware_info == 0x0000 || gaugeIC_firmware_info == 0x0001))
		dock_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DF_VERSION", which_battery);
	if(gaugeIC_firmware_info < 0x0001 || gaugeIC_firmware_info >= 0x0500)
		dock_gaugeIC_firmware_is_updated = false;

	asuspec_stop_gauge_firmware_update(which_battery);

	printk("Battery: dock gaugeIC's firmware is updated: %d,\n", dock_gaugeIC_firmware_is_updated);

	return dock_gaugeIC_firmware_is_updated;
}

static bool check_if_pad_gaugeIC_firmware_is_updated_work_func(void)
{
	u16 gaugeIC_firmware_info;
	enum which_battery_enum which_battery = pad_device_battery;

	printk("Battery: check if pad gaugeIC's firmware is updated...\n");
	printk("Battery: origin gaugeIC is updated status: %d\n", pad_gaugeIC_firmware_is_updated);

	pad_gaugeIC_firmware_is_updated = true;

	asuspec_start_gauge_firmware_update(which_battery);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DEVICE_TYPE", which_battery);
	if(gaugeIC_firmware_info != 0x0520)
		pad_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("FW_VERSION", which_battery);
	if(gaugeIC_firmware_info != 0x0329)
		pad_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
	if(!(gaugeIC_firmware_info == 0x0000 || gaugeIC_firmware_info == 0x0001))
		pad_gaugeIC_firmware_is_updated = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DF_VERSION", which_battery);
	if(gaugeIC_firmware_info < 0x0001 || gaugeIC_firmware_info >= 0x0500)
		pad_gaugeIC_firmware_is_updated = false;

	asuspec_stop_gauge_firmware_update(which_battery);

	printk("Battery: pad gaugeIC's firmware is updated: %d,\n", pad_gaugeIC_firmware_is_updated);

	return pad_gaugeIC_firmware_is_updated;
}

static void battery_status_polling_work_func(struct work_struct *work)
{
       struct pad_battery_struct *battery_gauge = container_of(work, struct pad_battery_struct, battery_status_polling_work.work);

	printk("battery polling work\n");

	if(!pad_battery_by_EC_driver_ready)
		printk("battery driver not ready\n");

	power_supply_changed(&battery_gauge->battery);

	if(pad_battery->dock_in)
		power_supply_changed(&pad_battery->dock_battery);

	/* Schedule next polling */
	queue_delayed_work(pad_battery_by_EC_workqueue, &battery_gauge->battery_status_polling_work, BATTERY_POLLING_RATE*HZ);
}

static void pad_battery_by_EC_low_low_battery_work_func(struct work_struct *work)
{
	cancel_delayed_work(&pad_battery->battery_status_polling_work);
	queue_delayed_work(pad_battery_by_EC_workqueue,&pad_battery->battery_status_polling_work, 0.1*HZ);
	msleep(2000);
	enable_irq(pad_battery->low_low_battery.irq);
}

static irqreturn_t pad_battery_by_EC_low_low_battery_detect_isr(int irq, void *dev_id)
{
	disable_irq_nosync(pad_battery->low_low_battery.irq);

	pad_battery->low_low_battery.low_low_battery_present = gpio_get_value(pad_battery->low_low_battery.GPIO_pin);
	printk("gpio LL_BAT_T40 = %d\n", pad_battery->low_low_battery.low_low_battery_present);

	wake_lock_timeout(&pad_battery->low_low_battery.low_low_battery_wake_lock, 10*HZ);
	queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->low_low_battery_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int pad_battery_by_EC_setup_low_low_battery_detect_irq()
{
	if(gpio_request(pad_battery->low_low_battery.GPIO_pin , "low_low_battery_detect") < 0){
		printk("pad_battery_by_EC error: gpio LL_BAT_T40 request failed\n");
		return -EINVAL;
	}

	if(gpio_direction_input(pad_battery->low_low_battery.GPIO_pin) < 0){
		printk("pad_battery_by_EC error: gpio LL_BAT_T40 unavailable for input\n");
		return -EINVAL;
	}

	if(request_irq(pad_battery->low_low_battery.irq, pad_battery_by_EC_low_low_battery_detect_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "pad battery by EC (low low battery)", NULL) < 0){
		printk("pad_battery_by_EC error: gpio LL_BAT_T40 irq request failed\n");
		return -EINVAL;
	}

	pad_battery->low_low_battery.low_low_battery_present = gpio_get_value(pad_battery->low_low_battery.GPIO_pin);

	return 0;
}

static void pad_battery_by_EC_haydn_AC_ok_work_func(struct work_struct *work)
{
	printk("haydn AC OK work function\n");

	cancel_delayed_work(&pad_battery->battery_status_polling_work);

	printk("========================================================\n");
	printk("         pad_battery: haydn AC in status: %s\n", pad_battery->haydn_AC_in? "AC in":"without AC");
       printk("========================================================\n");

	if(pad_battery->haydn_AC_in)
		power_supply_changed(&pad_battery->ac);

	power_supply_changed(&pad_battery->battery);

	queue_delayed_work(pad_battery_by_EC_workqueue,&pad_battery->battery_status_polling_work, 2*HZ);

	enable_irq(pad_battery->haydn_AC_ok.irq);
}

static irqreturn_t pad_battery_by_EC_haydn_setup_AC_ok_isr(int irq, void *dev_id)
{
	disable_irq_nosync(pad_battery->haydn_AC_ok.irq);

	pad_battery->haydn_AC_in = !gpio_get_value(pad_battery->haydn_AC_ok.GPIO_pin);
	printk("haydn AC in = %d, gpio value: %d\n", pad_battery->haydn_AC_in, gpio_get_value(pad_battery->haydn_AC_ok.GPIO_pin));

	wake_lock_timeout(&pad_battery->haydn_AC_ok.AC_ok_wake_lock, 10*HZ);
	queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->haydn_AC_ok_work, 1.8*HZ);
	return IRQ_HANDLED;
}

static int pad_battery_by_EC_haydn_setup_AC_ok_irq(void)
{

	if(gpio_request(pad_battery->haydn_AC_ok.GPIO_pin, "battery_haydn_AC_ok") < 0){
		printk("pad_battery_by_EC error: gpio haydn_AC_ok request failed\n");
		return -EINVAL;
	}

	if(gpio_direction_input(pad_battery->haydn_AC_ok.GPIO_pin) < 0){
		printk("pad_battery_by_EC error: gpio haydn_AC_ok unavailable for input\n");
		return -EINVAL;
	}

	if(request_irq(pad_battery->haydn_AC_ok.irq, pad_battery_by_EC_haydn_setup_AC_ok_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "battery_haydn_AC_ok", NULL) < 0){
		printk("pad_battery_by_EC error: gpio haydn_AC_ok irq request failed\n");
		return -EINVAL;
	}

	return 0;
}


static enum gaugeIC_update_ret_value change_EC_ret_value_to_gaugeIC_update_ret_value(
	int EC_return_value)
{
	if(EC_return_value <= -4)
		return EC_error_unknow;
	else if(EC_return_value >= 0)
		return update_success;
	else{
		switch(EC_return_value){
			case -1:
				return EC_i2c_busy;
				break;

			case -2:
				return EC_i2c_router_error;
				break;

			case -3:
				return gaugeIC_check_error;
				break;

			default:
				return EC_error_unknow;
				break;
		}
	}
}

static enum gaugeIC_update_ret_value analyze_dffs_string_to_hexadecimal_and_send_data(
	char dffs_string[], int string_length, enum which_battery_enum which_battery)
{
	char temp_dffs_string[string_length];
	char *temp_dffs_string_for_strsep_use;
	char *delim_token = " ";
	char *temp_char;
	u8 temp_result_array[string_length];
	int send_byte_count = 0;
	int j = 0;
	int wait_time = 0;
	int EC_return_vaule = 0;
	int re_send_to_EC_count = 0;
	enum gaugeIC_update_ret_value gaugeIC_update_return_value;
	#define RE_SEND_TO_EC_TIMES (3)

       memcpy(temp_dffs_string, dffs_string, sizeof(temp_dffs_string));

       switch(temp_dffs_string[0]){
	       case '#':
		   	// dffs's comment format is like:: # Date_2013_6_11
			// abstract the string Date_2013_6_11

			temp_dffs_string_for_strsep_use = temp_dffs_string;

			temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
			temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);

			printk("dffs_string: comment: %s\n", temp_char);

			break;

		case 'W':
			// dffs's write format is like:: W: 16 00 08

			temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

			send_byte_count = 0;
			for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
				temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
				temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

				send_byte_count++;
			}

			printk("dffs_string: write %d bytes: ", send_byte_count);
			j = 0;
			for(j = 0; j < send_byte_count; j++)
				printk("%02x ", temp_result_array[j]);

			printk("\n");

			do{
				EC_return_vaule = asuspec_gauge_firmware_update(0x01, temp_result_array, send_byte_count,
					which_battery);
				re_send_to_EC_count++;
			} while((EC_return_vaule == -1) && (re_send_to_EC_count < RE_SEND_TO_EC_TIMES));

			gaugeIC_update_return_value = change_EC_ret_value_to_gaugeIC_update_ret_value(EC_return_vaule);
			if(gaugeIC_update_return_value != 0)
				return EC_return_vaule;

			break;

		case 'C':
			// dffs's check format is like:: C: 16 04 34 11 3F A7

			temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

			send_byte_count = 0;
			for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
				temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
				temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

				send_byte_count++;
			}

			printk("dffs_string: check %d bytes: ", send_byte_count);
			j = 0;
			for(j = 0; j < send_byte_count; j++)
				printk("%02x ", temp_result_array[j]);

			printk("\n");

			do{
				EC_return_vaule = asuspec_gauge_firmware_update(0x00, temp_result_array,send_byte_count,
					which_battery);
				re_send_to_EC_count++;
			}while(EC_return_vaule == -1 && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

			if(EC_return_vaule < 0)
				return EC_return_vaule;

			break;

		case 'X':
			// dffs's check format is like:: X: 170

			temp_dffs_string_for_strsep_use = temp_dffs_string;

			temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);
			temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);

			wait_time = (int)simple_strtol(temp_char, NULL, 10);
			temp_result_array[0] = wait_time / 256;
			temp_result_array[1] = wait_time % 256;
			printk("dffs_string: wait: %02x, %02x\n", temp_result_array[0], temp_result_array[1]);

			do{
				EC_return_vaule = asuspec_gauge_firmware_update(0x02, temp_result_array, 2,
					which_battery);
				re_send_to_EC_count++;
			}while(EC_return_vaule == -1 && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

			if(EC_return_vaule < 0)
				return EC_return_vaule;

			break;
		default:
			printk("error\n");

			break;
	}

	return 0;
}

static int __update_gaugeIC_firmware_data_flash_blocks(struct file *fp,
	enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	int i = 0;
	int dffs_line_count = 0;
	int ret;
	int gaugeIC_check_error_count = 0;
	char temp_char;
	char dffs_string[max_dffs_string_length];
	enum gaugeIC_update_ret_value gaugeIC_update_return_value;
	#define F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR (0)
	#define GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES (5)

	printk("gaugeIC: update data flash blocks...\n");

	if (!(fp->f_op) || !(fp->f_op->read)){
		printk("gaugeIC: update data flash blocks error: no dffs file operation\n");
		return -EINVAL;
	}

	do{
		ret = fp->f_op->read(fp,&temp_char,1, &fp->f_pos);
		if(ret > 0){
			if(temp_char != '\n'){
				dffs_string[i] = temp_char;

				i++;
			}else{
				dffs_string[i] = '\0';

				printk("line %d: %s\n", dffs_line_count, dffs_string);

				gaugeIC_update_return_value =analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
												max_dffs_string_length, which_battery);

				switch(gaugeIC_update_return_value){
					case gaugeIC_check_error:
					case EC_i2c_router_error:
					case EC_i2c_busy:
						printk("pad_battery: gaugeIC firmware update: check error\n");
						fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
						gaugeIC_check_error_count++;
						i = 0;
						dffs_line_count = 0;

						if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
							return -EINVAL;

						break;

					case EC_error_unknow:
						printk("pad_battery: gaugeIC firmware update: Major errors\n");
						return -EINVAL;
						break;

					case update_success:
						printk("pad_battery: gaugeIC firmware update: success\n");
						i = 0;
						dffs_line_count++;
						break;

					default:
						printk("pad_battery: gaugeIC firmware update: no this case\n");
						return -EINVAL;
						break;
				}

			}
		}else{
			if(i != 0){
				dffs_string[i] = '\0';

				printk("line %d: %s\n", dffs_line_count, dffs_string);

				gaugeIC_update_return_value = analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
												max_dffs_string_length, which_battery);

				switch(gaugeIC_update_return_value){
					case gaugeIC_check_error:
					case EC_i2c_router_error:
					case EC_i2c_busy:
						printk("pad_battery: gaugeIC firmware update: check error\n");
						fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
						gaugeIC_check_error_count++;
						i = 0;
						dffs_line_count = 0;
						ret = 1;

						if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
							return -EINVAL;

						break;

					case EC_error_unknow:
						printk("pad_battery: gaugeIC firmware update: Major errors\n");
						return -EINVAL;
						break;

					case update_success:
						printk("pad_battery: gaugeIC firmware update: success\n");
						i = 0;
						dffs_line_count++;
						break;

					default:
						printk("pad_battery: gaugeIC firmware update: no this case\n");
						return -EINVAL;
						break;
				}

			}
		}
	}while(ret > 0);


	printk("gaugeIC: update data flash blocks success\n");

	return 0;
}

static int gaugeIC_exit_ROM_mode(enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	char dffs_string[max_dffs_string_length];

	printk("gaugeIC: exit ROM mode...\n");

	strcpy(dffs_string, "W: 16 00 0F");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: 16 64 0F 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 4000");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: AA 00 20 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: AA 00 20 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	printk("gaugeIC: exit ROM mode success\n");

	return 0;
}

static int gaugeIC_enter_ROM_mode(enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	char dffs_string[max_dffs_string_length];

	printk("gaugeIC: enter ROM mode...\n");

	strcpy(dffs_string, "W: AA 00 14 04");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;


	strcpy(dffs_string, "W: AA 00 72 16");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: AA 00 FF FF");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: AA 00 FF FF");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "W: AA 00 00 0F");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;

	printk("gaugeIC: enter ROM mode success\n");

	return 0;
}

static bool check_if_gaugeIC_in_ROM_mode(enum which_battery_enum which_battery)
{
	// from ec's spec
	// 0: not normal mode and not rom mode. the gauge may broken.
	// 1: gaugeIC in rom mode.
	// 2: gaugeIC in normal mode.

	int gaugeIC_mode = 0;

	gaugeIC_mode = asuspec_get_gauge_mode(which_battery);

	return (gaugeIC_mode == 2) ? false : true;	// since sometimes it will check error, so if not normal, feedback it is in rom mode
}

static int update_gaugeIC_firmware(struct file *fp, enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	int ret = 0;
	char temp_char;
	char dffs_string[max_dffs_string_length];
	u8 dffs_string_in_hexadecimal[max_dffs_string_length];

	if(!check_if_gaugeIC_in_ROM_mode(which_battery))
		ret = gaugeIC_enter_ROM_mode(which_battery);

	if(ret !=0)
		return -EINVAL;

	ret = __update_gaugeIC_firmware_data_flash_blocks(fp, which_battery);

	if(ret != 0)
		return -EINVAL;

	ret = gaugeIC_exit_ROM_mode(which_battery);

	return ret;
}

static int close_file(struct file *fp)
{
	filp_close(fp,NULL);

	return 0;
}

static struct file *open_file(char *path,int flag,int mode)
{
	struct file *fp;

	fp = filp_open(path, flag, 0);

	if(fp) return fp;
	else return NULL;
}

static void exit_kernel_to_read_file(mm_segment_t origin_fs)
{
	set_fs(origin_fs);
}

static mm_segment_t enable_kernel_to_read_file_and_return_origin_fs(void)
{
	mm_segment_t old_fs;

	old_fs = get_fs();

	set_fs(KERNEL_DS);

	return old_fs;
}

static ssize_t sysfs_pad_battery_by_EC_get_dock_gaugeIC_mode_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	// from ec's spec
	// 0: not normal mode and not rom mode. the gauge may broken.
	// 1: gaugeIC in rom mode.
	// 2: gaugeIC in normal mode.
	enum which_battery_enum which_battery = dock_device_battery;
	int gaugeIC_mode = 0;
	char gaugeIC_mode_in_sentence[15];

	gaugeIC_mode = asuspec_get_gauge_mode(which_battery);
	printk("dock gaugeIC mode: %d\n", gaugeIC_mode);
	switch(gaugeIC_mode){
		case 0: // not normal mode and not rom mode. the gauge may broken.
			strcpy(gaugeIC_mode_in_sentence, "may broken");
			break;
		case 1: // gaugeIC in rom mode.
			strcpy(gaugeIC_mode_in_sentence, "rom mode");
			break;
		case 2: // gaugeIC in normal mode.
			strcpy(gaugeIC_mode_in_sentence, "normal mode");
			break;
		default: // no this case
			strcpy(gaugeIC_mode_in_sentence, "no this case");
	}

	return sprintf(buf, "dock gaugeIC mode: %s\n", gaugeIC_mode_in_sentence);
}

static ssize_t sysfs_pad_battery_by_EC_get_pad_gaugeIC_mode_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	// from ec's spec
	// 0: not normal mode and not rom mode. the gauge may broken.
	// 1: gaugeIC in rom mode.
	// 2: gaugeIC in normal mode.
	enum which_battery_enum which_battery = pad_device_battery;
	int gaugeIC_mode = 0;
	char gaugeIC_mode_in_sentence[15];

	gaugeIC_mode = asuspec_get_gauge_mode(which_battery);
	printk("pad gaugeIC mode: %d\n", gaugeIC_mode);
	switch(gaugeIC_mode){
		case 0: // not normal mode and not rom mode. the gauge may broken.
			strcpy(gaugeIC_mode_in_sentence, "may broken");
			break;
		case 1: // gaugeIC in rom mode.
			strcpy(gaugeIC_mode_in_sentence, "rom mode");
			break;
		case 2: // gaugeIC in normal mode.
			strcpy(gaugeIC_mode_in_sentence, "normal mode");
			break;
		default: // no this case
			strcpy(gaugeIC_mode_in_sentence, "no this case");
	}

	return sprintf(buf, "pad gaugeIC mode: %s\n", gaugeIC_mode_in_sentence);
}

static ssize_t sysfs_pad_battery_by_EC_pad_charging_and_with_dock_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	u16 pad_charging_and_with_dock = 0;
	enum which_battery_enum which_battery = pad_device_battery;

	pad_charging_and_with_dock = asuspec_battery_monitor("is_pad_charging_and_with_dock", which_battery);

	return sprintf(buf, "%d\n", pad_charging_and_with_dock);
}

static ssize_t sysfs_pad_battery_by_EC_dock_gaugeIC_FW_version_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	u16 gaugeIC_firmware_info;
	char gaugeIC_name[10];
	char gaugeIC_FW_version[10];
	u16 gaugeIC_DF_version;
	u16 gaugeIC_battery_ID_from_pin;
	u16 gaugeIC_battery_ID_from_gaugeIC;
	int cycle_count;
	enum which_battery_enum which_battery = dock_device_battery;

	asuspec_start_gauge_firmware_update(which_battery);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DEVICE_TYPE", which_battery);
	printk("gaugeIC:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0520)
		strcpy(gaugeIC_name, "bq27520");
	else
		strcpy(gaugeIC_name, "unknow");

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("FW_VERSION", which_battery);
	printk("FW_VERSION:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0329)
		strcpy(gaugeIC_FW_version, "G4");
	else
		strcpy(gaugeIC_FW_version, "unknow");

	gaugeIC_DF_version = asuspec_gauge_ic_monitor("DF_VERSION", which_battery);
	printk("DF_VERSION:%x\n", gaugeIC_DF_version);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
	printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0000)
		gaugeIC_battery_ID_from_pin = 0x0167;
	else if(gaugeIC_firmware_info == 0x0001)
		gaugeIC_battery_ID_from_pin = 0x0251;
	else
		gaugeIC_battery_ID_from_pin = 0xFFFF;

	gaugeIC_battery_ID_from_gaugeIC = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_GAUGEIC", which_battery);
		printk("BATTERY_ID_FROM_GAUGEIC:%x\n", gaugeIC_battery_ID_from_gaugeIC);

	cycle_count = asuspec_battery_monitor("cycle_count", which_battery);
		printk("cycle_count = %d", cycle_count);

	asuspec_stop_gauge_firmware_update(which_battery);

	return sprintf(buf, "dock gaugeIC: %s_%s, DF_version: %x, battery ID: %x, battery ID in gaugeIC: %x, cycle count = %d\n", gaugeIC_name, gaugeIC_FW_version,
			gaugeIC_DF_version, gaugeIC_battery_ID_from_pin, gaugeIC_battery_ID_from_gaugeIC, cycle_count);
}

static ssize_t sysfs_pad_battery_by_EC_gaugeIC_FW_version_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	u16 gaugeIC_firmware_info;
	char gaugeIC_name[10];
	char gaugeIC_FW_version[10];
	u16 gaugeIC_DF_version;
	u16 gaugeIC_battery_ID_from_pin;
	u16 gaugeIC_battery_ID_from_gaugeIC;
	int cycle_count;
	enum which_battery_enum which_battery = pad_device_battery;

	asuspec_start_gauge_firmware_update(which_battery);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DEVICE_TYPE", which_battery);
	printk("gaugeIC:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0520)
		strcpy(gaugeIC_name, "bq27520");
	else
		strcpy(gaugeIC_name, "unknow");

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("FW_VERSION", which_battery);
	printk("FW_VERSION:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0329)
		strcpy(gaugeIC_FW_version, "G4");
	else
		strcpy(gaugeIC_FW_version, "unknow");

	gaugeIC_DF_version = asuspec_gauge_ic_monitor("DF_VERSION", which_battery);
	printk("DF_VERSION:%x\n", gaugeIC_DF_version);

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
	printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0000)
		gaugeIC_battery_ID_from_pin = 0x0368;
	else if(gaugeIC_firmware_info == 0x0001)
		gaugeIC_battery_ID_from_pin = 0x0335;
	else
		gaugeIC_battery_ID_from_pin = 0xFFFF;

	gaugeIC_battery_ID_from_gaugeIC = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_GAUGEIC", which_battery);
		printk("BATTERY_ID_FROM_GAUGEIC:%x\n", gaugeIC_battery_ID_from_gaugeIC);

	cycle_count = asuspec_battery_monitor("cycle_count", which_battery);
		printk("cycle_count = %d", cycle_count);

	asuspec_stop_gauge_firmware_update(which_battery);

	return sprintf(buf, "pad gaugeIC: %s_%s, DF_version: %x, battery ID: %x, battery ID in gaugeIC: %x, cycle count = %d\n", gaugeIC_name, gaugeIC_FW_version,
			gaugeIC_DF_version, gaugeIC_battery_ID_from_pin, gaugeIC_battery_ID_from_gaugeIC, cycle_count);
}

static ssize_t sysfs_pad_battery_by_EC_battery_capacity_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery = pad_device_battery;

	get_battery_capacity_in_percent(&ps_temp_value, which_battery);

	return sprintf(buf, "%d\n", ps_temp_value.intval);
}

static ssize_t sysfs_pad_battery_by_EC_chargerIC_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", pad_battery->chargerIC_is_work);
}

static ssize_t sysfs_pad_battery_by_EC_battery_current_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery = pad_device_battery;

	get_battery_current(&ps_temp_value, which_battery);

	return sprintf(buf, "%d\n", ps_temp_value.intval);
}

static ssize_t sysfs_pad_battery_by_EC_charge_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery = pad_device_battery;

	get_battery_status(&ps_temp_value, which_battery);

	switch(ps_temp_value.intval){
		case POWER_SUPPLY_STATUS_UNKNOWN:
			return sprintf(buf, "%d\n", 1);
			break;

		case POWER_SUPPLY_STATUS_CHARGING:
			return sprintf(buf, "%d\n", 2);
			break;

		case POWER_SUPPLY_STATUS_DISCHARGING:
			return sprintf(buf, "%d\n", 3);
			break;

		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			return sprintf(buf, "%d\n", 4);
			break;

		case POWER_SUPPLY_STATUS_FULL:
			return sprintf(buf, "%d\n", 5);
			break;

		default:
			return sprintf(buf, "%d\n", 1);
			break;

	}

}

static bool check_if_can_update_gaugeIC_firmware(enum which_battery_enum which_battery)
{
	u16 gaugeIC_firmware_info;
	bool can_update_gaugeIC_firmware = true;

	asuspec_start_gauge_firmware_update(which_battery);

	if(check_if_gaugeIC_in_ROM_mode(which_battery)){
		asuspec_stop_gauge_firmware_update(which_battery);
		return true;
	}

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("DEVICE_TYPE", which_battery);
	printk("gaugeIC:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info != 0x0520)
		can_update_gaugeIC_firmware = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("FW_VERSION", which_battery);
	printk("FW_VERSION:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info != 0x0329)
		can_update_gaugeIC_firmware = false;

	gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
	printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
	if(!(gaugeIC_firmware_info == 0x0000 || gaugeIC_firmware_info == 0x0001))
		can_update_gaugeIC_firmware = false;

	asuspec_stop_gauge_firmware_update(which_battery);

	return can_update_gaugeIC_firmware;
}

static ssize_t sysfs_pad_battery_by_EC_bq27520_FW_update_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long buf_type_in_num;
	int return_value;
	int ret;
	char read_buf[1024];
	struct file *fp;

	u8 temp_result_array[256];
	u16 gaugeIC_firmware_info;

	mm_segment_t old_fs;
	int max_dffs_string_length = 256;
	char dffs_string[max_dffs_string_length];

	enum which_battery_enum which_battery = pad_device_battery;

	if ( strict_strtol(buf, 0, &buf_type_in_num)){
		printk("pad_battery_by_EC: string to num fail\n");
		return -EINVAL;
	}

	if (buf_type_in_num == 5){		
		// update pad gaugeIC's firmware by dffs file in system image

		which_battery = pad_device_battery;
		
		cancel_delayed_work(&pad_battery->battery_status_polling_work);

		printk("Battery: pad gaugeIC firmware updating...\n");

		if(!check_if_can_update_gaugeIC_firmware(which_battery)){
			printk("wrong gaugeIC, cannot update firmware\n");
			strcpy(gaugeIC_firmware_message, "wrong gaugeIC, cannot update firmware\n");
			return count;
		}

		wake_lock_timeout(&pad_battery->gaugeIC_updating, 150*HZ);

		asuspec_start_gauge_firmware_update(which_battery);

		old_fs = enable_kernel_to_read_file_and_return_origin_fs();

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info == 0x0001)
			fp = open_file("/system/etc/firmware/battery_gauge/mozart_pad_0335.dffs", O_RDONLY, 0);
		else if(gaugeIC_firmware_info == 0x0000)
			fp = open_file("/system/etc/firmware/battery_gauge/mozart_pad_0368.dffs", O_RDONLY, 0);
		else
			fp = NULL;

		if (fp!=NULL){
			ret = update_gaugeIC_firmware(fp, which_battery);
			if(ret == 0){
				printk("gaugeIC firware update success\n");
				strcpy(gaugeIC_firmware_message, "pad gaugeIC firware update success\n");
				pad_gaugeIC_firmware_is_updated = true;
			}
			else{
				printk("gaugeIC firmware update fail\n");
				strcpy(gaugeIC_firmware_message, "pad gaugeIC firmware update fail\n");
				pad_gaugeIC_firmware_is_updated = false;
			}

			close_file(fp);
		}else{
			printk("pad gaugeIC firmware update fail: no file\n");
			strcpy(gaugeIC_firmware_message, "gaugeIC firmware update fail: no file\n");
		}

		exit_kernel_to_read_file(old_fs);

		asuspec_stop_gauge_firmware_update(which_battery);

		queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 1*HZ);
	}

	if (buf_type_in_num == 6){
		
		// update dock gaugeIC's firmware by dffs file in system image

		which_battery = dock_device_battery;

		cancel_delayed_work(&pad_battery->battery_status_polling_work);

		printk("Battery: dock gaugeIC firmware updating...\n");

		if(!check_if_can_update_gaugeIC_firmware(which_battery)){
			printk("wrong gaugeIC, cannot update firmware\n");
			strcpy(gaugeIC_firmware_message, "wrong gaugeIC, cannot update firmware\n");
			return count;
		}

		wake_lock_timeout(&pad_battery->gaugeIC_updating, 150*HZ);

		asuspec_start_gauge_firmware_update(which_battery);

		old_fs = enable_kernel_to_read_file_and_return_origin_fs();

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info == 0x0000)
			fp = open_file("/system/etc/firmware/battery_gauge/mozart_dock_0167.dffs", O_RDONLY, 0);
		else if(gaugeIC_firmware_info == 0x0001)
			fp = open_file("/system/etc/firmware/battery_gauge/mozart_dock_0251.dffs", O_RDONLY, 0);
		else
			fp = NULL;

		if (fp!=NULL){
			ret = update_gaugeIC_firmware(fp, which_battery);
			if(ret == 0){
				printk("gaugeIC firware update success\n");
				strcpy(gaugeIC_firmware_message, "dock gaugeIC firware update success\n");
				dock_gaugeIC_firmware_is_updated = true;
			}
			else{
				printk("gaugeIC firmware update fail\n");
				strcpy(gaugeIC_firmware_message, "dock gaugeIC firmware update fail\n");
				dock_gaugeIC_firmware_is_updated = false;
			}

			close_file(fp);
		}else{
			printk("gaugeIC firmware update fail: no file\n");
			strcpy(gaugeIC_firmware_message, "dock gaugeIC firmware update fail: no file\n");
		}

		exit_kernel_to_read_file(old_fs);

		asuspec_stop_gauge_firmware_update(which_battery);

		queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 1*HZ);
	}

	if (buf_type_in_num == 7){
		which_battery = pad_device_battery;

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_GAUGEIC", which_battery);
		printk("piter: BATTERY_ID_FROM_GAUGEIC:%x\n", gaugeIC_firmware_info);

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		printk("piter: BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);

		return_value = asuspec_battery_monitor("cycle_count", which_battery);
		printk("piter: cycle_count = %d", return_value);
	}

	if (buf_type_in_num == 8){
		which_battery = dock_device_battery;

		//check_if_dock_gaugeIC_firmware_is_updated_work_func();

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_GAUGEIC", which_battery);
		printk("piter: BATTERY_ID_FROM_GAUGEIC:%x\n", gaugeIC_firmware_info);

		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		printk("piter: BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);

		return_value = asuspec_battery_monitor("cycle_count", which_battery);
		printk("piter: cycle_count = %d", return_value);
	}

	if (buf_type_in_num == 9){
		//which_battery = pad_device_battery;
		which_battery = dock_device_battery;
		
		asuspec_start_gauge_firmware_update(which_battery);

		gaugeIC_exit_ROM_mode(which_battery);

		asuspec_stop_gauge_firmware_update(which_battery);
	}

	if (buf_type_in_num == 2){
		// for default gaugeIC firmware unseal. since new gaugeIC is unsealed.
		// this is the key of default gaugeIC's firmware

		which_battery = pad_device_battery;
		
		cancel_delayed_work(&pad_battery->battery_status_polling_work);

		msleep(100);

		asuspec_start_gauge_firmware_update(which_battery);

		strcpy(dffs_string, "W: AA 00 14 04");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		strcpy(dffs_string, "W: AA 00 72 36");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		strcpy(dffs_string, "X: 20");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		strcpy(dffs_string, "W: AA 00 FF FF");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		strcpy(dffs_string, "W: AA 00 FF FF");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		strcpy(dffs_string, "X: 20");
		if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
			which_battery) != update_success)
			return -EINVAL;

		asuspec_stop_gauge_firmware_update(which_battery);

		check_if_pad_gaugeIC_firmware_is_updated_work_func();

		queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 0.5*HZ);
	}

	if (buf_type_in_num == 3){
		printk("Battery: gaugeIC firmware updating...\n");
		strcpy(gaugeIC_firmware_message, "gaugeIC firmware updating...\n");
	}

	if (buf_type_in_num == 4){
		which_battery = pad_device_battery;

		asuspec_start_gauge_firmware_update(which_battery);
		if(check_if_gaugeIC_in_ROM_mode(which_battery))
			printk("piter: is rom mode\n");
		else
			printk("piter: is not rom mode\n");

		asuspec_stop_gauge_firmware_update(which_battery);
	}

	if (buf_type_in_num == 1){
		which_battery = pad_device_battery;

		cancel_delayed_work(&pad_battery->battery_status_polling_work);

		if(!check_if_can_update_gaugeIC_firmware(which_battery)){
			printk("wrong gaugeIC, cannot update firmware\n");
			strcpy(gaugeIC_firmware_message, "wrong gaugeIC, cannot update firmware\n");
			return count;
		}

		asuspec_start_gauge_firmware_update(which_battery);

		old_fs = enable_kernel_to_read_file_and_return_origin_fs();

		fp=open_file("/data/TF501T-SR_v0_2.dffs", O_RDONLY, 0);
		if (fp!=NULL){
			ret = update_gaugeIC_firmware(fp, which_battery);
			if(ret == 0){
				printk("gaugeIC firware update success\n");
				strcpy(gaugeIC_firmware_message, "gaugeIC firware update success\n");
				pad_gaugeIC_firmware_is_updated = true;
			}
			else{
				printk("gaugeIC firmware update fail\n");
				strcpy(gaugeIC_firmware_message, "gaugeIC firmware update fail\n");
				pad_gaugeIC_firmware_is_updated = false;
			}

			close_file(fp);
		}else{
			printk("gaugeIC firmware update fail: no file\n");
			strcpy(gaugeIC_firmware_message, "gaugeIC firmware update fail: no file\n");
		}

		exit_kernel_to_read_file(old_fs);

		asuspec_stop_gauge_firmware_update(which_battery);

		queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 1*HZ);
	}

	pad_battery_by_EC_bq27520_FW_updating_enable = (bool)buf_type_in_num;


	if (buf_type_in_num == 12){
		printk("piter: %d%02x\n", buf_type_in_num, delay_time);
	}

	return count;
}

static ssize_t sysfs_pad_battery_by_EC_bq27520_FW_update_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", gaugeIC_firmware_message);
}

static DEVICE_ATTR(pad_battery_by_EC_get_dock_gaugeIC_mode, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_get_dock_gaugeIC_mode_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_get_pad_gaugeIC_mode, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_get_pad_gaugeIC_mode_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_pad_charging_and_with_dock, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_pad_charging_and_with_dock_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_dock_gaugeIC_FW_version, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_dock_gaugeIC_FW_version_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_gaugeIC_FW_version, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_gaugeIC_FW_version_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_battery_capacity, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_battery_capacity_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_chargerIC_status, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_chargerIC_status_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_battery_current, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_battery_current_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_charge_status, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_charge_status_show, NULL);
static DEVICE_ATTR(pad_battery_by_EC_bq27520_FW_update_enable, S_IWUSR | S_IRUGO,
	sysfs_pad_battery_by_EC_bq27520_FW_update_enable_show,
	sysfs_pad_battery_by_EC_bq27520_FW_update_enable_store);


static struct attribute *pad_battery_by_EC_attributes[] = {
	&dev_attr_pad_battery_by_EC_get_dock_gaugeIC_mode.attr,
	&dev_attr_pad_battery_by_EC_get_pad_gaugeIC_mode.attr,
	&dev_attr_pad_battery_by_EC_pad_charging_and_with_dock.attr,
	&dev_attr_pad_battery_by_EC_dock_gaugeIC_FW_version.attr,
	&dev_attr_pad_battery_by_EC_gaugeIC_FW_version.attr,
	&dev_attr_pad_battery_by_EC_battery_capacity.attr,
	&dev_attr_pad_battery_by_EC_chargerIC_status.attr,
	&dev_attr_pad_battery_by_EC_battery_current.attr,
	&dev_attr_pad_battery_by_EC_bq27520_FW_update_enable.attr,
	&dev_attr_pad_battery_by_EC_charge_status.attr,
	NULL
};

static const struct attribute_group pad_battery_by_EC_group = {
	.attrs = pad_battery_by_EC_attributes,
};

static int pad_battery_by_EC_probe(struct i2c_client *client,	const struct i2c_device_id *id)
{
	printk("pad_battery_by_EC probe\n");

	pad_battery = devm_kzalloc(&client->dev, sizeof(*pad_battery), GFP_KERNEL);
	if(!pad_battery)
		return -ENOMEM;

	i2c_set_clientdata(client,pad_battery);

	project_is_haydn = machine_is_haydn();

	mutex_init(&pad_battery->mutex_lock);
	pad_battery->client = client;
	pad_battery->capacity_in_percent_this_time = INITIAL_CAPACITY_VALUE;
	pad_battery->capacity_in_percent_last_time = INITIAL_CAPACITY_VALUE;
	pad_battery->dock_capacity_in_percent_this_time = INITIAL_CAPACITY_VALUE;
	pad_battery->dock_capacity_in_percent_last_time = INITIAL_CAPACITY_VALUE;
	pad_battery->temperature_this_time = INITIAL_TEMPERATURE_VALUE;
	pad_battery->temperature_last_time = INITIAL_TEMPERATURE_VALUE;
	pad_battery->capacity_get_error_times = 0;
	pad_battery->dock_capacity_get_error_times = 0;
	pad_battery->temperature_get_error = false;
	pad_battery->low_low_battery.low_low_battery_present= false;
	pad_battery->low_low_battery.GPIO_pin = GPIO_PIN_LOW_BATTERY_DETECT;
	pad_battery->low_low_battery.irq = gpio_to_irq(pad_battery->low_low_battery.GPIO_pin);
	pad_battery->cable_type.this_time_cable_type = non_cable;
	pad_battery->cable_type.last_time_cable_type = non_cable;
	pad_battery->chargerIC_is_work = false;

	setup_timer(&pad_battery->dock_battery_status_detect_timer, dock_battery_status_timer_func, 0);

	CONFIG_POWER_SUPPLY_BATTERY(pad_battery->battery);
	CONFIG_POWER_SUPPLY_AC(pad_battery->ac);
	CONFIG_POWER_SUPPLY_USB(pad_battery->usb);
	CONFIG_POWER_SUPPLY_DOCK_BATTERY(pad_battery->dock_battery);
	CONFIG_POWER_SUPPLY_AC_FROM_DOCK(pad_battery->dock_ac);

	if(power_supply_register(&client->dev, &pad_battery->battery ) < 0)
		return -EINVAL;

	if(power_supply_register(&client->dev, &pad_battery->ac) < 0){
		power_supply_unregister(&pad_battery->battery);
		return -EINVAL;
	}

	if(power_supply_register(&client->dev, &pad_battery->usb) < 0){
		power_supply_unregister(&pad_battery->battery );
		power_supply_unregister(&pad_battery->ac);
		return -EINVAL;
	}

	if(power_supply_register(&client->dev, &pad_battery->dock_battery) < 0){
		power_supply_unregister(&pad_battery->battery );
		power_supply_unregister(&pad_battery->ac);
		power_supply_unregister(&pad_battery->usb);
		return -EINVAL;
	}

	if(power_supply_register(&client->dev, &pad_battery->dock_ac) < 0){
		power_supply_unregister(&pad_battery->battery );
		power_supply_unregister(&pad_battery->ac);
		power_supply_unregister(&pad_battery->usb);
		power_supply_unregister(&pad_battery->dock_battery);
		return -EINVAL;
	}

	pad_battery_by_EC_workqueue = create_singlethread_workqueue("pad_battery_by_EC_workqueue");
	INIT_DELAYED_WORK(&pad_battery->battery_status_polling_work, battery_status_polling_work_func);
	INIT_DELAYED_WORK(&pad_battery->low_low_battery_work, pad_battery_by_EC_low_low_battery_work_func);
	INIT_DELAYED_WORK(&pad_battery->check_pad_gaugeIC_firmware_is_updated,
		check_if_pad_gaugeIC_firmware_is_updated_work_func);
	INIT_DELAYED_WORK(&pad_battery->check_dock_gaugeIC_firmware_is_updated,
		check_if_dock_gaugeIC_firmware_is_updated_work_func);
	INIT_DELAYED_WORK(&pad_battery->battery_status_reupdate_at_booting,
		battery_status_reupdate_at_booting_work_func);

	wake_lock_init(&pad_battery->low_low_battery.low_low_battery_wake_lock,
		WAKE_LOCK_SUSPEND, "low_low_battery_detection");
	wake_lock_init(&pad_battery->cable_type.cable_type_change_event_wake_lock,
		WAKE_LOCK_SUSPEND, "battery_cable_type_changed_event");
	wake_lock_init(&pad_battery->gaugeIC_updating,
		WAKE_LOCK_SUSPEND, "gaugeIC updating");

	if(pad_battery_by_EC_setup_low_low_battery_detect_irq() < 0)
		printk("pad_battery_by_EC: setup low low battery fail\n");

	if(project_is_haydn){
		pad_battery->haydn_AC_in = false;
		pad_battery->haydn_AC_ok.GPIO_pin= HAYDN_GPIO_AC_OK;
		pad_battery->haydn_AC_ok.irq= gpio_to_irq(pad_battery->haydn_AC_ok.GPIO_pin);

		INIT_DELAYED_WORK(&pad_battery->haydn_AC_ok_work,
			pad_battery_by_EC_haydn_AC_ok_work_func);

		wake_lock_init(&pad_battery->haydn_AC_ok.AC_ok_wake_lock, WAKE_LOCK_SUSPEND, "charger_AC_ok_wakelock");

		if(pad_battery_by_EC_haydn_setup_AC_ok_irq() < 0)
			printk("pad_battery_by_EC: setup haydn AC ok fail\n");

		pad_battery->haydn_AC_in = !gpio_get_value(pad_battery->haydn_AC_ok.GPIO_pin);

		printk("pad_battery_by_EC: haydn probe %s\n", pad_battery->haydn_AC_in? "with AC ":"without AC");
	}

	if(sysfs_create_group(&client->dev.kobj, &pad_battery_by_EC_group))
		dev_err(&client->dev, "pad_battery_by_EC_probe: unable to create the sysfs\n");

	cable_status_register_client(&usb_cable_changed_callback_notifier);


	pad_battery_by_EC_driver_ready = true;

	for_udc_call_me_back_if_cable_type_change(NULL, query_cable_status(), &client->dev);

        if(machine_is_mozart()){
	    if(!image_is_user){
	        printk("it's not user image\n");
		queue_delayed_work(pad_battery_by_EC_workqueue,
			&pad_battery->check_pad_gaugeIC_firmware_is_updated, 10*HZ);
	    } else
		printk("it's user image, so assume gaugeIC is update:%d\n", pad_gaugeIC_firmware_is_updated);
        }

	queue_delayed_work(pad_battery_by_EC_workqueue,
		&pad_battery->battery_status_reupdate_at_booting, 14*HZ);
	queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 20*HZ);

	return 0;
}

static int pad_battery_by_EC_remove(struct i2c_client *client)
{
	power_supply_unregister(&pad_battery->battery);
	power_supply_unregister(&pad_battery->ac);
	power_supply_unregister(&pad_battery->usb);
	power_supply_unregister(&pad_battery->dock_battery);
	power_supply_unregister(&pad_battery->dock_ac);
	wake_lock_destroy(&pad_battery->low_low_battery.low_low_battery_wake_lock);
	wake_lock_destroy(&pad_battery->cable_type.cable_type_change_event_wake_lock);
	wake_lock_destroy(&pad_battery->gaugeIC_updating);
	del_timer_sync(&pad_battery->dock_battery_status_detect_timer);

	if(project_is_haydn)
		wake_lock_destroy(&pad_battery->haydn_AC_ok.AC_ok_wake_lock);

	return 0;
}

#if defined (CONFIG_PM)
static int pad_battery_by_EC_suspend(struct i2c_client *client, pm_message_t state)
{
	hw_rev revision = asustek_get_hw_rev();

	// Disable USB-HUB current by GPIO_PBB5.
	if (machine_is_haydn()) {
		switch (revision) {
		case HW_REV_A: break;
		case HW_REV_C: break;
		case HW_REV_E: break;
		default:
			printk(KERN_INFO "P1802 USB-HUB Power off\n");
			gpio_set_value(TEGRA_GPIO_PBB5, 0);
			break;
		}
	}

	printk("pad battery by EC suspend+\n");
	cancel_delayed_work_sync(&pad_battery->battery_status_polling_work);
	flush_workqueue(pad_battery_by_EC_workqueue);
	enable_irq_wake(pad_battery->low_low_battery.irq);
	del_timer_sync(&pad_battery->dock_battery_status_detect_timer);

	if(project_is_haydn)
		enable_irq_wake(pad_battery->haydn_AC_ok.irq);

	printk("pad battery by EC suspend-\n");
	return 0;
}

static int pad_battery_by_EC_resume(struct i2c_client *client)
{
	printk("pad battery by EC resume+\n");
	cancel_delayed_work(&pad_battery->battery_status_polling_work);
	queue_delayed_work(pad_battery_by_EC_workqueue, &pad_battery->battery_status_polling_work, 0.1*HZ);
	disable_irq_wake(pad_battery->low_low_battery.irq);

	if(project_is_haydn)
		disable_irq_wake(pad_battery->haydn_AC_ok.irq);

	printk("pad battery by EC resume-\n");
	return 0;
}
#endif


static const struct dev_pm_ops pad_battery_by_EC_pm_ops = {
	.suspend = pad_battery_by_EC_suspend,
	.resume = pad_battery_by_EC_resume,
};

static const struct i2c_device_id pad_battery_by_EC_id[] = {
	{ "pad_battery_by_EC", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, pad_battery_by_EC_id);

static struct i2c_driver pad_battery_by_EC_driver = {
	.driver = {
		.name = "pad_battery_by_EC",
		#if defined (CONFIG_PM)
		.pm = &pad_battery_by_EC_pm_ops,
		#endif
	},
	.probe        = pad_battery_by_EC_probe,
	.remove      = pad_battery_by_EC_remove,
	.id_table     = pad_battery_by_EC_id,
};

static int __init battery_by_EC_init(void)
{
	hw_rev hw_version = HW_REV_INVALID;

	if(machine_is_mozart()){
		hw_version = asustek_get_hw_rev();
		if(hw_version != HW_REV_A){
			printk("pad_battery_by_EC: init, it's mozart.\n");
			return i2c_add_driver(&pad_battery_by_EC_driver);
		}else
			return 0;
	}else if(machine_is_haydn()){
		printk("pad_battery_by_EC: init, it's haydn\n");
			return i2c_add_driver(&pad_battery_by_EC_driver);
	}
}
module_init(battery_by_EC_init);

static void __exit battery_by_EC_exit(void)
{
	hw_rev hw_version = HW_REV_INVALID;

	if(machine_is_mozart()){
		hw_version = asustek_get_hw_rev();
		if(hw_version != HW_REV_A){
			printk("pad_battery_by_EC: exit, it's mozart.\n");
			i2c_del_driver(&pad_battery_by_EC_driver);
		}
	}else if(machine_is_haydn()){
		printk("pad_battery_by_EC: init, it's haydn\n");
			return i2c_add_driver(&pad_battery_by_EC_driver);
	}
}
module_exit(battery_by_EC_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS pad battery gauge by ec driver");
MODULE_AUTHOR("Piter Hsu <piter_hsu@asus.com>");
