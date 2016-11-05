/*
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/time.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#if 1
#include <linux/wakelock.h>
#endif
#include <../gpio-names.h>
#include "asuspec.h"
#include "elan_i2c_asus.h"
#include <asm/mach-types.h>
#include <linux/statfs.h>

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */

static void asuspec_tp_enable(u8 cmd);
static int asuspec_dockram_write_data(int cmd, int length);
static int asusdec_dockram_write_data(int cmd, int length);
static int asuspec_dockram_read_data(int cmd);
static int asusdec_dockram_read_data(int cmd);
static int asuspec_dockram_read_battery(int cmd);
static int asuspec_i2c_write_data(struct i2c_client *client, u16 data);
static int asuspec_i2c_read_data(struct i2c_client *client);
static int asuspec_chip_init(struct i2c_client *client);
static void asuspec_send_ec_req(void);
static void asuspec_enter_s3_timer(unsigned long data);
static void asuspec_enter_s3_work_function(struct work_struct *dat);
static void asuspec_fw_update_work_function(struct work_struct *dat);
static void asuspec_lid_report_function(struct work_struct *dat);
static void asusdec_dock_init_work_function(struct work_struct *dat);
static void asuspec_work_function(struct work_struct *dat);
static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit asuspec_remove(struct i2c_client *client);
static ssize_t asuspec_status_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_gauge_status_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_dock_gauge_status_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_info_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_version_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_battery_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_dock_battery_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_control_flag_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_dock_control_flag_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_show_lid_status(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_send_ec_req_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_charging_led_store(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asuspec_led_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_enter_factory_mode_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_enter_normal_mode_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusdec_mute_dock_boost_power_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_cmd_data_store(struct device *class,
               struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asuspec_return_data_show(struct device *class,
               struct device_attribute *attr,char *buf, size_t count);
static ssize_t asuspec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asuspec_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf);
static int asuspec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asuspec_resume(struct i2c_client *client);
static int asuspec_open(struct inode *inode, struct file *flip);
static int asuspec_release(struct inode *inode, struct file *flip);
static long asuspec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static void asuspec_switch_apower_state(int state);
static void asuspec_enter_factory_mode(void);
static void asusdec_enter_factory_mode(void);
static void asuspec_enter_normal_mode(void);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static void BuffPush(char data);
static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf);
static int asusdec_input_device_create(struct i2c_client *client);
static int asusdec_lid_input_device_create(struct i2c_client *client);
static int asusdec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
static bool asuspec_gauge_rom_mode_check(bool pad);
static void asuspec_switch_apower_state(int state);
static void asuspec_switch_hdmi(void);
static void asuspec_win_shutdown(void);
static void asuspec_storage_info_update(void);
static ssize_t asuspec_pad_base_sync_show(struct device *class,
		struct device_attribute *attr,char *buf);
static bool asuspec_check_dock_in_control_flag(void);
/*
* extern variable
*/
extern unsigned int factory_mode;
#if EMC_NOTIFY
extern u8 mouse_dock_enable_flag;
#endif
#if BATTERY_DRIVER
extern int for_asuspec_call_me_back_if_dock_in_status_change(bool);
#endif

/*
 * global variable
 */
char* switch_value[]={"0", "10", "11", "12"}; //0: no dock, 1:mobile dock, 2:audio dock, 3: audio st

enum firmware_update_type {
	UPDATE_PAD_BYTE_MODE = 0,
	UPDATE_PAD_BLOCK_MODE,
	UPDATE_DOCK_BYTE_MODE,
	UPDATE_DOCK_BLOCK_MODE,
};

enum etd_i2c_command {
	ETP_HID_DESCR_CMD,
	ETP_HID_WAKE_UP_CMD,
	ETP_HID_RESET_CMD,
	ETP_HID_REPORT_DESCR_CMD,
	ETP_HID_READ_DATA_CMD,
	ASUS_KB_HID_READ_DATA_CMD,
	ETP_HID_SLEEP_CMD,
	ASUS_KB_WAKE_UP_CMD,
	ASUS_KB_SLEEP_CMD,
	ASUS_KB_RESET_CMD
};

static unsigned int asuspec_apwake_gpio = TEGRA_GPIO_PQ5;
static unsigned int asuspec_ps2_int_gpio = TEGRA_GPIO_PW2;
static unsigned int asuspec_kb_int_gpio = TEGRA_GPIO_PJ0;
static unsigned int asuspec_ecreq_gpio = TEGRA_GPIO_PQ2;
static unsigned int asuspec_dock_in_gpio = TEGRA_GPIO_PO0;
static unsigned int asuspec_hall_sensor_gpio = TEGRA_GPIO_PO5;
static unsigned int asuspec_bat_id_gpio = TEGRA_GPIO_PQ1;
static unsigned int lcd_bl_gpio = TEGRA_GPIO_PH2;

static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data
static int fu_block_mode = 0;
static int fu_type = 0;
static int gauge_fu_status = 0;
static int first_tp_ioctl = 0;

struct i2c_client dockram_client;
struct i2c_client tp_client;
struct i2c_client kb_client;
static struct class *asuspec_class;
static struct device *asuspec_device ;
static struct asuspec_chip *ec_chip;
struct timeval old_pad_battery_time;
struct timeval old_dock_battery_time;
struct timeval old_dock_retry_time;
struct cdev *asuspec_cdev ;
static dev_t asuspec_dev ;
static int asuspec_major = 0 ;
static int asuspec_minor = 0 ;
static bool led_success = 0 ;
static bool last_dock_in_stat = 0;
static bool last_lid_gpio = 0;
static int return_data_show_type = 0;
static int tp_init_retry = 3 ;
static int dock_init_retry = 3 ;
static int finish_first_dock_init = 0 ;
static int touchpad_enable_flag = ASUSDEC_TP_ON;
static int hid_device_sleep = 0 ;
static int kb_power_on = 0 ;
static int finish_touchpad_init = 0;
static int tp_in_ioctl = 0;
static struct workqueue_struct *asuspec_wq;
static struct workqueue_struct *asuspec_tp_wq;
struct delayed_work asuspec_stress_work;

static const struct i2c_device_id asuspec_id[] = {
	{"asuspec", 0},
	{}
};


MODULE_DEVICE_TABLE(i2c, asuspec_id);

struct file_operations asuspec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asuspec_ioctl,
	.open = asuspec_open,
	.write = ec_write,
	.read = ec_read,
	.release = asuspec_release,
};

static struct dev_pm_ops asuspec_dev_pm_ops ={
	.suspend = asuspec_suspend,
	.resume = asuspec_resume,

};

static struct i2c_driver asuspec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asuspec",
		.owner = THIS_MODULE,
		.pm = &asuspec_dev_pm_ops,
	},
	.probe	 = asuspec_probe,
	.remove	 = __devexit_p(asuspec_remove),
	.id_table = asuspec_id,
};

static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asuspec_status_show,NULL);
static DEVICE_ATTR(ec_gauge_status, S_IWUSR | S_IRUGO, asuspec_gauge_status_show,NULL);
static DEVICE_ATTR(ec_dock_gauge_status, S_IWUSR | S_IRUGO, asuspec_dock_gauge_status_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asuspec_info_show,NULL);
static DEVICE_ATTR(ec_version, S_IWUSR | S_IRUGO, asuspec_version_show,NULL);
static DEVICE_ATTR(ec_battery, S_IWUSR | S_IRUGO, asuspec_battery_show,NULL);
static DEVICE_ATTR(ec_dock_battery, S_IWUSR | S_IRUGO, asuspec_dock_battery_show,NULL);
static DEVICE_ATTR(ec_control_flag, S_IWUSR | S_IRUGO, asuspec_control_flag_show,NULL);
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asuspec_dock_control_flag_show,NULL);
static DEVICE_ATTR(ec_request, S_IWUSR | S_IRUGO, asuspec_send_ec_req_show,NULL);
static DEVICE_ATTR(ec_led, S_IWUSR | S_IRUGO, asuspec_led_show,NULL);
static DEVICE_ATTR(ec_charging_led, S_IWUSR | S_IRUGO,asuspec_charging_led_store , asuspec_charging_led_store);
static DEVICE_ATTR(ec_factory_mode, S_IWUSR | S_IRUGO, asuspec_enter_factory_mode_show,NULL);
static DEVICE_ATTR(ec_normal_mode, S_IWUSR | S_IRUGO, asuspec_enter_normal_mode_show,NULL);
static DEVICE_ATTR(ec_mute_dock_boost_power, S_IWUSR | S_IRUGO, asusdec_mute_dock_boost_power_show,NULL);
static DEVICE_ATTR(ec_cmd_data_send, S_IWUSR | S_IRUGO, NULL, asuspec_cmd_data_store);
static DEVICE_ATTR(ec_data_read, S_IWUSR | S_IRUGO,  asuspec_return_data_show, asuspec_return_data_show);
static DEVICE_ATTR(ec_lid, S_IWUSR | S_IRUGO, asuspec_show_lid_status,NULL);
static DEVICE_ATTR(ec_pad_base_sync_state, S_IWUSR | S_IRUGO, asuspec_pad_base_sync_show,NULL);


static struct attribute *asuspec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_gauge_status.attr,
	&dev_attr_ec_dock_gauge_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_ec_battery.attr,
	&dev_attr_ec_dock_battery.attr,
	&dev_attr_ec_control_flag.attr,
	&dev_attr_ec_dock_control_flag.attr,
	&dev_attr_ec_request.attr,
	&dev_attr_ec_led.attr,
	&dev_attr_ec_charging_led.attr,
	&dev_attr_ec_factory_mode.attr,
	&dev_attr_ec_normal_mode.attr,
	&dev_attr_ec_mute_dock_boost_power,
	&dev_attr_ec_cmd_data_send.attr,
	&dev_attr_ec_data_read.attr,
	&dev_attr_ec_lid.attr,
	&dev_attr_ec_pad_base_sync_state.attr,
NULL
};

static const struct attribute_group asuspec_smbus_group = {
	.attrs = asuspec_smbus_attributes,
};

#if ASUSPEC_DEBUG
int dbg_counter = 0;
#endif

static int asusdec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH,
		ASUSDEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSDEC_KEY_AUTOBRIGHT,
		KEY_CAMERA, -9, -10, -11,
		-12, -13, -14, -15,
		KEY_WWW, ASUSDEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE,
		KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP};
/*
 * functions definition
 */
int asuspec_audio_recording(int record_enable){
	if (record_enable)
		asuspec_send_ec_req();
	ec_chip->audio_recording = record_enable;
	ASUSPEC_NOTICE("audio_recording = %d\n", ec_chip->audio_recording);
	return 0;
}
EXPORT_SYMBOL(asuspec_audio_recording);

int asuspec_is_usb_charger(int charger_enable){
	int ret = 0;

	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}

	ec_chip->i2c_dm_data[0] = 8;
	if (charger_enable){
		ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[5] | 0x01;
	} else {
		ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[5] & 0xFE;
	}
	ret = asuspec_dockram_write_data(0x0A,9);
	mod_timer(&ec_chip->asuspec_timer,jiffies+(HZ * 1));
	return ret;
}
EXPORT_SYMBOL(asuspec_is_usb_charger);

void asusdec_touchpad_reinit(void){

	if(ec_chip->dock_status == 0){
		ASUSPEC_NOTICE("dock_status = 0, return\n");
        }else {
		ASUSPEC_NOTICE("touchpad reinit\n");
		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, 0);
	}
}
EXPORT_SYMBOL(asusdec_touchpad_reinit);

int is_pad_charging_and_with_dock(void){
	int ret = 0;

	memset(&ec_chip->i2c_dm_data, 0, 32);
	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}
	ret = (ec_chip->i2c_dm_data[1] & 0x80 && ec_chip->i2c_dm_data[1] & 0x04);
	return ret;
}

int asuspec_battery_monitor(char *cmd,bool pad){
	int ret_val = 0;
	struct timeval pad_battery_time;
	struct timeval dock_battery_time;

	do_gettimeofday(&pad_battery_time);
	do_gettimeofday(&dock_battery_time);

	if (!strcmp(cmd, "is_pad_charging_and_with_dock") && pad == 1){
		ret_val = is_pad_charging_and_with_dock();
		return ret_val;
	}

	if (ec_chip->ec_in_s3){
                asuspec_send_ec_req();
                msleep(200);
        }

	if(pad){//pad battery
		if(((pad_battery_time.tv_sec - old_pad_battery_time.tv_sec) > 15) || !strcmp(cmd, "status")){
			if(ec_chip->op_mode != 1){
				ret_val = asuspec_dockram_read_battery(0x14);
				old_pad_battery_time.tv_sec = pad_battery_time.tv_sec;
			}
		}
	}
	else {//dock battery
		if(machine_is_mozart()){
			if(((dock_battery_time.tv_sec - old_dock_battery_time.tv_sec) > 15) || !strcmp(cmd, "status")){
				if(ec_chip->op_mode != 1){
					ret_val = asuspec_dockram_read_battery(0x24);
					old_dock_battery_time.tv_sec = dock_battery_time.tv_sec;
				}
			}
		}
	}

	if (ret_val == -1){
		ASUSPEC_ERR("Fail to access battery info.\n");
		return -1;
	}
	else {
#if FACTORY_MODE
		if((factory_mode != 2) && (ec_chip->audio_recording == 0)){
			mod_timer(&ec_chip->asuspec_timer,jiffies+(HZ * 1));
		}
#endif
		if(pad){
			if (!strcmp(cmd, "status"))
				ret_val = (ec_chip->i2c_dm_battery[2] << 8 ) | ec_chip->i2c_dm_battery[1];
			else if (!strcmp(cmd, "temperature"))
				ret_val = (ec_chip->i2c_dm_battery[8] << 8 ) | ec_chip->i2c_dm_battery[7];
			else if (!strcmp(cmd, "voltage"))
				ret_val = (ec_chip->i2c_dm_battery[10] << 8 ) | ec_chip->i2c_dm_battery[9];
			else if (!strcmp(cmd, "current"))
				ret_val = (ec_chip->i2c_dm_battery[12] << 8 ) | ec_chip->i2c_dm_battery[11];
			else if (!strcmp(cmd, "capacity"))
				ret_val = (ec_chip->i2c_dm_battery[14] << 8 ) | ec_chip->i2c_dm_battery[13];
			else if (!strcmp(cmd, "remaining_capacity"))
				ret_val = (ec_chip->i2c_dm_battery[16] << 8 ) | ec_chip->i2c_dm_battery[15];
			else if (!strcmp(cmd, "avg_time_to_empty"))
				ret_val = (ec_chip->i2c_dm_battery[18] << 8 ) | ec_chip->i2c_dm_battery[17];
			else if (!strcmp(cmd, "avg_time_to_full"))
				ret_val = (ec_chip->i2c_dm_battery[20] << 8 ) | ec_chip->i2c_dm_battery[19];
			else if (!strcmp(cmd, "full_capacity"))
				ret_val = (ec_chip->i2c_dm_battery[22] << 8 ) | ec_chip->i2c_dm_battery[21];
			else if (!strcmp(cmd, "design_capacity"))
				ret_val = (ec_chip->i2c_dm_battery[24] << 8 ) | ec_chip->i2c_dm_battery[23];
			else if (!strcmp(cmd, "design_voltage"))
				ret_val = (ec_chip->i2c_dm_battery[26] << 8 ) | ec_chip->i2c_dm_battery[25];
			else if (!strcmp(cmd, "cycle_count"))
				ret_val = (ec_chip->i2c_dm_battery[28] << 8 ) | ec_chip->i2c_dm_battery[27];
			else {
				ASUSPEC_ERR("Unknown command\n");
				ret_val = -2;
			}
		}else{
			if (!strcmp(cmd, "status"))
				ret_val = (ec_chip->i2c_dm_dock_battery[2] << 8 ) | ec_chip->i2c_dm_dock_battery[1];
			else if (!strcmp(cmd, "temperature"))
				ret_val = (ec_chip->i2c_dm_dock_battery[8] << 8 ) | ec_chip->i2c_dm_dock_battery[7];
			else if (!strcmp(cmd, "voltage"))
				ret_val = (ec_chip->i2c_dm_dock_battery[10] << 8 ) | ec_chip->i2c_dm_dock_battery[9];
			else if (!strcmp(cmd, "current"))
				ret_val = (ec_chip->i2c_dm_dock_battery[12] << 8 ) | ec_chip->i2c_dm_dock_battery[11];
			else if (!strcmp(cmd, "capacity"))
				ret_val = (ec_chip->i2c_dm_dock_battery[14] << 8 ) | ec_chip->i2c_dm_dock_battery[13];
			else if (!strcmp(cmd, "remaining_capacity"))
				ret_val = (ec_chip->i2c_dm_dock_battery[16] << 8 ) | ec_chip->i2c_dm_dock_battery[15];
			else if (!strcmp(cmd, "avg_time_to_empty"))
				ret_val = (ec_chip->i2c_dm_dock_battery[18] << 8 ) | ec_chip->i2c_dm_dock_battery[17];
			else if (!strcmp(cmd, "avg_time_to_full"))
				ret_val = (ec_chip->i2c_dm_dock_battery[20] << 8 ) | ec_chip->i2c_dm_dock_battery[19];
			else if (!strcmp(cmd, "full_capacity"))
				ret_val = (ec_chip->i2c_dm_dock_battery[22] << 8 ) | ec_chip->i2c_dm_dock_battery[21];
			else if (!strcmp(cmd, "design_capacity"))
				ret_val = (ec_chip->i2c_dm_dock_battery[24] << 8 ) | ec_chip->i2c_dm_dock_battery[23];
			else if (!strcmp(cmd, "design_voltage"))
				ret_val = (ec_chip->i2c_dm_dock_battery[26] << 8 ) | ec_chip->i2c_dm_dock_battery[25];
			else if (!strcmp(cmd, "cycle_count"))
				ret_val = (ec_chip->i2c_dm_dock_battery[28] << 8 ) | ec_chip->i2c_dm_dock_battery[27];
			else {
				ASUSPEC_ERR("Unknown command\n");
				ret_val = -2;
			}
		}
		ASUSPEC_INFO("cmd %s, return %d\n", cmd, ret_val);
		return ret_val;
	}
}
EXPORT_SYMBOL(asuspec_battery_monitor);



u16 asuspec_get_battery_info(char *cmd, bool pad){
	u8 reg;
	u16 ret_val = 0x0000;
	int ret;

	gauge_fu_status = 0;
	if (!strcmp(cmd, "CONTROL_STATUS"))
		reg = 0x00;
	else if (!strcmp(cmd, "DEVICE_TYPE"))
		reg = 0x01;
	else if (!strcmp(cmd, "FW_VERSION"))
		reg = 0x02;
	else if (!strcmp(cmd, "BATTERY_ID_FROM_GAUGEIC"))
		reg = 0x08;
	else if (!strcmp(cmd, "DF_VERSION"))
		reg = 0x1f;
	else {
		ASUSPEC_ERR("Unknown command\n");
		return -2;
	}

	memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
	if(pad){
		ec_chip->i2c_fu_data[0] = 0x06;
		ec_chip->i2c_fu_data[1] = 0x31;
		ec_chip->i2c_fu_data[2] = 0x04;
		ec_chip->i2c_fu_data[3] = 0x55;
		ec_chip->i2c_fu_data[4] = 0x00;
		ec_chip->i2c_fu_data[5] = reg;
		ec_chip->i2c_fu_data[6] = 0x00;
		//use i2c router write cmd to access gauge register
		ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 7, ec_chip->i2c_fu_data);//FIXME:need err handler
		if (ret < 0){
			ASUSPEC_NOTICE("i2c write error\n");
		}
		msleep(500);//FIXME:will tune the time or use retry
		if(gauge_fu_status != 68){
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 7, ec_chip->i2c_fu_data);//FIXME:need err handler
			msleep(500);//FIXME:will tune the time or use retry
		}
	}else{
		ec_chip->i2c_fu_data[0] = 0x0b;
		ec_chip->i2c_fu_data[1] = 0x21;
		ec_chip->i2c_fu_data[2] = 0x09;
		ec_chip->i2c_fu_data[3] = 0x1b;
		ec_chip->i2c_fu_data[4] = 0x21;
		ec_chip->i2c_fu_data[5] = 0x06;
		ec_chip->i2c_fu_data[6] = 0x31;
		ec_chip->i2c_fu_data[7] = 0x04;
		ec_chip->i2c_fu_data[8] = 0x55;
		ec_chip->i2c_fu_data[9] = 0x00;
		ec_chip->i2c_fu_data[10] = reg;
		ec_chip->i2c_fu_data[11] = 0x00;
		//use i2c router write cmd to access gauge register
		ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 12, ec_chip->i2c_fu_data);//FIXME:need err handler
		if (ret < 0){
			ASUSPEC_NOTICE("i2c write error\n");
		}
		msleep(500);//FIXME:will tune the time or use retry
		if(gauge_fu_status != 0xa8){
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 12, ec_chip->i2c_fu_data);//FIXME:need err handler
			msleep(500);//FIXME:will tune the time or use retry
		}
	}
	memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
	if(pad){
		ec_chip->i2c_fu_data[0] = 0x05;
		ec_chip->i2c_fu_data[1] = 0x30;
		ec_chip->i2c_fu_data[2] = 0x03;
		ec_chip->i2c_fu_data[3] = 0x55;
		ec_chip->i2c_fu_data[4] = 0x00;
		ec_chip->i2c_fu_data[5] = 0x02;
		//use i2c router read cmd to read gauge register
		ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);//FIXME:need err handler
		if (ret < 0){
			ASUSPEC_NOTICE("i2c write error2\n");
		}
		msleep(100);//FIXME:will tune the time or use retry
		if(gauge_fu_status != 69){
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);//FIXME:need err handler
			msleep(500);//FIXME:will tune the time or use retry
		}
	}else{
		ec_chip->i2c_fu_data[0] = 0x0a;
		ec_chip->i2c_fu_data[1] = 0x21;
		ec_chip->i2c_fu_data[2] = 0x08;
		ec_chip->i2c_fu_data[3] = 0x1b;
		ec_chip->i2c_fu_data[4] = 0x21;
		ec_chip->i2c_fu_data[5] = 0x05;
		ec_chip->i2c_fu_data[6] = 0x30;
		ec_chip->i2c_fu_data[7] = 0x03;
		ec_chip->i2c_fu_data[8] = 0x55;
		ec_chip->i2c_fu_data[9] = 0x00;
		ec_chip->i2c_fu_data[10] = 0x02;
		//use i2c router read cmd to read gauge register
		ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);//FIXME:need err handler
		if (ret < 0){
			ASUSPEC_NOTICE("i2c write error2\n");
		}
		msleep(100);//FIXME:will tune the time or use retry
		if(gauge_fu_status != 69){
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);//FIXME:need err handler
			msleep(500);//FIXME:will tune the time or use retry
		}
	}

	//get data from ec
	memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
	i2c_smbus_read_i2c_block_data(&dockram_client, 0x21, 3, ec_chip->i2c_fu_data);//FIXME:need err handler
	if (ret < 0){
		ASUSPEC_NOTICE("i2c read error\n");
	}
	msleep(100);//FIXME:will tune the time or use retry
	ret_val = ec_chip->i2c_fu_data[1] | (ec_chip->i2c_fu_data[2] << 8);
	return ret_val;
}

u16 asuspec_get_dock_battery_source(void)
{
	int ret = 0;

	memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
	ec_chip->i2c_fu_data[0] = 0x05;
	ec_chip->i2c_fu_data[1] = 0x20;
	ec_chip->i2c_fu_data[2] = 0x03;
	ec_chip->i2c_fu_data[3] = 0x1b;
	ec_chip->i2c_fu_data[4] = 0x35;
	ec_chip->i2c_fu_data[5] = 0x06;

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);

	msleep(50);//FIXME:will tune the time or use retry
	memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
	ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x21, 7, ec_chip->i2c_fu_data);
	if (ret < 0){
		ASUSPEC_NOTICE("i2c read error :fail to read battery type\n");
		return 0xffff;
	}
	msleep(50);//FIXME:will tune the time or use retry
	if (ec_chip->i2c_fu_data[6] > 0x30)
		return 1;
	else
		return 0;
}

u16 asuspec_gauge_ic_monitor(char *cmd, bool pad){
	if(machine_is_haydn()){
		ASUSPEC_NOTICE("(machine is haydn skip _gauge ic monitor\n");
		return -2;
	}

	u16 ret_val = 0x0000;
	if (!strcmp(cmd, "CONTROL_STATUS"))
		ret_val = asuspec_get_battery_info("CONTROL_STATUS", pad);
	else if (!strcmp(cmd, "FW_VERSION"))
		ret_val = asuspec_get_battery_info("FW_VERSION", pad);
	else if (!strcmp(cmd, "DEVICE_TYPE"))
		ret_val = asuspec_get_battery_info("DEVICE_TYPE", pad);
	else if (!strcmp(cmd, "DF_VERSION"))
		ret_val = asuspec_get_battery_info("DF_VERSION", pad);
	else if (!strcmp(cmd, "BATTERY_ID_FROM_GAUGEIC"))
		ret_val = asuspec_get_battery_info("BATTERY_ID_FROM_GAUGEIC", pad);
	else if (!strcmp(cmd, "BATTERY_TYPE"))
		ret_val = asuspec_get_battery_info("BATTERY_TYPE", pad);
	else if (!strcmp(cmd, "BATTERY_ID_FROM_PIN"))
		if(pad)
			ret_val = gpio_get_value(asuspec_bat_id_gpio);
		else
			ret_val = asuspec_get_dock_battery_source();
	else if (!strcmp(cmd, "BATTERY_ID"))
		ret_val = gpio_get_value(asuspec_bat_id_gpio);
	else if (!strcmp(cmd, "IS_ROM_MODE"))
		ret_val = (u16)asuspec_gauge_rom_mode_check(pad);
	else {
		ASUSPEC_ERR("Unknown command\n");
		ret_val = -2;
	}
	ASUSPEC_INFO("cmd %s, return %d\n", cmd, ret_val);
	return ret_val;
}
EXPORT_SYMBOL(asuspec_gauge_ic_monitor);


int asuspec_start_dock_charging(void){
	int ret_val = 0;
	int i = 0;

	if(machine_is_haydn()){
		ASUSPEC_ERR("error, haydn should not call this function!\n");
		return -1;
	}

	if(machine_is_mozart()){
		ASUSPEC_NOTICE("start dock charging mozart+++\n");
		memset(&ec_chip->i2c_dm_data, 0, 32);
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[3] = 0x20;
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("pad EC start dock charging fail: %d\n",i);
			msleep(100);
		}
		else {
			break;
		}
	}
	if(ret_val < 0){
		ASUSPEC_ERR("pad EC start dock charging fail: %d\n",i);
		return -1;
	}

	memset(&ec_chip->i2c_dock_dm_data, 0, 32);
	ec_chip->i2c_dock_dm_data[0] = 8;
	ec_chip->i2c_dock_dm_data[3] = 0x20;

	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("dock EC start dock charging fail: %d\n",i);
			msleep(100);
		}
		else {
			break;
		}
	}
	if(ret_val < 0){
		ASUSPEC_ERR("dock EC start dock charging fail: %d\n",i);
		return -1;
	}
	ASUSPEC_NOTICE("asuspec start dock charging---\n");
	return ret_val;
}
EXPORT_SYMBOL(asuspec_start_dock_charging);

int asuspec_stop_dock_charging(void){
	int ret_val = 0;
	int i = 0;

	if(machine_is_haydn()){
		ASUSPEC_ERR("error, haydn should not call this function!\n");
		return -1;
	}

	if(machine_is_mozart()){
		ASUSPEC_NOTICE("stop dock charging mozart+++\n");
		memset(&ec_chip->i2c_dm_data, 0, 32);
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[7] = 0x20;
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("pad EC stop dock charging fail: %d\n",i);
			msleep(100);
		}
		else {
			break;
		}
	}
	if(ret_val < 0){
		ASUSPEC_ERR("pad EC stop dock charging fail: %d\n",i);
		return -1;
	}

	msleep(500);
	memset(&ec_chip->i2c_dock_dm_data, 0, 32);
	ec_chip->i2c_dock_dm_data[0] = 8;
	ec_chip->i2c_dock_dm_data[7] = 0x20;
	for ( i = 0; i < 3; i++ ){
		ret_val = asusdec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("dock EC stop dock charging fail: %d\n",i);
			msleep(100);
		}
		else {
			break;
		}
	}
	if(ret_val < 0){
		ASUSPEC_ERR("dock EC start dock charging fail: %d\n",i);
		return -1;
	}
	ASUSPEC_NOTICE("asuspec stop dock charging---\n");
	return ret_val;
}
EXPORT_SYMBOL(asuspec_stop_dock_charging);

int asuspec_start_gauge_firmware_update(bool pad){
	int ret_val = 0;
	int i = 0;
	if(!pad)
		asuspec_stop_dock_charging();
	msleep(500);
	if(machine_is_mozart()){
		if(pad){
			memset(&ec_chip->i2c_fu_data, 0, 32);
		        ec_chip->i2c_fu_data[0] = 8;
		        ec_chip->i2c_fu_data[7] = ec_chip->i2c_fu_data[7] | 0x08;//set bit
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x0a, 9, ec_chip->i2c_fu_data);
		}else{
			memset(&ec_chip->i2c_dock_dm_data, 0, 32);
		        ec_chip->i2c_dock_dm_data[0] = 8;
			ec_chip->i2c_dock_dm_data[7] = ec_chip->i2c_dock_dm_data[7] | 0x08;//set bit
			if (ec_chip->dock_in == 0){
				return -1;
			}
			for (i = 0; i < 8 ; i++){
			        ec_chip->i2c_dock_dm_data[i+9] = ec_chip->i2c_dock_dm_data[i+1];//skip length
			}
			ec_chip->i2c_dock_dm_data[0] = 0x11;//length
			ec_chip->i2c_dock_dm_data[1] = 0x0a;//i2c read block
			ec_chip->i2c_dock_dm_data[2] = 0x00;//result :read only
			ec_chip->i2c_dock_dm_data[3] = 0x36;//8bit dock i2c address
			ec_chip->i2c_dock_dm_data[4] = 0x0a;
			ec_chip->i2c_dock_dm_data[5] = 0x08;//write byte number
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x12, ec_chip->i2c_dock_dm_data);
			if (ret_val < 0) {
			        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret_val);
			}
		}
	}
	msleep(50);
	return ret_val;
}
EXPORT_SYMBOL(asuspec_start_gauge_firmware_update);

int asuspec_get_gauge_mode(bool pad){
	int normal_mode_fail, rom_mode_fail, ret;

	if(machine_is_mozart()){
		gauge_fu_status = 0;
		memset(&ec_chip->i2c_fu_data, 0, 32);//clear buffer
		if(pad){
			ec_chip->i2c_fu_data[0] = 0x05;
			ec_chip->i2c_fu_data[1] = 0x30;
			ec_chip->i2c_fu_data[2] = 0x03;
			ec_chip->i2c_fu_data[3] = 0x55;
			ec_chip->i2c_fu_data[4] = 0x00;
			ec_chip->i2c_fu_data[5] = 0x02;
			//use i2c router read cmd to read gauge register
			ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);
			if (ret < 0){
				ASUSPEC_NOTICE("i2c write error\n");
			}
			msleep(200);
			if(gauge_fu_status != 69){
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);
				msleep(300);
				if(gauge_fu_status == -2){
					ASUSPEC_ERR("normal mode write failed!\n");
					normal_mode_fail = 1;
				}else
					normal_mode_fail = 0;
			}else{
				normal_mode_fail = 0;
			}
			ec_chip->i2c_fu_data[3] = 0x0b;
			ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);
			if (ret < 0){
				ASUSPEC_NOTICE("i2c write error2\n");
			}
			msleep(200);
			if(gauge_fu_status != 69){
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 6, ec_chip->i2c_fu_data);
				msleep(300);
				if(gauge_fu_status == -2){
					ASUSPEC_ERR("rom mode write failed!\n");
					rom_mode_fail = 1;
				}else
					rom_mode_fail = 0;
			}else{
				rom_mode_fail = 0;
			}
		}else{
			ec_chip->i2c_fu_data[0] = 0x0a;
			ec_chip->i2c_fu_data[1] = 0x21;
			ec_chip->i2c_fu_data[2] = 0x08;
			ec_chip->i2c_fu_data[3] = 0x1b;
			ec_chip->i2c_fu_data[4] = 0x21;
			ec_chip->i2c_fu_data[5] = 0x05;
			ec_chip->i2c_fu_data[6] = 0x30;
			ec_chip->i2c_fu_data[7] = 0x03;
			ec_chip->i2c_fu_data[8] = 0x55;
			ec_chip->i2c_fu_data[9] = 0x00;
			ec_chip->i2c_fu_data[10] = 0x02;
			//use i2c router read cmd to read gauge register
			ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);
			if (ret < 0){
				ASUSPEC_NOTICE("i2c write error\n");
			}
			msleep(200);
			if(gauge_fu_status != 69){
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);
				msleep(300);
				if(gauge_fu_status == -2){
					ASUSPEC_ERR("normal mode write failed!\n");
					normal_mode_fail = 1;
				}else
					normal_mode_fail = 0;
			}else{
				normal_mode_fail = 0;
			}
			ec_chip->i2c_fu_data[8] = 0x0b;
			ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);
			if (ret < 0){
				ASUSPEC_NOTICE("i2c write error2\n");
			}
			msleep(200);
			if(gauge_fu_status != 69){
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, 11, ec_chip->i2c_fu_data);
				msleep(300);
				if(gauge_fu_status == -2){
					ASUSPEC_ERR("rom mode write failed!\n");
					rom_mode_fail = 1;
				}else
					rom_mode_fail = 0;
			}else{
				rom_mode_fail = 0;
			}
		}
		if(normal_mode_fail && rom_mode_fail){
			ASUSPEC_ERR("normal mode and rom mode write failed!\n");
			return 0;
		}else if (normal_mode_fail == 1 && rom_mode_fail == 0){
			ASUSPEC_NOTICE("gauge ic is in rom mode!\n");
			return 1;
		}else if (normal_mode_fail == 0 && rom_mode_fail == 1){
			ASUSPEC_NOTICE("gauge ic is in normal mode!\n");
			return 2;
		}
	}
}
EXPORT_SYMBOL(asuspec_get_gauge_mode);

int asuspec_stop_gauge_firmware_update(bool pad){
	int ret_val = 0;
	int i = 0;
	if(machine_is_mozart()){
		if(pad){
			memset(&ec_chip->i2c_fu_data, 0, 32);
		        ec_chip->i2c_fu_data[0] = 8;
		        ec_chip->i2c_fu_data[3] = ec_chip->i2c_fu_data[3] | 0x08;//clean bit
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x0a, 9, ec_chip->i2c_fu_data);
			msleep(50);
		}else{
			memset(&ec_chip->i2c_dock_dm_data, 0, 32);
		        ec_chip->i2c_dock_dm_data[0] = 8;
		        ec_chip->i2c_dock_dm_data[3] = ec_chip->i2c_dock_dm_data[3] | 0x08;//set bit
			if (ec_chip->dock_in == 0){
				asuspec_start_dock_charging();
				return -1;
			}
			for (i = 0; i < 8 ; i++){
			        ec_chip->i2c_dock_dm_data[i+9] = ec_chip->i2c_dock_dm_data[i+1];//skip length
			}
			ec_chip->i2c_dock_dm_data[0] = 0x11;//length
			ec_chip->i2c_dock_dm_data[1] = 0x0a;//i2c read block
			ec_chip->i2c_dock_dm_data[2] = 0x00;//result :read only
			ec_chip->i2c_dock_dm_data[3] = 0x36;//8bit dock i2c address
			ec_chip->i2c_dock_dm_data[4] = 0x0a;
			ec_chip->i2c_dock_dm_data[5] = 0x08;//write byte number
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x12, ec_chip->i2c_dock_dm_data);
			if (ret_val < 0) {
			        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret_val);
			}
		}
	}
	msleep(50);
	if(!pad)
		asuspec_start_dock_charging();
	return ret_val;
}
EXPORT_SYMBOL(asuspec_stop_gauge_firmware_update);

bool asuspec_gauge_rom_mode_check(bool pad){
	if(pad)
		asuspec_dockram_read_data(0x0A);
	else
		asusdec_dockram_read_data(0x0A);
	ASUSPEC_NOTICE("ec_chip->i2c_dm_data[7] = %02x\n",ec_chip->i2c_dm_data[7]);

	if(ec_chip->i2c_dm_data[7] & 0x08)
		return true;
	else
		return false;
}

/***************************************************
 This function is for gauge ic firmware update
 cmd : 0x01 for write, 0x00 for read and check, 0x02 for msleep
 buff : data buffer
 length : data buffer length in byte
****************************************************/
int asuspec_gauge_firmware_update(u8 cmd, u8 buff[], u8 length, bool pad){
	int ret_val = 0;
	int i = 0;
	int buf_len = length;
	int sleep_time = 0;
	gauge_fu_status = 0;

	switch(cmd){
	case ASUSPEC_GAUGE_IC_READ:
		if(pad){
			ec_chip->i2c_fu_data[0] = length + 0x02;
			ec_chip->i2c_fu_data[1] = 0x30;
			ec_chip->i2c_fu_data[2] = 3;//i2c add, reg, read lehgth
			ec_chip->i2c_fu_data[3] = buff[0] >> 1;
			ec_chip->i2c_fu_data[4] = buff[1];
			ec_chip->i2c_fu_data[5] = length - 2;
		}else{
			ec_chip->i2c_fu_data[0] = length + 7;
			ec_chip->i2c_fu_data[1] = 0x21;
			ec_chip->i2c_fu_data[2] = length + 5;
			ec_chip->i2c_fu_data[3] = 0x1b;
			ec_chip->i2c_fu_data[4] = 0x21;
			ec_chip->i2c_fu_data[5] = length + 0x02;
			ec_chip->i2c_fu_data[6] = 0x30;
			ec_chip->i2c_fu_data[7] = 3;//i2c add, reg, read lehgth
			ec_chip->i2c_fu_data[8] = buff[0] >> 1;
			ec_chip->i2c_fu_data[9] = buff[1];
			ec_chip->i2c_fu_data[10] = length - 2;
		}
	                ASUSPEC_NOTICE("I2c Router read %d byte\n",length - 2);
			for(i = 2; i < length; i++)//store data in temp buff to check later
			{
					ec_chip->i2c_dm_data[i] = buff[i];
			}
		if(pad)
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, length + 3, ec_chip->i2c_fu_data);
		else
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, length + 8, ec_chip->i2c_fu_data);
			break;
	case ASUSPEC_GAUGE_IC_WRITE:
		if(pad){
			ec_chip->i2c_fu_data[0] = length + 0x02;
			ec_chip->i2c_fu_data[1] = 0x31;
			ec_chip->i2c_fu_data[2] = length;
			for(i = 0; i < length; i++)
			{
				if(i == 0)
					ec_chip->i2c_fu_data[3+i] = buff[i] >> 1;
				else
					ec_chip->i2c_fu_data[3+i] = buff[i];
			}
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, length + 3, ec_chip->i2c_fu_data);
			break;
		}else{
			ec_chip->i2c_fu_data[0] = length + 7;
			ec_chip->i2c_fu_data[1] = 0x21;
			ec_chip->i2c_fu_data[2] = length + 5;
			ec_chip->i2c_fu_data[3] = 0x1b;
			ec_chip->i2c_fu_data[4] = 0x21;
			ec_chip->i2c_fu_data[5] = length + 0x02;
			ec_chip->i2c_fu_data[6] = 0x31;
			ec_chip->i2c_fu_data[7] = length;
			for(i = 0; i < length; i++)
			{
				if(i == 0)
					ec_chip->i2c_fu_data[8+i] = buff[i] >> 1;
				else
					ec_chip->i2c_fu_data[8+i] = buff[i];
			}
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x21, length + 8, ec_chip->i2c_fu_data);
			break;
		}
	case ASUSPEC_GAUGE_IC_SLEEP:
		if(buf_len == 1){
			sleep_time = buff[0];
		}else if(buf_len == 2)	{
			sleep_time = buff[0] * 256 + buff[1];
		}else
			ASUSPEC_NOTICE("I2c Router sleep unknow!\n");
		ASUSPEC_NOTICE("I2c Router sleep %d\n",sleep_time);
		msleep(sleep_time);
		return 100;
		break;
	default :
		ASUSPEC_ERR("I2c Router cmd unknow : %02x\n",cmd);
		break;
	}

	msleep(200);
	if(pad)
		while(gauge_fu_status == 0)//wait for interrupt feedback
		{
			msleep(30);
		}
	else
		while(gauge_fu_status == 0 || gauge_fu_status == 0x68)//wait for interrupt feedback
		{
			msleep(50);
		}
	switch (gauge_fu_status){
	case -I2C_ROUTER_EBUSY:
		ASUSPEC_NOTICE("I2C Router busy!\n");
		return -I2C_ROUTER_EBUSY;
		break;
	case -I2C_ROUTER_ERROR:
		ASUSPEC_NOTICE("I2C Router error!\n");
		return -I2C_ROUTER_ERROR;
		break;
	case I2C_ROUTER_WRITE_SUCCESS://do nothing
		ASUSPEC_NOTICE("I2C Router Write command success!\n");
		return I2C_ROUTER_WRITE_SUCCESS;
		break;
	case 0xa8:
		ASUSPEC_NOTICE("dock I2C Router Write command success!\n");
		return I2C_ROUTER_WRITE_SUCCESS;
		break;
	case I2C_ROUTER_READ_SUCCESS://compare read data and input buffer
		ret_val = i2c_smbus_read_i2c_block_data(&dockram_client, 0x21, length, ec_chip->i2c_fu_data);
		if((length-2) !=  ec_chip->i2c_fu_data[0])
			ASUSPEC_NOTICE("I2C Router read wrong length!  %02x : %02x\n",length-2,ec_chip->i2c_fu_data[0]);
		for(i = 2; i < length; i++)//store data in temp buff to check
		{
			if(ec_chip->i2c_dm_data[i] ==  ec_chip->i2c_fu_data[i-1])
			{}
			else
			{
				ASUSPEC_NOTICE("I2C Router got the wrong data! %02x : %02x\n",ec_chip->i2c_dm_data[i],ec_chip->i2c_fu_data[i-1]);
				return -3;
			}
		}
		ASUSPEC_INFO("I2C Router Read command success!\n");
		return I2C_ROUTER_READ_SUCCESS;
		break;
	default:
		ASUSPEC_NOTICE("I2C Router status unknow!\n");
		return -2;
		break;
	}
	if (ret_val < 0){
		ASUSPEC_ERR("Fail to write gauge ic cmd.\n");
		return -1;
	}
	return gauge_fu_status;
}
EXPORT_SYMBOL(asuspec_gauge_firmware_update);

static void asuspec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x17;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static void asuspec_kb_init(struct i2c_client *client){
	kb_client.adapter = client->adapter;
	kb_client.addr = 0x16;
	kb_client.detected = client->detected;
	kb_client.dev = client->dev;
	kb_client.driver = client->driver;
	kb_client.flags = client->flags;
	strcpy(kb_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static void asuspec_tp_init(struct i2c_client *client){
	tp_client.adapter = client->adapter;
	tp_client.addr = 0x14;
	tp_client.detected = client->detected;
	tp_client.dev = client->dev;
	tp_client.driver = client->driver;
	tp_client.flags = client->flags;
	strcpy(tp_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static int asusdec_dockram_read_data(int cmd)
{
	int ret = 0;
	int i = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}
	memset(&ec_chip->i2c_dm_data, 0, 32);
	ec_chip->i2c_dm_data[0] = 0x05;
	ec_chip->i2c_dm_data[1] = 0x0b;//i2c read block
	ec_chip->i2c_dm_data[2] = 0x00;//result :read only
	ec_chip->i2c_dm_data[3] = 0x36;//8bit dock i2c address
	ec_chip->i2c_dm_data[4] = (u8)cmd;
	ec_chip->i2c_dm_data[5] = (u8)24;//read byte number
	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 6, ec_chip->i2c_dm_data);
	if (ret < 0) {
	        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	}
	msleep(20);
	ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x11, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
	        ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	}
	//FIXME:read status data
	for(i=9; i<32; i++)
	{
		ec_chip->i2c_dm_data[i-9] = ec_chip->i2c_dm_data[i];
	}
	return ret;
}

static int asusdec_dockram_write_data(int cmd, int length)
{
	int ret = 0;
	int i = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}
        for (i = 0; i < length ; i++){
                ec_chip->i2c_dock_dm_data[i+9] = ec_chip->i2c_dock_dm_data[i];
        }
	ec_chip->i2c_dock_dm_data[0] = 0x11;//length
	ec_chip->i2c_dock_dm_data[1] = 0x0e;//i2c read block
	ec_chip->i2c_dock_dm_data[2] = 0x00;//result :read only
	ec_chip->i2c_dock_dm_data[3] = 0x36;//8bit dock i2c address
	ec_chip->i2c_dock_dm_data[4] = cmd;
	ec_chip->i2c_dock_dm_data[5] = 0x09;//write byte number
	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x12, ec_chip->i2c_dock_dm_data);
	if (ret < 0) {
	        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	}
	return ret;
}

static int asusdec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	//FIXME:use 0x11 cmd  ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}

static int asuspec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if(ec_chip->i2c_dm_data[0] == 0xff){
		ASUSPEC_ERR("read 0xFF from ec! read again \n");
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	}
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_write_storageinfo(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_storage);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_storageinfo(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_storage);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_battery(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if(cmd == 0x14){
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_battery);
	}else{
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_dock_battery);
	}
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram battery, status %d\n", ret);
		ret = -1;
	} else {
		if (ec_chip->apwake_disabled){
			mutex_lock(&ec_chip->irq_lock);
			if (ec_chip->apwake_disabled){
				enable_irq(gpio_to_irq(asuspec_apwake_gpio));
				enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
				ec_chip->apwake_disabled = 0;
				ASUSPEC_ERR("Enable pad apwake\n");
			}
			mutex_unlock(&ec_chip->irq_lock);
		}
		ec_chip->i2c_err_count = 0;
	}
	ASUSPEC_I2C_DATA(ec_chip->i2c_dm_battery, dbg_counter);
	return ret;
}

static int asuspec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		mutex_lock(&ec_chip->irq_lock);
		if(!ec_chip->apwake_disabled){
			disable_irq_nosync(gpio_to_irq(asuspec_apwake_gpio));
			disable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
			ec_chip->apwake_disabled = 1;
			ASUSPEC_ERR("Disable pad apwake\n");
		}
		mutex_unlock(&ec_chip->irq_lock);
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}
	ASUSPEC_I2C_DATA(ec_chip->i2c_data, dbg_counter);
	return ret;
}

static int asuspec_i2c_test(struct i2c_client *client){
	return asuspec_i2c_write_data(client, 0x0000);
}

static int asuspec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i = 0;
	u16 gaugeIC_firmware_info;

	ec_chip->op_mode = 0;

	if (asuspec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);

	if (asuspec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (asuspec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	ASUSPEC_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asuspec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("PCBA Version: %s\n", ec_chip->ec_pcba);

#if FACTORY_MODE
	if(factory_mode == 2)
		asuspec_enter_factory_mode();
	else
#endif
		asuspec_enter_normal_mode();

	ec_chip->status = 1;
	switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
fail_to_access_ec:
	return 0;

}

static void asusdec_kp_sci(void){
	int ec_signal = ec_chip->i2c_data[2];

	if(ec_chip->dock_status == 0){
		return;
	}
	if(kb_power_on == 0){
		if(elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0)){
			ASUSPEC_NOTICE("set kb power on!\n");
			kb_power_on = 1;
		}
	}
	if(ec_signal == 4){
		if(tp_in_ioctl == 1){
			ASUSPEC_NOTICE("tp in ioctl, skip touchpad sci event!\n");
			return;
		}else{
			tp_in_ioctl = 1;
		}
	}else {
		tp_in_ioctl = 0;
	}

	if(finish_touchpad_init == 0){
		ASUSPEC_NOTICE("waiting for touchpad init, skip sci event!\n", ec_chip->keypad_data.input_keycode);
		return;
	}

	ec_chip->keypad_data.input_keycode = asusdec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		ASUSPEC_NOTICE("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);

		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);

	}else{
		ASUSPEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
	}
}

static int asusdec_kp_key_mapping(int x)
{

	ASUSPEC_INFO("key = %d\n", x);
	switch (x){
		case ASUSDEC_KEYPAD_ESC:
			return KEY_BACK;

		case ASUSDEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSDEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSDEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSDEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSDEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSDEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSDEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSDEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSDEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSDEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSDEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSDEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSDEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSDEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSDEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSDEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSDEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSDEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSDEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSDEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSDEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSDEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSDEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSDEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSDEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSDEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSDEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSDEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSDEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSDEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSDEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSDEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSDEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSDEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSDEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSDEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSDEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSDEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSDEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSDEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSDEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSDEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSDEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSDEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSDEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSDEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSDEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSDEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSDEC_KEYPAD_HOME:
			return KEY_HOME;

		case ASUSDEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSDEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSDEC_KEYPAD_END:
			return KEY_END;

		case KEY_CAPSLOCK:
			return KEY_WLAN;

		case KEY_F1:
			return KEY_BLUETOOTH;

		case KEY_F2:
			return ASUSDEC_KEY_TOUCHPAD;

		case KEY_F3:
			return KEY_BRIGHTNESSDOWN;

		case KEY_F4:
			return KEY_BRIGHTNESSUP;

		case KEY_F5:
			return ASUSDEC_KEY_AUTOBRIGHT;

		case KEY_F6:
			return KEY_CAMERA;

		case KEY_F7:
			return KEY_WWW;

		case KEY_F8:
			return ASUSDEC_KEY_SETTING;

		case KEY_F9:
			return KEY_PREVIOUSSONG;

		case KEY_F10:
			return KEY_PLAYPAUSE;

		case KEY_NUMLOCK:
			return KEY_NEXTSONG;

		case KEY_KP8:
			return KEY_MUTE;

		case KEY_SCROLLLOCK:
			return KEY_VOLUMEDOWN;

		case KEY_KP9:
			return KEY_VOLUMEUP;

		case KEY_KP5:
			return KEY_SLEEP;

		//--- JP keys
		case ASUSDEC_YEN:
			return KEY_YEN;

		case ASUSDEC_RO:
			return KEY_RO;

		case ASUSDEC_MUHENKAN:
			return KEY_MUHENKAN;

		case ASUSDEC_HENKAN:
			return KEY_HENKAN;

		case ASUSDEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;

		//--- UK keys
		case ASUSDEC_EUROPE_2:
			return KEY_102ND;

		default:
			return -1;
	}
}

static int asusdec_tp_control(int arg){

	int ret_val = 0;
	touchpad_enable_flag = arg;
	if(ec_chip->dock_status != 1){
		ASUSPEC_NOTICE("dock_status = 0, return\n");
		tp_in_ioctl = 0;
		return -1;
	}
	if(finish_touchpad_init == 0){
		ASUSPEC_NOTICE("waiting for touchpad init,skip tp control!\n");
		tp_in_ioctl = 0;
		return -1;
	}
	ASUSPEC_NOTICE("asusdec_tp_control : %d\n",arg);

#if EMC_NOTIFY
	if(arg == 1)
		mouse_dock_enable_flag = mouse_dock_enable_flag | 0x1;
	else
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
#endif

	if(arg == ASUSDEC_TP_ON){
		if (ec_chip->tp_enable == 0 ){
			ec_chip->tp_wait_ack = 1;
			ec_chip->tp_enable = 1;
			disable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
			asuspec_tp_enable(0xf4);
			enable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
			ec_chip->d_index = 0;
		}
		if (ec_chip->touchpad_member == -1){
			ec_chip->init_success = -1;
			queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		}
		ret_val = 0;
	} else if (arg == ASUSDEC_TP_OFF){
		ec_chip->tp_wait_ack = 1;
//		ec_chip->touchpad_member = -1;
		disable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
		asuspec_tp_enable(0xf5);
		enable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
		ec_chip->tp_enable = 0;
		ec_chip->d_index = 0;
		ret_val = 0;
	} else
		ret_val = -ENOTTY;

	tp_in_ioctl = 0;
	return ret_val;

}

#if (!TOUCHPAD_MODE)
static void asusdec_tp_rel(void){
	ASUSPEC_INFO("tp_rel\n");
        ec_chip->touchpad_data.x_sign = ( ec_chip->i2c_tp_data[4] & X_SIGN_MASK) ? 1:0;
        ec_chip->touchpad_data.y_sign = (ec_chip->i2c_tp_data[4] & Y_SIGN_MASK) ? 1:0;
        ec_chip->touchpad_data.left_btn = (ec_chip->i2c_tp_data[4] & LEFT_BTN_MASK) ? 1:0;
        ec_chip->touchpad_data.right_btn = (ec_chip->i2c_tp_data[4] & RIGHT_BTN_MASK) ? 1:0;
        ec_chip->touchpad_data.delta_x =
                (ec_chip->touchpad_data.x_sign) ? (ec_chip->i2c_tp_data[5] - 0xff):ec_chip->i2c_tp_data[5];
        ec_chip->touchpad_data.delta_y =
                (ec_chip->touchpad_data.y_sign) ? (ec_chip->i2c_tp_data[6] - 0xff):ec_chip->i2c_tp_data[6];

        input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x * 10);
        input_report_rel(ec_chip->indev, REL_Y, (-1) * ec_chip->touchpad_data.delta_y * 10);
        input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
        input_report_key(ec_chip->indev, BTN_RIGHT, ec_chip->touchpad_data.right_btn);
        input_sync(ec_chip->indev);

}
#endif

#if TOUCHPAD_MODE
static void asusdec_tp_abs(void){
        unsigned char SA1,A1,B1,SB1,C1,D1;
        static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
        static int Null_data_times = 0;

        if (1){
                SA1= ec_chip->ec_data[0];
                A1 = ec_chip->ec_data[1];
                B1 = ec_chip->ec_data[2];
                SB1= ec_chip->ec_data[3];
                C1 = ec_chip->ec_data[4];
                D1 = ec_chip->ec_data[5];
                ASUSPEC_INFO("SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
                if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) &&
                     (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
                        Null_data_times ++;
                        goto asusdec_tp_abs_end;
                }

		if ( (((ec_chip->ec_data[1] & 0x0f) << 8) | ec_chip->ec_data[2]) > 1160 ||
		     (((ec_chip->ec_data[1] & 0x0f) << 8) | ec_chip->ec_data[2]) < 0    ||
		     (((ec_chip->ec_data[4] & 0x0f) << 8) | ec_chip->ec_data[5]) < 0    ||
		     (((ec_chip->ec_data[4] & 0x0f) << 8) | ec_chip->ec_data[5]) > 406	){
			goto skip_all;
		}

                if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O &&
                   SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
                        elantech_report_absolute_to_related(ec_chip, &Null_data_times);
                }

asusdec_tp_abs_end:
                SA1_O = SA1;
                A1_O = A1;
                B1_O = B1;
                SB1_O = SB1;
                C1_O = C1;
                D1_O = D1;
skip_all:
	        ASUSPEC_INFO("skip all\n");
        }
}
#endif


static void asusdec_touchpad_processing(void){
        int i;
        int length = 0;
        int tp_start = 0;
	ASUSPEC_INFO("TOUCHPAD_PROCESSING\n");
	ASUSPEC_INFO("length %d\n",ec_chip->i2c_tp_data[3]);
#if TOUCHPAD_MODE
        length = ec_chip->i2c_tp_data[3];
        for( i = 0; i < length ; i++){
		ec_chip->ec_data[i] = ec_chip->i2c_tp_data[i+4];
        }
	asusdec_tp_abs();
#else
	asusdec_tp_rel();
#endif
}

static irqreturn_t asuspec_interrupt_handler(int irq, void *dev_id){

	ASUSPEC_INFO("interrupt irq = %d", irq);
	if (irq == gpio_to_irq(asuspec_apwake_gpio)){
		disable_irq_nosync(irq);
		if (ec_chip->op_mode){
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_fw_update_work, 0);
		} else {
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_work, 0);
		}
	}
	else if (irq == gpio_to_irq(asuspec_ps2_int_gpio)){
		ASUSPEC_INFO("ps2 int = %d", irq);
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_report_work, 0);//enable irq here
	}
	else if (irq == gpio_to_irq(asuspec_kb_int_gpio)){
		ASUSPEC_INFO("kb int = %d", irq);
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_kb_report_work, 0);
	}
	else if (irq == gpio_to_irq(asuspec_dock_in_gpio)){
//		ec_chip->dock_in = 0;
//		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		ASUSPEC_NOTICE("dock in irq = %d", irq);
	} else if (irq == gpio_to_irq(asuspec_hall_sensor_gpio)){
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_hall_sensor_work, 0);
		ASUSPEC_INFO("lid irq = %d", irq);
	}
	else
		ASUSPEC_NOTICE("else int = %d", irq);

	return IRQ_HANDLED;
}

static int asuspec_irq_hall_sensor(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_hall_sensor_gpio;
	unsigned irq = gpio_to_irq(asuspec_hall_sensor_gpio);
	const char* label = "asuspec_hall_sensor" ;

	ASUSPEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(irq);

	ASUSPEC_INFO("LID irq = %d, rc = %d\n", irq, rc);

	if (gpio_get_value(gpio)){
		ASUSPEC_NOTICE("LID open\n");
		last_lid_gpio = 1;
	} else{
		ASUSPEC_NOTICE("LID close\n");
		last_lid_gpio = 0;
	}

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int asuspec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_ecreq_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_request" ;

	ASUSPEC_INFO("gpio = %d, irq = %d\n", gpio,irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_output failed for input %d\n", gpio);
		goto err_exit;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	return 0 ;

err_exit:
	return rc;
}

static int asuspec_irq_kb_int(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_kb_int_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_kb_int" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(gpio_to_irq(asuspec_kb_int_gpio));
	disable_irq(gpio_to_irq(asuspec_kb_int_gpio));
	ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}
static int asuspec_irq_ps2_int(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_ps2_int" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	disable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
	ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}

static int asuspec_irq_ec_apwake(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_apwake" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
	ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}

static int asuspec_irq_dock_in(struct i2c_client *client)
{
        int rc = 0 ;
        unsigned gpio = asuspec_dock_in_gpio;
        unsigned irq = gpio_to_irq(asuspec_dock_in_gpio);
        const char* label = "asuspec_dock_in" ;

        ASUSPEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
        ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        rc = gpio_request(gpio, label);
        if (rc) {
                ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
        }

        rc = gpio_direction_input(gpio) ;
        if (rc) {
                ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
                goto err_gpio_direction_input_failed;
        }
        ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        rc = request_irq(irq, asuspec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
        if (rc < 0) {
                ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
                rc = -EIO;
                goto err_gpio_request_irq_fail ;
        }
        ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

        return 0 ;
err_gpio_request_irq_fail :
        gpio_free(gpio);
err_gpio_direction_input_failed:
        return rc;
}


static int asuspec_irq_battery_id(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_bat_id_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_bat_id" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}
static void asuspec_enter_s3_timer(unsigned long data){
	if(machine_is_haydn()){
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_enter_s3_work, 0);
	}
}

static void asuspec_send_ec_req(void){
	ASUSPEC_NOTICE("send EC_Request\n");
	gpio_set_value(asuspec_ecreq_gpio, 0);
	msleep(DELAY_TIME_MS);
	gpio_set_value(asuspec_ecreq_gpio, 1);
}

static void asuspec_smi(void){

struct timeval dock_retry_time;

do_gettimeofday(&dock_retry_time);

	if(machine_is_mozart()){
		if (ec_chip->i2c_data[2] == ASUSPEC_SxI_LCD_Cover_Closed){
			ASUSPEC_NOTICE("LID cover closed with dock!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_LCD_Cover_Opened){
			ASUSPEC_NOTICE("LID cover opened with dock!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_AC_Event){
			ASUSPEC_NOTICE("36 pin-connector insert or remove!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_CycleChange){
			ASUSPEC_NOTICE("Battery_CycleChange!!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_FCCchange){
			ASUSPEC_NOTICE("Battery full charge capacity change!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_Low){
			ASUSPEC_NOTICE("battery lower than 5% without AC!!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_Updated){
			ASUSPEC_NOTICE("battery info or charge status changed\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_ECREQ_Received){
			ASUSPEC_NOTICE("EC got request\n");
			if(ec_chip->status == 0){
				if(finish_first_dock_init == 1)
					asuspec_chip_init(ec_chip->client);
				else
					ASUSPEC_NOTICE("skip chip init because init must auto run at boot\n");
			}
			ec_chip->ec_in_s3 = 0;
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_EC_WAKEUP){
			ASUSPEC_NOTICE("Dock insert! pad EC wake up!\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_BOOTBLOCK_RESET){
			ASUSPEC_NOTICE("reset EC\n");
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_WATCHDOG_RESET){
			ASUSPEC_NOTICE("reset EC\n");
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_REMOVE){
			ASUSPEC_NOTICE("dock remove\n");
			if(finish_first_dock_init == 1)
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
			else
				ASUSPEC_NOTICE("skip dock remove event because dock init must auto run at boot\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_INSERT){
			ASUSPEC_NOTICE("dock insert\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_FAIL){
			ASUSPEC_NOTICE("dock communication fail!\n");
#if BATTERY_DRIVER
			if(1 != last_dock_in_stat && finish_first_dock_init == 1)
				for_asuspec_call_me_back_if_dock_in_status_change(false);
#endif
			if(finish_first_dock_init == 1)
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
			else
				ASUSPEC_NOTICE("skip dock remove event because dock init must auto run at boot\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_HID_INIT_FAIL){
			if(finish_first_dock_init == 1){
				if((dock_retry_time.tv_sec - old_dock_retry_time.tv_sec) > 5){
					old_dock_retry_time.tv_sec = dock_retry_time.tv_sec;
#if BATTERY_DRIVER
					for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif
					cancel_delayed_work_sync(&ec_chip->asusdec_tp_enable_work);
					cancel_delayed_work_sync(&ec_chip->asusdec_dock_init_work);
					queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
					ASUSPEC_NOTICE("reinit dock\n");
				}else{
					ASUSPEC_NOTICE("reinit should over 5 seconds: skip\n");
				}
			}else
				ASUSPEC_NOTICE("skip dock event because dock init must auto run at boot:6C\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_ADAPTER_CHANGE){
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_PAD_BL_CHANGE){
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_HID_Status_Changed){
		} else if (ec_chip->i2c_data[2] == ASUSDEC_SxI_PAD_BL_CHANGE){
#if BATTERY_DRIVER
			if(1 != last_dock_in_stat && finish_first_dock_init == 1)
				for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif
		} else if (ec_chip->i2c_data[2] == ASUSDEC_SxI_HID_Status_Changed){
			ASUSPEC_NOTICE("dock HID status changed\n");
			//init dock
			if(finish_first_dock_init == 1){
				ASUSPEC_NOTICE("queue dock init work\n");
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
			}else
				ASUSPEC_NOTICE("skip dock init event because dock init must auto run at boot\n");
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_HID_WakeUp){
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_I2CRouter_Busy){
			gauge_fu_status = -1;
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_I2CRouter_Error){
			gauge_fu_status = -2;
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_I2CRouter_WR_success){
			gauge_fu_status = 68;
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_I2CRouter_RD_success){
			gauge_fu_status = 69;
		} else if (ec_chip->i2c_data[2] == 0xa8){//FIXME:name
			gauge_fu_status = 0xa8;
		} else if (ec_chip->i2c_data[2] == 0xa7){//FIXME:name
			gauge_fu_status = -2;
		} else if (ec_chip->i2c_data[2] == 0x69){//FIXME:name
			gauge_fu_status = 69;
		} else if (ec_chip->i2c_data[2] == ASUSDEC_SxI_AC_Event){
			ASUSPEC_NOTICE("dock 36 pin-connector insert or remove!\n");
			if(!gpio_get_value(asuspec_hall_sensor_gpio))
				queue_delayed_work(asuspec_wq, &ec_chip->asuspec_hall_sensor_work, 0);
		} else {
		}
	}else{
		if (ec_chip->i2c_data[2] == ASUSPEC_SMI_HANDSHAKING){
			ASUSPEC_NOTICE("ASUSPEC_SMI_HANDSHAKING\n");
			if(ec_chip->status == 0){
				asuspec_chip_init(ec_chip->client);
			}
			ec_chip->ec_in_s3 = 0;
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SMI_RESET){
			ASUSPEC_NOTICE("ASUSPEC_SMI_RESET\n");
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
		} else if (ec_chip->i2c_data[2] == ASUSPEC_SMI_WAKE){
			ASUSPEC_NOTICE("ASUSPEC_SMI_WAKE\n");
		}  else if (ec_chip->i2c_data[2] == APOWER_SMI_S5){
			ASUSPEC_NOTICE("APOWER_POWEROFF\n");
			asuspec_switch_apower_state(APOWER_POWEROFF);
		} else if (ec_chip->i2c_data[2] == APOWER_SMI_NOTIFY_SHUTDOWN){
			ASUSPEC_NOTICE("APOWER_NOTIFY_SHUTDOWN\n");
			asuspec_switch_apower_state(APOWER_NOTIFY_SHUTDOWN);
		} else if (ec_chip->i2c_data[2] == APOWER_SMI_RESUME){
			ASUSPEC_NOTICE("APOWER_SMI_RESUME\n");
			asuspec_switch_apower_state(APOWER_RESUME);
		}
	}
}

static void asuspec_enter_s3_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		mutex_unlock(&ec_chip->state_change_lock);
		return ;
	}

	if(machine_is_haydn()){
		asuspec_storage_info_update();
	}

	ec_chip->ec_in_s3 = 1;
	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Send s3 command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asuspec_init_work_function(struct work_struct *dat)
{
	asuspec_chip_init(ec_chip->client);
}

static void asuspec_stresstest_work_function(struct work_struct *dat)
{
	asuspec_i2c_read_data(ec_chip->client);
	queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
}

static void asusdec_tp_report_work_function(struct work_struct *dat)
{
	int gpio = asuspec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);

       	memset(&ec_chip->i2c_tp_data, 0, 32);
	if(touchpad_enable_flag == 0 || !gpio_get_value(asuspec_hall_sensor_gpio)){//tp_enable FIXME:check enable and disable irq
		elantech_i2c_command(&tp_client, ETP_HID_READ_DATA_CMD, ec_chip->i2c_tp_data, 10);
		enable_irq(irq);
	}else{
		elantech_i2c_command(&tp_client, ETP_HID_READ_DATA_CMD, ec_chip->i2c_tp_data, 10);
		enable_irq(irq);
		asusdec_touchpad_processing();
	}

}

static void asusdec_kb_report_work_function(struct work_struct *dat)
{
	int gpio = asuspec_kb_int_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;
	int i = 0;
	int j = 0;
	int scancode = 0;
	int the_same_key = 0;
       	memset(&ec_chip->i2c_kb_data, 0, 32);
	ret_val = elantech_i2c_command(&kb_client, ASUS_KB_HID_READ_DATA_CMD, ec_chip->i2c_kb_data, 11);
	enable_irq(irq);

	if(ec_chip->dock_status == 0){
		ASUSPEC_NOTICE("without dock, return!");
		return;
	}

	if(ec_chip->i2c_kb_data[0] == 0 && ec_chip->i2c_kb_data[1] == 0){//not press key
		ASUSPEC_NOTICE("hid data length :0\n");
		return;
	}

	ec_chip->keypad_data.extend = 0;

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
	        input_report_key(ec_chip->indev, KEY_LEFTCTRL, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
	        input_report_key(ec_chip->indev, KEY_LEFTCTRL, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
	        input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
	        input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
	        input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
	        input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
	        input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
	        input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
	        input_report_key(ec_chip->indev, KEY_RIGHTALT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
	        input_report_key(ec_chip->indev, KEY_RIGHTALT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
	        input_report_key(ec_chip->indev, KEY_HOMEPAGE, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
	        input_report_key(ec_chip->indev, KEY_HOMEPAGE, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
	        input_report_key(ec_chip->indev, KEY_SEARCH, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
	        input_report_key(ec_chip->indev, KEY_SEARCH, 0);
	}
	for(i = 0;i < 6;i++)//normal keys
	{
	        if(ec_chip->i2c_kb_data[i+5] > 0){//press key
			ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_kb_data[i+5]);
			ec_chip->keypad_data.value = 1;
	                ASUSPEC_NOTICE("keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
	                input_report_key(ec_chip->indev,
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		}else if(ec_chip->i2c_kb_data[i+5] == 0){
			break;
	        }else{
	                ASUSPEC_NOTICE("Unknown scancode = 0x%x\n", scancode);
	        }
	}
	for(i = 0;i < 6;i++)
	{
	        if(ec_chip->i2c_old_kb_data[i+5] > 0){
			for(j = 0;j < 6;j++)//check key break
			{
				if(ec_chip->i2c_kb_data[j+5] == ec_chip->i2c_old_kb_data[i+5]){
					the_same_key = 1;
					break;
				}
				else
					the_same_key = 0;
			}
			if(the_same_key == 0){
				ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_old_kb_data[i+5]);
				ASUSPEC_INFO("release keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
				input_report_key(ec_chip->indev,
					ec_chip->keypad_data.input_keycode, 0);
			}
		}else{
			break;
		}
	}
	for(i = 0;i < 8;i++)
	{
		ec_chip->i2c_old_kb_data[i+3] = ec_chip->i2c_kb_data[i+3];
	}
	input_sync(ec_chip->indev);
}

bool asuspec_check_hid_control_flag(void){
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x23);
	if (ret_val < 0)
		ASUSPEC_NOTICE("fail to read dockram data\n");
	if(ec_chip->i2c_dm_data[3] & 0x02)
		return true;
	else
		return false;
}

static void asusdec_tp_enable_work_function(struct work_struct *dat)
{
	struct elantech_data *etd = ec_chip->private;
        if(!asuspec_check_hid_control_flag()){
		ASUSPEC_ERR("fail to detect HID flag!\n");
	        ec_chip->touchpad_member = -1;
		return;
	}
	//tp hid power on
	memset(&ec_chip->i2c_tp_data, 0, 38);
	elantech_i2c_command(&tp_client, ETP_HID_WAKE_UP_CMD, ec_chip->i2c_tp_data, 0);
	msleep(TP_DELAY_TIME_MS);

#if TOUCHPAD_MODE
	if ((!elantech_detect(ec_chip,&tp_client)) && (!elantech_init(ec_chip,&tp_client))){
		ec_chip->touchpad_member = ELANTOUCHPAD;
		finish_touchpad_init = 1;
		ASUSPEC_NOTICE("tp init success\n");
		tp_init_retry = 3;
	} else {
	        ec_chip->touchpad_member = -1;
		if(tp_init_retry-- > 0){
			ASUSPEC_ERR("fail to detect tp : retry\n");
			queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, 1*HZ);
		}else{
			finish_touchpad_init = 1;
			ASUSPEC_ERR("enable touchpad failed!\n");
		}
		return;
	}
#endif
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x05;
	ec_chip->i2c_tp_data[2] = 0x00;
	ec_chip->i2c_tp_data[3] = 0x22;
	ec_chip->i2c_tp_data[4] = 0xd4;
	ec_chip->i2c_tp_data[5] = 0xf4;
	i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
	ASUSPEC_INFO("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));
#if EMC_NOTIFY
	if(touchpad_enable_flag == 1)
		mouse_dock_enable_flag = mouse_dock_enable_flag | 0x1;
	else
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
#endif
	ASUSPEC_NOTICE("finish tp enable work function: %d \n",touchpad_enable_flag);

}

void asuspec_tp_enable(u8 cmd)
{
	struct elantech_data *etd = ec_chip->private;
	if(first_tp_ioctl == 0){
		first_tp_ioctl = 1;
		ASUSPEC_NOTICE("first tp enable control!\n");
	}
	//tp hid power on
	memset(&ec_chip->i2c_tp_data, 0, 38);

	ASUSPEC_NOTICE("tp power on!\n");
	elantech_i2c_command(&tp_client, ETP_HID_WAKE_UP_CMD, ec_chip->i2c_tp_data, 0);
	msleep(TP_DELAY_TIME_MS);

#if TOUCHPAD_MODE
	if(ec_chip->touchpad_member != ELANTOUCHPAD){
		if(cmd == 0xf4){
	                if ((!elantech_detect(ec_chip,&tp_client)) && (!elantech_init(ec_chip,&tp_client))){
				ec_chip->touchpad_member = ELANTOUCHPAD;
	                } else {
	                        ec_chip->touchpad_member = -1;
				ASUSPEC_ERR("ioctl enable touchpad failed!\n");
				return;
	                }
		}
	}
#endif
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x05;
	ec_chip->i2c_tp_data[2] = 0x00;
	ec_chip->i2c_tp_data[3] = 0x22;
	ec_chip->i2c_tp_data[4] = 0xd4;
	ec_chip->i2c_tp_data[5] = cmd;
	i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
	ASUSPEC_INFO("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));
	msleep(60);
	elantech_i2c_command(&tp_client, ETP_HID_READ_DATA_CMD, ec_chip->i2c_tp_data, 10);
	if(cmd == 0xf4){
#if EMC_NOTIFY
		mouse_dock_enable_flag = mouse_dock_enable_flag | 0x1;
#endif
		ASUSPEC_NOTICE("tp enable\n");
	}else if(cmd == 0xf5){
#if EMC_NOTIFY
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
#endif
		ASUSPEC_NOTICE("tp disable\n");
	}else
		ASUSPEC_ERR("wrong tp cmd\n");
	tp_in_ioctl = 0;
}

static void asusdec_dock_status_report(void){
	ASUSPEC_NOTICE("dock_in = %d\n", ec_chip->dock_in);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_type]);
}

static bool asuspec_check_dock_in_control_flag(void){
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x0A);
	if (ret_val < 0)
		ASUSPEC_NOTICE("fail to read dockram data\n");
	if(ec_chip->i2c_dm_data[3] & ASUSPEC_MOBILE_DOCK_PRESENT){
		ASUSPEC_NOTICE("detect mobile dock!\n");
		return true;
	}
	else{
		ASUSPEC_NOTICE("fail to detect mobile dock!\n");
		return false;
	}
}

static int asusdec_is_init_running(void){
	int ret_val;

	mutex_lock(&ec_chip->dock_init_lock);
	ret_val = ec_chip->dock_init;
	ec_chip->dock_init = 1;
	mutex_unlock(&ec_chip->dock_init_lock);
	return ret_val;
}

static void asuspec_lid_report_function(struct work_struct *dat)
{
	int value = 0;

	wake_lock_timeout(&ec_chip->wake_lock, HZ/20);
	if (ec_chip->lid_indev == NULL){
		ASUSPEC_ERR("LID input device doesn't exist\n");
		return;
	}
	msleep(CONVERSION_TIME_MS);
	value = gpio_get_value(asuspec_hall_sensor_gpio);
	if (last_lid_gpio == value){
		ASUSPEC_ERR("same lid value! return\n");
		return;
	}else{
		last_lid_gpio = value;
	}
	input_report_switch(ec_chip->lid_indev, SW_LID, !value);
	input_sync(ec_chip->lid_indev);
	ASUSPEC_NOTICE("SW_LID report value = %d\n", !value);
}

static void asusdec_dock_init_work_function(struct work_struct *dat)
{
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;
	int err_count = 3;
	int ret = 0;
	int dock_in_stat = 0;
	tp_in_ioctl = 0;
	first_tp_ioctl = 0;
	ec_chip->touchpad_member = -1;

	memset(ec_chip->i2c_old_kb_data, 0, 38);

	if (!asuspec_check_dock_in_control_flag()){
		ASUSPEC_NOTICE("No dock detected\n");
		if(!gpio_get_value(asuspec_dock_in_gpio)){
			if(dock_init_retry-- > 0){
				ASUSPEC_ERR("dock in detect! something wrong with control flag\n");
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, HZ/5);
				return;
			}
			dock_init_retry = 3;
			ASUSPEC_ERR("dock init failed! remove dock!\n");
		}
		ec_chip->tp_enable = 0;
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->dock_status = 0;
		ec_chip->dock_init = 0;
		ec_chip->dock_type = DOCK_UNKNOWN;
		ec_chip->touchpad_member = -1;
		finish_touchpad_init = 0;
		dock_in_stat = 0;
		hid_device_sleep = 0;

		memset(ec_chip->dec_model_name, 0, 32);
		memset(ec_chip->dec_version, 0, 32);

		//unregister input device
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}
		if (ec_chip->private->abs_dev){
			input_unregister_device(ec_chip->private->abs_dev);
			ec_chip->private->abs_dev = NULL;
		}
		if(ec_chip->kb_and_ps2_enable){
			disable_irq_nosync(gpio_to_irq(asuspec_kb_int_gpio));
			disable_irq_nosync(gpio_to_irq(asuspec_ps2_int_gpio));
			ec_chip->kb_and_ps2_enable = 0;
		}

		if(finish_first_dock_init == 1){
			elantech_i2c_command(&kb_client, ASUS_KB_SLEEP_CMD, ec_chip->i2c_kb_data, 0);
		}
		asusdec_dock_status_report();
#if BATTERY_DRIVER
		if(dock_in_stat != last_dock_in_stat)
			for_asuspec_call_me_back_if_dock_in_status_change(false);
#endif
		last_dock_in_stat = 0;
		finish_first_dock_init = 1;
		touchpad_enable_flag = ASUSDEC_TP_ON;
#if EMC_NOTIFY
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
#endif
		cancel_delayed_work(&ec_chip->asusdec_tp_enable_work);
		return;
	}else {
		ASUSPEC_NOTICE("Dock-in detected\n");
		ec_chip->dock_type = MOBILE_DOCK;
		ec_chip->dock_in = 1;
		dock_in_stat = 1;
		tp_init_retry = 3;
		ASUSPEC_INFO("dock_in_gpio : %d\n",gpio_get_value(asuspec_dock_in_gpio));
		if(ec_chip->dock_status != 1){

			if (asusdec_dockram_read_data(0x02) < 0){
				goto fail_to_access_ec;
			}
			msleep(50);
			strcpy(ec_chip->dec_version, &ec_chip->i2c_dm_data[0]);
			ASUSPEC_NOTICE("DEC-FW Version: %s\n", ec_chip->dec_version);
			if(strncmp(ec_chip->dec_version, "DOCK-EC", 6)){
				if(!gpio_get_value(asuspec_dock_in_gpio)){
					ASUSPEC_ERR("dock in detect with wrong version!\n");
					queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, HZ/2);
					return;
				}
				ASUSPEC_ERR("fail to init dock--wrong dock version: %s\n", ec_chip->dec_model_name);
				ec_chip->dock_type = DOCK_UNKNOWN;
				ec_chip->dock_in = 0;
				return;
			}

			if (asuspec_dockram_read_data(0x23) < 0){
				goto fail_to_access_ec;
			}
			msleep(50);
			ec_chip->dock_behavior = ec_chip->i2c_dm_data[2] & 0x02;
			ASUSPEC_NOTICE("DEC-FW Behavior: %s\n", ec_chip->dock_behavior ?
				"susb on when receive ec_req" : "susb on when system wakeup");
		}
#if FACTORY_MODE
		if(factory_mode == 2)
			asusdec_enter_factory_mode();
#endif

		asusdec_input_device_create(&kb_client);

		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, HZ/2);

		////kb_init
		memset(&ec_chip->i2c_kb_data, 0, 32);
		//kb hid power on
		while(err_count-- > 0)
		{
			ret = elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0);
			if(ret < 0){
				kb_power_on = 0;
				ASUSPEC_ERR("set power write kb_client error: %d !\n", ret);
				msleep(10);
			}else{
				kb_power_on = 1;
				ASUSPEC_NOTICE("keyboard power on!\n");
				break;
			}
		}
		err_count = 3;
		msleep(10);
		while(err_count-- > 0)
		{
			ret = elantech_i2c_command(&kb_client, ASUS_KB_RESET_CMD, ec_chip->i2c_kb_data, 0);
			if(ret < 0){
				ASUSPEC_ERR("reset kb_client error: %d !\n", ret);
				msleep(10);
			}else{
				ASUSPEC_NOTICE("keyboard reset!\n");
				break;
			}
		}

		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, HZ/2);

		//enable kb_int irq
		if(ec_chip->kb_and_ps2_enable == 0){
			ec_chip->tp_enable = 1;
			enable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
			enable_irq(gpio_to_irq(asuspec_kb_int_gpio));
			ec_chip->kb_and_ps2_enable = 1;
		}
#if BATTERY_DRIVER
		if(1 != last_dock_in_stat && finish_first_dock_init == 0)
			for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif
		ec_chip->init_success = 1;
		ec_chip->dock_status = 1;
		hid_device_sleep = 0;
		asusdec_dock_status_report();
		last_dock_in_stat = 1;
		finish_first_dock_init = 1;
		return;
	}
fail_to_access_ec:
	if (asusdec_dockram_read_data(0x00) < 0){
		ASUSPEC_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		ASUSPEC_NOTICE("Need EC FW update\n");
		//asusdec_fw_reset();FIXME:
	}
}

static void asuspec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int i = 0;
	int ret = 0;

	mutex_lock(&ec_chip->lock);

        switch (fu_type){
        case UPDATE_PAD_BYTE_MODE:
    	    smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
    	    enable_irq(irq);
    	    BuffPush(smbus_data);
            break;
        case UPDATE_PAD_BLOCK_MODE:
    	    ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x6a, 32, ec_chip->i2c_fu_data);
    	    if (ret < 0)
    	    	ASUSPEC_ERR("fu work Fail to read data, status %02x\n", ret);
    	    for (i = 0; i < ec_chip->i2c_fu_data[0] + 2 ; i++){
    	    	BuffPush(ec_chip->i2c_fu_data[i]);//FIXME:only need push fa
    	    }
            enable_irq(irq);
            break;
        case UPDATE_DOCK_BYTE_MODE:
    	    memset(&ec_chip->i2c_fu_data, 0, 32);
    	    ec_chip->i2c_fu_data[0] = 0x05;
    	    ec_chip->i2c_fu_data[1] = 0x07;//i2c read byte
    	    ec_chip->i2c_fu_data[2] = 0x00;//result :read only
    	    ec_chip->i2c_fu_data[3] = 0x36;//8bit dock i2c address
    	    ec_chip->i2c_fu_data[4] = 0x6a;//cmd
    	    ec_chip->i2c_fu_data[5] = 0x01;//read byte number
            ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 6, ec_chip->i2c_fu_data);
            if (ret < 0) {
                    ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
            }
    	    msleep(10);
            ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x11, 32, ec_chip->i2c_fu_data);
            if (ret < 0) {
                    ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
            }
            ASUSPEC_INFO("read :  %02x\n", ec_chip->i2c_fu_data[9]);
            BuffPush(ec_chip->i2c_fu_data[9]);
            break;
        case UPDATE_DOCK_BLOCK_MODE:
    	    ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x6a, 32, ec_chip->i2c_fu_data);
    	    if (ret < 0)
    	    	ASUSPEC_ERR("fu work Fail to read data, status %02x\n", ret);
	    if (ec_chip->i2c_fu_data[2] == 0x61 && ec_chip->i2c_fu_data[1] == 0x41){
		ASUSPEC_INFO("send to dock success\n");
	    }else if (ec_chip->i2c_fu_data[1] == 0x81){
		ASUSPEC_NOTICE("pad apwake event in dock update mode: do nothing\n");
	    }else if (ec_chip->i2c_fu_data[1] == 0xC1 && ec_chip->i2c_fu_data[2] == 0x0C){
	    	    for (i = 0; i <= ec_chip->i2c_fu_data[0] ; i++){
	    	    	BuffPush(ec_chip->i2c_fu_data[i]);//FIXME:only need push fa, skip 0xca0c
	    	    }
	    }else{
		ASUSPEC_ERR("special case event: %x\n",ec_chip->i2c_fu_data[1]);
	    }
            enable_irq(irq);
            break;
        default:
            break;
        }
	mutex_unlock(&ec_chip->lock);
}

static void asuspec_work_function(struct work_struct *dat)
{
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	ret_val = asuspec_i2c_read_data(ec_chip->client);

	enable_irq(irq);

	ASUSPEC_NOTICE("0x%x 0x%x 0x%x 0x%x\n", ec_chip->i2c_data[0],
		ec_chip->i2c_data[1], ec_chip->i2c_data[2], ec_chip->i2c_data[3]);

	if (ret_val < 0){
		return ;
	}

	if (ec_chip->i2c_data[1] & ASUSPEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSPEC_SMI_MASK){
			asuspec_smi();
			return ;
		}else if(ec_chip->i2c_data[1] & ASUSDEC_SCI_MASK){
			if(machine_is_mozart()){
				if(ec_chip->i2c_data[2] >= 0 && ec_chip->i2c_data[2] < 24)
					asusdec_kp_sci();
			}
		}
	}
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
        int i = 0;
        set_bit(EV_KEY, dev->evbit);
        for ( i = 0; i < 246; i++)
                set_bit(i,dev->keybit);

	set_bit(REL_X, dev->evbit);
	set_bit(REL_Y, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);

	set_bit(EV_REL, dev->evbit);
        set_bit(REL_X, dev->relbit);
        set_bit(REL_Y, dev->relbit);
	set_bit(EV_SYN, dev->evbit);

        input_set_capability(dev, EV_LED, LED_CAPSL);
}

static int asusdec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value){
        ASUSPEC_INFO("type = 0x%x, code = 0x%x, value = 0x%x\n", type, code, value);
        if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
                if ((type == EV_LED) && (code == LED_CAPSL)){
                        if(value == 0){
                                //queue_delayed_work(asusdec_wq, &ec_chip->asusdec_led_off_work, 0);
                                return 0;
                        } else {
                                //queue_delayed_work(asusdec_wq, &ec_chip->asusdec_led_on_work, 0);
                                return 0;
                        }
                }
        }
        return -ENOTTY;
}

int asuspec_dock_p5vsus_control(bool activate){//need pad ec firmware newer than 9615
	int ret_val = 0;
	if(machine_is_mozart()){
		memset(&ec_chip->i2c_dm_data, 0, 32);
		ec_chip->i2c_dm_data[0] = 0x08;
		if(activate){
			ec_chip->i2c_dm_data[7] = 0x10;
		}else{
			ec_chip->i2c_dm_data[3] = 0x10;
		}
		ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x0a, 9, ec_chip->i2c_dm_data);
		if(ret_val < 0){
			ASUSPEC_ERR("fail to write dockram data status : %d\n",ret_val);
			return -1;
		}
	}
	return 0;
}
EXPORT_SYMBOL(asuspec_dock_p5vsus_control);

static void asusdec_lid_set_input_params(struct input_dev *dev)
{
	set_bit(EV_SW, dev->evbit);
	set_bit(SW_LID, dev->swbit);
}

static int asusdec_input_device_create(struct i2c_client *client){
	int err = 0;

	if (ec_chip->indev){
	        return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
	        ASUSPEC_ERR("input_dev allocation fails\n");
	        err = -ENOMEM;
	        goto exit;
	}
	ec_chip->indev->name = "asuspec";
	ec_chip->indev->phys = "/dev/input/asuspec";
	ec_chip->indev->event = asusdec_event;

	asusdec_keypad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
	        ASUSPEC_ERR("input registration fails\n");
	        goto exit_input_free;
	}
	return 0;

exit_input_free:
        input_free_device(ec_chip->indev);
        ec_chip->indev = NULL;
exit:
        return err;

}

static int asuspec_lid_input_device_create(struct i2c_client *client){
	int err = 0;

	ec_chip->lid_indev = input_allocate_device();
	if (!ec_chip->lid_indev) {
		ASUSPEC_ERR("lid_indev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->lid_indev->name = "lid_input";
	ec_chip->lid_indev->phys = "/dev/input/lid_indev";
	ec_chip->lid_indev->dev.parent = &client->dev;

	asusdec_lid_set_input_params(ec_chip->lid_indev);
	err = input_register_device(ec_chip->lid_indev);
	if (err) {
		ASUSPEC_ERR("lid_indev registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->lid_indev);
	ec_chip->lid_indev = NULL;
exit:
	return err;

}

static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ASUSPEC_INFO("asuspec probe\n");
	err = sysfs_create_group(&client->dev.kobj, &asuspec_smbus_group);
	if (err) {
		ASUSPEC_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct asuspec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSPEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

        ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
        if (!ec_chip->private) {
                ASUSPEC_ERR("Memory allocation (elantech_data) fails\n");
                err = -ENOMEM;
                goto exit;
        }

	i2c_set_clientdata(client, ec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &asuspec_driver;
	ec_chip->client->flags = 1;

	init_timer(&ec_chip->asuspec_timer);
	ec_chip->asuspec_timer.function = asuspec_enter_s3_timer;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asuspec_wake");
	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->irq_lock);
	mutex_init(&ec_chip->state_change_lock);
	mutex_init(&ec_chip->dock_init_lock);

        ec_chip->indev = NULL;
        ec_chip->lid_indev = NULL;
        ec_chip->private->abs_dev = NULL;

	ec_chip->ec_ram_init = 0;
	ec_chip->audio_recording = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;

	ec_chip->suspend_state = 0;
	ec_chip->dock_status = 0;
	ec_chip->dock_init = 0;
	ec_chip->kb_and_ps2_enable = 0;
	ec_chip->ec_wakeup = 0;
	asuspec_dockram_init(client);
	asuspec_tp_init(client);
	asuspec_kb_init(client);
	cdev_add(asuspec_cdev,asuspec_dev,1) ;

	ec_chip->pad_sdev.name = PAD_SDEV_NAME;
	ec_chip->pad_sdev.print_name = asuspec_switch_name;
	ec_chip->pad_sdev.print_state = asuspec_switch_state;
	if(switch_dev_register(&ec_chip->pad_sdev) < 0){
		ASUSPEC_ERR("switch_dev_register for pad failed!\n");
	}
	switch_set_state(&ec_chip->pad_sdev, 0);

	old_pad_battery_time.tv_sec = 0;
	old_pad_battery_time.tv_usec = 0;
	old_dock_battery_time.tv_sec = 0;
	old_dock_battery_time.tv_usec = 0;
	old_dock_retry_time.tv_sec = 0;
	old_dock_retry_time.tv_usec = 0;

	if(machine_is_mozart()){
		ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
		ec_chip->dock_sdev.print_name = asusdec_switch_name;
		ec_chip->dock_sdev.print_state = asusdec_switch_state;
		if(switch_dev_register(&ec_chip->dock_sdev) < 0){
			ASUSPEC_ERR("switch_dev_register for dock failed!\n");
			goto exit;
		}
		switch_set_state(&ec_chip->dock_sdev, 0);
		asuspec_lid_input_device_create(ec_chip->client);
	}
	if(machine_is_haydn()){
		ec_chip->apower_sdev.name = APOWER_SDEV_NAME;
		ec_chip->apower_sdev.print_name = apower_switch_name;
		ec_chip->apower_sdev.print_state = apower_switch_state;
		ec_chip->apower_state = 0;
		if(switch_dev_register(&ec_chip->apower_sdev) < 0){
			ASUSPEC_ERR("switch_dev_register for apower failed!\n");
		}
		switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	}

	asuspec_wq = create_singlethread_workqueue("asuspec_wq");
	asuspec_tp_wq = create_singlethread_workqueue("asuspec_tp_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_work, asuspec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_init_work, asuspec_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_fw_update_work, asuspec_fw_update_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_enter_s3_work, asuspec_enter_s3_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&asuspec_stress_work, asuspec_stresstest_work_function);

	if(machine_is_mozart()){
		INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_hall_sensor_work, asuspec_lid_report_function);
		INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
		INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_kb_report_work, asusdec_kb_report_work_function);
		INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_tp_report_work, asusdec_tp_report_work_function);
		INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_tp_enable_work, asusdec_tp_enable_work_function);
	}

	asuspec_irq_ec_request(client);
	asuspec_irq_ec_apwake(client);
	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	if(machine_is_mozart()){
		asuspec_irq_hall_sensor(client);
		asuspec_irq_ps2_int(&tp_client);
		asuspec_irq_kb_int(client);
		asuspec_irq_battery_id(client);
		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 8*HZ);
	}

	return 0;

exit:
	return err;
}

static int __devexit asuspec_remove(struct i2c_client *client)
{
	struct asuspec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t asuspec_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

static ssize_t asuspec_gauge_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", asuspec_get_gauge_mode(1));
}

static ssize_t asuspec_dock_gauge_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", asuspec_get_gauge_mode(0));
}

static ssize_t asuspec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_version_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_battery_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_status, bat_temp, bat_vol, bat_current, bat_capacity, remaining_cap;
	int avg_to_empty, avg_to_full, full_cap, design_cap, design_vol, cycle_cnt;
	int ret_val;
	char temp_buf[64];

	bat_status = asuspec_battery_monitor("status", 1);
	bat_temp = asuspec_battery_monitor("temperature", 1);
	bat_vol = asuspec_battery_monitor("voltage", 1);
	bat_current = asuspec_battery_monitor("current", 1);
	bat_capacity = asuspec_battery_monitor("capacity", 1);
	remaining_cap = asuspec_battery_monitor("remaining_capacity", 1);
	avg_to_empty = asuspec_battery_monitor("avg_time_to_empty", 1);
	avg_to_full = asuspec_battery_monitor("avg_time_to_full", 1);
	full_cap = asuspec_battery_monitor("full_capacity", 1);
	design_cap = asuspec_battery_monitor("design_capacity", 1);
	design_vol = asuspec_battery_monitor("design_voltage", 1);
	cycle_cnt = asuspec_battery_monitor("cycle_count", 1);

	if (ret_val < 0)
		return sprintf(buf, "fail to get battery info\n");
	else {
		sprintf(temp_buf, "status = 0x%x\n", bat_status);
		strcpy(buf, temp_buf);
		sprintf(temp_buf, "temperature = %d\n", bat_temp);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "voltage = %d\n", bat_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "current = %d\n", bat_current);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "capacity = %d\n", bat_capacity);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "remaining capacity = %d\n", remaining_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to empty = %d\n", avg_to_empty);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to full = %d\n", avg_to_full);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "full capacity = %d\n", full_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "desing capacity = %d\n", design_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "design voltage= %d\n", design_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "cycle count = %d\n", cycle_cnt);
		strcat(buf, temp_buf);

		return strlen(buf);
	}
}

static ssize_t asuspec_dock_battery_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_status, bat_temp, bat_vol, bat_current, bat_capacity, remaining_cap;
	int avg_to_empty, avg_to_full, full_cap, design_cap, design_vol, cycle_cnt;
	int ret_val;
	char temp_buf[64];

	bat_status = asuspec_battery_monitor("status", 0);
	bat_temp = asuspec_battery_monitor("temperature", 0);
	bat_vol = asuspec_battery_monitor("voltage", 0);
	bat_current = asuspec_battery_monitor("current", 0);
	bat_capacity = asuspec_battery_monitor("capacity", 0);
	remaining_cap = asuspec_battery_monitor("remaining_capacity", 0);
	avg_to_empty = asuspec_battery_monitor("avg_time_to_empty", 0);
	avg_to_full = asuspec_battery_monitor("avg_time_to_full", 0);
	full_cap = asuspec_battery_monitor("full_capacity", 0);
	design_cap = asuspec_battery_monitor("design_capacity", 0);
	design_vol = asuspec_battery_monitor("design_voltage", 0);
	cycle_cnt = asuspec_battery_monitor("cycle_count", 0);

	if (ret_val < 0)
		return sprintf(buf, "fail to get battery info\n");
	else {
		sprintf(temp_buf, "status = 0x%x\n", bat_status);
		strcpy(buf, temp_buf);
		sprintf(temp_buf, "temperature = %d\n", bat_temp);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "voltage = %d\n", bat_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "current = %d\n", bat_current);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "capacity = %d\n", bat_capacity);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "remaining capacity = %d\n", remaining_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to empty = %d\n", avg_to_empty);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to full = %d\n", avg_to_full);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "full capacity = %d\n", full_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "desing capacity = %d\n", design_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "design voltage= %d\n", design_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "cycle count = %d\n", cycle_cnt);
		strcat(buf, temp_buf);

		return strlen(buf);
	}
}

static ssize_t asuspec_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x0A);
	if (ret_val < 0)
		return sprintf(buf, "fail to get pad ec control-flag info\n");
	else{
		sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
		strcpy(buf, temp_buf);
		for (i = 1; i < 9; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
		}
		return strlen(buf);
	}
}
static ssize_t asuspec_dock_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
        int i = 0;
        char temp_buf[64];
        int ret_val = 0;
//FIXME:if no dock retrun -1
//        if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
                ret_val = asuspec_dockram_read_data(0x23);

                if (ret_val < 0)
                        return sprintf(buf, "fail to get control-flag info\n");
                else{
                        sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
                        strcpy(buf, temp_buf);
                        for (i = 1; i < 9; i++){
                                sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
                                strcat(buf, temp_buf);
                        }
                        return strlen(buf);
                }
//        }

        return sprintf(buf, "fail to get control-flag info\n");
}

static ssize_t asuspec_send_ec_req_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_send_ec_req();
	return sprintf(buf, "EC_REQ is sent\n");
}


static ssize_t asuspec_charging_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret_val = 0;

	if (ec_chip->op_mode == 0){
		asuspec_dockram_read_data(0x0A);
		if (buf[0] == '0'){
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[1] = 0;
			ec_chip->i2c_dm_data[2] = 0x06;//clean orange and green led bit
			ec_chip->i2c_dm_data[3] = 0;
			ec_chip->i2c_dm_data[4] = 0;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0){
				ASUSPEC_NOTICE("Fail to diable led test\n");
				led_success = 0;
			}else{
				ASUSPEC_NOTICE("Diable led test\n");
				led_success = 1;
			}
		} else if (buf[0] == '1'){
			asuspec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[1] = 0;
			ec_chip->i2c_dm_data[2] = 0x04;//clean green led bit
			ec_chip->i2c_dm_data[3] = 0;
			ec_chip->i2c_dm_data[4] = 0;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x02;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0){
				ASUSPEC_NOTICE("Fail to enable orange led test\n");
				led_success = 0;
			}else {
				ASUSPEC_NOTICE("Enable orange led test\n");
				led_success = 1;
			}
		} else if (buf[0] == '2'){
			asuspec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[1] = 0;
			ec_chip->i2c_dm_data[2] = 0x02;//clean orange led bit
			ec_chip->i2c_dm_data[3] = 0;
			ec_chip->i2c_dm_data[4] = 0;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x04;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0){
				ASUSPEC_NOTICE("Fail to enable green led test\n");
				led_success = 0;
			}else{
				ASUSPEC_NOTICE("Enable green led test\n");
				led_success = 1;
			}
		} else{
				ASUSPEC_NOTICE("error cmd : %c\n",buf[0]);
		}
	} else {
		ASUSPEC_NOTICE("Fail to enter led test\n");
		led_success = 0;
	}
	ASUSPEC_NOTICE("led test result: %d\n",led_success);
	return sprintf(buf,"%d\n", led_success);
}

static ssize_t asuspec_led_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return 0;
}

static ssize_t asuspec_enter_factory_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_enter_factory_mode();
	return sprintf(buf, "Entering factory mode\n");
}

static ssize_t asuspec_enter_normal_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_enter_normal_mode();
	return sprintf(buf, "Entering normal mode\n");
}

static ssize_t asuspec_pad_base_sync_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;

	ret_val = asuspec_dockram_read_data(0x0A);
	if (ret_val < 0)
		return sprintf(buf, "fail to get pad and base sync state\n");
	else{
		if(ec_chip->i2c_dm_data[8] & 0x02)
			sprintf(buf, "1\n");
		else
			sprintf(buf, "0\n");
		return strlen(buf);
	}
}

static ssize_t asusdec_mute_dock_boost_power_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	u8 i2c_data[16] = {0x11,0x0e,0x00,0x36,0x0a,0x09,0x00,0x00,0x00,0x08,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00};
	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x12, i2c_data);
	if(ret < 0)
		return sprintf(buf, "mute dock boost power fail!\n");
	else
		return sprintf(buf, "mute dock boost power show!\n");
}

static ssize_t asuspec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_switch_state(struct switch_dev *sdev, char *buf)
{
	if (201) {
		return sprintf(buf, "%s\n", "0");
	}
}

static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", APOWER_SDEV_NAME);
}

static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->apower_state);
}

static ssize_t asuspec_show_lid_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(asuspec_hall_sensor_gpio));
}

static int asuspec_suspend(struct i2c_client *client, pm_message_t mesg){
	printk("asuspec_suspend+\n");
	if(machine_is_mozart())
		cancel_delayed_work_sync(&ec_chip->asusdec_dock_init_work);
	flush_workqueue(asuspec_wq);
	if(machine_is_mozart()){
		cancel_delayed_work_sync(&ec_chip->asusdec_tp_enable_work);
		flush_workqueue(asuspec_tp_wq);
	}
	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	ec_chip->dock_init = 0;
	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 1;
	tp_in_ioctl = 0;
	if(machine_is_mozart()){
		if(ec_chip->ec_wakeup == 0 && ec_chip->dock_in == 1 && hid_device_sleep == 0){
			ASUSPEC_NOTICE("HID device sleep\n");
			hid_device_sleep = 1;
			elantech_i2c_command(&kb_client, ASUS_KB_SLEEP_CMD, ec_chip->i2c_kb_data, 0);
		}
		if(asuspec_check_hid_control_flag() && ec_chip->dock_in == 1 && touchpad_enable_flag == ASUSDEC_TP_ON){
			asuspec_tp_enable(0xf5);
		}
	}
	printk("asuspec_suspend-\n");
	return 0;
}

static int asuspec_resume(struct i2c_client *client){
	printk("asuspec_resume+\n");
        ec_chip->suspend_state = 0;
        ec_chip->dock_det = 0;
        ec_chip->init_success = 0;
        ec_chip->ec_in_s3 = 1;

	ec_chip->i2c_err_count = 0;
	if(machine_is_mozart()){
		if(!asuspec_check_dock_in_control_flag() && ec_chip->dock_status == 1){
			ASUSPEC_ERR("no mobile dock flag but dock_status = 1\n");
		}else if(ec_chip->ec_wakeup == 0 && ec_chip->dock_status == 1 && hid_device_sleep == 1){
			ASUSPEC_NOTICE("HID device wakeup\n");
			hid_device_sleep = 0;
			elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0);
		}
		if(asuspec_check_hid_control_flag() && ec_chip->dock_in == 1 && touchpad_enable_flag == ASUSDEC_TP_ON){
			ASUSPEC_NOTICE("touchpad wakeup\n");
			asuspec_tp_enable(0xf4);
		}
	}
	printk("asuspec_resume-\n");
	return 0;
}

static int asusdec_set_wakeup_cmd(void){
	int ret_val = 0;

	memset(&ec_chip->i2c_dm_data, 0, 32);
	ec_chip->i2c_dm_data[0] = 0x08;
	if (ec_chip->dock_in){
		if (ec_chip->ec_wakeup){
			ec_chip->i2c_dm_data[5] = 0xa0;
			ASUSPEC_NOTICE("hold dock power when suspend!\n");
		} else {
			ec_chip->i2c_dm_data[1] = 0x80;
			ec_chip->i2c_dm_data[5] = 0x20;
			ASUSPEC_NOTICE("release dock power when suspend!\n");
		}
		asuspec_dockram_write_data(0x0A,9);
	}
	return 0;
}

static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->dec_version);
}

static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", switch_value[ec_chip->dock_type]);
}

static int asuspec_open(struct inode *inode, struct file *flip){
	ASUSPEC_NOTICE("\n");
	return 0;
}
static int asuspec_release(struct inode *inode, struct file *flip){
	ASUSPEC_NOTICE("\n");
	return 0;
}

static long asuspec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char name_buf[64];
	int length = 0;
	char *envp[3];
	int env_offset = 0;

	if (_IOC_TYPE(cmd) != ASUSPEC_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSPEC_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	if(machine_is_mozart()){
		switch (cmd) {
	        case ASUSPEC_POLLING_DATA:
			if (arg == ASUSPEC_IOCTL_HEAVY){
				ASUSPEC_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSPEC_IOCTL_NORMAL){
				ASUSPEC_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSPEC_IOCTL_END){
				ASUSPEC_NOTICE("polling end\n");
		    	cancel_delayed_work_sync(&asuspec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSPEC_FW_UPDATE:
			ASUSPEC_NOTICE("ASUSPEC_FW_UPDATE\n");
			mutex_lock(&ec_chip->state_change_lock);
			msleep(200);
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			h2ec_count = 0;
			memset(host_to_ec_buffer, 0, EC_BUFF_LEN);
			memset(ec_to_host_buffer, 0, EC_BUFF_LEN);
			memset(&ec_chip->i2c_dm_data, 0, 32);
			ec_chip->status = 0;
			ec_chip->op_mode = 1;
			wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			msleep(2400);
			switch(arg){
			case 0:
				ASUSPEC_ERR("ASUSPEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
				switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
				msleep(2500);
				return -ENOTTY;
				break;
			case 1:
				ASUSPEC_NOTICE("ASUSPEC_FW_UPDATE use block mode\n");
				fu_block_mode = 1;
				fu_type = UPDATE_PAD_BLOCK_MODE;
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				break;
			case 2:
				if (ec_chip->dock_in){
					ASUSPEC_NOTICE("ASUSDEC_FW_UPDATE use byte mode\n");
					fu_type = UPDATE_DOCK_BYTE_MODE;
					ec_chip->i2c_dm_data[0] = 0x0b;
					ec_chip->i2c_dm_data[1] = 0x0e;
					ec_chip->i2c_dm_data[2] = 0x00;
					ec_chip->i2c_dm_data[3] = 0x36;
					ec_chip->i2c_dm_data[4] = 0x40;
					ec_chip->i2c_dm_data[5] = 0x03;
					ec_chip->i2c_dm_data[9] = 0x02;
					ec_chip->i2c_dm_data[10] = 0x55;
					ec_chip->i2c_dm_data[11] = 0xaa;
					i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x0c, ec_chip->i2c_dm_data);
				} else {
					ASUSPEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			case 3:
				if (ec_chip->dock_in){
					ASUSPEC_NOTICE("ASUSPEC_dock FW_UPDATE use block mode\n");
					fu_block_mode = 1;
					fu_type = UPDATE_DOCK_BLOCK_MODE;
					ec_chip->i2c_dm_data[0] = 0x0b;
					ec_chip->i2c_dm_data[1] = 0x0a;
					ec_chip->i2c_dm_data[2] = 0x00;
					ec_chip->i2c_dm_data[3] = 0x36;
					ec_chip->i2c_dm_data[4] = 0x41;
					ec_chip->i2c_dm_data[5] = 0x02;
					ec_chip->i2c_dm_data[9] = 0x55;
					ec_chip->i2c_dm_data[10] = 0xaa;
					i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x0b, ec_chip->i2c_dm_data);
				} else {
					ASUSPEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			default:
				ASUSPEC_ERR("error fu type!\n");
				break;
			}
			msleep(1000);
			mutex_unlock(&ec_chip->state_change_lock);
			break;
		case ASUSPEC_INIT:
			ASUSPEC_NOTICE("ASUSPEC_INIT\n");
			msleep(500);
			ec_chip->status = 0;
			ec_chip->op_mode = 0;
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			switch(fu_type){
			case UPDATE_PAD_BYTE_MODE:
			case UPDATE_PAD_BLOCK_MODE:
				queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
				switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
				break;
			case UPDATE_DOCK_BYTE_MODE:
			case UPDATE_DOCK_BLOCK_MODE:
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
				msleep(2500);
	ASUSPEC_NOTICE("ASUSDEC_INIT - EC version: %s\n", ec_chip->dec_version);
	length = strlen(ec_chip->dec_version);
	ec_chip->dec_version[length] = NULL;
	snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->dec_version);
	envp[env_offset++] = name_buf;
	envp[env_offset] = NULL;
	kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
				break;
			default:
				ASUSPEC_ERR("ASUSPEC_INIT unknow case!\n");
				break;
			}
			msleep(2500);
			break;
		case ASUSDEC_TP_CONTROL:
			ASUSPEC_NOTICE("ASUSDEC_TP_CONTROL\n");
			if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
				err = asusdec_tp_control(arg);
				return err;
			}
			else
				return -ENOTTY;
			break;
		case ASUSDEC_EC_WAKEUP:
			msleep(500);
			ASUSPEC_NOTICE("ASUSDEC_EC_WAKEUP, arg = %d\n", arg);
			if (arg == ASUSDEC_EC_OFF){
				ec_chip->ec_wakeup = 0;
				ASUSPEC_NOTICE("Set EC shutdown when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}else if (arg == ASUSDEC_EC_ON){
				ec_chip->ec_wakeup = 1;
				ASUSPEC_NOTICE("Keep EC active when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}else{
				ASUSPEC_ERR("Unknown argument");
				return -ENOTTY;
			}
		case ASUSPEC_FW_DUMMY:
			ASUSPEC_NOTICE("ASUSPEC_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			switch(fu_type){
			case UPDATE_PAD_BYTE_MODE:
				ASUSPEC_ERR("dont support byte mode\n");
				break;
			case UPDATE_PAD_BLOCK_MODE:
				fu_block_mode = 1;
				ASUSPEC_NOTICE("ASUSPEC_FW_DUMMY pad block mode\n");
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x41, 3, ec_chip->i2c_dm_data);
				dockram_client.flags = I2C_CLIENT_PEC;
				break;
			case UPDATE_DOCK_BYTE_MODE:
				if (ec_chip->dock_in){
					ASUSPEC_NOTICE("ASUSDEC_FW_UPDATE use byte mode\n");
					ec_chip->i2c_dm_data[0] = 0x0b;
					ec_chip->i2c_dm_data[1] = 0x0e;
					ec_chip->i2c_dm_data[2] = 0x00;
					ec_chip->i2c_dm_data[3] = 0x36;
					ec_chip->i2c_dm_data[4] = 0x40;
					ec_chip->i2c_dm_data[5] = 0x03;
					ec_chip->i2c_dm_data[9] = 0x02;
					ec_chip->i2c_dm_data[10] = 0x55;
					ec_chip->i2c_dm_data[11] = 0xaa;
					i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x0c, ec_chip->i2c_dm_data);
				} else {
					ASUSPEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			case UPDATE_DOCK_BLOCK_MODE:
				if (ec_chip->dock_in){
					ASUSPEC_NOTICE("ASUSPEC_dock FW_UPDATE use block mode\n");
					fu_block_mode = 1;
					fu_type = UPDATE_DOCK_BLOCK_MODE;
					ec_chip->i2c_dm_data[0] = 0x0b;
					ec_chip->i2c_dm_data[1] = 0x0a;
					ec_chip->i2c_dm_data[2] = 0x00;
					ec_chip->i2c_dm_data[3] = 0x36;
					ec_chip->i2c_dm_data[4] = 0x41;
					ec_chip->i2c_dm_data[5] = 0x02;
					ec_chip->i2c_dm_data[9] = 0x55;
					ec_chip->i2c_dm_data[10] = 0xaa;
					i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 0x0b, ec_chip->i2c_dm_data);
				} else {
					ASUSPEC_NOTICE("No dock detected\n");
					return -1;
				}
				break;
			default:
				ASUSPEC_ERR("error fu type!\n");
				break;
			}
			if(arg == 1 || arg == 3){
			}
			else{
				ASUSPEC_ERR("ASUSPEC_FW_UPDATE:forbid byte mode update to prevent update fail!\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
				switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
				msleep(2500);
				return -ENOTTY;
			}
			break;
	        default: /* redundant, as cmd was checked against MAXNR */
	            return -ENOTTY;
		}
	}
	if(machine_is_haydn()){
		switch (cmd) {
	        case ASUSPEC_POLLING_DATA:
				if (arg == ASUSPEC_IOCTL_HEAVY){
					ASUSPEC_NOTICE("heavy polling\n");
					ec_chip->polling_rate = 80;
					queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
				}
				else if (arg == ASUSPEC_IOCTL_NORMAL){
					ASUSPEC_NOTICE("normal polling\n");
					ec_chip->polling_rate = 10;
					queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
				}
				else if  (arg == ASUSPEC_IOCTL_END){
					ASUSPEC_NOTICE("polling end\n");
			    	cancel_delayed_work_sync(&asuspec_stress_work) ;
				}
				else
					return -ENOTTY;
				break;
			case ASUSPEC_FW_UPDATE:
				ASUSPEC_NOTICE("ASUSPEC_FW_UPDATE\n");
				mutex_lock(&ec_chip->state_change_lock);
				asuspec_send_ec_req();
				msleep(200);
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				h2ec_count = 0;
				memset(host_to_ec_buffer, 0, EC_BUFF_LEN);
				memset(ec_to_host_buffer, 0, EC_BUFF_LEN);
				ec_chip->status = 0;
				ec_chip->op_mode = 1;
				wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
				ec_chip->i2c_dm_data[0] = 0x02;
				ec_chip->i2c_dm_data[1] = 0x55;
				ec_chip->i2c_dm_data[2] = 0xAA;
				msleep(2400);
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
				msleep(1000);
				mutex_unlock(&ec_chip->state_change_lock);
				break;
			case ASUSPEC_INIT:
				ASUSPEC_NOTICE("ASUSPEC_INIT\n");
				msleep(500);
				ec_chip->status = 0;
				ec_chip->op_mode = 0;
				queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
				switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
				msleep(2500);
				break;
			case ASUSPEC_FW_DUMMY:
				ASUSPEC_NOTICE("ASUSPEC_FW_DUMMY\n");
				ec_chip->i2c_dm_data[0] = 0x02;
				ec_chip->i2c_dm_data[1] = 0x55;
				ec_chip->i2c_dm_data[2] = 0xAA;
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
				break;
			case ASUSPEC_SWITCH_HDMI:
				ASUSPEC_NOTICE("ASUSPEC_SWITCH_HDMI\n", arg);
				asuspec_switch_hdmi();
				break;
			case ASUSPEC_WIN_SHUTDOWN:
				ASUSPEC_NOTICE("ASUSPEC_WIN_SHUTDOWN\n", arg);
				asuspec_win_shutdown();
				break;
	        default: /* redundant, as cmd was checked against MAXNR */
	            return -ENOTTY;
		}
	}
    return 0;
}

static void asuspec_switch_apower_state(int state){
	ec_chip->apower_state = state;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->apower_state = APOWER_IDLE;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
}

static void asuspec_win_shutdown(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x40;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Win shutdown command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Win shutdown\n");
			break;
		}
	}
}

static void asuspec_switch_hdmi(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x01;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Switch hdmi command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Switching hdmi\n");
			break;
		}
	}
}

static void asuspec_storage_info_update(void){
	int ret_val = 0;
	int i = 0;
	struct kstatfs st_fs;
	unsigned long long block_size;
	unsigned long long f_blocks;
	unsigned long long f_bavail;
	unsigned long long mb;

	ret_val = user_statfs("/data", &st_fs);
	if (ret_val < 0){
		ASUSPEC_ERR("fail to get data partition size\n");
		ec_chip->storage_total = 0;
		ec_chip->storage_avail = 0;
	} else {
		block_size = st_fs.f_bsize;
		f_blocks = st_fs.f_blocks;
		f_bavail = st_fs.f_bavail;
		mb = MB;
		ec_chip->storage_total = block_size * f_blocks / mb;
		ec_chip->storage_avail = block_size * f_bavail / mb;
		ASUSPEC_NOTICE("Storage total size = %ld, available size = %ld\n", ec_chip->storage_total, ec_chip->storage_avail);
	}

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_storageinfo(0x28);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get PadInfo\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_storage[0] = 8;
	ec_chip->i2c_dm_storage[1] = ec_chip->storage_total & 0xFF;
	ec_chip->i2c_dm_storage[2] = (ec_chip->storage_total >> 8) & 0xFF;
	ec_chip->i2c_dm_storage[3] = ec_chip->storage_avail & 0xFF;;
	ec_chip->i2c_dm_storage[4] = (ec_chip->storage_avail >> 8) & 0xFF;;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_storageinfo(0x28,9);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to write PadInfo\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Write PadInof Total[H][L]: 0x%x, 0x%x\n", ec_chip->i2c_dm_storage[2], ec_chip->i2c_dm_storage[1]);
			ASUSPEC_NOTICE("Write PadInof Avail[H][L]: 0x%x, 0x%x\n", ec_chip->i2c_dm_storage[4], ec_chip->i2c_dm_storage[3]);
			break;
		}
	}
}

static void asuspec_enter_factory_mode(void){

	int ret_val = 0;
	int i = 0;
	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	if(machine_is_haydn()){
		ASUSPEC_NOTICE("Entering factory mode haydn\n");
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
	}
	if(machine_is_mozart()){
		ASUSPEC_NOTICE("Entering factory mode mozart\n");
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[1] = 0;
		ec_chip->i2c_dm_data[2] = 0;
		ec_chip->i2c_dm_data[3] = 0;
		ec_chip->i2c_dm_data[4] = 0;
		ec_chip->i2c_dm_data[5] = 0;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Entering factory mode fail\n");
			msleep(100);
		}
		else {
			break;
		}
	}
}

static void asusdec_enter_factory_mode(void){
//FIXME:
//	SUSPEC_NOTICE("dock Entering factory mode\n");
//	susdec_dockram_read_data(0x0A);
//	ec_chip->i2c_dm_data[0] = 8;
//	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
//#if CSC_IMAGE
//	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;
//#endif
//	asusdec_dockram_write_data(0x0A,9);
}

static void asuspec_enter_normal_mode(void){

	int ret_val = 0;
	int i = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	if(machine_is_mozart()){
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x00;
		ec_chip->i2c_dm_data[1] = 0x40;
	}
	if(machine_is_haydn()){
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Entering normal mode fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Entering normal mode\n");
			break;
		}
	}
}

static ssize_t asuspec_cmd_data_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
       int buf_len = strlen(buf);
       int data_len = (buf_len -1)/2;
       char chr[2], data_num[data_len];
       int i=0, j=0, idx=0, data_cnt, ret_val;
	int ret = 0;
	struct elantech_data *etd = ec_chip->private;
       chr[2] = '\0';
       u8 cmd,i2c_type;

       memset(&ec_chip->i2c_fu_data, 0, 32);
       memset(&ec_chip->i2c_kb_data, 0, 38);
       memset(&ec_chip->i2c_tp_data, 0, 38);

       printk("buf_len=%d, data_len=%d \n",buf_len, data_len);

       if(!(buf_len&0x01) || !data_len){
               return -1;
       }
       for(i=0;i<buf_len-1;i++){
               chr[j] = *(buf+i);
               if(j==1){
                       if (i == 1) {
                               i2c_type = (u8) simple_strtoul (chr,NULL,16);
                       } else if (i == 3) {
                               cmd = (u8) simple_strtoul (chr,NULL,16);
                       } else
                               data_num[idx++] = (u8) simple_strtoul (chr,NULL,16);
               }
               j++;
               if(j>1){
                       j=0;
               }
       }
       data_num[idx] = '\0';
       data_cnt = data_len - 2;//remove  i2c_type and cmd

       if(data_cnt > 32) {
               printk("Input data count is over length\n");
               return -1;
       }

       memcpy(&ec_chip->i2c_fu_data[0], data_num, data_cnt);
       memcpy(&ec_chip->i2c_kb_data[0], data_num, data_cnt);
       memcpy(&ec_chip->i2c_tp_data[0], data_num, data_cnt);

       printk("I2c type=0x%x\n", i2c_type);
       printk("I2c cmd=0x%x\n", cmd);
       for(i=0; i<data_cnt; i++){
               printk("I2c_fu_data[%d]=0x%x\n",i, ec_chip->i2c_fu_data[i]);
       }
	switch(i2c_type){
		case 0x01:
			//push data length than push data
			ret = i2c_smbus_write_block_data(&dockram_client, cmd, data_cnt, ec_chip->i2c_fu_data);
			printk("case1 i2c_smbus_write_block_data 0x%x\n",cmd);
			break;
		case 0x02:
			//push data only, you should add length in byte 0
			ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, data_cnt, ec_chip->i2c_fu_data);
			printk("case2 i2c_smbus_write_i2c_block_data%x\n",cmd);
			break;
		case 0x03:
			ret = i2c_smbus_read_block_data(&dockram_client, cmd, ec_chip->i2c_fu_data);
			return_data_show_type = 0;
               		printk("i2c_smbus_read_block_data\n");
			break;
		case 0x04:
		        memset(&ec_chip->i2c_fu_data, 0, 32);
			ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_fu_data);
			return_data_show_type = 0;
               		printk("i2c_smbus_read_i2c_block_data\n");
			break;
		case 0x05:
			ret = i2c_smbus_write_word_data(&dockram_client, cmd, ec_chip->i2c_fu_data);
			printk("case5 i2c_smbus_write_word_data 0x%x\n",cmd);
			break;
		case 0x06:
			//tp_init
			memset(&ec_chip->i2c_tp_data, 0, 38);
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x00;
			ec_chip->i2c_tp_data[2] = 0x08;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x22, 3, ec_chip->i2c_tp_data);
			msleep(500);
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf5;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			ASUSPEC_NOTICE("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));
			msleep(500);
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe6;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe6;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe6;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe9;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0x00;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0x10;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0x03;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe6;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0x01;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe9;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf8;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0x00;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
			ASUSPEC_NOTICE("tp \n");
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xe9;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			ASUSPEC_NOTICE("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));
			msleep(500);
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(500);
		                if(etd->abs_dev){
					ASUSPEC_NOTICE("abs_dev here , return 0");
		                        return 0;
		                }
			msleep(500);
			ec_chip->i2c_tp_data[0] = 0x00;
			ec_chip->i2c_tp_data[1] = 0x05;
			ec_chip->i2c_tp_data[2] = 0x00;
			ec_chip->i2c_tp_data[3] = 0x22;
			ec_chip->i2c_tp_data[4] = 0xd4;
			ec_chip->i2c_tp_data[5] = 0xf4;
			i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
			ASUSPEC_NOTICE("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));
			msleep(150);
			ASUSPEC_NOTICE("tp enable\n");
			i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
			msleep(150);
			ec_chip->tp_enable = 1;
			enable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
					break;
		case 0x07:
			//push data only, you should add length in byte 0
			ret = i2c_smbus_write_i2c_block_data(&kb_client, cmd, data_cnt, ec_chip->i2c_kb_data);
               		printk("kb i2c write i2c\n");
			break;
		case 0x08:
			//push data only, you should add length in byte 0
			ret = i2c_smbus_write_i2c_block_data(&tp_client, cmd, data_cnt, ec_chip->i2c_tp_data);
               		printk("tp i2c write i2c\n");
			break;
		case 0x09:
		        memset(&ec_chip->i2c_kb_data, 0, 38);
			ret = i2c_smbus_read_i2c_block_data(&kb_client, cmd, 38, ec_chip->i2c_kb_data);
			return_data_show_type = 1;
               		printk("kb i2c_smbus_read_i2c_block_data\n");
			break;
		case 0x0a:
       			memset(&ec_chip->i2c_tp_data, 0, 38);
			ret = i2c_smbus_read_i2c_block_data(&tp_client, cmd, 38, ec_chip->i2c_tp_data);
			return_data_show_type = 2;
               		printk("tp i2c_smbus_read_i2c_block_data\n");
			break;
		case 0x10:
			memset(&ec_chip->i2c_fu_data, 0, 32);
		        ec_chip->i2c_fu_data[0] = 8;
		        ec_chip->i2c_fu_data[7] = ec_chip->i2c_fu_data[7] | 0x08;
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x0a, 9, ec_chip->i2c_fu_data);
               		printk("enter gauge ic fu stop polling\n");
			break;
		case 0x11:
			memset(&ec_chip->i2c_fu_data, 0, 32);
		        ec_chip->i2c_fu_data[0] = 8;
		        ec_chip->i2c_fu_data[3] = ec_chip->i2c_fu_data[3] | 0x08;
			ret_val = i2c_smbus_write_i2c_block_data(&dockram_client, 0x0a, 9, ec_chip->i2c_fu_data);
               		printk("leave gauge ic fu start polling\n");
			break;
		case 0x12:
               		printk("enable apwake irq\n");
			enable_irq(gpio_to_irq(asuspec_apwake_gpio));
			break;
		case 0x13:
               		printk("disable apwake irq\n");
			disable_irq_nosync(gpio_to_irq(asuspec_apwake_gpio));
			break;
		case 0x14:
			if(ec_chip->op_mode==0)
				ec_chip->op_mode =1;
			else if(ec_chip->op_mode==1)
				ec_chip->op_mode =0;
               		printk("+++++++++op mode = %d++++++++++\n",ec_chip->op_mode);
			break;
		case 0x15:
			ASUSPEC_NOTICE("dock Entering gauge fu flag\n");
			asuspec_start_gauge_firmware_update(0);
			break;
		case 0x16:
			ASUSPEC_NOTICE("dock leave gauge fu flag\n");
			asuspec_stop_gauge_firmware_update(0);
			break;
		default:
			ASUSPEC_ERR("i2c type not support: %2x\n", i2c_type);
			break;
	}
       if (ret < 0) {
               ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
       } else
       return count;
}

static ssize_t asuspec_return_data_show(struct device *class,struct device_attribute *attr,char *buf, size_t count)
{
       int i, cmd, ret = 0;
       char temp_buf[64];
	switch(return_data_show_type){
	case 0://fu data
               if (ec_chip->i2c_fu_data[0]> 32)
                       return sprintf(buf, "EC return data length error\n");
               for (i = 0; i <= 32 ; i++){//print 32 byte
                       sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_fu_data[i]);
                       strcat(buf, temp_buf);
               }
		ASUSPEC_NOTICE("case %d output fu_data buffer to console\n", return_data_show_type);
               return strlen(buf);
	break;
	case 1://kb data
               for (i = 0; i <= 37 ; i++){//print 38 byte
                       sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_kb_data[i]);
                       strcat(buf, temp_buf);
               }
		ASUSPEC_NOTICE("case %d output kb_data buffer to console\n", return_data_show_type);
               return strlen(buf);

	break;
	case 2://ps2 data
               for (i = 0; i <= 37 ; i++){//print 38 byte
                       sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_tp_data[i]);
                       strcat(buf, temp_buf);
               }
		ASUSPEC_NOTICE("case %d output tp_data buffer to console\n", return_data_show_type);
               return strlen(buf);
	break;
	}
       return 0;
}


static int BuffDataSize(void)
{
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out){
        return (in - out);
    } else {
        return ((EC_BUFF_LEN - out) + in);
    }
}

static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1)){
        ASUSPEC_ERR("Error: EC work-buf overflow \n");
        return;
    }

    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN){
        buff_in_ptr = 0;
    }
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0){
        c = (char) ec_to_host_buffer[buff_out_ptr];
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN){
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;

	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);

    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
		ASUSPEC_INFO("tmp_buf[%d] = 0x%x, total_buf = %d\n", i, tmp_buf[i], total_buf);
        count--;
        i++;
		f_counter = 0;
		total_buf++;
    }

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i;
    }

    return ret;
}

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;
    int i;

    if (h2ec_count > 0)
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN)
    {
        return -EINVAL; /* data size is too big */
    }

    err = copy_from_user(host_to_ec_buffer, buf, count);
//FIXME: for debugging
//printk("user bugger length : %d\n",count);
    if (err)
    {
        ASUSPEC_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    switch (fu_type){
    case UPDATE_PAD_BYTE_MODE:
        for (i = 0; i < count ; i++){
        i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
        }
        break;
    case UPDATE_PAD_BLOCK_MODE:
        for (i = 0; i < count ; i++){
        	ec_chip->i2c_fu_data[i] = host_to_ec_buffer[i];
        }
        i2c_smbus_write_block_data(&dockram_client, 0x41, count, ec_chip->i2c_fu_data);
        break;
    case UPDATE_DOCK_BYTE_MODE:
        for (i = 0; i < count ; i++){
        	ec_chip->i2c_fu_data[i+9] = host_to_ec_buffer[i];
        }
	ec_chip->i2c_fu_data[0] = count+8;
	ec_chip->i2c_fu_data[1] = 0x0e;
	ec_chip->i2c_fu_data[2] = 0x00;
	ec_chip->i2c_fu_data[3] = 0x36;
	ec_chip->i2c_fu_data[4] = 0x41;
	ec_chip->i2c_fu_data[5] = count;
        i2c_smbus_write_block_data(&dockram_client, 0x11, count+9, ec_chip->i2c_fu_data);
        break;
    case UPDATE_DOCK_BLOCK_MODE:
        for (i = 0; i < count ; i++){
        	ec_chip->i2c_fu_data[i+9] = host_to_ec_buffer[i];
        }
	ec_chip->i2c_fu_data[0] = count+9;
	ec_chip->i2c_fu_data[1] = 0x8a;//protocal
	ec_chip->i2c_fu_data[2] = 0x00;
	ec_chip->i2c_fu_data[3] = 0x36;
	ec_chip->i2c_fu_data[4] = 0x41;
	ec_chip->i2c_fu_data[5] = count;
        i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, count+9, ec_chip->i2c_fu_data);
        break;
    default:
        break;
    }
    h2ec_count = 0;
    return count;

}

int asusdec_is_ac_over_10v_callback(void){

	int ret_val, err;

	ASUSPEC_NOTICE("access dockram\n");
	if (ec_chip->dock_in && (ec_chip->dock_type == MOBILE_DOCK)){
		msleep(250);
		err = asuspec_dockram_read_data(0x23);
		ASUSPEC_NOTICE("byte[1] = 0x%x\n", ec_chip->i2c_dm_data[1]);
		if(err < 0)
			goto fail_to_access_ec;

		return ec_chip->i2c_dm_data[1] & 0x20;
	}

fail_to_access_ec:
	ASUSPEC_NOTICE("dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusdec_is_ac_over_10v_callback);

static int __init asuspec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####\n", __func__);

	if (asuspec_major) {
		asuspec_dev = MKDEV(asuspec_major, asuspec_minor);
		err_code = register_chrdev_region(asuspec_dev, 1, "asuspec");
	} else {
		err_code = alloc_chrdev_region(&asuspec_dev, asuspec_minor, 1,"asuspec");
		asuspec_major = MAJOR(asuspec_dev);
	}

	ASUSPEC_NOTICE("cdev_alloc\n") ;
	asuspec_cdev = cdev_alloc() ;
	asuspec_cdev->owner = THIS_MODULE ;
	asuspec_cdev->ops = &asuspec_fops ;

	err_code=i2c_add_driver(&asuspec_driver);
	if(err_code){
		ASUSPEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asuspec_class = class_create(THIS_MODULE, "asuspec");
	if(asuspec_class <= 0){
		ASUSPEC_ERR("asuspec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asuspec_device = device_create(asuspec_class, NULL, MKDEV(asuspec_major, asuspec_minor), NULL, "asuspec" );
	if(asuspec_device <= 0){
		ASUSPEC_ERR("asuspec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	ASUSPEC_INFO("return value %d\n", err_code) ;
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;

device_create_fail :
	class_destroy(asuspec_class) ;
class_create_fail :
	i2c_del_driver(&asuspec_driver);
i2c_add_driver_fail :
	printk(KERN_INFO "%s- #####\n", __func__);
	return err_code;

}

static void __exit asuspec_exit(void)
{
	device_destroy(asuspec_class,MKDEV(asuspec_major, asuspec_minor)) ;
	class_destroy(asuspec_class) ;
	i2c_del_driver(&asuspec_driver);
	unregister_chrdev_region(asuspec_dev, 1);
	switch_dev_unregister(&ec_chip->pad_sdev);
	if(machine_is_haydn()){
		switch_dev_unregister(&ec_chip->apower_sdev);
	}
}

module_init(asuspec_init);
module_exit(asuspec_exit);
