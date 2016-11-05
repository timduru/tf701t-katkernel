#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/freezer.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/slab.h>

#include <../gpio-names.h>
#include "elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define TP_RET_FA_OFFSET 2

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

const u8 etp_hid_descriptor_cmd[] = {0x20, 0x00};
const u8 etp_hid_wake_up_cmd[] = {0x22, 0x00, 0x00, 0x08};
const u8 etp_hid_reset_cmd[] = {0x22, 0x00, 0x00, 0x01};
const u8 etp_hid_sleep_cmd[] = {0x22, 0x00, 0x01, 0x08};
const u8 etp_hid_report_descriptor_cmd[] = {0x21, 0x00};
const u8 etp_hid_read_data_cmd[] = {0x24, 0x00};
const u8 asus_kb_hid_read_data_cmd[] = {0x73, 0x00};
const u8 asus_kb_wake_up_cmd[] = {0x75, 0x00, 0x00, 0x08};
const u8 asus_kb_reset_cmd[] = {0x75, 0x00, 0x00, 0x01};
const u8 asus_kb_sleep_cmd[] = {0x75, 0x00, 0x01, 0x08};
static unsigned int asuspec_ps2_int_gpio = TEGRA_GPIO_PW2;
static unsigned int asuspec_dock_in_gpio = TEGRA_GPIO_PO0;

int elantech_i2c_command(struct i2c_client *client, int  command,
				unsigned char *buf_recv, int data_len)
{
	const u8 *cmd = NULL;
	unsigned char *rec_buf = buf_recv;
	int ret;
	int tries = 3;
	int length = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;

	switch (command) {
	case ETP_HID_DESCR_CMD:
		cmd = etp_hid_descriptor_cmd;
		length = sizeof(etp_hid_descriptor_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	case ETP_HID_WAKE_UP_CMD:
		cmd = etp_hid_wake_up_cmd;
		length = sizeof(etp_hid_wake_up_cmd);
		msg_num = 1;
		break;
	case ETP_HID_RESET_CMD:
		cmd = etp_hid_reset_cmd;
		length = sizeof(etp_hid_reset_cmd);
		msg_num = 1;
		break;
	case ETP_HID_REPORT_DESCR_CMD:
		cmd = etp_hid_report_descriptor_cmd;
		length = sizeof(etp_hid_report_descriptor_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	case ETP_HID_READ_DATA_CMD:
		cmd = etp_hid_read_data_cmd;
		length = sizeof(etp_hid_read_data_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	case ASUS_KB_HID_READ_DATA_CMD:
		cmd = asus_kb_hid_read_data_cmd;
		length = sizeof(asus_kb_hid_read_data_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	case ETP_HID_SLEEP_CMD:
		cmd = etp_hid_sleep_cmd;
		length = sizeof(etp_hid_sleep_cmd);
		msg_num = 1;
		break;
	case ASUS_KB_WAKE_UP_CMD:
		cmd = asus_kb_wake_up_cmd;
		length = sizeof(asus_kb_wake_up_cmd);
		msg_num = 1;
		break;
	case ASUS_KB_SLEEP_CMD:
		cmd = asus_kb_sleep_cmd;
		length = sizeof(asus_kb_sleep_cmd);
		msg_num = 1;
		break;
	case ASUS_KB_RESET_CMD:
		cmd = asus_kb_reset_cmd;
		length = sizeof(asus_kb_reset_cmd);
		msg_num = 1;
		break;
	default:
		ELAN_ERR("command=%d unknow.\n", command);
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = rec_buf;

	do {
		ret = i2c_transfer(client->adapter, msg, msg_num);
		if (ret != msg_num)
			ELAN_ERR("wrong length ret: %d, msg_num: %d\n");
		if (ret > 0)
			break;
		tries--;
		ELAN_ERR("retrying elantech_i2c_command:%d (%d)\n", command, tries);
		msleep(10);
	} while (tries > 0);

	return ret;
}

static int elan_i2c_asus_cmd(struct i2c_client *client,unsigned char *param, int command)
{

	u16 asus_ec_cmd;
	int ret = 0;
	int retry = ELAN_RETRY_COUNT;
	int i;
	int retry_data_count;
	int retry_ps2_int = 50;
	u8 i2c_data[16] = {0x00,0x05,0x00,0x22,0xd4};
	int index;

	i2c_data[5] = command;
	ret = i2c_smbus_write_i2c_block_data(client, 0x25, 7, i2c_data);

	if (ret < 0) {
		ELAN_ERR("Wirte to device fails status %x\n",ret);
		return ret;
	}

	while(gpio_get_value(asuspec_ps2_int_gpio)){
		msleep(CONVERSION_TIME_MS);
		if(retry_ps2_int-- < 0){
			ELAN_ERR("timeout! Fail to init touchpad!\n");
			return -1;
		}
	}

	while(retry-- > 0){
		ret = elantech_i2c_command(client, ETP_HID_READ_DATA_CMD, i2c_data, 10);
		if (ret < 0) {
			ELAN_ERR("Fail to read data, status %d\n", ret);
			return ret;
		}
		ASUSPEC_I2C_DATA(i2c_data, index);
		if (i2c_data[2 + TP_RET_FA_OFFSET] == PSMOUSE_RET_ACK){
			break;
		}
		else if (i2c_data[2 + TP_RET_FA_OFFSET] == PSMOUSE_RET_NAK){
			goto fail_elan_touchpad_i2c;
		}
		msleep(CONVERSION_TIME_MS/5);
	}

	retry_data_count = (command & 0x0f00) >> 8;
	for(i=1; i <= retry_data_count; i++){
		param[i-1] = i2c_data[i+2+TP_RET_FA_OFFSET];
	}

	return 0;

fail_elan_touchpad_i2c:
	ELAN_ERR("fail to get touchpad response");
	return -1;

}

/*
 * Interpret complete data packets and report absolute mode input events for
 * hardware version 2. (6 byte packets)
 */
void elantech_report_absolute_to_related(struct asuspec_chip *ec_chip, int *Null_data_times)
{
	struct elantech_data *etd;
	struct input_dev *dev;
	unsigned char *packet;
	unsigned int fingers;
	unsigned int width = 0;
	int left_button, right_button;
	int x, y;
	int last_fingers;
	int i;

	etd = (struct elantech_data *) ec_chip->private;
	dev = etd->abs_dev;
 	packet = ec_chip->ec_data;

	if(!etd->abs_dev){
		printk("asuspec: etd->abs_dev is null ! return\n");
		return;
	}

	// Report multitouch events for fingers.
	fingers = (packet[0] & 0xc0) >> 6;
	x = ((packet[1] & 0x0f) << 8) | packet[2];
	y = etd->ymax - (((packet[4] & 0x0f) << 8) | packet[5]);
	//printk("fingers=%d, x=%d, y=%d, packet=%02x,%02x,%02x,%02x,%02x,%02x\n", fingers, x, y, packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
        width = ((packet[0] & 0x30) >> 2) | ((packet[3] & 0x30) >> 4);

	last_fingers = etd->fingers;

	switch (fingers) {
		case 0:
			// No fingers down.
			etd->fingers = 0;
			break;

		case 1:
			// One finger down.
			etd->fingers = 1;
			etd->pos[0].x = x;
			etd->pos[0].y = y;
			break;

		case 2:
			// Two fingers down.
			// Wait to get data from both fingers.
			if (etd->fingers != 2) {
				etd->fingers = 2;
				etd->pos[0].x = -1;
				etd->pos[1].x = -1;
			}
			if ((packet[0] & 0x0c) == 0x04) {
				etd->pos[0].x = x;
				etd->pos[0].y = y;
			} else {
				etd->pos[1].x = x;
				etd->pos[1].y = y;
			}
			if (etd->pos[0].x < 0 || etd->pos[1].x < 0)
				return;
			break;

		case 3:
			// Three or more fingers down.
			// Wait for at least one finger to go up.
			return;
	}

	// Send finger reports.
	if (etd->fingers) {
		for (i = 0; i < etd->fingers; i++) {
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, width);
			input_report_abs(dev, ABS_MT_POSITION_X, etd->pos[i].x);
			input_report_abs(dev, ABS_MT_POSITION_Y, etd->pos[i].y);
			input_mt_sync(dev);
		}
	} else if (last_fingers) {
		input_mt_sync(dev);
	}

	// Send button press / release events.
	left_button = (packet[0] & 0x01);
	if (left_button != etd->left_button) {
		input_report_key(dev, BTN_LEFT, left_button);
		etd->left_button = left_button;
	}

	right_button = (packet[0] & 0x02) >> 1;
	if (right_button != etd->right_button) {
		input_report_key(dev, BTN_RIGHT, right_button);
		etd->right_button = right_button;
	}

	input_sync(dev);
}

/*
 * Put the touchpad into absolute mode
 */

static int elantech_set_absolute_mode(struct asuspec_chip *ec_chip, struct i2c_client *tp_client)
{

	struct i2c_client *client;
	unsigned char reg_10 = 0x03;

	ELAN_INFO("elantech_set_absolute_mode 2\n");
	client = tp_client;

	if ((!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_REGISTER_RW)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, 0x0010)) &&
	    (!elan_i2c_asus_cmd(client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(client, NULL, reg_10)) &&
	    (!elan_i2c_asus_cmd(client, NULL, PSMOUSE_CMD_SETSCALE11))) {

		return 0;
	}
	return -1;
}


/*
 * Set the appropriate event bits for the input subsystem
 */
static int elantech_set_input_rel_params(struct asuspec_chip *ec_chip, struct i2c_client *tp_client)
{
	struct elantech_data *etd = ec_chip->private;
	unsigned char param[3];
	int ret;

        if ((!elan_i2c_asus_cmd(tp_client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
            (!elan_i2c_asus_cmd(tp_client, NULL, 0x0001)) &&
            (!elan_i2c_asus_cmd(tp_client, param, PSMOUSE_CMD_GETINFO))){
                etd->fw_version = (param[0] << 16) | (param[1] << 8) | param[2];
        }
        else
                goto init_fail;

	if ((!elan_i2c_asus_cmd(tp_client, NULL, ETP_PS2_CUSTOM_COMMAND)) &&
	    (!elan_i2c_asus_cmd(tp_client, NULL, 0x0000)) &&
	    (!elan_i2c_asus_cmd(tp_client, param, PSMOUSE_CMD_GETINFO))){

		if(etd->abs_dev){
			return 0;
		}

		etd->xmax = (0x0F & param[0]) << 8 | param[1];
		etd->ymax = (0xF0 & param[0]) << 4 | param[2];

		etd->abs_dev = input_allocate_device();
		ELAN_INFO("1 elantech_touchscreen=%p\n",etd->abs_dev);
		if (etd->abs_dev != NULL){
			ELAN_INFO("2 elantech_touchscreen=%p\n",etd->abs_dev);
			etd->abs_dev->name = "elantech_touchscreen";
			etd->abs_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN);
			etd->abs_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);

			set_bit(EV_KEY, etd->abs_dev->evbit);
			set_bit(EV_ABS, etd->abs_dev->evbit);

			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_X, 0, etd->xmax, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_POSITION_Y, 0, etd->ymax, 0, 0);
			input_set_abs_params(etd->abs_dev, ABS_MT_TOUCH_MAJOR, 0, ETP_WMAX_V2, 0, 0);

			ret=input_register_device(etd->abs_dev);
			if (ret) {
			      ELAN_ERR("Unable to register %s input device\n", etd->abs_dev->name);
			}
		}
		return 0;
	}

 init_fail:
	return -1;

}


/*
 * Use magic knock to detect Elantech touchpad
 */
int elantech_detect(struct asuspec_chip *ec_chip,struct i2c_client *tp_client)
{
	struct i2c_client *client;
	unsigned char param[3];
	ELAN_INFO("2.6.2X-Elan-touchpad-2010-11-27\n");

	//client = ec_chip->client;
	client = tp_client;

	if (elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_DISABLE) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client,  NULL, PSMOUSE_CMD_SETSCALE11) ||
	    elan_i2c_asus_cmd(client, param, PSMOUSE_CMD_GETINFO)) {
		ELAN_ERR("sending Elantech magic knock failed.\n");
		return -1;
	}

	/*
	 * Report this in case there are Elantech models that use a different
	 * set of magic numbers
	 */
	if (param[0] != 0x3c ||param[1] != 0x03 || param[2]!= 0x00) {
		ELAN_ERR("unexpected magic knock result 0x%02x, 0x%02x, 0x%02x.\n",
			param[0], param[1],param[2]);
		return -1;
	}

	return 0;
}

/*
 * Initialize the touchpad and create sysfs entries
 */
int elantech_init(struct asuspec_chip *ec_chip, struct i2c_client *tp_client)
{
	ELAN_INFO("Elan et1059 elantech_init\n");

	if (elantech_set_absolute_mode(ec_chip, tp_client)){
		ELAN_ERR("failed to put touchpad into absolute mode.\n");
		return -1;
	}
	if (elantech_set_input_rel_params(ec_chip, tp_client)){
		ELAN_ERR("failed to elantech_set_input_rel_params.\n");
		return -1;
	}
	return 0;
}
