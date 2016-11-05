#ifndef _ASUSPEC_H
#define _ASUSPEC_H
#include <linux/wakelock.h>
#include <linux/switch.h>

/*
 * compiler option
 */
#define ASUSPEC_DEBUG			0
#define TOUCHPAD_MODE                   1       // 0: relative mode, 1: absolute mode
#define TOUCHPAD_ELAN                   1       // 0: not elan, 1:elantech
#define BATTERY_DRIVER			1	// 0: battery call back function not enabled
#define FACTORY_MODE			0
#define EMC_NOTIFY			1	// 0: battery call back function not enabled

/*
 * Debug Utility
 */
#if ASUSPEC_DEBUG
#define ASUSPEC_INFO(format, arg...)	\
	printk(KERN_INFO "asuspec: [%s] " format , __FUNCTION__ , ## arg)
#define ASUSPEC_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							ASUSPEC_INFO("pad_ec_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define ASUSPEC_INFO(format, arg...)
#define ASUSPEC_I2C_DATA(array, i)
#endif

#define ASUSPEC_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "asuspec: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSPEC_ERR(format, arg...)	\
	printk(KERN_ERR "asuspec: [%s] " format , __FUNCTION__ , ## arg)

//-----------------------------------------

#define DRIVER_DESC     		"ASUS PAD EC driver"
#define PAD_SDEV_NAME			"pad"
#define DOCK_SDEV_NAME			"dock"
#define APOWER_SDEV_NAME		"apower"

#define TP_DELAY_TIME_MS             	50
#define CONVERSION_TIME_MS              50
#define DELAY_TIME_MS			50
#define ASUSPEC_RETRY_COUNT		3
#define ASUSPEC_I2C_ERR_TOLERANCE	32

#define ASUSDEC_RELATIVE_MODE           0
#define ASUSDEC_ABSOLUTE_MODE           1

#define NUM_RELATIVE_MODE               3       // The number of bytes per packet in relative mode
#define NUM_ABSOLUTE_MODE               6       // The number of bytes per packet in absolute mode

/* relative mode packet formate */
#define Y_OVERFLOW_MASK                 0x80
#define X_OVERFLOW_MASK                 0x40
#define Y_SIGN_MASK                     0x20
#define X_SIGN_MASK                     0x10
#define RIGHT_BTN_MASK                  0x2
#define LEFT_BTN_MASK                   0x1

/* absolute mode packet formate */
#define TOUCHPAD_SAMPLE_RATE            20
#define ABSOLUTE_BIT_MASK               0x80
#define Z_SNESITIVITY                   30
#define X_LIMIT                         5600
#define Y_LIMIT                         1300
#define X_MAX                           5855
#define X_MIN                           1198
#define Y_MAX                           4942
#define Y_MIN                           946

/*************scan 2 make mapping***************/
#define ASUSDEC_KEY_TOUCHPAD		KEY_F2
#define ASUSDEC_KEY_AUTOBRIGHT		KEY_F3
#define ASUSDEC_KEY_SETTING		KEY_F4

#define ASUSDEC_KEYPAD_ESC		0x29
#define ASUSDEC_KEYPAD_KEY_WAVE		0x35
#define ASUSDEC_KEYPAD_KEY_1		0x1e
#define ASUSDEC_KEYPAD_KEY_2		0X1f
#define ASUSDEC_KEYPAD_KEY_3		0x20
#define ASUSDEC_KEYPAD_KEY_4		0x21
#define ASUSDEC_KEYPAD_KEY_5		0x22
#define ASUSDEC_KEYPAD_KEY_6        	0x23
#define ASUSDEC_KEYPAD_KEY_7        	0x24
#define ASUSDEC_KEYPAD_KEY_8        	0x25
#define ASUSDEC_KEYPAD_KEY_9        	0x26
#define ASUSDEC_KEYPAD_KEY_0        	0x27
#define ASUSDEC_KEYPAD_KEY_MINUS    	0x2d
#define ASUSDEC_KEYPAD_KEY_EQUAL	0x2e
#define ASUSDEC_KEYPAD_KEY_BACKSPACE	0x2a
#define ASUSDEC_KEYPAD_KEY_TAB      	0x2b
#define ASUSDEC_KEYPAD_KEY_Q        	0x14
#define ASUSDEC_KEYPAD_KEY_W        	0x1a
#define ASUSDEC_KEYPAD_KEY_E        	0x08
#define ASUSDEC_KEYPAD_KEY_R        	0x15
#define ASUSDEC_KEYPAD_KEY_T        	0x17
#define ASUSDEC_KEYPAD_KEY_Y        	0x1c
#define ASUSDEC_KEYPAD_KEY_U        	0x18
#define ASUSDEC_KEYPAD_KEY_I        	0x0c
#define ASUSDEC_KEYPAD_KEY_O        	0x12
#define ASUSDEC_KEYPAD_KEY_P        	0x13
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE	0x2f
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE 	0x30
#define ASUSDEC_KEYPAD_KEY_BACKSLASH	0x31
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK 	0x39
#define ASUSDEC_KEYPAD_KEY_A        	0x04
#define ASUSDEC_KEYPAD_KEY_S        	0x16
#define ASUSDEC_KEYPAD_KEY_D        	0x07
#define ASUSDEC_KEYPAD_KEY_F        	0x09
#define ASUSDEC_KEYPAD_KEY_G        	0x0a
#define ASUSDEC_KEYPAD_KEY_H        	0x0b
#define ASUSDEC_KEYPAD_KEY_J        	0x0d
#define ASUSDEC_KEYPAD_KEY_K        	0x0e
#define ASUSDEC_KEYPAD_KEY_L        	0x0f
#define ASUSDEC_KEYPAD_KEY_SEMICOLON	0x33
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE	0x34
#define ASUSDEC_KEYPAD_KEY_ENTER    	0x28
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT 	0x02
#define ASUSDEC_KEYPAD_KEY_Z        	0x1d
#define ASUSDEC_KEYPAD_KEY_X        	0x1b
#define ASUSDEC_KEYPAD_KEY_C        	0x06
#define ASUSDEC_KEYPAD_KEY_V        	0x19
#define ASUSDEC_KEYPAD_KEY_B        	0x05
#define ASUSDEC_KEYPAD_KEY_N        	0x11
#define ASUSDEC_KEYPAD_KEY_M        	0x10
#define ASUSDEC_KEYPAD_KEY_COMMA    	0x36
#define ASUSDEC_KEYPAD_KEY_DOT   	0x37
#define ASUSDEC_KEYPAD_KEY_SLASH    	0x38
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT  	0x20

#define ASUSDEC_KEYPAD_KEY_LEFT   	0x50
#define ASUSDEC_KEYPAD_KEY_RIGHT   	0x4f
#define ASUSDEC_KEYPAD_KEY_UP		0x52
#define ASUSDEC_KEYPAD_KEY_DOWN		0x51

#define ASUSDEC_KEYPAD_RIGHTWIN		0x04 //search
#define ASUSDEC_KEYPAD_LEFTCTRL		0x01
#define ASUSDEC_KEYPAD_LEFTWIN		0x08 //home page
#define ASUSDEC_KEYPAD_KEY_SPACE	0x2c
#define ASUSDEC_KEYPAD_RIGHTALT		0x40
#define ASUSDEC_KEYPAD_WINAPP		0x65
#define ASUSDEC_KEYPAD_RIGHTCTRL	0x10
#define ASUSDEC_KEYPAD_HOME		0x4a
#define ASUSDEC_KEYPAD_PAGEUP		0x4b
#define ASUSDEC_KEYPAD_PAGEDOWN		0x4e
#define ASUSDEC_KEYPAD_END		0x4d
/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU		0x94
#define ASUSDEC_YEN			0x89
#define ASUSDEC_MUHENKAN		0x8b
#define ASUSDEC_HENKAN			0x8a
#define ASUSDEC_HIRAGANA_KATAKANA	0x88
#define ASUSDEC_RO			0x87
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2		0x64
/********************************/


#define ASUSDEC_KEYPAD_LOCK		0x4c

//#define ASUSDEC_KEYPAD_KEY_BREAK   	0x
//#define ASUSDEC_KEYPAD_KEY_EXTEND   	0xE0

/*************scan 2 make code mapping***************/
#define ASUSPEC_OBF_MASK			0x1
#define ASUSDEC_OBF_MASK			0x1
#define ASUSPEC_KEY_MASK			0x4
#define ASUSPEC_KBC_MASK			0x8
#define ASUSPEC_AUX_MASK			0x20
#define ASUSDEC_AUX_MASK			0x20
#define ASUSPEC_SCI_MASK			0x40
#define ASUSDEC_SCI_MASK			0x40
#define ASUSPEC_SMI_MASK			0x80
#define ASUSDEC_SMI_MASK			0x80
/************* control flag ********************/
#define ASUSPEC_DOCK_PRESENT			0x80
#define ASUSPEC_MOBILE_DOCK_PRESENT		0x04
/************* SMI event ********************/
#define ASUSPEC_SxI_LCD_Cover_Closed		0x2B
#define ASUSPEC_SxI_LCD_Cover_Opened		0x2C
#define ASUSPEC_SxI_AC_Event			0x31
#define ASUSPEC_SxI_Battery_CycleChange		0x34
#define ASUSPEC_SxI_Battery_FCCchange		0x35
#define ASUSPEC_SxI_Battery_Low			0x36
#define ASUSPEC_SxI_Battery_Updated		0x37
#define ASUSPEC_SxI_ECREQ_Received		0x50
#define ASUSPEC_SMI_HANDSHAKING			0x50
#define APOWER_SMI_S3				0x83
#define ASUSPEC_SxI_EC_WAKEUP			0x53
#define ASUSPEC_SMI_WAKE			0x53
#define ASUSPEC_SxI_BOOTBLOCK_RESET		0x5E
#define ASUSPEC_SxI_WATCHDOG_RESET		0x5F
#define ASUSPEC_SMI_RESET			0x5F
#define APOWER_SMI_NOTIFY_SHUTDOWN		0x90
#define APOWER_SMI_RESUME			0x91
#define APOWER_SMI_S5				0x85
#define ASUSPEC_SxI_DOCK_REMOVE			0x62
#define ASUSPEC_SxI_DOCK_INSERT			0x61
#define ASUSPEC_SxI_ADAPTER_CHANGE		0x60
#define ASUSPEC_SxI_PAD_BL_CHANGE		0x63
#define ASUSPEC_SxI_HID_Status_Changed		0x64
#define ASUSPEC_SxI_HID_WakeUp			0x65
#define ASUSPEC_SxI_I2CRouter_Busy		0x66
#define ASUSPEC_SxI_I2CRouter_Error		0x67
#define ASUSPEC_SxI_I2CRouter_WR_success	0x68
#define ASUSPEC_SxI_I2CRouter_RD_success	0x69
#define ASUSPEC_SxI_DOCK_FAIL			0x6B
#define ASUSPEC_SxI_DOCK_HID_INIT_FAIL		0x6C
#define ASUSPEC_SxI_AUDIO_DOCK_ID		0x70
#define ASUSDEC_SxI_AC_Event			0x71
#define ASUSDEC_SxI_PAD_BL_CHANGE		0xA3
#define ASUSDEC_SxI_HID_Status_Changed		0xA4
/*************APOWER switch state***************/
#define APOWER_IDLE			0
#define APOWER_RESUME			1
#define APOWER_SUSPEND			2
#define APOWER_POWEROFF			3
#define APOWER_NOTIFY_SHUTDOWN		4
/*************IO control setting***************/
#define ASUSDEC_TP_ON		1
#define ASUSDEC_TP_OFF		0
#define ASUSDEC_EC_ON		1
#define ASUSDEC_EC_OFF		0
#define ASUSPEC_IOCTL_HEAVY	2
#define ASUSPEC_IOCTL_NORMAL	1
#define ASUSPEC_IOCTL_END	0
#define ASUSPEC_IOC_MAGIC	0xf4
#define ASUSPEC_IOC_MAXNR	11
#define ASUSPEC_POLLING_DATA	_IOR(ASUSPEC_IOC_MAGIC,	1,	int)
#define ASUSPEC_FW_UPDATE 	_IOR(ASUSPEC_IOC_MAGIC,	2,	int)
#define ASUSPEC_INIT 		_IOR(ASUSPEC_IOC_MAGIC,	3,	int)
#define ASUSDEC_TP_CONTROL	_IOR(ASUSPEC_IOC_MAGIC, 5,      int)
#define ASUSDEC_EC_WAKEUP	_IOR(ASUSPEC_IOC_MAGIC, 6,      int)
#define ASUSPEC_FW_DUMMY	_IOR(ASUSPEC_IOC_MAGIC,	7,	int)
#define ASUSPEC_SWITCH_HDMI	_IOR(ASUSPEC_IOC_MAGIC,	10,	int)
#define ASUSPEC_WIN_SHUTDOWN	_IOR(ASUSPEC_IOC_MAGIC,	11,	int)
/*****************************************/
#define ASUSPEC_MAGIC_NUM	0x19850604

/************* EC FW update ***********/
#define EC_BUFF_LEN		256
#define ASUSPEC_GAUGE_IC_READ	0x00
#define ASUSPEC_GAUGE_IC_WRITE	0x01
#define ASUSPEC_GAUGE_IC_SLEEP	0x02
#define I2C_ROUTER_EBUSY	1
#define I2C_ROUTER_ERROR	2
#define I2C_ROUTER_WRITE_SUCCESS	68
#define I2C_ROUTER_READ_SUCCESS		69
/**********************************/

/************* Dock Defifition ***********/
#define DOCK_UNKNOWN		0
#define MOBILE_DOCK		1
#define AUDIO_DOCK		2
#define AUDIO_STAND		3

/************* Dock State ***********/
#define DOCK_OUT		0
#define DOCK_IN			1

/************* Cable Type ***********/
#define BAT_CABLE_OUT		0
#define BAT_CABLE_USB		1
#define BAT_CABLE_AC		3
#define BAT_CABLE_UNKNOWN	-1
#define CABLE_0V		0x00
#define CABLE_5V		0x05
#define CABLE_12V		0x12
#define CABLE_15V		0x15

#define MB 1024*1024

/*
 * data struct
 */

struct asusdec_touchpad_relative{
        int y_overflow;
        int x_overflow;
        int y_sign;
        int x_sign;
        int left_btn;
        int right_btn;
        int delta_x;
        int delta_y;
};

struct asusdec_touchpad_absolute{
        int w_val;
        int x_pos;
        int y_pos;
        int z_val;
        int left;
        int right;
        int x_prev;
        int y_prev;
        int z_prev;
        int x2_pos;
        int y2_pos;
        int z2_val;
};

struct asusdec_keypad{
        int value;
        int input_keycode;
        int extend;
};

struct asuspec_chip {
	struct input_dev	*indev;
	struct input_dev	*lid_indev;
	struct switch_dev 	pad_sdev;
	struct switch_dev	dock_sdev;
	struct switch_dev 	apower_sdev;
	struct i2c_client	*client;
	struct elantech_data	*private;
	struct i2c_client       *tp_client;
	struct mutex		lock;
	struct mutex		irq_lock;
	struct mutex		dock_init_lock;
	struct mutex		state_change_lock;
	struct delayed_work asuspec_fw_update_work;
	struct delayed_work asuspec_init_work;
	struct delayed_work asuspec_work;
	struct delayed_work asuspec_enter_s3_work;
	struct delayed_work asusdec_kb_report_work;
	struct delayed_work asusdec_tp_report_work;
	struct delayed_work asusdec_tp_enable_work;
	struct delayed_work asusdec_dock_init_work;
	struct delayed_work asuspec_hall_sensor_work;
	struct asusdec_keypad keypad_data;
	struct wake_lock 	wake_lock;
struct wake_lock        wake_lock_init;
	struct timer_list	asuspec_timer;
	int polling_rate;
	int status;
int dock_status;
#if TOUCHPAD_MODE
        struct asusdec_touchpad_absolute t_abs;
#else
        struct asusdec_touchpad_relative touchpad_data;
#endif
	int ret_val;
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_fu_data[32];
	u8 i2c_kb_data[38];
	u8 i2c_old_kb_data[38];
	u8 i2c_tp_data[38];
	u8 i2c_dock_dm_data[32];
	u8 i2c_dm_data[32];
	u8 i2c_dm_battery[32];
	u8 i2c_dm_dock_battery[32];
	u8 i2c_dm_storage[32];
	char ec_model_name[32];
	char ec_version[32];
	char ec_pcba[32];
	u16 ec_gauge_control_status;
	u16 ec_gauge_firmware_version;
	u16 ec_gauge_device_type;
	u16 ec_battery_id;
	u16 ec_df_version;
	int op_mode;	// 0: normal mode, 1: fw update mode
	int ec_ram_init;	// 0: not init, MAGIC_NUM: init successfully
	int ec_in_s3;	// 0: normal mode, 1: ec in deep sleep mode
	int i2c_err_count;
	int apwake_disabled;	// 0: normal mode, 1: apwake gets disabled
	int audio_recording;	// 0: not recording, 1: audio recording
	int d_index;    // touchpad byte counter
	unsigned long storage_total;
	unsigned long storage_avail;
	unsigned int pad_pid;
	int apower_state;
	char dec_model_name[32];
	char dec_version[32];
	char dec_pcba[32];
	int touchpad_member;
	int kb_and_ps2_enable;
	int dock_in;		// 0: no dock, 1: dock in
	int dock_det;		// dock-in interrupt count
	int dock_init;		// 0: dock not init, 1: dock init successfully
	int tp_wait_ack;	// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;		// 0 : touchpad has not enabled, 1: touchpad has enabled
	int ec_wakeup;		// 0 : dec shutdown when PAD in LP0, 1 : keep dec active when PAD in LP0,
	int suspend_state;	// 0: normal, 1: suspend
	int init_success; 	// 0: ps/2 not ready. 1: init OK, -1: tp not ready
	int dock_type; 		//0: unknown, 1: mobile_dock, 2: audio_dock, 3: audio_stand
	int dock_behavior;      // 0: susb_on follows wakeup event, 1: susb_on follows ec_req
	char dock_pid[32];
};

#endif
