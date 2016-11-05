#ifndef _aw8ec_H
#define _aw8ec_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define aw8ec_DEBUG			0
/*
 * Debug Utility
 */
#if aw8ec_DEBUG
#define aw8ec_INFO(format, arg...)	\
	printk(KERN_INFO "aw8ec: [%s] " format , __FUNCTION__ , ## arg)
#define aw8ec_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							aw8ec_INFO("ec_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define aw8ec_INFO(format, arg...)
#define aw8ec_I2C_DATA(array, i)
#endif

#define aw8ec_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "aw8ec: [%s] " format , __FUNCTION__ , ## arg)

#define aw8ec_ERR(format, arg...)	\
	printk(KERN_ERR "aw8ec: [%s] " format , __FUNCTION__ , ## arg)

//----------------------------------------

#define DRIVER_DESC     		"ASUS Dock EC Driver"
#define DOCK_SDEV_NAME			"dock"
#define W8_SDEV_NAME			"aio"
#define SCALAR_SDEV_NAME		"scalar_status"
#define CONVERSION_TIME_MS		50

#define aw8ec_I2C_ERR_TOLERANCE	8
#define aw8ec_RETRY_COUNT		3
#define aw8ec_POLLING_RATE		80

/*************IO control setting***************/
#define aw8ec_SPLASHTOP_ON	1  //new
#define aw8ec_SPLASHTOP_OFF	0  //new
#define aw8ec_IOC_MAGIC	0xf4
#define aw8ec_IOC_MAXNR	8
#define aw8ec_DETECT_SPLASHTOP  _IOR(aw8ec_IOC_MAGIC, 8,	int)
/*************IO control setting***************/


/*
 * data struct
 */
struct aw8ec_chip {
	struct input_dev	*indev;
	struct input_dev	*lid_indev;
	struct switch_dev 	dock_sdev;
	struct switch_dev  w8_sdev;  //new
	struct switch_dev  scalar_status_sdev;
	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex		input_lock;
	struct mutex		dock_init_lock;
	struct wake_lock 	wake_lock;
	struct wake_lock 	wake_lock_init;
	struct wake_lock	wake_lock_w8;
	struct delayed_work aw8ec_dock_init_work;
	struct delayed_work aw8ec_w8_work;
	struct delayed_work aw8ec_scalar_status_work;

	int ret_val;
	int status;
	int polling_rate;
	int dock_in;	// 0: without dock, 1: with dock
	//new
	int w8_on;    // 0:w8 off, 1:w8 on
	int scalar_status; //0:dispaly pad, 1: dispaly win8
	int dock_det;	// dock-in interrupt count
	int d_index;	// touchpad byte counter
	int suspend_state; // 0: normal, 1: suspend
	int dock_behavior;	// 0: susb_on follows wakeup event, 1: susb_on follows ec_req
};

#endif
