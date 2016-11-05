#ifndef ___YUV_SENSOR_H__
#define ___YUV_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/* ---------------------------------Important---------------------------------
 * for changing the SENSOR_NAME, you must need to change the owner of the
 * device. For example
 * Please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * ---------------------------------Important---------------------------------
*/

/* special number to indicate this is wait time require */
#define SENSOR_WAIT_MS		0
/* special number to indicate this is end of table */
#define SENSOR_TABLE_END	1
#define SENSOR_MAX_RETRIES	3 /* max counter for retry I2C access */
#define SENSOR_IOCTL_SET_MODE		   _IOW('o', 1, struct sensor_mode)
#define SENSOR_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u16)
#define SENSOR_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define SENSOR_IOCTL_SET_SCENE_MODE	 _IOW('o', 5, __u8)
enum {
	ASUS_CUSTOM_IOCTL_NUMBASE = 40,
	ASUS_CUSTOM_IOCTL_AF_SET,
	ASUS_CUSTOM_IOCTL_GET_ID,
	ASUS_CUSTOM_IOCTL_SET_EV,
	ASUS_CUSTOM_IOCTL_GET_EV,
	ASUS_CUSTOM_IOCTL_AF_GET,
	ASUS_CUSTOM_IOCTL_GET_ET,
	ASUS_CUSTOM_IOCTL_FW_UPDATE_MODE,
	ASUS_CUSTOM_IOCTL_SET_TOUCH_AF,
	ASUS_CUSTOM_IOCTL_SET_FLASH_STATUS,
	ASUS_CUSTOM_IOCTL_GET_FLASH_STATUS,
	ASUS_CUSTOM_IOCTL_GET_ISO,
	ASUS_CUSTOM_IOCTL_SET_SCENEMODE,
	ASUS_CUSTOM_IOCTL_SET_ISO,
	ASUS_CUSTOM_IOCTL_SET_FRAME_RATE,
	ASUS_CUSTOM_IOCTL_SET_FLICKERING,
	ASUS_CUSTOM_IOCTL_SET_SHADING,
	ASUS_CUSTOM_IOCTL_SET_FACEDETECT,
	ASUS_CUSTOM_IOCTL_GET_FACEDETECT,
	ASUS_CUSTOM_IOCTL_SET_CONTINUOUS_AF,
	ASUS_CUSTOM_IOCTL_SET_AE_WINDOW,
	ASUS_CUSTOM_IOCTL_SET_WDR,
	ASUS_CUSTOM_IOCTL_SET_AE_LOCK,
	ASUS_CUSTOM_IOCTL_SET_AWB_LOCK,
	ASUS_CUSTOM_IOCTL_GET_AE_LOCK,
	ASUS_CUSTOM_IOCTL_GET_AWB_LOCK,
	ASUS_CUSTOM_IOCTL_INITIAL,
	ASUS_CUSTOM_IOCTL_SET_AF_CONTROL,
	ASUS_CUSTOM_IOCTL_SET_TOUCH_AE,
	ASUS_CUSTOM_IOCTL_SET_ICATCH_AE_WINDOW,
	ASUS_CUSTOM_IOCTL_SET_AURA,
	ASUS_CUSTOM_IOCTL_FETCH_EXIF,
	ASUS_CUSTOM_IOCTL_GET_CAF_STATE,
	ASUS_CUSTOM_IOCTL_GET_VIRTUAL_GAIN,
};
#define SENSOR_CUSTOM_IOCTL_SET_EV	_IOW('o', ASUS_CUSTOM_IOCTL_SET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_GET_EV	_IOR('o', ASUS_CUSTOM_IOCTL_GET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_AE_LOCK	_IOW('o', ASUS_CUSTOM_IOCTL_SET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_SET_AWB_LOCK	_IOW('o', ASUS_CUSTOM_IOCTL_SET_AWB_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AE_LOCK	_IOWR('o', ASUS_CUSTOM_IOCTL_GET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AWB_LOCK	_IOWR('o', ASUS_CUSTOM_IOCTL_GET_AWB_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_VIRTUAL_GAIN	_IOWR('o', ASUS_CUSTOM_IOCTL_GET_VIRTUAL_GAIN, __u32)
enum {
	YUV_ColorEffect_Invalid = 0xA000,
	YUV_ColorEffect_Aqua,
	YUV_ColorEffect_Blackboard,
	YUV_ColorEffect_Mono,
	YUV_ColorEffect_Negative,
	YUV_ColorEffect_None,
	YUV_ColorEffect_Posterize,
	YUV_ColorEffect_Sepia,
	YUV_ColorEffect_Solarize,
	YUV_ColorEffect_Whiteboard,
	YUV_ColorEffect_Vivid,
	YUV_ColorEffect_WaterColor,
	YUV_ColorEffect_Vintage,
	YUV_ColorEffect_Vintage2,
	YUV_ColorEffect_Lomo,
	YUV_ColorEffect_Red,
	YUV_ColorEffect_Blue,
	YUV_ColorEffect_Yellow,
	YUV_ColorEffect_Aura,
	YUV_ColorEffect_Max
};
enum {
	YUV_Whitebalance_Invalid = 0,
	YUV_Whitebalance_Auto,
	YUV_Whitebalance_Incandescent,
	YUV_Whitebalance_Fluorescent,
	YUV_Whitebalance_WarmFluorescent,
	YUV_Whitebalance_Daylight,
	YUV_Whitebalance_CloudyDaylight,
	YUV_Whitebalance_Shade,
	YUV_Whitebalance_Twilight,
	YUV_Whitebalance_Custom
};

enum {
	YUV_SceneMode_Invalid = 0,
	YUV_SceneMode_Auto,
	YUV_SceneMode_Action,
	YUV_SceneMode_Portrait,
	YUV_SceneMode_Landscape,
	YUV_SceneMode_Beach,
	YUV_SceneMode_Candlelight,
	YUV_SceneMode_Fireworks,
	YUV_SceneMode_Night,
	YUV_SceneMode_NightPortrait,
	YUV_SceneMode_Party,
	YUV_SceneMode_Snow,
	YUV_SceneMode_Sports,
	YUV_SceneMode_SteadyPhoto,
	YUV_SceneMode_Sunset,
	YUV_SceneMode_Theatre,
	YUV_SceneMode_Barcode,
	YUV_SceneMode_BackLight
};

struct sensor_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__
struct yuv_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_H__ */

