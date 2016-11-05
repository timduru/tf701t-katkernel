#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm_backlight.h>
#include <linux/edp.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <mach/dc.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include "../gpio-names.h"

#define ENABLE_DEBUG_I2C 0
#if ENABLE_DEBUG_I2C
#define printk_DEBUG printk
#else
#define printk_DEBUG(format, args...) ((void)0)
#endif

static int i2c_client_count = 0;
static struct i2c_client        *scalar_bl_client;
extern atomic_t sd_brightness;
extern int scalar_update_status; // 2: proceeding, -1: fail

#define MAX_LOOPS_RETRIES         20
u8 scalar_fw_version;
u8 scalar_fw_subversion;
u8 panel_type;

#define BL_CHANGED_AFTER_GET_BRIGHTNESS 1
#if BL_CHANGED_AFTER_GET_BRIGHTNESS
static int bl_change_disabled = 1;
#endif

#define scalar_status    TEGRA_GPIO_PH3  // 0:Pad, 1:win8

struct pwm_bl_data {
	struct pwm_device       *pwm;
	struct device           *dev;
	unsigned int            period;
	unsigned int            lth_brightness;
	unsigned int            pwm_gpio;
	struct edp_client *tegra_pwm_bl_edp_client;
	int *edp_brightness_states;
	int                     (*notify)(struct device *,
			int brightness);
	void                    (*notify_after)(struct device *,
			int brightness);
	int                     (*check_fb)(struct device *, struct fb_info *); 
	int (*display_init)(struct device *); 
};

static tegra_dc_bl_output haydn_bl_output_measured = {
	0, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 78, 79,
	80, 81, 82, 83, 84, 85, 86, 87,
	88, 89, 90, 91, 92, 93, 94, 95,
	96, 97, 98, 99, 100, 101, 102, 103,
	104, 105, 106, 107, 108, 109, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126, 127,
	128, 129, 130, 131, 132, 133, 134, 135,
	136, 137, 138, 139, 140, 141, 142, 143,
	144, 145, 146, 147, 148, 149, 150, 151,
	152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183,
	184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199,
	200, 201, 202, 203, 204, 205, 206, 207,
	208, 209, 210, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static int read_scalar_info(struct i2c_client *client)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[16];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = data;

	data[0] = 0x51;        //Source address
	data[1] = 0x82;        //length
	data[2] = 0x01;        //Get VCP Feature command
	data[3] = 0xC9;        //VCP opcode, brightness=10, scalar_info=C9
	data[4] = 0x6e^data[0]^data[1]^data[2]^data[3];        //Check sum
	printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4}={%x,%x,%x,%x,%x}\n",
			__func__, msg[0].addr, data[0], data[1], data[2], data[3], data[4]);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;        //read data, from slave to master

	msg[1].len = 11;        //include check sum byte
	msg[1].buf = data + 5;

	printk_DEBUG("%s: msg[0] transfer start\n", __func__);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err != 1)
		return -EINVAL;

	//from ddc/ci spec, need to wait 40ms in order to enable the decoding 
	// and preparation of the reply message in Monitor
	msleep(40);

	printk_DEBUG("%s: msg[1] transfer start\n", __func__);
	err = i2c_transfer(client->adapter, msg+1, 1);
	if (err != 1)
		return -EINVAL;

	printk_DEBUG("%s, check Source address=%x, length=%x,"
			" VCP feature reply opcode=%x\n"
			, __func__,data[5], data[6], data[7]);
	printk_DEBUG("%s, check Result code=%x, VCP code=%x,"
			" VCP type code=%x\n"
			, __func__,data[8], data[9], data[10]);
	printk_DEBUG("%s, check brightness max high=%x, max low=%x\n"
			, __func__,data[11], data[12]);
	printk_DEBUG("%s, check brightness high=%x, low=%x\n"
			, __func__,data[13], data[14]);
	printk_DEBUG("%s, check check sum=%x\n"
			, __func__,data[15]);

	//invalid length or invalid data
	if(data[6]!=0x88 || data[11]!=0xFF || data[12]!= 0xFF)
		return -EINVAL;

	scalar_fw_version = data[13];
	scalar_fw_subversion = (data[14] & 0xF0) >> 4;
	panel_type = data[14] & 0x0F;
	printk("%s: scalar_fw_version=%d, scalar_fw_subversion=%d,"
			"panel_type=%d\n", __func__, scalar_fw_version,
			scalar_fw_subversion, panel_type);

	return 0;
}

static void init_scalar_backlight_i2c(void) 
{
	int bus = 3;   // the same with hdmi edid bus
	int error;
	int retry;
	struct i2c_board_info  *info;
	struct i2c_adapter             *adapter;

	printk("scalar_bl_i2c:create a new adapter\n");
	info = kzalloc(sizeof(struct i2c_board_info), GFP_KERNEL);
	info->addr = 0x37;     // DDC/CI packets are transmitted using the I2C address 0x37
	strcpy(info->type, "scalar-backlight");
	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		printk("scalar_bl_i2c:can't get adpater for bus %d\n", bus);
		kfree(info);
	}

	scalar_bl_client = i2c_new_device(adapter, info);
	i2c_put_adapter(adapter);

	msleep(100);

	if (scalar_bl_client) {
		i2c_client_count++;

		error = read_scalar_info(scalar_bl_client);
		while (error != 0x00 && retry < MAX_LOOPS_RETRIES)
		{
			msleep(100);
			error = read_scalar_info(scalar_bl_client);
			retry += 1;
		}
	} else {
		printk(KERN_ERR "scalar_bl_i2c:can't create new device\n");
	}
	kfree(info);
}

static int i2c_write_backlight(struct i2c_client *client, u16 val_no_DIDIM, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[7];
	int retry = 0;

#if BL_CHANGED_AFTER_GET_BRIGHTNESS
	if (bl_change_disabled)
		return -EPERM;
#endif

	if (gpio_get_value(scalar_status))
		return -EPERM;

	if (scalar_update_status==2
	    || scalar_update_status==-1)
		return -EPERM;

	if (!client->adapter)
		return -ENODEV;

	data[0] = 0x51;        //Source address
	data[1] = 0x84;        //length: 0x80 OR n: Last 4 bits indicates the number of following bytes (excluding checksum)
	data[2] = 0x03;        //Set VCP command
	data[3] = 0x10;        //VCP opcode, brightness=10
	data[4] = (u8) (val_no_DIDIM & 0xff);        //High Byte
	data[5] = (u8) (val & 0xff);        //Low Byte
	//Check sum: Simple XOR of all preceding bytes (including the first)
	data[6] = 0x6e^data[0]^data[1]^data[2]^data[3]^data[4]^data[5];
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 7;
	msg.buf = data;
	printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4,5,6}={%x,%x,%x,%x,%x,%x,%x}, "
			"val_no_DIDIM=%d, val=%d\n", __func__, msg.addr,
			data[0], data[1], data[2], data[3], data[4], data[5], data[6],
			val_no_DIDIM, val);

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s : i2c transfer failed, set backlight=%d(%x), retry time %d\n"
				, __func__, val, val, retry);
	} while (retry <=3 /*DISPLAY_MAX_RETRIES*/);

	return err;
}

static int i2c_read_backlight(struct i2c_client *client, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[16];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = data;

	data[0] = 0x51;        //Source address
	data[1] = 0x82;        //length
	data[2] = 0x01;        //Get VCP Feature command
	data[3] = 0x10;        //VCP opcode, brightness=10
	data[4] = 0x6e^data[0]^data[1]^data[2]^data[3];        //Check sum
	printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4}={%x,%x,%x,%x,%x}\n"
			, __func__, msg[0].addr, data[0], data[1], data[2], data[3], data[4]);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;        //read data, from slave to master

	msg[1].len = 11;        //include check sum byte
	msg[1].buf = data + 5;

	printk_DEBUG("%s: msg[0] transfer start\n", __func__);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err != 1)
		return -EINVAL;

	//from ddc/ci spec, need to wait 40ms in order to enable
	//the decoding and preparation of the reply message in Monitor
	msleep(40);

	printk_DEBUG("%s: msg[1] transfer start\n", __func__);
	err = i2c_transfer(client->adapter, msg+1, 1);
	if (err != 1)
		return -EINVAL;


	printk_DEBUG("%s, check Source address=%x, length=%x, VCP feature reply opcode=%x\n"
			, __func__,data[5], data[6], data[7]);
	printk_DEBUG("%s, check Result code=%x, VCP code=%x, VCP type code=%x\n"
			, __func__ ,data[8], data[9], data[10]);
	printk_DEBUG("%s, check brightness max high=%x, max low=%x\n"
			, __func__,data[11], data[12]);
	printk_DEBUG("%s, check brightness high=%x, low=%x\n"
			, __func__,data[13], data[14]);
	printk_DEBUG("%s, check check sum=%x\n", __func__,data[15]);

	memcpy(val, data+13, 1);
	printk_DEBUG("%s, check brightness value low=%x, brightness=%d\n"
			, __func__,*val, *val);
	*val=*val&0xff;
	printk_DEBUG("%s, check brightness value low=%x, brightness=%d\n"
			, __func__,*val, *val);

	if(data[6]!=0x88)              //invalid length
		return -EINVAL;

	return 0;
}

static int brightness_no_sd = 100;
static int brightness_with_sd;

int brightness_write_disable = 0;

static int scalar_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	int approved;
	int edp_state;
	int i;
	int ret;
	int error = 0;

	printk_DEBUG("%s: brightness=%d\n", __func__, bl->props.brightness);

	if (pb->display_init && !pb->display_init(pb->dev))
		brightness = 0;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if(!brightness_write_disable) {
		brightness_no_sd = brightness;
	} else {
		//reset flag
		brightness_write_disable = 0;
	}

	int cur_sd_brightness = atomic_read(&sd_brightness);

	printk_DEBUG("%s: brightness=%d, cur_sd_brightness:%d\n"
			,__func__, bl->props.brightness, cur_sd_brightness);
	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	brightness_with_sd = brightness;

	if (pb->tegra_pwm_bl_edp_client) {
		for (i = 0; i < TEGRA_PWM_BL_EDP_NUM_STATES; i++) {
			if (brightness >= pb->edp_brightness_states[i])
				break;
		}
		edp_state = i;
		ret = edp_update_client_request(pb->tegra_pwm_bl_edp_client,
				edp_state, &approved);
		if (ret || approved != edp_state)
			dev_err(&bl->dev, "E state transition failed\n");
	}

	if (i2c_client_count==0) {
		init_scalar_backlight_i2c();
	}

	if (brightness == 0) {
#if !BL_CHANGED_AFTER_GET_BRIGHTNESS
		if (i2c_client_count !=0){
			printk_DEBUG("%s: call i2c_write_backlight, brightness=%d, "
					"duty=%d\n", __func__, brightness,
					brightness * 100 / max);
			error = i2c_write_backlight(scalar_bl_client,
					brightness_no_sd, brightness);
			printk_DEBUG("%s: write error? =%d\n", __func__, error);
		} else {
			printk(KERN_ERR "no scalar_bl_client\n");
		}
#endif
#if BL_CHANGED_AFTER_GET_BRIGHTNESS
		bl_change_disabled = 1;
		printk("bl_change_disabled:1\n");
#endif
	} else {
		//Need to map the brightness below 30% to 30% (77)
		/* Apply any backlight response curve */
		if (brightness > 255 || brightness_no_sd > 255) {
			printk("%s:brightness > 255\n", __func__);
			brightness = 255;
			brightness_no_sd = 255;
		}
		brightness = haydn_bl_output_measured[brightness];
		brightness_no_sd = haydn_bl_output_measured[brightness_no_sd];

		if (i2c_client_count !=0){
			printk_DEBUG("%s: call i2c_write_backlight, brightness=%d, "
					"duty=%d\n", __func__, brightness,
					(brightness*100+(max>>1)) / max);
			//the max value of hdmi display backlight is 255, need not convert
			error = i2c_write_backlight(scalar_bl_client,
					brightness_no_sd, brightness);
			printk_DEBUG("%s: write error? =%d\n", __func__, error);
		} else {
			printk(KERN_ERR "no scalar_bl_client\n");
		}
	}
	return 0;
}

static int scalar_backlight_get_brightness(struct backlight_device *bl)
{
	int brightness = 0;
	int error = 0;

	if (i2c_client_count !=0) {
		error = i2c_read_backlight(scalar_bl_client, &brightness);
		printk_DEBUG("%s: brightness=%d, read error? = %d\n", __func__, brightness, error);
		if(error)
			return error;
		else {
#if BL_CHANGED_AFTER_GET_BRIGHTNESS
			printk("bl_change_disabled:0\n");
			bl_change_disabled = 0;
#endif
			return brightness;
		}
	}
	return error; 
}

ssize_t haydn_pwm_backlight_get_brightness_with_sd(char *buf) {
	return sprintf(buf, "before sd: %d, after sd: %d\n"
			, brightness_no_sd, brightness_with_sd);
}

static int scalar_backlight_check_fb(struct backlight_device *bl,
		struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static void pwm_backlight_edpcb(unsigned int new_state, void *priv_data)
{
#if 0
        struct backlight_device *bl = (struct backlight_device *) priv_data;
        struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
        int max = bl->props.max_brightness;
        int brightness = pb->edp_brightness_states[new_state];

        if (brightness == 0) {
                pwm_config(pb->pwm, 0, pb->period);
                pwm_disable(pb->pwm);
        } else {
                brightness = pb->lth_brightness +
                        (brightness * (pb->period - pb->lth_brightness) / max);
                pwm_config(pb->pwm, brightness, pb->period);
                pwm_enable(pb->pwm);
        }

        if (pb->notify_after)
                pb->notify_after(pb->dev, brightness);
#endif
}

struct backlight_ops scalar_backlight_ops = {
	.update_status  = scalar_backlight_update_status,
	.get_brightness = scalar_backlight_get_brightness,
	.check_fb       = scalar_backlight_check_fb,
};

static int scalar_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	struct edp_manager *battery_manager = NULL;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->check_fb = data->check_fb;
	pb->dev = &pdev->dev;
	pb->display_init = data->init;
	pb->edp_brightness_states = data->edp_brightness;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;

	// hack backlight_device name to "pwm-backlight" because we use
	// this name in sysfs
	bl = backlight_device_register("pwm-backlight", &pdev->dev, pb,
			&scalar_backlight_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	pb->tegra_pwm_bl_edp_client = devm_kzalloc(&pdev->dev,
			sizeof(struct edp_client), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pb->tegra_pwm_bl_edp_client)) {
		dev_err(&pdev->dev, "could not allocate edp client\n");
		return PTR_ERR(pb->tegra_pwm_bl_edp_client);
	}
	strncpy(pb->tegra_pwm_bl_edp_client->name,
			"backlight", EDP_NAME_LEN - 1);
	pb->tegra_pwm_bl_edp_client->name[EDP_NAME_LEN - 1] = '\0';
	pb->tegra_pwm_bl_edp_client->states = data->edp_states;
	pb->tegra_pwm_bl_edp_client->num_states = TEGRA_PWM_BL_EDP_NUM_STATES;
	pb->tegra_pwm_bl_edp_client->e0_index = TEGRA_PWM_BL_EDP_ZERO;
	pb->tegra_pwm_bl_edp_client->private_data = bl;
	pb->tegra_pwm_bl_edp_client->priority = EDP_MAX_PRIO + 2;
	pb->tegra_pwm_bl_edp_client->throttle = pwm_backlight_edpcb;
	pb->tegra_pwm_bl_edp_client->notify_promotion = pwm_backlight_edpcb;

	battery_manager = edp_get_manager("battery");
	if (!battery_manager) {
		dev_err(&pdev->dev, "unable to get edp manager\n");
	} else {
		ret = edp_register_client(battery_manager,
					pb->tegra_pwm_bl_edp_client);
		if (ret) {
			dev_err(&pdev->dev, "unable to register edp client\n");
		} else {
			ret = edp_update_client_request(
					pb->tegra_pwm_bl_edp_client,
						TEGRA_PWM_BL_EDP_ZERO, NULL);
			if (ret) {
				dev_err(&pdev->dev,
					"unable to set E0 EDP state\n");
				edp_unregister_client(
					pb->tegra_pwm_bl_edp_client);
			} else {
				goto edp_success;
			}
		}
	}

	devm_kfree(&pdev->dev, pb->tegra_pwm_bl_edp_client);
	pb->tegra_pwm_bl_edp_client = NULL;

edp_success:

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int scalar_backlight_remove(struct platform_device *pdev)
{
        struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
        struct backlight_device *bl = platform_get_drvdata(pdev);
        struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

        backlight_device_unregister(bl);
        if (data->exit)
                data->exit(&pdev->dev);
        return 0;
}

#ifdef CONFIG_PM
static int scalar_backlight_suspend(struct device *dev)
{
        return 0;
}

static int scalar_backlight_resume(struct device *dev)
{
        struct backlight_device *bl = dev_get_drvdata(dev);

        backlight_update_status(bl);
        return 0;
}

static SIMPLE_DEV_PM_OPS(scalar_backlight_pm_ops, scalar_backlight_suspend,
                         scalar_backlight_resume);

#endif

static struct platform_driver scalar_backlight_driver = {
        .driver         = {
                .name   = "scalar-backlight",
                .owner  = THIS_MODULE,
#ifdef CONFIG_PM
                .pm     = &scalar_backlight_pm_ops,
#endif
        },  
        .probe          = scalar_backlight_probe,
        .remove         = scalar_backlight_remove,
};

module_platform_driver(scalar_backlight_driver);

MODULE_DESCRIPTION("Scalar I2C Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:scalar-backlight");
