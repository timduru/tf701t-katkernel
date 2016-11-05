/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/ad5823.h>
#include <linux/debugfs.h>
#include <media/ov5693.h>

#define AD5823_ACTUATOR_RANGE	1023
#define AD5823_POS_LOW_DEFAULT	(0)
#define AD5823_POS_HIGH_DEFAULT	(1023)
#define AD5823_FOCUS_MACRO	(568)
#define AD5823_FOCUS_INFINITY	(146)

#define SETTLETIME_MS	(12)
#define FOCAL_LENGTH	(4.507f)
#define FNUMBER		(2.4f)
#define	AD5823_MOVE_TIME_VALUE	(0x74)

#define AD5823_MAX_RETRIES (3)

#define FACTORY_MODE_AVAIL 0
#if FACTORY_MODE_AVAIL
extern unsigned int factory_mode;
#endif

/* ATD request: dbg to force set vcm position */
static int g_force_pos = -1;

/* Count of setting vcm position */
static int vcm_moving_count;

/* ASUS: Infinity position in OTP */
static u32 otp_inf_pos;

static struct ad5823_info *info;

struct ad5823_info {
	struct i2c_client *i2c_client;
	struct regulator *regulator;
	struct nv_focuser_config config;
	struct ad5823_platform_data *pdata;
	struct miscdevice miscdev;
	struct regmap *regmap;
};

static int ad5823_set_position(struct ad5823_info *info, u32 position)
{
	int ret = 0;

	if (position < info->config.pos_actual_low ||
		position > info->config.pos_actual_high) {
		dev_err(&info->i2c_client->dev,
			"%s: position(%d) out of bound([%d, %d])\n",
			__func__, position, info->config.pos_actual_low,
			info->config.pos_actual_high);
		if (position < info->config.pos_actual_low)
			position = info->config.pos_actual_low;
		if (position > info->config.pos_actual_high)
			position = info->config.pos_actual_high;
	}

	if (g_force_pos != -1) {
		/* Lock VCM position. */
		pr_info("%s(%d): force set pos to %d\n", __func__, __LINE__,
			g_force_pos);
		position = g_force_pos;
	}
	vcm_moving_count++;

	ret |= regmap_write(info->regmap, AD5823_MODE, 0x13);

	if (vcm_moving_count == 1) {
		/* Init ad5823 reg */
		u32 threshold_msb, threshold_lsb;
		threshold_msb = (info->config.pos_working_low >> 8) & 0x03;
		threshold_lsb = info->config.pos_working_low & 0xFF;

		ret |= regmap_write(info->regmap, AD5823_VCM_MOVE_TIME, 0x74);
		ret |= regmap_write(info->regmap,
			AD5823_VCM_THRESHOLD_MSB, threshold_msb);
		ret |= regmap_write(info->regmap,
			AD5823_VCM_THRESHOLD_LSB, threshold_lsb);
	}

	ret |= regmap_write(info->regmap, AD5823_VCM_CODE_MSB,
		((position >> 8) & 0x3) | (1 << 2));
	ret |= regmap_write(info->regmap, AD5823_VCM_CODE_LSB,
		position & 0xFF);

	return ret;
}

static long ad5823_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ad5823_info *info = file->private_data;
	struct ad5823_cal_data cal;

	switch (cmd) {
	case AD5823_IOCTL_GET_CONFIG:
	{
		u32 tmp;
		/* ASUS: otp[0:1] is inf posistion. otp[4:5] is macro posistion.
		 * DIT think a better AF range should be enlarged:
		 * {pos_low, pos_high} = {inf pos -120, macro pos + 120}
		 */
		tmp = otp[0];
		tmp = (tmp << 8) | otp[1];
		otp_inf_pos = tmp;

		if (tmp > 120)
			tmp -= 120;
		else
			tmp = 0;
		info->config.pos_working_low = tmp;

		tmp = otp[4];
		tmp = (tmp << 8) | otp[5];
		tmp += 120;
		if (tmp > 1023)
			tmp = 1023;
		info->config.pos_working_high = tmp;

		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config))) {
			dev_err(&info->i2c_client->dev, "%s: 0x%x\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		pr_info("%s(%d): AF range= {0x%X, 0x%X}\n", __func__, __LINE__,
			info->config.pos_working_low, info->config.pos_working_high);

		break;
	}
	case AD5823_IOCTL_SET_POSITION:
		return ad5823_set_position(info, (u32) arg);

	case AD5823_IOCTL_SET_CAL_DATA:
		if (copy_from_user(&cal, (const void __user *)arg,
					sizeof(struct ad5823_cal_data))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get mode from user.\n",
				__func__);
			return -EFAULT;
		}
		info->config.pos_working_low = cal.pos_low;
		info->config.pos_working_high = cal.pos_high;
		break;

	case AD5823_IOCTL_SET_CONFIG:
	{
		if (info->config.pos_working_low != 0)
			cal.pos_low = info->config.pos_working_low;
		if (info->config.pos_working_high != 0)
			cal.pos_high = info->config.pos_working_high;

		if (copy_from_user(&info->config, (const void __user *)arg,
			sizeof(struct nv_focuser_config))) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to get config from user.\n",
				__func__);
		return -EFAULT;
		}

		if (cal.pos_low != 0) {
			info->config.pos_working_low = cal.pos_low;

			/* ASUS:
			 * Use AF data in OTP for inf position instead. */
			info->config.focuser_set[0].inf = otp_inf_pos;
		}
		if (cal.pos_high != 0) {
			info->config.pos_working_high = cal.pos_high;
			info->config.focuser_set[0].macro = cal.pos_high;
		}
	}
	break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int ad5823_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct miscdevice *miscdev = file->private_data;
	struct ad5823_info *info = dev_get_drvdata(miscdev->parent);

	vcm_moving_count = 0;

	if (info->regulator)
		err = regulator_enable(info->regulator);

	if (info->pdata->power_on)
		err = info->pdata->power_on(info->pdata);

	file->private_data = info;
	msleep(100);
	return err;
}

int ad5823_release(struct inode *inode, struct file *file)
{
	struct ad5823_info *info = file->private_data;

	if (info->pdata->power_off)
		info->pdata->power_off(info->pdata);

	if (info->regulator)
		regulator_disable(info->regulator);
	file->private_data = NULL;

	return 0;
}


static const struct file_operations ad5823_fileops = {
	.owner = THIS_MODULE,
	.open = ad5823_open,
	.unlocked_ioctl = ad5823_ioctl,
	.release = ad5823_release,
};

static struct miscdevice ad5823_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "focuser",
	.fops = &ad5823_fileops,
};

static struct of_device_id ad5823_of_match[] = {
	{ .compatible = "nvidia,ad5823", },
	{ },
};

MODULE_DEVICE_TABLE(of, ad5823_of_match);

static int ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 1);

	return err;
}

static int ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);
	return 0;
}

static struct ad5823_platform_data *ad5823_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ad5823_platform_data *pdata;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	pdata->gpio = of_get_named_gpio_flags(np, "af-pwdn-gpios", 0, NULL);
	if (!gpio_is_valid(pdata->gpio)) {
		dev_err(&client->dev, "Invalid af-pwdn-gpios\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->power_on = ad5823_power_on;
	pdata->power_off = ad5823_power_off;

	return pdata;
}

/* ++++++++++++++++++++++++++++++++ dbgfs ++++++++++++++++++++++++++++++++ */
#define CATCH_BIT(a, b) ((a & (1 << b)) >> b)

static ssize_t dbg_ad5823_reg_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_ad5823_reg_dump_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int reg;

	int err = 0;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	for (reg = AD5823_RESET; reg <= AD5823_VCM_THRESHOLD_LSB; reg++) {
		unsigned int data;

		err = regmap_read(info->regmap, reg, &data);

		if (err) {
			len = snprintf(bp, dlen,
				"dumpreg [0x%x] err= %d. Stop.\n", reg, err);
			tot += len; bp += len; dlen -= len;
			break;
		} else {
			int bit = 0;
			len = snprintf(bp, dlen, "[0x%X]= b", reg);
			tot += len; bp += len; dlen -= len;

			for (bit = 7; bit >= 0; bit--) {
				len = snprintf(bp, dlen,
					"%d", CATCH_BIT(data, bit));
				tot += len; bp += len; dlen -= len;
			}
			len = snprintf(bp, dlen, " (0x%02X)\n", data);
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ad5823_reg_dump_fops = {
	.open		= dbg_ad5823_reg_dump_open,
	.read		= dbg_ad5823_reg_dump_read,
};

static ssize_t dbg_set_ad5823_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_ad5823_reg_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt, err;
	unsigned int ofst = 0, reg_val = 0;

	pr_info("%s: buf=%p, count=%d, ppos=%p\n", __func__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &ofst, &reg_val);

	err = regmap_write(info->regmap, ofst, reg_val);

	if (err)
		pr_info("%s(%d): fail. err=%d\n", __func__, __LINE__, err);

	return count;
}


static const struct file_operations dbg_set_ad5823_reg_fops = {
	.open		= dbg_set_ad5823_reg_open,
	.write		= dbg_set_ad5823_reg_write,
};

static ssize_t dbg_set_ad5823_pos_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_ad5823_pos_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt, err = 0 ;

	pr_info("%s: buf=%p, count=%d, ppos=%p\n", __func__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &g_force_pos);

	if (g_force_pos != -1) {
		err |= regmap_write(info->regmap, AD5823_MODE, 0x13);
		err |= regmap_write(info->regmap, AD5823_VCM_CODE_MSB,
			((g_force_pos >> 8) & 0x3) | (1 << 2));
		err |= regmap_write(info->regmap, AD5823_VCM_CODE_LSB,
			g_force_pos & 0xFF);
	}
	if (err)
		pr_info("%s(%d): fail. err=%d\n", __func__, __LINE__, err);

	return count;
}


static const struct file_operations dbg_set_ad5823_pos_fops = {
	.open		= dbg_set_ad5823_pos_open,
	.write		= dbg_set_ad5823_pos_write,
};

/* -------------------------------- dbgfs -------------------------------- */

static int ad5823_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct dentry *debugfs_dir;

	static struct regmap_config ad5823_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	dev_dbg(&client->dev, "ad5823: probing sensor.\n");

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "ad5823: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	ad5823_device.parent = &client->dev;
	err = misc_register(&ad5823_device);
	if (err) {
		dev_err(&client->dev, "ad5823: Unable to register misc device!\n");
		goto ERROR_RET;
	}

	if (client->dev.of_node) {
		info->pdata = ad5823_parse_dt(client);
		if (IS_ERR(info->pdata)) {
			err = PTR_ERR(info->pdata);
			dev_err(&client->dev,
				"Failed to parse OF node: %d\n", err);
			goto ERROR_RET;
		}
	} else if (client->dev.platform_data) {
		info->pdata = client->dev.platform_data;
	} else {
		info->pdata = NULL;
		dev_err(&client->dev, "ad5823: Platform data missing.");
		err = EINVAL;
		goto ERROR_RET;
	}
	err = gpio_request_one(info->pdata->gpio, GPIOF_OUT_INIT_LOW,
				"af_pwdn");
	if (err)
		goto ERROR_RET;

	info->regulator = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR_OR_NULL(info->regulator)) {
		dev_err(&client->dev, "unable to get regulator %s\n",
			dev_name(&client->dev));
		info->regulator = NULL;
	} else {
		regulator_enable(info->regulator);
	}

	info->regmap = devm_regmap_init_i2c(client, &ad5823_regmap_config);
	if (IS_ERR(info->regmap)) {
		err = PTR_ERR(info->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", err);
		goto ERROR_RET;
	}

	if (info->regulator)
		regulator_disable(info->regulator);

	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.max_aperture = FNUMBER;
	info->config.range_ends_reversed = 0;

	info->config.pos_working_low = AD5823_FOCUS_INFINITY;
	info->config.pos_working_high = AD5823_FOCUS_MACRO;
	info->config.pos_actual_low = AD5823_POS_LOW_DEFAULT;
	info->config.pos_actual_high = AD5823_POS_HIGH_DEFAULT;

	info->config.num_focuser_sets = 1;
	info->config.focuser_set[0].macro = AD5823_FOCUS_MACRO;
	info->config.focuser_set[0].hyper = AD5823_FOCUS_INFINITY;
	info->config.focuser_set[0].inf = AD5823_FOCUS_INFINITY;

#if FACTORY_MODE_AVAIL
	if (factory_mode == 2)
		info->config.focuser_set[0].settle_time = 60;
	else
		info->config.focuser_set[0].settle_time = SETTLETIME_MS;
#else
	info->config.focuser_set[0].settle_time = SETTLETIME_MS;
#endif
	info->miscdev			= ad5823_device;

	i2c_set_clientdata(client, info);

	debugfs_dir = debugfs_create_dir("ad5823", NULL);
	(void) debugfs_create_file("regdump", S_IRUGO,
		debugfs_dir, NULL, &dbg_ad5823_reg_dump_fops);
	(void) debugfs_create_file("i2c_write", S_IRUGO | S_IWUSR,
		debugfs_dir, NULL, &dbg_set_ad5823_reg_fops);
	(void) debugfs_create_file("pos", S_IRUGO | S_IWUSR,
		debugfs_dir, NULL, &dbg_set_ad5823_pos_fops);

	return 0;

ERROR_RET:
	if (info->regulator)
		regulator_disable(info->regulator);

	misc_deregister(&ad5823_device);
	return err;
}

static int ad5823_remove(struct i2c_client *client)
{
	struct ad5823_info *info;
	info = i2c_get_clientdata(client);

	gpio_free(info->pdata->gpio);
	misc_deregister(&ad5823_device);
	return 0;
}

static const struct i2c_device_id ad5823_id[] = {
	{ "ad5823", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ad5823_id);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = "ad5823",
		.owner = THIS_MODULE,
		.of_match_table = ad5823_of_match,
	},
	.probe = ad5823_probe,
	.remove = ad5823_remove,
	.id_table = ad5823_id,
};

module_i2c_driver(ad5823_i2c_driver);
MODULE_LICENSE("GPL v2");
