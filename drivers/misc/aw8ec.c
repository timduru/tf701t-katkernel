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
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#if 1
#include <linux/wakelock.h>
#endif
#include <../gpio-names.h>
#include "aw8ec.h"
#include <asm/mach-types.h>

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static void aw8ec_dock_status_report(void);
static void aw8ec_w8_status_report(void);
static void aw8ec_scalar_status_report(void);
static void aw8ec_dock_init_work_function(struct work_struct *dat);
static int __devinit aw8ec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit aw8ec_remove(struct i2c_client *client);
static ssize_t aw8ec_show_dock(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_w8_status(struct device *class,
             struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_scalar_status(struct device *class,
             struct device_attribute *attr,char *buf);
static int aw8ec_suspend(struct i2c_client *client, pm_message_t mesg);
static int aw8ec_resume(struct i2c_client *client);
static int aw8ec_open(struct inode *inode, struct file *flip);
static int aw8ec_release(struct inode *inode, struct file *flip);
static long aw8ec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static int aw8ec_input_device_create(struct i2c_client *client);
static ssize_t aw8ec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t aw8ec_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t aw8ec_aio_switch_name(struct switch_dev * sdev, char * buf);
static ssize_t aw8ec_aio_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t aw8ec_scalar_switch_name(struct switch_dev * sdev, char * buf);
static ssize_t aw8ec_scalar_switch_state(struct switch_dev *sdev, char *buf);

bool p1801isDockIn = 0;
EXPORT_SYMBOL(p1801isDockIn);


static unsigned int aw8ec_dock_in_gpio = TEGRA_GPIO_PO0;
static unsigned int aw8ec_splash_gpio = TEGRA_GPIO_PI3;
static unsigned int aw8ec_w8_gpio = TEGRA_GPIO_PJ0;
static unsigned int aw8ec_scalar_status_gpio = TEGRA_GPIO_PH3;

static struct class *aw8ec_class;
static struct device *aw8ec_device ;
static struct aw8ec_chip *ec_chip;

struct cdev *aw8ec_cdev ;
static dev_t aw8ec_dev ;
static int aw8ec_major = 0 ;
static int aw8ec_minor = 0 ;
static int w8_lock_state = 0;
static struct workqueue_struct *aw8ec_wq;

static const struct i2c_device_id aw8ec_id[] = {
	{"aw8ec", 0},
	{}
};
ODULE_DEVICE_TABLE(i2c, aw8ec_id);

struct file_operations aw8ec_fops = {
	.owner = THIS_MODULE,
.unlocked_ioctl = aw8ec_ioctl,
	.open = aw8ec_open,
	.release = aw8ec_release,
};

static struct i2c_driver aw8ec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "aw8ec",
		.owner = THIS_MODULE,
	},
	.probe	 = aw8ec_probe,
	.remove	 = __devexit_p(aw8ec_remove),
	.suspend = aw8ec_suspend,
.resume = aw8ec_resume,
	.id_table = aw8ec_id,
};

static DEVICE_ATTR(ec_dock, S_IWUSR | S_IRUGO, aw8ec_show_dock,NULL);
static DEVICE_ATTR(ec_w8, S_IWUSR | S_IRUGO, aw8ec_show_w8_status,NULL);  //new
static DEVICE_ATTR(ec_scalar_status, S_IWUSR | S_IRUGO, aw8ec_show_scalar_status,NULL); 

static struct attribute *aw8ec_smbus_attributes[] = {
	&dev_attr_ec_dock.attr,
	&dev_attr_ec_w8.attr,
	&dev_attr_ec_scalar_status.attr,
NULL
};

static const struct attribute_group aw8ec_smbus_group = {
	.attrs = aw8ec_smbus_attributes,
};

static irqreturn_t aw8ec_interrupt_handler(int irq, void *dev_id){

	if (irq == gpio_to_irq(aw8ec_dock_in_gpio)){
	ec_chip->dock_in = 0;
		ec_chip->dock_det++;
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);
	} else if (irq == gpio_to_irq(aw8ec_w8_gpio)){
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_w8_work, 0);
	}else if (irq == gpio_to_irq(aw8ec_scalar_status_gpio)){
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_scalar_status_work, 0);
}
	return IRQ_HANDLED;
}

static int aw8ec_irq_w8_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_w8_gpio;
	unsigned irq = gpio_to_irq(gpio);
	const char* label = "aw8ec_w8" ;
	aw8ec_INFO("aw8ec_irq_w8_in\n");
	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}
	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
	rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

	err_gpio_request_irq_fail :
	gpio_free(gpio);
	err_gpio_direction_input_failed:
	return rc;
}

static int aw8ec_irq_scalar_status(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_scalar_status_gpio;
	unsigned irq = gpio_to_irq(gpio);
	const char* label = "aw8ec_scalar_status" ;

	aw8ec_INFO("aw8ec_irq_scalar_statusn\n");
	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
	aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

	err_gpio_request_irq_fail :
	gpio_free(gpio);
	err_gpio_direction_input_failed:
	return rc;
}
static int aw8ec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_dock_in_gpio;
	unsigned irq = gpio_to_irq(aw8ec_dock_in_gpio);
	const char* label = "aw8ec_dock_in" ;

	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
	rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

	err_gpio_request_irq_fail :
	gpio_free(gpio);
	err_gpio_direction_input_failed:
	return rc;
}

static int aw8ec_irq_splash_request(struct i2c_client *client)
{
        int rc = 0 ;
        unsigned gpio = aw8ec_splash_gpio;
        unsigned irq = gpio_to_irq(gpio);
        const char* label = "aw8ec_splash" ;

        aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
        aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
        rc = gpio_request(gpio, label);
        if (rc) {
                aw8ec_ERR("gpio_request failed for input %d\n", gpio);
                goto err_exit;
        }

        rc = gpio_direction_output(gpio, 1) ;
        if (rc) {
                aw8ec_ERR("gpio_direction_output failed for input %d\n", gpio);
                goto err_exit;
        }
        aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        return 0 ;

	err_exit:
        return rc;
}

static void aw8ec_dock_status_report(void){
	aw8ec_INFO("dock_in = %d\n", ec_chip->dock_in);
	printk("aw8ec_dock_status_report sucessed\n");
	switch_set_state(&ec_chip->dock_sdev, gpio_get_value(aw8ec_dock_in_gpio) ? 0 : 13);
}

static void aw8ec_w8_status_report(void){
	aw8ec_INFO("w8_on = %d\n", ec_chip->w8_on);
	printk("aw8ec_w8_status_report sucessed\n");
	switch_set_state(&ec_chip->w8_sdev, gpio_get_value(aw8ec_w8_gpio) ? 0 : 1);
}

static void aw8ec_scalar_status_report(void){
	aw8ec_INFO("dispaly win8 = %d\n", ec_chip->scalar_status);
	switch_set_state(&ec_chip->scalar_status_sdev, gpio_get_value(aw8ec_scalar_status_gpio) ? 1 : 0);
}

static void aw8ec_dock_init_work_function(struct work_struct *dat)
{
	int gpio = aw8ec_dock_in_gpio;
	int irq = gpio_to_irq(gpio);
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;
	aw8ec_INFO("Dock-init function\n");
	wake_lock(&ec_chip->wake_lock_init);
	if (machine_is_haydn()){
		aw8ec_NOTICE("haydn dock-init\n");
		if (ec_chip->dock_det){
			gpio_state = gpio_get_value(gpio);
		for(i = 0; i < 40; i++){
				msleep(50);
				if (gpio_state == gpio_get_value(gpio)){
				d_counter++;
				} else {
					gpio_state = gpio_get_value(gpio);
					d_counter = 0;
			}
				if (d_counter > 4){
					break;
				}
			}
		ec_chip->dock_det--;
			}

		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			aw8ec_NOTICE("No dock detected\n");
			ec_chip->dock_in = 0;
			p1801isDockIn = 0;

			if (ec_chip->indev){
				input_unregister_device(ec_chip->indev);
				ec_chip->indev = NULL;
			}
			aw8ec_dock_status_report();
		} else {
			aw8ec_NOTICE("Dock-in detected\n");
                    aw8ec_dock_status_report();
		}
		mutex_unlock(&ec_chip->input_lock);
	}
	wake_unlock(&ec_chip->wake_lock_init);
}


static void aw8ec_w8_report_function(struct work_struct *dat)
{

        int gpio = aw8ec_w8_gpio;
	int irq = gpio_to_irq(gpio);

	aw8ec_INFO("w8-init function\n");
	wake_lock(&ec_chip->wake_lock_init);

		mutex_lock(&ec_chip->input_lock);
	if (gpio_get_value(gpio)){
			aw8ec_NOTICE("w8 off\n");
			ec_chip->w8_on = 0;
			aw8ec_w8_status_report();
			if(w8_lock_state){
				w8_lock_state=0;
				wake_unlock(&ec_chip->wake_lock_w8);
				printk(KERN_INFO "aw8ec wake_unlock\n");
			}


		} else {
			w8_lock_state=1;
			aw8ec_NOTICE("w8 on\n");
			aw8ec_w8_status_report();
			wake_lock(&ec_chip->wake_lock_w8);
		}
		mutex_unlock(&ec_chip->input_lock);
		wake_unlock(&ec_chip->wake_lock_init);

}

static void aw8ec_scalar_status_report_function(struct work_struct *dat)
{
	int gpio = aw8ec_scalar_status_gpio;
	int irq = gpio_to_irq(gpio);

	aw8ec_INFO("scalar-init function\n");
	wake_lock(&ec_chip->wake_lock_init);

		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			aw8ec_NOTICE("dispaly win8\n");
			ec_chip->scalar_status = 1;
			aw8ec_scalar_status_report();
		} else {
			aw8ec_NOTICE("display PAD on\n");
			aw8ec_scalar_status_report();
		}
		mutex_unlock(&ec_chip->input_lock);
		wake_unlock(&ec_chip->wake_lock_init);
}

static int aw8ec_input_device_create(struct i2c_client *client){
	int err = 0;

	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		aw8ec_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "aw8ec";
	ec_chip->indev->phys = "/dev/input/aw8ec";
	ec_chip->indev->dev.parent = &client->dev;

	err = input_register_device(ec_chip->indev);
	if (err) {
		aw8ec_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

	exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
	exit:
	return err;
}


static int __devinit aw8ec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	printk("aw8ec probe\n");
	err = sysfs_create_group(&client->dev.kobj, &aw8ec_smbus_group);
	if (err) {
		aw8ec_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof(struct aw8ec_chip), GFP_KERNEL);
	if (!ec_chip) {
		aw8ec_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, ec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &aw8ec_driver;
	ec_chip->client->flags = 1;


	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->input_lock);
	mutex_init(&ec_chip->dock_init_lock);

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "aw8ec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "aw8ec_wake_init");
	wake_lock_init(&ec_chip->wake_lock_w8, WAKE_LOCK_SUSPEND, "aw8ec_wake_w8");

	ec_chip->status = 0;
	ec_chip->dock_det = 0;
	ec_chip->dock_in = 0;
	ec_chip->w8_on= 0;  //ne
	ec_chip->scalar_status= 0;
	ec_chip->d_index = 0;
	ec_chip->suspend_state = 0;
	ec_chip->indev = NULL;

	cdev_add(aw8ec_cdev,aw8ec_dev,1) ;

	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = aw8ec_switch_name;
	ec_chip->dock_sdev.print_state = aw8ec_switch_state;
	ec_chip->w8_sdev.name = W8_SDEV_NAME;
	ec_chip->w8_sdev.print_name = aw8ec_aio_switch_name;
	ec_chip->w8_sdev.print_state = aw8ec_aio_switch_state;

	ec_chip->scalar_status_sdev.name = SCALAR_SDEV_NAME;
	ec_chip->scalar_status_sdev.print_name = aw8ec_scalar_switch_name;
	ec_chip->scalar_status_sdev.print_state = aw8ec_scalar_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		aw8ec_ERR("switch_dev_register for dock failed!\n");
		goto exit;
	}

	if(switch_dev_register(&ec_chip->w8_sdev) < 0){
		aw8ec_ERR("switch_w8_dec_register for dock failed!\n");
		goto exit;
	}

	if(switch_dev_register(&ec_chip->scalar_status_sdev) < 0){
		aw8ec_ERR("switch_scalar_status__register for haydn failed!\n");
		goto exit;
	}

	switch_set_state(&ec_chip->dock_sdev, 0);
	switch_set_state(&ec_chip->w8_sdev, 0);
	switch_set_state(&ec_chip->scalar_status_sdev, 0);

	aw8ec_wq = create_singlethread_workqueue("aw8ec_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_w8_work, aw8ec_w8_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_scalar_status_work, aw8ec_scalar_status_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_dock_init_work, aw8ec_dock_init_work_function);

	aw8ec_irq_dock_in(client);
	aw8ec_irq_splash_request(client);
	aw8ec_irq_w8_in(client);
	aw8ec_irq_scalar_status(client);

	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_w8_work, 0);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_scalar_status_work, 0);

	return 0;

	exit:
	return err;
}


static int __devexit aw8ec_remove(struct i2c_client *client)
{
	struct aw8ec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t aw8ec_show_dock(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "dock detect = %d\n", ec_chip->dock_in);
}


static ssize_t aw8ec_show_w8_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(aw8ec_w8_gpio));
}

static ssize_t aw8ec_show_scalar_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(aw8ec_scalar_status_gpio));
}

static int aw8ec_suspend(struct i2c_client *client, pm_message_t mesg){
	int ret_val;

	aw8ec_NOTICE("aw8ec_suspend+\n");
	flush_workqueue(aw8ec_wq);
	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	aw8ec_NOTICE("aw8ec_suspend-\n");
	return 0;
}

static int aw8ec_resume(struct i2c_client *client){

	printk("aw8ec_resume+\n");
	ec_chip->suspend_state = 0;
	wake_lock(&ec_chip->wake_lock_init);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);

	printk("aw8ec_resume-\n");
	return 0;
}

static ssize_t aw8ec_switch_name(struct switch_dev *sdev, char *buf)
{
	return ;
}

static ssize_t aw8ec_switch_state(struct switch_dev *sdev, char *buf)
{

	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_dock_in_gpio) ? "0" : "13"));
}

//new
static ssize_t aw8ec_aio_switch_name(struct switch_dev *sdev, char *buf)
{
	return ;
}


//new
static ssize_t aw8ec_aio_switch_state(struct switch_dev *sdev, char *buf)
{

	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_w8_gpio)? "0" : "1"));

}

static ssize_t aw8ec_scalar_switch_name(struct switch_dev *sdev, char *buf)
{
	return ;
}

static ssize_t aw8ec_scalar_switch_state(struct switch_dev *sdev, char *buf)
{
       //gpio low:display PAD, high: dispaly win8
	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_scalar_status_gpio)? "1" : "0"));

}

static int aw8ec_open(struct inode *inode, struct file *flip){
	aw8ec_NOTICE(" ");
	return 0;
}
static int aw8ec_release(struct inode *inode, struct file *flip){
	aw8ec_NOTICE(" ");
	return 0;
}

static long aw8ec_ioctl(struct file *flip,unsigned int cmd, unsigned long arg){
	int err = 1;
	int ret=0;
	printk(KERN_INFO "%d+ #####splahstop\n", gpio_get_value(aw8ec_splash_gpio));


	if (_IOC_TYPE(cmd) != aw8ec_IOC_MAGIC){
	 return -ENOTTY;
	}
	if (_IOC_NR(cmd) > aw8ec_IOC_MAXNR){
	return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ){
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE){
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
     }
	if (err){
        return -EFAULT;
	}
        printk(KERN_INFO "start switch");
	switch (cmd) {
	case aw8ec_DETECT_SPLASHTOP:
                    aw8ec_NOTICE("aw8ec_DETECT_SPLASHTOP\n");
                    if(arg==aw8ec_SPLASHTOP_ON){
                            gpio_set_value(aw8ec_splash_gpio, 0);
                            msleep(20);
                            ret=gpio_get_value(aw8ec_splash_gpio);
                            if(ret){
                            printk(KERN_INFO"splashtop on failed\n");
                            return -ENOTTY;
                            }
                            else {
                            printk(KERN_INFO"splashtop on sucessed\n");
                            return ret;
                            }
                    }
                     if(arg==aw8ec_SPLASHTOP_OFF){
                            gpio_set_value(aw8ec_splash_gpio, 1);
                            msleep(20);
                            ret=gpio_get_value(aw8ec_splash_gpio);
                            if(ret) {
                            printk(KERN_INFO"splashtop off sucessed\n");
                            return ret;
                            }
                            else {
                            printk(KERN_INFO"splashtop off failed\n");
                            return -ENOTTY;
                            }
                      }
        default:
            return -ENOTTY;
	}
	return 0;
}

static int __init aw8ec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####3333\n", __func__);
	if (aw8ec_major) {
		aw8ec_dev = MKDEV(aw8ec_major, aw8ec_minor);
		err_code = register_chrdev_region(aw8ec_dev, 1, "aw8ec");
	} else {
		err_code = alloc_chrdev_region(&aw8ec_dev, aw8ec_minor, 1,"aw8ec");
		aw8ec_major = MAJOR(aw8ec_dev);
	}

	aw8ec_NOTICE("cdev_alloc\n") ;
	aw8ec_cdev = cdev_alloc() ;
	aw8ec_cdev->owner = THIS_MODULE ;
	aw8ec_cdev->ops = &aw8ec_fops ;

	err_code=i2c_add_driver(&aw8ec_driver);
	if(err_code){
		aw8ec_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	aw8ec_class = class_create(THIS_MODULE, "aw8ec");
	if(aw8ec_class <= 0){
		aw8ec_ERR("aw8ec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	aw8ec_device = device_create(aw8ec_class, NULL, MKDEV(aw8ec_major, aw8ec_minor), NULL, "aw8ec" );
	if(aw8ec_device <= 0){
		aw8ec_ERR("aw8ec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	aw8ec_INFO("return value %d\n", err_code) ;
	return 0;

	device_create_fail :
	class_destroy(aw8ec_class) ;
	class_create_fail :
	i2c_del_driver(&aw8ec_driver);
	i2c_add_driver_fail :
	printk(KERN_INFO "%s- #####\n", __func__);
	return err_code;

}

static void __exit aw8ec_exit(void)
{
	device_destroy(aw8ec_class,MKDEV(aw8ec_major, aw8ec_minor)) ;
	class_destroy(aw8ec_class) ;
	i2c_del_driver(&aw8ec_driver);
	unregister_chrdev_region(aw8ec_dev, 1);
	switch_dev_unregister(&ec_chip->dock_sdev);
}

module_init(aw8ec_init);
module_exit(aw8ec_exit);

