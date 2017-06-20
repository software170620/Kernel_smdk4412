/*
 *  Light Sensor Driver
 *  Copyright (c) 2016 Uniquenet
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/kthread.h>

//#include <linux/hwmon-sysfs.h>

#include <linux/gpio.h>
#include <mach/gpio-kcppk.h>
#include <linux/mfd/kcppk/kcppk_common.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#define SI1133_MGS 0

static const char *SOAP_SENSOR_NAME = "kcppk_sensor";

#define DEVICE_ID 0x00
#define HARDWARE_ID 0x01
#define HW_REV_ID 0x02
#define RESPONSE1 0x10
#define RESPONSE0 0x11
#define IRQENABLE 0x0f
#define INPUT0    0x0A

#define COMMAND_REG 0x0b
#define RESET_CMD_CTR 0x00
#define SENSOR_START 0x13
#define SENSOR_FORCE_START 0x11
#define SENSOR_RESET 0x01

// CHAN LIST
#define PARA_CHAN_LIST 0x01
#define SEL_CH0        0x01

#define PARA_WRITE     0x80
#define PARA_READ      0x40

enum SI1133_Error {
    SYS_CREATE_ERROR,
	MEM_ALLOC_ERROR,
	DEV_CREATE_ERROR,
};


void si1133_not_found(int value)
{
    printk(KERN_INFO " Light Sensor not found (%x) !!! \n", value);		
	return;
}
void si1133_task_mem_alloc_error(enum SI1133_Error value)
{
	switch(value) {
    case SYS_CREATE_ERROR:
		printk(KERN_INFO "Sysf Create Error(%s)\n",SOAP_SENSOR_NAME);		
		break;
    case MEM_ALLOC_ERROR:
		printk(KERN_INFO "kthread_kcppk_sensor mem Allocation Error\n");		
		break;				  
    case DEV_CREATE_ERROR:
		printk(KERN_INFO " /dev/%s Device Creating Error !!!!! \n",SOAP_SENSOR_NAME);		
		break;
    default:
		break;
		
	}
	return;
}

struct SI1133_chipset {
	struct mutex lock;
	struct device *dev;
	struct i2c_client *client;
	int device_id;
	void (*return_error1) (int value);
	void (*return_error2) (enum SI1133_Error value);
};

struct si1133_reg_value {
	u8 reg;
	u8 value;
	u8 mask;
	u32 delay_ms;
};

struct sensor_value
{
	char msb_data[2];
	char lsb_data[2];
	char reserve[2];
};

struct sensor_value *rcv_sensor; 

struct SI1133_chipset *si1133;

struct task_struct *kcppk_sensor;
DECLARE_WAIT_QUEUE_HEAD(wait_sensor);
DECLARE_WAIT_QUEUE_HEAD(sensor_to_mainui); 

struct timer_list sensor_sensing_timer;
bool SENSOR_READY = false;
bool SOAP_SENSOR_START = false;
bool MainUI_READY = false;


void SOAP_SENSOR_ENABLE(bool on) {
	SOAP_SENSOR_START = on;
}

void sensor_sensing_timer_init(void);
void sensor_timer_func(unsigned long data) {
	if(SENSOR_READY && SOAP_SENSOR_START) {
	wake_up_interruptible(&wait_sensor);
	}
    sensor_sensing_timer_init();
}

void sensor_sensing_timer_init() {
	sensor_sensing_timer.expires = jiffies + msecs_to_jiffies(500);
	sensor_sensing_timer.data = 0;
	sensor_sensing_timer.function = &sensor_timer_func;
	add_timer(&sensor_sensing_timer);
}

void sensor_sensing_timer_remove(void){
    del_timer_sync(&sensor_sensing_timer);
}

static struct si1133_reg_value channel_table[] = {
/* channel 0 */	
	{0x02, 0x02, 0x00, 0}, // ADCCONFIG0
//	{0x02, 0x19, 0x00, 0}, // ADCCONFIG0    no good
//	{0x02, 0x0d, 0x00, 0}, // ADCCONFIG0    0x0000 ~ 0x 0029 ~ 0x800
//	{0x02, 0x0b, 0x00, 0}, // ADCCONFIG0	0x0000~ 0x 0015 ~ 0x02 a0
	{0x03, 0x80, 0x00, 0}, // ADCSENS0
	{0x04, 0x00, 0x00, 0}, // ADCPOST0
	{0x05, 0x00, 0x00, 0}, // MEASCONFIG0	
#if 0
/* channel 1 */
	{0x06, 0x00, 0x00, 0}, // ADCCONFIG1
	{0x07, 0x00, 0x00, 0}, // ADCSENS1
	{0x08, 0x00, 0x00, 0}, // ADCPOST1
	{0x09, 0x00, 0x00, 0}, // MEASCONFIG1	

/* channel 2 */
	{0x0a, 0x00, 0x00, 0}, // ADCCONFIG2
	{0x0b, 0x00, 0x00, 0}, // ADCSENS2
	{0x0c, 0x00, 0x00, 0}, // ADCPOST2
	{0x0d, 0x00, 0x00, 0}, // MEASCONFIG2	

/* channel 3 */
	{0x0e, 0x00, 0x00, 0}, // ADCCONFIG3
	{0x0f, 0x00, 0x00, 0}, // ADCSENS3
	{0x10, 0x00, 0x00, 0}, // ADCPOST3
	{0x11, 0x00, 0x00, 0}, // MEASCONFIG3	

/* channel 4 */
	{0x12, 0x00, 0x00, 0}, // ADCCONFIG4
	{0x13, 0x00, 0x00, 0}, // ADCSENS4
	{0x14, 0x00, 0x00, 0}, // ADCPOST4
	{0x15, 0x00, 0x00, 0}, // MEASCONFIG4	

/* channel 5 */
	{0x16, 0x00, 0x00, 0}, // ADCCONFIG5
	{0x17, 0x00, 0x00, 0}, // ADCSENS5
	{0x18, 0x00, 0x00, 0}, // ADCPOST5
	{0x19, 0x00, 0x00, 0}, // MEASCONFIG5	
#endif		
};

static struct si1133_reg_value si1133_status[] = {
	{0x00, 0x00, 0x00, 0}, // PART_ID
	{0x01, 0x00, 0x00, 0}, // HW_ID
	{0x02, 0x00, 0x00, 0}, // REV_ID

	{0x0a, 0x00, 0x00, 0}, // HOSTIN0
	{0x0b, 0x00, 0x00, 0}, // COMMAND

	{0x0f, 0x00, 0x00, 0}, // RESET
	{0x10, 0x00, 0x00, 0}, // RESPONSE1		
	{0x11, 0x00, 0x00, 0}, // RESPONSE0
	{0x12, 0x00, 0x00, 0}, // IRQ_STATUS
	{0x13, 0x00, 0x00, 0}, // HOSTOUT0
	{0x14, 0x00, 0x00, 0}, 
	{0x15, 0x00, 0x00, 0}, 
	{0x16, 0x00, 0x00, 0}, 
	{0x17, 0x00, 0x00, 0},
	{0x18, 0x00, 0x00, 0}, 
	{0x19, 0x00, 0x00, 0}, 
	{0x1a, 0x00, 0x00, 0}, 
	{0x1b, 0x00, 0x00, 0}, 
	{0x1c, 0x00, 0x00, 0},
	{0x1d, 0x00, 0x00, 0}, 
	{0x1e, 0x00, 0x00, 0}, 	
	{0x1f, 0x00, 0x00, 0}, 	
	{0x20, 0x00, 0x00, 0}, 
	{0x21, 0x00, 0x00, 0}, 
	{0x22, 0x00, 0x00, 0}, 
	{0x23, 0x00, 0x00, 0}, 		
	{0x24, 0x00, 0x00, 0}, 
	{0x25, 0x00, 0x00, 0}, 
	{0x26, 0x00, 0x00, 0}, 
	{0x27, 0x00, 0x00, 0},
	{0x28, 0x00, 0x00, 0}, 
	{0x29, 0x00, 0x00, 0}, 
	{0x2a, 0x00, 0x00, 0}, 
	{0x2b, 0x00, 0x00, 0}, 
	{0x2c, 0x00, 0x00, 0},	
};

static struct si1133_reg_value si1133_sensing_value[] = {
	{0x13, 0x00, 0x00, 0}, // HOSTOUT0
	{0x14, 0x00, 0x00, 0}, 
};

static inline int SI1133_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;
    
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_dbg(&client->dev, "read reg error: ret = %d\n", ret);
	}
	return ret;
}


static inline int SI1133_write_reg(struct i2c_client *client,
		u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_dbg(&client->dev, "write reg error: ret = %d", ret);
	}

	return ret;
}

void read_respons0(struct i2c_client *client) {
	if( 0x01 == ((SI1133_read_reg(client, RESPONSE0) >> 4) & 0x01)) {
		printk(KERN_INFO " si1133 respond0 has CMD_ERR !!! \n");
		SI1133_write_reg(client, COMMAND_REG, RESET_CMD_CTR);
		printk(KERN_INFO " si1133 CMD_ERR cleared !!! \n");		
		SI1133_read_reg(client, RESPONSE0);
	} else {

	}
}

void read_respons1(struct i2c_client *client) {
//	printk(KERN_INFO " si1133 respond1 : %x \n", SI1133_read_reg(client, RESPONSE1));
}

int init_light_sensor(void) {
    int i;
	mutex_lock(&si1133->lock);

    i = 0;
	while(ARRAY_SIZE(channel_table) > i) 
	{
    read_respons0(si1133->client);

	SI1133_write_reg(si1133->client, INPUT0, channel_table[i].value);
	
    SI1133_write_reg(si1133->client, COMMAND_REG, PARA_WRITE | channel_table[i].reg);

	read_respons1(si1133->client);

    read_respons0(si1133->client);
	i++;
	}

    read_respons0(si1133->client);

	SI1133_write_reg(si1133->client, INPUT0, SEL_CH0);
	
    SI1133_write_reg(si1133->client, COMMAND_REG, PARA_WRITE | PARA_CHAN_LIST);

    read_respons0(si1133->client);

	read_respons1(si1133->client);
	
	mutex_unlock(&si1133->lock);	
	return 0;
}

static int start_sensing_once(void) {

	mutex_lock(&si1133->lock);

    read_respons0(si1133->client);
	
    SI1133_write_reg(si1133->client, COMMAND_REG, SENSOR_FORCE_START);
	
    read_respons0(si1133->client);
	
    SI1133_write_reg(si1133->client, COMMAND_REG, SENSOR_START);
	
    read_respons0(si1133->client);
	
	mutex_unlock(&si1133->lock);
	
	return 0;
}

int get_current_sensing_value(char *MSB_VAL,char *LSB_VAL) {

	char buf[6];
	int  error;
    int  i;
	i = 0;

    while(ARRAY_SIZE(si1133_sensing_value) > i)
    {
		buf[0] = si1133_sensing_value[i].reg;

		error = i2c_master_send(si1133->client, buf, 1);
		if (error < 0)
		       return error;

		error = i2c_master_recv(si1133->client, buf, 1);
		if (error < 0)
			return error;
		if(i == 0) {
			*MSB_VAL = buf[0];
		}
		else {
			*LSB_VAL = buf[0];			
		}
		i++;
    }
	return 0;
}

static int kthread_kcppk_sensor(void *arg)
{
  init_light_sensor();
  while(!kthread_should_stop())
  {  	
  	SENSOR_READY = true;
   	interruptible_sleep_on(&wait_sensor); 		
	SENSOR_READY = false;
	start_sensing_once();	
	if(MainUI_READY) wake_up_interruptible(&sensor_to_mainui);
 }
  return 0;
} 

static int si1133_get_device_id(struct i2c_client *client, int *id)
{
	char buf[6];
	int  error;


	buf[0] = DEVICE_ID;

	error = i2c_master_send(client, buf, 1);
	if (error < 0)
	       return error;

	error = i2c_master_recv(client, buf, 1);
	if (error < 0)
		return error;

	/* Return the device ID */
	*id = buf[0];
	
	return 0; 
}

static int si1133_get_device_status(struct i2c_client *client)
{
	char buf[6];
	int  error;
    int  i;
	char  data[2];
	i = 0;
    while(ARRAY_SIZE(si1133_status) > i)
    {
		buf[0] = si1133_status[i].reg;

		error = i2c_master_send(client, buf, 1);
		if (error < 0)
		       return error;

		error = i2c_master_recv(client, buf, 1);
		if (error < 0)
			return error;

		printk(KERN_INFO " si1133_reg[%x] = %x \n",si1133_status[i].reg, buf[0]);
		i++;
    }
    get_current_sensing_value(&data[0],&data[1]); 	
	printk(KERN_INFO "========  data[0] : %x,  data[1]: %x  ======== \n",data[0],data[1]);		
	return 0; 
}

static ssize_t i1133_show_device_id(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct SI1133_chipset *si1133 = i2c_get_clientdata(client);

	mutex_lock(&si1133->lock);
    si1133_get_device_id(client, &si1133->device_id);

    printk(KERN_INFO " SI1133 HW ID : %x \n", SI1133_read_reg(client, HARDWARE_ID));
    printk(KERN_INFO " SI1133 Rev ID : %x \n", SI1133_read_reg(client, HW_REV_ID));	
	
	mutex_unlock(&si1133->lock);
	return sprintf(buf, "%x\n", si1133->device_id);
}

static ssize_t i1133_show_device_status(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct SI1133_chipset *si1133 = i2c_get_clientdata(client);

	mutex_lock(&si1133->lock);
	
    si1133_get_device_status(client);
	
	mutex_unlock(&si1133->lock);
	return sprintf(buf, "%x\n", 0x00);
}

static ssize_t i1133_show_device_start(struct device *dev,	struct device_attribute *attr, char *buf)
{

	start_sensing_once();
	
	return sprintf(buf, "%x\n", 0x00);
}

static ssize_t i1133_show_device_reset(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct SI1133_chipset *si1133 = i2c_get_clientdata(client);

	mutex_lock(&si1133->lock);
    read_respons0(client);
	
    SI1133_write_reg(client, COMMAND_REG, SENSOR_RESET);

    read_respons0(client);
	
	mutex_unlock(&si1133->lock);
	return sprintf(buf, "%x\n", 0x00);
}

static ssize_t i1133_show_device_respond(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct SI1133_chipset *si1133 = i2c_get_clientdata(client);

	mutex_lock(&si1133->lock);
    read_respons0(client);
	mutex_unlock(&si1133->lock);
	return sprintf(buf, "%x\n", 0x00);
}

static ssize_t i1133_show_device_irqenable(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct SI1133_chipset *si1133 = i2c_get_clientdata(client);

	mutex_lock(&si1133->lock);
    read_respons0(client);
	
    SI1133_write_reg(client, IRQENABLE, 0x0f);

    read_respons0(client);
	
	mutex_unlock(&si1133->lock);
	return sprintf(buf, "%x\n", 0x00);
}

static ssize_t i1133_show_device_parameter_ch0_set(struct device *dev,	struct device_attribute *attr, char *buf)
{

    init_light_sensor();
	
	return sprintf(buf, "%x\n", 0x00);
}

static DEVICE_ATTR(device_id, S_IRUGO, i1133_show_device_id,	NULL);
static DEVICE_ATTR(device_status, S_IRUGO, i1133_show_device_status,	NULL);
static DEVICE_ATTR(device_start, S_IRUGO, i1133_show_device_start,	NULL);
static DEVICE_ATTR(device_reset, S_IRUGO, i1133_show_device_reset,	NULL);
static DEVICE_ATTR(device_respond, S_IRUGO, i1133_show_device_respond,	NULL);
static DEVICE_ATTR(device_irqenable, S_IRUGO, i1133_show_device_irqenable,	NULL);
static DEVICE_ATTR(device_para_ch0_set, S_IRUGO, i1133_show_device_parameter_ch0_set,	NULL);


/* Attribute array */
static struct attribute *si1133_attributes[7];

static const struct attribute_group si1133_attr_group = {
	.attrs = si1133_attributes,
};




static int kcppk_sensor_open (struct inode *inode, struct file *filp)
{
	return 0;
}

static int kcppk_sensor_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t kcppk_sensor_write(struct file *filp, const char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	*f_pos = count;
	return count;
}

static ssize_t kcppk_sensor_read(struct file *filp, char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	int buff_size = 14;
	char data[2];
	u8 SEND_TO_UI[buff_size];
	char *default_Value = "00,00,00,00,00";
	if(*f_pos > 0) return 0; 
	memcpy(SEND_TO_UI,default_Value,buff_size);	
    MainUI_READY = true;
  	interruptible_sleep_on(&sensor_to_mainui);
	MainUI_READY = false;
    get_current_sensing_value(&data[0],&data[1]); 
	sprintf(&rcv_sensor->msb_data[0],"%02x", data[0]);
	sprintf(&rcv_sensor->lsb_data[0],"%02x", data[1]);	
	msleep(1);
	memcpy(&SEND_TO_UI[0],rcv_sensor->msb_data,2);
	memcpy(&SEND_TO_UI[3],rcv_sensor->lsb_data,2);
	msleep(1);	

	if(copy_to_user(buf,SEND_TO_UI,12) != 0) {		
	   printk(KERN_ERR "Error : copy to user !!\n");
	   return -EFAULT;
	} 	

	return 12;
}

struct file_operations kcppk_sensor_fops = {
   	.owner 	= THIS_MODULE,
	.open 	= kcppk_sensor_open,
	.write  = kcppk_sensor_write,
	.read   = kcppk_sensor_read,
	.release	= kcppk_sensor_release,
};

static struct cdev kcppk_sensor_char_cdev = {
	.kobj = {
			.name="kcppk_sensor",
		},
	.owner = THIS_MODULE,
	.ops = &kcppk_sensor_fops,
};

static struct miscdevice kcppk_sensor_misc_device = {
		.minor = 0,
		.name = "kcppk_sensor",
		.fops = &kcppk_sensor_fops,
		.mode = (S_IRUGO | S_IRWXUGO | S_IALLUGO),
};

int sensor_device_create(void) {
	   	dev_t dev;
		int err = 0;	
	    struct class *cl;
	    struct miscdevice *misd;	
		misd = &kcppk_sensor_misc_device;
	    dev = MKDEV(Kcppk_Char_Major, Kcppk_Char_Dev_4);
	    err = register_chrdev_region(dev, 1, SOAP_SENSOR_NAME);

	    cdev_init(&kcppk_sensor_char_cdev, &kcppk_sensor_fops);
	    kcppk_sensor_char_cdev.owner = THIS_MODULE;
	    kcppk_sensor_char_cdev.ops  = &kcppk_sensor_fops;

	    cl = class_create(THIS_MODULE, "chardrv_kcppk_sensor");
	    device_create(cl, NULL, dev, NULL, SOAP_SENSOR_NAME);

	    if(cdev_add(&kcppk_sensor_char_cdev, dev, 1)) {
	        printk(KERN_INFO "/dev/%s : cdev creation failed.\n",SOAP_SENSOR_NAME);
			goto err_create_1;
	    }
	    printk(KERN_INFO "/dev/%s: cdev creation Ok.\n",SOAP_SENSOR_NAME);	
		return 0;
 err_create_1:
 	    return -1;
}


static int SI1133_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{

	int error;
	
	si1133 = kzalloc(sizeof(*si1133), GFP_KERNEL);
	if (si1133 == NULL)
		return -ENOMEM;
	i2c_set_clientdata(client, si1133);

	si1133->client = client;

	mutex_init(&si1133->lock);
	mutex_lock(&si1133->lock);
	printk(KERN_INFO " SI1133 i2c addr : %x \n", client->addr);
    printk("Si1133: probe %s\n", dev_name(&client->dev) );	
	error = si1133_get_device_id(client, &si1133->device_id); //si1133_get_device_id	

    si1133->return_error1 = si1133_not_found;
	si1133->return_error2 = si1133_task_mem_alloc_error;

	mutex_unlock(&si1133->lock);

	if (error < 0)
	{
		(si1133->return_error1)(si1133->device_id);
		kfree(si1133);	
		return error;
	}	

	printk(KERN_INFO " Light Sensor : %x  found \n",si1133->device_id);	
	
	si1133_attributes[0] = &dev_attr_device_id.attr;	
	si1133_attributes[1] = &dev_attr_device_status.attr;	
	si1133_attributes[2] = &dev_attr_device_start.attr;
	si1133_attributes[3] = &dev_attr_device_reset.attr;
	si1133_attributes[4] = &dev_attr_device_respond.attr;	
	si1133_attributes[5] = &dev_attr_device_irqenable.attr;		
	si1133_attributes[6] = &dev_attr_device_para_ch0_set.attr;		
	/* Create the sysfs group */
   	printk("trying sysfs_create_group.. \n");		
	error = sysfs_create_group(&client->dev.kobj, &si1133_attr_group);
	
	if (error || si1133->device_id < 0) {
		(si1133->return_error2)(SYS_CREATE_ERROR);	
		goto error_2;
	}

	kcppk_sensor = (struct task_struct *)kthread_run(kthread_kcppk_sensor, NULL, "kthread_kcppk_sensor");
    if(kcppk_sensor == NULL) {		
		(si1133->return_error2)(MEM_ALLOC_ERROR);			
		goto error_2;
    } else
    {
		printk(KERN_INFO "Memory Allocation OK.. : %s \n",SOAP_SENSOR_NAME);	   	
    }

    rcv_sensor = kzalloc(sizeof(struct sensor_value), GFP_KERNEL);
    if(rcv_sensor == NULL){   	 
    	(si1133->return_error2)(MEM_ALLOC_ERROR);		
        kfree(kcppk_sensor);	
    	goto error_2;
   }
	
	init_timer(&sensor_sensing_timer); 	
	
	sensor_sensing_timer_init();  	

    if(sensor_device_create() < 0) {
		(si1133->return_error2)(DEV_CREATE_ERROR);	
        kfree(kcppk_sensor);
        kfree(rcv_sensor);		
		goto error_2;
    }

	return 0;

error_2:	
		kfree(si1133);		
		return -1;	
}


static int SI1133_i2c_remove(struct i2c_client *i2c_client)
{

	return 0;
}


static const struct i2c_device_id SI1133_i2c_id[] = {
	{"kcppk-light", 0},	
	{},
};
MODULE_DEVICE_TABLE(i2c, SI1133_i2c_id);

static struct i2c_driver SI1133_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "light",
		   },
	.probe = SI1133_i2c_probe,
	.remove = SI1133_i2c_remove,
	.id_table = SI1133_i2c_id,
};


static __init int SI1133_init(void)
{
	int err;

	err = i2c_add_driver(&SI1133_i2c_driver);

	if (err < 0)
		pr_err("%s: driver registration failed, error=%d\n",
			__func__, err);
    printk(KERN_INFO " ---------------- %s init ------------ \n",__func__);
	return err;
}

static void __exit SI1133_clean(void)
{
	i2c_del_driver(&SI1133_i2c_driver);
	sensor_sensing_timer_remove();
}

module_init(SI1133_init);
module_exit(SI1133_clean);

MODULE_AUTHOR(RABBIT "rabbit-girl@uniquenet.co.kr>");
MODULE_DESCRIPTION("MY Third virtual Driver");
