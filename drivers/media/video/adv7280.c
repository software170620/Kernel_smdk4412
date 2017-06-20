/*
 * adv7280.c video decoder driver
 * Copyright (c) 2016 Uniquenet company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
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
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/gpio.h>
#include <mach/gpio-kcppk.h>

#include <plat/gpio-cfg.h>
#include <linux/mfd/kcppk/kcppk_common.h>

#define ADV7280_MSG 0

#define ADV7280_SAMPLE_SILICON         0x40
#define ADV7280_IDENT                  0x11
#define ADV7280_CSI_TX_ADDR            0x44
#define ADV7280_ADI_CONTROL1           0x0E
#define ADV7280_VPP_ADDR               0x42

#define CAM_CHECK_ERR_RET(x, msg)					\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
		return x;						\
	}
#define CAM_CHECK_ERR(x, msg)						\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
	}
#define CAM_CHECK_ERR_GOTO(x, out, fmt, ...) \
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR fmt, ##__VA_ARGS__); \
		goto out; \
	}

static int debug;
module_param(debug, int, 0);


struct reg_value {
	u8 reg;
	u8 value;
	u8 mask;
	u32 delay_ms;
};

struct adv7280_chipset {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *client_vpp;
	struct i2c_client *client_csi_tx;	
	struct adv7280_platform_data *pdata;
	struct v4l2_subdev sd;		
};

#define POWER_DWN_REMOVE 1
int ADV7280_power_on(void)
{
	
	struct regulator *regulator;
	int ret = 0;
    if(ADV7280_MSG) printk("===== %s ===== \n",__func__);
    BIT_STATUS(BIT_ADV7280_OV,BIT_OK);	
	printk(KERN_ERR "%s: in\n", __func__);
	
	ret = gpio_request(GPIO_CAM_nRST_L, "GPM4");
	//0921
	s3c_gpio_setpull(GPIO_CAM_nRST_L, S3C_GPIO_PULL_DOWN);	
	if (unlikely(ret)) {
		printk(KERN_ERR "request GPIO_CAM_nRST_L\n");
		return ret;
	}
#ifndef POWER_DWN_REMOVE
	ret = gpio_request(GPIO_CAM_POWER_DOWN_H, "GPM4");
	if (unlikely(ret)) {
		printk(KERN_ERR "request GPIO_CAM_POWER_DOWN_H\n");
		return ret;
	}
#endif

	/* CAM_IO_1V8 */
	regulator = regulator_get(NULL, "CAM_IO_1V8");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "CAM_IO_1V8");
	udelay(1000);
	
#ifndef POWER_DWN_REMOVE
	/* GPIO_CAM_PDN */
	ret = gpio_direction_output(GPIO_CAM_POWER_DOWN_H, 1);
	CAM_CHECK_ERR_RET(ret, "GPIO_CAM_PDM");
	udelay(10);
	if(ADV7280_MSG) printk(KERN_ERR "PDN : 1\n");
#endif
	/* GPIO_CAM_nRST_L */
	ret = gpio_direction_output(GPIO_CAM_nRST_L, 1);
	CAM_CHECK_ERR_RET(ret, "GPIO_CAM_nRST_L");
	mdelay(400);
	printk(KERN_ERR "high status GPIO_CAM_nRST_L : %d \n",gpio_get_value(GPIO_CAM_nRST_L));	
	if(ADV7280_MSG) printk(KERN_ERR "RST : 1\n");
	
	/* GPIO_CAM_PDN */
#ifndef POWER_DWN_REMOVE
	ret = gpio_direction_output(GPIO_CAM_POWER_DOWN_H, 0);
	CAM_CHECK_ERR_RET(ret, "GPIO_CAM_PDM");
	mdelay(100);
	if(ADV7280_MSG) printk(KERN_ERR "PDN : 0\n");
#endif

	/* GPIO_CAM_nRST_L */
	ret = gpio_direction_output(GPIO_CAM_nRST_L, 0);
	CAM_CHECK_ERR_RET(ret, "GPIO_CAM_nRST_L");
	mdelay(100);
	if(ADV7280_MSG) printk(KERN_ERR "RST : 0\n");
	
	printk(KERN_ERR "low status GPIO_CAM_nRST_L : %d \n",gpio_get_value(GPIO_CAM_nRST_L));
	
	gpio_free(GPIO_CAM_nRST_L);
#ifndef POWER_DWN_REMOVE	
	gpio_free(GPIO_CAM_POWER_DOWN_H);
#endif
	if(ADV7280_MSG) printk(KERN_ERR "%s:ret:%i\n",__func__, ret);
    return ret;
}

int ADV7280_power_off(void)
{
	struct regulator *regulator;
	int ret = 0;
	if(ADV7280_MSG) printk(KERN_ERR "%s: in\n", __func__);

	/* CAM_IO_1V8 */
	regulator = regulator_get(NULL, "CAM_IO_1V8");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "CAM_IO_1V8");
	udelay(100);

	return ret;
}

int ADV7280_power(int onoff)
{
	int ret = 0;
	if(ADV7280_MSG) printk(KERN_ERR "=== ADV7280_onoff value : %d \n",onoff);

	if (onoff) {
		ret = ADV7280_power_on();
	} else
		ret = ADV7280_power_off();

	if (unlikely(ret)) {
		pr_err("%s: power-on/down failed\n", __func__);
		return ret;
	}

	return ret;
}


static inline struct adv7280_chipset *to_adv7280(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7280_chipset, sd);
}

static int adv7280_reset(struct v4l2_subdev *sd, u32 val)
{
	return 0;
};

static int adv7280_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int adv7280_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	return 0;
}

static int adv7280_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
	
	int err = 0;
	if (fmt == NULL)
		return -EINVAL;

	return err;
}

static int adv7280_s_stream(struct v4l2_subdev *sd, int enable)
{
	if(enable)
	{
		printk(KERN_ERR "Stream On!!  \n");	
	}
	else
	{
		printk(KERN_ERR "Stream Off!!  \n");	
	}
	return 0;
}

static const struct v4l2_subdev_core_ops adv7280_core_ops = {
	.init = adv7280_reset,
	.s_power = adv7280_s_power,
};


static const struct v4l2_subdev_video_ops adv7280_video_ops = {
	.s_mbus_fmt = adv7280_s_fmt,
	.enum_framesizes = adv7280_enum_framesizes,
	.s_stream = adv7280_s_stream,
};

static const struct v4l2_subdev_ops adv7280_ops = {
	.core = &adv7280_core_ops,
	.video = &adv7280_video_ops,
};

static struct reg_value adv7280_init_params[] = {
	{0x0F, 0x00, 0x00, 0}, 
	{0x00, 0x00, 0x00, 0}, // default 00 > 08(YC in);
	{0x0e, 0x80, 0x00, 0}, 
	{0x9c, 0x00, 0x00, 0}, 
	{0x9c, 0xff, 0x00, 0}, 
	{0x0e, 0x00, 0x00, 0}, 		
	{0x03, 0x0c, 0x00, 0},   //fixed..
	{0x04, 0xb7, 0x00, 0},   // embedded sync ok bt.656 4
	{0x1d, 0x40, 0x00, 0}, 
	{0x52, 0xcd, 0x00, 0}, 
	{0x80, 0x51, 0x00, 0}, 
	{0x81, 0x51, 0x00, 0}, 
	{0x82, 0x68, 0x00, 0}, 
	{0x2c, 0xae, 0x00, 0}, // chroma & Luma control   default : 0xae, 0x87....  1010 1110
	{0x2d, 0x08, 0x00, 0}, // Chroma Gain   3:0 bit
	{0x2e, 0x00, 0x00, 0}, // Chroma Gain   7:0 bit //Ã¤µµ 
	{0x2f, 0x00, 0x00, 0}, //Luma   3:0 bit //±¤µµ 
	{0x30, 0x00, 0x00, 0}, //Luma   7:0 bit
    {0x0c, 0x3e, 0x00, 0}, // 0x0c=0e,0x0d=f0 = red. 0x0c=7e,0x0d = 88 = gray
    {0x0d, 0x88, 0x00, 0}, // 0x0c=0e,0x0d=0f = blue.
	{0x02, 0x54, 0x00, 0}, //force NTSC-M
    {0x08, 0x80, 0x00, 0}, // contrast 0x00  luma=0, 0x80  luma =1, 0xff luma = 2
    {0x0a, 0x00, 0x00, 0}, // Br 0x80(-30ire) ~ 0x7f(+30ire)
    {0x0b, 0x00, 0x00, 0}, // hue 0x80(-90) ~ 0x7f(+90)
	{0xe3, 0x80, 0x00, 0}, // 
	{0xe4, 0x80, 0x00, 0}, //
	{0x37, 0x00, 0x00, 0}, 
	{0xfd, 0x84, 0x00, 0}, 	
	{0xfe, 0x88, 0x00, 0}, 			
};

static struct reg_value adv7280_init_VPP[] = {  //write 84
	{0xa3, 0x00, 0x00, 10}, 	
	{0x5b, 0x00, 0x00, 10}, 	
	{0x55, 0x80, 0x00, 10}, 
};


static struct reg_value adv7280_init_CSI_MAP[] = {
	{0x17, 0x02, 0x00, 0}, 
	{0x00, 0x9c, 0x00, 0}, 
	{0x01, 0x70, 0x00, 0}, 		
	{0x30, 0x1c, 0x00, 0}, //ntsc	{0x30, 0x04, 0x00, 0},
	{0x31, 0x01, 0x00, 0}, 
};

static struct reg_value adv7280_status[] = {
	{0x00, 0x00, 0x00, 0}, 
	{0x01, 0x00, 0x00, 0}, 
	{0x02, 0x00, 0x00, 0},
	{0x03, 0x00, 0x00, 0}, 
	{0x04, 0x00, 0x00, 0}, 
	{0x05, 0x00, 0x00, 0}, 
	{0x06, 0x00, 0x00, 0}, 
	{0x07, 0x00, 0x00, 0},
	{0x08, 0x00, 0x00, 0}, 
	{0x09, 0x00, 0x00, 0}, 
	{0x0a, 0x00, 0x00, 0}, 
	{0x0b, 0x00, 0x00, 0}, 
	{0x0c, 0x00, 0x00, 0},
	{0x0d, 0x00, 0x00, 0}, 
	{0x0e, 0x00, 0x00, 0}, 		
	{0x0f, 0x00, 0x00, 0}, 
	{0x10, 0x00, 0x00, 0}, 		
	{0x11, 0x00, 0x00, 0}, 
	{0x12, 0x00, 0x00, 0},
	{0x13, 0x00, 0x00, 0}, 
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
};

static inline int adv7280_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;
    if(ADV7280_MSG) printk("===== %s ===== \n",__func__);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_dbg(&client->dev, "read reg error: ret = %d\n", ret);
	}

	return ret;
}

static inline int adv7280_write_reg(struct i2c_client *client,
		u8 reg, u8 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_dbg(&client->dev, "write reg error: ret = %d", ret);
	}

	return ret;
}

static int adv7280_config(struct i2c_client *c,
		struct reg_value *config, int size) {
	int i, ret;
	for (i = 0; i < size; i++) {
		if(ADV7280_MSG) printk(KERN_INFO "addr : %x ,  reg = 0x%02x, value = 0x%02x\n",c->addr, config[i].reg,  config[i].value);
		ret = adv7280_write_reg(c, config[i].reg,
				config[i].value | config[i].mask);
		if (ret < 0) {
			pr_err("%s: write error %x\n", __func__, ret);
			return ret;
		}

		if (config[i].delay_ms)
			msleep(config[i].delay_ms);
	}

	return 0;
}


static int adv7280_status_read(struct i2c_client *c,
		struct reg_value *config, int size) {
	int i;
	int ret;		
	for (i = 0; i < size; i++) {
		ret = adv7280_read_reg(c, config[i].reg);
	    if(ADV7280_MSG) printk(KERN_INFO "===read Reg(%x) : %x ============ \n",config[i].reg,ret);
	}

	return 0;
}

static int adv7280_default_config(struct adv7280_chipset *adv7280)
{
	int ret;
	adv7280_write_reg(adv7280->client, ADV7280_ADI_CONTROL1, 0x00);

	ret = adv7280_config(adv7280->client, adv7280_init_params,
			ARRAY_SIZE(adv7280_init_params));
	if (ret < 0) {
		pr_err("%s: config device error %x\n", __func__, ret);
		goto err;
	}

	adv7280_write_reg(adv7280->client, ADV7280_ADI_CONTROL1, 0x20);
	ret = adv7280_config(adv7280->client_vpp, adv7280_init_VPP,
			ARRAY_SIZE(adv7280_init_VPP));
	if (ret < 0) {
		pr_err("%s: config device error csi_top %x\n", __func__, ret);
		goto err;
	}

	adv7280_write_reg(adv7280->client, ADV7280_ADI_CONTROL1, 0x40);

	ret = adv7280_config(adv7280->client_csi_tx, adv7280_init_CSI_MAP,
			ARRAY_SIZE(adv7280_init_CSI_MAP));
	if (ret < 0) {
		pr_err("%s: config device error csi_top %x\n", __func__, ret);
		goto err;
	}

	adv7280_write_reg(adv7280->client, ADV7280_ADI_CONTROL1, 0x00);
	ret = adv7280_status_read(adv7280->client_csi_tx, adv7280_status,
			ARRAY_SIZE(adv7280_status));
	
err:
	i2c_unregister_device(adv7280->client_vpp);
	i2c_unregister_device(adv7280->client_csi_tx);
	adv7280->client_vpp = NULL;
	adv7280->client_csi_tx = NULL;
	
	return ret;
}

static int adv7280_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	u8 csi_addr;
	struct adv7280_chipset *adv7280;
	struct v4l2_subdev *sd;

	ADV7280_power(1);
	
	adv7280 = kzalloc(sizeof(struct adv7280_chipset), GFP_KERNEL);
	if (!adv7280)
		return -ENOMEM;

	i2c_set_clientdata(client, adv7280);

	adv7280->dev = &client->dev;
	adv7280->client = client;
	adv7280->pdata = client->dev.platform_data;

	sd = &adv7280->sd;
	strcpy(sd->name, "adv7280");
	
	v4l2_i2c_subdev_init(sd, client, &adv7280_ops);
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		 client->addr << 1, client->adapter->name);

    mdelay(500);

	csi_addr = ADV7280_VPP_ADDR;
	adv7280->client_vpp = i2c_new_dummy(client->adapter,
									csi_addr);	
	csi_addr = ADV7280_CSI_TX_ADDR;
	adv7280->client_csi_tx= i2c_new_dummy(client->adapter,
									csi_addr);	


	ret = adv7280_read_reg(client, ADV7280_IDENT);
	if (ret < 0) {
		pr_err("%s: read id error %x\n", __func__, ret);
		goto err;
	}
	if(ADV7280_MSG) printk(KERN_ERR " ADV7280_IDENT ret = %x \n",ret);

	if ((ret & ADV7280_SAMPLE_SILICON) != 0x40) {
		pr_err("%s: device ADV7280 not found, ret = 0x%02x\n",
				__func__, ret);
		goto err;
	}
	printk(KERN_INFO " ADV7280 found !!!!!  \n");
	
	if(ADV7280_MSG) pr_info("%s: device found, rev_id 0x%02x\n", __func__, ret);
	ret = adv7280_default_config(adv7280);
	if (ret < 0)
		goto err;


    BIT_STATUS(BIT_ADV7280,BIT_OK);
	return 0;
err:

    BIT_STATUS(BIT_ADV7280,BIT_ERROR);
	kfree(adv7280);
	return ret;
}

static int adv7280_i2c_remove(struct i2c_client *i2c_client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c_client);


	v4l2_device_unregister_subdev(sd);
	kfree(to_adv7280(sd));
	return 0;
}

static const struct i2c_device_id adv7280_i2c_id[] = {
	{"adv7280", 0},
	{"adv7280_vpp", 0},		
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7280_i2c_id);

static struct i2c_driver adv7280_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "adv7280",
		   },
	.probe = adv7280_i2c_probe,
	.remove = adv7280_i2c_remove,
	.id_table = adv7280_i2c_id,
};

static __init int adv7280_init(void)
{
	int err;

	err = i2c_add_driver(&adv7280_i2c_driver);

	if (err < 0)
		pr_err("%s: driver registration failed, error=%d\n",
			__func__, err);

    printk(KERN_INFO " ---------------- %s init ------------ \n",__func__);
	return err;
}

static void __exit adv7280_clean(void)
{
	i2c_del_driver(&adv7280_i2c_driver);
}

module_init(adv7280_init);
module_exit(adv7280_clean);

MODULE_DESCRIPTION("ADV7280 video decoder driver");
MODULE_AUTHOR("Uniquenet");
MODULE_LICENSE("GPL v2");

