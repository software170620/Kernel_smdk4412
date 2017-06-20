/*
 *  Button Virtual Driver
 *  Copyright (c) 2016 Uniquenet
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */


#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/mfd/kcppk/kcppk_common.h>
#include <linux/mfd/kcppk/kcppk_buttons.h>
#include <linux/init.h>
#include <mach/gpio-kcppk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <plat/gpio-cfg.h>
#include <linux/delay.h>
#include <linux/leds_pwm.h>
#include <linux/pwm.h>
#include <linux/leds.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/ctype.h>

#define BUTTON_DELAY_BT 2
#define BUTTON_PRESS_DELAY 50
#define AVOID_CHATTER_DELAY 20
#define CHECK_TERM 6 * 100  // 6 : 60 seconds
//#define one_times_per_second 1
//#define LED_PWM_FUNC_ENABLE 1

#ifdef LED_PWM_FUNC_ENABLE
#include <linux/pwm.h>
#include <linux/leds.h>
#endif

bool Debug_MSG = false;
bool Debug_MSG_IN = false;
bool BUTTON_LED_BLINKING = true;
bool BUTTON_LED_IS_ON = true;
bool LCD_FIRST_CMD =  true;
bool WAKE_LOCK_ENABLE = true;
bool SHUTDOWN_PROCESSING = false;

int HIGH_STATE = 1;
int LOW_STATE = 0;

static const char *THIS_MODULE_DEV_NAME = "IPC_UI_HAL";
static const char *SOAP_Monitor_DEV_NAME = "kcppk_kernel";
static const char *SOAP_LED_BR_DEV_NAME = "kcppk_led_br";


bool Boot_complete_hsc = true;  

bool KThread_kcppk_button_ing = true; 
bool KThread_low_power_ing = true;
bool KThread_low_power_cbit_ing = true;
bool KThread_heating_ing = true;
bool KThread_send_bit_ing = true;
bool KThread_FBC_CHECK_ing = true;
bool KThread_sure_key_ing = true;
bool ICBIT_CHECK_READY = true;
bool MADE_COMB_KEY = false;   //up & down key
int comb1 = UP_KEY_BTN; //CAM_WB_BTN;
int comb2 = DW_KEY_BTN; //CAM_WB_BTN;

char *Default_inform_date = ",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
char inform_to_java[30];
char inform_to_mainui[DATA_SIZE_TO_MAIN_UI];
bool wait_inform = false;
bool wait_inform_mainui = false;
bool wait_kcppk_kernel = false;
bool skip_up_down_key = false;
bool Is_Able_to_Accept_event = true;
int IPC_READ_OPEN = 0;

u8  SOAP_BUTTON_STATUS = 0x00;
u8  SOAP_BUTTON_MASK = (1 << UP_KEY_BTN) | (1 << DW_KEY_BTN); 


DECLARE_WAIT_QUEUE_HEAD(wait_key_input);
DECLARE_WAIT_QUEUE_HEAD(wait_sure_key);
DECLARE_WAIT_QUEUE_HEAD(wait_inform_to_java);
DECLARE_WAIT_QUEUE_HEAD(wait_inform_to_mainui);
DECLARE_WAIT_QUEUE_HEAD(wait_low_power_event);
DECLARE_WAIT_QUEUE_HEAD(wait_low_power_cbit_event);
DECLARE_WAIT_QUEUE_HEAD(wait_heating_event);
DECLARE_WAIT_QUEUE_HEAD(wait_send_bit_event); 	
DECLARE_WAIT_QUEUE_HEAD(wait_FBC_START_event);

struct task_struct *kcppk_buttons=NULL;
struct task_struct *send_to_android=NULL;
struct task_struct *alert_msg=NULL;
struct task_struct *alert_msg_2=NULL;
struct task_struct *heat_alert_msg=NULL;
struct task_struct *send_bit_msg=NULL;
struct task_struct *Button_CBIT=NULL;
struct mfd_buttons_platform_data *mfd_buttons;

struct temp_wake_lock {
	struct wake_lock wake_lock;
	char wake_lock_name[100];
} twakelock;

struct skcppk_button_switch {
	char   *name;
	int    gpio_num;
	bool   pull_down;
	int    check_time;
	bool   two_step;     // 1 button has two steps
	int    check_time2;
};

struct send_to_mainui
{
	char bt_info[2];
	char reserve_1[2];
	char reserve_2[2];
	char reserve_3[2];
	u8  BIT_INFO;
};

#ifdef LED_PWM_FUNC_ENABLE
struct led_pwm_data {
	struct led_classdev	cdev;
	struct pwm_device	*pwm;
	unsigned int 		active_low;
	unsigned int		period;
};
#endif

struct send_to_mainui *sendtomainui;

struct skcppk_button_switch kcppk_button_switch[] = {
	[0] = {
		.name        = "LCD_BT",
		.gpio_num    = Kcppk_LCD_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,      // BUTTON_DELAY_BT * 50ms
		.two_step    = false,
	},
	[1] = {
		.name        = "POWER_BT",
		.gpio_num    = Kcppk_Power_BTN,
		.pull_down   = true,
		.check_time  = BUTTON_DELAY_BT * 8,    	
		.two_step    = false,		
	},	
	[2] = {
		.name        = "MENU_BT",
		.gpio_num    = Kcppk_MENU_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,      	
		.two_step    = true,
		.check_time2 = BUTTON_DELAY_BT * 6,
	},	
	[3] = {
		.name        = "UP_BT",
		.gpio_num    = Kcppk_UP_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,     
		.two_step    = false,		
	},	
	[4] = {
		.name        = "DOWN_BT",		
		.gpio_num    = Kcppk_DOWN_BTN,
		.pull_down   = false, //
		.check_time  = BUTTON_DELAY_BT,     	
		.two_step    = false,		
	},	
	[5] = {
		.name        = "SELECT_BT",
		.gpio_num    = Kcppk_SELECT_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,    
		.two_step    = false,	
		.check_time2 = BUTTON_DELAY_BT * 20,	 //	50ms * 20 = 1000ms
	},	
	[6] = {
		.name        = "CALIB_BT",
		.gpio_num    = Kcppk_CALIB_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,     
		.two_step    = false,		
	},	
	[7] = {
		.name        = "WB_BT",
		.gpio_num    = Kcppk_WB_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,     	
		.two_step    = false,		
	},	
	[8] = {
		.name        = "VIEW_BT",
		.gpio_num    = Kcppk_VIEW_BTN,
		.pull_down   = false,
		.check_time  = BUTTON_DELAY_BT,    		
		.two_step    = false,		
	},		
};

#ifdef LED_PWM_FUNC_ENABLE
void set_led_pwm(struct led_classdev *led_cdev, int brightness) {
	struct led_pwm_data *led_dat =
		container_of(led_cdev, struct led_pwm_data, cdev);
	unsigned int max = led_dat->cdev.max_brightness;
	unsigned int period =  led_dat->period;

	if (brightness == 0) {
		pwm_config(led_dat->pwm, 0, period);
		pwm_disable(led_dat->pwm);
	} else {	
		pwm_config(led_dat->pwm, (brightness * 282) + 30000, period);
		pwm_enable(led_dat->pwm);
	}
}
#endif

void BUTTON_LED_TOGGLE_STOP() {
	BUTTON_LED_BLINKING = false;
}

bool BUTTON_LED_TOGGLE_CMD(int value) {  
	struct platform_device *pdev = get_bt_let_pdev();		
	struct led_classdev *led_cdev = dev_get_drvdata(&pdev->dev);
		
       int count;
	   bool set_value;
	   
	   set_value = false;
	   count = 0;

	   while(!set_value && count < 5) {
       gpio_set_value(BUTTON_LED_ON, value);
	   if(gpio_get_value(BUTTON_LED_ON) == value) set_value = true;
	   count++;
	   }
#ifdef LED_PWM_FUNC_ENABLE	   
	   if(led_cdev != NULL) {	
	       if(value == 0) {
	         set_led_pwm(led_cdev,0);   	
	       }
		   else {
	         set_led_pwm(led_cdev,64);	  	   	
		   }
       }	 
#endif	   
    return set_value;	
}

bool GPIO_SET_CMD(int GPIO_NUM, int value) {  //driver/mfd/kcppk_button
       int count;
	   bool set_value;
	   
	   set_value = false;
	   count = 0;

	   while(!set_value && count < 5) {
       gpio_set_value(GPIO_NUM, value);
	   if(gpio_get_value(GPIO_NUM) == value) set_value = true;
	   count++;
	   }
	   if(!set_value) {
	   	printk(KERN_ERR "Fail : GPIO_SET_CMD(gpio: %d, value: %d)\n",GPIO_NUM, value);
	   } 
    return set_value;	
}

static int send_data_android(int value, int step);
struct timer_list Boot_Completed_timer;
struct timer_list button_led_toggle;
struct timer_list Send_Comb_key_timer;
void Boot_Completed_timer_func(unsigned long data) {
		printk(KERN_INFO " ================== Kernel Boot Completed : %s =================== \n", utsname()->version);
}

void button_led_toggle_timer_func(unsigned long data) {
		if(BUTTON_LED_IS_ON) {
			 BUTTON_LED_IS_ON = false;
		     BUTTON_LED_TOGGLE_CMD(0);
		} else {
			 BUTTON_LED_IS_ON = true;
		     BUTTON_LED_TOGGLE_CMD(1);
		}
		if (BUTTON_LED_BLINKING) {
#ifdef one_times_per_second			
		  button_led_toggle.expires = jiffies + (HZ / 2);	
#else
		  button_led_toggle.expires = jiffies + (HZ);		
#endif
		  add_timer(&button_led_toggle);	
		} else {
			 BUTTON_LED_IS_ON = true;
		     BUTTON_LED_TOGGLE_CMD(1);		
		}
}

void Boot_Completed_timer_init(void) {
	Boot_Completed_timer.expires = jiffies + (HZ * 5);
	Boot_Completed_timer.data = 0;
	Boot_Completed_timer.function = &Boot_Completed_timer_func;
	add_timer(&Boot_Completed_timer);
}

void button_led_toggle_timer_init(void) {
	button_led_toggle.expires = jiffies + 170;
	button_led_toggle.data = 0;
	button_led_toggle.function = &button_led_toggle_timer_func;
	add_timer(&button_led_toggle);
}


void Send_Comb_key_timer_func(unsigned long data) {	
        skip_up_down_key = false;
}
void Send_Comb_key_timer_init(void) {
	Send_Comb_key_timer.expires = jiffies + (HZ * 1);
	Send_Comb_key_timer.data = 0;
	Send_Comb_key_timer.function = &Send_Comb_key_timer_func;
}


struct mfd_buttons_platform_data mfd_buttons_pd[] = {
	[0] = {
	.name           = "LCD_ON_OFF",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_LCD_IRQ,
	.gpio_num       = 0,
	},		
	[1] = {
	.name           = "POWER",
	.wakeup			= 1,
	.active_high    = true,
	.irq            = Kcppk_Power_IRQ,
	.gpio_num       = 0,
	},			
	[2] = {
	.name           = "MENU",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_MENU_IRQ,
	.gpio_num       = 0,
	},		
	[3] = {
	.name           = "UP",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_UP_IRQ,
	.gpio_num       = 0,
	},	
	[4] = {
	.name           = "DOWN",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_DOWN_IRQ,
	.gpio_num       = 0,
	},		
	[5] = {
	.name           = "SELECT",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_SELECT_IRQ,
	.gpio_num       = 0,
	},	
	[6] = {
	.name           = "CALIB",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_CALIB_IRQ,
	.gpio_num       = 0,
	},		
	[7] = {
	.name           = "WB",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_WB_IRQ,
	.gpio_num       = 0,
	},	
	[8] = {
	.name           = "VIEW",
	.wakeup			= 1,
	.active_high    = false,
	.irq            = Kcppk_VIEW_IRQ,
	.gpio_num       = 0,
	},		
	
};

struct mfd_buttons_platform_data_info mfd_buttons_pd_info = {
	.counts      = 9,
};

#if 1
bool wait_for_ready_read_dev(void);
void BIT_CHECK_RTN(void);
int  GPIO_CONFIGURE_FOR_KCPPK(); 
int  GPIO_CONFIGURE_FOR_KCPPK_2();	
int  Interrupt_CONFIGURE_FOR_KCPPK();
int kthread_FBC_CHECK(void *arg)
{
  int i;
  signed int pushed_button_count = 0;
  KThread_FBC_CHECK_ing = false;	
  interruptible_sleep_on(&wait_FBC_START_event);  
  KThread_FBC_CHECK_ing = true;  
  while(!kthread_should_stop())
  {  	
    i=0;
	while (i < mfd_buttons_pd_info.counts) {
	   if(i != 1 && i != 3 && i != 4){		
	       if(mfd_buttons[i].active_high && gpio_get_value(mfd_buttons[i].gpio_num) == HIGH_STATE)
	       {
	         mfd_buttons[i].error_count++;
	       } else if(!(mfd_buttons[i].active_high) && gpio_get_value(mfd_buttons[i].gpio_num) == LOW_STATE) {
	         mfd_buttons[i].error_count++;  
	       } else {
	         mfd_buttons[i].error_count = 0;
			 if(mfd_buttons[i].error_occurs) {
			 	pushed_button_count--;
				if(pushed_button_count == 0){
			        BIT_STATUS(BIT_F_BUTTON,BIT_OK | (1 << BIT_NMI));	
					if(Debug_MSG) printk("------------- CLEAR --------------\n");
				}
			 }
			 mfd_buttons[i].error_occurs = false;
	       }
	    if(mfd_buttons[i].error_count == CHECK_TERM) {
			mfd_buttons[i].error_count = (CHECK_TERM - 10);
			if(mfd_buttons[i].error_occurs == false) {
				mfd_buttons[i].error_occurs = true;
				pushed_button_count++;				
					if(Debug_MSG) printk("------------- SET ERROR : %d --------------\n",pushed_button_count);				
			}
			if(pushed_button_count == 1){			
		        BIT_STATUS(BIT_F_BUTTON,BIT_ERROR | (1 << BIT_NMI));	
			}
	    }
	   }
	i++;
	}
    msleep(95); //100
  }
  return 0;
}

int kthread_low_power(void *arg)
{
  bool Alert_INFORM_TO_UI = false;
  
  while(!kthread_should_stop())
  {  	

	if(gpio_get_value(LOW_POWER_ALERT) == HIGH_STATE) {	
	  	KThread_low_power_ing = false;
		Alert_INFORM_TO_UI = false;
		if(Debug_MSG) printk(KERN_INFO "KThread_low_power_ing = false \n");		
     	interruptible_sleep_on(&wait_low_power_event);
	}
	KThread_low_power_ing = true;
	if(Debug_MSG) printk(KERN_INFO "KThread_low_power_ing = true \n");	
		if(wait_for_ready_read_dev()){
			if(Alert_INFORM_TO_UI) {
				if(Is_Able_to_Accept_event){
					Is_Able_to_Accept_event = false;
					if(Debug_MSG) printk(KERN_INFO " ===== LOW_POWER_ALERT ===== \n");					
					sprintf(sendtomainui->bt_info,"%02x", LOW_P_ALERT);	
					wake_up_interruptible(&wait_inform_to_mainui);
				}
			} else {
				if(Debug_MSG) printk(KERN_INFO " ===== SKIP !!!! First LOW_POWER_ALERT ===== \n");					
				Alert_INFORM_TO_UI = true;
			}
		}
  msleep(1500);	
  }
  return 0;
}

int kthread_low_cbit_power(void *arg)
{
  while(!kthread_should_stop())
  {  	
	if(gpio_get_value(LOW_POWER_CBIT_ALERT) == HIGH_STATE) {	
	  	KThread_low_power_cbit_ing = false;
		if(Debug_MSG) printk(KERN_INFO "KThread_low_power_cbit_ing = false \n");		
     	interruptible_sleep_on(&wait_low_power_cbit_event);
	}
	KThread_low_power_cbit_ing = true;
	if(Debug_MSG) printk(KERN_INFO "KThread_low_power_cbit_ing = true \n");	

		if(wait_for_ready_read_dev()){
			    if(Is_Able_to_Accept_event){
					Is_Able_to_Accept_event = false;
					if(Debug_MSG) printk(KERN_INFO " ===== LOW_POWER_CBIT_ALERT ===== \n");					
					sprintf(sendtomainui->bt_info,"%02x", LOW_BIT_ALERT);	
					wake_up_interruptible(&wait_inform_to_mainui);
			    }
		}
    msleep(1500);
  }
  return 0;
}

static int kthread_heating(void *arg)
{
  while(!kthread_should_stop())
  {  	
    	KThread_heating_ing = false;		
     	interruptible_sleep_on(&wait_heating_event);
		KThread_heating_ing = true;
		if(Debug_MSG) printk(KERN_ERR "HEATING ALERT !!!!!! \n");
		BIT_STATUS(BIT_HEATING,BIT_ERROR);
  msleep(1500);	
  }
  return 0;
}

static int kthread_send_bit(void *arg)
{
  while(!kthread_should_stop())
  {  	
    	KThread_send_bit_ing = false;		
     	interruptible_sleep_on(&wait_send_bit_event);
		KThread_send_bit_ing = true;
	    BIT_CHECK_RTN();		 
  }
  return 0;
}


static irqreturn_t Low_Power_interrupt_handler(int irq, void *data)
{
	if(!KThread_low_power_ing){
	   wake_up_interruptible(&wait_low_power_event);
	}
	return IRQ_HANDLED;
}


static irqreturn_t Low_Power_Cbit_interrupt_handler(int irq, void *data)
{

	if(!KThread_low_power_cbit_ing){
	wake_up_interruptible(&wait_low_power_cbit_event);
	}
	return IRQ_HANDLED;
}

static irqreturn_t Heating_interrupt_handler(int irq, void *data)
{

	if(!KThread_heating_ing){
	wake_up_interruptible(&wait_heating_event);
	}
	return IRQ_HANDLED;
}

static int Low_Power_Check_GPIO(void) {
	   int ret = 0;
		ret = gpio_request(LOW_POWER_ALERT,"Low_Power_Alert");
	    if(ret == 0) {
			s3c_gpio_cfgpin(LOW_POWER_ALERT, S3C_GPIO_INPUT);
			s3c_gpio_setpull(LOW_POWER_ALERT, S3C_GPIO_PULL_UP);
            if(Debug_MSG) printk(KERN_INFO "Low_Power_Alert pin pull-up ok !!!!\n");			
	    }
	
	   ret = request_irq(gpio_to_irq(LOW_POWER_ALERT), Low_Power_interrupt_handler,
	   	                          (IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
	   	                          "Low_Power_Alert", NULL);
		if (ret) {
		    printk(KERN_ERR " Can't allocate request irq for %s memory space !!! \n","Low_Power_Alert");
		}
//add low power alert for CBIT
		ret = gpio_request(LOW_POWER_CBIT_ALERT,"LOW_POWER_CBIT_ALERT");
	    if(ret == 0) {
			s3c_gpio_cfgpin(LOW_POWER_CBIT_ALERT, S3C_GPIO_INPUT);
			s3c_gpio_setpull(LOW_POWER_CBIT_ALERT, S3C_GPIO_PULL_UP);
            if(Debug_MSG) printk(KERN_INFO "LOW_POWER_CBIT_ALERT pin pull-up ok !!!!\n");			
	    }
	
	    ret = request_irq(gpio_to_irq(LOW_POWER_CBIT_ALERT), Low_Power_Cbit_interrupt_handler,
	   	                          (IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
	   	                          "LOW_POWER_CBIT_ALERT", NULL);
		if (ret) {
		    printk(KERN_ERR " Can't allocate request irq for %s memory space !!! \n","LOW_POWER_CBIT_ALERT");
		}

		return ret;
}


static int Heating_Check_GPIO(void) {
	// kcppk Low_Power_Alert
	   int ret = 0;
		ret = gpio_request(HEATING_ALERT,"Heating_Alert");
	    if(ret == 0) {
			s3c_gpio_cfgpin(HEATING_ALERT, S3C_GPIO_INPUT);
			s3c_gpio_setpull(HEATING_ALERT, S3C_GPIO_PULL_UP);
            if(Debug_MSG) printk(KERN_INFO "Heating_Alert pin pull-up ok !!!!\n");			
	    }
	
	   ret = request_irq(gpio_to_irq(HEATING_ALERT), Heating_interrupt_handler,
	   	                          (IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
	   	                          "Heating_Alert", NULL);
		if (ret) {
		    printk(KERN_ERR " Can't allocate request irq for %s memory space !!! \n","Heating_Alert");
		}
		return ret;
}
#endif


static int kcppk_buttons_gpio_init(void) { 
       struct skcppk_button_switch *kcppk_buttonsw = kcppk_button_switch;
       int ret = 0;
       int i = 0;
	   while (i < mfd_buttons_pd_info.counts) {
	       ret = gpio_request(kcppk_buttonsw[i].gpio_num,kcppk_buttonsw[i].name);
	       if (ret < 0) {
	               printk(KERN_ERR "failed to request %s GPIO# %d: %d\n",kcppk_buttonsw[i].name, kcppk_buttonsw[i].gpio_num,ret);
	               return ret;
	       } else {
	               printk(KERN_INFO "Successed to request %s GPIO# %d: %d\n",kcppk_buttonsw[i].name, kcppk_buttonsw[i].gpio_num,ret);
	       }

	       if(kcppk_buttonsw[i].pull_down)
	       {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_DOWN);
				gpio_direction_input(kcppk_buttonsw[i].gpio_num);


                mdelay(10);
                if(gpio_get_value(kcppk_buttonsw[i].gpio_num) == HIGH_STATE)  {
					printk("=============  %s == Button Pressed  ==========\n", kcppk_buttonsw[i].name);
					             BIT_STATUS(BIT_F_BUTTON,BIT_ERROR);
                }			
	       } else {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_UP);
				gpio_direction_input(kcppk_buttonsw[i].gpio_num);      
	                mdelay(10);			
                if(gpio_get_value(kcppk_buttonsw[i].gpio_num) == LOW_STATE)  {
					printk("=============  %s == Button Pressed  ==========\n", kcppk_buttonsw[i].name);
					             BIT_STATUS(BIT_F_BUTTON,BIT_ERROR);
                }			
	       }

	    	gpio_export(kcppk_buttonsw[i].gpio_num,1);		//test LCD on/off
			i++;
			if (i == 1) i++; //power button skip			
	   }
	   
    return 0;
}

static void Bit_gpio_input_check_Rtn(void)
{
       struct skcppk_button_switch *kcppk_buttonsw = kcppk_button_switch;
	   int front_button_error_count = 0;
       int ret = 0;
       int i = 0;
	   while (i < mfd_buttons_pd_info.counts) {
	       ret = gpio_request(kcppk_buttonsw[i].gpio_num,kcppk_buttonsw[i].name);
	       if(kcppk_buttonsw[i].pull_down)
	       {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_DOWN);
				gpio_direction_input(kcppk_buttonsw[i].gpio_num);
			   
	       } else {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_UP);
				gpio_direction_input(kcppk_buttonsw[i].gpio_num);   	   
	       }
		    ret = gpio_get_value(kcppk_buttonsw[i].gpio_num);
            if(Debug_MSG) printk(" %s status : %d (0:low, 1:high) \n",kcppk_buttonsw[i].name,ret );
			if(ret == 0) front_button_error_count++;
	    	gpio_free(kcppk_buttonsw[i].gpio_num);		
			i++;
			if (i == 1) i++; //power button skip
	   }
	   if(front_button_error_count == (mfd_buttons_pd_info.counts - 1)) {
	   	 printk("------   Check Cable for Front Button !!!!!   ------\n");
		 BIT_STATUS(BIT_F_BUTTON,BIT_ERROR);
	   }
}

static void Bit_gpio_check_Rtn(void)
{
       struct skcppk_button_switch *kcppk_buttonsw = kcppk_button_switch;
       int ret = 0;
       int i = 0;
	   bool GPIO_Error;
				   if(Debug_MSG) printk("------------------------  GPIO  ----------------------\n");
	   while (i < mfd_buttons_pd_info.counts) {
				   if(Debug_MSG) printk("-----------------------------------------------------\n");	
		   GPIO_Error = false;
	       ret = gpio_request(kcppk_buttonsw[i].gpio_num,kcppk_buttonsw[i].name);
	       if (ret < 0) {
			GPIO_Error = true;
	       } 

	       if(kcppk_buttonsw[i].pull_down)
	       {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_DOWN);
				gpio_direction_output(kcppk_buttonsw[i].gpio_num,0);

		           if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,1)) {
				   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set High !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;
		           }
				   mdelay(10);				   
				   if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,0)){
				   	if(Debug_MSG) printk(" %s OK : Set Low  !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set Low !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }
				   mdelay(10);				   
		           if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,1)) {
				   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set High !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }		
				   mdelay(10);				   
				   if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,0)){
				   	if(Debug_MSG) printk(" %s OK : Set Low  !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set Low !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }	
				   mdelay(10);				   
		           if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,1)) {
				   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set High !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }					   
				
	       } else {
				s3c_gpio_setpull(kcppk_buttonsw[i].gpio_num, S3C_GPIO_PULL_UP);
				gpio_direction_output(kcppk_buttonsw[i].gpio_num, 1);   

		           if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,0)) {
				   	if(Debug_MSG) printk(" %s OK :  Set Low !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail :  Set Low !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }
				   mdelay(10);
				   if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,1)){
				   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set High !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }
				   mdelay(10);				   
		           if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,0)) {
				   	if(Debug_MSG) printk(" %s OK :  Set Low !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail :  Set Low !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }	
				   mdelay(10);
				   if(GPIO_SET_CMD(kcppk_buttonsw[i].gpio_num,1)){
				   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_buttonsw[i].name);						   	
		           } else {
				   	printk("== %s fail : Set High !!\n", kcppk_buttonsw[i].name);
					GPIO_Error = true;					
		           }				   
	       }
           if(GPIO_Error) {
             BIT_STATUS(BIT_F_BUTTON,BIT_ERROR);
           }
	    	gpio_free(kcppk_buttonsw[i].gpio_num);		//test LCD on/off
			i++;
			if (i == 1) i++; //power button skip
	   }
}

static void Bit_gpio_check2_Rtn(void)
{
	   struct mfd_buttons_platform_data *kcppk_test_irq = mfd_buttons_pd;

       int ret = 0;
       int i = 0;
	   bool GPIO_Error;
				  if(Debug_MSG)  printk("------------------------  IRQ   ----------------------\n");
	   while (i < mfd_buttons_pd_info.counts) {
				  if(Debug_MSG)  printk("-----------------------------------------------------\n");	
		   GPIO_Error = false;
	       ret = gpio_request(kcppk_test_irq[i].irq, kcppk_test_irq[i].name);
	       if (ret < 0) {
			 GPIO_Error = true;	   	
	       } 


			s3c_gpio_setpull(kcppk_test_irq[i].irq, S3C_GPIO_PULL_DOWN);
			gpio_direction_output(kcppk_test_irq[i].irq,0);

	           if(GPIO_SET_CMD(kcppk_test_irq[i].irq,1)) {
			   	if(Debug_MSG) printk(" %s OK : Set High !!\n", kcppk_test_irq[i].name);						   	
	           } else {
	  	       GPIO_Error = true;	         
			   	printk("== %s fail : Set High !!\n", kcppk_test_irq[i].name);
	           }
			   mdelay(10);				   
			   if(GPIO_SET_CMD(kcppk_test_irq[i].irq,0)){
			   if(Debug_MSG) 	printk(" %s OK : Set Low  !!\n", kcppk_test_irq[i].name);						   	
	           } else {
	  	       GPIO_Error = true;	           
			   	printk("== %s fail : Set Low !!\n", kcppk_test_irq[i].name);
	           }
			   mdelay(10);				   
	           if(GPIO_SET_CMD(kcppk_test_irq[i].irq,1)) {
			   if(Debug_MSG) 	printk(" %s OK : Set High !!\n", kcppk_test_irq[i].name);						   	
	           } else {
	  	       GPIO_Error = true;	           
			   	printk("== %s fail : Set High !!\n", kcppk_test_irq[i].name);
	           }		
			   mdelay(10);				   
			   if(GPIO_SET_CMD(kcppk_test_irq[i].irq,0)){
			   if(Debug_MSG) 	printk(" %s OK : Set Low  !!\n", kcppk_test_irq[i].name);						   	
	           } else {
	  	       GPIO_Error = true;	           
			   	printk("== %s fail : Set Low !!\n", kcppk_test_irq[i].name);
	           }				   
			   mdelay(10);				   
	           if(GPIO_SET_CMD(kcppk_test_irq[i].irq,1)) {
			   if(Debug_MSG) 	printk(" %s OK : Set High !!\n", kcppk_test_irq[i].name);						   	
	           } else {
	  	       GPIO_Error = true;	           
			   	printk("== %s fail : Set High !!\n", kcppk_test_irq[i].name);
	           }	
			   
	           if(GPIO_Error) {
	             BIT_STATUS(BIT_F_BUTTON,BIT_ERROR);
	           }
			   
	    	gpio_free(kcppk_test_irq[i].irq);		//test LCD on/off
			i++;
			if (i == 1) i++; //power button skip
	   }
}


static irqreturn_t dummy_kcppk_interrupt_handler(int irq, void *data)
{
    int i = 0;
	bool event_found;
	
	if(!KThread_kcppk_button_ing){
		event_found = false;
	    while(i < mfd_buttons_pd_info.counts) {
			if(mfd_buttons[i].irq == irq && mfd_buttons[i].event_occurs == 0) {
						mfd_buttons[i].event_occurs = 1;
			    		if(Debug_MSG) printk(KERN_INFO " %s event occurs !!! , gpio num : %d , button_num : %d \n",mfd_buttons[i].name ,mfd_buttons[i].gpio_num,i );	
						event_found = true;
			} else {
						mfd_buttons[i].event_occurs = 0;
			}
			i++;
		}
		if(event_found) wake_up_interruptible(&wait_key_input);		
	}
	return IRQ_HANDLED;
}

bool wait_for_ready_read_dev() {
	int count;

	if(IPC_READ_OPEN > 0){
	   count = 0;
	   while(wait_inform_mainui && count < 20) {
		   	msleep(1);
			count++;
			if(Debug_MSG) printk(KERN_INFO " ===== %d ====\n",count);
	   }
       if(!wait_inform_mainui) return true;
	} 
	if(Debug_MSG) printk(KERN_INFO " ===== return false ====\n");	
	return false;
}

static int send_data_android(int value, int step) {
	switch(value) {
		case LCD_ON_OFF:

			   sprintf(sendtomainui->bt_info,"%02x", LCD_ON_OFF);
			   break;
		case POWER_BTN:
	
			   sprintf(sendtomainui->bt_info,"%02x", POWER_BTN);
			   break;	
		case MENU_BTN:

			   if(step == TWO_STEP) {
		   	
			   sprintf(sendtomainui->bt_info,"%02x", HOME_BTN);			   	
			   } else {
		   
			   sprintf(sendtomainui->bt_info,"%02x", MENU_BTN);
			   }
			   break;
		case UP_KEY_BTN:
	
			   sprintf(sendtomainui->bt_info,"%02x", UP_KEY_BTN);
			   break;	
		case DW_KEY_BTN:
				
			   sprintf(sendtomainui->bt_info,"%02x", DW_KEY_BTN);
			   break;
		case SELECT_BTN:
			   if(step == TWO_STEP) {
			   	
			   sprintf(sendtomainui->bt_info,"%02x", SELECT_BTN);			   	
			   } else {
			   
			   sprintf(sendtomainui->bt_info,"%02x", SELECT_BTN);
			   }			   
			   break;	
		case CAM_CAL_BTN:
	
			   sprintf(sendtomainui->bt_info,"%02x", CAM_CAL_BTN);
			   break;
		case CAM_WB_BTN:
				
			   sprintf(sendtomainui->bt_info,"%02x", CAM_WB_BTN);
			   break;	
		case CAM_VW_BTN:
				
			   sprintf(sendtomainui->bt_info,"%02x", CAM_VW_BTN);
			   break;
		case CAM_ONOFF_BTN:
					
			   sprintf(sendtomainui->bt_info,"%02x", CAM_ONOFF_BTN);
			   break;				
		case BIT_CHECK:
		
			   sprintf(sendtomainui->bt_info,"%02x", 0x80 | sendtomainui->BIT_INFO);
			   break;					   
	    default:
					
			   sprintf(sendtomainui->bt_info,"%02x", HOME_BTN);
			   break;
	}
	if(wait_for_ready_read_dev()){
		wake_up_interruptible(&wait_inform_to_mainui);
	} else {
		Is_Able_to_Accept_event = true;
	}
	return 0;
}

void IBIT_CHECK_RTN(void) {  
	int i = 0;
	int ret = 0;
	BIT_STATUS(BIT_F_BUTTON,BIT_OK);	
	while(i < mfd_buttons_pd_info.counts) {
        free_irq(mfd_buttons[i].irq,mfd_buttons);
		gpio_free(mfd_buttons[i].gpio_num);
        printk(KERN_INFO " Free gpio : %d,  irq : %s \n",mfd_buttons[i].gpio_num,mfd_buttons[i].name );		
		i++;
	}

	msleep(1000);
	
    GPIO_CONFIGURE_FOR_KCPPK(); 

    ret = Interrupt_CONFIGURE_FOR_KCPPK();
	if(ret < 0) {printk(KERN_ERR " Can't allocate request irq for %s memory space !!! \n","Front Buttons" );}
	
    GPIO_CONFIGURE_FOR_KCPPK_2();
	
	while(KThread_send_bit_ing){
		  	msleep(1);}
	if(!KThread_send_bit_ing) {
		wake_up_interruptible(&wait_send_bit_event);	
	}	
    return;	
}

void BIT_CHECK_RTN(void) {  
    int count;

    count = 0;
	
	if(gpio_get_value(HEATING_ALERT) == HIGH_STATE) {
		BIT_STATUS(BIT_HEATING,BIT_OK);		
	}
	while(!Is_Able_to_Accept_event) {
		msleep(1);
	}
		Is_Able_to_Accept_event = false;
		send_data_android(BIT_CHECK, 0);
    return;	
}

void BIT_STATUS(u8 value, u8 setvalue)
{
	bool NMI_Check;
	NMI_Check = (setvalue & (1 << BIT_NMI))? true:false;
	setvalue = setvalue & ~(1 << BIT_NMI);
	sendtomainui->BIT_INFO &= ~(1 << value);
	if(setvalue == BIT_ERROR){
		sendtomainui->BIT_INFO |= (1 << value);
		if(value == BIT_ADV7280_OV || NMI_Check) {
		  if(!KThread_send_bit_ing) {
			wake_up_interruptible(&wait_send_bit_event);	
		  }
		}
	} else if(NMI_Check) {
	      while(KThread_send_bit_ing){
		  	msleep(1);}
		  if(!KThread_send_bit_ing) {
			wake_up_interruptible(&wait_send_bit_event);	
		  }	
	}
	if(Debug_MSG) printk("=================sendtomainui->BIT_INFO : %x ========\n",sendtomainui->BIT_INFO);
}

static int kthread_kcppk_buttons(void *arg)
{
  int index = 0;
  int checking_time = 0;
  bool ignore = false;
  bool Activated = false;
  bool Activated2 = false;  //step two
  while(!kthread_should_stop())
  {  	
  	KThread_kcppk_button_ing = false;
	if(Debug_MSG) printk(KERN_INFO "KThread_kcppk_button_ing = false \n");
  	interruptible_sleep_on(&wait_key_input);
	if(Debug_MSG)printk(KERN_INFO "KThread_kcppk_button_ing = true \n");
	KThread_kcppk_button_ing = true;

    ignore = false;
	Activated = false;
	Activated2 = false;

    	msleep(AVOID_CHATTER_DELAY);	// avoid chatter    
        if(Is_Able_to_Accept_event) { 
        Is_Able_to_Accept_event = false;
        index = 0;
        while(index < mfd_buttons_pd_info.counts) {
          if(Debug_MSG) printk(KERN_INFO " ===  index : %d , mfd_buttons[index].event_occurs : %d =====!!\n", index, mfd_buttons[index].event_occurs);			
		  if(mfd_buttons[index].event_occurs == 1) {
                if(Debug_MSG) printk(KERN_INFO " %s event occurs !!! , gpio num : %d , event : %d \n",mfd_buttons[index].name ,mfd_buttons[index].gpio_num,mfd_buttons[index].event_occurs );	
			    if(!(gpio_get_value(mfd_buttons[index].gpio_num) == mfd_buttons[index].active_high))
			    {
					if(Debug_MSG) printk(KERN_INFO " ===  ignore!!!!!  event cleared=====!!\n");
					mfd_buttons[index].event_occurs = 0;		
					ignore = true;
			    } 
				else if(index == comb1 || index == comb2){
					if(!KThread_sure_key_ing) 
					   wake_up_interruptible(&wait_sure_key);
					goto COMB_KEY_JMP;			
				} else  {
					checking_time = 0;
					while(gpio_get_value(mfd_buttons[index].gpio_num) == mfd_buttons[index].active_high) {  //while pushed button..
						if(Debug_MSG) printk(KERN_INFO " ===  pushed =====!!\n");
						checking_time++;
	                    if(checking_time > mfd_buttons[index].check_time) { 
							Activated = true;	
							if(mfd_buttons[index].two_step) {
								if( checking_time > mfd_buttons[index].check_time2) {
								  Activated2 = true;
							      break;
								} else {
								 //wait... 
								}
							 } else {
							   break;  //one_step break;
							}
	                    }
						mdelay(BUTTON_PRESS_DELAY - 20);
					}

					break;  //exit while with current event number
				}
		   } 
			  index++;
	    }
	}

	if(!ignore) {

			   if(Activated2) {
					send_data_android(index,TWO_STEP);
			   }
			   
			   else if(Activated) {
					send_data_android(index,ONE_STEP);
			   }
	
			MADE_COMB_KEY = false;
			if(Debug_MSG) printk(KERN_INFO " ===event cleared===== %s ========= !!\n",__func__);	
			mfd_buttons[index].event_occurs = 0;
	    } 
	      Is_Able_to_Accept_event = true;  // kthread_kcppk_buttons is done

COMB_KEY_JMP:
		if(Debug_MSG) printk(KERN_INFO " === key input end ==== \n");	
   }


  return 0;
} 

static int kthread_wait_combkey(void *arg)
{
  int current_key;
  int checking_time = 0;
  bool ignore = false;
  bool Activated = false;  
  
  
  while(!kthread_should_stop())
  {  	
  	KThread_sure_key_ing = false;
  	interruptible_sleep_on(&wait_sure_key);
	KThread_sure_key_ing = true;

	ignore = false;
	Activated = false;  
    current_key = comb1;
    if(mfd_buttons[current_key].event_occurs != 1) {
		current_key = comb2;
    } 

    if(!(gpio_get_value(mfd_buttons[current_key].gpio_num) == mfd_buttons[current_key].active_high))
	{
	      mfd_buttons[comb1].event_occurs = 0;	
	      mfd_buttons[comb2].event_occurs = 0;		
	} else { 
		while(gpio_get_value(mfd_buttons[current_key].gpio_num) == mfd_buttons[current_key].active_high ) {  //continue..
				checking_time = 0;
				while(gpio_get_value(mfd_buttons[current_key].gpio_num) == mfd_buttons[current_key].active_high ) {  //while pushed button..
	                  if((gpio_get_value(mfd_buttons[comb1].gpio_num) == mfd_buttons[comb1].active_high) && (gpio_get_value(mfd_buttons[comb2].gpio_num) == mfd_buttons[comb2].active_high)){
	                    MADE_COMB_KEY = true;
						Activated = true;
	                    break;
					  }			
					if(Debug_MSG) printk(KERN_INFO " ===== pushed up_down=====!!\n");
					checking_time++;

		            if(checking_time > mfd_buttons[current_key].check_time) { 
						Activated = true;	
						break;  //one_step break;
		            }
					msleep(BUTTON_PRESS_DELAY);
				}

				if(Activated && MADE_COMB_KEY) {
	                skip_up_down_key = true;
	                send_data_android(CAM_ONOFF_BTN,ONE_STEP);
			        if(IPC_READ_OPEN > 0){		// do not use timer until device ready	
	                	add_timer(&Send_Comb_key_timer);
			        }
	                break;
				} else {
				    if(!skip_up_down_key)
					send_data_android(current_key,ONE_STEP);
				}
				msleep(AVOID_CHATTER_DELAY * 5);
		}
	      MADE_COMB_KEY = false;
	      mfd_buttons[comb1].event_occurs = 0;	
	      mfd_buttons[comb2].event_occurs = 0;			  
	      Is_Able_to_Accept_event = true; 
	}
  }
  return 0;
} 

int kcppk_buttons_open (struct inode *inode, struct file *filp)
{
	if(Debug_MSG) printk(KERN_INFO " === %s === \n",__func__);
	IPC_READ_OPEN += 1;
	return 0;
}

int kcppk_buttons_release(struct inode *inode, struct file *filp)
{
	if(Debug_MSG)printk(KERN_INFO " === %s === \n",__func__);	
	IPC_READ_OPEN -= 1;
	return 0;
}

ssize_t kcppk_buttons_write(struct file *filp, const char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	*f_pos = count;
	return count;
}

long kcppk_buttons_ioctl(struct file *filp, unsigned int cmd, unsigned long value)
{
	int ret;
	int arg;
	ret = 0;

	arg = value;

    if(Debug_MSG) printk(KERN_ERR " INPUT CMD = %d , arg = %d \n",cmd,arg);
	switch(cmd) 
	{
		case KCPPK_LCD:
			if(Debug_MSG_IN) printk("KCPPK_LCD !!!!!!!!!!! : %x \n",arg);
			if(!BUTTON_LED_BLINKING && !SHUTDOWN_PROCESSING) {
				GPIO_SET_CMD(LCD_POWER_UPDOWN,arg);	
				GPIO_SET_CMD(BUTTON_LED_ON,arg);		
			} else if(LCD_FIRST_CMD)
			{
				LCD_FIRST_CMD = false;
				GPIO_SET_CMD(LCD_POWER_UPDOWN,arg);					
			}
			break;
		case KCPPK_BRT_AUTO:
	
			break;
		case CAMERA_CTRL:

			GPIO_SET_CMD(HTC_CAM_POWER,arg);		
			break;		
		case KCPPK_POWER_OFF:

			SHUTDOWN_PROCESSING = true;
			GPIO_SET_CMD(LCD_POWER_UPDOWN,0);	
			GPIO_SET_CMD(BUTTON_LED_ON,0);				
			memcpy(inform_to_java,"VUL_SHUTDOWN_SET,00,",20);		
			msleep(1);
			if(wait_kcppk_kernel) wake_up_interruptible(&wait_inform_to_java);		
			break;	
		case KCPPK_REBOOT_CMD:

			memcpy(inform_to_java,"VUL_REBOOT_SET,00,",18);		
			msleep(1);
			if(wait_kcppk_kernel) wake_up_interruptible(&wait_inform_to_java);		
			break;				
		case ANI_START:

			memcpy(inform_to_java,"VUL_ANISTART_SET,01,",20);		
			msleep(1);
			if(wait_kcppk_kernel) wake_up_interruptible(&wait_inform_to_java);		
			break;		
		case ANI_STOP:

			memcpy(inform_to_java,"VUL_ANISTART_SET,00,",20);		
			msleep(1);
			if(wait_kcppk_kernel) wake_up_interruptible(&wait_inform_to_java);		
			break;			
		case LIGHT_SENSOR_ON:

			SOAP_SENSOR_ENABLE(true);
			break;
		case LIGHT_SENSOR_OFF:		

			SOAP_SENSOR_ENABLE(false);	 
			break;		
		case BIT_CHECK:		
			if(ICBIT_CHECK_READY) {
				ICBIT_CHECK_READY = false;
				if(Debug_MSG) printk("BIT_CHECK !!!!!!!!!!! \n"); 
				BIT_CHECK_RTN();
				ICBIT_CHECK_READY = true;				
			}
			break;			
		case IBIT_CHECK:	
			while(!ICBIT_CHECK_READY) { msleep(1); }
			ICBIT_CHECK_READY = false;			
			if(Debug_MSG) printk("IBIT_CHECK !!!!!!!!!!! \n");
			IBIT_CHECK_RTN();	
			ICBIT_CHECK_READY = true;					
			break;					
		case BUTTON_LED_TOOGLE_STOP:		

			BUTTON_LED_TOGGLE_STOP();	 
			break;					
		default:
			ret = -1;
			break;
	}

	return ret;
}

ssize_t kcppk_buttons_read(struct file *filp,char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	int i;
	if(*f_pos > 0) return 0; 
	wait_inform_mainui = false;

  	interruptible_sleep_on(&wait_inform_to_mainui);
    wait_inform_mainui = true;

	printk("===sendtomainui: %s  === \n",sendtomainui->bt_info);		
	if(copy_to_user(buf,sendtomainui->bt_info,2) != 0) {		
	   printk(KERN_ERR "Error : copy to user !!\n");
	   return -EFAULT;
	} 
	if(Debug_MSG) {
		i = 0;
		while(i < mfd_buttons_pd_info.counts) {
			printk(" %d ",mfd_buttons[i].event_occurs);
				i++;
		}
	}
	i = 0;
	while(i < mfd_buttons_pd_info.counts) {
		mfd_buttons[i].event_occurs = 0;
			i++;
			while(i == comb1 || i== comb2) { i++;}
	}
    Is_Able_to_Accept_event = true;  //key event terminated..	
	printk(" \n ");
	return 2;
}


struct file_operations kcppk_buttons_fops = {
   	.owner 	= THIS_MODULE,
	.open 	= kcppk_buttons_open,
	.write  = kcppk_buttons_write,
	.read   = kcppk_buttons_read,
	.unlocked_ioctl  = kcppk_buttons_ioctl,
	.release	= kcppk_buttons_release,
};

struct cdev kcppk_buttons_char_cdev = {
	.kobj = {
			.name="kcppk_buttons",
		},
	.owner = THIS_MODULE,
	.ops = &kcppk_buttons_fops,
};

struct miscdevice kcppk_buttons_misc_device = {
		.minor = 0,
		.name = "kcppk_buttons",
		.fops = &kcppk_buttons_fops,
		.mode = (S_IRUGO | S_IRWXUGO | S_IALLUGO),
};

int buttons_device_create(void) {
	   	dev_t dev;
		int err = 0;	
	    struct class *cl;
	    struct miscdevice *misd;	
		misd = &kcppk_buttons_misc_device;
	    dev = MKDEV(Kcppk_Char_Major, Kcppk_Char_Dev_2);
	    err = register_chrdev_region(dev, 1, THIS_MODULE_DEV_NAME);

	    cdev_init(&kcppk_buttons_char_cdev, &kcppk_buttons_fops);
	    kcppk_buttons_char_cdev.owner = THIS_MODULE;
	    kcppk_buttons_char_cdev.ops  = &kcppk_buttons_fops;

	    cl = class_create(THIS_MODULE, "chardrv_kcppk_buttons");
	    device_create(cl, NULL, dev, NULL, THIS_MODULE_DEV_NAME);

	    if(cdev_add(&kcppk_buttons_char_cdev, dev, 1)) {
	        printk(KERN_INFO "/dev/%s : cdev creation failed.\n",THIS_MODULE_DEV_NAME);
			goto err_create;
	    }
	    printk(KERN_INFO "/dev/%s: cdev creation Ok.\n",THIS_MODULE_DEV_NAME);
		if(kcppk_buttons == NULL){ 
		   kcppk_buttons = (struct task_struct *)kthread_run(kthread_kcppk_buttons, NULL, "kthread_kcppk_buttons");
           if(kcppk_buttons == NULL) {
	    	printk(KERN_INFO "Memory Allocation Error : %s buffer\n",THIS_MODULE_DEV_NAME);			   	
		    } else
		   {
	    	printk(KERN_INFO "Memory Allocation OK.. : %s \n",THIS_MODULE_DEV_NAME);	   	
		   }
		}


		
		if(send_to_android== NULL){ 
		   sendtomainui = kzalloc(sizeof(struct send_to_mainui), GFP_KERNEL);
           if(sendtomainui == NULL) {
	    	printk(KERN_INFO "Memory Allocation Error : sendtomainui read buffer\n");			   	
		    } else
		   {
	    	printk(KERN_INFO "Memory Allocation OK.. : sendtomainui \n");	   	
		   }		   
		   send_to_android = (struct task_struct *)kthread_run(kthread_wait_combkey, NULL, "kthread_send_to_android");
           if(send_to_android == NULL) {
	    	printk(KERN_INFO "Memory Allocation Error : %s buffer\n",THIS_MODULE_DEV_NAME);			   	
		    } else
		   {
	    	printk(KERN_INFO "Memory Allocation OK.. : %s \n",THIS_MODULE_DEV_NAME);	   	
		   }
		}
        if(!Low_Power_Check_GPIO()){
			if(alert_msg == NULL){ 
			   alert_msg = (struct task_struct *)kthread_run(kthread_low_power, NULL, "kthread_low_power");
	           if(alert_msg == NULL) {
		    	printk(KERN_ERR "Memory Allocation Error : %s buffer\n","LOW POWER ALERT");			   	
			    } else
			   {
		    	printk(KERN_INFO "Memory Allocation OK.. : %s \n","LOW POWER ALERT");	   	
			   }
			}
			if(alert_msg_2 == NULL){ 
			   alert_msg_2 = (struct task_struct *)kthread_run(kthread_low_cbit_power, NULL, "kthread_low_cbit_power");
	           if(alert_msg_2 == NULL) {
		    	printk(KERN_ERR "Memory Allocation Error : %s buffer\n","LOW POWER CBIT ALERT");			   	
			    } else
			   {
		    	printk(KERN_INFO "Memory Allocation OK.. : %s \n","LOW POWER CBIT ALERT");	   	
			   }
			}			
        }

		if(send_bit_msg == NULL){ 
		   send_bit_msg = (struct task_struct *)kthread_run(kthread_send_bit, NULL, "kthread_send_bit");
           if(send_bit_msg == NULL) {
	    	printk(KERN_ERR "Memory Allocation Error : %s buffer\n","SEND BIT Message");			   	
		    } else
		   {
	    	printk(KERN_INFO "Memory Allocation OK.. : %s \n","SEND BIT Message");	   	
		   }
		}

        if(!Heating_Check_GPIO()){
			if(heat_alert_msg == NULL){ 
			   heat_alert_msg = (struct task_struct *)kthread_run(kthread_heating, NULL, "kthread_heating");
	           if(heat_alert_msg == NULL) {
		    	printk(KERN_ERR "Memory Allocation Error : %s buffer\n","HEATING ALERT");			   	
			    } else
			   {
		    	printk(KERN_INFO "Memory Allocation OK.. : %s \n","HEATING ALERT");	   	
			   }
			}
        }


		sendtomainui->BIT_INFO = 0x00;
		
		return 0;
err_create:
	 return -1;
}


int kcppk_kernel_open (struct inode *inode, struct file *filp)
{
	if(Boot_complete_hsc) {
		Boot_complete_hsc = false;
     	Boot_Completed_timer_init();  	
		Send_Comb_key_timer_init();
		if(KThread_FBC_CHECK_ing == false) {
	        wake_up_interruptible(&wait_FBC_START_event);			
		}
	}
	return 0;
}

int kcppk_kernel_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t kcppk_kernel_write(struct file *filp, const char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	*f_pos = count;
	return count;
}

long kcppk_kernel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	ret = 0;
	return ret;
}

ssize_t kcppk_kernel_read(struct file *filp,char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	size_t current_len;
	if(*f_pos > 0) return 0; 
	memcpy(inform_to_java,Default_inform_date,30);	
	wait_kcppk_kernel = true;
  	interruptible_sleep_on(&wait_inform_to_java);
	wait_kcppk_kernel = false;
	current_len = strlen(inform_to_java);
	if(copy_to_user(buf,inform_to_java,current_len) != 0) {		
	   printk(KERN_ERR "Error : copy to user !!\n");
	   return -EFAULT;
	} 
	return current_len;
}

struct file_operations kcppk_kernel_fops = {
   	.owner 	= THIS_MODULE,
	.open 	= kcppk_kernel_open,
	.write  = kcppk_kernel_write,
	.read   = kcppk_kernel_read,
	.unlocked_ioctl  = kcppk_kernel_ioctl,
	.release	= kcppk_kernel_release,
};

struct cdev kcppk_kernel_char_cdev = {
	.kobj = {
			.name="kcppk_kernel",
		},
	.owner = THIS_MODULE,
	.ops = &kcppk_kernel_fops,
};

struct miscdevice kcppk_kernel_misc_device = {
		.minor = 0,
		.name = "kcppk_kernel",
		.fops = &kcppk_kernel_fops,
		.mode = (S_IRUGO | S_IRWXUGO | S_IALLUGO),
};

int kernel_device_create(void) {
	   	dev_t dev;
		int err = 0;	
	    struct class *cl;
	    struct miscdevice *misd;	
		misd = &kcppk_kernel_misc_device;
	    dev = MKDEV(Kcppk_Char_Major, Kcppk_Char_Dev_3);
	    err = register_chrdev_region(dev, 1, SOAP_Monitor_DEV_NAME);

	    cdev_init(&kcppk_kernel_char_cdev, &kcppk_kernel_fops);
	    kcppk_kernel_char_cdev.owner = THIS_MODULE;
	    kcppk_kernel_char_cdev.ops  = &kcppk_kernel_fops;

	    cl = class_create(THIS_MODULE, "chardrv_kcppk_kernel");
	    device_create(cl, NULL, dev, NULL, SOAP_Monitor_DEV_NAME);

	    if(cdev_add(&kcppk_kernel_char_cdev, dev, 1)) {
	        printk(KERN_INFO "/dev/%s : cdev creation failed.\n",SOAP_Monitor_DEV_NAME);
			goto err_create_1;
	    }
	    printk(KERN_INFO "/dev/%s: cdev creation Ok.\n",SOAP_Monitor_DEV_NAME);
		return 0;
 err_create_1:
 	    return -1;
}
#ifdef LED_PWM_FUNC_ENABLE
int kcppk_led_br_open (struct inode *inode, struct file *filp)
{

	return 0;
}

int kcppk_led_br_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t kcppk_led_br_write(struct file *filp, const char __user *buf, size_t size,
	                         loff_t *f_pos)
{
	struct platform_device *pdev = get_bt_let_pdev();		
	struct led_classdev *led_cdev = dev_get_drvdata(&pdev->dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long brightness = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

       set_led_pwm(led_cdev, brightness);
	}

	return ret;
}

long kcppk_led_br_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	ret = 0;
	return ret;
}

ssize_t kcppk_led_br_read(struct file *filp,char __user *buf, size_t count,
	                         loff_t *f_pos)
{
	*f_pos = count;
	return count;
}

struct file_operations kcppk_led_br_fops = {
   	.owner 	= THIS_MODULE,
	.open 	= kcppk_led_br_open,
	.write  = kcppk_led_br_write,

	.release	= kcppk_led_br_release,
};

struct cdev kcppk_led_br_char_cdev = {
	.kobj = {
			.name="kcppk_led_br",
		},
	.owner = THIS_MODULE,
	.ops = &kcppk_led_br_fops,
};

struct miscdevice kcppk_led_br_misc_device = {
		.minor = 0,
		.name = "kcppk_led_br",
		.fops = &kcppk_led_br_fops,
		.mode = (S_IRUGO | S_IRWXUGO | S_IALLUGO),
};

int led_br_device_create(void) {
	   	dev_t dev;
		int err = 0;	
	    struct class *cl;
	    struct miscdevice *misd;	
		misd = &kcppk_led_br_misc_device;
	    dev = MKDEV(Kcppk_Char_Major, Kcppk_Char_Dev_1);
	    err = register_chrdev_region(dev, 1, SOAP_LED_BR_DEV_NAME);

	    cdev_init(&kcppk_led_br_char_cdev, &kcppk_led_br_fops);
	    kcppk_led_br_char_cdev.owner = THIS_MODULE;
	    kcppk_led_br_char_cdev.ops  = &kcppk_led_br_fops;

	    cl = class_create(THIS_MODULE, "chardrv_kcppk_led_br");
	    device_create(cl, NULL, dev, NULL, SOAP_LED_BR_DEV_NAME);

	    if(cdev_add(&kcppk_led_br_char_cdev, dev, 1)) {
	        printk(KERN_INFO "/dev/%s : cdev creation failed.\n",SOAP_LED_BR_DEV_NAME);
			goto err_create_1;
	    }
	    printk(KERN_INFO "/dev/%s: cdev creation Ok.\n",SOAP_LED_BR_DEV_NAME);
		return 0;
 err_create_1:
 	    return -1;
}
#endif
int GPIO_CONFIGURE_FOR_KCPPK() {
    int ret = 0;
    Bit_gpio_input_check_Rtn();
    Bit_gpio_check_Rtn();
	Bit_gpio_check2_Rtn();
    return ret;
}

int GPIO_CONFIGURE_FOR_KCPPK_2() {
    int ret = 0;
    ret = kcppk_buttons_gpio_init();
	if(ret < 0) {
		    printk(KERN_ERR " Error : kcppk_buttons_gpio_init() !!! \n");
	}	
    return ret;
}

int Interrupt_CONFIGURE_FOR_KCPPK(){
	int ret;
    int i = 0;
	while(i < mfd_buttons_pd_info.counts) {
       if(mfd_buttons[i].active_high)
       {
	   ret = request_irq(mfd_buttons[i].irq, dummy_kcppk_interrupt_handler,
	   	                          (IRQF_TRIGGER_RISING|IRQF_ONESHOT),
	   	                          mfd_buttons[i].name, mfd_buttons);
       } else {
	   ret = request_irq(mfd_buttons[i].irq, dummy_kcppk_interrupt_handler,
	   	                          (IRQF_TRIGGER_FALLING|IRQF_ONESHOT),
	   	                          mfd_buttons[i].name, mfd_buttons);  
       }
		if (ret) {
			return ret;
		}
		i++;
	}
	return ret;
}
int kcppk_buttons_probe(struct platform_device *pd)
{
    
    struct skcppk_button_switch *kcppk_buttonsw = kcppk_button_switch;
//	struct mfd_buttons_platform_data_info *mfd_buttons_info;
    int i = 0;
    int ret = 0;
	
//button led blinking	
    button_led_toggle_timer_init();
    msleep(500);
    ret = buttons_device_create();
	if(ret < 0) {
		    printk(KERN_ERR " Error : buttons_device_create() !!! \n");
	}
	ret = kernel_device_create();
	if(ret < 0) {
		    printk(KERN_ERR " Error : kernel_device_create() !!! \n");
	}	
#ifdef LED_PWM_FUNC_ENABLE
	ret = led_br_device_create();
	if(ret < 0) {
		    printk(KERN_ERR " Error : led_br_device_create() !!! \n");
	}		
#endif

    GPIO_CONFIGURE_FOR_KCPPK();

	if(mfd_buttons == NULL) {
	    mfd_buttons = kzalloc(sizeof(struct mfd_buttons_platform_data) * (mfd_buttons_pd_info.counts), GFP_KERNEL);
		if (mfd_buttons == NULL) {
			return -ENOMEM;
			printk(KERN_ERR " Can't allocate mfd_buttons memory space !!! \n");
		}
	}

	while(i < mfd_buttons_pd_info.counts) {
		mfd_buttons[i].name = mfd_buttons_pd[i].name;
		mfd_buttons[i].wakeup = mfd_buttons_pd[i].wakeup;
		mfd_buttons[i].active_high= mfd_buttons_pd[i].active_high;		
		mfd_buttons[i].irq = gpio_to_irq(mfd_buttons_pd[i].irq);
		mfd_buttons[i].gpio_num = kcppk_buttonsw[i].gpio_num;
		mfd_buttons[i].check_time = kcppk_buttonsw[i].check_time;
		mfd_buttons[i].two_step= kcppk_buttonsw[i].two_step;
		mfd_buttons[i].error_count = 0;
		mfd_buttons[i].error_occurs = false;		
		if(mfd_buttons[i].two_step) mfd_buttons[i].check_time2 = kcppk_buttonsw[i].check_time2;		
		if(Debug_MSG) printk(KERN_INFO "======mfd_buttons[i].name : %s  ====irq : %d ,  gpio : %d ======= \n",mfd_buttons[i].name, mfd_buttons[i].irq,mfd_buttons[i].gpio_num);
		i++;
	}

    ret = Interrupt_CONFIGURE_FOR_KCPPK();
	if(ret < 0) printk(KERN_ERR " Can't allocate request irq for %s memory space !!! \n",mfd_buttons[i].name );

	if(WAKE_LOCK_ENABLE) wake_lock(&twakelock.wake_lock);

    GPIO_CONFIGURE_FOR_KCPPK_2();

	if(Button_CBIT == NULL){ 
	   Button_CBIT = (struct task_struct *)kthread_run(kthread_FBC_CHECK, NULL, "kthread_FBC_CHECK");
       if(Button_CBIT == NULL) {
    	printk(KERN_ERR "Memory Allocation Error : %s buffer\n","kthread_FBC_CHECK");			   	
	    } else
	   {
    	printk(KERN_INFO "Memory Allocation OK.. : %s \n","kthread_FBC_CHECK");	   	
	   }
	}


	
	return 0;
}

int kcppk_buttons_remove(struct platform_device *pd) {
	//Can't remove button driver..
	return 0;
}

struct platform_driver kcppk_buttons_driver = {
	.driver = {
		   .name = "kcppk_buttons_drv",
		   .owner = THIS_MODULE,
	},
	.probe = kcppk_buttons_probe,
	.remove = kcppk_buttons_remove,
};

int __init kcppk_buttons_init(void)
{
	printk(KERN_INFO"------------  %s  -------------\n",__func__);
	init_timer(&Boot_Completed_timer);
	init_timer(&button_led_toggle);		
	init_timer(&Send_Comb_key_timer);	
	wake_lock_init(&twakelock.wake_lock, WAKE_LOCK_SUSPEND,
			 "kcppk_WakeLock");
	return platform_driver_register(&kcppk_buttons_driver);
}

subsys_initcall(kcppk_buttons_init);

static void __exit kcppk_buttons_exit(void)
{
	platform_driver_unregister(&kcppk_buttons_driver);
}
module_exit(kcppk_buttons_exit);

MODULE_AUTHOR(RABBIT "rabbit-girl@uniquenet.co.kr>");
MODULE_DESCRIPTION("MY Second virtual Driver");
