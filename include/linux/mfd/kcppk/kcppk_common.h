/*
 * kcppk_common.h
 *
 *
 */

#ifndef __LINUX_MFD_KCPPK_SOAP_COMMON_H
#define __LINUX_MFD_KCPPK_SOAP_COMMON_H

#define Kcppk_Char_Major 241
#define Kcppk_Char_Dev_1 1 // Front_LED_Brightness
#define Kcppk_Char_Dev_2 2 // IPC_UI_HAL (Between kernel and Main-UI)
#define Kcppk_Char_Dev_3 3 // kcppk_kernel (Between kernel and Android Service)
#define Kcppk_Char_Dev_4 4 // kcppk_sensor (Between kernel and Android Service)

#define BIT_CHECK 80
#define IBIT_CHECK 81
/*
// 0x80 : BIT_OK
// bit#0 : Low Power Alert
// bit#1 : FCT Data Input Error
// bit#2 : ADV7280 Overflow Error Occurs
// bit#3 : ADV7280 Decoder initialize Error
// bit#4 : Heating cable status 
// bit#5 : Kcppk Display Front Button function status
*/
#define BIT_STATUS_GOOD   80
#define BIT_LOW_POWER 0
#define BIT_ADV7280_OV 2
#define BIT_ADV7280 3
#define BIT_HEATING 4
#define BIT_F_BUTTON 5
#define BIT_NMI 7

#define BIT_OK 0x00
#define BIT_ERROR 0x01


struct mfd_wakeup_platform_data
{
       const  char   *  name;
       int            wakeup;
       int            irq;
	   bool           active_high;
       int            event_occurs;
};

struct mfd_buttons_platform_data
{
       const  char   *  name;
       int            wakeup;
       int            irq;
	   bool           active_high;
       int            event_occurs;
	   int            gpio_num;
	   int            check_time;
	   bool           two_step;
	   int            check_time2;
	   int            error_count;
	   bool           error_occurs;
};

struct mfd_wakeup_platform_data_info
{
       struct mfd_wakeup_platform_data *wake_device;
       int            counts;
};

struct mfd_buttons_platform_data_info
{
       struct mfd_buttons_platform_data *button_device;
       int            counts;
};

struct mfd_wakeup_device
{
       bool          wakeup;
       int           irq;
};

extern void SOAP_SENSOR_ENABLE(bool on);
extern void BIT_STATUS(u8 value, u8 setvalue);
extern struct platform_device *get_bt_let_pdev();
#endif /*  */
