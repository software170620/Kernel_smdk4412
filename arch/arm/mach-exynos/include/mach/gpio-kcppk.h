/* linux/arch/arm/mach-exynos/include/mach/gpio-b2cs.h
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - MIDAS GPIO lib
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_GPIO_KCPPK_H
#define __ASM_ARCH_GPIO_KCPPK_H __FILE__
#include <mach/gpio.h>

#define GPIO_5V_EN_H				EXYNOS4_GPX0(3) 

#define GPIO_GPS_nRST_H				EXYNOS4_GPX0(7) 

#define GPIO_POWER_KEY				EXYNOS4_GPX1(3)//EXYNOS4_GPX3(3)// pull-up register required...

#define GPIO_UART2_DETECT           EXYNOS4_GPX2(7)

#define GPIO_PMIC_INT				EXYNOS4_GPX3(2)
#define GPIO_PMIC_IRQ				IRQ_EINT(26)
#define GPIO_PMIC_BUCK_EN_H			EXYNOS4_GPY2(0)
#define GPIO_PMIC_LDO18_BUCK9_EN_H	EXYNOS4_GPK0(2)
#define GPIO_PMIC_LDO23_4_EN_H		EXYNOS4_GPK0(2)
#define GPIO_PMIC_DS2				EXYNOS4_GPL0(0)
#define GPIO_PMIC_DS3				EXYNOS4_GPL0(1)
#define GPIO_PMIC_DS4				EXYNOS4_GPL0(2)
#define GPIO_PMIC_DVS1				EXYNOS4_GPL0(3)
#define GPIO_PMIC_DVS2				EXYNOS4_GPL0(4)
#define GPIO_PMIC_DVS3				EXYNOS4_GPL0(6)

#define GPIO_HDMI_CEC				EXYNOS4_GPX3(6)
#define GPIO_HDMI_DETECT			EXYNOS4_GPX3(7)
#define GPIO_HDMI_nRST_L			EXYNOS4212_GPM4(1)

#define GPIO_LCD_PWM				EXYNOS4_GPD0(1)
#define GPIO_UART0_RXD				EXYNOS4_GPA0(0)
#define GPIO_CAM_MCLK				EXYNOS4212_GPJ1(3)
#define GPIO_CAM_nRST_L				EXYNOS4212_GPM4(4)
#define GPIO_CAM_POWER_DOWN_H		EXYNOS4212_GPM4(5)

#define GPIO_OTG_EN_H		EXYNOS4_GPL2(3)

//light sensor
#define GPIO_SI1133_INT 	 EXYNOS4_GPX3(4)  //XEINT28

#define LCD_POWER_UPDOWN     EXYNOS4_GPX2(3) //XEINT19 
#define HTC_CAM_POWER        EXYNOS4_GPX1(5) //XEINT13

#define BUTTON_LED_ON        EXYNOS4_GPX3(0) //XEINT24
#define BUTTON_LED_PWM       EXYNOS4_GPD0(0)

#define LOW_POWER_ALERT      EXYNOS4_GPX3(1) //XEINT25

#define LOW_POWER_CBIT_ALERT EXYNOS4_GPX0(0) //XEINT0

#define HEATING_ALERT        EXYNOS4_GPX2(1) //XEINT17

#define Kcppk_Power_BTN     EXYNOS4_GPX3(3) //XEINT27
#define Kcppk_Power_IRQ     EXYNOS4_GPX2(5) //XEINT21

#define Kcppk_LCD_BTN       EXYNOS4212_GPM4(2) //CAM_GPIO12
#define Kcppk_LCD_IRQ       EXYNOS4_GPX2(0) //XEINT16

#define Kcppk_MENU_BTN      EXYNOS4212_GPM4(3) //CAM_GPIO13
#define Kcppk_MENU_IRQ      EXYNOS4_GPX1(7) //XEINT15 

#define Kcppk_UP_BTN        EXYNOS4212_GPM4(5) //CAM_GPIO15 
#define Kcppk_UP_IRQ        EXYNOS4_GPX1(1) //XEINT9

#define Kcppk_DOWN_BTN      EXYNOS4212_GPM4(6) //CAM_GPIO16  //LCD ON_OFF gpio287
#define Kcppk_DOWN_IRQ      EXYNOS4_GPX1(0) //XEINT8 

#define Kcppk_SELECT_BTN    EXYNOS4_GPX1(6) //XEINT14
#define Kcppk_SELECT_IRQ    EXYNOS4_GPX0(6) //XEINT6

#define Kcppk_CALIB_BTN     EXYNOS4_GPL2(6) //XGNSS_GPIO_6
#define Kcppk_CALIB_IRQ     EXYNOS4_GPX1(2) //XEINT10 

#define Kcppk_WB_BTN        EXYNOS4_GPL2(2) //XGNSS_GPIO_2
#define Kcppk_WB_IRQ        EXYNOS4_GPX2(2) //XEINT18 

#define Kcppk_VIEW_BTN      EXYNOS4212_GPM4(7) //CAM_GPIO17  
#define Kcppk_VIEW_IRQ      EXYNOS4_GPX2(4) //XEINT20 

#endif /* __ASM_ARCH_GPIO_KCPPK_H */
