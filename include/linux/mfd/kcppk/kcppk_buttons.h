/*
 * kcppk_buttons.h
 *
 *
 */

#ifndef __LINUX_MFD_KCPPK_SOAP_BUTTONS_H
#define __LINUX_MFD_KCPPK_SOAP_BUTTONS_H

/*  Button value to Main UI */
#define LCD_ON_OFF 0
#define POWER_BTN  1
#define MENU_BTN   2
#define UP_KEY_BTN 3
#define DW_KEY_BTN 4
#define SELECT_BTN 5
#define CAM_CAL_BTN 6
#define CAM_WB_BTN 7
#define CAM_VW_BTN 8
#define CAM_ONOFF_BTN 9
#define HOME_BTN 10

#define LOW_P_ALERT 20
#define LOW_BIT_ALERT 21  //add one more.. low power alert for C-Bit
#define RESERVED_VALUE 255

/*  Main UI to button_driver */
#define KCPPK_LCD 0
#define KCPPK_BRT_AUTO 1
// value 2 fail..
#define ANI_START 3
#define ANI_STOP 4
#define CAMERA_CTRL 5
#define LIGHT_SENSOR_ON 6
#define LIGHT_SENSOR_OFF 7
#define KCPPK_REBOOT_CMD 8
#define KCPPK_POWER_OFF 9
#define BUTTON_LED_TOOGLE_STOP 10


#define KCPPK_CMD_ON 1
#define KCPPK_CMD_OFF 2

/* Two Step button function */
#define ONE_STEP 0
#define TWO_STEP 1

#define DATA_SIZE_TO_MAIN_UI 8



#endif /*  */
