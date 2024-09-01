/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidkbd.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef HIDKBD_H
#define HIDKBD_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */




// Task Events
#define START_DEVICE_EVT          0x0001
#define START_REPORT_EVT          0x0002
#define START_PARAM_UPDATE_EVT    0x0004
#define START_PHY_UPDATE_EVT      0x0008

#define FLASH_POWER_LED_EVT			  0x0010
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#define MODE_FORWARD 0
#define MODE_BACKWARD 1

#define BUTTON_PIN GPIO_Pin_15  // PB15
#define BUTTON_SELECT_PIN GPIO_Pin_4

#define USB_CONNECTED_PIN GPIO_Pin_14

#define LED_PIN				GPIO_Pin_7      // PB7
#define LED_FORWARD_PIN		GPIO_Pin_15 //PA15
#define LED_BACKWARD_PIN	GPIO_Pin_8 // PA8

#define LED_MIDDLE_LEFT_PIN     GPIO_Pin_13 // PA13
#define LED_MIDDLE_RIGHT_PIN    GPIO_Pin_14 // PA13

		 // On clignote à 2Hz
 #define DEBOUNCE_TIME_MS  50  // Total debounce time in milliseconds
 #define CHECKS_COUNT   10  // Number of checks within the debounce period

 #define DEBOUNCE_TICKS    ((DEBOUNCE_TIME_MS / CHECKS_COUNT) * (FREQ_SYS / 1000))  // Ticks per check interval


#define HID_KEY_ARROW_RIGHT 0x4F
#define HID_KEY_ARROW_LEFT 0x50
#define HID_KEY_ARROW_DOWN 0x51
#define HID_KEY_ARROW_UP 0x52
#define HID_KEY_VOLUME_UP 0x52
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Task Initialization for the BLE Application
 */
extern void HidEmu_Init(void);

/*
 * Task Event Processor for the BLE Application
 */
extern uint16_t HidEmu_ProcessEvent(uint8_t task_id, uint16_t events);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
