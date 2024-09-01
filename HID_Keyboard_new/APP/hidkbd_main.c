/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 蓝牙键盘应用主函数及任务系统初始化
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "HAL.h"
#include "hiddev.h"
#include "hidkbd.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   主循环
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
void Main_Circulation()
{
    while(1)
    {
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */


int main(void)
{
#if(defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    SetSysClock(CLK_SOURCE_PLL_60MHz);
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif

// set PB12 low, for restance divider for measuring batt voltage
GPIOB_ResetBits(GPIO_Pin_12);
GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA);
GPIOB_ResetBits(GPIO_Pin_12);

// init ADC on PA12
GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_Floating);
ADC_ExtSingleChSampInit(SampleFreq_3_2, ADC_PGA_1_4);
//  ADC channel 2

GPIOB_ModeCfg(BUTTON_PIN, GPIO_ModeIN_PU);
GPIOB_ITModeCfg(BUTTON_PIN, GPIO_ITMode_FallEdge);
GPIOB_ClearITFlagBit(BUTTON_PIN);
PFIC_EnableIRQ(GPIO_B_IRQn);


GPIOB_ModeCfg(BUTTON_SELECT_PIN, GPIO_ModeIN_PU);
GPIOB_ITModeCfg(BUTTON_SELECT_PIN, GPIO_ITMode_FallEdge);
GPIOB_ClearITFlagBit(BUTTON_SELECT_PIN);
PFIC_EnableIRQ(GPIO_B_IRQn);

// check usb charging
GPIOB_ModeCfg(USB_CONNECTED_PIN, GPIO_ModeIN_PD);

//GPIOB_ITModeCfg(USB_CONNECTED_PIN, GPIO_ITMode_FallEdge);


GPIOB_SetBits(LED_PIN);
GPIOB_ModeCfg(LED_PIN, GPIO_ModeOut_PP_5mA);

GPIOA_SetBits(LED_FORWARD_PIN);
GPIOA_ModeCfg(LED_FORWARD_PIN, GPIO_ModeOut_PP_5mA);

GPIOA_SetBits(LED_BACKWARD_PIN);
GPIOA_ModeCfg(LED_BACKWARD_PIN, GPIO_ModeOut_PP_5mA);

GPIOA_SetBits(LED_MIDDLE_LEFT_PIN);
GPIOA_ModeCfg(LED_MIDDLE_LEFT_PIN, GPIO_ModeOut_PP_5mA);
GPIOA_SetBits(LED_MIDDLE_RIGHT_PIN);
GPIOA_ModeCfg(LED_MIDDLE_RIGHT_PIN, GPIO_ModeOut_PP_5mA);

unsigned int leds[]={LED_PIN,LED_BACKWARD_PIN,LED_MIDDLE_LEFT_PIN,LED_MIDDLE_RIGHT_PIN,LED_FORWARD_PIN};
for (int j=0; j<3; j++) {
	for (int i=0; i<5; i++) {

		if (i==0) {
			GPIOB_ResetBits(leds[i]);
			DelayMs(100);
			GPIOB_SetBits(leds[i]);
			//DelayMs(50);

		} else {
			GPIOA_ResetBits(leds[i]);
			DelayMs(100);
			GPIOA_SetBits(leds[i]);
			//DelayMs(50);
		}
	}
}

DelayMs(500);
GPIOA_SetBits(LED_FORWARD_PIN);
GPIOA_ResetBits(LED_BACKWARD_PIN);

TMR0_TimerInit(DEBOUNCE_TICKS);
TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
PFIC_EnableIRQ(TMR0_IRQn);

#define DEBUG 1
#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif
    PRINT("%s\n", VER_LIB);
    PRINT("hallo van de uart");
    CH58X_BLEInit();
    HAL_Init();
    GAPRole_PeripheralInit();
    HidDev_Init();
    HidEmu_Init();

    //tmos_start_task(hidEmuTaskId, FLASH_POWER_LED_EVT, 2000);

    Main_Circulation();
}

/******************************** endfile @ main ******************************/
