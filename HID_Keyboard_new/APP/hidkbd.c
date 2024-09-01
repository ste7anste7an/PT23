/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidkbd.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        : À¶ÑÀ¼üÅÌÓ¦ÓÃ³ÌÐò£¬³õÊ¼»¯¹ã²¥Á¬½Ó²ÎÊý£¬È»ºó¹ã²¥£¬Ö±ÖÁÁ¬½ÓÖ÷»úºó£¬¶¨Ê±ÉÏ´«¼üÖµ
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hidkbdservice.h"
#include "hiddev.h"
#include "hidkbd.h"
#include <CH58x_common.h>
#include <stdbool.h>

/*********************************************************************
 * MACROS
 */
// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN              8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN                  1

/*********************************************************************
 * CONSTANTS
 */
// Param update delay
#define START_PARAM_UPDATE_EVT_DELAY         12800

// Param update delay
#define START_PHY_UPDATE_DELAY               1600

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT             60000

// Minimum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    8

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL    8

// Slave latency to use if parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
#define DEFAULT_DESIRED_CONN_TIMEOUT         500

// Default passcode
#define DEFAULT_PASSCODE                     0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                 GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                    FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                 TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES              GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL          6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
static uint8_t hidEmuTaskId = INVALID_TASK_ID;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */



uint16_t global_adc=0;
uint16_t check_bat_counter=50; // start seting global_adc

uint16_t check_low_battery(void) {

	check_bat_counter++;
	//printf("check_bat_counter %d\n\r",check_bat_counter);
	if (check_bat_counter>20) {
		// check USB
		uint16_t adc;
		uint8_t  percent;
		uint16_t RoughCalib_Value = ADC_DataCalib_Rough(); // ╙├╙┌╝╞╦πADC─┌▓┐╞½▓εú¼╝╟┬╝╡╜╚½╛╓▒Σ┴┐ RoughCalib_Value╓╨
		ADC_ChannelCfg(2);
		uint16_t batlev= ADC_ExcutSingleConver();
		adc = batlev+ RoughCalib_Value;
		printf("check bat: adc = %d low %d\r\n",batlev+ RoughCalib_Value,(adc < 2460));
		check_bat_counter=0;
		global_adc=adc;
		if (GPIOB_ReadPortPin(USB_CONNECTED_PIN) == 0) {
			printf("battery operation\r\n");
			//return 0;
		} else {
			printf("USB charging\r\n");
			if (global_adc<2600){
				GPIOA_ResetBits(LED_PIN);
			    return 1;
			} else {
				GPIOA_SetBits(LED_PIN);
			    return 0;
			}
		}


		tmos_start_task(hidEmuTaskId, FLASH_POWER_LED_EVT, 500);

	}
	return (global_adc < 2480);
}


volatile int cur_mode=MODE_FORWARD;
 volatile bool keyPressed = false;
 volatile bool keyPressed_select = false;
 volatile bool debounceActive = false;
 volatile bool debounceActive_select = false;
 volatile unsigned int stableCount = 0;  // Count for stable readings
 volatile unsigned int stableCount_select = 0;  // Count for stable readings
 volatile unsigned int tries = 0;  // Count for stable readings
 volatile unsigned int tries_select = 0;  // Count for stable readings
 __INTERRUPT
 __HIGH_CODE
 void TMR0_IRQHandler() {
 	if (TMR0_GetITFlag(TMR0_3_IT_CYC_END)) {
 		TMR0_ClearITFlag(TMR0_3_IT_CYC_END);

 		if (debounceActive) {
 			if (GPIOB_ReadPortPin(BUTTON_PIN) == 0) {
 						stableCount++;  // Increase count if button is still pressed
 					} else {
 						stableCount = 0;  // Reset count if button state is unstable
 						tries++;
 					}

 			if (stableCount >= CHECKS_COUNT) {
 					PRINT("count > CHECK_COUNT: %d tries %d \r\n",stableCount,tries);
 					tries=0;
 					keyPressed = true;  // Set the key pressed flag
 					GPIOA_InverseBits(LED_MIDDLE_RIGHT_PIN);
 					tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 20);

 					debounceActive = false;  // Stop debounce process

 					//TMR0_Disable() ;
 				}
 		}

 		if (debounceActive_select) {
 				if (GPIOB_ReadPortPin(BUTTON_SELECT_PIN) == 0) {
 				            stableCount_select++;  // Increase count if button is still pressed
 				        } else {
 				            stableCount_select = 0;  // Reset count if button state is unstable
 				            tries_select++;
 				        }

 			    if (stableCount_select >= CHECKS_COUNT) {
 			    		PRINT("count_select > CHECK_COUNT: %d tries %d \r\n",stableCount_select,tries_select);
 			    		tries_select=0;
 			            keyPressed_select = true;  // Set the key pressed flag

 			          	if (cur_mode==MODE_FORWARD)
 			       			{cur_mode=MODE_BACKWARD;
 			          		//tmos_start_task(hidEmuTaskId, FLASH_POWER_LED_EVT, 500);
 			       			}
 			       		else
 			       		    cur_mode=MODE_FORWARD;
 			       		if (cur_mode==MODE_FORWARD) {
 			       			GPIOA_SetBits(LED_FORWARD_PIN);
 			       			GPIOA_ResetBits(LED_BACKWARD_PIN);
 			       		} else {
 			       			GPIOA_ResetBits(LED_FORWARD_PIN);
 			       			GPIOA_SetBits(LED_BACKWARD_PIN);

 			       		}
 			       		PRINT("cur_mode = %d\r\n",cur_mode);


 			            debounceActive_select = false;  // Stop debounce process
 			            //TMR0_Disable() ;
 			        }
 				}


 	}

 	if (debounceActive || debounceActive_select) {
 		TMR0_Enable();
 	} else {
 		TMR0_Disable();
 	}
 }

 __INTERRUPT
 __HIGH_CODE
 void GPIOB_IRQHandler() {
 	if (GPIOB_ReadITFlagBit(BUTTON_PIN)) {
 		GPIOB_ClearITFlagBit(BUTTON_PIN);
 		PRINT(".");
 		//PRINT("Timer = %d\r\n",TMR0_GetCurrentTimer());
 		 if (!debounceActive) {
 		        debounceActive = true;  // Start debounce process
 		        stableCount = 0;  // Reset the stable count
 		        //TMR0_TimerInit(DEBOUNCE_TICKS);  // Initialize (start) the timer with debounce ticks
 		        TMR0_Enable();
 		        TMR0_ClearITFlag(TMR0_3_IT_CYC_END);


 		 }

 	}

 	if (GPIOB_ReadITFlagBit(BUTTON_SELECT_PIN)) {
 		GPIOB_ClearITFlagBit(BUTTON_SELECT_PIN);
 		PRINT("-");
 		//PRINT("Timer = %d\r\n",TMR0_GetCurrentTimer());
 		 if (!debounceActive_select) {
 		        debounceActive_select = true;  // Start debounce process
 		        stableCount_select = 0;  // Reset the stable count
 		        //TMR0_TimerInit(DEBOUNCE_TICKS);  // Initialize (start) the timer with debounce ticks
 		        TMR0_Enable();
 		        TMR0_ClearITFlag(TMR0_3_IT_CYC_END);


 		 }

 	}
 }



/*********************************************************************
 * LOCAL VARIABLES
 */



// GAP Profile - Name attribute for SCAN RSP data
static uint8_t scanRspData[] = {
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // service UUIDs
    0x05, // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// Advertising data
static uint8_t advertData[] = {
    // flags
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03, // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
    HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

    0x0D,                           // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE, // AD Type = Complete local name
    'P',
    'a',
    'g',
    'e',
    ' ',
    'T',
    'u',
    'r',
    'n',
    'e',
    'r',
    ' ',  // connection interval range
};

// Device name attribute value
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Page Turner";

// HID Dev configuration
static hidDevCfg_t hidEmuCfg = {
    DEFAULT_HID_IDLE_TIMEOUT, // Idle timeout
    HID_FEATURE_FLAGS         // HID feature flags
};

static uint16_t hidEmuConnHandle = GAP_CONNHANDLE_INIT;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void    hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void    hidEmuSendKbdReport(uint8_t keycode);
static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData);
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void    hidEmuEvtCB(uint8_t evt);
static void    hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuHidCBs = {
    hidEmuRptCB,
    hidEmuEvtCB,
    NULL,
    hidEmuStateCB};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmu_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HidEmu_Init()
{
    hidEmuTaskId = TMOS_ProcessEventRegister(HidEmu_ProcessEvent);

    // Setup the GAP Peripheral Role Profile
    {
        uint8_t initial_advertising_enable = TRUE;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);

        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    }

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), (void *)attDeviceName);

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = DEFAULT_PASSCODE;
        uint8_t  pairMode = DEFAULT_PAIRING_MODE;
        uint8_t  mitm = DEFAULT_MITM_MODE;
        uint8_t  ioCap = DEFAULT_IO_CAPABILITIES;
        uint8_t  bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Setup Battery Characteristic Values
    {
        uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;
        Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t), &critical);
    }

    // Set up HID keyboard service
    Hid_AddService();

    // Register for HID Dev callback
    HidDev_Register(&hidEmuCfg, &hidEmuHidCBs);

    // Setup a delayed profile startup
    tmos_set_event(hidEmuTaskId, START_DEVICE_EVT);

    // enable power led alarm monitor
    tmos_set_event(hidEmuTaskId, FLASH_POWER_LED_EVT);
}

/*********************************************************************
 * @fn      HidEmu_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t HidEmu_ProcessEvent(uint8_t task_id, uint16_t events)
{

    static uint8_t send_char;

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(hidEmuTaskId)) != NULL)
        {
            hidEmu_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & START_DEVICE_EVT)
    {
        return (events ^ START_DEVICE_EVT);
    }

    if(events & START_PARAM_UPDATE_EVT)
    {
        // Send connect param update request
        GAPRole_PeripheralConnParamUpdateReq(hidEmuConnHandle,
                                             DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                             DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                             DEFAULT_DESIRED_SLAVE_LATENCY,
                                             DEFAULT_DESIRED_CONN_TIMEOUT,
                                             hidEmuTaskId);

        return (events ^ START_PARAM_UPDATE_EVT);
    }

    if(events & START_PHY_UPDATE_EVT)
    {
        // start phy update
        PRINT("Send Phy Update %x...\r\n", GAPRole_UpdatePHY(hidEmuConnHandle, 0,
                    GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));

        return (events ^ START_PHY_UPDATE_EVT);
    }

    if(events & START_REPORT_EVT)
    {
    	//if (buttonPressed == TRUE) {
    		if (cur_mode==MODE_FORWARD) {
    			send_char = HID_KEY_ARROW_RIGHT; //4
    		}
    		else if (cur_mode==MODE_BACKWARD) {
    			send_char = HID_KEY_ARROW_LEFT; //5
    		}
    		hidEmuSendKbdReport(send_char);
    		PRINT("Send KeyCode %02x\r\n",send_char);
    		//buttonPressed = FALSE;
    	    //send_char++;
            //if(send_char >= 0x53)
            //   send_char = 0x4f;
            hidEmuSendKbdReport(0x00);
    	//}
        // tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 2000);
        return (events ^ START_REPORT_EVT);
    }

    if(events & FLASH_POWER_LED_EVT)
       {
    		if (check_low_battery()) {
    			GPIOA_InverseBits(LED_PIN);
    			//printf("flash power\r\n");
    		} else {
    			GPIOA_SetBits(LED_PIN); // switch led off
    		}
    		// ost new event for next 500ms
            tmos_start_task(hidEmuTaskId, FLASH_POWER_LED_EVT, 500);
            return (events ^ FLASH_POWER_LED_EVT);
       }

    return 0;
}

/*********************************************************************
 * @fn      hidEmu_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidEmu_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuSendKbdReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidEmuSendKbdReport(uint8_t keycode)
{
    uint8_t buf[HID_KEYBOARD_IN_RPT_LEN];

    buf[0] = 0;       // Modifier keys
    buf[1] = 0;       // Reserved
    buf[2] = keycode; // Keycode 1
    buf[3] = 0;       // Keycode 2
    buf[4] = 0;       // Keycode 3
    buf[5] = 0;       // Keycode 4
    buf[6] = 0;       // Keycode 5
    buf[7] = 0;       // Keycode 6

    HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                  HID_KEYBOARD_IN_RPT_LEN, buf);
}

/*********************************************************************
 * @fn      hidEmuStateCB
 *
 * @brief   GAP state change callback.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void hidEmuStateCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
        {
            uint8_t ownAddr[6];
            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddr);
            GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, ownAddr);
            PRINT("Initialized..\r\n");
        }
        break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Advertising..\r\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

                // get connection handle
                hidEmuConnHandle = event->connectionHandle;
                tmos_start_task(hidEmuTaskId, START_PARAM_UPDATE_EVT, START_PARAM_UPDATE_EVT_DELAY);
                PRINT("Connected..\r\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Connected Advertising..\r\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                PRINT("Waiting for advertising..\r\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                PRINT("Disconnected.. Reason:%x\r\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                PRINT("Advertising timeout..\r\n");
            }
            // Enable advertising
            {
                uint8_t initial_advertising_enable = TRUE;
                // Set the GAP Role Parameters
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
            }
            break;

        case GAPROLE_ERROR:
            PRINT("Error %x ..\r\n", pEvent->gap.opcode);
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      hidEmuRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8_t hidEmuRcvReport(uint8_t len, uint8_t *pData)
{
    // verify data length
    if(len == HID_LED_OUT_RPT_LEN)
    {
        // set LEDs
        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
    }
}

/*********************************************************************
 * @fn      hidEmuRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8_t hidEmuRptCB(uint8_t id, uint8_t type, uint16_t uuid,
                           uint8_t oper, uint16_t *pLen, uint8_t *pData)
{
    uint8_t status = SUCCESS;

    // write
    if(oper == HID_DEV_OPER_WRITE)
    {
        if(uuid == REPORT_UUID)
        {
            // process write to LED output report; ignore others
            if(type == HID_REPORT_TYPE_OUTPUT)
            {
                status = hidEmuRcvReport(*pLen, pData);
            }
        }

        if(status == SUCCESS)
        {
            status = Hid_SetParameter(id, type, uuid, *pLen, pData);
        }
    }
    // read
    else if(oper == HID_DEV_OPER_READ)
    {
        status = Hid_GetParameter(id, type, uuid, pLen, pData);
    }
    // notifications enabled
    else if(oper == HID_DEV_OPER_ENABLE)
    {
        tmos_start_task(hidEmuTaskId, START_REPORT_EVT, 500);
    }
    return status;
}

/*********************************************************************
 * @fn      hidEmuEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidEmuEvtCB(uint8_t evt)
{
    // process enter/exit suspend or enter/exit boot mode
    return;
}

/*********************************************************************
*********************************************************************/
