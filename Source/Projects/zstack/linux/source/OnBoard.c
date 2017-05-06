/*********************************************************************
 Filename:       OnBoard.c
 Revised:        $Date: 2008-10-07 15:05:56 -0700 (Tue, 07 Oct 2008) $
 Revision:       $Revision: 18215 $

 Description:    This file contains the UI and control for the
 peripherals on the EVAL development board
 Notes:

 Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License").  You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless used solely and exclusively in conjunction with
 a Texas Instruments radio frequency device, which is integrated into
 your product.  Other than for the foregoing purpose, you may not use,
 reproduce, copy, prepare derivative works of, modify, distribute, perform,
 display or sell this Software and/or its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.
 *********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "comdef.h"
#include "OnBoard.h"
#include "OSAL.h"
/*
 #include "MTEL.h"
 #include "DebugTrace.h"
 #include "hal_mcu.h"
 */
/* HAL */

#include "hal_drivers.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_timer.h"

#include "stdlib.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task ID not initialized
#define NO_TASK_ID 0xFF

// Minimum length RAM "pattern" for Stack check
#define MIN_RAM_INIT 12

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 OnboardKeyIntEnable;
uint8 OnboardTimerIntEnable;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

#ifdef OSAL_TOTAL_MEM
extern short osal_mem_high;
extern short osal_mem_used;
#endif

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Registered keys task ID, initialized to NOT USED.
static uint8 registeredKeysTaskID = NO_TASK_ID;

// Last confirmed MAC power state
static volatile uint8 macState;

#ifdef POWER_SAVING
// Sleep wake-up from keys
static volatile uint8 wakeKeys;
// Sleep wake-up from timer
static volatile uint8 wakeTimer;
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */

#ifdef LCD_SUPPORTED
#ifdef OSAL_TOTAL_MEM
void LCDShowMemoryUsed( void );
#endif
#endif

#if defined ( USE_KEY )
static void keyCB( uint8 keys, uint8 state );
#endif

/*********************************************************************
 * @fn      InitBoard()
 *
 * @brief   Initialize the CC2420DB Board Peripherals
 *
 * @param   level: COLD,WARM,READY
 *
 * @return  None
 *
 *********************************************************************/
void InitBoard( uint8 level )
{

#if defined ( USE_KEY )
  HalKeyConfig( TRUE, keyCB );
#endif
}

/*********************************************************************
 *                    EVENT PROCESSING FUNCTIONS
 *
 * These functions perform event-based (interrupt & timer) processing
 * of board level I/O, such as, keypress inputs and LED displays.
 */

/*********************************************************************
 * Keypad Support
 */

#if defined ( USE_KEY )
static void keyCB( uint8 keys, uint8 state )
{
  uint8 sendkey = 0;

  switch( keys )
  {
    case 1:
      sendkey = HAL_KEY_SW_1;
      break;

    case 2:
      sendkey = HAL_KEY_SW_2;
      break;

    case 3:
      sendkey = HAL_KEY_SW_3;
      break;

    case 4:
      sendkey = HAL_KEY_SW_4;
      break;

    case 5:
      sendkey = HAL_KEY_SW_5;
      break;
  }

  if ( sendkey != 0 )
  {
    OnBoard_SendKeys( sendkey, 0 );
  }
}
#endif

/*********************************************************************
 * @fn      RegisterForKeys
 *
 * @brief   The keyboard handler is setup to send all keyboard changes to
 *          one task (if a task is registered).
 *
 *          If a task registers, it will get all the keys. You can change this
 *          to register for individual keys.
 *
 * @param   task_id - task Id of the application where the keys will be sent
 *
 * @return  status
 *********************************************************************/
uint8 RegisterForKeys( uint8 task_id )
{
  // Allow only the first task
  if ( registeredKeysTaskID == NO_TASK_ID )
  {
    registeredKeysTaskID = task_id;
    return (true);
  }
  else
  {
    return (false);
  }
}

/*********************************************************************
 * @fn      OnBoard_SendKeys
 *
 * @brief   Send "Key Pressed" message to application.
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  status
 *********************************************************************/
uint8 OnBoard_SendKeys( uint8 keys, uint8 state )
{
  keyChange_t *msgPtr;

  if ( registeredKeysTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (keyChange_t *) osal_msg_allocate( sizeof(keyChange_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = KEY_CHANGE;
      msgPtr->state = state;
      msgPtr->keys = keys;

      osal_msg_send( registeredKeysTaskID, (uint8 *) msgPtr );
    }
    return (SUCCESS);
  }
  else
    return (FAILURE);
}

/* New Stuff */
/*********************************************************************
 * @fn      OnBoard_KeyCallback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 *********************************************************************/
void OnBoard_KeyCallback( uint8 keys, uint8 state )
{

  if ( OnBoard_SendKeys( keys, state ) != SUCCESS )
  {
    // Process SW1 here
    if ( keys & HAL_KEY_SW_1 )  // Switch 1
    {
    }
    // Process SW2 here
    if ( keys & HAL_KEY_SW_2 )  // Switch 2
    {
    }
    // Process SW3 here
    if ( keys & HAL_KEY_SW_3 )  // Switch 3
    {
    }
    // Process SW4 here
    if ( keys & HAL_KEY_SW_4 )  // Switch 4
    {
    }
    // Process SW5 here
    if ( keys & HAL_KEY_SW_5 )  // Switch 5
    {
    }
    // Process SW6 here
    if ( keys & HAL_KEY_SW_6 )  // Switch 6
    {
    }
  }
}

/*********************************************************************
 * @fn      Osal_TimerCallBack()
 *
 * @brief   Update the timer per tick
 *
 * @param   none
 *
 * @return  local clock in milliseconds
 **********************************************************************/
void Onboard_TimerCallBack( uint8 timerId, uint8 channel, uint8 channelMode )
{
  if ( (timerId == OSAL_TIMER) )
  {
    //osal_update_timers();
  }
}

/*********************************************************************
 *                    SLEEP MANAGEMENT FUNCTIONS
 *
 * These functions support processing of MAC and ZStack power mode
 * transitions, used when the system goes into or awakes from sleep.
 */

/*********************************************************************
 * @fn       mpmSetConfirm()
 *
 * @brief    Receive confirmation of MAC power mode change. Save
 *           current state of MAC power mode for sleep processing.
 *
 * @param    status - indication that MAC power mode update
 *
 * @return   none
 */
void mpmSetConfirm( uint8 status )
{
}

/*********************************************************************
 * @fn      OnBoard_stack_used()
 *
 * @brief
 *
 *   Runs through the stack looking for touched memory.
 *
 * @param   none
 *
 * @return  number of uint8s used by the stack
 *********************************************************************/
uint16 OnBoard_stack_used( void )
{
  return 0;
}

#ifdef LCD_SUPPORTED

#ifdef OSAL_TOTAL_MEM
/*********************************************************************
 * @fn      LCDShowMemoryUsed
 *
 * @brief   Show memory used
 *
 * @param   void
 *
 * @return  void
 *
 *********************************************************************/
void LCDShowMemoryUsed( void )
{
  WriteLCDString( 1, "Memory Used,High" );
  WriteLCDStringValueValue( " =", osal_mem_used, 10, osal_mem_high, 10, 2 );
}
#endif

#endif // LCD_SUPPORTED
/*********************************************************************
 * @fn        Onboard_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  uint16 - new random number
 */
uint16 Onboard_rand( void )
{
  uint16 ret = BUILD_UINT16( (uint8)rand(), (uint8)rand() );
  return ( ret );
}
/*********************************************************************
 *                    EXTERNAL I/O FUNCTIONS
 *
 * User defined functions to control external devices. Add your code
 * to the following functions to control devices wired to DB outputs.
 *
 */

void BigLight_On( void )
{
  // Put code here to turn on an external light
}

void BigLight_Off( void )
{
  // Put code here to turn off an external light
}

void BuzzerControl( uint8 on )
{
  // Put code here to turn a buzzer on/off
}

// No dip switches on this board
uint8 GetUserDipSw( void )
{
  return 0;
}

void Onboard_soft_reset( void )
{

}

/*********************************************************************
 *********************************************************************/
