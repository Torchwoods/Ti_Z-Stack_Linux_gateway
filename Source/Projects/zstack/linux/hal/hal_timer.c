/*********************************************************************
 Filename:       hal_timer.c
 Revised:        $Date: 2010-11-16 10:58:50 +0000 (Tue, 16 Nov 2010) $
 Revision:       $Revision: 6 $

 Description:   This file contains the interface to the Timer Service.


 Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
 **************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/timerfd.h>
#include <signal.h>
#include <strings.h>

#include "hal_types.h"
#include "hal_timer.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */
#define HAL_TIMER_INIT()

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef void (*timerCBack_t)( timer_t timerid, int myarg );

/***************************************************************************************************
 *                                              Private Functions
 ***************************************************************************************************/

halTimerCBack_t halTimerCb;

int timerfd;

/***************************************************************************************************
 *                                              FUNCTIONS - API
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      HalTimerInit
 *
 * @brief   Initialize Timer Service
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void HalTimerInit( void )
{
//Stub function
}

/***************************************************************************************************
 * @fn      HalTimerConfig
 *
 * @brief   Configure the Timer Serivce
 *
 * @param   timerId - Id of the timer
 *          opMode  - Operation mode
 *          channel - Channel where the counter operates on
 *          channelMode - Mode of that channel
 *          prescale - Prescale of the clock
 *          cBack - Pointer to the callback function
 *
 * @return  Status of the configuration
 ***************************************************************************************************/
uint8 HalTimerConfig( uint8 timerId, uint8 opMode, uint8 channel,
                      uint8 channelMode, bool intEnable, halTimerCBack_t cBack )
{
  struct itimerspec timspec;

  uiPrintfEx(trUNMASKABLE,  "HalTimerConfig++\n" );

  timerfd = timerfd_create( CLOCK_MONOTONIC, 0 );

  bzero( &timspec, sizeof(timspec) );
  timspec.it_interval.tv_sec = 0;
  timspec.it_interval.tv_nsec = 10000000;
  timspec.it_value.tv_sec = 0;
  timspec.it_value.tv_nsec = 10000000;

  int res = timerfd_settime( timerfd, 0, &timspec, 0 );
  if ( res < 0 )
  {
    perror( "timerfd_settime:" );
  }

  uiPrintfEx(trUNMASKABLE,  "HalTimerConfig--\n" );

  return 0;
}

/***************************************************************************************************
 * @fn      HalTimerStart
 *
 * @brief   Start the Timer Servic//Stub function
 e
 *
 * @param   timerId      - ID of the timer
 *          timerPerTick - number of micro sec per tick, (ticks x prescale) / clock = usec/tick
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerStart( uint8 timerId, uint32 timePerTick )
{
//Stub function
  return HAL_TIMER_OK;
}

/***************************************************************************************************
 * @fn      HalTimerTick
 *
 * @brief   Check the counter for expired counter.
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void HalTimerTick( void )
{
//Stub function
}

/***************************************************************************************************
 * @fn      HalTimerStop
 *
 * @brief   Stop the Timer Service
 *
 * @param   timerId - ID of the timer
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerStop( uint8 timerId )
{
  //Stub function
  return HAL_TIMER_OK;
}

/***************************************************************************************************
 * @fn      HalTimerInterruptEnable
 *
 * @brief   Enable or disable interrupt and set channel mode
 *
 * @param   timerId - ID of the timer
 *          channelMode - channel mode
 *          enable - TRUE or FALSE
 *
 * @return  Status - OK or Not OK
 ***************************************************************************************************/
uint8 HalTimerInterruptEnable( uint8 timerId, uint8 channelMode, bool enable )
{
//Stub function
  return HAL_TIMER_OK;
}

/***************************************************************************************************
 ***************************************************************************************************/

