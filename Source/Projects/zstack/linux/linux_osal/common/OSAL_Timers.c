/**************************************************************************************************
 Filename:       OSAL_Timers.c
 Revised:        $Date: 2011-09-12 15:20:05 +0100 (Mon, 12 Sep 2011) $
 Revision:       $Revision: 42 $

 Description:    OSAL Timer definition and manipulation functions.


 Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

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
#include <sys/timerfd.h> 
#include <time.h> 
#include <string.h> 
#include <stdint.h> 
#include <unistd.h>

#include "comdef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "hal_timer.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  void *next;
  int timerfd;
  uint16 timeout;
  uint16 event_flag;
  uint8 task_id;
  uint16 reloadTimeout;
} osalTimerRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

osalTimerRec_t *timerHead;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
osalTimerRec_t *osalAddTimer( uint8 task_id, uint16 event_flag, uint16 timeout );
osalTimerRec_t *osalFindTimer( uint8 task_id, uint16 event_flag );
void osalDeleteTimer( osalTimerRec_t *rmTimer );

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalTimerInit
 *
 * @brief   Initialization for the OSAL Timer System.
 *
 * @param   none
 *
 * @return
 */
void osalTimerInit( void )
{
  //depricated function
}

/*********************************************************************
 * @fn      osalAddTimer
 *
 * @brief   Add a timer to the timer list.
 *          Ints must be disabled.
 *
 * @param   task_id
 * @param   event_flag
 * @param   timeout
 *
 * @return  osalTimerRec_t * - pointer to newly created timer
 */
osalTimerRec_t * osalAddTimer( uint8 task_id, uint16 event_flag,
                               uint16 timeout )
{
  osalTimerRec_t *newTimer;
  osalTimerRec_t *srchTimer;

  // Look for an existing timer first
  newTimer = osalFindTimer( task_id, event_flag );
  if ( newTimer )
  {
    // Timer is found - delete and create again.
    osalDeleteTimer( newTimer );
  }

  // New Timer
  newTimer = osal_mem_alloc( sizeof(osalTimerRec_t) );

  if ( newTimer )
  {
    // Fill in new timer
    newTimer->task_id = task_id;
    newTimer->event_flag = event_flag;
    newTimer->timeout = timeout;
    newTimer->next = (void *) NULL;
    newTimer->reloadTimeout = 0;
    newTimer->timerfd = timerfd_create( CLOCK_MONOTONIC, TFD_NONBLOCK );

    //uiPrintfEx(trUNMASKABLE, "osal add timer fd:%d\n", newTimer->timerfd);

    // Does the timer list already exist
    if ( timerHead == NULL )
    {
      // Start task list
      timerHead = newTimer;
    }
    else
    {
      // Add it to the end of the timer list
      srchTimer = timerHead;

      // Stop at the last record
      while ( srchTimer->next )
        srchTimer = srchTimer->next;

      // Add to the list
      srchTimer->next = newTimer;
    }

    return (newTimer);
  }
  else
    return ((osalTimerRec_t *) NULL);
}

/*********************************************************************
 * @fn      osalFindTimer
 *
 * @brief   Find a timer in a timer list.
 *          Ints must be disabled.
 *
 * @param   task_id
 * @param   event_flag
 *
 * @return  osalTimerRec_t *
 */
osalTimerRec_t *osalFindTimer( uint8 task_id, uint16 event_flag )
{
  osalTimerRec_t *srchTimer;

  // Head of the timer list
  srchTimer = timerHead;

  // Stop when found or at the end
  while ( srchTimer )
  {
    if ( srchTimer->event_flag == event_flag && srchTimer->task_id == task_id )
      break;

    // Not this one, check another
    srchTimer = srchTimer->next;
  }

  return (srchTimer);
}

/*********************************************************************
 * @fn      osalDeleteTimer
 *
 * @brief   Delete a timer from a timer list.
 *
 * @param   table
 * @param   rmTimer
 *
 * @return  none
 */
void osalDeleteTimer( osalTimerRec_t *rmTimer )
{
  // Does the timer list really exist
  if ( rmTimer )
  {
    //find prev timer
    osalTimerRec_t *srchTimer, *prevTimer = NULL;

    // Head of the timer list
    srchTimer = timerHead;

    // Stop when timer found or at the end
    while ( (srchTimer != rmTimer) && (srchTimer->next) )
    {
      prevTimer = srchTimer;
      // over to next
      srchTimer = srchTimer->next;
    }

    if ( srchTimer != rmTimer )
    {
      return; //timer not found
    }

    // delete the timer from the list
    if ( prevTimer == NULL )
      timerHead = srchTimer->next;
    else
      prevTimer->next = srchTimer->next;

    close( rmTimer->timerfd );
    osal_mem_free( rmTimer );
  }
}

/*********************************************************************
 * @fn      osal_start_timerEx
 *
 * @brief
 *
 *   This function is called to start a timer to expire in n mSecs.
 *   When the timer expires, the calling task will get the specified event.
 *
 * @param   uint8 taskID - task id to set timer for
 * @param   uint16 event_id - event to be notified with
 * @param   UNINT16 timeout_value - in milliseconds.
 *
 * @return  SUCCESS, or NO_TIMER_AVAIL.
 */
uint8 osal_start_timerEx( uint8 taskID, uint16 event_id, uint16 timeout_value )
{
  halIntState_t intState;
  osalTimerRec_t *newTimer;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  // Add timer
  newTimer = osalAddTimer( taskID, event_id, timeout_value );

  if ( newTimer )
  {
    struct itimerspec timspec;
    unsigned int timeout_value_s = timeout_value / 1000;
    unsigned long timeout_value_ns = (timeout_value * 1000000);

    if ( timeout_value_ns > (1000000000 - 1) )
    {
      timeout_value_ns -= (timeout_value_s * 1000000000);
    }

    bzero( &timspec, sizeof(timspec) );
    timspec.it_value.tv_sec = timeout_value_s;
    timspec.it_value.tv_nsec = timeout_value_ns;
    //timer is single shot
    timspec.it_interval.tv_sec = 0;
    timspec.it_interval.tv_nsec = 0;

    if ( newTimer->timerfd == 0 )
      uiPrintfEx(trUNMASKABLE,  "osal_start_timerEx: timerfd NULL\n" );
    if ( timeout_value == 0 )
      uiPrintfEx(trUNMASKABLE,  "osal_start_timerEx: timeout_value is 0\n" );

    int res = timerfd_settime( newTimer->timerfd, 0, &timspec, 0 );

    if ( res < 0 )
    {
      perror( "timerfd_settime:" );
    }
  }
  else
  {
    perror( "timer allocation failed\n" );
  }

  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

  return ((newTimer != NULL) ? SUCCESS : NO_TIMER_AVAIL);
}

/*********************************************************************
 * @fn      osal_start_reload_timer
 *
 * @brief
 *
 *   This function is called to start a timer to expire in n mSecs.
 *   When the timer expires, the calling task will get the specified event
 *   and the timer will be reloaded with the timeout value.
 *
 * @param   uint8 taskID - task id to set timer for
 * @param   uint16 event_id - event to be notified with
 * @param   UNINT16 timeout_value - in milliseconds.
 *
 * @return  SUCCESS, or NO_TIMER_AVAIL.
 */
uint8 osal_start_reload_timer( uint8 taskID, uint16 event_id,
                               uint16 timeout_value )
{
  halIntState_t intState;
  osalTimerRec_t *newTimer;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  // Add timer
  newTimer = osalAddTimer( taskID, event_id, timeout_value );

  if ( newTimer )
  {
    struct itimerspec timspec;
    unsigned int timeout_value_s = timeout_value / 1000;
    unsigned long timeout_value_ns = (timeout_value * 1000000);

    if ( timeout_value_ns > (1000000000 - 1) )
    {
      timeout_value_ns -= (timeout_value_s * 1000000000);
    }

    // Load the reload timeout value
    newTimer->reloadTimeout = timeout_value;

    bzero( &timspec, sizeof(timspec) );
    timspec.it_value.tv_sec = timeout_value_s;
    timspec.it_value.tv_nsec = timeout_value_ns;
    //timer is single shot, will be reloaded manually when expired
    timspec.it_interval.tv_sec = 0;
    timspec.it_interval.tv_nsec = 0;

    if ( newTimer->timerfd == 0 )
      uiPrintfEx(trUNMASKABLE,  "osal_start_reload_timer: timerfd NULL\n" );
    if ( timeout_value == 0 )
      uiPrintfEx(trUNMASKABLE,  "osal_start_reload_timer: timeout_value is 0\n" );

    //uiPrintfEx(trUNMASKABLE, "osal_start_reload_timer: timerfd_settime fd(%x) %d - %d, %d\n", newTimer->timerfd, timeout_value, timeout_value_s, timeout_value_ns);
    int res = timerfd_settime( newTimer->timerfd, 0, &timspec, 0 );
    if ( res < 0 )
    {
      perror( "timerfd_settime:" );
      uiPrintfEx(trUNMASKABLE,  "osal_start_reload_timer: Error in timerfd_settime\n" );
    }
  }
  else
  {
    perror( "timer allocation failed\n" );
  }

  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

  return ((newTimer != NULL) ? SUCCESS : NO_TIMER_AVAIL);
}

/*********************************************************************
 * @fn      osal_stop_timerEx
 *
 * @brief
 *
 *   This function is called to stop a timer that has already been started.
 *   If ZSUCCESS, the function will cancel the timer and prevent the event
 *   associated with the timer from being set for the calling task.
 *
 * @param   uint8 task_id - task id of timer to stop
 * @param   uint16 event_id - identifier of the timer that is to be stopped
 *
 * @return  SUCCESS or INVALID_EVENT_ID
 */
uint8 osal_stop_timerEx( uint8 task_id, uint16 event_id )
{
  halIntState_t intState;
  osalTimerRec_t *foundTimer;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  // Find the timer to stop
  foundTimer = osalFindTimer( task_id, event_id );
  if ( foundTimer )
  {
    osalDeleteTimer( foundTimer );
  }

  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

  return ((foundTimer != NULL) ? SUCCESS : INVALID_EVENT_ID);
}

/*********************************************************************
 * @fn      osal_get_timeoutEx
 *
 * @brief
 *
 * @param   uint8 task_id - task id of timer to check
 * @param   uint16 event_id - identifier of timer to be checked
 *
 * @return  Return the timer's tick count if found, zero otherwise.
 */
uint16 osal_get_timeoutEx( uint8 task_id, uint16 event_id )
{
  //depricated function
  return 0;
}

/*********************************************************************
 * @fn      osal_timer_num_active
 *
 * @brief
 *
 *   This function counts the number of active timers.
 *
 * @return  uint8 - number of timers
 */
uint8 osal_timer_num_active( void )
{
  halIntState_t intState;
  uint8 num_timers = 0;
  osalTimerRec_t *srchTimer;

  HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

  // Head of the timer list
  srchTimer = timerHead;

  // Count timers in the list
  while ( srchTimer != NULL )
  {
    num_timers++;
    srchTimer = srchTimer->next;
  }

  HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.

  return num_timers;
}

/*********************************************************************
 * @fn      osalTimerUpdate
 *
 * @brief   Update the timer structures for a timer tick.
 *
 * @param   none
 *
 * @return  none
 *********************************************************************/
void osalTimerUpdate( uint16 updateTime )
{
  osalTimerRec_t *srchTimer, *nextTimer;

  //depricated param
  (void) updateTime;

  //Search through the timers to see which one has expired.
  // Head of the timer list
  srchTimer = timerHead;

  // Count timers in the list
  while ( srchTimer != NULL )
  {
    uint64_t expiration = 0;

    //save next timer
    nextTimer = srchTimer->next;

    read( srchTimer->timerfd, &expiration, sizeof(expiration) );

    if ( expiration > 0 )
    {
      //uiPrintfEx(trUNMASKABLE, "osal timer expired fd:%d\n", srchTimer->timerfd);
      //trigger event
      osal_set_event( srchTimer->task_id, srchTimer->event_flag );

      // Check for reloading and create new timer if needed
      if ( srchTimer->reloadTimeout )
      {
        osal_start_reload_timer( srchTimer->task_id, srchTimer->event_flag,
            srchTimer->timeout );
      }
      else
      {
        // delete the timer from the list
        osalDeleteTimer( srchTimer );
      }
    }
    //go to next timer
    srchTimer = nextTimer;
  }

}

/*********************************************************************
 * @fn      osal_GetTimerFds()
 *
 * @brief   Read the local system clock.
 *
 * @param   none
 *
 * @return  list of Timerfd's
 */
void osal_GetTimerFds( int *fds, int maxFds )
{
  int fdidx = 0;
  osalTimerRec_t *srchTimer = timerHead;

  while ( (srchTimer != NULL) && (fdidx < maxFds) )
  {
    fds[fdidx] = srchTimer->timerfd;

    //go to next in the list
    srchTimer = srchTimer->next;
    fdidx++;
  }

  return;
}

/*********************************************************************
 * @fn      osal_GetSystemClock()
 *
 * @brief   Read the local system clock.
 *
 * @param   none
 *
 * @return  local clock in milliseconds
 */
uint32 osal_GetSystemClock( void )
{
  //depticated function
  return (0);
}

/*********************************************************************
 *********************************************************************/
