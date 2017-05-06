/**************************************************************************************************
 Filename:       OSAL_Memory.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    OSAL Heap Memory management functions.


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

#include "OSAL_Memory.h"
#include <malloc.h>

/*********************************************************************
 * MACROS
 */

/*
 *  The MAC_ASSERT macro is for use during debugging.
 *  The given expression must evaluate as "true" or else fatal error occurs.
 *  At that point, the call stack feature of the debugger can pinpoint where
 *  the problem occurred.
 *
 *  To disable this feature and save code size, the project should define
 *  OSALMEM_NODEBUG to TRUE.
 */
#if ( OSALMEM_NODEBUG )
#define OSALMEM_ASSERT( expr )
#else
#define OSALMEM_ASSERT( expr)        HAL_ASSERT( expr )
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

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
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      osal_mem_init
 *
 * @brief   Initialize the heap memory management system.
 *
 * @param   void
 *
 * @return  void
 */
void osal_mem_init( void )
{
}

/*********************************************************************
 * @fn      osal_mem_kick
 *
 * @brief   Kick the ff1 pointer out past the long-lived OSAL Task blocks.
 *          Invoke this once after all long-lived blocks have been allocated -
 *          presently at the end of osal_init_system().
 *
 * @param   void
 *
 * @return  void
 */
void osal_mem_kick( void )
{
}

/*********************************************************************
 * @fn      osal_mem_alloc
 *
 * @brief   Implementation of the allocator functionality.
 *
 * @param   size - number of bytes to allocate from the heap.
 *
 * @return  void * - pointer to the heap allocation; NULL if error or failure.
 */
void *osal_mem_alloc( uint16 size )
{
  return malloc( size );
}

/*********************************************************************
 * @fn      osal_mem_free
 *
 * @brief   Implementation of the de-allocator functionality.
 *
 * @param   ptr - pointer to the memory to free.
 *
 * @return  void
 */
void osal_mem_free( void *ptr )
{
  free( ptr );
}

#undef  OSALMEM_METRICS
#if ( OSALMEM_METRICS )
/*********************************************************************
 * @fn      osal_heap_block_max
 *
 * @brief   Return the maximum number of blocks ever allocated at once.
 *
 * @param   none
 *
 * @return  Maximum number of blocks ever allocated at once.
 */
uint16 osal_heap_block_max( void )
{
  //Stub out
  return 0;
}

/*********************************************************************
 * @fn      osal_heap_block_cnt
 *
 * @brief   Return the current number of blocks now allocated.
 *
 * @param   none
 *
 * @return  Current number of blocks now allocated.
 */
uint16 osal_heap_block_cnt( void )
{
  //Stub out
  return 0;
}

/*********************************************************************
 * @fn      osal_heap_block_free
 *
 * @brief   Return the current number of free blocks.
 *
 * @param   none
 *
 * @return  Current number of free blocks.
 */
uint16 osal_heap_block_free( void )
{
  //Stub out
  return 0;
}

/*********************************************************************
 * @fn      osal_heap_mem_used
 *
 * @brief   Return the current number of bytes allocated.
 *
 * @param   none
 *
 * @return  Current number of bytes allocated.
 */
uint16 osal_heap_mem_used( void )
{
  //Stub out
  return 0;
}
#endif

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
/*********************************************************************
 * @fn      osal_heap_high_water
 *
 * @brief   Return the highest byte ever allocated in the heap.
 *
 * @param   none
 *
 * @return  Highest number of bytes ever used by the stack.
 */
uint16 osal_heap_high_water( void )
{
  //Stub out
  return 0;
}
#endif

/*********************************************************************
 *********************************************************************/
