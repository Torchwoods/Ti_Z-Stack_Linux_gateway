/**************************************************************************************************
 Filename:       hal_mcu.h
 Revised:        $Date: 2010-11-16 10:58:50 +0000 (Tue, 16 Nov 2010) $
 Revision:       $Revision: 6 $

 Description:    Describe the purpose and contents of the file.


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

#ifndef HAL_MCU_H
#define HAL_MCU_H

/*
 *  Target : Linux - this file is ept for compatability
 *
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_defs.h"
#include "hal_types.h"

/* ------------------------------------------------------------------------------------------------
 *                                        Target Defines
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ---------------------- IAR Compiler ---------------------- */
#ifdef __IAR_SYSTEMS_ICC__
#include <msp430.h>
#define HAL_COMPILER_IAR
#define HAL_MCU_LITTLE_ENDIAN()   1
#define _PRAGMA(x) _Pragma(#x)
#define HAL_ISR_FUNC_DECLARATION(f,v) _PRAGMA(vector=v) __interrupt void f(void)
#define HAL_ISR_FUNC_PROTOTYPE(f,v)   _PRAGMA(vector=v) __interrupt void f(void)
#define HAL_ISR_FUNCTION(f,v)         HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)

/* ---------------------- Linux GCC Compiler ---------------------- */
#elif __GNUC__
#define _PRAGMA(x) 

/* ------------------ Unrecognized Compiler ------------------ */
#else
#error "ERROR: Unknown compiler."
#endif

/* ------------------------------------------------------------------------------------------------
 *                                       Interrupt Macros
 * ------------------------------------------------------------------------------------------------
 */
int halInterupts;
/*
 #define HAL_ENABLE_INTERRUPTS() halInterupts = 1;
 #define HAL_DISABLE_INTERRUPTS()  halInterupts = 0;
 #define HAL_INTERRUPTS_ARE_ENABLED() halInterupts

 typedef uint16 halIntState_t;
 #define HAL_ENTER_CRITICAL_SECTION(x)   HAL_DISABLE_INTERRUPTS();
 #define HAL_EXIT_CRITICAL_SECTION(x)    HAL_ENABLE_INTERRUPTS();
 #define HAL_CRITICAL_STATEMENT(x)       HAL_ENTER_CRITICAL_SECTION(s); HAL_EXIT_CRITICAL_SECTION(s);
 */
#define HAL_ENABLE_INTERRUPTS() halInterupts = 1;
#define HAL_DISABLE_INTERRUPTS()  halInterupts = 0;
#define HAL_INTERRUPTS_ARE_ENABLED() halInterupts

/* Dummy for this platform */
#define HAL_AES_ENTER_WORKAROUND() 
#define HAL_AES_EXIT_WORKAROUND() 

typedef uint16 halIntState_t;
#define HAL_ENTER_CRITICAL_SECTION(x) \
{                                     \
  x=1;                                \
  HAL_DISABLE_INTERRUPTS();           \
}

#define HAL_EXIT_CRITICAL_SECTION(x)  \
{                                     \
  x=0;                                \
  HAL_ENABLE_INTERRUPTS();            \
}

#define HAL_CRITICAL_STATEMENT(x)       HAL_ENTER_CRITICAL_SECTION(s); HAL_EXIT_CRITICAL_SECTION(s);

/* ------------------------------------------------------------------------------------------------
 *                                        Reset Macro
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_SYSTEM_RESET()              //Resetart application????????

/* ------------------------------------------------------------------------------------------------
 *                                     Simulated MCU real-time clock
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 */
#endif
