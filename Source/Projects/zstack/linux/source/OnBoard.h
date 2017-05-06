#ifndef ONBOARD_H
#define ONBOARD_H

/*********************************************************************
 Filename:       OnBoard.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:    Defines stuff for EVALuation boards
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

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/* Hal */
#include "hal_uart.h"
#include "OSAL.h"

#if defined __IAR_SYSTEMS_ICC__
// Standard IAR include files for AVR
#include <inavr.h>
#include <iom128.h>
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

#ifdef __GNUC__
// Start of initialized RAM
extern unsigned char __data_start;
// End of initialized RAM
extern unsigned char __heap_start;
#endif

// 64-bit Extended Address of this device
extern uint8 aExtendedAddress[8];

/*********************************************************************
 * MACROS
 */

// Wait for specified microseconds
#define MicroWait(t) halWait(t)

// Restart system from absolute beginning
#define SystemReset()  SIM_Reboot();

#define SystemResetSoft()  Onboard_soft_reset()

// Jump to F8W Serial Port Bootloader
#if (FLASHEND == 0xFFFF)
// Bootloader address: 0x0F800 (0x7C00 word)
//#define BootLoader()  asm ("cli"); asm ("jmp 0xF800");
#define BootLoader()
#else
// Bootloader address: 0x01F800 (0xFC00 word)
//#define BootLoader()  asm ("cli"); asm ("jmp 0x1F800");
#define BootLoader()
#endif

// Reset identification flags
#define RSFJT _BV(JTRF)  // Reset from JTAG
#define RSFWD _BV(WDRF)  // Reset from Watchdog
#define RSFBO _BV(BORF)  // Reset from Brown-Out
#define RSFEX _BV(EXTRF)  // Reset from External
#define RSFPO _BV(PORF)  // Reset from Power-On
#define RSBTS (RSFJT | RSFWD | RSFBO | RSFEX | RSFPO)

// JTAG disable
#define JTAGD _BV(JTD)  // Disables JTAG interface
// Disable JTAG - set JTD bit twice within 4 cycles
#define DISABLE_JTAG() \
{ \
  MCUCSR |= JTAGD; \
  MCUCSR |= JTAGD; \
}

// Enable JTAG - clear JTD bit twice within 4 cycles
#define ENABLE_JTAG() \
{ \
  MCUCSR &= ~JTAGD; \
  MCUCSR &= ~JTAGD; \
}

// Restore JTAG - write JSB bit twice within 4 cycles
#define RESTORE_JTAG(jsb) \
{ \
  MCUCSR |= jsb; \
  MCUCSR |= jsb; \
}

// Configure Unused I/O pins
#define CFG_UNUSED() (PORTE |= _BV(PE7))

// Power conservation
#define SLEEP_IDLE     0
#define SLEEP_PWRDOWN _BV(SM1)
#define SLEEP_MASK (_BV(SM0) | _BV(SM1) | _BV(SM2))

#define enter_sleep_mode(mode) \
{                              \
  MCUCR &= ~SLEEP_MASK;        \
  MCUCR |= (_BV(SE) | mode);   \
  asm ("sleep");               \
  MCUCR &= ~_BV(SE);           \
}

// Sleep mode definitions (enabled with POWER_SAVING compile option)
#define SLEEP_LITE 1  // AVR idle, all wakeup sources
#define SLEEP_DEEP 2  // AVR powerdown, external int wakeup
#define OSAL_SET_CPU_INTO_SLEEP(m) OnBoard_Sleep(m);  // Called from OSAL_PwrMgr
/*********************************************************************
 * CONSTANTS
 */
// MAC/2420 Power State
#define MAC_PWR_OFF  0
#define MAC_PWR_WAIT 1
//#define MAC_PWR_ON   2

// Timer clock and power-saving definitions
#ifdef CPU8MHZ
#define TICK_COUNT  125
#define RETUNE_THRESHOLD 250  // 8 Mhz threshold for power saving algorithm
#else
#define TICK_COUNT  250
#define RETUNE_THRESHOLD 500  // 16 Mhz threshold for power saving algorithm
#endif
#define TIMER_DECR_TIME    1  // 1ms - has to be matched with TC_OCC
/* OSAL timer defines */
#define TICK_TIME   1000   /* Timer per tick - in micro-sec */
#if OSAL_TIMER_16_BIT == TRUE
#define OSAL_TIMER  HAL_TIMER_3
#else
#define OSAL_TIMER  HAL_TIMER_1
#endif

// Defualt Serial Port Application Mapping
#if !defined (ZAPP_P1)  && !defined (ZAPP_P2) && \
    !defined (ZTOOL_P1) && !defined (ZTOOL_P2)
#if defined (MT_TASK)
#if defined (SERIAL_XFER)
#define ZAPP_P1
#else
#define ZTOOL_P1
#endif
#endif
#endif // !ZAPP && !ZTOOL
// Serial Ports ID Codes
#if defined (ZAPP_P1) || defined (ZTOOL_P1)
#define SERIAL_PORT1 0x01
#else
#undef SERIAL_PORT1
#endif
#if defined (ZAPP_P2) || defined (ZTOOL_P2)
#define SERIAL_PORT2 0x02
#else
#undef SERIAL_PORT2
#endif

// Application Serial Port Assignments
#if defined (ZAPP_P1)
#define ZAPP_PORT HAL_UART_PORT_1 //SERIAL_PORT1
#elif defined (ZAPP_P2)
#define ZAPP_PORT HAL_UART_PORT_0 //SERIAL_PORT2
#else
#undef ZAPP_PORT
#endif
#if defined (ZTOOL_P1)
#define ZTOOL_PORT HAL_UART_PORT_1 //SERIAL_PORT1
#elif defined (ZTOOL_P2)
#define ZTOOL_PORT HAL_UART_PORT_0 //SERIAL_PORT2
#else
#undef ZTOOL_PORT
#endif

/* Tx and Rx buffer size defines */
#define SPI_THRESHOLD    20
#define SPI_TX_BUFF_MAX  255
#define SPI_RX_BUFF_MAX  255
#define SPI_IDLE_TIMEOUT 5

// Internal (MCU) RAM addresses
#define MCU_RAM_BEG 0x0100
#define MCU_RAM_END 0xffffffff
#define MCU_RAM_LEN (MCU_RAM_END - MCU_RAM_BEG + 1)

// Internal (MCU) Stack addresses
#ifdef __GNUC__
#undef RSTK_PTR
#define CSTK_PTR SP
#define MCU_BSS_LEN (&__heap_start - &__data_start)
#define CSTK_BEG (MCU_RAM_BEG + MCU_BSS_LEN)
#else
#define RSTK_PTR SP
#define RSTK_BEG _Pragma("segment=\"RSTACK\"") __segment_begin("RSTACK")
//  __no_init __tiny const volatile unsigned int CSTK_PTR @ 0x1C;
#define CSTK_BEG _Pragma("segment=\"CSTACK\"") __segment_begin("CSTACK")
#endif

#define STACK_INIT_VALUE  0xCD

// External (SRAM) RAM addresses
#define EXT_RAM_BEG 0x8000
#define EXT_RAM_END 0xFFFF
#define EXT_RAM_LEN (EXT_RAM_END - EXT_RAM_BEG)

// EEPROM (MCU) addresses
#define MCU_EEPROM_BEG 0x0000
#define MCU_EEPROM_END E2END
#define MCU_EEPROM_LEN (MCU_EEPROM_END - MCU_EEPROM_BEG + 1)

// OSAL Non-Volatile (NV) size
#define OSAL_NV_SIZE 0x0FFF

// Internal (MCU) heap size
#if !defined( INT_HEAP_LEN )
#if defined( ZDO_COORDINATOR )
#define INT_HEAP_LEN  15000  // 0.75K
#else
#define INT_HEAP_LEN  3000  // 1.00K
#endif
#endif

// Memory Allocation Heap
#if defined( EXTERNAL_RAM )
#define MAXMEMHEAP EXT_RAM_LEN   // Typically, 32K
#else
#define MAXMEMHEAP INT_HEAP_LEN  // Typically, 0.70-1.50K
#endif

#define KEY_CHANGE_SHIFT_IDX 1
#define KEY_CHANGE_KEYS_IDX  2

// These Key definitions are unique to this development system.
// They are used to bypass functions when starting up the device.
#define SW_BYPASS_NV    HAL_KEY_SW_5  // Bypass Network layer NV restore
#define SW_BYPASS_START HAL_KEY_SW_1  // Bypass Network initialization
// Initialization levels
#define OB_COLD  0
#define OB_WARM  1
#define OB_READY 2

#ifdef LCD_SUPPORTED
#define BUZZER_OFF  0
#define BUZZER_ON   1
#define BUZZER_BLIP 2
#endif

typedef struct
{
  osal_event_hdr_t hdr;
  uint8 state; // shift
  uint8 keys;  // keys
} keyChange_t;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Initialize the Peripherals
 *    level: 0=cold, 1=warm, 2=ready
 */
extern void InitBoard( uint8 level );

/*
 * Get elapsed timer clock counts
 *   reset: reset count register if TRUE
 */
extern uint16 TimerElapsed( uint8 reset );

/*
 * Register for all key events
 */
extern uint8 RegisterForKeys( uint8 task_id );

/* Keypad Control Functions */

/*
 * Send "Key Pressed" message to application
 */
extern uint8 OnBoard_SendKeys( uint8 keys, uint8 state );

/* LCD Emulation/Control Functions */
/*
 * Convert an interger to an ascii string
 */
//  extern void _itoa(uint16 num, uint8 *buf, uint8 radix);

/* External I/O Processing Functions */
/*
 * Turn on an external lamp
 */
extern void BigLight_On( void );

/*
 * Turn off an external lamp
 */
extern void BigLight_Off( void );

/*
 * Turn on/off an external buzzer
 *   on:   BUZZER_ON or BUZZER_OFF
 */
extern void BuzzerControl( uint8 on );

/*
 * Get setting of external dip switch
 */
extern uint8 GetUserDipSw( void );

/*
 * Calculate the size of used stack
 */
extern uint16 OnBoard_stack_used( void );

/*
 * Enter specified sleep mode
 *   mode: sleep mode ID
 *           0 = None
 *           1 = Light
 *           2 = Deep
 */
extern void OnBoard_Sleep( uint8 mode );

/*
 * Interrupt driven Rx/Tx for Equos SIO comm with Micro32.
 */
// extern void write( uint8 port, const uint8 *buf, uint8 cnt );
// extern uint8 read( uint8 port, uint8 *buf, uint8 cnt );
/*
 * Callback routine to handle keys
 */
extern void OnBoard_KeyCallback( uint8 keys, uint8 state );

/*
 * Callback function to handle timer
 */
extern void Onboard_TimerCallBack( uint8 timerId, uint8 channel,
    uint8 channelMode );

/*
 * Set wakeTime which is used by power_saving
 */
extern void Onboard_SetWakeTime( uint8 status );

/*
 * Set wakeKeys wich is used by power_saving
 */
extern void Onboard_SetWakeKeys( uint8 keys );

/*
 * Get macState parameter
 */
extern uint8 Onboard_GetMacState( );

/*
 * Board specific random number generator
 */
extern uint16 Onboard_rand( void );

extern void Onboard_soft_reset( void );

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // ONBOARD_H
