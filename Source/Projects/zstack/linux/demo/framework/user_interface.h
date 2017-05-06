/*******************************************************************************
 Filename:      user_interface.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:   User Interface


 Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,l
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
*******************************************************************************/
#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <gateway.pb-c.h>
#include <otasrvr.pb-c.h>

#include "types.h"

/******************************************************************************
 * Constants
 *****************************************************************************/
#define UI_STATUS_NOTIFICATION_TIMEOUT 1000

#define UI_LOG_LINE_LENGTH 80

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void ui_refresh_display(void); //shall be called whenever making any change to the global data-structure
int ui_init(char * log_filename);
void ui_deinit(void);
void _ui_print_log(const char * func_name, bool use_default_color, char * format, ...);
void ui_print_status(uint64_t timeout_ms, char * format, ...);
char * ui_make_string_GwAddrStructT(GwAddressStructT * addr);
char * ui_make_string_OtaAddrStruct(AddressStruct * addr);
void ui_print_packet_to_log(pkt_buf_t * pkt, char * prefix, char * hilight);
void ui_redraw_server_state(void);
void ui_redraw_network_info(void);
void ui_redraw_toggles_indications(void);

/******************************************************************************
 * Functional Macros
 *****************************************************************************/
#define  UI_PRINT_LOG(fmt, ...) do {_ui_print_log( __func__, true, (fmt), ##__VA_ARGS__);} while (0)

#define LITTLE_ENDIAN_TO_INT16(ADDR) \
((int32_t)( \
	(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
	(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) \
))

#define LITTLE_ENDIAN_TO_UINT16(ADDR) \
((uint32_t)( \
	(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
	(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) \
))


#define LITTLE_ENDIAN_TO_INT24(ADDR) \
	((int32_t)( \
		(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
		(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) + \
		(((uint32_t)(((uint8_t *)(ADDR))[2])) * 0x10000)+ \
		(((((uint8_t *)(ADDR))[2]) & 0x80) ? 0xFF000000 : 0) \
	))

#define LITTLE_ENDIAN_TO_UINT24(ADDR) \
((uint32_t)( \
	(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
	(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) + \
	(((uint32_t)(((uint8_t *)(ADDR))[2])) * 0x10000) \
))
	
#define LITTLE_ENDIAN_TO_INT32(ADDR) \
((int32_t)( \
	(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
	(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) + \
	(((uint32_t)(((uint8_t *)(ADDR))[2])) * 0x10000) + \
	(((uint32_t)(((uint8_t *)(ADDR))[3])) * 0x1000000) \
))

#define LITTLE_ENDIAN_TO_UINT32(ADDR) \
((uint32_t)( \
	(((uint32_t)(((uint8_t *)(ADDR))[0])) * 1) + \
	(((uint32_t)(((uint8_t *)(ADDR))[1])) * 0x100) + \
	(((uint32_t)(((uint8_t *)(ADDR))[2])) * 0x10000) + \
	(((uint32_t)(((uint8_t *)(ADDR))[3])) * 0x1000000) \
))

#define STRING_START(str, format, ...) snprintf((str), sizeof(str), format, ##__VA_ARGS__)
#define STRING_REMAINING_CHARS(str) (sizeof(str) - strlen(str) - 1)
#define STRING_ADD(str, format, ...) snprintf((str) + strlen(str), sizeof(str) - strlen(str), format, ##__VA_ARGS__)
#define STRING_ADD_64BIT_HEX(str, val) snprintf((str) + strlen(str), sizeof(str) - strlen(str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", (unsigned int)(((val) >> 56) & 0xFF), (unsigned int)(((val) >> 48) & 0xFF), (unsigned int)(((val) >> 40) & 0xFF), (unsigned int)(((val) >> 32) & 0xFF), (unsigned int)(((val) >> 24) & 0xFF), (unsigned int)(((val) >> 16) & 0xFF), (unsigned int)(((val) >> 8) & 0xFF), (unsigned int)(((val) >> 0) & 0xFF))

#define RESET_ATTRIBUTES	"\x1B[0m"
#define BOLD				"\x1B[1m"
#define UNDERSCORE			"\x1B[4m"
#define NO_UNDERSCORE		"\x1B[24m"
#define NORMAL				"\x1B[22m"
#define BLINK				"\x1B[5m"
#define REVERSED			"\x1B[7m"
#define CONCEALED			"\x1B[8m"

#define BLACK				"\x1B[30m"
#define RED					"\x1B[31m"
#define GREEN				"\x1B[32m"
#define YELLOW				"\x1B[33m"
#define BLUE				"\x1B[34m"
#define MAGENTA				"\x1B[35m"
#define CYAN				"\x1B[36m"
#define WHITE				"\x1B[37m"

#define onBLACK				"\x1B[40m"
#define onRED				"\x1B[41m"
#define onGREEN				"\x1B[42m"
#define onYELLO				"\x1B[43m"
#define onBLUE				"\x1B[44m"
#define onMAGENTA			"\x1B[45m"
#define onCYAN				"\x1B[46m"
#define onWHITE				"\x1B[47m"

#define CURSOR_HOME			"\x1B[H"
#define CURSOR_BACK			"\x1B[D"
#define CURSOR_GET_POS		"\x1B[6n"
#define CURSOR_SAVE			"\x1B[s"
#define CURSOR_RESTORE		"\x1B[u"

#define DARK_BLUE			RESET_ATTRIBUTES BLUE
#define LIGHT_BLUE			RESET_ATTRIBUTES BOLD BLUE

#define DARK_YELLOW			RESET_ATTRIBUTES YELLOW
#define LIGHT_YELLOW		RESET_ATTRIBUTES BOLD YELLOW

#define DARK_GREEN			RESET_ATTRIBUTES GREEN
#define LIGHT_GREEN			RESET_ATTRIBUTES BOLD GREEN

#define DARK_CYAN			RESET_ATTRIBUTES CYAN
#define LIGHT_CYAN			RESET_ATTRIBUTES BOLD CYAN

#define DARK_RED			RESET_ATTRIBUTES RED
#define LIGHT_RED			RESET_ATTRIBUTES BOLD RED

#define DARK_MAGENTA		RESET_ATTRIBUTES MAGENTA
#define LIGHT_MAGENTA		RESET_ATTRIBUTES BOLD MAGENTA

#define DARK_WHITE			RESET_ATTRIBUTES WHITE
#define LIGHT_WHITE			RESET_ATTRIBUTES BOLD WHITE

#endif /* USER_INTERFACE_H */
