/**************************************************************************************************
  Filename:       hal_gpio.h
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:     This file contains the interface to the GPIO Service.


  Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/


   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

     Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the
     distribution.

     Neither the name of Texas Instruments Incorporated nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

//#include "hal_board.h"
#include "hal_types.h"

/*********************************************************************
 * MACROS
 */

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

#define HAL_RNP_RUN()
#define HAL_RNP_RST()

#define HAL_GPIO_ACTIVE_HIGH		1
#define HAL_GPIO_ACTIVE_LOW			0

#define ERROR -1
/*******************************************/

/*** We will be using SPI mode so no need to support MRDY and SRDY ***/

#define HAL_RNP_MRDY_CFG() HalGpioMrdyInit(0)

#define HAL_RNP_MRDY_CLR() HalGpioMrdySet(0)

#define HAL_RNP_MRDY_SET() HalGpioMrdySet(1)

#define HAL_RNP_SRDY_CFG() HalGpioSrdyInit(0)

#define HAL_RNP_SRDY_CLR() HalGpioSrdyCheck(0)
#define HAL_RNP_SRDY_SET() HalGpioSrdyCheck(1)

#define HAL_RNP_SRDY_RX() HAL_RNP_SRDY_SET()
#define HAL_RNP_SRDY_TX() HAL_RNP_SRDY_CLR()


/*********************************************************************
 * CONSTANTS
 */

#define HAL_WAIT_SRDY_HIGH_TIMEOUT				500
#define HAL_WAIT_SRDY_HIGH_INTERMEDIATE_TIMEOUT  50
#define HAL_WAIT_SRDY_LOW_TIMEOUT				500
#define HAL_WAIT_SRDY_LOW_INTERMEDIATE_TIMEOUT   50

/*********************************************************************
 * TYPEDEFS
 */
#if !defined PACK_1
#define PACK_1
#endif

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif


PACK_1 typedef struct ATTR_PACKED
{
	char value[128];
	char direction[128];
	char edge[128];
	uint8 active_high_low;
} gpioCfg_t;

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

PACK_1 typedef struct ATTR_PACKED
{
	gpioCfg_t gpio;
	gpioCfg_t levelshifter;
} halGpioCfg_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */
void HalGpioUsage(void);
int HalGpioSrdyInit(halGpioCfg_t *gpioCfg);
int HalGpioMrdyInit(halGpioCfg_t *gpioCfg);
int HalGpioResetInit(halGpioCfg_t *gpioCfg);
int HalGpioMrdySet(uint8 state);
int HalGpioMrdyCheck(uint8 state);
int HalGpioSrdyCheck(uint8 state);
int HalGpioWaitSrdyClr(void);
int HalGpioWaitSrdySet(void);
void HalGpioSrdyClose( void );
void HalGpioMrdyClose( void );
void HalGpioResetClose( void );
int HalGpioResetSet(uint8 state);
int HalGpioReset( void );

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************
******************************************************************************/
