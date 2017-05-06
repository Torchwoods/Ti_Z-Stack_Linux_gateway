/**************************************************************************************************
  Filename:       hal_spi.h
  Revised:        $Date: 2012-03-05 14:25:42 -0800 (Mon, 05 Mar 2012) $
  Revision:       $Revision: 219 $

  Description:     This file contains the interface to the SPI Service.


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
#ifndef HAL_SPI_H
#define HAL_SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

//#include "hal_board.h"
#include "hal_types.h"
#include "hal_gpio.h"

/////////////////////////////////////////////////////////////////////////////
// Typedefs

PACK_1 typedef struct ATTR_PACKED
{
	  uint32 speed;
	  uint8 mode;
	  uint8 bitsPerWord;
	  uint8 useFullDuplexAPI;
} halSpiCfg_t;


#if ((defined NPI_SPI) && (NPI_SPI == TRUE))
/*********************************************************************
 * MACROS
 */

//SRDY GPIO Pin
#define HAL_SRDY_VALUE "/sys/class/gpio/gpio132/value"
#define HAL_SRDY_DIR "/sys/class/gpio/gpio132/direction"

//SRDY Level shifter dirrection GPIO PIN
#define HAL_SRDY_DIR_VALUE "/sys/class/gpio/gpio137/value"
#define HAL_SRDY_DIR_DIR "/sys/class/gpio/gpio137/direction"

//MRDY GPIO Pin
#define HAL_MRDY_VALUE "/sys/class/gpio/gpio135/value"
#define HAL_MRDY_DIR "/sys/class/gpio/gpio135/direction"

//MRDY Level shifter dirrection GPIO PIN
#define HAL_MRDY_DIR_VALUE "/sys/class/gpio/gpio136/value"
#define HAL_MRDY_DIR_DIR "/sys/class/gpio/gpio136/direction"

//RESET GPIO Pin
#define HAL_RESET_VALUE "/sys/class/gpio/gpio134/value"
#define HAL_RESET_DIR "/sys/class/gpio/gpio134/direction"

//RESET Level shifter dirrection GPIO PIN
#define HAL_RESET_DIR_VALUE "/sys/class/gpio/gpio136/value"
#define HAL_RESET_DIR_DIR "/sys/class/gpio/gpio136/direction"

#define HAL_RNP_RUN()
#define HAL_RNP_RST()

/******************************************************/

/*
#define HAL_SPI_INIT() \
st ( \
  UCB1CTL1 = UCSSEL1 | UCSWRST;                 \
  UCB1CTL0 = UCCKPL | UCMSB | UCMST | UCSYNC ;  \
  UCB1BR0  = 3;                                 \
  UCB1BR1  = 0;                                 \
  P5SEL |= 0x0E;                                \
  P5SEL &=~0x01;                                \
  HAL_SPI_SS_OFF();                             \
  P5DIR |= 0x0B;                                \
  UCB1CTL1 &= ~UCSWRST;                         \
)
*/
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */

void HalSpiFlush(uint8 port, uint8 len);
int HalSpiInit(const char *devpath, halSpiCfg_t *halSpiCfg);
void HalSpiPoll(void);
int HalSpiRead(uint8 port, uint8 *pBuf, uint8 len);
int HalSpiWrite(uint8 port, uint8 *pBuf, uint8 len);
int HalSpiWriteRead(uint8 port, uint8 *pBuf, uint8 len);
void HalSpiClose( void );


#endif  // #if (defined HAL_SPI) && (HAL_SPI == TRUE)

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************
******************************************************************************/
