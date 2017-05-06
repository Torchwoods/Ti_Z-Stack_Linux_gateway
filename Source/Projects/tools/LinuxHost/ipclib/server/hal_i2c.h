/**************************************************************************************************
  Filename:       hal_i2c.h
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

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
#ifndef HAL_I2C_H
#define HAL_I2C_H

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


#if ((defined NPI_I2C) && (NPI_I2C == TRUE))
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


/******************************************************/


/*********************************************************************
 * CONSTANTS
 */
#define I2C_OPEN_100MS_TIMEOUT 100
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS - API
 */

int HalI2cInit(const char *devpath);
int HalI2cWrite(uint8 port, uint8 *pBuf, uint8 len);
int HalI2cRead(uint8 port, uint8 *pBuf, uint8 len);
int HalI2cClose(void);

#endif  // #if (defined HAL_I2C) && (HAL_I2C == TRUE)

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************
******************************************************************************/
