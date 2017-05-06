/**************************************************************************************************
  Filename:       hal_gpio.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the HAL GPIO.


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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/poll.h>

#include "pthread.h"

//#include  "hal_board.h"
#include  "hal_types.h"
#include  "hal_gpio.h"

#include "npi_lnx_error.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#elif defined __DEBUG_TIME__
#include <sys/time.h>
#endif //__DEBUG_TIME__

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__); fflush(stdout);
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#ifdef __DEBUG_TIME__
#define time_printf(fmt, ...) st (if ((__DEBUG_TIME_ACTIVE == TRUE) && (__BIG_DEBUG_ACTIVE == TRUE)) printf( fmt, ##__VA_ARGS__);)
#endif //__DEBUG_TIME__
/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int gpioSrdyFd;
static int gpioMrdyFd;
static int gpioResetFd;

static halGpioCfg_t srdyGpioCfg;
static halGpioCfg_t mrdyGpioCfg;
static halGpioCfg_t resetGpioCfg;

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
extern struct timeval prevTimeI2C;
#elif defined __DEBUG_TIME__
struct timeval curTime, prevTime;
extern struct timeval startTime;
#endif //__DEBUG_TIME__

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalGpioSrdyClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioSrdyClose(void)
{
	close(gpioSrdyFd);
}

/**************************************************************************************************
 * @fn      HalGpioMrdyClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioMrdyClose(void)
{
	close(gpioMrdyFd);
}

/**************************************************************************************************
 * @fn      HalGpioResetClose
 *
 * @brief   Close SRDY IO
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalGpioResetClose(void)
{
	close(gpioResetFd);
}

/**************************************************************************************************
 * @fn      HalGpioSrdyInit
 *
 *
 * @brief   Initialise SRDY GPIO.
 *
 * @param   gpioCfg - SRDY pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioSrdyInit(halGpioCfg_t *gpioCfg)
{
	memcpy(srdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("[GPIO]resetGpioCfg.gpio.value = '%s'\n", srdyGpioCfg.gpio.value);
	memcpy(srdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("[GPIO]srdyGpioCfg.gpio.direction = '%s'\n", srdyGpioCfg.gpio.direction);

	srdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	memcpy(srdyGpioCfg.gpio.edge,
			gpioCfg->gpio.edge,
			strlen(gpioCfg->gpio.edge));
	debug_printf("[GPIO]srdyGpioCfg.gpio.edge = '%s'\n", srdyGpioCfg.gpio.edge);

	if ( ( gpioCfg->levelshifter.value) &&
		 ( gpioCfg->levelshifter.active_high_low) &&
		 ( gpioCfg->levelshifter.direction))
	{

	memcpy(srdyGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
		debug_printf("[GPIO]srdyGpioCfg.levelshifter.value = '%s'\n", srdyGpioCfg.levelshifter.value);
	memcpy(srdyGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	srdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
		debug_printf("[GPIO]srdyGpioCfg.levelshifter.direction = '%s'\n", srdyGpioCfg.levelshifter.direction);

	//open the GPIO DIR file for the level shifter direction signal
	gpioSrdyFd = open(srdyGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\n[GPIO]%s open failed\n",srdyGpioCfg.levelshifter.direction);
	    npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the direction of the GPIO to output
	if (ERROR == write(gpioSrdyFd, "out", 3))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\n[GPIO]can't write in %s \n",srdyGpioCfg.levelshifter.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioSrdyFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioSrdyFd = open(srdyGpioCfg.levelshifter.value, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.levelshifter.value);
		debug_printf("[GPIO]%s open failed\n",srdyGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the value of the GPIO to 0 (level shifter direction from CC2531 to Host)

	if (ERROR == write(gpioSrdyFd, "0", 1))
	{
		perror(srdyGpioCfg.levelshifter.value);
		debug_printf("\n[GPIO]can't write in %s \n",srdyGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_LVLSHFT_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioSrdyFd);
	}
	else
	{
		debug_printf("WARNING: Wrong Configuration File, one of the  following Key value are missing for SRDY.Level Shifter definition: '\n");
		debug_printf("value: %s\n", srdyGpioCfg.gpio.value);
		debug_printf("direction: %s\n", srdyGpioCfg.gpio.direction);
		debug_printf("active_high_low: %d\n", srdyGpioCfg.gpio.active_high_low);
		debug_printf("Level Shifter is optional, please check if you need it or not before continuing...\n");
	}
	//TODO: Lock the shift register GPIO.

	//open the SRDY GPIO DIR file
	gpioSrdyFd = open(srdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s open failed\n",srdyGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set SRDY GPIO as input
	if(ERROR == write(gpioSrdyFd, "in", 2))
	{
		perror(srdyGpioCfg.levelshifter.direction);
		debug_printf("\n[GPIO]can't write in %s \n",srdyGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close SRDY DIR file
	close(gpioSrdyFd);

	//open the SRDY GPIO Edge file
	gpioSrdyFd = open(srdyGpioCfg.gpio.edge, O_RDWR);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.edge);
		debug_printf("[GPIO]%s open failed\n",srdyGpioCfg.gpio.edge);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_EDGE_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set SRDY GPIO edge detection for both rising and falling
	if(ERROR == write(gpioSrdyFd, "both", 4))
	{
		perror(srdyGpioCfg.levelshifter.edge);
		debug_printf("\n[GPIO]can't write in %s \n",srdyGpioCfg.gpio.edge);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_EDGE_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close SRDY edge file
	close(gpioSrdyFd);

	//open the SRDY GPIO VALUE file so it can be written to using the file handle later
	gpioSrdyFd = open(srdyGpioCfg.gpio.value, O_RDWR| O_NONBLOCK);
	if(gpioSrdyFd == 0)
	{
		perror(srdyGpioCfg.gpio.value);
		debug_printf("%s open failed\n",srdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}



	return(gpioSrdyFd);
}
/**************************************************************************************************
 * @fn      HalGpioMrdyInit
 *
 *
 * @brief   Initialise MRDY GPIO.
 *
 * @param   gpioCfg - MRDY pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioMrdyInit(halGpioCfg_t *gpioCfg)
{
	memcpy(mrdyGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("[GPIO]resetGpioCfg.gpio.value = '%s'\n", mrdyGpioCfg.gpio.value);
	memcpy(mrdyGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("[GPIO]mrdyGpioCfg.gpio.direction = '%s'\n", mrdyGpioCfg.gpio.direction);
	mrdyGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	if ( ( gpioCfg->levelshifter.value) &&
		 ( gpioCfg->levelshifter.active_high_low) &&
		 ( gpioCfg->levelshifter.direction))
	{


	memcpy(mrdyGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
	debug_printf("[GPIO]mrdyGpioCfg.levelshifter.value = '%s'\n", mrdyGpioCfg.levelshifter.value);
	memcpy(mrdyGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	mrdyGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
	debug_printf("[GPIO]mrdyGpioCfg.levelshifter.direction = '%s'\n", mrdyGpioCfg.levelshifter.direction);

	//open the GPIO DIR file for the level shifter direction signal
	gpioMrdyFd = open(mrdyGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.levelshifter.direction);
		debug_printf("[GPIO]%s open failed\n",mrdyGpioCfg.levelshifter.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the direction of the GPIO to output
	if(ERROR == write(gpioMrdyFd, "out", 3))
	{
		perror(mrdyGpioCfg.levelshifter.direction);
		debug_printf("\n[GPIO]can't write in %s \n",mrdyGpioCfg.levelshifter.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioMrdyFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioMrdyFd = open(mrdyGpioCfg.levelshifter.value, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.levelshifter.value);
		debug_printf("[GPIO]%s open failed\n",mrdyGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
	if(ERROR == write(gpioMrdyFd, "1", 1))
	{
		perror(mrdyGpioCfg.levelshifter.value);
		debug_printf("\n[GPIO]can't write in %s \n",mrdyGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_LVLSHFT_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioMrdyFd);
	}
	else
	{
		debug_printf("WARNING: Wrong Configuration File, one of the  following Key value are missing for MRDY.Level Shifter definition: '\n");
		debug_printf("value: %s\n", srdyGpioCfg.gpio.value);
		debug_printf("direction: %s\n", srdyGpioCfg.gpio.direction);
		debug_printf("active_high_low: %d\n", srdyGpioCfg.gpio.active_high_low);
		debug_printf("Level Shifter is optional, please check if you need it or not before continuing...\n");
	}

	//open the MRDY GPIO DIR file
	gpioMrdyFd = open(mrdyGpioCfg.gpio.direction, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s open failed\n",mrdyGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO as output
	if(ERROR == write(gpioMrdyFd, "out", 3))
	{
		perror(mrdyGpioCfg.gpio.direction);
		debug_printf("\n[GPIO]can't write in %s \n",mrdyGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close MRDY DIR file
	close(gpioMrdyFd);

	//open the MRDY GPIO VALUE file so it can be written to using the file handle later
	gpioMrdyFd = open(mrdyGpioCfg.gpio.value, O_RDWR);
	if(gpioMrdyFd == 0)
	{
		perror(mrdyGpioCfg.gpio.value);
		debug_printf("[GPIO]%s open failed\n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO to 1 as default

	if (ERROR == write(gpioMrdyFd, "1", 3))
	{
		perror(mrdyGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s \n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioResetInit
 *
 *
 * @brief   Initialise RESET GPIO.
 *
 * @param   gpioCfg - Reset pin configuration parameters
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioResetInit(halGpioCfg_t *gpioCfg)
{
	memcpy(resetGpioCfg.gpio.value,
			gpioCfg->gpio.value,
			strlen(gpioCfg->gpio.value));
	debug_printf("[GPIO]resetGpioCfg.gpio.value = '%s'\n", resetGpioCfg.gpio.value);
	memcpy(resetGpioCfg.gpio.direction,
			gpioCfg->gpio.direction,
			strlen(gpioCfg->gpio.direction));
	debug_printf("[GPIO]resetGpioCfg.gpio.direction = '%s'\n", resetGpioCfg.gpio.direction);
	resetGpioCfg.gpio.active_high_low = gpioCfg->gpio.active_high_low;

	if ( ( gpioCfg->levelshifter.value) &&
		 ( gpioCfg->levelshifter.active_high_low) &&
		 ( gpioCfg->levelshifter.direction))
	{


	memcpy(resetGpioCfg.levelshifter.value,
			gpioCfg->levelshifter.value,
			strlen(gpioCfg->levelshifter.value));
	debug_printf("[GPIO]resetGpioCfg.levelshifter.value = '%s'\n", resetGpioCfg.levelshifter.value);
	memcpy(resetGpioCfg.levelshifter.direction,
			gpioCfg->levelshifter.direction,
			strlen(gpioCfg->levelshifter.direction));
	resetGpioCfg.levelshifter.active_high_low = gpioCfg->levelshifter.active_high_low;
	debug_printf("[GPIO]resetGpioCfg.levelshifter.direction = '%s'\n", resetGpioCfg.levelshifter.direction);


	//open the GPIO DIR file for the level shifter direction signal
	gpioResetFd = open(resetGpioCfg.levelshifter.direction, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.levelshifter.direction);
		debug_printf("[GPIO]%s open failed\n",resetGpioCfg.levelshifter.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the direction of the GPIO to output
	if (ERROR == write(gpioResetFd, "out", 3))
	{
		perror(resetGpioCfg.levelshifter.direction);
		debug_printf("\n[GPIO]can't write in %s \n",resetGpioCfg.levelshifter.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioResetFd);

	//open the GPIO VALUE file for the level shifter direction signal
	gpioResetFd = open(resetGpioCfg.levelshifter.value, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.levelshifter.value);
		debug_printf("[GPIO]%s open failed\n",resetGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set the value of the GPIO to 0 (level shifter direction from Host to CC2531)
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.levelshifter.value);
		debug_printf("\n[GPIO]can't write in %s \n",resetGpioCfg.levelshifter.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_LVLSHFT_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close the DIR file
	close(gpioResetFd);
	}
	else
	{
		debug_printf("WARNING: Wrong Configuration File, one of the  following Key value are missing for RESET.Level Shifter definition: '\n");
		debug_printf("value: %s\n", resetGpioCfg.gpio.value);
		debug_printf("direction: %s\n", resetGpioCfg.gpio.direction);
		debug_printf("active_high_low: %d\n", resetGpioCfg.gpio.active_high_low);
		debug_printf("Level Shifter is optional, please check if you need it or not before continuing...\n");
	}

	//open the RESET GPIO DIR file
	gpioResetFd = open(resetGpioCfg.gpio.direction, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.direction);
		debug_printf("[GPIO]%s open failed\n",resetGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_DIR_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO as output
	if(ERROR == write(gpioResetFd, "out", 3))
	{
		perror(resetGpioCfg.gpio.direction);
		debug_printf("\n[GPIO]can't write in %s \n",resetGpioCfg.gpio.direction);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_DIR_WRITE;
	    return NPI_LNX_FAILURE;
	}
	//close MRDY DIR file
	close(gpioResetFd);

	//open the MRDY GPIO VALUE file so it can be writen to using the file handle later
	gpioResetFd = open(resetGpioCfg.gpio.value, O_RDWR);
	if(gpioResetFd == 0)
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("[GPIO]%s open failed\n",resetGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_OPEN;
	    return NPI_LNX_FAILURE;
	}

	//Set MRDY GPIO to 1 as default
	if(ERROR == write(gpioResetFd, "1", 3))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s \n",resetGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE;
	    return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalGpioMrdySet
 *
 *
 * @brief   Set MRDY.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioMrdySet(uint8 state)
{
	if(state == 0)
	{
		debug_printf("[%u][GPIO] MRDY set to low\n", (unsigned int) pthread_self());
		if (ERROR == write(gpioMrdyFd, "0", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE_SET_LOW;
		    return NPI_LNX_FAILURE;
		}
	}
	else
	{
		debug_printf("[%u][GPIO] MRDY set to High\n", (unsigned int) pthread_self());
    	if(ERROR == write(gpioMrdyFd, "1", 1))
		{
			perror(mrdyGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;

}
/**************************************************************************************************
 * @fn      HalGpioMrdyCheck
 *
 *
 * @brief   Check MRDY Clear.
 *
 * @param   state	- Active  or  Inactive
 *
 * @return  None
 **************************************************************************************************/
int HalGpioMrdyCheck(uint8 state)
{
	char mrdy=2;
	lseek(gpioMrdyFd,0,SEEK_SET);
	if(ERROR == read(gpioMrdyFd,&mrdy, 1))
	{
		perror(mrdyGpioCfg.gpio.value);
		debug_printf("\ncan't read in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_MRDY_GPIO_VAL_READ;
		return NPI_LNX_FAILURE;
	}

	debug_printf("[%u][GPIO]===>check MRDY: %c  (%c) \n", (unsigned int) pthread_self(), mrdy, mrdy);

	return (state == ((mrdy == '1') ? 1 : 0));
}

/**************************************************************************************************
 * @fn      HalGpioResetSet
 *
 *
 * @brief   Set Reset.
 *
 * @param
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioResetSet(uint8 state)
{
	if(state == 0)
	{
		debug_printf("[%u][GPIO]RESET set to low\n", (unsigned int) pthread_self());
		if (ERROR == write(gpioResetFd, "0", 1))
		{
			perror(resetGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",resetGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_LOW;
		    return NPI_LNX_FAILURE;
		}
	}
	else
	{
		debug_printf("[%u][GPIO]RESET set to High\n", (unsigned int) pthread_self());
    	if(ERROR == write(gpioResetFd, "1", 1))
		{
			perror(resetGpioCfg.gpio.value);
			debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",resetGpioCfg.gpio.value);
			npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
			return NPI_LNX_FAILURE;
		}
	}

	return NPI_LNX_SUCCESS;

}
/**************************************************************************************************
 * @fn      HalGpioReset
 *
 *
 * @brief   Set MRDY.
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
int HalGpioReset(void)
{
#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	long int diffPrev;
	int t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;
	int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	//	debug_
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] Reset High\n",
			hours,										// hours
			minutes,									// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,	// seconds
			(long int)curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)
	debug_printf("Reset High\n");
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
		return NPI_LNX_FAILURE;
	}
#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;

	hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	//	debug_
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] Reset Low\n",
			hours,										// hours
			minutes,									// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,	// seconds
			(long int)curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)
	debug_printf("[%u][GPIO]Reset low\n", (unsigned int) pthread_self());
	if(ERROR == write(gpioResetFd, "0", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_LOW;
		return NPI_LNX_FAILURE;
	}
	//Add A Delay here:
	// Reset Should last at least 1us from datasheet, set it to 500us.
	usleep(500);

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;
	hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	//	debug_
	printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] Reset High\n",
			hours,										// hours
			minutes,									// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,	// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)
	debug_printf("[%u][GPIO]Reset High\n", (unsigned int) pthread_self());
	if(ERROR == write(gpioResetFd, "1", 1))
	{
		perror(resetGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't write in %s , is something already accessing it? abort everything for debug purpose...\n",mrdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_RESET_GPIO_VAL_WRITE_SET_HIGH;
		return NPI_LNX_FAILURE;
	}

	return NPI_LNX_SUCCESS;
}
/**************************************************************************************************
 * @fn      HalGpioSrdyCheck
 *
 *
 * @brief   Check SRDY Clear.
 *
 * @param   state	- Active  or  Inactive
 *
 * @return  STATUS if error, otherwise boolean TRUE/FALSE if state is matching
 **************************************************************************************************/
int HalGpioSrdyCheck(uint8 state)
{
	char srdy=2;
	lseek(gpioSrdyFd,0,SEEK_SET);
	if(ERROR == read(gpioSrdyFd,&srdy, 1))
	{
		perror(srdyGpioCfg.gpio.value);
		debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_SRDY_GPIO_VAL_READ_FAILED;
		return NPI_LNX_FAILURE;
	}

	debug_printf("[%u][GPIO]===>check SRDY: %c [%d]  (%c) \n", (unsigned int) pthread_self(), srdy, srdy, atoi(&srdy));

	return (state == ((srdy == '1') ? 1 : 0));
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdyClr
 *
 * @brief       Check that SRDY is low, if not, wait until it gets low.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitSrdyClr(void)
{
	char srdy= '1';
	int ret = NPI_LNX_SUCCESS, attempts = 0, accTimeout = 0;

	debug_printf("[%u][GPIO] Wait SRDY Low, \n", (unsigned int) pthread_self());

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLPRI;

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	long int diffPrev;
	int t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;
	int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] SRDY: wait to go Low\n",
			hours,										// hours
			minutes,									// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,	// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);

	struct timeval limitPrints = curTime;
	limitPrints.tv_sec = curTime.tv_sec - 5;
#endif //(defined __DEBUG_TIME__)

	lseek(gpioSrdyFd,0,SEEK_SET);
	read(gpioSrdyFd,&srdy, 1);

	while(srdy == '1')
	{
		pollRet = poll((struct pollfd*)&ufds, 1, HAL_WAIT_SRDY_LOW_INTERMEDIATE_TIMEOUT);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			accTimeout++;
			// Timeout
			lseek(gpioSrdyFd,0,SEEK_SET);
			read(gpioSrdyFd,&srdy, 1);

			if(srdy == '1')
			{
				if (accTimeout >= (HAL_WAIT_SRDY_LOW_TIMEOUT / HAL_WAIT_SRDY_LOW_INTERMEDIATE_TIMEOUT) )
				{
					printf("[%u][GPIO][WARNING] Waiting for SRDY to go low timed out.\n", (unsigned int) pthread_self());
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT;
					ret = NPI_LNX_FAILURE;
					break;
				}
				else
				{
					// This timeout is expected, and ok. Nothing to report, only for debug.
					debug_printf("[%u][GPIO][WARNING] Waiting for SRDY to go low intermediate timed out, %d\n",
							(unsigned int) pthread_self(), accTimeout);
				}
			}
			else
			{
				// Missed interrupt waiting for SRDY to go high
				// This timeout is expected, and ok. Nothing to report, only for debug.
				debug_printf("[%u][GPIO][WARNING] Waiting for SRDY to go low intermediate timed out, %d. However, SRDY is now low\n",
						(unsigned int) pthread_self(), accTimeout);
				break;
			}
		}
		else
		{
			if (ufds[0].revents & POLLPRI)
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_READ_FAILED;
					ret = NPI_LNX_FAILURE;
					break;
				}
#ifdef __DEBUG_TIME__
				gettimeofday(&curTime, NULL);

				// Only print every 250ms
				if ( (curTime.tv_usec > 250000 ) && (((curTime.tv_usec - limitPrints.tv_usec) % 250000) == 0) )
				{
					debug_printf("[0x%.2X , %c (0x%.2X)] ufds[0].revents = 0x%.4X\n", atoi(&srdy), srdy, srdy, ufds[0].revents);
					// Start over
					limitPrints = curTime;
					attempts++;
				}
				// Abort after 1 second
				if (attempts > 4)
				{
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT;
					ret = NPI_LNX_FAILURE;
					break;
				}
#endif //(defined __DEBUG_TIME__)
			}
			else
			{
				printf("[%u][GPIO](%d)\n", (unsigned int) pthread_self(), ufds[0].revents);
			}
		}
	}
#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;
	hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] SRDY: %c  (%d)\n",
			hours,										// hours
			minutes,									// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,	// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev,
			srdy,
			(int)srdy);
#endif //(defined __DEBUG_TIME__)


#ifdef __STRESS_TEST__
  //	debug_
  gettimeofday(&curTime, NULL);
  long int diffPrev;
  int t = 0;
  if (curTime.tv_usec >= prevTimeI2C.tv_usec)
  {
	  diffPrev = curTime.tv_usec - prevTimeI2C.tv_usec;
  }
  else
  {
	  diffPrev = (curTime.tv_usec + 1000000) - prevTimeI2C.tv_usec;
	  t = 1;

  prevTimeI2C = curTime;

  hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
  minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
  time_printf("[%.3d:%.2d:%.2d.%.6ld (+%ld.%6ld)] SRDY Low\n",
		  hours,										// hours
		  minutes,										// minutes
		  (curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
		  curTime.tv_usec,
		  curTime.tv_sec - prevTimeI2C.tv_sec - t,
		  diffPrev);
#endif //__STRESS_TEST__

  debug_printf("[%u][GPIO]==>SRDY change to : %c  (%d) \n", (unsigned int) pthread_self(), srdy, srdy);

  return ret;
}

/**************************************************************************************************
 * @fn          HalGpioWaitSrdySet
 *
 * @brief       Check that SRDY is High, if not, wait until it gets high, or times out.
 * 				0xFFFF means never time out.
 *
 * input parameters
 *
 * None
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int HalGpioWaitSrdySet()
{
	char srdy= '0';

	int ret = NPI_LNX_SUCCESS, accTimeout = 0;

	debug_printf("[%u][GPIO]Wait SRDY High, \n", (unsigned int) pthread_self());

	struct pollfd ufds[1];
	int pollRet;
	ufds[0].fd = gpioSrdyFd;
	ufds[0].events = POLLPRI;

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	long int diffPrev;
	int t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;

	int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] SRDY: wait to go High\n",
			hours,											// hours
			minutes,										// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)

	lseek(gpioSrdyFd,0,SEEK_SET);
	read(gpioSrdyFd,&srdy, 1);

	while( (srdy == '0') )
	{
		// There is still a chance we can miss an interrupt. So we need to split the
		// big HAL_WAIT_SRDY_HIGH_TIMEOUT timeout into smaller timeouts
		pollRet = poll((struct pollfd*)&ufds, 1, HAL_WAIT_SRDY_HIGH_INTERMEDIATE_TIMEOUT);
		if (pollRet == -1)
		{
			// Error occured in poll()
			perror("poll");
		}
		else if (pollRet == 0)
		{
			accTimeout++;
			// Timeout
			lseek(gpioSrdyFd,0,SEEK_SET);
			read(gpioSrdyFd,&srdy, 1);

			if(srdy == '0')
			{
				if (accTimeout >= (HAL_WAIT_SRDY_HIGH_TIMEOUT / HAL_WAIT_SRDY_HIGH_INTERMEDIATE_TIMEOUT) )
				{
					printf("[%u][GPIO][WARNING] Waiting for SRDY to go high timed out.\n", (unsigned int) pthread_self());
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT;
					ret = NPI_LNX_FAILURE;
					break;
				}
				else
				{
					// This timeout is expected, and ok. Nothing to report, only for debug.
					debug_printf("[%u][GPIO][WARNING] Waiting for SRDY to go high intermediate timed out, %d\n",
							(unsigned int) pthread_self(), accTimeout);
				}
			}
			else
			{
				// Missed interrupt waiting for SRDY to go high
				// This timeout is expected, and ok. Nothing to report, only for debug.
				debug_printf("[%u][GPIO][WARNING] Waiting for SRDY to go high intermediate timed out, %d. However, SRDY is now high\n",
						(unsigned int) pthread_self(), accTimeout);
				break;
			}
		}
		else
		{
			if (ufds[0].revents & POLLPRI)
			{
				lseek(gpioSrdyFd,0,SEEK_SET);
				if(ERROR == read(gpioSrdyFd,&srdy, 1))
				{
					perror(srdyGpioCfg.gpio.value);
					debug_printf("\n[GPIO]can't read in %s , is something already accessing it? abort everything for debug purpose...\n",srdyGpioCfg.gpio.value);
					npi_ipc_errno = NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_READ_FAILED;
					return NPI_LNX_FAILURE;
				}
			}
			else
			{
				printf("[%u][GPIO](%d)", (unsigned int) pthread_self(), ufds[0].revents);
			}
		}
	}

#ifdef __DEBUG_TIME__
	gettimeofday(&curTime, NULL);
	t = 0;
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}

	prevTime = curTime;

	hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	time_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] SRDY: %c  (%c)\n",
			hours,											// hours
			minutes,										// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev,
			srdy,
			srdy);
#endif //__DEBUG_TIME__

	debug_printf("[%u][GPIO]==>SRDY change to : %c  (%d) \n", (unsigned int) pthread_self(), srdy, srdy);

	return ret;
}

/**************************************************************************************************
 **************************************************************************************************/
