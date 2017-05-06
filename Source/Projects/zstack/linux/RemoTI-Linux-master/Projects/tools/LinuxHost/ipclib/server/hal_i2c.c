/**************************************************************************************************
  Filename:       hal_i2c.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains the interface to the HAL I2C.


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
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "pthread.h"

//RemoTI Include
//#include  "hal_board.h"
#include  "hal_types.h"
#include  "hal_i2c.h"

#include "npi_lnx_error.h"

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#if (defined NPI_I2C) && (NPI_I2C == TRUE)
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                         GLOBAL VARIABLES
 **************************************************************************************************/
static int i2cDevFd = -1;

#define RNP_I2C_ADDRESS 0x41

pthread_mutex_t I2cMutex1 = PTHREAD_MUTEX_INITIALIZER;

/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalI2cInit
 *
 * @brief   Initialize I2C Service
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
int HalI2cInit(const char *devpath)
{
	// If the device is open, attempt to close it first.
	if (i2cDevFd >= 0)
	{
		/* The device is open, attempt to close it first */
		debug_printf("Closing %s ...\n",devpath);
		close(i2cDevFd);
	}

	/* open the device */
	debug_printf("Opening %s ...\n",devpath);

	i2cDevFd = open(devpath, O_RDWR | O_NONBLOCK);
	if(i2cDevFd<0)
	{
		fprintf(stderr, "Error: Could not open file "
				"%s: %s, already used?\n", devpath, strerror(errno));
		npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_INIT_FAILED_TO_OPEN_DEVICE;
		return NPI_LNX_FAILURE;
	}

	if(ioctl(i2cDevFd, I2C_SLAVE, RNP_I2C_ADDRESS) < 0)
	{
		fprintf(stderr,
				"Error: Could not set address to 0x%02x: %s\n",
				RNP_I2C_ADDRESS, strerror(errno));
		close(i2cDevFd);
		npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_INIT_FAILED_TO_SET_ADDRESS;
		return NPI_LNX_FAILURE;
	}

	debug_printf("Open I2C Driver: %s for Address 0x%.2x ...\n",devpath, RNP_I2C_ADDRESS);

	return NPI_LNX_SUCCESS;
}
/**************************************************************************************************
 * @fn      HalI2cClose
 *
 * @brief   Close I2C Service
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
int HalI2cClose(void)
{
	debug_printf("Closing I2C file descriptor\n");
	if (-1 ==close(i2cDevFd))
	{
		fprintf(stderr, "Error: Could not Close I2C device file, reason:  %s,\n", strerror(errno));

	    npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_CLOSE_GENERIC;
	    return NPI_LNX_FAILURE;
	}
	else
		return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalI2cWrite
 *
 * @brief   Write a buffer to the I2C Slave.
 *
 * @param   port - I2C port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write/read.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalI2cWrite(uint8 port, uint8 *pBuf, uint8 len)
{
  int res, ret = NPI_LNX_SUCCESS;
  int timeout = I2C_OPEN_100MS_TIMEOUT;
  int i;
  debug_printf("I2C Write: \t");

  for(i=0; i<len; i++)
	  debug_printf("0x%.2x ", pBuf[i]);

  debug_printf("\n");

  pthread_mutex_lock(&I2cMutex1);

  while( (-1 == (res = write(i2cDevFd,pBuf,len))) && timeout)
  {
    timeout--;
    debug_printf("I2C Write timeout %d, %s \n", timeout, strerror(errno));

    if ((errno == ETIMEDOUT) && (timeout < (I2C_OPEN_100MS_TIMEOUT - 2)) )
    {
    	// When ETIMEDOUT is received the function did not immediately return, so the 100 ms timeout
    	// is not accurate. Hence, cut it short after 3 attempts.
    	timeout = 0;
    }
    else
    {
    	usleep(1000); //Wait 1ms before re-trying
    }
  }

  if ( (-1 == res) && !timeout)
  {
    /* ERROR HANDLING: i2c transaction failed */
    printf("Failed to write to the i2c bus for %d ms. read %d byte(s)...reason: (#%d)%s\n", I2C_OPEN_100MS_TIMEOUT, res, errno, strerror(errno));
    if (errno == ETIMEDOUT)
    {
    	// RNP may be hung, report error and perform reset.
    	printf("May have to reset RNP...\n");
        npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_WRITE_TIMEDOUT_PERFORM_RESET;
    }
    else
    {
    	printf("RNP may have reset unexpectedly. Keep going...\n");
        npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_WRITE_TIMEDOUT;
    }
    printf("\n\n");
    ret = NPI_LNX_FAILURE;
  }
  else
  {
	  debug_printf("I2C Wrote: %d bytes\n", res);
  }

  pthread_mutex_unlock(&I2cMutex1);
  return ret;
}

/**************************************************************************************************
 * @fn      HalI2cRead
 *
 * @brief   Read a buffer from the I2C Slave.
 *
 * @param   port - I2C port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write/read.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalI2cRead(uint8 port, uint8 *pBuf, uint8 len)
{
  int timeout = I2C_OPEN_100MS_TIMEOUT, ret = NPI_LNX_SUCCESS;
  int i;
  int res = -1;
  pthread_mutex_lock(&I2cMutex1);

  while( (-1 == (res = read(i2cDevFd,pBuf,len))) && timeout)
  {
    timeout--;
    printf("I2C Read timeout %d, %s \n", timeout, strerror(errno));

    if ((errno == ETIMEDOUT) && (timeout < (I2C_OPEN_100MS_TIMEOUT - 2)) )
    {
    	// When ETIMEDOUT is received the function did not immediately return, so the 100 ms timeout
    	// is not accurate. Hence, cut it short after 3 attempts.
    	timeout = 0;
    }
    else
    {
    	usleep(1000); //Wait 1ms before re-trying
    }
  }

  if ( (-1 == res) && !timeout)
  {
    /* ERROR HANDLING: i2c transaction failed */
    printf("Failed to read to the i2c bus for %d ms. read %d byte(s)...reason: (#%d)%s\n",
    		I2C_OPEN_100MS_TIMEOUT,
    		res,
    		errno,
    		strerror(errno));
    if (errno == ETIMEDOUT)
    {
    	// RNP may be hung, report error and perform reset.
    	printf("May have to reset RNP...\n");
        npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_READ_TIMEDOUT_PERFORM_RESET;
    }
    else
    {
    	printf("RNP may have reset unexpectedly. Keep going...\n");
        npi_ipc_errno = NPI_LNX_ERROR_HAL_I2C_READ_TIMEDOUT;
    }
    printf("\n");
    ret = NPI_LNX_FAILURE;
  }
  else
  {
	  debug_printf("I2C Read: \t");

	  for(i=0; i<len; i++)
		  debug_printf("0x%.2x ", pBuf[i]);

	  debug_printf("\n");
  }

  pthread_mutex_unlock(&I2cMutex1);

  return ret;
}

#endif

/**************************************************************************************************
**************************************************************************************************/
