/**************************************************************************************************
  Filename:       hal_spi.c
  Revised:        $Date: 2012-03-02 08:19:21 -0800 (Fri, 02 Mar 2012) $
  Revision:       $Revision: 217 $

  Description:    This file contains the interface to the HAL SPI.


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
#include <linux/spi/spidev.h>

#include "pthread.h"

#include  "hal_types.h"
#include  "hal_spi.h"

#include "npi_lnx_error.h"


#if (defined NPI_SPI) && (NPI_SPI == TRUE)

#if (defined __DEBUG_TIME__)
#include <sys/time.h>
#endif // __STRESS_TEST__

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif
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
static int spiDevFd;

#if (defined __DEBUG_TIME__)
struct timeval curTime, prevTime;
extern struct timeval startTime;
#endif //__STRESS_TEST__

static uint8 bits = 8;
static uint32 speed = 500000; //Hz
static uint16 delay = 0;
static uint8 useFullDuplexAPI = TRUE;

pthread_mutex_t spiMutex1 = PTHREAD_MUTEX_INITIALIZER;
/**************************************************************************************************
 *                                          FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalSpiFlush
 *
 * @brief   Write a buffer to the SPI.
 *
 * @param   port - SPI port.
 *          len - Number of bytes to flush.
 *
 * @return  None
 **************************************************************************************************/
void HalSpiFlush(uint8 port, uint8 len)
{

}

/**************************************************************************************************
 * @fn      HalSpiInit
 *
 * @brief   Initialize SPI Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
int HalSpiInit(const char *devpath, halSpiCfg_t *halSpiCfg)
{
  /* open the device */
  int ret;
  uint8_t mode = halSpiCfg->mode;
  uint8_t bits = halSpiCfg->bitsPerWord;
  speed = halSpiCfg->speed;
  useFullDuplexAPI = halSpiCfg->useFullDuplexAPI;

#ifdef __BIG_DEBUG__
  printf("Opening %s ...\n",devpath);
#endif
  spiDevFd = open(devpath, O_RDWR );
  if (spiDevFd <0)
  {
    perror(devpath);
    printf("%s open failed\n",devpath);
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_OPEN_DEVICE;
    return NPI_LNX_FAILURE;
  }

/*
* spi mode
*/
  ret = ioctl(spiDevFd, SPI_IOC_WR_MODE, &mode);
  if (ret < 0 )
  {
    perror("can't set spi mode\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_SET_MODE;
    return NPI_LNX_FAILURE;
  }

/*
 * bits per word
 */
  ret = ioctl(spiDevFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret < 0 )
  {
    perror("can't set bits per word\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_SET_BPW;
    return NPI_LNX_FAILURE;
  }

/*
 * max speed hz
 */
  ret = ioctl(spiDevFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret < 0 )
  {
    perror("can't set max speed hz\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_SET_MAX_SPEED;
    return NPI_LNX_FAILURE;
  }

/*
 * Read back for verification
 */
  ret = ioctl(spiDevFd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret < 0 )
  {
    perror("can't get bits per word\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_GET_BPW;
    return NPI_LNX_FAILURE;
  }

  ret = ioctl(spiDevFd, SPI_IOC_RD_MODE, &mode);
  if (ret < 0 )
  {
    perror("can't get spi mode\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_GET_MODE;
    return NPI_LNX_FAILURE;
  }

  ret = ioctl(spiDevFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret < 0 )
  {
    perror("can't get max speed hz\n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_INIT_FAILED_TO_GET_MAX_SPEED;
    return NPI_LNX_FAILURE;
  }

  debug_printf("[HAL SPI] spi mode: 0x%02X\n", mode);
  debug_printf("[HAL SPI] bits per word: %d\n", bits);
  debug_printf("[HAL SPI] max speed: %d Hz (%d KHz)\n", speed, speed/1000);

  return NPI_LNX_SUCCESS;
}

/**************************************************************************************************
 * @fn      HalSpiClose
 *
 * @brief   Close SPI Service
 *
 * @param   None
 *
 * @return  STATUS
 **************************************************************************************************/
void HalSpiClose( void )
{
    close(spiDevFd);
}

/**************************************************************************************************
 * @fn      HalSpiPoll
 *
 * @brief   Poll the SPI.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiPoll(void)
{
}

/**************************************************************************************************
 * @fn      HalSpiWrite
 *
 * @brief   Write a to the SPI. Half duplex API. Read is ignored
 *
 * @param   port - SPI port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalSpiWrite(uint8 port, uint8 *pBuf, uint8 len)
{
	int ret;
	if (useFullDuplexAPI == TRUE)
	{
		ret = HalSpiWriteRead(port, pBuf, len);
	}
	else
	{
#ifdef __BIG_DEBUG__
		uint8 i;
#endif
		(void)port;

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
		debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- WRITE SPI LOCK MUTEX ---------\n",
				hours,											// hours
				minutes,										// minutes
				(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif //(defined __DEBUG_TIME__)
		pthread_mutex_lock(&spiMutex1);
#ifdef __BIG_DEBUG__
		printf("SPI: Sending ...");
		for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",tx[i]);
		printf("\n");
#endif

		ret = write(spiDevFd, pBuf, len);
		if (ret < 0 )
		{
			perror("can't write to SPI \n");
			npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_WRITE_FAILED;
			ret = NPI_LNX_FAILURE;
		}
		else if (ret == len)
		{
			ret = NPI_LNX_SUCCESS;
		}
		else
		{
			npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_WRITE_FAILED_INCORRECT_NUM_OF_BYTES;
			ret = NPI_LNX_FAILURE;
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
		debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- WRITE SPI DONE ---------------\n",
				hours,											// hours
				minutes,										// minutes
				(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif //(defined __DEBUG_TIME__)

		pthread_mutex_unlock(&spiMutex1);

	}
	return ret;
}

/**************************************************************************************************
 * @fn      HalSpiWriteRead
 *
 * @brief   Read a buffer from the SPI. Half duplex API.
 *
 * @param   port - SPI port.
 *          pBuf - Pointer to the buffer that will be read to.
 *          len - Number of bytes to read.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalSpiRead(uint8 port, uint8 *pBuf, uint8 len)
{
	int ret;
	if (useFullDuplexAPI == TRUE)
	{
		ret = HalSpiWriteRead(port, pBuf, len);
	}
	else
	{
#ifdef __BIG_DEBUG__
		uint8 i;
#endif
		(void)port;

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
		debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- READ SPI LOCK MUTEX ---------\n",
				hours,											// hours
				minutes,										// minutes
				(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif //(defined __DEBUG_TIME__)
		pthread_mutex_lock(&spiMutex1);

#ifdef __BIG_DEBUG__
		printf("SPI: Receive ...");
		for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",rx[i]);
		printf("\n");
#endif

		ret = read(spiDevFd, pBuf, len);
		if (ret < 0 )
		{
			perror("can't read from SPI \n");
			npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_READ_FAILED;
			ret = NPI_LNX_FAILURE;
		}
		else if (ret == len)
		{
			ret = NPI_LNX_SUCCESS;
		}
		else
		{
			npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_READ_FAILED_INCORRECT_NUM_OF_BYTES;
			ret = NPI_LNX_FAILURE;
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
		debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- READ SPI DONE ---------------\n",
				hours,											// hours
				minutes,										// minutes
				(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif //(defined __DEBUG_TIME__)

		pthread_mutex_unlock(&spiMutex1);
	}
	return ret;
}

/**************************************************************************************************
 * @fn      HalSpiWriteRead
 *
 * @brief   Write and Read a buffer to/from the SPI. Full duplex API.
 *
 * @param   port - SPI port.
 *          pBuf - Pointer to the buffer that will be written.
 *          len - Number of bytes to write/read.
 *
 * @return  STATUS
 **************************************************************************************************/
int HalSpiWriteRead(uint8 port, uint8 *pBuf, uint8 len)
{
  uint8* tx;
  uint8* rx;
  int ret;
#ifdef __BIG_DEBUG__
  uint8 i;
#endif
  (void)port;
  struct spi_ioc_transfer tr;
  tx = pBuf;
  rx = malloc (len);

  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = len;
  tr.delay_usecs = delay;
  tr.speed_hz = speed;
  tr.bits_per_word = bits;

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
	debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- WRITE_READ SPI LOCK MUTEX ---------\n",
			hours,											// hours
			minutes,										// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)
  pthread_mutex_lock(&spiMutex1);
#ifdef __BIG_DEBUG__
  printf("SPI: Sending ...");
  for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",tx[i]);
  printf("\n");
#endif

  ret = ioctl(spiDevFd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 0 )
  {
    perror("can't write to SPI \n");
    npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_WRITE_READ_FAILED;
    ret = NPI_LNX_FAILURE;
  }
  else if (ret == len)
  {
	  ret = NPI_LNX_SUCCESS;
  }
  else
  {
	  npi_ipc_errno = NPI_LNX_ERROR_HAL_SPI_WRITE_READ_FAILED_INCORRECT_NUM_OF_BYTES;
	  ret = NPI_LNX_FAILURE;
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
	debug_printf("[%.3d:%.2d:%.2ld.%.6ld (+%ld.%6ld)] ----- WRITE_READ SPI DONE ---------------\n",
			hours,											// hours
			minutes,										// minutes
			(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#endif //(defined __DEBUG_TIME__)

  memcpy(pBuf, rx, len);
#ifdef __BIG_DEBUG__
  printf("SPI: Receive ...");
  for (i = 0 ; i < len; i++ ) printf(" 0x%.2x",rx[i]);
  printf("\n");
#endif
  free(rx);
  pthread_mutex_unlock(&spiMutex1);

  return ret;
}

#endif

/**************************************************************************************************
**************************************************************************************************/
