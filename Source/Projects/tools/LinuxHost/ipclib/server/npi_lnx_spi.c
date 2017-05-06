/**************************************************************************************************
  Filename:       npi_lnx_spi.c
  Revised:        $Date: 2012-03-15 13:45:31 -0700 (Thu, 15 Mar 2012) $
  Revision:       $Revision: 237 $

  Description:    This file contains linux specific implementation of Network Processor Interface
                  module.


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

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/input.h>
#include <poll.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_spi.h"
#include "hal_rpc.h"
#include "hal_gpio.h"

#include "npi_lnx_error.h"

#ifdef __STRESS_TEST__
#include <sys/time.h>
#elif (defined __DEBUG_TIME__)
#include <sys/time.h>
#else
#include <sys/time.h>
#endif // __STRESS_TEST__

// -- macros --
#if (defined NPI_SPI) && (NPI_SPI == TRUE)

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__); fflush(stdout);
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

// -- Constants --

// -- Local Variables --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;
static uint8 earlyMrdyDeAssert = TRUE;
static uint8 detectResetFromSlowSrdyAssert = TRUE;
static uint8 forceRun = NPI_LNX_UINT8_ERROR;
static uint8 srdyMrdyHandshakeSupport = TRUE;

// NPI device related variables
static int              npi_poll_terminate;
static pthread_mutex_t  npiPollLock;
static pthread_mutex_t  npi_poll_mutex;
static int 				GpioSrdyFd;
#ifndef SRDY_INTERRUPT
static pthread_cond_t   npi_poll_cond;
#endif

// Polling thread
//---------------
static pthread_t        npiPollThread;
static int     			PollLockVar = 0;

// thread subroutines
static void npi_termpoll(void);
static void *npi_poll_entry(void *ptr);


#ifdef SRDY_INTERRUPT
// Event thread
//---------------
static pthread_t        npiEventThread;
static void *npi_event_entry(void *ptr);
static int global_srdy;

static pthread_cond_t   npi_srdy_H2L_poll;

static pthread_mutex_t  npiSrdyLock;
#define INIT 0
#define READY 1
#endif

// -- Forward references of local functions --

// -- Public functions --

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
struct timeval prevTimeI2C;
#elif (defined __DEBUG_TIME__)
struct timeval curTime, prevTime;
extern struct timeval startTime;
#else
struct timeval curTime, prevTime;
#endif //__STRESS_TEST__

struct timeval curTimeSPIisrPoll, prevTimeSPIisrPoll;

// -- Private functions --
static int npi_initsyncres(void);
static int npi_initThreads(void);
void time_printf(char *strToPrint);

/******************************************************************************
 * @fn         time_printf
 *
 * @brief      This function adds timestamp to a printf.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return
 *
 * None.
 ******************************************************************************
 */
void time_printf(char *strToPrint)
{
	long int diffPrev;
	int t = 0;

	gettimeofday(&curTime, NULL);
	if (curTime.tv_usec >= prevTime.tv_usec)
	{
		diffPrev = curTime.tv_usec - prevTime.tv_usec;
	}
	else
	{
		diffPrev = (curTime.tv_usec + 1000000) - prevTime.tv_usec;
		t = 1;
	}
	int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
	int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
	debug_printf("[%.3d:%.2d:%.2d.%.6ld (+%ld.%6ld)] %s\n",
			hours,											// hours
			minutes,										// minutes
			(int)(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
			(long int)curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev,
			strToPrint);
	prevTime = curTime;
}

/******************************************************************************
 * @fn         PollLockVarError
 *
 * @brief      This function kill the program due to a major Mutex problem.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return
 *
 * None.
 ******************************************************************************
 */
int PollLockVarError(int originator)
{
    printf("PollLock Var ERROR, it is %d, it should be %d. Called by 0x%.8X\n", PollLockVar, !PollLockVar, originator);
    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_LOCK_VAR_ERROR;
    return NPI_LNX_FAILURE;
}
/******************************************************************************
 * @fn         NPI_OpenDevice
 *
 * @brief      This function establishes a serial communication connection with
 *             a network processor device.
 *             As windows machine does not have a single dedicated serial
 *             interface, this function will designate which serial port shall
 *             be used for communication.
 *
 * input parameters
 *
 * @param   portName 	– name of the serial port
 * @param	gpioCfg		– GPIO settings for SRDY, MRDY and RESET
 *
 * output parameters
 *
 * None.
 *
 * @return     TRUE if the connection is established successfully.
 *             FALSE, otherwise.
 ******************************************************************************
 */
int NPI_SPI_OpenDevice(const char *portName, void *pCfg)
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_OPEN_DEVICE;

	if(npiOpenFlag)
	{
		npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_ALREADY_OPEN;
		return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	debug_printf("Opening Device File: %s\n", portName);


	// Setup parameters that differ between ZNP and RNP
	earlyMrdyDeAssert = ((npiSpiCfg_t *)pCfg)->earlyMrdyDeAssert;
	detectResetFromSlowSrdyAssert = ((npiSpiCfg_t *)pCfg)->detectResetFromSlowSrdyAssert;
	forceRun = ((npiSpiCfg_t *)pCfg)->forceRunOnReset;
	srdyMrdyHandshakeSupport = ((npiSpiCfg_t *)pCfg)->srdyMrdyHandshakeSupport;

	ret = HalSpiInit(portName, ((npiSpiCfg_t*)pCfg)->spiCfg);
	if (ret != NPI_LNX_SUCCESS)
	{
		return ret;
	}

	debug_printf("((npiSpiCfg_t *)pCfg)->gpioCfg[0] \t @%p\n",
			(void *)&(((npiSpiCfg_t *)pCfg)->gpioCfg[0]));

	if ( NPI_LNX_FAILURE == (GpioSrdyFd = HalGpioSrdyInit(((npiSpiCfg_t *)pCfg)->gpioCfg[0])))
	{
		return GpioSrdyFd;
	}
	if ( NPI_LNX_FAILURE == (ret = HalGpioMrdyInit(((npiSpiCfg_t *)pCfg)->gpioCfg[1])))
	{
		return ret;
	}
	if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit(((npiSpiCfg_t *)pCfg)->gpioCfg[2])))
	{
		return ret;
	}

	// initialize thread synchronization resources
	if ( NPI_LNX_FAILURE == (ret = npi_initsyncres()))
	{
		return ret;
	}


	//Polling forbid until the Reset and Sync is done
	debug_printf("LOCK POLL WHILE INIT\n");
	pthread_mutex_lock(&npiPollLock);
	if (PollLockVar)
	{
		ret = PollLockVarError(funcID++);
	}
	else
	{
		PollLockVar = 1;
		debug_printf("[SPI POLL] PollLockVar set to %d\n", PollLockVar);
	}

	debug_printf("PollLockVar = %d\n", PollLockVar);

	// TODO: it is ideal to make this thread higher priority
	// but Linux does not allow real time of FIFO scheduling policy for
	// non-privileged threads.

	if (ret == NPI_LNX_SUCCESS)
	{
		// create Polling thread
		ret = npi_initThreads();
	}
	else
	{
		debug_printf("Did not attempt to start Threads\n");
	}

	return ret;
}


/******************************************************************************
 * @fn         NPI_SPI_CloseDevice
 *
 * @brief      This function closes connection with a network processor device
 *
 * input parameters
 *
 * @param      pDevice   - pointer to a device data structure
 *
 * output parameters
 *
 * None.
 *
 * @return     None
 ******************************************************************************
 */
void NPI_SPI_CloseDevice(void)
{
  npi_termpoll();
  HalSpiClose();
  HalGpioSrdyClose();
  HalGpioMrdyClose();
  HalGpioResetClose();
  npiOpenFlag = FALSE;
}

/**************************************************************************************************
 * @fn          NPI_SPI_SendAsynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent asynchronously. This routine allocates an AREQ buffer,
 *              copies the client's payload, and sets up the send.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent asynchronously (i.e. AREQ).
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_SendAsynchData( npiMsgData_t *pMsg )
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_SEND_ASYNCH;
	debug_printf("[ASYNCH] Sync Lock SRDY ...");
	fflush(stdout);
	//Lock the polling until the command is send
	pthread_mutex_lock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	if (PollLockVar)
	{
		ret = PollLockVarError(funcID++);
	}
	else
	{
		PollLockVar = 1;
		debug_printf("[ASYNCH][SPI POLL] PollLockVar set to %d\n", PollLockVar);
	}
	debug_printf("[ASYNCH] (Sync) success \n");

	debug_printf("\n******************** START SEND ASYNC DATA ********************\n");
	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	if (ret == NPI_LNX_SUCCESS)
		if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			return ret;

	debug_printf("[AREQ]");

	//Wait for SRDY Clear
	ret = HalGpioWaitSrdyClr();

	if (ret == NPI_LNX_SUCCESS)
		ret = HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if (!PollLockVar)
	{
		ret = PollLockVarError(funcID++);
	}
	else
	{
		PollLockVar = 0;
		debug_printf("[ASYNCH] [SPI POLL] PollLockVar set to %d\n", PollLockVar);
	}
	pthread_mutex_unlock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
#endif
	debug_printf("[ASYNCH] Sync unLock SRDY ...\n\n");
	debug_printf("\n******************** STOP SEND ASYNC DATA ********************\n");

	return ret;
}

/**************************************************************************************************
 * @fn          npi_spi_pollData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent synchronously. This routine allocates a SREQ buffer,
 *              copies the client's payload, sends the data, and waits for the
 *              reply. The input buffer is used for the output data.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent synchronously (i.e. the SREQ).
 *
 * output parameters
 *
 * @param *pMsg  - Pointer to replay data (i.e. the SRSP).
 *
 * @return      STATUS
 **************************************************************************************************
 */
int npi_spi_pollData(npiMsgData_t *pMsg)
{
	int i, ret = NPI_LNX_SUCCESS;
	debug_printf("\n-------------------- START POLLING DATA --------------------\n");

#ifdef __BIG_DEBUG__
	printf("Polling Command ...");

	for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
		printf(" 0x%.2x", ((uint8*)pMsg)[i]);

	printf("\n");
#endif

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
	}

	prevTimeI2C = curTime;

	printf("[--> %.5ld.%.6ld (+%ld.%6ld)] MRDY Low \n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTimeI2C.tv_sec - t,
			diffPrev);
#endif //__STRESS_TEST__

	if (ret == NPI_LNX_SUCCESS)
		if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			return ret;

	ret = HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

	struct timeval t1, t2;

	gettimeofday(&t1, NULL);
#if __BIG_DEBUG__
	printf("[POLL] %.5ld.%.6ld]\n", t1.tv_sec, t1.tv_usec);
#endif

	int bigDebugWas = __BIG_DEBUG_ACTIVE;
	if (bigDebugWas == FALSE)
	{
		__BIG_DEBUG_ACTIVE = FALSE;
	}
	//Wait for SRDY set
	if (ret == NPI_LNX_SUCCESS)
		ret = HalGpioWaitSrdySet();

	// Check how long it took to wait for SRDY to go High. May indicate that this Poll was considered
	// a handshake by the RNP.
	gettimeofday(&t2, NULL);
	long int diffPrev;
	if (t2.tv_usec >= t1.tv_usec)
	{
		diffPrev = t2.tv_usec - t1.tv_usec;
		diffPrev += (t2.tv_sec - t1.tv_sec) * 1000000;
	}
	else
	{
		diffPrev = (t2.tv_usec + 1000000) - t1.tv_usec;
		diffPrev += (t2.tv_sec - t1.tv_sec - 1) * 1000000;
	}

	if (detectResetFromSlowSrdyAssert == TRUE)
	{
		// If it took more than NPI_LNX_SPI_NUM_OF_MS_TO_DETECT_RESET_AFTER_SLOW_SRDY_ASSERT ms
		// then it's likely a reset handshake.
		if (diffPrev > (NPI_LNX_SPI_NUM_OF_MS_TO_DETECT_RESET_AFTER_SLOW_SRDY_ASSERT * 1000) )
		{
			debug_printf("[POLL] SRDY took %ld us to go high\n", diffPrev);
			npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_DATA_SRDY_CLR_TIMEOUT_POSSIBLE_RESET;
			return NPI_LNX_FAILURE;
		}
	}

	if (earlyMrdyDeAssert == TRUE)
	{
		//We Set MRDY here to avoid GPIO latency with the beagle board
		// if we do here later, the RNP see it low at the end of the transaction and
		// therefore think a new transaction is starting and lower its SRDY...
		if (ret == NPI_LNX_SUCCESS)
			ret = HAL_RNP_MRDY_SET();
		else
			(void)HAL_RNP_MRDY_SET();
	}
	__BIG_DEBUG_ACTIVE = bigDebugWas;

	//Do a Three Byte Dummy Write to read the RPC Header
	for (i = 0 ;i < RPC_FRAME_HDR_SZ; i++ ) ((uint8*)pMsg)[i] = 0;
	if (ret == NPI_LNX_SUCCESS)
		ret = HalSpiRead( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);


	//Do a write/read of the corresponding length
	for (i = 0 ;i < ((uint8*)pMsg)[0]; i++ ) ((uint8*)pMsg)[i+RPC_FRAME_HDR_SZ] = 0;
	if ( (ret == NPI_LNX_SUCCESS) && (((uint8*)pMsg)[0] > 0) )
	{
		ret = HalSpiRead( 0, pMsg->pData, ((uint8*)pMsg)[0]);
	}

#ifdef __BIG_DEBUG__
	if (TRUE == HAL_RNP_SRDY_CLR())
		printf("SRDY set\n");
	else
		printf("SRDY Clear\n");
#endif

#ifdef __BIG_DEBUG__
	printf("Poll Response Received ...");
	for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	printf("\n");
#endif

	if (earlyMrdyDeAssert == FALSE)
	{
		if (ret == NPI_LNX_SUCCESS)
		{
			ret = HAL_RNP_MRDY_SET();
		}
		else
		{
			(void)HAL_RNP_MRDY_SET();
		}
	}

	debug_printf("\n-------------------- END POLLING DATA --------------------\n");

	return ret;
}

/**************************************************************************************************
 * @fn          NPI_SPI_SendSynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent synchronously. This routine allocates a SREQ buffer,
 *              copies the client's payload, sends the data, and waits for the
 *              reply. The input buffer is used for the output data.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent synchronously (i.e. the SREQ).
 *
 * output parameters
 *
 * @param *pMsg  - Pointer to replay data (i.e. the SRSP).
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_SendSynchData( npiMsgData_t *pMsg )
{
	int i, ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_SEND_SYNCH;
	// Do not attempt to send until polling is finished

	int lockRetPoll = 0, lockRetSrdy = 0;
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		time_printf("[SYNCH] Lock POLL mutex");
	}
	//Lock the polling until the command is send
	lockRetPoll = pthread_mutex_lock(&npiPollLock);
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		time_printf("[SYNCH] POLL mutex locked");
	}
#ifdef SRDY_INTERRUPT
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		time_printf("[SYNCH] Lock SRDY mutex");
	}
	lockRetSrdy = pthread_mutex_lock(&npiSrdyLock);
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		time_printf("[SYNCH] SRDY mutex locked");
	}
#endif
	if (PollLockVar)
	{
		ret = PollLockVarError(funcID++);
	}
	else
	{
		PollLockVar = 1;
		debug_printf("[SYNCH] PollLockVar set to %d\n", PollLockVar);
	}
	debug_printf("[SYNCH] (Sync) success \n");
	debug_printf("==================== START SEND SYNC DATA ====================\n");
	if (lockRetPoll != 0)
	{
		printf("[SYNCH] [ERR] Could not get POLL mutex lock\n");
		perror("mutex lock");
	}
	if (lockRetSrdy != 0)
	{
		printf("[SYNCH] [ERR] Could not get SRDY mutex lock\n");
		perror("mutex lock");
	}

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;


#ifdef __BIG_DEBUG__
	if (TRUE == HAL_RNP_SRDY_CLR())
		printf("[SYNCH] SRDY set\n");
	else
		printf("[SYNCH] SRDY Clear\n");
#endif

	if (ret == NPI_LNX_SUCCESS)
		if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			return ret;

	//Wait for SRDY Clear
	ret = HalGpioWaitSrdyClr();

	debug_printf("[SYNCH] Sync Data Command ...");
	for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) debug_printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	debug_printf("\n");
	if (ret == NPI_LNX_SUCCESS)
		ret = HalSpiWrite( 0, (uint8*) pMsg, (pMsg->len)+RPC_FRAME_HDR_SZ);

	debug_printf("[SYNCH] [SREQ]");
	//Wait for SRDY set
	if (ret == NPI_LNX_SUCCESS)
		ret = HalGpioWaitSrdySet();

	if (earlyMrdyDeAssert == TRUE)
	{
		//We Set MRDY here to avoid GPIO latency with the beagle board
		// if we do here later, the RNP see it low at the end of the transaction and
		// therefore think a new transaction is starting and lower its SRDY...
		if (ret == NPI_LNX_SUCCESS)
			ret = HAL_RNP_MRDY_SET();
		else
			(void)HAL_RNP_MRDY_SET();
	}

	//Do a Three Byte Dummy Write to read the RPC Header
	for (i = 0 ;i < RPC_FRAME_HDR_SZ; i++ ) ((uint8*)pMsg)[i] = 0;
	if (ret == NPI_LNX_SUCCESS)
	{
		ret = HalSpiRead( 0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);
	}
	debug_printf("[SYNCH] Read %d bytes ...", RPC_FRAME_HDR_SZ);
	for (i = 0 ; i < (RPC_FRAME_HDR_SZ); i++ ) debug_printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	debug_printf("\n");

	//Do a write/read of the corresponding length
	for (i = 0 ;i < ((uint8*)pMsg)[0]; i++ ) ((uint8*)pMsg)[i+RPC_FRAME_HDR_SZ] = 0;
	if ( (ret == NPI_LNX_SUCCESS) && (((uint8*)pMsg)[0] > 0) )
	{
		ret = HalSpiRead( 0, pMsg->pData, ((uint8*)pMsg)[0]);
	}

	debug_printf("[SYNCH] Read %d more bytes ...", ((uint8*)pMsg)[0]);
	for (i = RPC_FRAME_HDR_SZ ; i < (pMsg->len); i++ ) debug_printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	debug_printf("\n");

	if (earlyMrdyDeAssert == FALSE)
	{
		//End of transaction
		if (ret == NPI_LNX_SUCCESS)
		{
			ret = HAL_RNP_MRDY_SET();
		}
		else
		{
			(void)HAL_RNP_MRDY_SET();
		}
	}

#ifdef __BIG_DEBUG__
	if (TRUE == HAL_RNP_SRDY_CLR())
		printf("[SYNCH] SRDY set\n");
	else
		printf("[SYNCH] SRDY Clear\n");
#endif

#ifdef __BIG_DEBUG__
	printf("[SYNCH] Sync Data Receive ...");
	for (i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++ ) printf(" 0x%.2x", ((uint8*)pMsg)[i]);
	printf("\n");
#endif

	//Release the polling lock
    debug_printf("\n==================== END SEND SYNC DATA ====================\n");
	if (!PollLockVar)
	{
		ret = PollLockVarError(funcID++);
	}
	else
	{
		PollLockVar = 0;
		debug_printf("[SYNCH] PollLockVar set to %d\n", PollLockVar);
	}
	pthread_mutex_unlock(&npiPollLock);
#ifdef SRDY_INTERRUPT
    pthread_mutex_unlock(&npiSrdyLock);
#endif
    debug_printf("[SYNCH] Sync unLock SRDY ...\n\n");

	return ret;
}


/**************************************************************************************************
 * @fn          NPI_SPI_ResetSlave
 *
 * @brief       do the HW synchronization between the host and the RNP
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_ResetSlave( void )
{
	int ret = NPI_LNX_SUCCESS;

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

	//	debug_
	printf("[%.5ld.%.6ld (+%ld.%6ld)] ----- START RESET SLAVE ------------\n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#else //(!defined __DEBUG_TIME__)
  printf("\n\n-------------------- START RESET SLAVE -------------------\n");
#endif //(defined __DEBUG_TIME__)

  ret = HalGpioReset();

  if (forceRun != NPI_LNX_UINT8_ERROR)
  {
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalGpioWaitSrdyClr();
	  }
	  //Send force run command
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalSpiWrite( 0, &forceRun, 1);
	  }
	  //Wait for SRDY High, do this regardless of error
	  if (ret == NPI_LNX_SUCCESS)
	  {
		  ret = HalGpioWaitSrdySet();
	  }
	  else
	  {
		  // Keep previous error message, but still de-assert to unlock Network Processor
		  HalGpioWaitSrdySet();
	  }
  }

  printf("Wait 500us for RNP to initialize after a Reset... This may change in the future, check for RTI_ResetInd()...\n");
  usleep(500); //wait 500us for RNP to initialize

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

	//	debug_
	printf("[%.5ld.%.6ld (+%ld.%6ld)] ----- END RESET SLAVE --------------\n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTime.tv_sec - t,
			diffPrev);
#else //(!defined __DEBUG_TIME__)
  printf("-------------------- END RESET SLAVE -------------------\n");
#endif //(defined __DEBUG_TIME__)

  return ret;
}

/* Initialize thread synchronization resources */
static int npi_initThreads(void)
{
	int ret = NPI_LNX_SUCCESS;
	// create Polling thread
  // initialize SPI receive thread related variables
	npi_poll_terminate = 0;

	// TODO: it is ideal to make this thread higher priority
	// but linux does not allow realtime of FIFO scheduling policy for
	// non-priviledged threads.

	if(pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
	{
		// thread creation failed
		NPI_SPI_CloseDevice();
    	npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_THREAD;
    	return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT

	if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
	{
		// thread creation failed
		NPI_SPI_CloseDevice();
    	npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_EVENT_THREAD;
    	return NPI_LNX_FAILURE;
	}
#endif

	return ret;

}

/**************************************************************************************************
 * @fn          NPI_SPI_SynchSlave
 *
 * @brief       do the HW synchronization between the host and the RNP
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_SPI_SynchSlave( void )
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_SYNCH_SLAVE;

	if (srdyMrdyHandshakeSupport == TRUE)
	{
		printf("\n\n-------------------- START GPIO HANDSHAKE -------------------\n");

		// At this point we already have npiPollMutex lock
		int lockRetSrdy = 0;
#ifdef SRDY_INTERRUPT
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			time_printf("[HANDSHAKE] Lock SRDY mutex");
		}
		lockRetSrdy = pthread_mutex_lock(&npiSrdyLock);
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			time_printf("[HANDSHAKE] SRDY mutex locked");
		}
#endif
		if (!PollLockVar)
		{
			ret = PollLockVarError(funcID);
		}
		else
		{
			PollLockVar = 1;
			debug_printf("[HANDSHAKE] PollLockVar set to %d\n", PollLockVar);
		}
		if (lockRetSrdy != 0)
		{
			printf("[HANDSHAKE] [ERR] Could not get SRDY mutex lock\n");
			perror("mutex lock");
		}

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

		//	debug_
		printf("[%.5ld.%.6ld (+%ld.%6ld)] Handshake Lock SRDY... Wait for SRDY to go Low\n",
				curTime.tv_sec - startTime.tv_sec,
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#else //(!defined __DEBUG_TIME__)
		printf("Handshake Lock SRDY ...\n");
#endif // defined __DEBUG_TIME__

		// Check that SRDY is low
		ret = HalGpioWaitSrdyClr();

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

		//	debug_
		printf("[%.5ld.%.6ld (+%ld.%6ld)] Set MRDY Low\n",
				curTime.tv_sec - startTime.tv_sec,
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif // defined __DEBUG_TIME__

		// set MRDY to Low
		if (ret == NPI_LNX_SUCCESS)
		{
			if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			{
				return ret;
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

		//	debug_
		printf("[%.5ld.%.6ld (+%ld.%6ld)] Wait for SRDY to go High\n",
				curTime.tv_sec - startTime.tv_sec,
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif // defined __DEBUG_TIME__

		// Wait for SRDY to go High
		ret = HalGpioWaitSrdySet();

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

		//	debug_
		printf("[%.5ld.%.6ld (+%ld.%6ld)] Set MRDY High\n",
				curTime.tv_sec - startTime.tv_sec,
				curTime.tv_usec,
				curTime.tv_sec - prevTime.tv_sec - t,
				diffPrev);
#endif // defined __DEBUG_TIME__
		// Set MRDY to High
		if (ret == NPI_LNX_SUCCESS)
			ret = HAL_RNP_MRDY_SET();
		else
			(void)HAL_RNP_MRDY_SET();

		if (ret == NPI_LNX_SUCCESS)
			ret = HalGpioSrdyCheck(1);

		if (!PollLockVar)
		{
			ret = PollLockVarError(funcID++);
		}
		else
		{
			PollLockVar = 0;
			debug_printf("[HANDSHAKE] PollLockVar set to %d\n", PollLockVar);
		}

		printf("[HANDSHAKE] unLock Poll ...");
		pthread_mutex_unlock(&npiPollLock);
		printf("(Handshake) success \n");
#ifdef SRDY_INTERRUPT
		pthread_mutex_unlock(&npiSrdyLock);
#endif
		printf("-------------------- END GPIO HANDSHAKE -------------------\n");
	}
	else
	{
		printf("\n\n----------------- SYNCHRONISING MUTEX'S ----------------\n");
		pthread_mutex_unlock(&npiPollLock);
		printf("unLock Poll ...\n");
#ifdef SRDY_INTERRUPT
		pthread_mutex_unlock(&npiSrdyLock);
		printf("unLock SRDY ...\n");
#endif
		printf("----------------- END SYNCHRONISING MUTEX'S ----------------\n");
	}

	return ret;
}

/**************************************************************************************************
 * @fn          npi_initsyncres
 *
 * @brief       Thread initialization
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static int npi_initsyncres(void)
{
  // initialize all mutexes
	debug_printf("LOCK POLL CREATED\n");
  if (pthread_mutex_init(&npiPollLock, NULL))
  {
    printf("Fail To Initialize Mutex npiPollLock\n");
    npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_LOCK_MUTEX;
    return NPI_LNX_FAILURE;
  }

  if(pthread_mutex_init(&npi_poll_mutex, NULL))
  {
    printf("Fail To Initialize Mutex npi_poll_mutex\n");
    npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_MUTEX;
    return NPI_LNX_FAILURE;
  }
#ifdef SRDY_INTERRUPT
	if(pthread_cond_init(&npi_srdy_H2L_poll, NULL))
	{
		printf("Fail To Initialize Condition npi_srdy_H2L_poll\n");
	    npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_SRDY_COND;
	    return NPI_LNX_FAILURE;
	}
	if (pthread_mutex_init(&npiSrdyLock, NULL))
	{
		printf("Fail To Initialize Mutex npiSrdyLock\n");
	    npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_SRDY_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}
#else
  if(pthread_cond_init(&npi_poll_cond, NULL))
  {
    printf("Fail To Initialize Condition npi_poll_cond\n");
    npi_ipc_errno = NPI_LNX_ERROR_SPI_OPEN_FAILED_POLL_COND;
    return NPI_LNX_FAILURE;
  }
#endif
  return NPI_LNX_SUCCESS;
}


/**************************************************************************************************
 * @fn          npi_poll_entry
 *
 * @brief       Poll Thread entry function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void *npi_poll_entry(void *ptr)
{
	int ret = NPI_LNX_SUCCESS, funcID = NPI_LNX_ERROR_FUNC_ID_POLL_ENTRY;
	uint8 readbuf[128];
#ifndef SRDY_INTERRUPT
	uint8 pollStatus = FALSE;
#endif //SRDY_INTERRUPT

	printf("[POLL] Locking Mutex for Poll Thread \n");

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_poll_mutex);

	printf("[POLL] Thread Started \n");

	//This lock wait for Initialization to finish (reset+sync)
	pthread_mutex_lock(&npiPollLock);

	printf("[POLL] Thread Continues After Synchronization\n");

#ifdef SRDY_INTERRUPT
	debug_printf("[POLL] Lock Poll mutex (SRDY=%d) \n", global_srdy);
	pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
	debug_printf("[POLL] Locked Poll mutex (SRDY=%d) \n", global_srdy);
#else
	pthread_mutex_unlock(&npiPollLock);
#endif

	/* thread loop */
	while(!npi_poll_terminate)
	{

#ifndef SRDY_INTERRUPT
		pthread_mutex_lock(&npiPollLock);
#endif
		if (PollLockVar)
		{
			ret = PollLockVarError(funcID+1);
		}
		else
		{
			PollLockVar = 1;
			debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
		}

		//Ready SRDY Status
		// This Test check if RNP has asserted SRDY line because it has some Data pending.
		// If SRDY is not Used, then this line need to be commented, and the Poll command need
		// to be sent regularly to check if any data is pending. this is done every 10ms (see below npi_poll_cond)
#ifndef SRDY_INTERRUPT
		ret =  HAL_RNP_SRDY_CLR();
		if(TRUE == ret)
#else
		//Interruption case, In case of a SREQ, SRDY will go low a end generate an event.
		// the npiPollLock will prevent us to arrive to this test,
		// BUT an AREQ can immediately follow  a SREQ: SRDY will stay low for the whole process
		// In this case, we need to check that the SRDY line is still LOW or is HIGH.
		if(1)
#endif
		{
			debug_printf("[POLL] Polling received...\n");

			//RNP is polling, retrieve the data
			*readbuf = 0; //Poll Command has zero data bytes.
			*(readbuf+1) = RPC_CMD_POLL;
			*(readbuf+2) = 0;
			ret = npi_spi_pollData((npiMsgData_t *)readbuf);
			if (ret == NPI_LNX_SUCCESS)
			{
				//Check if polling was successful
				if ((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
				{
//					((uint8 *)readbuf)[RPC_POS_CMD0] =  RPC_SUBSYSTEM_MASK;
					ret = NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
					if (ret != NPI_LNX_SUCCESS)
					{
						// Exit thread to invoke report to main thread
						npi_poll_terminate = 1;
					}
				}
			}
			else
			{
				// Exit thread to invoke report to main thread
				npi_poll_terminate = 1;
				if (ret == NPI_LNX_ERROR_SPI_POLL_DATA_SRDY_CLR_TIMEOUT_POSSIBLE_RESET)
				{
					printf("[POLL][WARNING] Unexpected handshake received. RNP may have reset. \n");
				}
			}

			if (!PollLockVar)
			{
				ret = PollLockVarError(funcID+2);
			}
			else
			{
				PollLockVar = 0;
				debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
			}

#ifndef SRDY_INTERRUPT
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				pollStatus = TRUE;
				debug_printf("[POLL] UnLock SRDY ...\n");
			}
			else
			{
				debug_printf("[POLL] UnLock SRDY FAILED...\n");
			    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_UNLOCK;
			    ret = NPI_LNX_FAILURE;
				npi_poll_terminate = 1;
			}
#endif //SRDY_INTERRUPT
		}
		else
		{
			if (!PollLockVar)
			{
				ret = PollLockVarError(funcID+3);
			}
			else
			{
				PollLockVar = 0;
				debug_printf("[POLL] PollLockVar set to %d\n", PollLockVar);
			}

#ifndef SRDY_INTERRUPT
			if ( 0 == pthread_mutex_unlock(&npiPollLock))
			{
				debug_printf("[POLL] UnLock SRDY ...\n");
			}
			else
			{
				debug_printf("[POLL] UnLock SRDY FAILED...\n");
			    npi_ipc_errno = NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_UNLOCK;
			    ret = NPI_LNX_FAILURE;
				npi_poll_terminate = 1;
			}
			pollStatus = FALSE;
#endif //SRDY_INTERRUPT
		}

#ifdef SRDY_INTERRUPT
		debug_printf("[POLL] Unlock POLL mutex by conditional wait (SRDY=%d) \n", global_srdy);
		if (!npi_poll_terminate)
		{
			pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
		}
		else
		{
			// Just unlock mutex, while loop will exit next
			pthread_mutex_unlock(&npiPollLock);
		}
		debug_printf("[POLL] Locked POLL mutex because condition was met (SRDY=%d) \n", global_srdy);
#else
		if (!pollStatus) //If previous poll failed, wait 10ms to do another one, else do it right away to empty the RNP queue.
		{
			struct timespec expirytime;
			struct timeval curtime;

			gettimeofday(&curtime, NULL);
			expirytime.tv_sec = curtime.tv_sec;
			expirytime.tv_nsec = (curtime.tv_usec * 1000) + 10000000;
			if (expirytime.tv_nsec >= 1000000000) {
				expirytime.tv_nsec -= 1000000000;
				expirytime.tv_sec++;
			}
			pthread_cond_timedwait(&npi_poll_cond, &npi_poll_mutex, &expirytime);
		}
#endif
	}
	printf("[POLL] Thread Exiting... \n");
	pthread_mutex_unlock(&npi_poll_mutex);

	char *errorMsg;
	if ( (ret != NPI_LNX_SUCCESS) && (npi_ipc_errno != NPI_LNX_ERROR_SPI_POLL_THREAD_SREQ_CONFLICT) )
	{
		errorMsg = "[POLL] Thread exited with error. Please check global error message\n";
	}
	else
	{
		errorMsg = "[POLL] Thread exited without error\n";
	}

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_SPI_POLL_THREAD_POLL_LOCK), errorMsg);

	return NULL;
}

/**************************************************************************************************
 * @fn          npi_termpoll
 *
 * @brief       Poll Thread terminate function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void npi_termpoll(void)
{
  //This will cause the Thread to exit
  npi_poll_terminate = 1;

#ifdef SRDY_INTERRUPT
	pthread_cond_signal(&npi_srdy_H2L_poll);
#else

	// In case of polling mechanism, send the Signal to continue
	pthread_cond_signal(&npi_poll_cond);
#endif

#ifdef SRDY_INTERRUPT
	pthread_mutex_destroy(&npiSrdyLock);
#endif
  pthread_mutex_destroy(&npi_poll_mutex);

  // wait till the thread terminates
  pthread_join(npiPollThread, NULL);

#ifdef SRDY_INTERRUPT
  pthread_join(npiEventThread, NULL);
#endif //SRDY_INTERRUPT
}

#ifdef SRDY_INTERRUPT
/**************************************************************************************************
 * @fn          npi_event_entry
 *
 * @brief       Poll Thread entry function
 *
 * input parameters
 *
 * @param      ptr
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void *npi_event_entry(void *ptr)
{
#define SPI_ISR_POLL_TIMEOUT_MS_MIN		3
#define SPI_ISR_POLL_TIMEOUT_MS_MAX		100
	int result = -1;
	int missedInterrupt = 0;
	int consecutiveTimeout = 0;
	int whileIt = 0;
	int ret = NPI_LNX_SUCCESS;
	int timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
	/* Timeout in msec. Drop down to SPI_ISR_POLL_TIMEOUT_MS_MIN if two consecutive interrupts are missed */
	struct pollfd pollfds[1];
	int val;
	char tmpStr[512];

	printf("[INT]: Interrupt Event Thread Started \n");

	/* thread loop */
	while (!npi_poll_terminate)
	{
		whileIt++;
		memset((void*) pollfds, 0, sizeof(pollfds));
		pollfds[0].fd = GpioSrdyFd; /* Wait for input */
		pollfds[0].events = POLLPRI; /* Wait for input */
		result = poll(pollfds, 1, timeout);

		// Make sure we're not in Asynch data or Synch data, so check if npiSrdyLock is available
		if (pthread_mutex_trylock(&npiSrdyLock) != 0)
		{
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				// We are in Asynch or Synch, so return to poll
				if (HAL_RNP_SRDY_SET() == TRUE)
				{
					time_printf("[INT]: SRDY found to be de-asserted while we are transmitting");
				}
				else
				{
					time_printf("[INT]: SRDY found to be asserted while we are transmitting");
				}
			}
			continue;
		}
		else
		{
			if ( __BIG_DEBUG_ACTIVE == TRUE)
			{
				snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread has SRDY mutex lock, result = %d", result);
				time_printf(tmpStr);
			}
			// We got lock, move on
			switch (result)
			{
			case 0:
			{
				//Should not happen by default no Timeout.
				int bigDebugWas = __BIG_DEBUG_ACTIVE;
				if (bigDebugWas == TRUE)
				{
					__BIG_DEBUG_ACTIVE = FALSE;
				}
				if (__BIG_DEBUG_ACTIVE == TRUE)
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: poll() timeout (timeout set to %d), poll() returned %d", timeout, result);
					time_printf(tmpStr);
				}
				result = 2; //Force wrong result to avoid deadlock caused by timeout
				//#ifdef __BIG_DEBUG__
				if (  NPI_LNX_FAILURE == (val = HalGpioSrdyCheck(1)))
				{
					ret = val;
					npi_poll_terminate = 1;
				}
				else
				{
					// Accept this case as a missed interrupt. We may stall if not attempting to handle asserted SRDY
					if (!val)
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "%d (it #%d)", missedInterrupt, whileIt);
							time_printf(tmpStr);
						}
						missedInterrupt++;
						consecutiveTimeout = 0;
					}
					else
					{
						// Timed out. If we're in rapid poll mode, i.e. timeout == SPI_ISR_POLL_TIMEOUT_MS_MAX
						// then we should only allow 10 consecutive such timeouts before resuming normal
						// timeout
						consecutiveTimeout++;
						if ( (timeout == SPI_ISR_POLL_TIMEOUT_MS_MAX) &&
								(consecutiveTimeout > 100) )
						{
							// Timed out 100 times, for 300ms, without a single
							// SRDY assertion. Set back to 100ms timeout.
							consecutiveTimeout = 0;
							// Set timeout back to 100ms
							timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
						}

						missedInterrupt = 0;
					}
					result = val;
				}
				//#endif
				debug_printf("[INT]: SRDY: %d\n", val);
				__BIG_DEBUG_ACTIVE = bigDebugWas;
				break;
			}
			case -1:
			{
				debug_printf("[INT]:poll() error \n");
				npi_ipc_errno = NPI_LNX_ERROR_SPI_EVENT_THREAD_FAILED_POLL;
				ret = NPI_LNX_FAILURE;
				consecutiveTimeout = 0;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
				break;
			}
			default:
			{
				consecutiveTimeout = 0;
				//				char * buf[64];
				//				read(pollfds[0].fd, buf, 64);
				if (missedInterrupt)
				{
					if ( (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 )
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "[INT]:Poll returned error (it #%d), revent[0] = %d",
								whileIt,
								pollfds[0].revents);
						}
					}
					else
					{
						if ( __BIG_DEBUG_ACTIVE == TRUE )
						{
							snprintf(tmpStr, sizeof(tmpStr), "[INT]:Clearing missed INT (it #%d), results = %d, revent[0] = %d",
								whileIt,
								result,
								pollfds[0].revents);
						}
						missedInterrupt = 0;
						// Set timeout back to 100ms
						timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
					}
					time_printf(tmpStr);
				}
				result = global_srdy = HalGpioSrdyCheck(1);
				debug_printf("[INT]:Set global SRDY: %d\n", global_srdy);

				break;
			} //default:
			} //switch (result)
		} // else of if (pthread_mutex_trylock(npiSrdyLock) != 0)

		fflush(stdout);

		if (FALSE == result) //Means SRDY switch to low state
		{
			if (gettimeofday(&curTimeSPIisrPoll, NULL) == 0)
				// Adjust poll timeout based on time between packets, limited downwards
				// to SPI_ISR_POLL_TIMEOUT_MS_MIN and upwards to SPI_ISR_POLL_TIMEOUT_MS_MAX
			{
				// Calculate delta
				long int diffPrev;
				if (curTimeSPIisrPoll.tv_usec >= prevTimeSPIisrPoll.tv_usec)
				{
					diffPrev = curTimeSPIisrPoll.tv_usec - prevTimeSPIisrPoll.tv_usec;
				}
				else
				{
					diffPrev = (curTimeSPIisrPoll.tv_usec + 1000000) - prevTimeSPIisrPoll.tv_usec;
				}
				prevTimeSPIisrPoll = curTimeSPIisrPoll;

				if (diffPrev < (SPI_ISR_POLL_TIMEOUT_MS_MIN / 1000) )
				{
					timeout = SPI_ISR_POLL_TIMEOUT_MS_MIN;
				}
				else if (diffPrev > (SPI_ISR_POLL_TIMEOUT_MS_MAX / 1000) )
				{
					timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
				}
				else
				{
					timeout = diffPrev / 1000;
				}
			}
			else
			{
				// Not good, can't trust time. Set timeout to its max
				timeout = SPI_ISR_POLL_TIMEOUT_MS_MAX;
			}



			if ( (NPI_LNX_FAILURE == (ret = HalGpioMrdyCheck(1))))
			{
				debug_printf("[INT]:Fail to check MRDY \n");
				ret = NPI_LNX_FAILURE;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
			}

			if (ret != NPI_LNX_FAILURE)
			{
				if (missedInterrupt > 0)
				{
					// Two consecutive interrupts; turn down timeout to SPI_ISR_POLL_TIMEOUT_MS_MIN
					timeout = SPI_ISR_POLL_TIMEOUT_MS_MIN;
	
					if ( __BIG_DEBUG_ACTIVE == TRUE )
					{
						snprintf(tmpStr, sizeof(tmpStr), "[INT] Missed interrupt, but SRDY is asserted! %d (it #%d)", 
missedInterrupt, whileIt);
						time_printf(tmpStr);
					}
				}

				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock");
					time_printf(tmpStr);
				}

				// Unlock before signaling poll thread
				pthread_mutex_unlock(&npiSrdyLock);
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					//MRDY High, This is a request from the RNP
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: MRDY High??: %d, send H2L to POLL (srdy = %d)", ret, global_srdy);
					time_printf(tmpStr);
				}
				pthread_cond_signal(&npi_srdy_H2L_poll);
			}
			else
			{
				if ( __BIG_DEBUG_ACTIVE == TRUE )
				{
					snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock");
					time_printf(tmpStr);
				}
				pthread_mutex_unlock(&npiSrdyLock);
			}
		}
		else
		{
			//Unknown Event
			//Do nothing for now ...
			//debug_printf("Unknown Event or timeout, ignore it, result:%d \n",result);
			if ( __BIG_DEBUG_ACTIVE == TRUE )
			{
				snprintf(tmpStr, sizeof(tmpStr), "[INT]: Event thread is releasing SRDY mutex lock, result = %d",result);
				time_printf(tmpStr);
			}
			pthread_mutex_unlock(&npiSrdyLock);
		}
	}

	return NULL;
}
#endif //SRDY_INTERRUPT

#endif //#if (defined NPI_SPI) && (NPI_SPI == TRUE)

/**************************************************************************************************
*/
