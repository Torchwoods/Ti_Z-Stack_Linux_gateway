/**************************************************************************************************
  Filename:       npi_lnx_i2c.c
  Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
  Revision:       $Revision: 246 $

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
#include <poll.h>
#include <linux/input.h>

#include "aic.h"
#include "npi_lnx.h"
#include "npi_lnx_i2c.h"
#include "hal_rpc.h"
#include "hal_gpio.h"

#include "npi_lnx_error.h"

#if (defined __STRESS_TEST__) || (defined __DEBUG_TIME__I2C)
#include <sys/time.h>
#endif // __STRESS_TEST__

#if (defined NPI_I2C) && (NPI_I2C == TRUE)

// -- macros --

#ifndef TRUE
# define TRUE (1)
#endif

#ifndef FALSE
# define FALSE (0)
#endif

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#ifdef __DEBUG_TIME__I2C
struct timeval curTime, initTime;
#endif
#if (defined __DEBUG_TIME__)
struct timeval curTime, prevTime;
extern struct timeval startTime;
#endif
// -- Constants --

// State variable used to indicate that a device is open.
static int npiOpenFlag = FALSE;

// NPI device related variables
static int              	npi_poll_terminate;
static pthread_mutex_t  	npiPollLock;
static pthread_mutexattr_t 	npiPollLock_attr;
static pthread_mutex_t  	npi_poll_mutex;
static int 					GpioSrdyFd;
#ifndef SRDY_INTERRUPT
static pthread_cond_t   	npi_poll_cond;
#endif
// Polling thread
static pthread_t        	npiPollThread;
// Event thread
#ifdef SRDY_INTERRUPT
// Event thread
//---------------
static pthread_t        npiEventThread;
static void 				*npi_event_entry(void *ptr);

static pthread_cond_t   npi_srdy_H2L_poll;

static pthread_mutex_t  npiSrdyLock;
//static pthread_mutex_t  npi_Srdy_mutex;
static int global_srdy = 1;
#endif

// For debugging mainly
uint8 writeOnce = 0;

// thread termination subroutines
static void npi_termpoll(void);
static void 			*npi_poll_entry(void *ptr);

// -- Public functions --

#ifdef __STRESS_TEST__
extern struct timeval curTime, startTime;
struct timeval prevTimeI2C;
#endif //__STRESS_TEST__

// -- Private functions --
static int npi_initsyncres(void);
static int npi_initThreads(void);
static int npi_i2c_pollData(npiMsgData_t *pMsg);

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
int NPI_I2C_OpenDevice(const char *portName, void *pCfg)
{
	int ret = NPI_LNX_SUCCESS;
	if(npiOpenFlag)
	{
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_ALREADY_OPEN;
	    return NPI_LNX_FAILURE;
	}

	npiOpenFlag = TRUE;

	debug_printf("Opening Device File: %s\n", portName);

	ret = HalI2cInit(portName);
	if (ret != NPI_LNX_SUCCESS)
		return ret;

	debug_printf("((npiI2cCfg_t *)pCfg)->gpioCfg[0] \t @%p\n",
			(void *)&(((npiI2cCfg_t *)pCfg)->gpioCfg[0]));

  	if ( NPI_LNX_FAILURE == (GpioSrdyFd = HalGpioSrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[0])))
		return GpioSrdyFd;
	if ( NPI_LNX_FAILURE == (ret = HalGpioMrdyInit(((npiI2cCfg_t *)pCfg)->gpioCfg[1])))
		return ret;
	if ( NPI_LNX_FAILURE == (ret = HalGpioResetInit(((npiI2cCfg_t *)pCfg)->gpioCfg[2])))
		return ret;

	// initialize thread synchronization resources
	if ( NPI_LNX_FAILURE == (ret = npi_initsyncres()))
		return ret;

#ifdef __DEBUG_TIME__I2C
	gettimeofday(&initTime, NULL);
	printf("NPI I2C timer started\n");
#endif //__DEBUG_TIME__I2C

	// Reset The RNP
	ret = NPI_I2C_ResetSlave();
	if (ret != NPI_LNX_SUCCESS)
		return ret;

	ret = npi_initThreads();

	return ret;
}

/******************************************************************************
 * @fn         NPI_I2C_CloseDevice
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
void NPI_I2C_CloseDevice(void)
{
  npi_termpoll();
  HalI2cClose();
  HalGpioSrdyClose();
  HalGpioMrdyClose();
  HalGpioResetClose();
  npiOpenFlag = FALSE;
}

/**************************************************************************************************
 * @fn          NPI_I2C_SendAsynchData
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
int NPI_I2C_SendAsynchData(npiMsgData_t *pMsg)
{
	int ret = NPI_LNX_SUCCESS;
	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	if (0 == pthread_mutex_lock(&npiPollLock))
	{
		writeOnce = 0;

#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		debug_printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendAsynchData has lock (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX] NPI_I2C_SendAsynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
	else
	{
#if (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX-ERR] NPI_I2C_SendAsynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
		perror("Mutex unlock Poll:");
		npi_ipc_errno = NPI_LNX_ERROR_I2C_SEND_ASYNCH_FAILED_LOCK;
		ret = NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	debug_printf("(Sync) success \n");
	debug_printf("\n******************** START SEND ASYNC DATA ********************\n");

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_AREQ;

	if (ret == NPI_LNX_SUCCESS)
		if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			return ret;

	//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
	// To avoid any problem, just add a timeout, or ignore it.
	ret = HalGpioWaitSrdyClr();

	//Send LEN, CMD0 and CMD1 (comand Header)
	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	if (0 == pthread_mutex_unlock(&npiPollLock))
	{
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		debug_printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendAsynchData released (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX] NPI_I2C_SendAsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
	else
	{
#if (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX-ERR] NPI_I2C_SendAsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
		perror("Mutex unlock Poll:");
		npi_ipc_errno = NPI_LNX_ERROR_I2C_SEND_ASYNCH_FAILED_UNLOCK;
		ret = NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
	// Also signal poll thread so it can poll for a NULL message from the RNP. Otherwise we won't later get
	// the interrupt from the RNP, since the RNP will keep SRDY low until it has answered NULL to a poll.
	pthread_cond_signal(&npi_srdy_H2L_poll);
#endif
	debug_printf("Sync unLock SRDY ...\n");
	debug_printf("******************** STOP SEND ASYNC DATA ********************\n\n");

	return ret;
}


/**************************************************************************************************
 * @fn          npi_i2c_pollData
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
int npi_i2c_pollData(npiMsgData_t *pMsg)
{
	int ret = NPI_LNX_SUCCESS;

#ifdef __BIG_DEBUG__
	printf("Polling Command ...");

	int i;
	for(i = 0 ; i < (RPC_FRAME_HDR_SZ+pMsg->len); i++)
		printf(" 0x%.2x", ((uint8*)pMsg)[i]);

	printf("\n");
#endif
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	debug_printf("\n-------------------- START POLLING DATA --------------------\n");

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

	printf("[POLL %.5ld.%.6ld (+%ld.%6ld)] MRDY Low \n",
			curTime.tv_sec - startTime.tv_sec,
			curTime.tv_usec,
			curTime.tv_sec - prevTimeI2C.tv_sec - t,
			diffPrev);
#endif //__STRESS_TEST__

	if (ret == NPI_LNX_SUCCESS)
		if ( NPI_LNX_SUCCESS != (ret = HAL_RNP_MRDY_CLR()))
			return ret;

	//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
	// To avoid any problem, just add a timeout, or ignore it.
	ret = HalGpioWaitSrdyClr();


	//Send LEN, CMD0 and CMD1 (command Header) and payload
	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cRead(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

	if(pMsg->len != 0)
	{
		debug_printf("I2C Read Response\n");
		//Read RSP Data
		if (ret == NPI_LNX_SUCCESS)
			ret = HalI2cRead(0, (uint8*) &(pMsg->pData[0]), (pMsg->len));
	}

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

	debug_printf("\n-------------------- END POLLING DATA --------------------\n");
#ifdef SRDY_INTERRUPT
	pthread_mutex_unlock(&npiSrdyLock);
#endif

	return ret;
}

/**************************************************************************************************
 * @fn          NPI_I2C_SendSynchData
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
int NPI_I2C_SendSynchData(npiMsgData_t *pMsg)
{
	int ret = NPI_LNX_SUCCESS;
	// Do not attempt to send until polling is finished

	int lockRet = 0;
	debug_printf("Sync Lock SRDY ...");
	//Lock the polling until the command is send
	lockRet = pthread_mutex_lock(&npiPollLock);
#ifdef SRDY_INTERRUPT
	pthread_mutex_lock(&npiSrdyLock);
#endif
	debug_printf("(Sync) success \n");
	debug_printf("==================== START SEND SYNC DATA ====================\n");
	if (lockRet != 0)
	{
		debug_printf("[ERR] Could not get lock\n");
		perror("mutex lock");
		npi_ipc_errno = NPI_LNX_ERROR_I2C_SEND_SYNCH_FAILED_LOCK;
		ret = NPI_LNX_FAILURE;
	}
	else
	{
		writeOnce = 0;
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		debug_printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendSynchData has lock (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX] NPI_I2C_SendSynchData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}

	// Add Proper RPC type to header
	((uint8*)pMsg)[RPC_POS_CMD0] = (((uint8*)pMsg)[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SREQ;

#ifdef __BIG_DEBUG__
	printf("Sync Data Command ...");

	int i;
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

	//This wait is only valid if RNP manage MRDY line, if not SRDY will never be set low on SREQ.
	// To avoid any problem, just add a timeout, or ignore it.
	ret = HalGpioWaitSrdyClr();

	//Send LEN, CMD0 and CMD1 (comand Header)
	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cWrite(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ + (pMsg->len));

	if (ret == NPI_LNX_SUCCESS)
		ret = HalI2cRead(0, (uint8*) pMsg, RPC_FRAME_HDR_SZ);

	if(pMsg->len != 0)
	{
		//Read RSP Data
		if (ret == NPI_LNX_SUCCESS)
			ret = HalI2cRead(0, (uint8*) &(pMsg->pData[0]), (pMsg->len));

		if ((ret != NPI_LNX_SUCCESS) && (npi_ipc_errno == NPI_LNX_ERROR_HAL_I2C_READ_TIMEDOUT))
		{
			debug_printf("Failed to read out SRSP payload\n");
			// Failed to read out payload. This may be caused by an unexpected reset.
			// Indicate to calling application that the read failed by replacing the length byte.
			pMsg->len = 0;
		}
	}

	//Release the polling lock
	//This is the SRSP, clear out the PC type in header
	((uint8 *)pMsg)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;

	if (ret == NPI_LNX_SUCCESS)
		ret = HAL_RNP_MRDY_SET();
	else
		(void)HAL_RNP_MRDY_SET();

    debug_printf("\n==================== END SEND SYNC DATA ====================\n");
	if (0 == pthread_mutex_unlock(&npiPollLock))
	{
#ifdef __DEBUG_TIME__I2C
		//	debug_
		gettimeofday(&curTime, NULL);
		debug_printf("[MUTEX %.5ld.%.6ld] NPI_I2C_SendSynchData released (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
				((int *)&npiPollLock)[1],
				syscall(224));
#elif (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX] NPI_I2C_SendSynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
	}
	else
	{
#if (defined __DEBUG_MUTEX__)
		debug_printf("[MUTEX-ERR] NPI_I2C_SendSsynchData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
		perror("Mutex unlock Poll:");
		npi_ipc_errno = NPI_LNX_ERROR_I2C_SEND_SYNCH_FAILED_UNLOCK;
		ret = NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
    pthread_mutex_unlock(&npiSrdyLock);
#endif
	debug_printf("Sync unLock SRDY ...\n\n");
	return ret;
}

/**************************************************************************************************
 * @fn          NPI_I2C_ResetSlave
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
int NPI_I2C_ResetSlave(void)
{
	int ret = NPI_LNX_SUCCESS;

	printf("\n\n-------------------- START RESET SLAVE -------------------\n");
	ret = HalGpioReset();
	printf("Wait 1.2s for RNP to initialize after a Reset... This may change in the future, check for RTI_ResetInd()...\n");
	usleep(1200000); //wait 1.2s for RNP to initialize
	printf("-------------------- END RESET SLAVE -------------------\n");

	return ret;
}

/* Initialize thread synchronization resources */
static int npi_initThreads(void)
{
  // create Polling thread
  // initialize I2C receive thread related variables
  npi_poll_terminate = 0;

  // TODO: it is ideal to make this thread higher priority
  // but linux does not allow realtime of FIFO scheduling policy for
  // non-priviledged threads.

  if(pthread_create(&npiPollThread, NULL, npi_poll_entry, NULL))
  {
    // thread creation failed
    NPI_I2C_CloseDevice();
    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_THREAD;
    return NPI_LNX_FAILURE;
  }
#ifdef SRDY_INTERRUPT

  if(pthread_create(&npiEventThread, NULL, npi_event_entry, NULL))
  {
    // thread creation failed
    NPI_I2C_CloseDevice();
    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_EVENT_THREAD;
    return NPI_LNX_FAILURE;
  }
#endif

  return NPI_LNX_SUCCESS;
}

/* Initialize thread synchronization resources */
static int npi_initsyncres(void)
{
	printf("[MUTEX] Initializing mutexes\n");
	// initialize all mutexes
	pthread_mutexattr_init(&npiPollLock_attr);
	pthread_mutexattr_settype(&npiPollLock_attr, PTHREAD_MUTEX_ERRORCHECK);
	if ( pthread_mutex_init(&npiPollLock, &npiPollLock_attr) )
//	if ( pthread_mutex_init(&npiPollLock, NULL) )
	{
		printf("Fail To Initialize Mutex npiPollLock\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}

	if(pthread_mutex_init(&npi_poll_mutex, NULL))
	{
		printf("Fail To Initialize Mutex npi_poll_mutex\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_MUTEX;
	    return NPI_LNX_FAILURE;
	}
#ifdef SRDY_INTERRUPT
	if(pthread_cond_init(&npi_srdy_H2L_poll, NULL))
	{
		printf("Fail To Initialize Condition npi_srdy_H2L_poll\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_SRDY_COND;
	    return NPI_LNX_FAILURE;
	}
	if (pthread_mutex_init(&npiSrdyLock, NULL))
	{
		printf("Fail To Initialize Mutex npiSrdyLock\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_SRDY_LOCK_MUTEX;
	    return NPI_LNX_FAILURE;
	}
#else

	if(pthread_cond_init(&npi_poll_cond, NULL))
	{
		printf("Fail To Initialize Condition npi_poll_cond\n");
	    npi_ipc_errno = NPI_LNX_ERROR_I2C_OPEN_FAILED_POLL_COND;
	    return NPI_LNX_FAILURE;
	}
#endif

	return NPI_LNX_SUCCESS;
}


/* I2C RX thread entry routine */
static void *npi_poll_entry(void *ptr)
{
	int ret = NPI_LNX_SUCCESS;
	uint8 readbuf[128];
	uint8 pollStatus = FALSE;

	/* lock mutex in order not to lose signal */
	pthread_mutex_lock(&npi_poll_mutex);

	printf("POLL: Thread Started \n");

#ifdef SRDY_INTERRUPT
	debug_printf("POLL: Lock Poll mutex -- (SRDY=%d) \n", global_srdy);
	pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
	debug_printf("POLL: Locked Poll mutex -- (SRDY=%d) \n", global_srdy);
#endif

	/* thread loop */
	while(!npi_poll_terminate)
	{

		pthread_mutex_lock(&npiPollLock);

		if (writeOnce < 4)
		{
#ifdef __DEBUG_TIME__I2C
			//	debug_
			gettimeofday(&curTime, NULL);
			debug_printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData has lock (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
					((int *)&npiPollLock)[1],
					syscall(224));
#elif (defined __DEBUG_MUTEX__)
			debug_printf("[MUTEX] npi_i2c_pollData has lock (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
		}

		//Ready SRDY Status
		// This Test check if RNP has asserted SRDY line because it has some Data pending.
		// If SRDY is not Used, then this line need to be commented, and the Poll command need
		// to be send regulary to check if any data is pending. this is done every 10ms (see below npi_poll_cond)
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
				//   exit(-1);
				//RNP is polling, retrieve the data
				*readbuf = 0; //Poll Command has zero data bytes.
				*(readbuf+1) = RPC_CMD_POLL;
				*(readbuf+2) = 0;

				//Send the Polling Command,
				ret = npi_i2c_pollData((npiMsgData_t *)readbuf);
				if ( ret != NPI_LNX_SUCCESS)
				{
					// An error has occurred
					// Check what error it is, some errors are expected
					char *errorMsg;
					switch (npi_ipc_errno)
					{
					case NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT:
						errorMsg = "npi_i2c_pollData timed out waiting for SRDY. Could be SBL which only responds to SREQ. Please check global error message\n";
						NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT), errorMsg);
						// The RNP may be in SBL or may only respond to SREQ. We should not exit POLL thread.
						ret = NPI_LNX_SUCCESS;
					break;
					case  NPI_LNX_ERROR_HAL_I2C_READ_TIMEDOUT:
						// The RNP may have reset unexpectedly, keep going
						ret = NPI_LNX_SUCCESS;
						break;
					case  NPI_LNX_ERROR_HAL_I2C_WRITE_TIMEDOUT:
						// The RNP may have reset unexpectedly, keep going
						ret = NPI_LNX_SUCCESS;
						break;
					default:
						// Exit clean so main knows...
						npi_poll_terminate = 1;
						break;
					}
				}

#ifdef __DEBUG_TIME__I2C
				//	debug_
				gettimeofday(&curTime, NULL);
#endif //__DEBUG_TIME__I2C

				if(0 == pthread_mutex_unlock(&npiPollLock))
				{
#ifdef __DEBUG_TIME__I2C
					debug_printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData released (%ld cnt: %d) [tid: %ld]\n",
				curTime.tv_sec - initTime.tv_sec,
				curTime.tv_usec,
				npiPollLock.__align,
							((int *)&npiPollLock)[1],
							syscall(224));
#elif (defined __DEBUG_MUTEX__)

					debug_printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n",
							npiPollLock,
							&npiPollLock);
#endif //__DEBUG_TIME__I2C
					pollStatus = TRUE;
				}
				else
				{
#if (defined __DEBUG_MUTEX__)
					printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
					perror("Mutex unlock Poll:");
					npi_ipc_errno = NPI_LNX_ERROR_I2C_POLL_THREAD_FAILED_UNLOCK;
					ret = NPI_LNX_FAILURE;
					// Exit clean so main knows...
					npi_poll_terminate = 1;
				}


				//Check if polling was successful, if so send message to host.
				if((readbuf[RPC_POS_CMD0] & RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
				{
					((uint8 *)readbuf)[RPC_POS_CMD0] &=  RPC_SUBSYSTEM_MASK;
					ret = NPI_AsynchMsgCback((npiMsgData_t *)(readbuf));
					writeOnce = 0;
					if ( ret != NPI_LNX_SUCCESS)
					{
						// An error has occurred
						// Exit clean so main knows...
						npi_poll_terminate = 1;
					}
				}
			}
			else
			{
				pollStatus = FALSE;
#ifndef SRDY_INTERRUPT
				if ( ret != NPI_LNX_SUCCESS)
				{
					// An error has occurred
					// Exit clean so main knows...
					npi_poll_terminate = 1;
				}
#endif

				if (pollStatus == FALSE)
				{
					if(0 == pthread_mutex_unlock(&npiPollLock))
					{
						if (writeOnce < 4)
						{
#ifdef __DEBUG_TIME__I2C
							//	debug_
							gettimeofday(&curTime, NULL);
							debug_printf("[MUTEX %.5ld.%.6ld] npi_i2c_pollData released (%ld cnt: %d) [tid: %ld]\n",
									curTime.tv_sec - initTime.tv_sec,
									curTime.tv_usec,
									npiPollLock.__align,
									((int *)&npiPollLock)[1],
									syscall(224));
#elif (defined __DEBUG_MUTEX__)
							debug_printf("[MUTEX] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //__DEBUG_TIME__I2C
							writeOnce++;
						}
					}
					else
					{
#if (defined __DEBUG_MUTEX__)
						printf("[MUTEX-ERR] npi_i2c_pollData released (%d @ 0x%.16X)\n", npiPollLock, &npiPollLock);
#endif //(defined __DEBUG_MUTEX__)
						perror("Mutex unlock Poll:");
						npi_ipc_errno = NPI_LNX_ERROR_I2C_POLL_THREAD_FAILED_UNLOCK;
						ret = NPI_LNX_FAILURE;
						// Exit clean so main knows...
						npi_poll_terminate = 1;
					}
				}
			}

#ifdef SRDY_INTERRUPT
		if (HAL_RNP_SRDY_SET())
		{
			pthread_mutex_lock(&npiPollLock);
			debug_printf("POLL: Locked SRDY mutex -- (SRDY=%d), waiting for SRDY \n", global_srdy);
			pthread_cond_wait(&npi_srdy_H2L_poll, &npiPollLock);
			debug_printf("POLL: Unlocked SRDY mutex -- (SRDY=%d) \n", global_srdy);
		}
		else
		{
			// After a synchronous request we need to poll RNP again. The RNP will only release SRDY after
			// replying with 0x00 0x00 0x00 to a poll request.
		}
#else
		if (!pollStatus) //If previous poll failed, wait 10ms to do another one, else do it right away to empty the RNP queue.
		{
			struct timespec expirytime;
			struct timeval curtime;

			gettimeofday(&curtime, NULL);
			expirytime.tv_sec = curtime.tv_sec;
			expirytime.tv_nsec = (curtime.tv_usec * 1000) + 1000000; //Wait 1000us for next polling

			if(expirytime.tv_nsec >= 1000000000)
			{
				expirytime.tv_nsec -= 1000000000;
				expirytime.tv_sec++;
			}

			pthread_cond_timedwait(&npi_poll_cond, &npi_poll_mutex, &expirytime);
		}
#endif
	}
	pthread_mutex_unlock(&npi_poll_mutex);

	char *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "I2C Poll thread exited with error. Please check global error message\n";
	else
		errorMsg = "I2C Poll thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_I2C_POLL_THREAD_FAILED_LOCK), errorMsg);

	return ptr;
}

/* Terminate Polling thread */
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
	// In case of polling mechanism, send the Signal to continue
  pthread_mutex_lock(&npi_poll_mutex);
  pthread_mutex_unlock(&npi_poll_mutex);

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
	int result = -1;
	int ret = NPI_LNX_SUCCESS;
	int timeout = 2000; /* Timeout in msec. */
	struct pollfd pollfds[1];

	printf("EVENT: Thread Started \n");

	/* thread loop */
	while (!npi_poll_terminate) 
	{
		memset((void*) pollfds, 0, sizeof(pollfds));
		pollfds[0].fd = GpioSrdyFd; /* Wait for input */
		pollfds[0].events = POLLPRI; /* Wait for input */
		result = poll(pollfds, 1, timeout);
		//		debug_printf("poll() timeout\n");
		switch (result) 
		{
		case 0:
		{
			//Should not happen by default no Timeout.
			result = 2; //FORCE WRONG RESULT TO AVOID DEADLOCK CAUSE BY TIMEOUT
			debug_printf("[INT]:poll() timeout\n");
			//#ifdef __BIG_DEBUG__
			int val;
			if (  NPI_LNX_FAILURE == (val = HalGpioSrdyCheck(1)))
			{
				ret = val;
				npi_poll_terminate = 1;
			}
			else
			{
				// This should only happen once.
				result = global_srdy = val;
			}
			debug_printf("[INT]: SRDY: %d\n", val);
			//#endif
			break;
		}
		case -1:
		{
			debug_printf("[INT]:poll() error \n");
			npi_ipc_errno = NPI_LNX_ERROR_I2C_EVENT_THREAD_FAILED_POLL;
			ret = NPI_LNX_FAILURE;
			// Exit clean so main knows...
			npi_poll_terminate = 1;
		}
		default:
		{
			char * buf[64];
			read(pollfds[0].fd, buf, 64);
			result = global_srdy = HalGpioSrdyCheck(1);
			debug_printf("[INT]:Set global SRDY: %d\n", global_srdy);
		}
		break;
		}
		fflush(stdout);

		if (FALSE == result) //Means SRDY switch to low state
		{
			if ( (NPI_LNX_FAILURE == (ret = HalGpioMrdyCheck(1))))
			{
				debug_printf("[INT]:Fail to check MRDY \n");
				ret = NPI_LNX_FAILURE;
				// Exit clean so main knows...
				npi_poll_terminate = 1;
			}

			if (ret != NPI_LNX_FAILURE)
			{
				//MRDY High, This is a request from the RNP
				debug_printf("[INT]: MRDY High??: %d \n", ret);
				debug_printf("[INT]: send H2L to poll (srdy = %d)\n",
						global_srdy);
				pthread_cond_signal(&npi_srdy_H2L_poll);
			}

		} 
		else 
		{
			//Unknown Event
			//Do nothing for now ...
			//debug_printf("Unknown Event or timeout, ignore it, result:%d \n",result);
		}

	}

	pthread_cond_signal(&npi_srdy_H2L_poll);

	char *errorMsg;
	if (ret == NPI_LNX_FAILURE)
		errorMsg = "I2C Event thread exited with error. Please check global error message\n";
	else
		errorMsg = "I2C Event thread exited without error\n";

	NPI_LNX_IPC_NotifyError(NPI_LNX_ERROR_MODULE_MASK(NPI_LNX_ERROR_I2C_EVENT_THREAD_FAILED_LOCK), errorMsg);

	return ptr;
}
#endif //SRDY_INTERRUPT

#endif // NPI_I2C
/**************************************************************************************************
*/
