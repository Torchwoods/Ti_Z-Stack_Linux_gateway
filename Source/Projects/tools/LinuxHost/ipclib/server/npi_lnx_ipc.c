/**************************************************************************************************
 Filename:       npi_lnx_ipc.c
 Revised:        $Date: 2012-03-21 17:37:33 -0700 (Wed, 21 Mar 2012) $
 Revision:       $Revision: 246 $

 Description:    This file contains Linux platform specific NPI socket server
 implementation

 Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/

 Beej's Guide to Unix IPC was used in the development of this software:
 http://beej.us/guide/bgipc/output/html/multipage/intro.html#audience
 A small portion of the code from the associated code was also used. This
 code is Public Domain.


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
 *                                           Includes
 **************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// For stress testing data dump
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>

#ifndef NPI_UNIX
#include <ifaddrs.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <sys/time.h>

/* NPI includes */
#include "npi_lnx.h"
#include "npi_lnx_error.h"
#include "npi_lnx_ipc_rpc.h"

#if (defined NPI_SPI) && (NPI_SPI == TRUE)
#include "npi_lnx_spi.h"
#include "hal_spi.h"
#endif

#if (defined NPI_I2C) && (NPI_I2C == TRUE)
#include "npi_lnx_i2c.h"
#include "hal_i2c.h"
#endif

#if (defined NPI_UART) && (NPI_UART == TRUE)
#include "npi_lnx_uart.h"
#endif

// The following is only necessary because we always read out GPIO configuration
#include "hal_gpio.h"

#if (!defined NPI_SPI) || (NPI_SPI == FALSE)
#if (!defined NPI_I2C) || (NPI_I2C == FALSE)
#if (!defined NPI_UART) || (NPI_UART == FALSE)
#error "neither NPI_I2C, NPI_SPI, NPI_UART defined to TRUE, at least one mandatory. verify your makefile"
#endif
#endif
#endif

#ifdef __BIG_DEBUG__
#define debug_printf(fmt, ...) printf( fmt, ##__VA_ARGS__)
#else
#define debug_printf(fmt, ...) st (if (__BIG_DEBUG_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)
#endif

#define time_printf(fmt, ...) st (if (__DEBUG_TIME_ACTIVE == TRUE) printf( fmt, ##__VA_ARGS__);)

/**************************************************************************************************
 *                                        Externals
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Defines
 **************************************************************************************************/
#define NPI_SERVER_CONNECTION_QUEUE_SIZE        20

#define MAX(a,b)								((a > b) ? a : b)

/**************************************************************************************************
 *                                           Constant
 **************************************************************************************************/

const char* sectionNamesArray[3][2] =
{
	{
		"GPIO_SRDY.GPIO",
		"GPIO_SRDY.LEVEL_SHIFTER"
	},
	{
		"GPIO_MRDY.GPIO",
		"GPIO_MRDY.LEVEL_SHIFTER"
	},
	{
		"GPIO_RESET.GPIO",
		"GPIO_RESET.LEVEL_SHIFTER"
	},
};

//const char *port = "";
char port[128];

enum
{
	NPI_UART_FN_ARR_IDX,  	//0
	NPI_SPI_FN_ARR_IDX,		//1
	NPI_I2C_FN_ARR_IDX,		//2
};

const pNPI_OpenDeviceFn NPI_OpenDeviceFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_OpenDevice,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_OpenDevice,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_OpenDevice
#else
		NULL,
#endif
};

const pNPI_CloseDeviceFn NPI_CloseDeviceFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_CloseDevice,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_CloseDevice,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_CloseDevice
#else
		NULL,
#endif
};
const pNPI_SendAsynchDataFn NPI_SendAsynchDataFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_SendAsynchData,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SendAsynchData,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_SendAsynchData
#else
		NULL,
#endif
};
const pNPI_SendSynchDataFn NPI_SendSynchDataFnArr[] =
{
#if (defined NPI_UART) && (NPI_UART == TRUE)
		NPI_UART_SendSynchData,
#else
		NULL,
#endif
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SendSynchData,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_SendSynchData
#else
		NULL,
#endif
};

const pNPI_ResetSlaveFn NPI_ResetSlaveFnArr[] =
{
		NULL,
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_ResetSlave,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NPI_I2C_ResetSlave,
#else
		NULL,
#endif
};

const pNPI_SynchSlaveFn NPI_SynchSlaveFnArr[] =
{
		NULL,
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		NPI_SPI_SynchSlave,
#else
		NULL,
#endif
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
		NULL,
#else
		NULL,
#endif
};

/**************************************************************************************************
 *                                        Type definitions
 **************************************************************************************************/

/**************************************************************************************************
 *                                        Global Variables
 **************************************************************************************************/

int npi_ipc_errno;
int __BIG_DEBUG_ACTIVE = FALSE;
int __DEBUG_TIME_ACTIVE = FALSE;		// Do not enable by default.

/**************************************************************************************************
 *                                        Local Variables
 **************************************************************************************************/

// Socket handles
uint32 sNPIlisten;

// Socket connection file descriptors
fd_set activeConnectionsFDs;
int fdmax;
struct
{
	int list[NPI_SERVER_CONNECTION_QUEUE_SIZE];
	int size;
} activeConnections;

// NPI IPC Server buffers
char npi_ipc_buf[2][sizeof(npiMsgData_t)];

// Variables for Configuration
FILE *serialCfgFd;
char* pStrBufRoot;
char* devPath;
char* logPath;
halGpioCfg_t** gpioCfg;

#if (defined __DEBUG_TIME__) || (__STRESS_TEST__)
struct timeval curTime, startTime, prevTimeSend, prevTimeRec;
#endif //__DEBUG_TIME__

#ifdef __STRESS_TEST__
#define TIMING_STATS_SIZE                                                     500
#define TIMING_STATS_MS_DIV                                                    10
unsigned int timingStats[2][TIMING_STATS_SIZE + 1];
FILE *fpStressTestData;
#define STRESS_TEST_SUPPORTED_NUM_PAIRING_ENTRIES                              10
struct
{
	uint32 currentSeqNumber[STRESS_TEST_SUPPORTED_NUM_PAIRING_ENTRIES];
	struct{
		uint32 errorInSeqNum;
		uint32 seqNumIdentical;
	} recErrors;
} ST_Parameters_t[2] =
{
		{
				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0}
		},
		{
				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				{0, 0}
		}
};
#endif //__STRESS_TEST__
/**************************************************************************************************
 *                                     Local Function Prototypes
 **************************************************************************************************/

// NPI Callback Related
int NPI_AsynchMsgCback(npiMsgData_t *pMsg);

int SerialConfigParser(FILE* serialCfgFd, const char* section,
		const char* key, char* resString);

void NPI_LNX_IPC_Exit(int ret);

static uint8 devIdx = 0;

int NPI_LNX_IPC_SendData(uint8 len, int connection);
int NPI_LNX_IPC_ConnectionHandle(int connection);

int removeFromActiveList(int c);
int addToActiveList(int c);

void writeToNpiLnxLog(const char* str);

static int npi_ServerCmdHandle(npiMsgData_t *npi_ipc_buf);

/**************************************************************************************************
 * @fn          halDelay
 *
 * @brief       Delay for milliseconds.
 *              Do not invoke with zero.
 *              Do not invoke with greater than 500 msecs.
 *              Invoking with very high frequency and/or with long delays will start to
 *              significantly impact the real time performance of TimerA tasks because this will
 *              invisibly overrun the period when the TimerA count remaining, when this function
 *              is invoked, is less than the delay requested.
 *
 * input parameters
 *
 * @param       msecs - Milliseconds to delay in low power mode.
 * @param       sleep - Enforces blocking delay in low power mode if set.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halDelay(uint8 msecs, uint8 sleep)
{
	if (sleep)
	{
		//    usleep(msecs * 1000);
	}
}

void writeToNpiLnxLog(const char* str)
{
	int npiLnxLogFd, i = 0;
	char *fullStr = (char *)malloc(255);
	char *inStr = (char *)malloc(255);

	time_t timeNow;
	struct tm * timeNowinfo;

	time ( &timeNow );
	timeNowinfo = localtime ( &timeNow );

	sprintf(fullStr, "[%s", asctime(timeNowinfo));
	sprintf(inStr, "%s", str);
	// Remove \n characters
	fullStr[strlen(fullStr) - 2] = 0;
	for (i = strlen(str) - 1; i > MAX(strlen(str), 4); i--)
	{
		if (inStr[i] == '\n')
			inStr[i] = 0;
	}
	sprintf(fullStr, "%s] %s", fullStr, inStr);

	// Add global error code
	sprintf(fullStr, "%s. Error: %.8X\n", fullStr, npi_ipc_errno);

	// Write error message to /dev/npiLnxLog
	npiLnxLogFd = open(logPath,  O_WRONLY | O_APPEND | O_CREAT, S_IRWXU);
	if (npiLnxLogFd > 0)
	{
		write(npiLnxLogFd, fullStr, strlen(fullStr) + 1);
//		printf("Wrote:\n%s\n to npiLnxLog.log\n", fullStr, errno);
	}
	else
	{
		printf("Could not write \n%s\n to npiLnxLog. Error: %.8X\n", str, errno);
		perror("open");
	}
	close(npiLnxLogFd);
	free(fullStr);
	free(inStr);
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [config_file_name] [debug]\n", prog);
	puts("  config_file_name: the name of the config file to use, if not set, the default is ./RemoTI_RNP.cfg");
	puts("  debug: set debug options. 'debugAll' for both BIG and TIME, 'debugTime' for just TIME or 'debugBig' for just BIG \n");
	exit(1);
}

/**************************************************************************************************
 *
 * @fn          NPI Linux IPC Socket Server
 *
 * @brief       This is the main function
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 *
 **************************************************************************************************/
int main(int argc, char ** argv)
{
	int ret = NPI_LNX_SUCCESS;

	/**********************************************************************
	 * First step is to Configure the serial interface
	 **********************************************************************/

	// Variables for Configuration. Some are declared global to be used in unified graceful
	// exit function.
	char* strBuf;
	uint8 gpioIdx = 0;
	char* configFilePath;

	if (argc==1)
	{
		configFilePath = "./RemoTI_RNP.cfg";
	}
	else if (argc==2)
	{
		configFilePath = argv[1];
	}
	else if (argc==3)
	{
		configFilePath = argv[1];
		if (strcmp(argv[2], "debugAll") == 0)
		{
			__BIG_DEBUG_ACTIVE = TRUE;
			__DEBUG_TIME_ACTIVE = TRUE;
		}
		else if (strcmp(argv[2], "debugBig") == 0)
		{
			__BIG_DEBUG_ACTIVE = TRUE;
		}
		else if (strcmp(argv[2], "debugTime") == 0)
		{
			__DEBUG_TIME_ACTIVE = TRUE;
		}
		else
		{
			print_usage(argv[0]);
		}
	}
	else
	{
		printf("Too many arguments\n");
		print_usage(argv[0]);
	}


	// Allocate memory for string buffer and configuration buffer
	strBuf = (char*) malloc(128);
	memset(strBuf, 0, 128);
	pStrBufRoot = strBuf;
	devPath = (char*) malloc(128);
	logPath = (char*) malloc(128);
	memset(devPath, 0, 128);
	memset(logPath, 0, 128);
	gpioCfg = (halGpioCfg_t**) malloc(3 * sizeof(halGpioCfg_t*));
	debug_printf("gpioCfg \t\t\t\t%p\n",
			(void *)&(gpioCfg));
	for (gpioIdx = 0; gpioIdx < 3; gpioIdx++)
	{
		gpioCfg[gpioIdx] = (halGpioCfg_t*) malloc(sizeof(halGpioCfg_t));
		memset(gpioCfg[gpioIdx], 0, sizeof(halGpioCfg_t));
		debug_printf("gpioCfg[%d] \t\t\t\t%p\n",
				gpioIdx, (void *)&(gpioCfg[gpioIdx]));
		debug_printf("gpioCfg[%d].gpio \t\t\t%p\n",
				gpioIdx, (void *)&(gpioCfg[gpioIdx]->gpio));
		debug_printf("gpioCfg[%d].levelshifter \t\t%p\n",
				gpioIdx, (void *)&(gpioCfg[gpioIdx]->levelshifter));
	}

	// Open file for parsing
	serialCfgFd = fopen(configFilePath, "r");
	if (serialCfgFd == NULL)
	{
		//                            debug_
		printf("Could not open file '%s'\n", configFilePath);
		npi_ipc_errno = NPI_LNX_ERROR_IPC_OPEN_REMOTI_RNP_CFG;
		print_usage(argv[0]);
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

	// Get device type
	if (NPI_LNX_FAILURE == (SerialConfigParser(serialCfgFd, "DEVICE", "deviceKey", strBuf)))
	{
		printf("Could not find 'deviceKey' inside config file '%s'\n", configFilePath);
		npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_DEVICE_KEY;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

	// Copy from buffer to variable
	devIdx = strBuf[0] - '0';
	//            debug_
	printf("deviceKey = %i  (%s)\n", devIdx, strBuf);

	// Get path to the device
	strBuf = pStrBufRoot;
	if (NPI_LNX_FAILURE == (SerialConfigParser(serialCfgFd, "DEVICE", "devPath", strBuf)))
	{
		printf("Could not find 'devPath' inside config file '%s'\n", configFilePath);
		npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_DEVICE_PATH;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}
	// Copy from buffer to variable
	memcpy(devPath, strBuf, strlen(strBuf));
	//            debug_
	printf("devPath = '%s'\n", devPath);

	//            printf("devPath = ");
	//            for (i = 0; i < strlen(strBuf); i++)
	//            {
	//                            printf("_");
	//            }
	//            printf("<\n");
	// Get path to the log file
	strBuf = pStrBufRoot;
	if (NPI_LNX_FAILURE == (SerialConfigParser(serialCfgFd, "LOG", "log", strBuf)))
	{
		printf("Could not find 'log' inside config file '%s'\n", configFilePath);
		npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_LOG_PATH;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}
	// Copy from buffer to variable
	memcpy(logPath, strBuf, strlen(strBuf));
	//            debug_
	printf("logPath = '%s'\n", logPath);

	// GPIO configuration
	if ((devIdx == 1) || (devIdx == 2))
	{
		for (gpioIdx = 0; gpioIdx < 3; gpioIdx++) {
			// Get SRDY, MRDY or RESET GPIO
			debug_printf("gpioCfg[gpioIdx]->gpio \t\t\t%p\n",
					(void *)&(gpioCfg[gpioIdx]->gpio));

			// Get SRDY, MRDY or RESET GPIO value
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0],
					"value", strBuf)))
			{
			// Copy from buffer to variable
				debug_printf("strBuf \t\t\t\t\t%p\n",
						(void *)&strBuf);
				debug_printf("gpioCfg[gpioIdx]->gpio.value \t\t%p\n",
						(void *)&(gpioCfg[gpioIdx]->gpio.value));
				memcpy(gpioCfg[gpioIdx]->gpio.value, strBuf, strlen(strBuf));
				debug_printf("gpioCfg[%i]->gpio.value = '%s'\n",
						gpioIdx, gpioCfg[gpioIdx]->gpio.value);
			}
			else
			{
				printf("[CONFIG] ERROR , key 'value' is missing for mandatory GPIO %s\n", sectionNamesArray[gpioIdx][0]);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_DEVICE_GPIO(gpioIdx, 0, devIdx);
				NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
			}

			// Get SRDY, MRDY or RESET GPIO direction
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0],
					"direction", strBuf)))
			{
			// Copy from buffer to variable
				debug_printf("strBuf \t\t\t\t\t%p\n",
						(void *)&strBuf);
				debug_printf("gpioCfg[gpioIdx]->gpio.direction \t%p\n",
						(void *)&(gpioCfg[gpioIdx]->gpio.direction));
				memcpy(gpioCfg[gpioIdx]->gpio.direction, strBuf,
						strlen(strBuf));
				debug_printf("gpioCfg[%i]->gpio.direction = '%s'\n",
						gpioIdx, gpioCfg[gpioIdx]->gpio.direction);
			}
			else
			{
				printf("[CONFIG] ERROR , key 'direction' is missing for mandatory GPIO %s\n", sectionNamesArray[gpioIdx][0]);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_DEVICE_GPIO(gpioIdx, 0, devIdx);
				NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
			}

#ifdef SRDY_INTERRUPT
			// Get SRDY, MRDY or RESET GPIO edge
			if (gpioIdx == 0)
			{
				strBuf = pStrBufRoot;
				if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, sectionNamesArray[gpioIdx][0],
						"edge", strBuf)))
				{
					// Copy from buffer to variable
					debug_printf("strBuf \t\t\t\t\t%p\n",
							(void *)&strBuf);
					debug_printf("gpioCfg[gpioIdx]->gpio.edge \t%p\n",
							(void *)&(gpioCfg[gpioIdx]->gpio.edge));
					memcpy(gpioCfg[gpioIdx]->gpio.edge, strBuf, strlen(strBuf));
					debug_printf("gpioCfg[%i]->gpio.edge = '%s'\n",
							gpioIdx, gpioCfg[gpioIdx]->gpio.edge);
				}
				else
				{
					printf("[CONFIG] ERROR , key 'edge' is missing for mandatory GPIO %s\n", sectionNamesArray[gpioIdx][0]);
					npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOTI_RNP_CFG_PARSER_DEVICE_GPIO(gpioIdx, 0, devIdx);
					NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
				}
			}
#endif
			// Get SRDY, MRDY or RESET GPIO Active High/Low
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd,
					sectionNamesArray[gpioIdx][1], "active_high_low",
					strBuf))) {
			// Copy from buffer to variable
			gpioCfg[gpioIdx]->gpio.active_high_low = strBuf[0] - '0';
			debug_printf("gpioCfg[%i]->gpio.active_high_low = %d\n",
							gpioIdx, gpioCfg[gpioIdx]->gpio.active_high_low);
			}
			else
				printf("[CONFIG] Warning , key 'active_high_low' is missing for optional GPIO %s\n", sectionNamesArray[gpioIdx][0]);

			// Get SRDY, MRDY or RESET Level Shifter
			debug_printf("gpioCfg[gpioIdx]->levelshifter \t\t\t%p\n",
					(void *)&(gpioCfg[gpioIdx]->levelshifter));

			// Get SRDY, MRDY or RESET Level Shifter value
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd,
					sectionNamesArray[gpioIdx][1], "value", strBuf)))
			{
			// Copy from buffer to variable
			memcpy(gpioCfg[gpioIdx]->levelshifter.value, strBuf,
					strlen(strBuf));
			debug_printf("gpioCfg[%i]->levelshifter.value = '%s'\n",
						gpioIdx, gpioCfg[gpioIdx]->levelshifter.value);
			}
			else
				printf("[CONFIG] Warning , key 'value' is missing for optional GPIO %s\n", sectionNamesArray[gpioIdx][1]);

			// Get SRDY, MRDY or RESET Level Shifter direction
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd,
					sectionNamesArray[gpioIdx][1], "direction", strBuf)))
			{
			// Copy from buffer to variable
			memcpy(gpioCfg[gpioIdx]->levelshifter.direction, strBuf,
					strlen(strBuf));
			debug_printf("gpioCfg[%i]->levelshifter.direction = '%s'\n",
						gpioIdx, gpioCfg[gpioIdx]->levelshifter.direction);
			}
			else
				printf("[CONFIG] Warning , key 'direction' is missing for optional GPIO %s\n", sectionNamesArray[gpioIdx][1]);


			// Get SRDY, MRDY or RESET Level Shifter Active High/Low
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd,
					sectionNamesArray[gpioIdx][1], "active_high_low", strBuf)))
			{
			// Copy from buffer to variable
			gpioCfg[gpioIdx]->levelshifter.active_high_low = atoi(strBuf);
			debug_printf("gpioCfg[%i]->levelshifter.active_high_low = %d\n",
					gpioIdx, gpioCfg[gpioIdx]->levelshifter.active_high_low);
			}
			else
				printf("[CONFIG] Warning , key 'active_high_low' is missing for optional GPIO %s\n", sectionNamesArray[gpioIdx][1]);
		}
	}


	/**********************************************************************
	 * Now open the serial interface
	 */
	switch(devIdx)
	{
		case NPI_UART_FN_ARR_IDX:
		#if (defined NPI_UART) && (NPI_UART == TRUE)
		{
			npiUartCfg_t uartCfg;
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "UART", "speed", strBuf)))
			{
				uartCfg.speed = atoi(strBuf);
			}
			else
			{
				uartCfg.speed=115200;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "UART", "flowcontrol", strBuf)))
			{
				uartCfg.flowcontrol = atoi(strBuf);
			}
			else
			{
				uartCfg.flowcontrol=0;
			}
			ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiUartCfg_t *)&uartCfg);
		}
		#endif
		break;
	case NPI_SPI_FN_ARR_IDX:
		#if (defined NPI_SPI) && (NPI_SPI == TRUE)
		{
			halSpiCfg_t halSpiCfg;
			npiSpiCfg_t npiSpiCfg;
			strBuf = pStrBufRoot;
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "speed", strBuf)))
			{
				halSpiCfg.speed = strtol(strBuf, NULL, 10);
			}
			else
			{
				halSpiCfg.speed = 500000;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "mode", strBuf)))
			{
				halSpiCfg.mode = strtol(strBuf, NULL, 16);
			}
			else
			{
				halSpiCfg.mode = 0;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "bitsPerWord", strBuf)))
			{
				halSpiCfg.bitsPerWord = strtol(strBuf, NULL, 10);
			}
			else
			{
				halSpiCfg.bitsPerWord = 0;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "useFullDuplexAPI", strBuf)))
			{
				halSpiCfg.useFullDuplexAPI = strtol(strBuf, NULL, 10);
			}
			else
			{
				halSpiCfg.useFullDuplexAPI = TRUE;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "earlyMrdyDeAssert", strBuf)))
			{
				npiSpiCfg.earlyMrdyDeAssert = strtol(strBuf, NULL, 10);
			}
			else
			{
				// If it is not defined then set value for RNP
				npiSpiCfg.earlyMrdyDeAssert = TRUE;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "detectResetFromSlowSrdyAssert", strBuf)))
			{
				npiSpiCfg.detectResetFromSlowSrdyAssert = strtol(strBuf, NULL, 10);
			}
			else
			{
				// If it is not defined then set value for RNP
				npiSpiCfg.detectResetFromSlowSrdyAssert = TRUE;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "forceRunOnReset", strBuf)))
			{
				npiSpiCfg.forceRunOnReset = strtol(strBuf, NULL, 16);
			}
			else
			{
				// If it is not defined then set value for RNP
				npiSpiCfg.forceRunOnReset = NPI_LNX_UINT8_ERROR;
			}
			if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "srdyMrdyHandshakeSupport", strBuf)))
			{
				npiSpiCfg.srdyMrdyHandshakeSupport = strtol(strBuf, NULL, 10);
			}
			else
			{
				// If it is not defined then set value for RNP
				npiSpiCfg.srdyMrdyHandshakeSupport = TRUE;
			}

			npiSpiCfg.spiCfg = &halSpiCfg;
			npiSpiCfg.gpioCfg = gpioCfg;

			// Now open device for processing
			ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiSpiCfg_t *) &npiSpiCfg);

			// Perform Reset of the RNP
			(NPI_ResetSlaveFnArr[devIdx])();

			// Do the Hw Handshake
			(NPI_SynchSlaveFnArr[devIdx])();
		}
		#endif
		break;

	case NPI_I2C_FN_ARR_IDX:
		#if (defined NPI_I2C) && (NPI_I2C == TRUE)
			{
				npiI2cCfg_t i2cCfg;
				i2cCfg.gpioCfg = gpioCfg;

				// Open the Device and perform a reset
				ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiI2cCfg_t *) &i2cCfg);
			}
		#endif
		break;
	default:
		ret = NPI_LNX_FAILURE;
		break;
	}

	// Get port from configuration file
	if (NPI_LNX_FAILURE == (SerialConfigParser(serialCfgFd, "PORT", "port", strBuf)))
	{
		// Fall back to default if port was not found in the configuration file
		strncpy(port, NPI_PORT, 128);
		printf(
				"Warning! Port not found in configuration file. Will use default port: %s\n",
				port);
	} 
	else 
	{
		strncpy(port, strBuf, 128);
	}


	// The following will exit if ret != SUCCESS
	NPI_LNX_IPC_Exit(ret);


#ifdef __STRESS_TEST__
	/**********************************************************************
	 * Setup StressTesting
	 **********************************************************************/

	int i = 0, fdStressTestData, done=0;
	char pathName[128];
	do
	{
		sprintf(pathName, "results/stressTestData%.4d.txt", i++);
		printf("%s\n", pathName);
		fdStressTestData = open( pathName , O_CREAT | O_EXCL | O_WRONLY, S_IWRITE | S_IREAD );
		printf("fd = %d\n", fdStressTestData);
		if (fdStressTestData >= 0)
			done = 1;
		else
			close(fdStressTestData);
	} while (done == 0);
	// Now it's safe to open the file
	fpStressTestData = fopen(pathName, "w");

	time_t rawTime;
	time(&rawTime);
	fprintf(fpStressTestData, "*******************************************************************\n");
	fprintf(fpStressTestData, "\nTiming Statistics file created on %s\n\n", ctime(&rawTime));
	fprintf(fpStressTestData, "*******************************************************************\n");
#endif //__STRESS_TEST__


	/**********************************************************************
	 * Now that everything has been initialized and configured, let's open
	 * a socket and begin listening.
	 **********************************************************************/

#ifdef NPI_UNIX
	int len;
	struct sockaddr_un local, their_addr;
#else
	struct sockaddr_storage their_addr;
	int status;
	struct addrinfo hints;
	struct addrinfo *servinfo;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	printf("Port: %s\n", port);

	if ((status = getaddrinfo(NULL, port, &hints, &servinfo)) != 0)
	{
		fprintf(stderr, "getaddrinfo error: %s\n", gai_strerror(status));
		//                port = NPI_PORT;
		strncpy(port, NPI_PORT, 128);
		printf("Trying default port: %s instead\n", port);
		if ((status = getaddrinfo(NULL, port, &hints, &servinfo)) != 0)
		{
			fprintf(stderr, "getaddrinfo error: %s\n", gai_strerror(status));
			npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_GET_ADDRESS_INFO;
			NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
		}
	}

	printf("Following IP addresses are available:\n\n");
	{
		struct ifaddrs * ifAddrStruct=NULL;
		struct ifaddrs * ifa=NULL;
		void * tmpAddrPtr=NULL;

		getifaddrs(&ifAddrStruct);

		for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
		{
			if (ifa ->ifa_addr->sa_family==AF_INET)
			{ // check it is IP4
				// is a valid IP4 Address
				tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
				char addressBuffer[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
				printf(" IPv4: interface: %s\t IP Address %s\n", ifa->ifa_name, addressBuffer);
			}
			else if (ifa->ifa_addr->sa_family==AF_INET6)
			{ // check it is IP6
				// is a valid IP6 Address
				tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
				char addressBuffer[INET6_ADDRSTRLEN];
				inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
				printf(" IPv6: interface: %s\t IP Address %s\n", ifa->ifa_name, addressBuffer);
			}
		}
		if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
	}

	printf("The socket will listen on the following IP addresses:\n\n");


	struct addrinfo *p;
	char ipstr[INET6_ADDRSTRLEN];
	for (p = servinfo; p != NULL; p = p->ai_next)
	{
		void *addr;
		char *ipver;

		// get the pointer to the address itself,
		// different fields in IPv4 and IPv6:
		if (p->ai_family == AF_INET)
		{ // IPv4
			struct sockaddr_in *ipv4 = (struct sockaddr_in *) p->ai_addr;
			addr = &(ipv4->sin_addr);
			ipver = "IPv4";
		} else
		{ // IPv6
			struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *) p->ai_addr;
			addr = &(ipv6->sin6_addr);
			ipver = "IPv6";
		}

		// convert the IP to a string and print it:
		inet_ntop(p->ai_family, addr, ipstr, sizeof ipstr);
		printf("  %s: %s\n", ipver, ipstr);
	}
	printf("0.0.0.0 means it will listen to all available IP address\n\n");

#endif

#ifdef NPI_UNIX
	// Create the socket
	sNPIlisten = socket(AF_UNIX, SOCK_STREAM, 0);

	// Bind socket to a Unix domain address
	local.sun_family = AF_UNIX;
	strcpy(local.sun_path, "echo_socket");
	unlink(local.sun_path);
	len = strlen(local.sun_path) + sizeof(local.sun_family);
	if (bind(sNPIlisten, (struct sockaddr *)&local, len) == -1)
	{
		perror("bind");
		writeToNpiLnxLog("Port is probably already in use, please select an available port\n");
		debug_printf("Port is probably already in use, please select an available port\n");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_BIND;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

#else
	sNPIlisten = socket(servinfo->ai_family, servinfo->ai_socktype,
			servinfo->ai_protocol);

	int yes = 1;
	// avoid "Address already in use" error message
	if (setsockopt(sNPIlisten, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))
			== -1)
	{
		perror("setsockopt");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_SET_REUSE_ADDRESS;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

	// Bind socket
	if (bind(sNPIlisten, servinfo->ai_addr, servinfo->ai_addrlen) == -1)
	{
		perror("bind");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_BIND;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

#endif

	// Listen, allow 20 connections in the queue
	if (listen(sNPIlisten, NPI_SERVER_CONNECTION_QUEUE_SIZE) == -1)
	{
		perror("listen");
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_LISTEN;
		NPI_LNX_IPC_Exit(NPI_LNX_FAILURE);
	}

	fd_set activeConnectionsFDsSafeCopy;
	int justConnected, c;

	// Connection main loop. Cannot get here with ret != SUCCESS

	char *toNpiLnxLog = (char *)malloc(AP_MAX_BUF_LEN);

	// Clear file descriptor sets
	FD_ZERO(&activeConnectionsFDs);
	FD_ZERO(&activeConnectionsFDsSafeCopy);

	// Add the listener to the set
	FD_SET(sNPIlisten, &activeConnectionsFDs);
	fdmax = sNPIlisten;

#if (defined __DEBUG_TIME__) || (__STRESS_TEST__)
	gettimeofday(&startTime, NULL);
#endif // (defined __DEBUG_TIME__) || (__STRESS_TEST__)
	//                                            debug_
	printf("waiting for first connection on #%d...\n", sNPIlisten);

	while (ret == NPI_LNX_SUCCESS)
	{
		activeConnectionsFDsSafeCopy = activeConnectionsFDs;

		// First use select to find activity on the sockets
		if (select (fdmax + 1, &activeConnectionsFDsSafeCopy, NULL, NULL, NULL) == -1)
		{
			if (errno != EINTR)
			{
				perror("select");
				npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_SELECT_CHECK_ERRNO;
				ret = NPI_LNX_FAILURE;
				break;
			}
			continue;
		}

		// Then process this activity
		for (c = 0; c <= fdmax; c++)
		{
			if (FD_ISSET(c, &activeConnectionsFDsSafeCopy))
			{
				if (c == sNPIlisten)
				{
					int addrLen = 0;
					// Accept a connection from a client.
					addrLen = sizeof(their_addr);
					justConnected = accept(sNPIlisten,
							(struct sockaddr *) &their_addr,
							(socklen_t *) &addrLen);

					if (justConnected == -1)
					{
						perror("accept");
						npi_ipc_errno = NPI_LNX_ERROR_IPC_SOCKET_ACCEPT;
						ret = NPI_LNX_FAILURE;
						break;
					}
					else
					{
#ifndef NPI_UNIX
						char ipstr[INET6_ADDRSTRLEN];
						char ipstr2[INET6_ADDRSTRLEN];
#endif //NPI_UNIX
						FD_SET(justConnected, &activeConnectionsFDs);
						if (justConnected > fdmax)
							fdmax = justConnected;
#ifdef NPI_UNIX
						sprintf(toNpiLnxLog, "Connected to #%d.", justConnected);
#else
						//                                            debug_
						inet_ntop(AF_INET, &((struct sockaddr_in *) &their_addr)->sin_addr, ipstr, sizeof ipstr);
						inet_ntop(AF_INET6, &((struct sockaddr_in6 *)&their_addr)->sin6_addr, ipstr2, sizeof ipstr2);
						sprintf(toNpiLnxLog, "Connected to #%d.(%s / %s)", justConnected, ipstr, ipstr2);
#endif //NPI_UNIX
						printf("%s\n", toNpiLnxLog);
						writeToNpiLnxLog(toNpiLnxLog);
						ret = addToActiveList(justConnected);

#ifdef __DEBUG_TIME__
						if (__DEBUG_TIME_ACTIVE == TRUE)
						{
							gettimeofday(&startTime, NULL);
						}
#endif //__DEBUG_TIME__
					}
				}
				else
				{
					ret = NPI_LNX_IPC_ConnectionHandle(c);
					if (ret == NPI_LNX_SUCCESS)
					{
						// Everything is ok
					}
					else
					{
						uint8 childThread;
						switch (npi_ipc_errno)
						{
						case NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT:
							close(c);
							printf("Removing connection #%d\n", c);
							// Connection closed. Remove from set
							FD_CLR(c, &activeConnectionsFDs);
							// We should now set ret to NPI_SUCCESS, but there is still one fatal error
							// possibility so simply set ret = to return value from removeFromActiveList().
							ret = removeFromActiveList(c);
							sprintf(toNpiLnxLog, "Removed connection #%d", c);
							//							printf("%s\n", toNpiLnxLog);
							writeToNpiLnxLog(toNpiLnxLog);
							break;
						case NPI_LNX_ERROR_UART_SEND_SYNCH_TIMEDOUT:
							//This case can happen in some particular condition:
							// if the network is in BOOT mode, it will not answer any synchronous request other than SYS_BOOT request.
							// if we exit immediately, we will never be able to recover the NP device.
							// This may be replace in the future by an update of the RNP behavior
							printf("Synchronous Request Timeout...");
							sprintf(toNpiLnxLog, "Removed connection #%d", c);
							//							printf("%s\n", toNpiLnxLog);
							writeToNpiLnxLog(toNpiLnxLog);
							ret = NPI_LNX_SUCCESS;
							npi_ipc_errno = NPI_LNX_SUCCESS;
							break;
						default:
							if (npi_ipc_errno == NPI_LNX_SUCCESS)
							{
								// Do not report and abort if there is no real error.
								ret = NPI_LNX_SUCCESS;
							}
							else if (NPI_LNX_ERROR_JUST_WARNING(npi_ipc_errno))
							{
								// This may be caused by an unexpected reset. Write it to the log,
								// but keep going.
								// Everything about the error can be found in the message, and in npi_ipc_errno:
								childThread = ((npiMsgData_t *) npi_ipc_buf[0])->cmdId;
								sprintf(toNpiLnxLog, "Child thread with ID %d in module %d reported error:\t%s",
										NPI_LNX_ERROR_THREAD(childThread),
										NPI_LNX_ERROR_MODULE(childThread),
										(char *)(((npiMsgData_t *) npi_ipc_buf[0])->pData));
								//							printf("%s\n", toNpiLnxLog);
								writeToNpiLnxLog(toNpiLnxLog);
								// Force continuation
								ret = NPI_LNX_SUCCESS;
							}
							else
							{
								//							debug_
								printf("[ERR] npi_ipc_errno 0x%.8X\n", npi_ipc_errno);
								// Everything about the error can be found in the message, and in npi_ipc_errno:
								childThread = ((npiMsgData_t *) npi_ipc_buf[0])->cmdId;
								sprintf(toNpiLnxLog, "Child thread with ID %d in module %d reported error:\t%s",
										NPI_LNX_ERROR_THREAD(childThread),
										NPI_LNX_ERROR_MODULE(childThread),
										(char *)(((npiMsgData_t *) npi_ipc_buf[0])->pData));
								//							printf("%s\n", toNpiLnxLog);
								writeToNpiLnxLog(toNpiLnxLog);
							}
							break;
						}

						// Check if error requested a reset
						if (NPI_LNX_ERROR_RESET_REQUESTED(npi_ipc_errno))
						{
							// Yes, utilize server control API to reset current device
							// Do it by reconnecting so that threads are kept synchronized
							npiMsgData_t npi_ipc_buf_tmp;
							int localRet = NPI_LNX_SUCCESS;
							printf("Reset was requested, so try to disconnect device %d\n", devIdx);
							npi_ipc_buf_tmp.cmdId = NPI_LNX_CMD_ID_DISCONNECT_DEVICE;
							localRet = npi_ServerCmdHandle((npiMsgData_t *)&npi_ipc_buf_tmp);
							printf("Disconnection from device %d was %s\n", devIdx, (localRet == NPI_LNX_SUCCESS) ? "successful" : "unsuccessful");
							if (localRet == NPI_LNX_SUCCESS)
							{
								printf("Then try to connect device %d again\n", devIdx);
								int bigDebugWas = __BIG_DEBUG_ACTIVE;
								if (bigDebugWas == FALSE)
								{
									__BIG_DEBUG_ACTIVE = TRUE;
									printf("__BIG_DEBUG_ACTIVE set to TRUE\n");
								}
								npi_ipc_buf_tmp.cmdId = NPI_LNX_CMD_ID_CONNECT_DEVICE;
								localRet = npi_ServerCmdHandle((npiMsgData_t *)&npi_ipc_buf_tmp);
								printf("Reconnection to device %d was %s\n", devIdx, (localRet == NPI_LNX_SUCCESS) ? "successful" : "unsuccessful");
								if (bigDebugWas == FALSE)
								{
									__BIG_DEBUG_ACTIVE = FALSE;
									printf("__BIG_DEBUG_ACTIVE set to FALSE\n");
								}
							}
						}

						// If this error was sent through socket; close this connection
						if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_CMD_TYPE_MASK) == RPC_CMD_NOTIFY_ERR)
						{
							close(c);
							printf("Removing connection #%d\n", c);
							// Connection closed. Remove from set
							FD_CLR(c, &activeConnectionsFDs);
						}
					}
				}
			}
		}
	}
	free(toNpiLnxLog);


	printf("Exit socket while loop\n");
	/**********************************************************************
	 * Remember to close down all connections
	 *********************************************************************/

#ifndef NPI_UNIX
	freeaddrinfo(servinfo); // free the linked-list
#endif //NPI_UNIX
	(NPI_CloseDeviceFnArr[devIdx])();

	// Free all remaining memory
	NPI_LNX_IPC_Exit(NPI_LNX_SUCCESS + 1);

#if (defined __STRESS_TEST__) && (__STRESS_TEST__ == TRUE)
	//            close(fpStressTestData);
	//            close(fdStressTestData);
#endif //(defined __STRESS_TEST__) && (__STRESS_TEST__ == TRUE)

	return ret;
}


/**************************************************************************************************
 *
 * @fn          addToActiveList
 *
 * @brief       Manage active connections, add to list
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      -1 if something went wrong, 0 if success
 *
 **************************************************************************************************/

int addToActiveList(int c)
{
	if (activeConnections.size <= NPI_SERVER_CONNECTION_QUEUE_SIZE)
	{
		// Entry at position activeConnections.size is always the last available entry
		activeConnections.list[activeConnections.size] = c;

		// Increment size
		activeConnections.size++;

		return NPI_LNX_SUCCESS;
	}
	else
	{
		// There's no more room in the list
		npi_ipc_errno = NPI_LNX_ERROR_IPC_ADD_TO_ACTIVE_LIST_NO_ROOM;
		return NPI_LNX_FAILURE;
	}
}

/**************************************************************************************************
 *
 * @fn          removeFromActiveList
 *
 * @brief       Manage active connections, remove from list. Re organize so list is full
 * 				up to its declared size
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      -1 if something went wrong, 0 if success
 *
 **************************************************************************************************/

int removeFromActiveList(int c)
{
	int i;
	// Find entry
	for (i = 0; i < activeConnections.size; i++)
	{
		if (activeConnections.list[i] == c)
			break;
	}


	if (i < activeConnections.size)
	{
		//Check if the last active conection has been removed
		if (activeConnections.size == 1)
		{
			//continue to wait for new connection
			activeConnections.size = 0;
			activeConnections.list[0] = 0;
			debug_printf("No  Active Connections");
		}
		else
		{

			// Found our entry, replace this entry by the last entry
			activeConnections.list[i] = activeConnections.list[activeConnections.size - 1];

			// Decrement size
			activeConnections.size--;
#ifdef __BIG_DEBUG__
			printf("Remaining Active Connections: #%d", activeConnections.list[0]);
			// Send data to all connections, except listener
			for (i = 1; i < activeConnections.size; i++)
			{
				printf(", #%d", activeConnections.list[i]);
			}
			printf("\n");
#endif //__BIG_DEBUG__
		}
		return NPI_LNX_SUCCESS;
	}
	else
	{
		// Could not find entry
		npi_ipc_errno = NPI_LNX_ERROR_IPC_REMOVE_FROM_ACTIVE_LIST_NOT_FOUND;
		return NPI_LNX_FAILURE;
	}
}

/**************************************************************************************************
 *
 * @fn          NPI_LNX_IPC_ConnectionHandle
 *
 * @brief       Handle connections
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 *
 **************************************************************************************************/
int NPI_LNX_IPC_ConnectionHandle(int connection)
{
	int n, i, ret = NPI_LNX_SUCCESS;

	// Handle the connection
	debug_printf("Receive message...\n");

	// Receive only NPI header first. Then then number of bytes indicated by length.
	n = recv(connection, npi_ipc_buf[0], RPC_FRAME_HDR_SZ, 0);
	if (n <= 0)
	{
		if (n < 0)
		{
			perror("recv");
			if ( (errno == ENOTSOCK) || (errno == EPIPE))
			{
				debug_printf("[ERROR] Tried to read #%d as socket\n", connection);
				debug_printf("Will disconnect #%d\n", connection);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
				ret = NPI_LNX_FAILURE;
			}
			else if (errno == ECONNRESET)
			{
//				debug_
				printf("[WARNING] Client disconnect while attempting to send to it\n");
				debug_printf("Will disconnect #%d\n", connection);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
				ret = NPI_LNX_FAILURE;
			}
			else
			{
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_CHECK_ERRNO;
				ret = NPI_LNX_FAILURE;
			}
		}
		else
		{
			debug_printf("Will disconnect #%d\n", connection);
			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
			ret = NPI_LNX_FAILURE;
		}
	}
	else if (n == RPC_FRAME_HDR_SZ)
	{
		// Now read out the payload of the NPI message, if it exists
		if (((npiMsgData_t *) npi_ipc_buf[0])->len > 0)
		{
			n = recv(connection, (uint8*) &npi_ipc_buf[0][RPC_FRAME_HDR_SZ], ((npiMsgData_t *) npi_ipc_buf[0])->len , 0);
			if (n != ((npiMsgData_t *) npi_ipc_buf[0])->len)
			{
				printf("[ERR] Could not read out the NPI payload. Requested %d, but read %d!\n",
						((npiMsgData_t *) npi_ipc_buf[0])->len, n);
				npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_TOO_FEW_BYTES;
				ret = NPI_LNX_FAILURE;
				if (n < 0)
				{
					perror("recv");
					// Disconnect this
					npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT;
					ret = NPI_LNX_FAILURE;
				}
			}
			else
			{
				ret = NPI_LNX_SUCCESS;
			}
			// n is only used by debug traces from here on, but add header length
			// so the whole message is written out
			n += RPC_FRAME_HDR_SZ;
		}
		/*
		 * Take the message from the client and pass it to the NPI
		 */
#ifdef __DEBUG_TIME__
		if (__DEBUG_TIME_ACTIVE == TRUE)
		{
			//            debug_
			gettimeofday(&curTime, NULL);
			long int diffPrev;
			int t = 0;
			if (curTime.tv_usec >= prevTimeRec.tv_usec)
			{
				diffPrev = curTime.tv_usec - prevTimeRec.tv_usec;
			}
			else
			{
				diffPrev = (curTime.tv_usec + 1000000) - prevTimeRec.tv_usec;
				t = 1;
			}

#ifdef __STRESS_TEST__
			if (diffPrev < 500000)
				timingStats[0][diffPrev % 1000]++;
			else
				timingStats[0][500]++;
#endif //__STRESS_TEST__
			int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
			int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
			//            debug_
			time_printf("[<-- %.3d:%.2d:%.2d.%.6ld (+%ld.%6ld)] %.2d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData: \040 ",
					hours,											// hours
					minutes,										// minutes
					(int)(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
					(long int)curTime.tv_usec,
					curTime.tv_sec - prevTimeRec.tv_sec - t,
					diffPrev,
					((npiMsgData_t *) npi_ipc_buf[0])->len,
					((npiMsgData_t *) npi_ipc_buf[0])->subSys,
					((npiMsgData_t *) npi_ipc_buf[0])->cmdId);
			prevTimeRec = curTime;

			for (i = 0; i < ((npiMsgData_t *) npi_ipc_buf[0])->len; i++)
			{
				//			debug_
				time_printf(" 0x%.2X",
						((npiMsgData_t *) npi_ipc_buf[0])->pData[i]);
			}
			//            debug_
			time_printf("\n");
		}
#endif //__DEBUG_TIME__


		if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_CMD_TYPE_MASK) == RPC_CMD_SREQ)
		{
			debug_printf("NPI SREQ:  (len %d)", n);
			for (i = 0; i < n; i++)
			{
				debug_printf(" 0x%.2X", (uint8)npi_ipc_buf[0][i]);
			}
			debug_printf("\n");

			if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_SUBSYSTEM_MASK) == RPC_SYS_SRV_CTRL)
			{

				//SREQ Command send to this server.
				ret = npi_ServerCmdHandle((npiMsgData_t *)npi_ipc_buf[0]);
			}
			else
			{
				uint8 sreqHdr[RPC_FRAME_HDR_SZ] = {0};
				// Retain the header for later integrety check
				memcpy(sreqHdr, npi_ipc_buf[0], RPC_FRAME_HDR_SZ);
				// Synchronous request requires an answer...
				ret = (NPI_SendSynchDataFnArr[devIdx])(
						(npiMsgData_t *) npi_ipc_buf[0]);
				if ( (ret != NPI_LNX_SUCCESS) &&
						( (npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT) ||
							(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT) ))
				{
					// Report this error to client through a pseudo response
					((npiMsgData_t *) npi_ipc_buf[0])->len = 1;
					((npiMsgData_t *) npi_ipc_buf[0])->pData[0] = 0xFF;
				}
				else
				{
					// Capture incoherent SRSP, check type and subsystem
					if ( ( (((npiMsgData_t *) npi_ipc_buf[0])->subSys & ~(RPC_SUBSYSTEM_MASK)) != RPC_CMD_SRSP )
						||
						 ( (((npiMsgData_t *) npi_ipc_buf[0])->subSys & (RPC_SUBSYSTEM_MASK)) != (sreqHdr[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK))
						)
					{
						// Report this error to client through a pseudo response
						((npiMsgData_t *) npi_ipc_buf[0])->len = 1;
						((npiMsgData_t *) npi_ipc_buf[0])->subSys = (sreqHdr[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) | RPC_CMD_SRSP;
						((npiMsgData_t *) npi_ipc_buf[0])->cmdId = sreqHdr[RPC_POS_CMD1];
						((npiMsgData_t *) npi_ipc_buf[0])->pData[0] = 0xFF;
					}
				}
			}

			if ( (ret == NPI_LNX_SUCCESS) ||
						(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT) ||
						(npi_ipc_errno == NPI_LNX_ERROR_HAL_GPIO_WAIT_SRDY_SET_POLL_TIMEDOUT) )
			{
				n = ((npiMsgData_t *) npi_ipc_buf[0])->len + RPC_FRAME_HDR_SZ;

				// Copy response into transmission buffer
				memcpy(npi_ipc_buf[1], npi_ipc_buf[0], n);

				// Command type is not set, so set it here
				((npiMsgData_t *) npi_ipc_buf[1])->subSys |= RPC_CMD_SRSP;

				if (n > 0)
				{
					debug_printf("NPI SRSP: (len %d)", n);
					for (i = 0; i < n; i++)
					{
						debug_printf(" 0x%.2X", (uint8)npi_ipc_buf[1][i]);
					}
					debug_printf("\n");
				}
				else
				{
					printf("[ERR] SRSP is 0!\n");
				}

				//			pthread_mutex_lock(&npiSyncRespLock);
				// Send bytes
				ret = NPI_LNX_IPC_SendData(n, connection);
			}
			else
			{
				// Keep status from NPI_SendSynchDataFnArr
				debug_printf("[ERR] SRSP: npi_ipc_errno 0x%.8X\n", npi_ipc_errno);
			}
		}
		else if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_CMD_TYPE_MASK) == RPC_CMD_AREQ)
		{
			debug_printf("NPI AREQ: (len %d)", n);
			for (i = 0; i < n; i++)
			{
				debug_printf(" 0x%.2X", (uint8)npi_ipc_buf[0][i]);
			}
			debug_printf("\n");

			if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_SUBSYSTEM_MASK) == RPC_SYS_SRV_CTRL)
			{
				//AREQ Command send to this server.
				ret = npi_ServerCmdHandle((npiMsgData_t *)npi_ipc_buf[0]);
			}
			else
			{
				// Asynchronous request may just be sent
				ret = (NPI_SendAsynchDataFnArr[devIdx])(
						(npiMsgData_t *) npi_ipc_buf[0]);
			}
		}
		else if (((uint8) (((npiMsgData_t *) npi_ipc_buf[0])->subSys) & (uint8) RPC_CMD_TYPE_MASK)  == RPC_CMD_NOTIFY_ERR)
		{
			// An error occurred in a child thread.
			ret = NPI_LNX_FAILURE;
		}
		else
		{
			printf("Can only accept AREQ or SREQ for now...\n");
			printf("Unknown: (len %d)", n);
			for (i = 0; i < n; i++)
			{
				printf(" 0x%.2X", (uint8)npi_ipc_buf[0][i]);
			}
			printf("\n");

			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INCOMPATIBLE_CMD_TYPE;
			ret = NPI_LNX_FAILURE;
		}
	}

#if (defined __BIG_DEBUG__) && (__BIG_DEBUG__ == TRUE)
	// This will effectively result in an echo
	memcpy(npi_ipc_buf[1], npi_ipc_buf[0], sizeof(npiMsgData_t));
#endif

	if ((ret == NPI_LNX_FAILURE) && (npi_ipc_errno == NPI_LNX_ERROR_IPC_RECV_DATA_DISCONNECT))
	{
		debug_printf("Done with %d\n", connection);
	}
	else
	{
		debug_printf("!Done\n");
	}

	return ret;
}

/**************************************************************************************************
 *
 * @fn          NPI_LNX_IPC_SendData
 *
 * @brief       Send data from NPI to client
 *
 * input parameters
 *
 * @param          len                                                          - length of message to send
 * @param          connection                         - connection to send message (for synchronous response) otherwise -1 for all connections
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 *
 **************************************************************************************************/
int NPI_LNX_IPC_SendData(uint8 len, int connection)
{
	int bytesSent = 0, i, ret = NPI_LNX_SUCCESS;

#ifdef __DEBUG_TIME__
	if (__DEBUG_TIME_ACTIVE == TRUE)
	{
		gettimeofday(&curTime, NULL);
		long int diffPrev;
		int t = 0;
		if (curTime.tv_usec >= prevTimeSend.tv_usec)
		{
			diffPrev = curTime.tv_usec - prevTimeSend.tv_usec;
		}
		else
		{
			diffPrev = (curTime.tv_usec + 1000000) - prevTimeSend.tv_usec;
			t = 1;
		}

#ifdef __STRESS_TEST__
		if (diffPrev < (TIMING_STATS_SIZE * 1000))
			timingStats[1][diffPrev / (1000 * TIMING_STATS_MS_DIV)]++;
		else
			timingStats[1][TIMING_STATS_SIZE]++;

		// Save timingStats if inactive for > 10 seconds
		if ((curTime.tv_sec - prevTimeSend.tv_sec) > 10)
		{
			time_t rawTime;
			time(&rawTime);
			printf("\nTiming Statistics as of %s:\n", ctime(&rawTime));
			fprintf(fpStressTestData, "\nTiming Statistics as of %s:\n", ctime(&rawTime));
			for (i = 0; i < (TIMING_STATS_SIZE / TIMING_STATS_MS_DIV); i++ )
			{
				printf(" %4d: \t %8d\n", i * TIMING_STATS_MS_DIV, timingStats[1][i]);
				fprintf(fpStressTestData, " %4d: \t %8d\n", i * TIMING_STATS_MS_DIV, timingStats[1][i]);
			}
			printf(" More than %u: \t %8u\n", TIMING_STATS_SIZE, timingStats[1][TIMING_STATS_SIZE]);
			fprintf(fpStressTestData, " More than %u: \t %8u\n", TIMING_STATS_SIZE, timingStats[1][TIMING_STATS_SIZE]);

			// Then clear statistics for next set.
			memset(timingStats[1], 0, TIMING_STATS_SIZE + 1);
		}

#endif //__STRESS_TEST__

		int hours = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 3600))/3600;
		int minutes = ((curTime.tv_sec - startTime.tv_sec) - ((curTime.tv_sec - startTime.tv_sec) % 60))/60;
		//            debug_
		time_printf("[--> %.3d:%.2d:%.2d.%.6ld (+%ld.%6ld)] %.2d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData: \040 ",
				hours,											// hours
				minutes,										// minutes
				(int)(curTime.tv_sec - startTime.tv_sec) % 60,		// seconds
				(long int) curTime.tv_usec,
				curTime.tv_sec - prevTimeSend.tv_sec - t,
				diffPrev,
				((npiMsgData_t *) npi_ipc_buf[1])->len,
				((npiMsgData_t *) npi_ipc_buf[1])->subSys,
				((npiMsgData_t *) npi_ipc_buf[1])->cmdId);

		for (i = 0; i < ((npiMsgData_t *) npi_ipc_buf[1])->len; i++)
		{
			//                            debug_
			time_printf(" 0x%.2X", ((npiMsgData_t *) npi_ipc_buf[1])->pData[i]);
		}
		//            debug_
		time_printf("\n");

		prevTimeSend = curTime;
	}
#endif //__DEBUG_TIME__

	if (connection < 0)
	{
#ifdef __BIG_DEBUG__
		printf("Dispatch AREQ to all active connections: #%d", activeConnections.list[0]);
		// Send data to all connections, except listener
		for (i = 1; i < activeConnections.size; i++)
		{
			printf(", %d", activeConnections.list[i]);
		}
		printf(".\n");
#endif //__BIG_DEBUG__
		// Send data to all connections, except listener
		for (i = 0; i < activeConnections.size; i++)
		{
			if (activeConnections.list[i] != sNPIlisten)
			{
				bytesSent = send(activeConnections.list[i], npi_ipc_buf[1], len, MSG_NOSIGNAL);
				
				debug_printf("...sent %d bytes to Client #%d\n", bytesSent, activeConnections.list[i]);
				
				if (bytesSent < 0)
				{
					if (errno != ENOTSOCK)
					{
						char *errorStr = (char *)malloc(30);
						sprintf(errorStr, "send %d, %d", activeConnections.list[i], errno);
						perror(errorStr);
						// Remove from list if detected bad file descriptor
						if (errno == EBADF)
						{
							printf("Removing connection #%d\n", activeConnections.list[i]);
							close(activeConnections.list[i]);
							// Connection closed. Remove from set
							FD_CLR(activeConnections.list[i], &activeConnectionsFDs);
							ret = removeFromActiveList(activeConnections.list[i]);
						}
						else
						{
							npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_ALL;
							ret = NPI_LNX_FAILURE;
						}
					}
				}
				else if (bytesSent != len)
				{
					printf("[ERROR] Failed to send all %d bytes on socket\n", len);
				}
			}
		}
	}
	else
	{
		// Send to specific connection only
		bytesSent = send(connection, npi_ipc_buf[1], len, MSG_NOSIGNAL);

		debug_printf("...sent %d bytes to Client #%d\n", bytesSent, connection);

		if (bytesSent < 0)
		{
			perror("send");
			// Remove from list if detected bad file descriptor
			if (errno == EBADF)
			{
				printf("Removing connection #%d\n", connection);
				close(connection);
				// Connection closed. Remove from set
				FD_CLR(connection, &activeConnectionsFDs);
				ret = removeFromActiveList(connection);
				if (ret == NPI_LNX_SUCCESS)
				{
					npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_SPECIFIC_CONNECTION_REMOVED;
					ret = NPI_LNX_FAILURE;
				}
			}
			else
			{
				npi_ipc_errno = NPI_LNX_ERROR_IPC_SEND_DATA_SPECIFIC;
				ret = NPI_LNX_FAILURE;
			}
		}
	}

	return ret;
}

/**************************************************************************************************
 *
 * @fn          SerialConfigParser
 *
 * @brief       This function searches for a string a returns its value
 *
 * input parameters
 *
 * @param          configFilePath   - path to configuration file
 * @param          section          - section to search for
 * @param          key                                                         - key to return value of within section
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 *
 **************************************************************************************************/
int SerialConfigParser(FILE* serialCfgFd, const char* section,
		const char* key, char* resultString)
{
	uint8 sectionFound = FALSE, invalidLineLen = FALSE;
	char* resString = NULL;
	char* resStringToFree = NULL;
	char* psStr; // Processing string pointer
	int res = NPI_LNX_FAILURE;


	resString = malloc (128);
	if (resString == NULL)
	{
		npi_ipc_errno = NPI_LNX_ERROR_IPC_GENERIC;
		return NPI_LNX_FAILURE;
	}
	resStringToFree = resString;
	debug_printf("------------------------------------------------------\n");
	debug_printf("Serial Config Parsing:\n");
	debug_printf("- \tSection: \t%s\n", section);
	debug_printf("- \tKey: \t\t%s\n", key);

	// Do nothing if the file doesn't exist
	if (serialCfgFd != NULL)
	{
		// Make sure we start search from the beginning of the file
		fseek(serialCfgFd, 0, SEEK_SET);

		// Search through the configuration file for the wanted
		while ((resString = fgets(resString, 128, serialCfgFd)) != NULL)
		{
			// Check if we have a valid line, i.e. begins with [.
			// Note! No valid line can span more than 128 bytes. Hence we
			// must hold off parsing until we hit a newline.
			if (strlen(resString) == 128)
			{
				invalidLineLen = TRUE;
				debug_printf("Found line > 128 bytes\r");
				fflush(stdout);
			}
			else
			{
				// First time we find a valid line length after having
				// found invalid line length may be the end of the
				// invalid line. Hence, do not process this string.
				// We set the invalidLineLen parameter to FALSE after
				// the processing logic.
				if (invalidLineLen == FALSE)
				{
					// Remove the newline character (ok even if line had length 128)
					resString[strlen(resString) - 1] = '\0';

					debug_printf("Found line < 128 bytes\r");
					fflush(stdout);
					if (resString[0] == '[')
					{
						debug_printf("Found section %s\n", resString);
						// Search for wanted section
						psStr = strstr(resString, section);
						if (psStr != NULL)
						{
							resString = psStr;
							// We found our wanted section. Now search for wanted key.
							sectionFound = TRUE;
							debug_printf("Found wanted section!\n");
						}
						else
						{
							// We found another section.
							sectionFound = FALSE;
						}
					}
					else if (sectionFound == TRUE)
					{
						debug_printf("Line to process %s (strlen=%zd)\n",
								resString,
								strlen(resString));
						// We have found our section, now we search for wanted key
						// Check for commented lines, tagged with '#', and
						// lines > 0 in length
						if ((resString[0] != '#') && (strlen(resString) > 0))
						{
							// Search for wanted section
							psStr = strstr(resString, key);
							if (psStr != NULL)
							{
								debug_printf("Found key \t'%s' in \t'%s'\n", key, resString);
								// We found our key. The value is located after the '='
								// after the key.
								//                                                                                                                            printf("%s\n", psStr);
								psStr = strtok(psStr, "=");
								//                                                                                                                            printf("%s\n", psStr);
								psStr = strtok(NULL, "=;\"");
								//                                                                                                                            printf("%s\n", psStr);

								resString = psStr;
								res = NPI_LNX_SUCCESS;
								debug_printf("Found value '%s'\n", resString);
								strcpy(resultString, resString);
								debug_printf("Found value2 '%s'\n", resultString);
								// We can return this string to the calling function
								break;
							}
						}
					}
				}
				else
				{
					debug_printf("Found end of line > 128 bytes\n");
					invalidLineLen = FALSE;
				}
			}
		}
	}
	else
	{
		npi_ipc_errno = NPI_LNX_ERROR_IPC_SERIAL_CFG_FILE_DOES_NOT_EXIST;
		free(resStringToFree);
		return NPI_LNX_FAILURE;
	}

	free(resStringToFree);
	return res;
}

/**************************************************************************************************
 * @fn          NPI_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that indicates an
 *              asynchronous message has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asynchronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      STATUS
 **************************************************************************************************
 */
int NPI_AsynchMsgCback(npiMsgData_t *pMsg)
{
	int i;
	//	int ret = NPI_LNX_SUCCESS;

	debug_printf("[-->] %d bytes, subSys 0x%.2X, cmdId 0x%.2X, pData:",
			pMsg->len,
			pMsg->subSys,
			pMsg->cmdId);
	for (i = 0; i < pMsg->len; i++)
	{
		debug_printf(" 0x%.2X", pMsg->pData[i]);
	} debug_printf("\n");


#ifdef __STRESS_TEST__

	// If packet is an AREQ RTI_ReceiveDataInd use first byte in payload as sequence number
	if ((pMsg->cmdId == 0x05) && (pMsg->pData[7] == 0x03)) //RTIS_CMD_ID_RTI_REC_DATA_IND && RTI_CMD_TEST_DATA_SEQUENCED
	{
		uint32 *incomingSeqNum = (uint32 *) &pMsg->pData[8];
		if (*incomingSeqNum != (ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]] + 1))
		{
			if (*incomingSeqNum == ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]])
				ST_Parameters_t[1].recErrors.seqNumIdentical++;
			else
				ST_Parameters_t[1].recErrors.errorInSeqNum++;

			printf("\n [ERR] Sequence Number \t (==: %d, !=: %d)\n",
					ST_Parameters_t[1].recErrors.seqNumIdentical,
					ST_Parameters_t[1].recErrors.errorInSeqNum);
			fprintf(fpStressTestData, " [ERR] Sequence Number \t (==: %d, !=: %d)\n",
					ST_Parameters_t[1].recErrors.seqNumIdentical,
					ST_Parameters_t[1].recErrors.errorInSeqNum);

			printf("\tLast Sequence Number: (srcIdx: 0x%.2X) \t %d\n", pMsg->pData[0], ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]]);
			fprintf(fpStressTestData, "\tLast Sequence Number: (srcIdx: 0x%.2X) \t %d\n", pMsg->pData[0], ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]]);

			printf("\tNew \040 Sequence Number: (srcIdx: 0x%.2X) \t %d", pMsg->pData[0], *incomingSeqNum);
			fprintf(fpStressTestData, "\tNew \040 Sequence Number: (srcIdx: 0x%.2X) \t %d", pMsg->pData[0], *incomingSeqNum);

			printf("\n");
			fprintf(fpStressTestData, "\n");
		}

		ST_Parameters_t[1].currentSeqNumber[pMsg->pData[0]] = *incomingSeqNum;
	}
#endif //__STRESS_TEST__

	memcpy(npi_ipc_buf[1], (uint8*) pMsg, pMsg->len + RPC_FRAME_HDR_SZ);

	return NPI_LNX_IPC_SendData(pMsg->len + RPC_FRAME_HDR_SZ, -1);
}


/**************************************************************************************************
 * @fn          NPI_LNX_IPC_Exit
 *
 * @brief       This function will exit gracefully
 *
 *
 * input parameters
 *
 * @param       ret	-	exit condition. Return on Success, exit on Failure
 *
 * output parameters
 *
 * None.
 *
 * @return      None
 **************************************************************************************************
 */
void NPI_LNX_IPC_Exit(int ret)
{
	printf("... freeing memory (ret %d)\n", ret);

	// Close file for parsing
	if (serialCfgFd != NULL)
	{
		fclose(serialCfgFd);
		serialCfgFd = NULL;
	}

	// Free memory for configuration buffers
	if (pStrBufRoot != NULL)
	{
		free(pStrBufRoot);
		pStrBufRoot = NULL;
	}

	if (ret != NPI_LNX_SUCCESS)
	{
		// Keep path for later use
		if (devPath != NULL)
		{
			free(devPath);
			devPath = NULL;
		}
	}

	uint8 gpioIdx;
	if (ret != NPI_LNX_SUCCESS)
	{
		// Keep GPIO paths for later use
		for (gpioIdx = 0; gpioIdx < 3; gpioIdx++)
		{
			if (gpioCfg[gpioIdx] != NULL)
			{
				free(gpioCfg[gpioIdx]);
				gpioCfg[gpioIdx] = NULL;
			}
		}
		if (gpioCfg != NULL)
		{
			free(gpioCfg);
			gpioCfg = NULL;
		}
	}

	if (ret == NPI_LNX_FAILURE)
	{
		// Don't even bother open a socket; device opening failed..
		printf("Could not open device... exiting\n");

		// Write error message to /dev/npiLnxLog
		writeToNpiLnxLog("Could not open device");

		exit(npi_ipc_errno);
	}
}

/**************************************************************************************************
 * @fn          NPI_LNX_IPC_NotifyError
 *
 * @brief       This function allows other threads to notify of error conditions.
 *
 *
 * input parameters
 *
 * @param       source		- source identifier
 * @param       *errorMsg 	- A string containing the error message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None
 **************************************************************************************************
 */
int NPI_LNX_IPC_NotifyError(uint16 source, const char* errorMsg)
{
	int ret = NPI_LNX_SUCCESS;
	int sNPIconnected;
#ifndef NPI_UNIX
	struct addrinfo *resAddr;
#endif //NPI_UNIX

	const char *ipAddress = "127.0.0.1";


	/**********************************************************************
	 * Connect to the NPI server
	 **********************************************************************/

#ifdef NPI_UNIX
	int len;
	struct sockaddr_un remote;

	if ((sNPIconnected = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
	{
		perror("socket");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CREATE_SOCKET;
	}
#else
	struct addrinfo hints;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	//    ipAddress = "192.168.128.133";
	//    if ((res = getaddrinfo(NULL, ipAddress, &hints, &resAddr)) != 0)
	if (port == NULL)
	{
		// Fall back to default if port was not found in the configuration file
		printf("Warning! Port not sent to NPI_LNX_IPC_NotifyError. Will use default port: %s", NPI_PORT);
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_GET_ADDR_INFO;
	}
	else
	{
		debug_printf("[NOTIFY_ERROR] Port: %s\n", port);
		if ((ret = getaddrinfo(ipAddress, port, &hints, &resAddr)) != 0)
		{
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
			ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_GET_ADDR_INFO;
		}
		else
		{
			ret = NPI_LNX_SUCCESS;
		}
	}


	if ((sNPIconnected = socket(resAddr->ai_family, resAddr->ai_socktype, resAddr->ai_protocol)) == -1)
	{
		perror("socket");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CREATE_SOCKET;
	}
#endif

	debug_printf("[NOTIFY_ERROR] Trying to connect...\n");

#ifdef NPI_UNIX
	remote.sun_family = AF_UNIX;
	strcpy(remote.sun_path, ipAddress);
	len = strlen(remote.sun_path) + sizeof(remote.sun_family);
	if (connect(sNPIconnected, (struct sockaddr *)&remote, len) == -1)
	{
		perror("connect");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CONNECT;
	}
#else
	if (connect(sNPIconnected, resAddr->ai_addr, resAddr->ai_addrlen) == -1)
	{
		perror("connect");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_CONNECT;
	}
#endif

	if (ret == NPI_LNX_SUCCESS)
		debug_printf("[NOTIFY_ERROR] Connected.\n");


	int no = 0;
	// allow out-of-band data
	if (setsockopt(sNPIconnected, SOL_SOCKET, SO_OOBINLINE, &no, sizeof(int)) == -1)
	{
		perror("setsockopt");
		ret = NPI_LNX_ERROR_IPC_NOTIFY_ERR_SET_SOCKET_OPTIONS;
	}

	npiMsgData_t msg;

	if (strlen(errorMsg) <= AP_MAX_BUF_LEN)
	{
		memcpy(msg.pData, errorMsg, strlen(errorMsg));
	}
	else
	{
		errorMsg = "Default msg. Requested msg too long.\n";
		memcpy(msg.pData, errorMsg, strlen(errorMsg));
		debug_printf("[NOTIFY_ERROR] Size of error message too long (%zd, max %d).\n",
				strlen(errorMsg),
				AP_MAX_BUF_LEN);
	}
	// If last character is \n then remove it.
	if ((msg.pData[strlen(errorMsg) - 1]) == '\n')
	{
		msg.pData[strlen(errorMsg) - 1] = 0;
		msg.len = strlen(errorMsg);
	}
	else
	{
		msg.pData[strlen(errorMsg)] = 0;
		msg.len = strlen(errorMsg) + 1;
	}

	// For now the only required info here is command type.
	msg.subSys = RPC_CMD_NOTIFY_ERR;
	// CmdId is filled with the source identifier.
	msg.cmdId = source;

	send(sNPIconnected, &msg, msg.len + RPC_FRAME_HDR_SZ, MSG_NOSIGNAL);

	return ret;
}

static int npi_ServerCmdHandle(npiMsgData_t *pNpi_ipc_buf)
{
	int ret = NPI_LNX_SUCCESS;

	switch(pNpi_ipc_buf->cmdId)
	{
		case NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ:
			{
		#ifdef __DEBUG_TIME__
				__DEBUG_TIME_ACTIVE = pNpi_ipc_buf->pData[0];
				if (__DEBUG_TIME_ACTIVE == FALSE)
				{
					printf("__DEBUG_TIME_ACTIVE set to FALSE\n");
				}
				else
				{
					printf("__DEBUG_TIME_ACTIVE set to TRUE\n");
				}
				// Set return status
				pNpi_ipc_buf->len = 1;
				pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
		#else //__DEBUG_TIME__
				printf("NPI_Server not compiled to support time stamps\n");
				// Set return status
				pNpi_ipc_buf->len = 1;
				pNpi_ipc_buf->pData[0] = (uint8) NPI_LNX_FAILURE;
		#endif //__DEBUG_TIME__
				pNpi_ipc_buf->subSys = RPC_SYS_SRV_CTRL;
				ret = NPI_LNX_SUCCESS;
		}
		break;
		case NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ:
		{
			__BIG_DEBUG_ACTIVE = pNpi_ipc_buf->pData[0];
			if (__BIG_DEBUG_ACTIVE == FALSE)
			{
				printf("__BIG_DEBUG_ACTIVE set to FALSE\n");
			}
			else
			{
				printf("__BIG_DEBUG_ACTIVE set to TRUE\n");
			}
			// Set return status
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
			ret = NPI_LNX_SUCCESS;
			break;
		}
		case NPI_LNX_CMD_ID_VERSION_REQ:
		{
			// Set return status
			pNpi_ipc_buf->len = 4;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
			pNpi_ipc_buf->pData[1] = NPI_LNX_MAJOR_VERSION;
			pNpi_ipc_buf->pData[2] = NPI_LNX_MINOR_VERSION;
			pNpi_ipc_buf->pData[3] = NPI_LNX_REVISION;
			ret = NPI_LNX_SUCCESS;
		}
		break;
		case NPI_LNX_CMD_ID_GET_PARAM_REQ:
		{
			// Set return status
			switch(pNpi_ipc_buf->pData[0])
			{
				case NPI_LNX_PARAM_NB_CONNECTIONS:
					pNpi_ipc_buf->len = 3;
					pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
					//Number of Active Connections
					pNpi_ipc_buf->pData[1] = activeConnections.size;
					//Max number of possible connections.
					pNpi_ipc_buf->pData[2] = NPI_SERVER_CONNECTION_QUEUE_SIZE;

					ret = NPI_LNX_SUCCESS;
					break;
				case NPI_LNX_PARAM_DEVICE_USED:
					pNpi_ipc_buf->len = 2;
					pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
					//device open and used by the sever
					pNpi_ipc_buf->pData[1] = devIdx;

					ret = NPI_LNX_SUCCESS;
					break;

				default:
					npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INVALID_GET_PARAM_CMD;
					ret = NPI_LNX_FAILURE;
					break;
			}
		}
		break;
		case NPI_LNX_CMD_ID_RESET_DEVICE:
		{
			if (devIdx == NPI_SPI_FN_ARR_IDX)
			{
				// Perform Reset of the RNP
				(NPI_ResetSlaveFnArr[devIdx])();

				// Do the Hw Handshake
				(NPI_SynchSlaveFnArr[devIdx])();
			}
			else if (devIdx == NPI_I2C_FN_ARR_IDX)
			{
				// Perform Reset of the RNP
				(NPI_ResetSlaveFnArr[devIdx])();
			}
			else
			{
				ret = HalGpioResetSet(FALSE);
				debug_printf("Resetting device\n");
				usleep(1);
				ret = HalGpioResetSet(TRUE);
			}
		}
		break;
		case NPI_LNX_CMD_ID_DISCONNECT_DEVICE:
		{
			debug_printf("Trying to disconnect device %d\n", devIdx);
			(NPI_CloseDeviceFnArr[devIdx])();
			debug_printf("Preparing return message after disconnecting device %d\n", devIdx);
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = NPI_LNX_SUCCESS;
		}
		break;
		case NPI_LNX_CMD_ID_CONNECT_DEVICE:
		{
			debug_printf("Trying to connect to device %d, %s\n", devIdx, devPath);
			switch(devIdx)
			{
			case NPI_UART_FN_ARR_IDX:
#if (defined NPI_UART) && (NPI_UART == TRUE)
			{
				npiUartCfg_t uartCfg;
				char* strBuf;
				strBuf = (char*) malloc(128);
				strBuf = pStrBufRoot;
				if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "UART", "speed", strBuf)))
				{
					uartCfg.speed = atoi(strBuf);
				}
				else
				{
					uartCfg.speed=115200;
				}
				if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "UART", "flowcontrol", strBuf)))
				{
					uartCfg.flowcontrol = atoi(strBuf);
				}
				else
				{
					uartCfg.flowcontrol=0;
				}
				free(strBuf);
				ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiUartCfg_t *)&uartCfg);
			}
#endif
			break;
			case NPI_SPI_FN_ARR_IDX:
#if (defined NPI_SPI) && (NPI_SPI == TRUE)
			{
				halSpiCfg_t halSpiCfg;
				npiSpiCfg_t npiSpiCfg;
				char* strBuf;
				strBuf = (char*) malloc(128);
				if (serialCfgFd != NULL)
				{
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "speed", strBuf)))
					{
						halSpiCfg.speed = atoi(strBuf);
					}
					else
					{
						halSpiCfg.speed=500000;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "mode", strBuf)))
					{
						halSpiCfg.mode = strtol(strBuf, NULL, 16);
					}
					else
					{
						halSpiCfg.mode = 0;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "bitsPerWord", strBuf)))
					{
						halSpiCfg.bitsPerWord = strtol(strBuf, NULL, 10);
					}
					else
					{
						halSpiCfg.bitsPerWord = 0;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "useFullDuplexAPI", strBuf)))
					{
						halSpiCfg.useFullDuplexAPI = strtol(strBuf, NULL, 10);
					}
					else
					{
						halSpiCfg.useFullDuplexAPI = TRUE;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "earlyMrdyDeAssert", strBuf)))
					{
						npiSpiCfg.earlyMrdyDeAssert = strtol(strBuf, NULL, 10);
					}
					else
					{
						// If it is not defined then set value for RNP
						npiSpiCfg.earlyMrdyDeAssert = TRUE;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "detectResetFromSlowSrdyAssert", strBuf)))
					{
						npiSpiCfg.detectResetFromSlowSrdyAssert = strtol(strBuf, NULL, 10);
					}
					else
					{
						// If it is not defined then set value for RNP
						npiSpiCfg.detectResetFromSlowSrdyAssert = TRUE;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "forceRunOnReset", strBuf)))
					{
						npiSpiCfg.forceRunOnReset = strtol(strBuf, NULL, 16);
					}
					else
					{
						// If it is not defined then set value for RNP
						npiSpiCfg.forceRunOnReset = NPI_LNX_UINT8_ERROR;
					}
					if (NPI_LNX_SUCCESS == (SerialConfigParser(serialCfgFd, "SPI", "srdyMrdyHandshakeSupport", strBuf)))
					{
						npiSpiCfg.srdyMrdyHandshakeSupport = strtol(strBuf, NULL, 10);
					}
					else
					{
						// If it is not defined then set value for RNP
						npiSpiCfg.srdyMrdyHandshakeSupport = TRUE;
					}
				}
				else
				{
					halSpiCfg.speed=500000;
					halSpiCfg.mode = 0;
					halSpiCfg.bitsPerWord = 8;
					// If it is not defined then set value for RNP
					npiSpiCfg.earlyMrdyDeAssert = TRUE;
					// If it is not defined then set value for RNP
					npiSpiCfg.detectResetFromSlowSrdyAssert = TRUE;
					// If it is not defined then set value for RNP
					npiSpiCfg.forceRunOnReset = NPI_LNX_UINT8_ERROR;
					// If it is not defined then set value for RNP
					npiSpiCfg.srdyMrdyHandshakeSupport = TRUE;
				}
				// GPIO config is stored
				npiSpiCfg.gpioCfg = gpioCfg;
				npiSpiCfg.spiCfg = &halSpiCfg;
				free(strBuf);

				// Now open device for processing
				ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiSpiCfg_t *) &npiSpiCfg);

				// Must also reset and synch

				// Perform Reset of the RNP
				(NPI_ResetSlaveFnArr[devIdx])();

				// Do the Hw Handshake
				(NPI_SynchSlaveFnArr[devIdx])();

				// Since SPI does not indicate reset to host we should notify here
				// but there's no unified way of doing it for RNP and ZNP...
				// For RemoTI we can send RTI_ResetInd(). This message should just
				// be discarded by anything but RNP, so should be safe.
				((npiMsgData_t *) npi_ipc_buf[1])->len = 0;
				((npiMsgData_t *) npi_ipc_buf[1])->subSys = 0x4A;
				((npiMsgData_t *) npi_ipc_buf[1])->cmdId = 0x0D;
				NPI_LNX_IPC_SendData(((npiMsgData_t *) npi_ipc_buf[1])->len + RPC_FRAME_HDR_SZ, -1);
			}
#endif
			break;

			case NPI_I2C_FN_ARR_IDX:
#if (defined NPI_I2C) && (NPI_I2C == TRUE)
			{
				npiI2cCfg_t i2cCfg;
				i2cCfg.gpioCfg = gpioCfg;

				// Open the Device and perform a reset
				ret = (NPI_OpenDeviceFnArr[devIdx])(devPath, (npiI2cCfg_t *) &i2cCfg);
			}
#endif
			break;
			default:
				ret = NPI_LNX_FAILURE;
				break;
			}
			debug_printf("Preparing return message after connecting to device %d (ret == 0x%.2X, npi_ipc_errno == 0x%.2X)\n",
					devIdx, ret, npi_ipc_errno);
			pNpi_ipc_buf->len = 1;
			pNpi_ipc_buf->pData[0] = ret;
		}
		break;
		default:
		{
			npi_ipc_errno = NPI_LNX_ERROR_IPC_RECV_DATA_INVALID_SREQ;
			ret = NPI_LNX_FAILURE;
			break;
		}
	}
	return ret;
}

/**************************************************************************************************
 **************************************************************************************************/

