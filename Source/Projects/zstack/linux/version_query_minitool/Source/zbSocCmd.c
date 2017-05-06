
/*
 * zbSocCmd.c
 *
 * This module contains the API for the ZigBee SoC Host Interface.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*********************************************************************
 * INCLUDES
 */
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <sys/timerfd.h>
#include <stdbool.h>

#include "zbSocCmd.h"
#include "zbSocTransportUart.h"

/*********************************************************************
 * CONSTANTS
 */
#define MAX_TIMEOUT_RETRIES 10

/* The 5 LSB's of the 1st command field byte are for the subsystem. */
#define MT_RPC_SUBSYSTEM_MASK 0x1F

#define MT_RPC_SOF         0xFE

#define RPC_BUF_CMD0 0
#define RPC_BUF_CMD1 1
#define RPC_BUF_INCOMING_RESULT 2

#define SB_RPC_CMD_AREQ             0x40

#define MT_SYS_RESET_REQ            0x00
#define MT_SYS_VERSION_REQ          0x02
#define MT_SYS_OSAL_NV_WRITE                 0x09

#define MT_SAPI_WRITE_CFG_REQ               0x05

#define ZNP_NV_RF_TEST_PARMS    0x0F07



#define SB_FORCE_BOOT               0x10
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)

#define ZBSOC_SBL_MAX_FRAME_SIZE			71

/************************************************************
 * TYPEDEFS
 */
typedef enum {
  MT_RPC_SYS_RES0,   /* Reserved. */
  MT_RPC_SYS_SYS,
  MT_RPC_SYS_MAC,
  MT_RPC_SYS_NWK,
  MT_RPC_SYS_AF,
  MT_RPC_SYS_ZDO,
  MT_RPC_SYS_SAPI,   /* Simple API. */
  MT_RPC_SYS_UTIL,
  MT_RPC_SYS_DBG,
  MT_RPC_SYS_APP,
  MT_RPC_SYS_OTA,
  MT_RPC_SYS_ZNP,
  MT_RPC_SYS_SPARE_12,
  MT_RPC_SYS_SBL = 13,  // 13 to be compatible with existing RemoTI. // AKA MT_RPC_SYS_UBL
  MT_RPC_SYS_MAX        // Maximum value, must be last (so 14-32 available, not yet assigned).
} mtRpcSysType_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
FILE * zbSocSblImageFile;
uint32_t image_size = 0;
	
int timeout_retries;

timerFDs_t * timerFDs = NULL;
static int current_input_byte_index = 0;

int finish_state = STATE_NOT_FINISHED;

uint32_t software_revision = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t uartDebugPrintsEnabled;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void perform_gpio_sequence_from_config_file(char * section);
int write_gpio_file(char * filename, char * value);
char * config_read_pair(FILE * config_file, char * section, char ** key, char ** value, size_t * context);

static void calcFcs(uint8_t *msg, int size)
{
	uint8_t result = 0;
	int idx = 1; //skip SOF
	int len = (size - 2);  // skip FCS

	while ((len--) != 0)
	{
		result ^= msg[idx++];
	}

	msg[(size-1)] = result;
}

bool zbSocOpen( char *_devicePath  )
{
	if (zbSocTransportOpen(_devicePath) == false) 
	{
		perror(_devicePath); 
		printf("%s open failed\n",_devicePath);
		return false;
	}

	current_input_byte_index = 0;

	return true;
}

void zbSocForceRun(void)
{
	uint8_t forceBoot = SB_FORCE_RUN;

	//Send the bootloader force boot incase we have a bootloader that waits
	zbSocTransportWrite(&forceBoot, 1);
}

void zbSocClose( void )
{
	zbSocTransportClose();

	return;
}

void zbSocSblSendMtFrame(uint8_t cmd, uint8_t * payload, uint8_t payload_len)
{
	uint8_t buf[ZBSOC_SBL_MAX_FRAME_SIZE];

	buf[0] = 0xFE;
	buf[1] = payload_len;
	buf[2] = SB_RPC_CMD_AREQ | MT_RPC_SYS_SBL;
	buf[3] = cmd; 

	if (payload_len > 0)
	{
		memcpy(buf + 4, payload, payload_len);
	}
	calcFcs(buf, payload_len + 5);
	zbSocTransportWrite(buf, payload_len + 5);
}

void zbSocGetSystemVersion(void)
{
	uint8_t cmd[] = {
		0xFE,
		0,   //RPC payload Len          
		SB_RPC_CMD_AREQ | MT_RPC_SYS_SYS,      
		MT_SYS_VERSION_REQ,         
		0x00       //FCS - fill in later
	};

	calcFcs(cmd, sizeof(cmd));
	zbSocTransportWrite(cmd,sizeof(cmd));
}

void zbSocProcessRpc (void)
{
	int x;
	uint8_t len;
	static uint8_t incoming_packet[300];
	uint8_t bytes_requested;
	int bytes_read;
	int i;
	uint8_t original_fcs;
	int resource_temporarily_unavailable_counter = 0;

	while (current_input_byte_index < 2)
	{
		bytes_requested =  2 - current_input_byte_index;
		bytes_read = zbSocTransportRead(&incoming_packet[current_input_byte_index], bytes_requested);

		if (bytes_read == -1)
		{
			if (errno == 11)
			{
				resource_temporarily_unavailable_counter++;
			}

			if ((errno != 11) || (resource_temporarily_unavailable_counter > 20))
			{
				printf("zbSocProcessRpc: read failed - %s (%d)\n", strerror(errno), errno);
			}
			return;
		}
		
		resource_temporarily_unavailable_counter = 0;

		current_input_byte_index += bytes_read;

		if (current_input_byte_index < 2)
		{
			return;
		}

		if (incoming_packet[0] != MT_RPC_SOF)
		{
			if (incoming_packet[1] != MT_RPC_SOF)
			{
				debug_printf("UART IN  <-- discarding: %02X:%02X\n", incoming_packet[0], incoming_packet[1]);
				current_input_byte_index = 0;
			}
			else
			{
				debug_printf("UART IN  <-- discarding: %02X\n", incoming_packet[0]);
				incoming_packet[0] = MT_RPC_SOF;
				current_input_byte_index = 1;
			}
		}
	}

	len = incoming_packet[1];
	bytes_requested = len + 3 - (current_input_byte_index - 2);
	bytes_read = zbSocTransportRead(&incoming_packet[current_input_byte_index], bytes_requested);
	if (uartDebugPrintsEnabled)
	{
		printf("2.read %d of %d bytes: ", bytes_read, bytes_requested);
		for (i = 0; i < bytes_read; i++)
		{
			printf("%02X:", incoming_packet[current_input_byte_index + i]);
		}
		printf("\n");
	}
	
	if (bytes_read == -1)
	{
		printf("zbSocProcessRpc: read failed - %s (%d)\n", strerror(errno), errno);
		return;
	}

	current_input_byte_index += bytes_read;

	if (bytes_read < bytes_requested)
	{
		return;
	}

	if (uartDebugPrintsEnabled)
	{
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);

		printf("UART IN  %04d-%02d-%02d %02d:%02d:%02d<-- %d Bytes: SOF:%02X, Len:%02X, CMD0:%02X, CMD1:%02X, Payload:", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, len+5, MT_RPC_SOF, len, incoming_packet[2], incoming_packet[3]);
		for (x = 0; x < len; x++)
		{
			printf("%02X%s", incoming_packet[x + 4], x < len - 1 ? ":" : ",");
		}
		printf(" FCS:%02X\n", incoming_packet[len + 4]);
	}

	original_fcs = incoming_packet[len + 4];
	calcFcs(incoming_packet, len + 5);

	if (original_fcs != incoming_packet[len + 4])
	{
		printf("UART IN  <-- Bad FCS. Discarding packet\n");
	}
	else
	{
		switch (incoming_packet[2] & MT_RPC_SUBSYSTEM_MASK) 
		{
			case MT_RPC_SYS_SYS:
				if (incoming_packet[3] == 0x02)
				{
					printf("Received System version.\n"
						"  Transport Protocol Version: %d\n"
						"  Product ID: %d\n"
						"  Software Release: %d.%d.%d\n"
						, incoming_packet[4], incoming_packet[5], incoming_packet[6], incoming_packet[7], incoming_packet[8]);

					if (len > 5)
					{
						software_revision = incoming_packet[9] + (incoming_packet[10] << 8) + (incoming_packet[11] << 16) + (incoming_packet[12] << 24);
					}
					
					printf("  Software Revision: %u\n", software_revision);

					if (len <= 5)
					{
						printf("  (Revision not specified)\n");
					}

					finish_state = STATE_FINISHED_OK;
				}
				else
				{
					printf("zbSocProcessRpc: CMD0:%x, CMD1:%x, not handled\n", incoming_packet[2], incoming_packet[3] );
				}
			break;
			default:
				printf("zbSocProcessRpc: CMD0:%x, CMD1:%x, not handled\n", incoming_packet[2], incoming_packet[3] );
				break;
		}
	}
	
	current_input_byte_index = 0;
}

