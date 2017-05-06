
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
#include <sys/mman.h>

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

#define ZBSOC_SBL_STATE_IDLE 0
#define ZBSOC_SBL_STATE_HANDSHAKING 1
#define ZBSOC_SBL_STATE_PROGRAMMING 2
#define ZBSOC_SBL_STATE_VERIFYING 3
#define ZBSOC_SBL_STATE_EXECUTING 4
#define ZBSOC_SBL_STATE_SWITCH_BAUDRATE 5
#define ZBSOC_SBL_STATE_SWITCH_BAUDRATE_CONFIRMATION 6

#define ZBSOC_SBL_MAX_RETRY_ON_ERROR_REPORTED 5
#define ZBSOC_SBL_IMAGE_BLOCK_SIZE 64

#define SB_WRITE_CMD                0x01
#define SB_READ_CMD                 0x02
#define SB_ENABLE_CMD               0x03
#define SB_HANDSHAKE_CMD            0x04
#define SB_VERIFICATION_IND         0x05
#define SB_SWITCH_BAUDRATE_CMD      0x06
#define SB_ENABLE_REPORTING_CMD     0x07

//verification status
#define SB_STATE_VERIFYING            0
#define SB_VERIFICATION_ABORTED       1
#define SB_VERIFICATION_IMAGE_VALID   2
#define SB_VERIFICATION_IMAGE_INVALID 3
#define SB_VERIFICATION_FAILED        4
#define SB_STATE_BOOTLOADER_ACTIVE    5        
#define SB_STATE_WAITING              6
#define SB_STATE_EXECUTING_IMAGE      7

#define SB_RESPONSE					0x80

#define SB_SUCCESS                  0
#define SB_FAILURE                  1

#define SB_MB_WAIT_HS               2

#define SB_RPC_CMD_AREQ             0x40

#define MT_SYS_RESET_REQ            0x00
#define MT_SYS_VERSION_REQ          0x02

#define SB_FORCE_BOOT               0x10
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)

#define ZBSOC_SBL_MAX_FRAME_SIZE			71

#define BBB_RESET_MODE_PUT_IN_RESET			0x01
#define BBB_RESET_MODE_RELEASE_FROM_RESET	0x02
#define BBB_RESET_MODE_FULL_RESET			(BBB_RESET_MODE_PUT_IN_RESET | BBB_RESET_MODE_RELEASE_FROM_RESET)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

//register 
#define MUSB_DEVCT 0x47401C60

//register bit position
#define MUSB_DEVCTL_SESSION  0

const char * BOOTLOADER_RESULT_STRINGS[] = 
{
	"SBL_SUCCESS",
	"SBL_INTERNAL_ERROR",
	"SBL_BUSY",
	"SBL_OUT_OF_MEMORY",
	"SBL_PENDING",
	"SBL_ABORTED_BY_USER",
	"SBL_NO_ACTIVE_DOWNLOAD",
	"SBL_ERROR_OPENING_FILE",
	"SBL_TARGET_WRITE_FAILED",
	"SBL_EXECUTION_FAILED",
	"SBL_HANDSHAKE_FAILED",
	"SBL_LOCAL_READ_FAILED",
	"SBL_VERIFICATION_FAILED",
	"SBL_COMMUNICATION_FAILED",
	"SBL_ABORTED_BY_ANOTHER_USER",
	"SBL_REMOTE_ABORTED_BY_USER",
	"SBL_TARGET_READ_FAILED",
//	"SBL_TARGET_STILL_WORKING", not an actual return code. Also - it corresponds to the value 0xFF, so irrelevant in this table.
};

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
uint16_t zbSocSblState = ZBSOC_SBL_STATE_IDLE;

timerFDs_t * timerFDs = NULL;
static int current_input_byte_index = 0;

int finish_state = STATE_NOT_FINISHED;

size_t context = 0; //for some unknown reason, defining this variable as local inside perform_gpio_sequence_from_config_file() caused the reset sequence after bootloading to fail, even though it seems to perform exactly the same...

FILE * config_file = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t uartDebugPrintsEnabled;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void zbSocTimeoutCallback(void);
void zbSocSblReportingCallback(void);
static void processRpcSysSbl(uint8_t *rpcBuff);
void perform_gpio_sequence_from_config_file(char * section);
int write_gpio_file(char * filename, char * value);
char * config_read_pair(FILE * config_file, char * section, char ** key, char ** value, size_t * context);
void bbb_usbreset(uint8_t mode);

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

void zbSocGetTimerFds(timerFDs_t *fds)
{
	int i;

	timerFDs = fds;

	for (i = 0; i < NUM_OF_TIMERS; i++)
	{
		fds[i].fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);

		if (fds[i].fd == -1)
		{
			printf("Error creating timer\n");
			exit(-3);
		}
		else
		{
			debug_printf("Timer created OK\n");
		}
	}

	fds[TIMEOUT_TIMER].callback = zbSocTimeoutCallback;
	fds[REPORTING_TIMER].callback = zbSocSblReportingCallback;
}

uint8_t zbSocSblInitiateImageDownload(char * filename)
{
	zbSocSblImageFile = fopen(filename, "rb");
	if (zbSocSblImageFile == NULL)
	{
		printf("opening file failed with %s (errno=%d)\n", strerror( errno ), errno);
		return FAILURE;
	}
	if ((fseek(zbSocSblImageFile, 0, SEEK_END) != 0) || (image_size = ftell(zbSocSblImageFile), 	fseek(zbSocSblImageFile, 0, SEEK_SET) != 0))
	{
		printf("determining file size failed with %s (errno=%d)\n", strerror( errno ), errno);
		return FAILURE;
	}

	printf("Image size: %d bytes\n", image_size);

	printf("Press [ENTER] any time to abort downloading image.\n");

	zbSocSblState = ZBSOC_SBL_STATE_HANDSHAKING;
	timeout_retries = MAX_TIMEOUT_RETRIES;
	processRpcSysSbl(NULL);

	return SUCCESS;
}

void zbSocDisableTimeout(int timer)
{
	struct itimerspec its;

	its.it_interval.tv_sec = 0;
	its.it_interval.tv_nsec = 0;
	its.it_value.tv_sec = 0;
	its.it_value.tv_nsec = 0;

	if (timerfd_settime(timerFDs[timer].fd, 0, &its, NULL) != 0)
	{
		printf("Error disabling timer %d\n", timer);
		exit (0); //todo: assert
	}
}

void zbSocEnableTimeout(int timer, uint32_t milliseconds)
{
	struct itimerspec its;

	debug_printf("zbSocEnableTimeout #%d\n",timer);

	its.it_interval.tv_sec = 0;
	its.it_interval.tv_nsec = 0;
	its.it_value.tv_sec = ((uint64_t)milliseconds*(uint64_t)1000000) / (uint64_t)1000000000;;
	its.it_value.tv_nsec = ((uint64_t)milliseconds*(uint64_t)1000000) % (uint64_t)1000000000;

	if (timerfd_settime(timerFDs[timer].fd, 0, &its, NULL) != 0)
	{
		printf("Error setting timer %d\n", timer);
		exit (0); //todo: assert
	}
}
	
void zbSocEnableTimeoutContinious(int timer, uint32_t milliseconds)
{
	struct itimerspec its;

	debug_printf("zbSocEnableTimeout #%d\n",timer);

	its.it_interval.tv_sec = ((uint64_t)milliseconds*(uint64_t)1000000) / (uint64_t)1000000000;;
	its.it_interval.tv_nsec = ((uint64_t)milliseconds*(uint64_t)1000000) % (uint64_t)1000000000;
	its.it_value.tv_sec = its.it_interval.tv_sec;
	its.it_value.tv_nsec = its.it_interval.tv_nsec;

	if (timerfd_settime(timerFDs[timer].fd, 0, &its, NULL) != 0)
	{
		printf("Error setting timer %d\n", timer);
		exit (0); //todo: assert
	}
}

void zbSocFinishLoadingImage(uint8_t finish_code)
{
	if (zbSocSblState == ZBSOC_SBL_STATE_IDLE)
	{
		return;
	}

	if (zbSocSblImageFile != NULL)
	{
		fclose(zbSocSblImageFile);
	}

	zbSocDisableTimeout(TIMEOUT_TIMER);

	if (finish_code == SBL_ABORTED_BY_USER)
	{
		printf("\nImage download aborted by user\n");
	}
	else if (finish_code != SBL_SUCCESS)
	{
		printf("\nImage download aborted due to error (error id: %d)\n", finish_code);
	}


	if (zbSocSblState == ZBSOC_SBL_STATE_PROGRAMMING)
	{
		printf("Note:\n"
			"  Image download was interrupted while writing.\n"
			"  Most probably, the SOC has an invalid FW image now.\n"
			"  You may download a new image at any time.\n"
			"  The SOC will now internally calculate the CRC of the flash content (takes about 30 seconds).\n"
			"  if the CRC is valid, the new image will be executed.\n");
	}
	else if (zbSocSblState == ZBSOC_SBL_STATE_VERIFYING)
	{
		printf("Note:\n"
			"  Image download was interrupted while verifying the new image.\n"
			"  The SOC will now internally calculate the CRC of the flash content (takes about 30 seconds).\n"
			"  if the CRC is valid, the new image will be executed.\n");
	}

	zbSocSblState = ZBSOC_SBL_STATE_IDLE;

	if (finish_code != SBL_SUCCESS)
	{
		zbSocClose();
		printf("Performing HW reset\n");
		if (usb_target)
		{
			bbb_usbreset(BBB_RESET_MODE_PUT_IN_RESET);
		}
		else
		{
			perform_gpio_sequence_from_config_file("[HOLD_SOC_IN_RESET]");
		}

		zbSocTransportUpdateBaudrate(B115200);

		if (usb_target)
		{
			bbb_usbreset(BBB_RESET_MODE_RELEASE_FROM_RESET);
			usleep(5000000);
		}
		else
		{
			perform_gpio_sequence_from_config_file("[RELEASE_SOC_FROM_RESET]");		
			usleep(500000);
		}
		
		if( zbSocOpen( NULL ) == false )
		{
		  exit(-1);
		}
		zbSocSblEnableReporting();
	}
	else
	{
		zbSocTransportUpdateBaudrate(B115200);
	}
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

void zbSocResetLocalDevice(void)
{
	uint8_t cmd[] = {
		0xFE,
		1,   //RPC payload Len          
		SB_RPC_CMD_AREQ | MT_RPC_SYS_SYS,      
		MT_SYS_RESET_REQ,         
		0x01, //activate serial bootloader
		0x00       //FCS - fill in later
	};

	calcFcs(cmd, sizeof(cmd));
	zbSocTransportWrite(cmd,sizeof(cmd));
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

void zbSocSblExecuteImage(void)
{
	zbSocSblSendMtFrame(SB_ENABLE_CMD, NULL, 0);
	zbSocEnableTimeout(TIMEOUT_TIMER, BOOTLOADER_GENERIC_TIMEOUT);
}

void zbSocSblHandshake(void)
{
	uint8_t payload[] = {SB_MB_WAIT_HS};

	zbSocSblSendMtFrame(SB_HANDSHAKE_CMD, payload, sizeof(payload));
	zbSocEnableTimeout(TIMEOUT_TIMER, BOOTLOADER_HANDSHAKE_TIMEOUT);
}

void zbSocSblSwitchBaudrate(void)
{
	uint8_t payload[] = {216, 13};
	zbSocSblSendMtFrame(SB_SWITCH_BAUDRATE_CMD, payload, sizeof(payload));
	zbSocEnableTimeout(TIMEOUT_TIMER, BOOTLOADER_SWITCH_BAUDRATE_TIMEOUT);
}

void zbSocSblEnableReporting(void)
{
	uint8_t payload[] = {true};
	zbSocSblSendMtFrame(SB_ENABLE_REPORTING_CMD, payload, sizeof(payload));
}

void zbSocSblSendImageBlock(uint8_t buf[], uint8_t size)
{
	zbSocSblSendMtFrame(SB_WRITE_CMD, buf, size);
	zbSocEnableTimeout(TIMEOUT_TIMER, BOOTLOADER_GENERIC_TIMEOUT);
}

void zbSocSblReadImageBlock(uint32_t address)
{
	uint8_t payload[] = {(address / 4) & 0xFF, ((address / 4) >> 8) & 0xFF};
	zbSocSblSendMtFrame(SB_READ_CMD, payload, sizeof(payload));
	zbSocEnableTimeout(TIMEOUT_TIMER, BOOTLOADER_GENERIC_TIMEOUT);
}

int write_gpio_file(char * filename, char * value)
{
	FILE * f;

	f = fopen(filename, "wt");
	if (f == NULL)
	{
		printf("Error writing %s to %s", value, filename);
		return -1;
	}

	fprintf(f, "%s", value);

	fclose(f);

	return 0;
}

FILE * config_init(char * config_filename)
{
	return fopen(config_filename, "rt");
}

void config_deinit(FILE * config_file)
{
	if (config_file != NULL)
	{
		fclose(config_file);
	}
}

char * config_read_string(FILE * config_file, char * section, char * requested_key)
{
	char * key;
	char * value;

	context = 0;

	if (config_file != NULL)
	{
		while (config_read_pair(config_file, section, &key, &value, &context) != NULL)
		{
			if (strcmp(requested_key, key) == 0)
			{
				return value;
			}
		}
	}

	return NULL;
}

//if the argument string is prefixed by optional whitespaces followed by a quote, and suffixed by another quote followed by optional whitespaces, than the string is stripped from these whitespaces and quotes. Otherwise - the string is unchanged..
char * unquote(char * str)
{
	char * string_start = str;
	
	if (str[strspn(str," \t")] == '\"') //first quote found
	{
		string_start = str + strspn(str," \t") + 1;
		
		if ((strrchr(string_start, '\"') != NULL) //second quote found
			&& (strspn(strrchr(string_start, '\"'),"\" \t\r\n") == strlen(strrchr(string_start, '\"')))) //second quote postfixed by nothing than white spaces
		{
			*strrchr(string_start, '\"') = '\0';
		}
		else
		{
			string_start = str;
		}
	}

	return string_start;
}

char * config_read_pair(FILE * config_file, char * section, char ** key, char ** value, size_t * context)
{
	static char rec[300];

	if (ftell(config_file) != *context)
	{
		debug_printf("config_read_pair: seeking to: %d\n", *context);
		fseek(config_file, *context, SEEK_SET);
	}

	rec[0] = '0';

	if (*context == 0)
	{
		while ((fgets(rec, sizeof(rec), config_file) != NULL) && (strstr(rec, section) != rec))
		{
			debug_printf("config_read_pair: read: %s\n", rec);
		}
		
		if (strstr(rec, section) != rec)
		{
			debug_printf("config_read_pair: section not found\n");
			return NULL;
		}
	}
	
	while ((fgets(rec, sizeof(rec), config_file) != NULL) && (rec[0] != '['))
	{
		if (strchr(rec, '\r') != NULL)
		{
			*strchr(rec, '\r') = '\0';
		}
		if ((rec[0] != ';') && (strchr(rec, '=') != NULL))
		{
			*value = unquote(strchr(rec, '=') + 1);
			*strchr(rec, '=') = '\0';
			*key = unquote(rec);
			*context = ftell(config_file);
			return *key;
		}
		
		debug_printf("config_read_pair: skipping non-pair line\n");
	}
	
	debug_printf("config_read_pair: no pairs found\n");
	return NULL;
}

void perform_gpio_sequence_from_config_file(char * section)
{
	char * key;
	char * value;

	context = 0;

	if (config_file != NULL)
	{
		while (config_read_pair(config_file, section, &key, &value, &context) != NULL)
		{
			if (strcmp(key, "delay_us") == 0)
			{
				usleep(atoi(value));
			}
			else
			{
				//printf("%s",""); //at some point, without a printf here, the reset sequence failed...
				write_gpio_file(key, value);
			}
		}
	}
}

void hw_reset_soc(void)
{
  if (usb_target)
  {       
     bbb_usbreset(BBB_RESET_MODE_FULL_RESET);
     usleep(5000000);
  }
  else
  {
    perform_gpio_sequence_from_config_file("[HOLD_SOC_IN_RESET]");
    usleep(100000);
    perform_gpio_sequence_from_config_file("[RELEASE_SOC_FROM_RESET]");  
    usleep(5);

  }        
}

static void processRpcSysSbl(uint8_t *rpcBuff)
{
	static uint8_t buf[ZBSOC_SBL_IMAGE_BLOCK_SIZE + 2];
	
	static uint32_t zbSocCurrentImageBlockAddress;
	static uint16_t zbSocCurrentImageBlockTriesLeft;

	int bytes_read;
	uint8_t finish_code = SBL_TARGET_STILL_WORKING;
	uint8_t discard_current_frame = true;
	uint8_t load_next_block_from_file = false;

	if (rpcBuff == NULL)
	{
		if (timeout_retries-- == 0)
		{
			finish_code = SBL_COMMUNICATION_FAILED;
		}
	}
	else
	{

		if (rpcBuff[RPC_BUF_CMD1] == (SB_VERIFICATION_IND | SB_RESPONSE))
		{
			switch (rpcBuff[RPC_BUF_INCOMING_RESULT])
			{
				case SB_STATE_VERIFYING:
					printf("The bootloader FW is now calculating the CRC of the actual flash content\n");
					printf("Press [ENTER] any time to exit.\n");
					zbSocEnableTimeoutContinious(REPORTING_TIMER, 500);
					finish_state = STATE_INTERNAL_CRC_CALC;
					break;
				case SB_VERIFICATION_ABORTED:
					debug_printf("SB_VERIFICATION_ABORTED\n");
					break;
				case SB_VERIFICATION_IMAGE_VALID:
					zbSocDisableTimeout(REPORTING_TIMER);
					printf("\n");
					printf("Flash CRC verification successful.\n");
					zbSocForceRun();
					finish_state = STATE_FINISHED_OK_WAIT;
					zbSocEnableTimeoutContinious(REPORTING_TIMER, 10000);
					finish_code = SBL_SUCCESS;
					break;
				case SB_VERIFICATION_IMAGE_INVALID:
					zbSocDisableTimeout(REPORTING_TIMER);
					printf("\n");
					printf("Flash CRC verification failed\n");
					finish_state = STATE_FINISHED_WITH_ERRORS;
					break;
				case SB_VERIFICATION_FAILED:
					zbSocDisableTimeout(REPORTING_TIMER);
					printf("\n");
					printf("Failed to mark image as valid\n");
					finish_state = STATE_FINISHED_WITH_ERRORS;
					break;
				case SB_STATE_BOOTLOADER_ACTIVE:
					debug_printf("SB_STATE_BOOTLOADER_ACTIVE\n");
					break;
				case SB_STATE_WAITING:
					debug_printf("SB_STATE_WAITING\n");
					break;
				case SB_STATE_EXECUTING_IMAGE:
					debug_printf("SB_STATE_EXECUTING_IMAGE\n");
					break;
				default:
					debug_printf("Unknown verification status %d\n", rpcBuff[RPC_BUF_INCOMING_RESULT]);
					break;
			}

			return;
		}

		switch (zbSocSblState)
		{
			case ZBSOC_SBL_STATE_IDLE:
				/* do nothing */
				break;
				
			case ZBSOC_SBL_STATE_HANDSHAKING:
				if (rpcBuff[RPC_BUF_CMD1] == (SB_HANDSHAKE_CMD | SB_RESPONSE))
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						zbSocSblState = ZBSOC_SBL_STATE_SWITCH_BAUDRATE;
					}
					else
					{
						finish_code = SBL_HANDSHAKE_FAILED;
					}
				}
				break;
				
			case ZBSOC_SBL_STATE_SWITCH_BAUDRATE:
				if (rpcBuff[RPC_BUF_CMD1] == (SB_SWITCH_BAUDRATE_CMD | SB_RESPONSE))
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						zbSocTransportUpdateBaudrate(B460800);
						zbSocSblState = ZBSOC_SBL_STATE_SWITCH_BAUDRATE_CONFIRMATION;
					}
					else
					{
						finish_code = SBL_SWITCH_BAUDRATE_FAILED;
					}
				}
				break;
					
			case ZBSOC_SBL_STATE_SWITCH_BAUDRATE_CONFIRMATION:
				if (rpcBuff[RPC_BUF_CMD1] == (SB_SWITCH_BAUDRATE_CMD | SB_RESPONSE))
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						zbSocSblState = ZBSOC_SBL_STATE_PROGRAMMING;
						zbSocCurrentImageBlockAddress = 0x00000000;
						load_next_block_from_file = true;
					}
					else
					{
						finish_code = SBL_SWITCH_BAUDRATE_CONFIRMATION_FAILED;
					}
				}
				break;
							
			case ZBSOC_SBL_STATE_PROGRAMMING:
				if (rpcBuff[RPC_BUF_CMD1] == (SB_WRITE_CMD | SB_RESPONSE))
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						zbSocCurrentImageBlockAddress += ZBSOC_SBL_IMAGE_BLOCK_SIZE;
						load_next_block_from_file = true;
					}
					else if (zbSocCurrentImageBlockTriesLeft-- == 0)
					{
						finish_code = SBL_TARGET_WRITE_FAILED;
					}
				}
				break;

			case ZBSOC_SBL_STATE_VERIFYING:
				if ((rpcBuff[RPC_BUF_CMD1] == (SB_READ_CMD | SB_RESPONSE)) && (memcmp(buf, rpcBuff + 3, 2) == 0)) //process only if the address reported is the requested address. otherwise - discard frame. It was observed that soometimes when switching from writing to reading, the first read response is sent twice. 
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						if (memcmp(buf+2, rpcBuff + 5, ZBSOC_SBL_IMAGE_BLOCK_SIZE) == 0)
						{
							zbSocCurrentImageBlockAddress += ZBSOC_SBL_IMAGE_BLOCK_SIZE;
							load_next_block_from_file = true;
						}
						else
						{
							finish_code = SBL_VERIFICATION_FAILED;
						}
					}
					else if (zbSocCurrentImageBlockTriesLeft-- == 0)
					{
						finish_code = SBL_TARGET_READ_FAILED;
					}
				}
				break;

			case ZBSOC_SBL_STATE_EXECUTING:
				if (rpcBuff[RPC_BUF_CMD1] == (SB_ENABLE_CMD | SB_RESPONSE))
				{
					discard_current_frame = false;
					if (rpcBuff[RPC_BUF_INCOMING_RESULT] == SB_SUCCESS)
					{
						finish_code = SBL_SUCCESS;
					
						if (usb_target)
						{
							zbSocClose();
							
							usleep(5000000);
							
							if( zbSocOpen( NULL ) == false )
							{
							  exit(-1);
							}
							zbSocGetSystemVersion();
						}
					}
					else
					{
						finish_code = SBL_EXECUTION_FAILED;
					}
				}
				break;

			default:
				//unexpected error
				break;
		}

		if (discard_current_frame)
		{
			return;
		}
		
		timeout_retries = MAX_TIMEOUT_RETRIES;
	}

	if (finish_code != SBL_TARGET_STILL_WORKING)
	{
		zbSocFinishLoadingImage(finish_code);

		zbSocTransportUpdateBaudrate(B115200);

		if (finish_code == SBL_SUCCESS);
		{
//			usleep(5000000);
//			zbSocForceRun();
			finish_state = STATE_FINISHED_OK_WAIT;
			printf("Waiting for optional system version information. Press [ENTER] to stop waiting.\n");
			zbSocEnableTimeoutContinious(REPORTING_TIMER, 10000);
		}
		return;
	}

	debug_printf("state = %d\n", zbSocSblState);

	while (load_next_block_from_file) //executed maximum 2 times. Usually 1 time. The second time is when reading past the end of the file in programming mode, and then going to read the block at the beginning of the file
	{
		load_next_block_from_file = false;
		
		zbSocCurrentImageBlockTriesLeft = ZBSOC_SBL_MAX_RETRY_ON_ERROR_REPORTED;
		
		bytes_read = fread(buf + 2, 1, ZBSOC_SBL_IMAGE_BLOCK_SIZE, zbSocSblImageFile);
		
		if ((bytes_read < ZBSOC_SBL_IMAGE_BLOCK_SIZE) && (!feof(zbSocSblImageFile)))
		{
			finish_code = SBL_LOCAL_READ_FAILED;
		}
		else if (bytes_read == 0)
		{
			if (zbSocSblState == ZBSOC_SBL_STATE_PROGRAMMING)
			{
				zbSocSblState = ZBSOC_SBL_STATE_VERIFYING;
				zbSocCurrentImageBlockAddress = 0x00000000;
				fseek(zbSocSblImageFile, 0, SEEK_SET);
				load_next_block_from_file = true;
			}
			else
			{
				zbSocSblState = ZBSOC_SBL_STATE_EXECUTING;
			}
		}
		else
		{
			memset(buf + bytes_read, 0xFF, ZBSOC_SBL_IMAGE_BLOCK_SIZE - bytes_read); //pad the last block with 0xFF
			buf[0] = (zbSocCurrentImageBlockAddress / 4) & 0xFF; //the addresses reported in the packet are word addresses, not byte addresses, hence divided by 4
			buf[1] = ((zbSocCurrentImageBlockAddress / 4) >> 8) & 0xFF;
		}
	}

	switch (zbSocSblState)
	{
		case ZBSOC_SBL_STATE_HANDSHAKING:
			debug_printf("Handshaking:\n");

			if (!usb_target)
			{
				zbSocTransportUpdateBaudrate(B115200);
				debug_printf("  Sending reset command\n");
				zbSocResetLocalDevice(); //will only be accepted if the main application is currently active (i.e. bootloader not listening)
				usleep(500000); //todo/tbd make sure this is enough (e.g. to flush NV by current FW, if required)

				debug_printf("  Closing port\n");
				zbSocClose();

				debug_printf("  Performing HW reset\n");
				hw_reset_soc();

				debug_printf("  Opening port\n");

				if (zbSocOpen( NULL ) == false)
				{
				  exit(-1);
				}

				usleep(10000); //in case in the middle of calculating CRC
			}
			
			if (uartDebugPrintsEnabled) 
			{
				zbSocSblEnableReporting(); //optional
			}
			
			debug_printf("  Sending handshake command\n");
			zbSocSblHandshake();//will only be accepted if the bootloader is listening
			break;

		case ZBSOC_SBL_STATE_SWITCH_BAUDRATE:
			zbSocSblSwitchBaudrate();
			break;

		case ZBSOC_SBL_STATE_SWITCH_BAUDRATE_CONFIRMATION:
			// do nothing
			break;
			
		case ZBSOC_SBL_STATE_PROGRAMMING:
			if (zbSocCurrentImageBlockAddress == 0)
			{
				printf("%.*s\r", ((image_size / 0x1000) + (image_size % 0x1000 > 0 ? 1 : 0)), "-----------------------------------------------------------------------------------------------------------------------------------------------");
			}
			if ((zbSocCurrentImageBlockAddress & 0xFFF) == 0)
			{
				printf("w");
			}
			zbSocSblSendImageBlock(buf, ZBSOC_SBL_IMAGE_BLOCK_SIZE + 2);
			break;

		case ZBSOC_SBL_STATE_VERIFYING:
			if (zbSocCurrentImageBlockAddress == 0)
			{
				printf("\r");
			}
			if ((zbSocCurrentImageBlockAddress & 0xFFF) == 0)
			{
				printf("R");
			}
			zbSocSblReadImageBlock(zbSocCurrentImageBlockAddress);
			break;

		case ZBSOC_SBL_STATE_EXECUTING:
			printf("\n"
				   "Verified successfully\n"
				   "Executing image...\n");
			zbSocSblExecuteImage();
			break;
		
		default:
			//unexpected error
			break;
	}
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

		if (uartDebugPrintsEnabled)
		{
			printf("1.read %d of %d bytes: ", bytes_read, bytes_requested);
			for (i = 0; i < bytes_read; i++)
			{
				printf("%02X:", incoming_packet[current_input_byte_index + i]);
			}
			printf("\n");
		}
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
			case MT_RPC_SYS_SBL:
				processRpcSysSbl(incoming_packet + 2);		  
				break;
			case MT_RPC_SYS_SYS:
				if (incoming_packet[3] == 0x80)
				{
					printf("Received System reset indication.\n"
						"  Transport Protocol Version: %d\n"
						"  Product ID: %d\n"
						"  Software Release: %d.%d.%d\n"
						, incoming_packet[5], incoming_packet[6], incoming_packet[7], incoming_packet[8], incoming_packet[9] );
					finish_state = STATE_FINISHED_OK;
				}
				else if (incoming_packet[3] == 0x02)
				{
					printf("Received System version.\n"
						"  Transport Protocol Version: %d\n"
						"  Product ID: %d\n"
						"  Software Release: %d.%d.%d\n"
						, incoming_packet[4], incoming_packet[5], incoming_packet[6], incoming_packet[7], incoming_packet[8] );
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

void zbSocTimeoutCallback(void)
{
	
	zbSocClose();
	usleep(100000); //100 ms

	if( zbSocOpen( NULL ) == false )
	{
	  exit(-1);
	}

	printf("-- TIMEOUT --\n"); //todo / tbd / remove this print
	processRpcSysSbl(NULL);
}

void zbSocSblReportingCallback(void)
{
	uint64_t exp;

	if (read(timerFDs[REPORTING_TIMER].fd, &exp, sizeof(uint64_t)) != sizeof(uint64_t))
	{
		//unexpected
	}

	if (finish_state == STATE_FINISHED_OK_WAIT)
	{
		finish_state = STATE_FINISHED_OK;
		zbSocDisableTimeout(REPORTING_TIMER);
	}
	else
	{
		printf(".");
	}
}

void bbb_usbreset(uint8_t mode)
{
	int fd;
	int bit_position; 
	void *map_base, *virt_addr; 
	unsigned char read_result, writeval;
	off_t target;

	target = MUSB_DEVCT ;
	bit_position = MUSB_DEVCTL_SESSION;

	fd = open("/dev/mem", O_RDWR | O_SYNC);

	if (fd == -1) 
	{
		printf("ERROR: Cannot access /dev/mem. Failed to reset the USB host port. Execution aborted\n"); 
		exit (1);
	}

	// Map one page
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	
	if(map_base == (void *) -1)
	{
		printf("ERROR: Failed to map a required memory page. Execution aborted.\n"); 
		exit (1);
	}

	virt_addr = map_base + (target & MAP_MASK);

	if (mode & BBB_RESET_MODE_PUT_IN_RESET)
	{
		//Reset bit 0
		read_result = *((unsigned char *) virt_addr);
		writeval = read_result & (~(1 << bit_position));
		*((unsigned short *) virt_addr) = writeval;
	}

	if (mode == BBB_RESET_MODE_FULL_RESET)
	{
		//sleep for 1 sec
		usleep(500000);
	}
	
	if (mode & BBB_RESET_MODE_RELEASE_FROM_RESET)
	{
		//set bit 0
		read_result = *((unsigned char *) virt_addr);
		writeval = read_result | (1 << bit_position);
		*((unsigned char *) virt_addr) = writeval;
	}
	
	if(munmap(map_base, MAP_SIZE) == -1) 
	{
		printf("ERROR: Failed to unmap a memory page.\n"); 
	}
	
	close(fd);
}

