/*
 * zbSocCmd.h
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

#ifndef ZBSOCCMD_H
#define ZBSOCCMD_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************
 * CONSTANTS
 */
#define SBL_SUCCESS 0
#define SBL_INTERNAL_ERROR 1
#define SBL_BUSY 2
#define SBL_OUT_OF_MEMORY 3
#define SBL_PENDING 4
#define SBL_ABORTED_BY_USER 5
#define SBL_NO_ACTIVE_DOWNLOAD 6
#define SBL_ERROR_OPENING_FILE 7
#define SBL_TARGET_WRITE_FAILED 8
#define SBL_EXECUTION_FAILED 9
#define SBL_HANDSHAKE_FAILED 10
#define SBL_LOCAL_READ_FAILED 11
#define SBL_VERIFICATION_FAILED 12
#define SBL_COMMUNICATION_FAILED 13
#define SBL_ABORTED_BY_ANOTHER_USER 14
#define SBL_REMOTE_ABORTED_BY_USER 15
#define SBL_TARGET_READ_FAILED 16
#define SBL_SWITCH_BAUDRATE_FAILED 17
#define SBL_SWITCH_BAUDRATE_CONFIRMATION_FAILED 18

#define SBL_TARGET_STILL_WORKING 0xFF

#define SUCCESS 0
#define FAILURE 1

#define BOOTLOADER_HANDSHAKE_TIMEOUT 5000 //milliseconds -waiting for vdd to stabilize (for NV write voltage)
#define BOOTLOADER_GENERIC_TIMEOUT 100 //milliseconds
#define BOOTLOADER_SWITCH_BAUDRATE_TIMEOUT 1000 //milliseconds

#define NUM_OF_TIMERS 2

#define STATE_NOT_FINISHED 0
#define STATE_FINISHED_OK 1
#define STATE_FINISHED_WITH_ERRORS 2
#define STATE_ABORTED_BY_USER 3
#define STATE_FINISHED_OK_WAIT 4
#define STATE_INTERNAL_CRC_CALC 9

#define TIMEOUT_TIMER 0
#define REPORTING_TIMER 1

/*********************************************************************
 * FUNCTIONAL MACROS
 */
#define debug_printf(...) do {if (uartDebugPrintsEnabled) {printf(__VA_ARGS__);}} while (0)

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern int finish_state;
extern FILE * config_file;
extern const char * BOOTLOADER_RESULT_STRINGS[];
extern bool usb_target;

/************************************************************
 * TYPEDEFS
 */
typedef void (*timerCallback_t)(void);

typedef struct 
{
	int fd;
	timerCallback_t callback;
}timerFDs_t;

/************************************************************
 * FUNCTIONS
 */
bool zbSocOpen( char *devicePath );
void zbSocClose( void );
void zbSocProcessRpc (void);
void zbSocSblHandshake(void);
void zbSocResetLocalDevice(void);
uint8_t zbSocSblInitiateImageDownload(char * filename);
void zbSocFinishLoadingImage(uint8_t finish_code);
void zbSocDisableTimeout(int timer);
void zbSocEnableTimeout(int timer, uint32_t milliseconds);
void zbSocForceRun(void);
void zbSocSblEnableReporting(void);
FILE * config_init(char * config_filename);
void config_deinit(FILE * config_file);
void zbSocGetTimerFds(timerFDs_t *fds);
void hw_reset_soc(void);

#ifdef __cplusplus
}
#endif

#endif /* ZBSOCCMD_H */
