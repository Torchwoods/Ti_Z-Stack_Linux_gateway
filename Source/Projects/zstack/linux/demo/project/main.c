/*******************************************************************************
 Filename:       main.c
 Revised:        $Date: 2014-05-15 21:04:15 -0700 (Thu, 15 May 2014) $
 Revision:       $Revision: 38561 $

 Description:    Sample application to demonstrate the TI Home Automation ZigBee Gateway Linux solution


 Copyright 2013 Texas Instruments Incorporated. All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License").  You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless used solely and exclusively in conjunction with
 a Texas Instruments radio frequency device, which is integrated into
 your product.  Other than for the foregoing purpose, you may not use,
 reproduce, copy, prepare derivative works of, modify, distribute, perform,
 display or sell this Software and/or its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,l
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <poll.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <signal.h>
#include <execinfo.h>
#include <string.h>

#include "timer_utils.h"
#include "polling.h"
#include "socket_interface.h"
#include "user_interface.h"
#include "network_info_engine.h"
#include "device_list_engine.h"
#include "data_structures.h"
#include "ota_engine.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define CALL_STACK_TRACE_DEPTH 10

/*******************************************************************************
 * Functions
 ******************************************************************************/

void unregister_segmentation_fault_handler(void)
{
	struct sigaction action;

	action.sa_flags = 0;
	action.sa_handler = SIG_DFL;
	memset(&action.sa_mask, 0, sizeof(action.sa_mask));
	action.sa_restorer = NULL;

	sigaction(SIGSEGV, &action, NULL);
}

void segmentation_fault_handler(int signum, siginfo_t *info, void *context)
{
	void *array[CALL_STACK_TRACE_DEPTH];
	size_t size;

	fprintf(stderr, "ERROR: signal %d was trigerred:\n", signum);

	fprintf(stderr, "  Fault address: %p\n", info->si_addr);
	
	switch (info->si_code)
	{
		case SEGV_MAPERR:
			fprintf(stderr, "  Fault reason: address not mapped to object\n");
			break;
		case SEGV_ACCERR:
			fprintf(stderr, "  Fault reason: invalid permissions for mapped object\n");
			break;
		default:
			fprintf(stderr, "  Fault reason: %d (this value is unexpected).\n", info->si_code);
			break;
	}

	// get pointers for the stack entries
	size = backtrace(array, CALL_STACK_TRACE_DEPTH);

	if (size == 0)
	{
		fprintf(stderr, "Stack trace unavailable\n");
	}
	else
	{
		fprintf(stderr, "Stack trace folows%s:\n", (size < CALL_STACK_TRACE_DEPTH) ? "" : " (partial)");
		
		// print out the stack frames symbols to stderr
		backtrace_symbols_fd(array, size, STDERR_FILENO);
	}


	/* unregister this handler and let the default action execute */
	fprintf(stderr, "Executing original handler...\n");
	unregister_segmentation_fault_handler();
}

void register_segmentation_fault_handler(void)
{
	struct sigaction action;

	action.sa_flags = SA_SIGINFO;
	action.sa_sigaction = segmentation_fault_handler;
	memset(&action.sa_mask, 0, sizeof(action.sa_mask));
	action.sa_restorer = NULL;

	if (sigaction(SIGSEGV, &action, NULL) < 0)
	{
		perror("sigaction");
	}
}

/*******************************************************************************
 * Main function
 ******************************************************************************/
int main(int argc, char **argv) 
{
	char * log_filename = NULL;

	register_segmentation_fault_handler();

    /* check command line arguments */
    if ((argc < 7) || (argc > 8))
	{
       fprintf(stderr,"usage: %s <nwk_mgr_hostname> <nwk_mgr_port> <gateway_hostname> <gateway_port> <ota_hostname> <ota_port> [<log_filename>]\n", argv[0]);
       exit(0);
    }

	if (argc > 7)
	{
		log_filename = argv[7];
	}

	ds_init();
	
	if (ui_init(log_filename) != 0)
	{
		return -1;
	}

	if (si_init(argv[1], htons( atoi(argv[2])), argv[3], htons( atoi(argv[4])), argv[5], htons( atoi(argv[6]))) == 0)
	{
		while (polling_process_activity());

		si_deinit();
	}
	
	ui_deinit();

	unregister_segmentation_fault_handler();
	
    return 0;
}
