/**************************************************************************************************
 * Filename:       main.c
 * Description:    This file contains the main function of the Serial BootLoader tool.
 *
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <termios.h>

#include "zbSocCmd.h"
#include "zbSocTransportUart.h"

/*********************************************************************
 * CONSTANTS
 */
#define VER_MAJOR 0
#define VER_MINOR 83
#define VER_WORD ""


/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8_t uartDebugPrintsEnabled = 0;

bool usb_target = false;

struct termios old_tio;


/*********************************************************************
 * FUNCTIONS
 */
int main(int argc, char* argv[])
{
	char * selected_serial_port;
	int numTimerFDs = NUM_OF_TIMERS;
	timerFDs_t *timer_fds = malloc(  NUM_OF_TIMERS * sizeof( timerFDs_t ) );
	char ch;
	struct termios new_tio;

	setvbuf(stdout, NULL, _IONBF, 0); //apparently this one does not need to be undone at deinit.

	printf("\n"
		"***** TI LPRF ZigBee Serial Bootloader Tool for Linux v. %d.%d%s *****\n"
		"Executed %s on %s %s\n", VER_MAJOR, VER_MINOR, VER_WORD, argv[0], __DATE__, __TIME__ );

	if (argc < 2)
	{
		printf("Error, wrong number of argument. specify filename and optionaly the serial port to use\n");
		printf("When running on beaglebone platform, please specify \"USB\" (without the quotes) instead of a serial port\n");
		exit(-1);
	}

	printf("Requested file file: %s\n", argv[1]);

	if( argc == 2 )
	{
		selected_serial_port = "/dev/ttyO4";
		printf("Serial port not specified. attempting to use /dev/ttyO4\n");
	}
	else
	{
		if (strcasecmp(argv[2], "usb") == 0)
		{
			usb_target = true;
			selected_serial_port = "/dev/ttyACM0";
			printf("Using USB transport\n");
		}
		else
		{
			selected_serial_port = argv[2];
			printf("Requested serial port: %s\n", selected_serial_port);
		}
	}

	if (argc > 3)
	{
		uartDebugPrintsEnabled = atoi(argv[3]);
		printf("Debug traces %s\n", uartDebugPrintsEnabled ? "enabled" : "disabled");
	}

	if (usb_target)
	{
		printf("Resetting USB connection. It takes a few seconds...\n");
		hw_reset_soc();
	}
	else
	{
		config_file = config_init("sbl_tool.cfg");
	}

	zbSocOpen( selected_serial_port );

	if (uartDebugPrintsEnabled) 
	{
		zbSocSblEnableReporting();
	}

	if( serialPortFd == -1 )
	{
		exit(-1);
	}

	zbSocGetTimerFds(timer_fds);


	tcgetattr(STDIN_FILENO,&old_tio);
	new_tio=old_tio;
	new_tio.c_lflag &=(~ICANON & ~ECHO);
	tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

	if (zbSocSblInitiateImageDownload(argv[1]) == SUCCESS)
	{
		while((finish_state == STATE_NOT_FINISHED) || (finish_state == STATE_INTERNAL_CRC_CALC) || (finish_state == STATE_FINISHED_OK_WAIT))
		{          
			int timerFdIdx;
			struct pollfd *pollFds = malloc(  ((1 + numTimerFDs + 1) * sizeof( struct pollfd )) );

			if(pollFds)	
			{
				pollFds[0].fd = serialPortFd;
				pollFds[0].events = POLLIN;

				for(timerFdIdx=0; timerFdIdx < numTimerFDs; timerFdIdx++)
				{
					pollFds[1+timerFdIdx].fd = timer_fds[timerFdIdx].fd;
					pollFds[1+timerFdIdx].events =POLLIN;
				} 

				pollFds[1+timerFdIdx].fd = 0;  //STDIN 
				pollFds[1+timerFdIdx].events = POLLIN;

				debug_printf("%s: waiting for poll()\n", argv[0]);

				poll(pollFds, (1+numTimerFDs + 1), -1);

				if(pollFds[0].revents)
				{
					debug_printf("Message from the ZigBee SoC\n");
					zbSocProcessRpc();
				}

				for(timerFdIdx=0; timerFdIdx < numTimerFDs; timerFdIdx++)
				{
					if (pollFds[1+timerFdIdx].revents & POLLIN)
					{
						debug_printf("Timer expired: #%d\n", timerFdIdx);
						timer_fds[timerFdIdx].callback();
					}
				}

				if(pollFds[1+timerFdIdx].revents & POLLIN)
				{
					debug_printf("Got a keypress from user\n");

					if (read(STDIN_FILENO, &ch, 1) > 0)
					{
						if (ch == 10)
						{
							if (finish_state == STATE_INTERNAL_CRC_CALC)
							{
								zbSocDisableTimeout(REPORTING_TIMER);
								printf("\n");
								finish_state = STATE_ABORTED_BY_USER;
							}
							else if (finish_state == STATE_FINISHED_OK_WAIT)
							{
								zbSocDisableTimeout(REPORTING_TIMER);
								finish_state = STATE_FINISHED_OK;
							}
							else
							{
								zbSocFinishLoadingImage(SBL_ABORTED_BY_USER);
							}
						}
					}
				}

				free( pollFds );	  		
			}
		}    

		usleep(100000);
	}

	config_deinit(config_file);

	printf("Done %s\n"
		"\n", finish_state == STATE_FINISHED_OK  ? "successfully." : finish_state == STATE_ABORTED_BY_USER ? "(aborted by user).": "with errors.");

	old_tio.c_lflag |=(ICANON | ECHO);
	tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);

	return finish_state;
}

