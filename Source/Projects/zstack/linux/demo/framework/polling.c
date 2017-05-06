/*******************************************************************************
 Filename:      polling.c
 Revised:        $Date: 2014-05-19 16:36:17 -0700 (Mon, 19 May 2014) $
 Revision:       $Revision: 38594 $

 Description:	Provide generic mechanism to define and poll file descriptors


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
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <stdio.h>

#include "polling.h"
#include "user_interface.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define POLL_FDS_TO_ADD_EACH_TIME 3

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
struct pollfd * polling_fds = NULL;
int polling_fds_count = 0;
bool polling_quit = false;

/*******************************************************************************
 * Internal Variables
 ******************************************************************************/
poll_fds_sidestruct_t * poll_fds_sidestruct = NULL;

/*******************************************************************************
 * Functions
 ******************************************************************************/

int add_poll_fds(int additional_count)
{
	struct pollfd * temp_fds;
	poll_fds_sidestruct_t * temp_poll_fds_sidestruct;
	int i;

	temp_fds = malloc(sizeof(struct pollfd) * (polling_fds_count + additional_count));
	if (temp_fds == NULL)
	{
		return -1;
	}
	
	temp_poll_fds_sidestruct = malloc(sizeof(poll_fds_sidestruct_t) * (polling_fds_count + additional_count));
	if (temp_poll_fds_sidestruct == NULL)
	{
		free(temp_fds);
		return -1;
	}

	memcpy(temp_fds, polling_fds, sizeof(struct pollfd) * polling_fds_count);
	memcpy(temp_poll_fds_sidestruct, poll_fds_sidestruct, sizeof(poll_fds_sidestruct_t) * polling_fds_count);

	for (i = polling_fds_count; i < (polling_fds_count + additional_count); i++)
	{
		temp_fds[i].fd = -1;
		temp_poll_fds_sidestruct[i].in_use = false;
	}

	free(polling_fds);
	free(poll_fds_sidestruct);

	polling_fds = temp_fds;
	poll_fds_sidestruct = temp_poll_fds_sidestruct;
	polling_fds_count += additional_count;

	return 0;
}

int polling_define_poll_fd(int fd, short events, event_handler_cb_t event_handler_cb, void * event_handler_arg)
{
	int i;

	for (i = 0; i < polling_fds_count; i++)
	{
		if (poll_fds_sidestruct[i].in_use == false)
		{
			break;
		}
	}
	
	if (i == polling_fds_count)
	{
		if (add_poll_fds(POLL_FDS_TO_ADD_EACH_TIME) != 0)
		{
			return -1;
		}
	}

	poll_fds_sidestruct[i].in_use = true;
	poll_fds_sidestruct[i].event_handler_cb = event_handler_cb;
	poll_fds_sidestruct[i].event_handler_arg = event_handler_arg;
	
	polling_fds[i].fd = fd;
	polling_fds[i].events = events;
		
	return i;
}

void polling_undefine_poll_fd(int fd_index)
{
	poll_fds_sidestruct[fd_index].in_use = false;
	polling_fds[fd_index].fd = -1;
}

bool polling_process_activity(void)
{
	int i;
	
	if (poll(polling_fds, polling_fds_count, -1) > 0)
	{
		for (i = 0; i < polling_fds_count; i++)
		{
			if (poll_fds_sidestruct[i].in_use && (polling_fds[i].revents & polling_fds[i].events))
			{
				poll_fds_sidestruct[i].event_handler_cb(poll_fds_sidestruct[i].event_handler_arg);
			}
		}
	}
	else
	{
		UI_PRINT_LOG("ERROR with poll()\n");
		polling_quit = true;
	}
	
	return (!polling_quit);
}
