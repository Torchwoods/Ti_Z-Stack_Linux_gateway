/*******************************************************************************
 Filename:       timer_utils.c
 Revised:        $Date: 2014-05-19 16:36:17 -0700 (Mon, 19 May 2014) $
 Revision:       $Revision: 38594 $

 Description:   Timer utilities


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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <sys/timerfd.h>
#include <stdio.h>
#include <unistd.h>

#include "timer_utils.h"
#include "user_interface.h"
#include "polling.h"

/******************************************************************************
 * Functions
 *****************************************************************************/
void tu_timer_handler(void * arg)
{
	tu_timer_t * timer = arg;
	uint64_t exp;
	
	if (timer->continious)
	{
		if (read(polling_fds[timer->fd_index].fd, &exp, sizeof(uint64_t)) != sizeof(uint64_t))
		{
			UI_PRINT_LOG("%p ERROR timer read. Killing timer.\n", timer);
			tu_kill_timer(timer);
		}
	}
	else
	{
		tu_kill_timer(timer);
	}
	
	timer->timer_handler_cb(timer->timer_handler_arg);
}

int tu_set_timer(tu_timer_t * timer, uint64_t milliseconds, bool continious, timer_handler_cb_t timer_handler_cb, void * timer_handler_arg)
{
	int fd;
	struct itimerspec its;

	tu_kill_timer(timer);
		
	fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
	if (fd == -1)
	{
		UI_PRINT_LOG("Error creating timer");
		return -1;
	}

	its.it_value.tv_sec = (milliseconds * 1000000) / 1000000000;
	its.it_value.tv_nsec = (milliseconds * 1000000) % 1000000000;
	its.it_interval.tv_sec = continious ? its.it_value.tv_sec : 0;
	its.it_interval.tv_nsec = continious ? its.it_value.tv_nsec : 0;

	timer->timer_handler_cb = timer_handler_cb;
	timer->timer_handler_arg = timer_handler_arg;
	timer->continious = continious;

	if ((timerfd_settime(fd, 0, &its, NULL) == 0) 
		&& ((timer->fd_index = polling_define_poll_fd(fd, POLLIN, tu_timer_handler, timer)) != -1))
	{
		timer->in_use = true;
		return 0;
	}
	
	UI_PRINT_LOG("Error setting timer\n");
	
	close(fd);
	
	return -1;
}

void tu_kill_timer(tu_timer_t * timer)
{
	struct itimerspec its;

	if (timer->in_use)
	{
		its.it_value.tv_sec = 0;
		its.it_value.tv_nsec = 0;
		its.it_interval.tv_sec = 0;
		its.it_interval.tv_nsec = 0;

		if (timerfd_settime(polling_fds[timer->fd_index].fd, 0, &its, NULL) != 0)
		{
			UI_PRINT_LOG("ERROR killing timer\n");
		}

		close(polling_fds[timer->fd_index].fd);
		
		polling_undefine_poll_fd(timer->fd_index);
		
		timer->in_use = false;
	}
}
