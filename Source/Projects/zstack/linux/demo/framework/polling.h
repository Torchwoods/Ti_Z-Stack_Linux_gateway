/*******************************************************************************
 Filename:      polling.h
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
#ifndef POLLING_H
#define POLLING_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>
#include <poll.h>

/*******************************************************************************
 * Types
 ******************************************************************************/
typedef void (* event_handler_cb_t)(void * arg);

typedef struct
{
	event_handler_cb_t event_handler_cb;
	void * event_handler_arg;
	bool in_use;
} poll_fds_sidestruct_t;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern struct pollfd * polling_fds;
extern int polling_fds_count;
extern bool polling_quit;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
int polling_define_poll_fd(int fd, short events, event_handler_cb_t event_handler_cb, void * event_handler_arg);
void polling_undefine_poll_fd(int fd_index);
bool polling_process_activity(void);

#endif /* POLLING_H */
