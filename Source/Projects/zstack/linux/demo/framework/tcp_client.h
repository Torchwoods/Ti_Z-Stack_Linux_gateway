/*******************************************************************************
 Filename:       tcp_client.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:   Client communication via TCP ports


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
#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <sys/socket.h>
#include <netinet/in.h>

#include "timer_utils.h"

/******************************************************************************
 * Types
 *****************************************************************************/

typedef void (* server_incoming_data_handler_t)(void * buf, int len);
typedef void (* server_connected_disconnected_handler_t)(void);

typedef struct
{
	struct sockaddr_in serveraddr;
	server_incoming_data_handler_t server_incoming_data_handler;
	int fd_index;
	tu_timer_t server_reconnection_timer;
	char * name;
	server_connected_disconnected_handler_t server_connected_disconnected_handler;
	bool connected;
	int confirmation_timeout_interval;
} server_details_t;

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
int tcp_new_server_connection(server_details_t * server_details, char * hostname, u_short port, server_incoming_data_handler_t server_incoming_data_handler, char * name, server_connected_disconnected_handler_t server_connected_disconnected_handler);
int tcp_disconnect_from_server(server_details_t * server);
int tcp_send_packet(server_details_t * server_details, uint8_t * buf, int len);

#endif /* TCP_CLIENT_H */
