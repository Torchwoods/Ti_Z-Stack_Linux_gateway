/*******************************************************************************
 Filename:      socket_interface.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:	Socket interface


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
#ifndef SOCKET_INTERFACE_H
#define SOCKET_INTERFACE_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "types.h"
#include "gateway.pb-c.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define SI_SERVER_ID_NWK_MGR 0
#define SI_SERVER_ID_GATEWAY 1
#define SI_SERVER_ID_OTA     2

/*******************************************************************************
 * types
 ******************************************************************************/
typedef void (* confirmation_processing_cb_t)(pkt_buf_t * pkt, void * arg);
typedef void (* si_idle_calback_t)(bool timed_out, void * arg);

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
int si_init(char * nwk_manager_server_hostname, u_short nwk_manager_server_port, char * gateway_server_hostname, u_short gateway_server_port, char * ota_server_hostname, u_short ota_server_port);
int si_send_packet(pkt_buf_t * pkt, confirmation_processing_cb_t confirmation_processing_cb, void * _confirmation_processing_arg);
void si_deinit(void);
int si_register_idle_callback(si_idle_calback_t idle_cb, void * idle_cb_arg);
void si_initiate_idle_callback(void);
void si_unregister_idle_callback(void);
uint8_t si_is_gateway_server_ready(void);
si_idle_calback_t si_get_idle_callback(void);
bool si_is_waiting_for_confirmation(void);
bool si_is_server_ready(int server_id);
void si_set_confirmation_timeout_for_next_request(int server_id, int confirmation_timeout_interval);
void si_delay_next_idle_state_machine_state(uint64_t milliseconds);
void si_compose_address(zb_addr_t * addr,	GwAddressStructT * dstaddr);

#endif /* SOCKET_INTERFACE_H */
