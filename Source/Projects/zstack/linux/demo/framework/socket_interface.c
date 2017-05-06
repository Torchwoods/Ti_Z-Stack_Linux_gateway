/*******************************************************************************
 Filename:      socket_interface.c
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "polling.h"
#include "tcp_client.h"
#include "socket_interface.h"
#include "user_interface.h"
#include "nwkmgr.pb-c.h"
#include "gateway.pb-c.h"
#include "otasrvr.pb-c.h"
#include "network_info_engine.h"
#include "device_list_engine.h"
#include "sensor_engine.h"
#include "timer_utils.h"
#include "state_reflector.h"
#include "attribute_engine.h"
#include "commissioning_engine.h"
#include "data_structures.h"
#include "ota_engine.h"
#include "group_scene_engine.h"
#include "system_engine.h"
#include "macros.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define INITIAL_CONFIRMATION_TIMEOUT 5000 
#define STANDARD_CONFIRMATION_TIMEOUT 1000

#define INIT_STATE_MACHINE_STARTUP_DELAY 1000

/*******************************************************************************
 * Functional Macros
 ******************************************************************************/
#define IDLE_CB (idle_cb_list == NULL ? NULL : idle_cb_list->idle_cb)
#define IDLE_CB_ARG (idle_cb_list == NULL ? NULL : idle_cb_list->idle_cb_arg)

/*******************************************************************************
 * Types
 ******************************************************************************/

typedef struct _idle_cb_element
{
	si_idle_calback_t idle_cb;
	void * idle_cb_arg;
	struct _idle_cb_element * next;
} idle_cb_element;

/*******************************************************************************
 * Variables
 ******************************************************************************/
int current_cponfirmation_timeout = INITIAL_CONFIRMATION_TIMEOUT;
server_details_t network_manager_server;
server_details_t gateway_server;
server_details_t ota_server;
tu_timer_t confirmation_wait_timer = TIMER_RESET_VALUE;
confirmation_processing_cb_t confirmation_processing_cb = NULL;
void * confirmation_processing_arg = NULL;
bool waiting_for_confirmation = false;
idle_cb_element * idle_cb_list = NULL;


/*******************************************************************************
 * Functions
 ******************************************************************************/
void confirmation_timeout_handler(void * arg)
{
	UI_PRINT_LOG(RED "TIMEOUT waiting for confirmation");
	ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Operation timed out");
	waiting_for_confirmation = false;
	confirmation_processing_cb = NULL;
	
	if (IDLE_CB != NULL)
	{
		IDLE_CB(true, IDLE_CB_ARG);
	}
}

si_idle_calback_t si_get_idle_callback(void)
{
	return IDLE_CB;
}

void si_initiate_idle_callback(void)
{
	if (idle_cb_list->next == NULL) //if the current entry is the first in the list
	{
		IDLE_CB(false, IDLE_CB_ARG);
	}
}

int si_register_idle_callback(si_idle_calback_t idle_cb, void * idle_cb_arg)
{
	idle_cb_element ** current_idle_cb_entry_ptr = &idle_cb_list;
	idle_cb_element * current_idle_cb_entry = idle_cb_list;
	uint16_t idle_cb_count = 0;

	while (current_idle_cb_entry != NULL)
	{
		current_idle_cb_entry_ptr = &current_idle_cb_entry->next;
		current_idle_cb_entry = current_idle_cb_entry->next;
		idle_cb_count++;
	}
	
	*current_idle_cb_entry_ptr = malloc(sizeof(idle_cb_element));
	current_idle_cb_entry = *current_idle_cb_entry_ptr;
	if (current_idle_cb_entry != NULL)
	{
		current_idle_cb_entry->idle_cb = idle_cb;
		current_idle_cb_entry->idle_cb_arg = idle_cb_arg;
		current_idle_cb_entry->next = NULL;
		UI_PRINT_LOG("Created new idle_cb entry (#%d) for func (%08x), arg (%08X)", idle_cb_count, (int)IDLE_CB, (int)IDLE_CB_ARG);
	}
	else
	{
		UI_PRINT_LOG("ERROR: Failed to create new idle_cb entry (#%d) for func (%08x)", idle_cb_count, (int)IDLE_CB);
		return -1;
	}

	return 0;
}

void si_unregister_idle_callback(void)
{
	idle_cb_element * temp_idle_cb_element;

	UI_PRINT_LOG("Unregistering state machine (%08x)", (int)IDLE_CB);

	if (idle_cb_list != NULL)
	{
		temp_idle_cb_element = idle_cb_list;
		idle_cb_list = idle_cb_list->next;
		free(temp_idle_cb_element);
	}

	if (IDLE_CB != NULL)
	{
		IDLE_CB(false, IDLE_CB_ARG);
	}
}

server_details_t * si_get_server_details(int server_id)
{
	server_details_t * server_details;
	
	switch (server_id)
	{
		case SI_SERVER_ID_NWK_MGR:
			server_details = &network_manager_server;
			break;
		case SI_SERVER_ID_GATEWAY:
			server_details = &gateway_server;
			break;
		case SI_SERVER_ID_OTA:
			server_details = &ota_server;
			break;
		default:
			server_details = NULL;
	}

	return server_details;
}

bool si_is_server_ready(int server_id)
{
	server_details_t * server_details = si_get_server_details(server_id);

	if (server_details != NULL)
	{
		return (server_details->connected);
	}
	else
	{
		return false;
	}
}

void si_set_confirmation_timeout_for_next_request(int server_id, int confirmation_timeout_interval)
{
	server_details_t * server_details = si_get_server_details(server_id);

	server_details->confirmation_timeout_interval = confirmation_timeout_interval;
}

int si_send_packet(pkt_buf_t * pkt, confirmation_processing_cb_t _confirmation_processing_cb, void * _confirmation_processing_arg)
{
	server_details_t * server;
	char status_string[30];
	uint16_t macro_size;

	if (record_macro)
	{
		record_macro = false;

		if (pending_macro != NULL)
		{
			UI_PRINT_LOG("Overwriting previously recorded macro");
			free(pending_macro);
		}
		macro_size = sizeof(pkt_buf_hdr_t) + pkt->header.len;
		
		pending_macro = malloc(macro_size);
		if (pending_macro != NULL)
		{
			UI_PRINT_LOG("Macro recorded successfully");
			memcpy(pending_macro, pkt, macro_size);
		}

		ui_redraw_toggles_indications();
	}

	if (pkt->header.subsystem == Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR)
	{
		server = &network_manager_server;
	}
	else if (pkt->header.subsystem == Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW)
	{
		server = &gateway_server;
	}
	else if (pkt->header.subsystem == ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR)
	{
		server = &ota_server;
	}
	else
	{
		UI_PRINT_LOG("ERROR unknown subsystem ID %d. Following packet discarded:", pkt->header.subsystem);
		ui_print_packet_to_log(pkt,"not sent: ", BOLD);
	}
		
	if (!server->connected)
	{
		ui_print_status(0, "Please wait while connecting to server");
		return -1;
	}

	if (waiting_for_confirmation)
	{
		ui_print_status(0, "BUSY - please wait for previous operation to complete");
		return -1;
	}

	if (tcp_send_packet(server, (uint8_t *)pkt, pkt->header.len + sizeof(pkt_buf_hdr_t)) == -1)
	{
		UI_PRINT_LOG("ERROR sending packet to %s\n", server->name);
		return -1;
	}
	
	STRING_START(status_string, "sent to %s: ", server->name);
	ui_print_packet_to_log(pkt, status_string, BOLD);

	confirmation_processing_cb = _confirmation_processing_cb;
	confirmation_processing_arg = _confirmation_processing_arg;
	ui_print_status(0, "BUSY");
	waiting_for_confirmation = true;
	tu_set_timer(&confirmation_wait_timer, server->confirmation_timeout_interval, false, confirmation_timeout_handler, NULL);

	if (server->confirmation_timeout_interval != STANDARD_CONFIRMATION_TIMEOUT)
	{
		server->confirmation_timeout_interval = STANDARD_CONFIRMATION_TIMEOUT;
	}
	
	return 0;
}

bool si_is_waiting_for_confirmation(void)
{
	return waiting_for_confirmation;
}

void confirmation_receive_handler(pkt_buf_t * pkt)
{
	waiting_for_confirmation = false;
	tu_kill_timer(&confirmation_wait_timer);
	ui_print_status(0, "");

	if (confirmation_processing_cb != NULL)
	{
		UI_PRINT_LOG("Calling confirmation callback");
		confirmation_processing_cb(pkt, confirmation_processing_arg);
	}
	
	if (IDLE_CB != NULL)
	{
		IDLE_CB(false, IDLE_CB_ARG);
	}
}

void ui_print_packet_to_log(pkt_buf_t * pkt, char * prefix, char * hilight)
{
	int remaining_len = sizeof(pkt_buf_hdr_t) + pkt->header.len;
	uint8_t * data = (uint8_t *)pkt;
	char log_string[180]; //TBD (size)

	STRING_START(log_string, "%s", prefix);
	STRING_ADD(log_string, "len=%d,", pkt->header.len);
	STRING_ADD(log_string, " cmd_id=%d,", pkt->header.cmd_id);
	STRING_ADD(log_string, " subsystem=%d", pkt->header.subsystem);
	UI_PRINT_LOG(log_string);
	
	STRING_START(log_string, " Raw=");
	
	while(remaining_len--)
	{
#ifdef PRINT_FULL_RAW_PACKETS	
		if (STRING_REMAINING_CHARS(log_string) < (2 + ((remaining_len > 0) ? 1 : 0)))
		{
			UI_PRINT_LOG("%s%s", hilight, log_string);
			STRING_START(log_string, "     ");
		}
#else
		if ((STRING_REMAINING_CHARS(log_string) <= 4) && (remaining_len >= 1))
		{
			STRING_ADD(log_string,	"...");
			break;
		}
		else if ((STRING_REMAINING_CHARS(log_string) <= 7) && (remaining_len >= 2))
		{
			STRING_ADD(log_string,	"%02X", *data++);
			STRING_ADD(log_string,	"...");
			break;
		}
#endif
		STRING_ADD(log_string,  "%02X", *data++);
		if (remaining_len > 0)
		{
			STRING_ADD(log_string,	":");
		}
	}
	
	UI_PRINT_LOG("%s%s", hilight, log_string);
}

void si_nwk_manager_incoming_data_handler(pkt_buf_t * pkt, int len)
{
	switch (pkt->header.cmd_id)
	{
		case NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF:
		case NWK_MGR_CMD_ID_T__NWK_ZIGBEE_SYSTEM_RESET_CNF:
		case NWK_MGR_CMD_ID_T__NWK_SET_ZIGBEE_POWER_MODE_CNF:
		case NWK_MGR_CMD_ID_T__NWK_GET_LOCAL_DEVICE_INFO_CNF:
		case NWK_MGR_CMD_ID_T__NWK_ZIGBEE_NWK_INFO_CNF:
		case NWK_MGR_CMD_ID_T__NWK_GET_NWK_KEY_CNF:
		case NWK_MGR_CMD_ID_T__NWK_GET_DEVICE_LIST_CNF:
			confirmation_receive_handler(pkt);
			break;
		case NWK_MGR_CMD_ID_T__NWK_ZIGBEE_DEVICE_IND:
			device_process_change_indication(pkt);
			break;
		case NWK_MGR_CMD_ID_T__NWK_ZIGBEE_NWK_READY_IND:
			nwk_process_ready_ind(pkt);
			break;
		case NWK_MGR_CMD_ID_T__NWK_SET_BINDING_ENTRY_RSP_IND:
			comm_device_binding_entry_request_rsp_ind(pkt);
			break;
		default:
			UI_PRINT_LOG("Unsupported incoming command id from nwk manager server (cmd_id %d)", (pkt->header.cmd_id));
			break;
	}
}

void si_gateway_incoming_data_handler(pkt_buf_t * pkt, int len)
{
	switch (pkt->header.cmd_id)
	{
		case GW_CMD_ID_T__DEV_GET_TEMP_RSP_IND:
			snsr_process_temperature_response(pkt);
			break;
		case GW_CMD_ID_T__DEV_GET_POWER_RSP_IND:
			snsr_process_power_response(pkt);
			break;
		case GW_CMD_ID_T__ZIGBEE_GENERIC_RSP_IND:
			sr_process_generic_response_indication(pkt);
			break;
		case GW_CMD_ID_T__ZIGBEE_GENERIC_CNF:
			confirmation_receive_handler(pkt);
			break;
		case GW_CMD_ID_T__GW_READ_DEVICE_ATTRIBUTE_RSP_IND:
			attr_process_read_attribute_response(pkt);
			break;
		case GW_CMD_ID_T__GW_ZCL_FRAME_RECEIVE_IND:
			system_process_zcl_frame_receive_indication(pkt);
			break;
		case GW_CMD_ID_T__GW_SET_ATTRIBUTE_REPORTING_RSP_IND:
			attr_process_set_attribute_reporting_rsp_ind(pkt);
			break;
		case GW_CMD_ID_T__GW_ATTRIBUTE_REPORTING_IND:
			attr_process_attribute_report_ind(pkt);
			break;
		case GW_CMD_ID_T__GW_GET_DEVICE_ATTRIBUTE_LIST_RSP_IND:
			attr_process_attribute_list_rsp_ind(pkt);
			break;

		default:
			UI_PRINT_LOG("Unsupported incoming command id from gateway server (cmd_id %d)", (pkt->header.cmd_id));
			break;
	}
}

void si_ota_incoming_data_handler(pkt_buf_t * pkt, int len)
{
	switch (pkt->header.cmd_id)
	{
		case OTA_MGR_CMD_ID_T__OTA_UPDATE_DL_FINISHED_IND:
			ota_process_download_finished_indication(pkt);
			break;
		case OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF:
			confirmation_receive_handler(pkt);
			break;
		case OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF:
			confirmation_receive_handler(pkt);
			break;
		default:
			UI_PRINT_LOG("Unsupported incoming command id from ota server (cmd_id %d)", (pkt->header.cmd_id));
			break;
	}
}

void si_init_state_machine(bool timed_out, void * arg)
{
	static int state = 0;

	if (!network_manager_server.connected)
	{
		state = 4;
	}
	else if ((!timed_out) || (state == 0))
	{
		state++;
	}
	
	UI_PRINT_LOG("Init state %d", state);
	
	switch (state)
	{
		case 1:
			si_register_idle_callback(si_init_state_machine, NULL);
			
			if(!waiting_for_confirmation)
			{
				nwk_send_info_request();
			}
			else
			{
				state = 0;
			}
			break;
		case 2:
			device_send_local_info_request();
			break;
		case 3:
			device_send_list_request();
			break;
		case 4:
			si_unregister_idle_callback();
			state = 0;
			break;
		default:
			break;
	}
}

void init_state_machine_startup_handler(void * arg)
{
	si_init_state_machine(false, NULL);
}

void nwk_mgr_server_connected_disconnected_handler(void)
{
	static tu_timer_t init_state_machine_timer = {0,0,0,0,0};
	
	UI_PRINT_LOG("nwk_mgr_server_connected_disconnected_handler");
	network_manager_server.confirmation_timeout_interval = INITIAL_CONFIRMATION_TIMEOUT;
	if (!network_manager_server.connected)
	{
		tu_kill_timer(&init_state_machine_timer);
		si_init_state_machine(false, NULL);
		ds_network_status.state = ZIGBEE_NETWORK_STATE_UNAVAILABLE;
		ui_redraw_network_info();
	}
	else
	{
		tu_set_timer(&init_state_machine_timer, INIT_STATE_MACHINE_STARTUP_DELAY, false, init_state_machine_startup_handler, NULL);
	}
}

void gateway_server_connected_disconnected_handler(void)
{
	UI_PRINT_LOG("gateway_server_connected_disconnected_handler");
	gateway_server.confirmation_timeout_interval = INITIAL_CONFIRMATION_TIMEOUT;
}

void ota_server_connected_disconnected_handler(void)
{
	UI_PRINT_LOG("ota_server_connected_disconnected_handler");
	ota_server.confirmation_timeout_interval = INITIAL_CONFIRMATION_TIMEOUT;
}

int si_init(char * nwk_manager_server_hostname, u_short nwk_manager_server_port, char * gateway_server_hostname, u_short gateway_server_port, char * ota_server_hostname, u_short ota_server_port)
{
	if (tcp_new_server_connection(&network_manager_server, nwk_manager_server_hostname, nwk_manager_server_port, (server_incoming_data_handler_t)si_nwk_manager_incoming_data_handler, "NWK_MGR", nwk_mgr_server_connected_disconnected_handler) == -1)
	{
		fprintf(stderr,"ERROR, wrong network manager server\n");
		return -1;
	}
	
	if (tcp_new_server_connection(&gateway_server, gateway_server_hostname, gateway_server_port, (server_incoming_data_handler_t)si_gateway_incoming_data_handler, "GATEWAY", gateway_server_connected_disconnected_handler) == -1)
	{
		tcp_disconnect_from_server(&network_manager_server);
		fprintf(stderr,"ERROR, wrong gateway server\n");
		return -1;
	}

	if (tcp_new_server_connection(&ota_server, ota_server_hostname, ota_server_port, (server_incoming_data_handler_t)si_ota_incoming_data_handler, "OTA", ota_server_connected_disconnected_handler) == -1)
	{
		tcp_disconnect_from_server(&gateway_server);
		tcp_disconnect_from_server(&network_manager_server);
		fprintf(stderr,"ERROR, wrong ota server\n");
		return -1;
	}

	return 0;
}

void si_deinit(void)
{
	//TBD
}

void si_idle_callbacl_timer_handler(void * arg)
{
	if (IDLE_CB != NULL)
	{
		IDLE_CB(false, IDLE_CB_ARG);
	}
}

void si_delay_next_idle_state_machine_state(uint64_t milliseconds)
{
	static tu_timer_t attribute_reporting_req_wait_timer = TIMER_RESET_VALUE;
	tu_set_timer(&attribute_reporting_req_wait_timer, milliseconds, false, si_idle_callbacl_timer_handler, NULL);
}

void si_compose_address(zb_addr_t * addr,	GwAddressStructT * dstaddr)
{
	if (addr->ieee_addr != 0)
	{
		dstaddr->addresstype = GW_ADDRESS_TYPE_T__UNICAST;
		dstaddr->has_ieeeaddr = true;
		dstaddr->ieeeaddr = addr->ieee_addr;
		dstaddr->has_endpointid = true;
		dstaddr->endpointid = addr->endpoint;
	}
	else if (addr->groupaddr == 0xFFFFFFFF)
	{
		dstaddr->addresstype = GW_ADDRESS_TYPE_T__BROADCAST;
		dstaddr->has_broadcastaddr = true;
		dstaddr->broadcastaddr = 0xFFFF;
		dstaddr->has_endpointid = true;
		dstaddr->endpointid = 0xFF;
	}
	else
	{
		dstaddr->addresstype = GW_ADDRESS_TYPE_T__GROUPCAST;
		dstaddr->has_groupaddr = true;
		dstaddr->groupaddr = addr->groupaddr;
		dstaddr->has_endpointid = true; //TBD: is it needed?
		dstaddr->endpointid = 0xFF;
	}
}

