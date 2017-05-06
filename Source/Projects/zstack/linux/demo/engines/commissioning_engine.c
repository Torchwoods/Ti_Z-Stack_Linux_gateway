/*******************************************************************************
 Filename:      commissioning_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:	 Commissioning Engine handles the addition/deletion of devices in the network.


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
#include <stdint.h>
#include <time.h>

#include "data_structures.h"
#include "socket_interface.h"
#include "user_interface.h"
#include "nwkmgr.pb-c.h"
#include "commissioning_engine.h"
#include "timer_utils.h"

/*******************************************************************************
 * Locals
 ******************************************************************************/
static uint8_t requested_join_time = 0;
static tu_timer_t pj_timer = TIMER_RESET_VALUE;

/*******************************************************************************
 * Functions
 ******************************************************************************/

void comm_permit_join_timer_handler(void * arg)
{
	if ((ds_network_status.permit_remaining_time == 0) || (ds_network_status.permit_remaining_time == 255))
	{
		tu_kill_timer(&pj_timer);
	} 

	ui_refresh_display();

	if ((ds_network_status.permit_remaining_time > 0) && (ds_network_status.permit_remaining_time < 255))
	{
		ds_network_status.permit_remaining_time--;
	}
}

void comm_process_permit_join (pkt_buf_t *pkt, void *cbArg) 
{

	NwkZigbeeGenericCnf * msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("comm_process_permit_join: Received ZIGBEE_GENERIC_CNF");

	msg = nwk_zigbee_generic_cnf__unpack(NULL, pkt->header.len,	pkt->packed_protobuf_packet);   

	if (msg)
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("comm_process_permit_join: Status SUCCESS.");

			ds_network_status.permit_remaining_time = requested_join_time;

			UI_PRINT_LOG("comm_process_permit_join: Requested join time %d",
			requested_join_time);

			if ((requested_join_time > 0) && (requested_join_time < 255))
			{
				tu_set_timer(&pj_timer, 1000, true, &comm_permit_join_timer_handler, NULL);
			}
			
			comm_permit_join_timer_handler(NULL);
		}
		else
		{
			UI_PRINT_LOG("comm_process_permit_join: Error: Status FAILURE.");
		}

		nwk_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("comm_process_permit_join: Could not unpack msg");
	}
}

void comm_remove_device_confirm (pkt_buf_t *pkt, void *cbArg) 
{

	NwkZigbeeGenericCnf * msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("comm_remove_device_confirm: Received ZIGBEE_GENERIC_CNF");

	msg = nwk_zigbee_generic_cnf__unpack(NULL, pkt->header.len,	pkt->packed_protobuf_packet);   

	if (msg)
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("comm_remove_device_confirm: Status SUCCESS.");
		}

		nwk_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("comm_remove_device_confirm: Error Could not unpack msg.");
	}
}

void comm_device_binding_entry_request_confirm (pkt_buf_t *pkt, void *cbArg)
{

	NwkZigbeeGenericCnf * msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("comm_device_binding_entry_request_confirm: Received ZIGBEE_GENERIC_CNF");

	msg = nwk_zigbee_generic_cnf__unpack(NULL, pkt->header.len,	pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("comm_device_binding_entry_request_confirm: Status SUCCESS.");
		}

		nwk_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("comm_device_binding_entry_request_confirm: Error Could not unpack msg.");
	}
}


void comm_device_binding_entry_request_rsp_ind (pkt_buf_t *pkt)
{

	NwkSetBindingEntryRspInd * msg = NULL;
	//NwkAddressStructT *srcaddr = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_SET_BINDING_ENTRY_RSP_IND)
	{
		UI_PRINT_LOG("comm_device_binding_entry_request_rsp_ind wrong command id");
		return;
	}

	UI_PRINT_LOG("comm_device_binding_entry_request_rsp_ind Received NWK_MGR_CMD_ID_T__NWK_SET_BINDING_ENTRY_RSP_IND");

	msg = nwk_set_binding_entry_rsp_ind__unpack(NULL, pkt->header.len,	pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("Bind/Unbind successful");
			UI_PRINT_LOG("sourceaddress 0x%Lx endpoint 0x%x", msg->srcaddr->ieeeaddr,msg->srcaddr->endpointid);
		}
		else
		{
			UI_PRINT_LOG("Bind/Unbind not successful (status=%d)", msg->status);
		}

		nwk_set_binding_entry_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("comm_device_binding_entry_request_rsp_ind: Error Could not unpack msg.");
	}
}



void comm_send_permit_join(uint8_t joinTime)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkSetPermitJoinReq msg = NWK_SET_PERMIT_JOIN_REQ__INIT;

	msg.permitjoin = NWK_PERMIT_JOIN_TYPE_T__PERMIT_NETWORK;
	msg.permitjointime = joinTime;
	len = nwk_set_permit_join_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	UI_PRINT_LOG("comm_send_permit_join: Sending NWK_SET_PERMIT_JOIN_REQ with Join Time 0x%x", joinTime);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id= NWK_MGR_CMD_ID_T__NWK_SET_PERMIT_JOIN_REQ;

		requested_join_time = joinTime;

		nwk_set_permit_join_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, &comm_process_permit_join, NULL) !=0 )
		{
			UI_PRINT_LOG("comm_send_permit_join: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("comm_send_permit_join: Error: Could not unpack msg");
	}
}

void comm_remove_device_request(zb_addr_t * addr)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkRemoveDeviceReq msg = NWK_REMOVE_DEVICE_REQ__INIT;
	NwkAddressStructT nwkaddr = NWK_ADDRESS_STRUCT_T__INIT;

	UI_PRINT_LOG("comm_remove_device_request: Sending NWK_REMOVE_DEVICE_REQ with addr 0x%Lx endpoint 0x%x", addr->ieee_addr, addr->endpoint); 

	nwkaddr.addresstype = NWK_ADDRESS_TYPE_T__UNICAST; 
	nwkaddr.has_ieeeaddr = true; 
	nwkaddr.ieeeaddr = addr->ieee_addr; 
	nwkaddr.has_endpointid = true;
	nwkaddr.endpointid = addr->endpoint;

	msg.leavemode = NWK_LEAVE_MODE_T__LEAVE;
	msg.dstaddr = &nwkaddr;

	len = nwk_remove_device_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id = NWK_MGR_CMD_ID_T__NWK_REMOVE_DEVICE_REQ;

		nwk_remove_device_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, &comm_remove_device_confirm, NULL) !=0 )
		{
			UI_PRINT_LOG("comm_remove_device_request: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("comm_remove_device_request: Error: Could not pack msg");
	}
}

void comm_device_binding_entry_request(zb_addr_t * source_addr, zb_addr_t * dst_addr, uint32_t cluster_id, binding_mode_t binding_mode )
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkSetBindingEntryReq msg = NWK_SET_BINDING_ENTRY_REQ__INIT;
	NwkAddressStructT source_address = NWK_ADDRESS_STRUCT_T__INIT;
	NwkAddressStructT destination_address = NWK_ADDRESS_STRUCT_T__INIT;

	source_address.addresstype = NWK_ADDRESS_TYPE_T__UNICAST;
	source_address.has_ieeeaddr = true;
	source_address.ieeeaddr = source_addr->ieee_addr;
	source_address.has_endpointid = true;
	source_address.endpointid = source_addr->endpoint;

	UI_PRINT_LOG("binding  source_address 0x%Lx endpoint 0x%x", source_address.ieeeaddr, source_address.endpointid);

	destination_address.addresstype = NWK_ADDRESS_TYPE_T__UNICAST;
	destination_address.has_ieeeaddr = true;
	destination_address.ieeeaddr = dst_addr->ieee_addr;
	destination_address.has_endpointid = true;
	destination_address.endpointid = dst_addr->endpoint;

	UI_PRINT_LOG("binding  destination_address 0x%Lx endpoint 0x%x", destination_address.ieeeaddr, destination_address.endpointid);

	msg.srcaddr = &source_address;
	msg.dstaddr = &destination_address;
	msg.clusterid = cluster_id;
	msg.bindingmode = (binding_mode == BINDING_MODE_BIND) ? NWK_BINDING_MODE_T__BIND : NWK_BINDING_MODE_T__UNBIND;

	len = nwk_set_binding_entry_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id = NWK_MGR_CMD_ID_T__NWK_SET_BINDING_ENTRY_REQ;

		nwk_set_binding_entry_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, &comm_device_binding_entry_request_confirm, NULL) !=0 )
		{
			UI_PRINT_LOG("comm_device_binding_entry_request: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("comm_device_binding_entry_request: Error: Could not pack msg");
	}
}

