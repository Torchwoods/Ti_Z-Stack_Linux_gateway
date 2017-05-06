/*******************************************************************************
 Filename:       group_scene_engine.c
 Revised:        $Date: 2014-06-03 19:06:22 -0700 (Tue, 03 Jun 2014) $
 Revision:       $Revision: 38790 $

 Description:     Handle Group and Scene related APIs


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
#include <stdint.h>
#include <stdlib.h>

#include "data_structures.h"
#include "socket_interface.h"
#include "group_scene_engine.h"
#include "nwkmgr.pb-c.h"
#include "gateway.pb-c.h"
#include "user_interface.h" 


/******************************************************************************
 * Consts
 *****************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

int convert_address(zb_addr_t * addr,	GwAddressStructT * dstaddr)
{
	if (addr->ieee_addr != 0)
	{
		dstaddr->addresstype = GW_ADDRESS_TYPE_T__UNICAST;
		dstaddr->has_ieeeaddr = true;
		dstaddr->ieeeaddr = addr->ieee_addr;
		dstaddr->has_endpointid = true;
		dstaddr->endpointid = addr->endpoint;
	}
	else if (addr->groupaddr != 0xFFFFFFFF )
	{
		dstaddr->addresstype = GW_ADDRESS_TYPE_T__GROUPCAST;
		dstaddr->has_groupaddr = true;
		dstaddr->groupaddr = addr->groupaddr;
		dstaddr->has_endpointid = true;
		dstaddr->endpointid = 0xFF; //TBD: is it needed?
	}
	else
	{
		//broadcast not supported
		return -1;
	}

	return 0;
}


void gs_generic_cnf(pkt_buf_t * pkt, zb_addr_t * addr)
{
	GwZigbeeGenericCnf *msg = NULL;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("gs_generic_cnf: Received ZIGBEE_GENERIC_CNF");

	msg = gw_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("gs_generic_cnf:  Status SUCCESS.");

		}
		else
		{
			UI_PRINT_LOG("gs_generic_cnf:  Status FAILURE.");
		}

		gw_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("gs_generic_cnf: Error Could not unpack msg.");
	}
}

void gs_add_group(zb_addr_t * addr, uint16_t groupid, char * groupname)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwAddGroupReq msg = GW_ADD_GROUP_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	if (convert_address(addr, &dstaddr) != 0)
	{
		UI_PRINT_LOG("gs_add_group Broadcast Address Not supported for this command");
		return ;
	}

	msg.dstaddress = &dstaddr;
	msg.groupid = groupid ;
	msg.groupname = groupname ;

	len = gw_add_group_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_ADD_GROUP_REQ;

		gw_add_group_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&gs_generic_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("gs_add_group: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("gs_add_group: Error: Could not unpack msg.");
	}
}

void gs_remove_from_group(zb_addr_t * addr, uint16_t groupid)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwRemoveFromGroupReq msg = GW_REMOVE_FROM_GROUP_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	if (convert_address(addr, &dstaddr) != 0)
	{
		UI_PRINT_LOG("gs_remove_from_group Broadcast Address Not supported for this command");
		return ;
	}

	msg.dstaddress = &dstaddr;

	msg.has_groupid = true;
	msg.groupid = groupid ;

	len = gw_remove_from_group_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_REMOVE_FROM_GROUP_REQ;
		gw_remove_from_group_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&gs_generic_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("gs_remove_from_group: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("gs_remove_from_group: Error: Could not unpack msg.");
	}
}

void gs_store_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwStoreSceneReq msg = GW_STORE_SCENE_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	if (convert_address(addr, &dstaddr) != 0)
	{
		UI_PRINT_LOG("gs_store_scene Broadcast Address Not supported for this command");
		return ;
	}

	msg.dstaddress = &dstaddr;
	msg.groupid = groupid ;
	msg.sceneid = sceneid ;

	len = gw_store_scene_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_STORE_SCENE_REQ;

		gw_store_scene_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&gs_generic_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("gs_store_scene: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("gs_store_scene: Error: Could not unpack msg.");
	}
}

void gs_remove_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwRemoveSceneReq msg = GW_REMOVE_SCENE_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	if (convert_address(addr, &dstaddr) != 0)
	{
		UI_PRINT_LOG("gs_remove_scene Broadcast Address Not supported for this command");
		return ;
	}

	msg.dstaddress = &dstaddr;
	msg.groupid = groupid ;
	msg.sceneid = sceneid ;

	len = gw_remove_scene_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_REMOVE_SCENE_REQ;

		gw_remove_scene_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&gs_generic_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("gs_remove_scene: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("gs_remove_scene: Error: Could not unpack msg.");
	}
}

void gs_recall_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwRecallSceneReq msg = GW_RECALL_SCENE_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	if (convert_address(addr, &dstaddr) != 0)
	{
		UI_PRINT_LOG("gs_recall_scene Broadcast Address Not supported for this command");
		return ;
	}

	msg.dstaddress = &dstaddr;
	msg.groupid = groupid ;
	msg.sceneid = sceneid ;

	len = gw_recall_scene_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_RECALL_SCENE_REQ;

		gw_recall_scene_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&gs_generic_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("gs_recall_scene: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("gs_recall_scene: Error: Could not unpack msg.");
	}
}

