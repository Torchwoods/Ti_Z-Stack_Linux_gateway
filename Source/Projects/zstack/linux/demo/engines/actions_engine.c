/*******************************************************************************
 Filename:      actions_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:	Handle On/Off, Level, Color commands 


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

#include "data_structures.h"
#include "socket_interface.h"
#include "actions_engine.h"
#include "gateway.pb-c.h"
#include "user_interface.h"
#include "state_reflector.h"

/*******************************************************************************
 * Globals
 ******************************************************************************/
zb_addr_t g_addr ;
int16_t cluster_id = 0; 
uint16_t attribute_list[2];

/*******************************************************************************
 * Functions
 ******************************************************************************/

void act_process_set_color_cnf(pkt_buf_t * pkt, zb_addr_t * addr)
{
	GwZigbeeGenericCnf *msg = NULL;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("act_process_set_color_cnf: Received ZIGBEE_GENERIC_CNF");

	msg = gw_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg) 
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS) 
		{
			UI_PRINT_LOG("act_process_set_color_cnf:  Status SUCCESS.");


			if (addr->ieee_addr != 0)
			{
				cluster_id = ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL;
				attribute_list[0] = ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_HUE;
				attribute_list[1] = ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_SATURATION;
				sr_register_attribute_read_request(msg->sequencenumber,addr, cluster_id, attribute_list, 2);
			}
		}
		else 
		{
			UI_PRINT_LOG("act_process_set_color_cnf:  Status FAILURE.");
		}

		gw_zigbee_generic_cnf__free_unpacked(msg, NULL);	
	}
	else
	{
		UI_PRINT_LOG("act_process_set_color_cnf: Error Could not unpack msg.");
	}
}

void act_process_set_onoff_cnf(pkt_buf_t * pkt, zb_addr_t * addr)
{
	GwZigbeeGenericCnf *msg = NULL;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF) 
	{
		UI_PRINT_LOG("(pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF)");
		return;
	}

	UI_PRINT_LOG("act_process_set_onoff_cnf: Received ZIGBEE_GENERIC_CNF");


	msg = gw_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg) 
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS) 
		{
			UI_PRINT_LOG("act_process_set_onoff_cnf: Status SUCCESS.");

			if (addr->ieee_addr != 0)
			{
				UI_PRINT_LOG("act_process_set_onoff_cnf: seq_num=%d", msg->sequencenumber);

				cluster_id = ZCL_CLUSTER_ID_GEN_ON_OFF;
				attribute_list[0] = ATTRID_ON_OFF;
				sr_register_attribute_read_request(msg->sequencenumber, addr, cluster_id, attribute_list, 1);
			}
		}
		else 
		{
		UI_PRINT_LOG("act_process_set_onoff_cnf:  Status FAILURE.");
		}

		gw_zigbee_generic_cnf__free_unpacked(msg, NULL);	
	}
	else
	{
		UI_PRINT_LOG("act_process_set_onoff_cnf: Error Could not unpack msg.");
	}
}

void act_process_set_level_cnf(pkt_buf_t * pkt, zb_addr_t * addr)
{
	GwZigbeeGenericCnf *msg = NULL;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("act_process_set_level_cnf: Received ZIGBEE_GENERIC_CNF");

	msg = gw_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg) 
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS) 
		{
			UI_PRINT_LOG("act_process_set_level_cnf:  Status SUCCESS.");

			if (addr->ieee_addr != 0)
			{
				cluster_id = ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL;
				attribute_list[0] = ATTRID_LEVEL_CURRENT_LEVEL;
				sr_register_attribute_read_request(msg->sequencenumber, addr, cluster_id, attribute_list, 1);
			}
		}
		else 
		{
			UI_PRINT_LOG("act_process_set_level_cnf:  Status FAILURE.");
		}

		gw_zigbee_generic_cnf__free_unpacked(msg, NULL);	
	}
	else
	{
		UI_PRINT_LOG("act_process_set_level_cnf: Error Could not unpack msg.");
	}
}

void act_set_color(zb_addr_t * addr, uint8_t hue, uint8_t saturation)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	g_addr = *addr;
	DevSetColorReq msg = DEV_SET_COLOR_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	si_compose_address(addr, &dstaddr);

	msg.dstaddress = &dstaddr;
	msg.huevalue = hue;
	msg.saturationvalue = saturation;

	len = dev_set_color_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 

	if (pkt) 
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__DEV_SET_COLOR_REQ ;

		dev_set_color_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&act_process_set_color_cnf, &g_addr) != 0)
		{
			UI_PRINT_LOG("act_set_color: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("act_set_color: Error: Could not unpack msg.");
	}
}

void act_set_onoff(zb_addr_t * addr, uint8_t state)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	g_addr = *addr ;
	DevSetOnOffStateReq msg = DEV_SET_ON_OFF_STATE_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	si_compose_address(addr, &dstaddr);

	msg.dstaddress = &dstaddr;
	msg.state = state;

	len = dev_set_on_off_state_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);
	
	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__DEV_SET_ONOFF_STATE_REQ;

		dev_set_on_off_state_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&act_process_set_onoff_cnf, &g_addr) != 0)
		{
			UI_PRINT_LOG("act_set_onoff: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("act_set_onoff: Error: Could not unpack msg.");
	}
}

void act_set_level(zb_addr_t * addr, uint16_t transition_time, uint8_t level)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	g_addr = *addr;
	DevSetLevelReq msg = DEV_SET_LEVEL_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	si_compose_address(addr, &dstaddr);

	msg.dstaddress = &dstaddr;
	msg.transitiontime = transition_time;
	msg.levelvalue = level;

	len = dev_set_level_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 
	
	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__DEV_SET_LEVEL_REQ;

		dev_set_level_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&act_process_set_level_cnf, &g_addr) != 0)
		{
			UI_PRINT_LOG("act_set_level: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("act_set_level: Error: Could not unpack msg.");
	}
}

