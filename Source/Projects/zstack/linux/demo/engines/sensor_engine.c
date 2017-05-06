/*******************************************************************************
 Filename:      sensor_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:	Handle sensor activity


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
#include <string.h>

#include "data_structures.h"
#include "socket_interface.h"
#include "actions_engine.h"
#include "gateway.pb-c.h"
#include "user_interface.h"
#include "state_reflector.h"
#include "attribute_engine.h"
#include "string.h"

/*******************************************************************************
 * Functions
 ******************************************************************************/

void snsr_process_power_response(pkt_buf_t * pkt)
{
	DevGetPowerRspInd *msg = NULL;
	int cluster_id = ZCL_CLUSTER_ID_SE_SIMPLE_METERING;
	attribute_info_t attr_info;

	if (pkt->header.cmd_id != GW_CMD_ID_T__DEV_GET_POWER_RSP_IND)
	{
		return;
	}

	msg = dev_get_power_rsp_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("snsr_process_power_response: SUCCESS.");

			attr_info.valid = true;
			attr_info.attr_id = ATTRID_SE_METERING_INSTANTANEOUS_DEMAND;
			attr_info.attr_type =  GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_INT24;
			memcpy(attr_info.attr_val, &msg->powervalue, 4);

			attr_update_attribute_in_dev_table(msg->srcaddress->ieeeaddr, 
			msg->srcaddress->endpointid, cluster_id, 1, &attr_info);

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("snsr_process_power_response: FAILURE (%d)", msg->status);
		}

		dev_get_power_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("snsr_process_power_response: Error Could not unpack msg");
	}
}

void snsr_process_temperature_response(pkt_buf_t * pkt)
{
	DevGetTempRspInd *msg = NULL;
	int cluster_id = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
	attribute_info_t attr_info;

	if (pkt->header.cmd_id != GW_CMD_ID_T__DEV_GET_TEMP_RSP_IND)
	{
		return;
	}
	
	msg = dev_get_temp_rsp_ind__unpack(NULL, pkt->header.len, 
	pkt->packed_protobuf_packet);

	if (msg )
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("snsr_process_temperature_response: Status SUCCESS.");

			attr_info.valid = true;
			attr_info.attr_id = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
			attr_info.attr_type =  GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_INT16;
			memcpy(attr_info.attr_val, &msg->temperaturevalue, 2);

			attr_update_attribute_in_dev_table(msg->srcaddress->ieeeaddr, msg->srcaddress->endpointid, cluster_id, 1, &attr_info);

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("snsr_process_temperature_response: Error Status FAILURE");
		}

		dev_get_temp_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("snsr_process_temperature_response: Error Could not unpack msg");
	}
}

void snsr_process_confirmation(pkt_buf_t * pkt, void * arg)
{

	GwZigbeeGenericCnf *msg = NULL;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	msg = gw_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		UI_PRINT_LOG("Sensor read confirmation: Status : %s", (msg->status == GW_STATUS_T__STATUS_SUCCESS) ? "SUCCESS." : "FAILURE.");

		gw_zigbee_generic_cnf__free_unpacked(msg, NULL);	
	}
}

void snsr_get_power(zb_addr_t addr)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	DevGetPowerReq msg = DEV_GET_POWER_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	UI_PRINT_LOG("snsr_get_power: started.");

	dstaddr.addresstype = GW_ADDRESS_TYPE_T__UNICAST;
	dstaddr.has_ieeeaddr = true;
	dstaddr.ieeeaddr = addr.ieee_addr;
	dstaddr.has_endpointid = true;
	dstaddr.endpointid = addr.endpoint;

	msg.dstaddress = &dstaddr;

	len = dev_get_power_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__DEV_GET_POWER_REQ;

		dev_get_power_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt,(confirmation_processing_cb_t)&snsr_process_confirmation, NULL) != 0)
		{
			UI_PRINT_LOG("snsr_get_power: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("snsr_get_power: Error: Could not pack msg.");
	}
}
 
void snsr_get_temperature(zb_addr_t addr)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	DevGetTempReq msg = DEV_GET_TEMP_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	dstaddr.addresstype = GW_ADDRESS_TYPE_T__UNICAST;
	dstaddr.has_ieeeaddr = true;
	dstaddr.ieeeaddr = addr.ieee_addr;
	dstaddr.has_endpointid = true;
	dstaddr.endpointid = addr.endpoint;

	msg.dstaddress = &dstaddr;

	len = dev_get_temp_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 
	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW; //gateway
		pkt->header.cmd_id = GW_CMD_ID_T__DEV_GET_TEMP_REQ;

		dev_get_temp_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&snsr_process_confirmation, NULL) != 0)
		{
			UI_PRINT_LOG("snsr_get_temperature: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("snsr_get_temperature: Error: Could not pack msg.");
	}
}
