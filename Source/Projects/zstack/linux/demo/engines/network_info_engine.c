/*******************************************************************************
 Filename:       network_info_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:


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
#include "network_info_engine.h"
#include "nwkmgr.pb-c.h"
#include "user_interface.h"

/*******************************************************************************
 * Functions
 ******************************************************************************/

void nwk_process_ready_ind(pkt_buf_t * pkt)
{
	NwkZigbeeNwkReadyInd *msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_ZIGBEE_NWK_READY_IND)
	{
		return;
	}

	UI_PRINT_LOG("nwk_process_ready_ind: Received NWK_ZIGBEE_NWK_READY_IND");

	msg = nwk_zigbee_nwk_ready_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		ds_network_status.state = ZIGBEE_NETWORK_STATE_READY;
		ds_network_status.nwk_channel = msg->nwkchannel; 
		ds_network_status.pan_id = msg->panid;
		ds_network_status.ext_pan_id = msg->extpanid;
		ds_network_status.permit_remaining_time = 0x0;
		ds_network_status.num_pending_attribs = 0x0;

		nwk_zigbee_nwk_ready_ind__free_unpacked(msg, NULL);

		ui_refresh_display();
	}
	else
	{
		UI_PRINT_LOG("nwk_process_ready_ind: Error: Could not unpack msg");
	}
}

void nwk_process_info_cnf(pkt_buf_t * pkt, void * cbarg)
{
	NwkZigbeeNwkInfoCnf *msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_ZIGBEE_NWK_INFO_CNF)
	{
		return;
	}

	UI_PRINT_LOG("nwk_process_info_cnf: Received NWK_ZIGBEE_NWK_INFO_CNF");

	msg = nwk_zigbee_nwk_info_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		UI_PRINT_LOG("msg->status = %d", msg->status);

		/* Update network info structure with received information */
		if (msg->status == NWK_NETWORK_STATUS_T__NWK_UP)
		{
			ds_network_status.state = ZIGBEE_NETWORK_STATE_READY;
			ds_network_status.nwk_channel = msg->nwkchannel; 
			ds_network_status.pan_id = msg->panid;
			ds_network_status.ext_pan_id = msg->extpanid;

			UI_PRINT_LOG("nwk_process_info_cnf: Network ready");
		}
		else
		{
			ds_network_status.state = ZIGBEE_NETWORK_STATE_INITIALIZING;
		}

		nwk_zigbee_nwk_info_cnf__free_unpacked(msg, NULL);	

		ui_refresh_display();
	}
	else
	{
		UI_PRINT_LOG("nwk_process_info_cnf: Error: Could not unpack msg");
	}
}

void nwk_send_info_request(void)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkZigbeeNwkInfoReq msg = NWK_ZIGBEE_NWK_INFO_REQ__INIT;

	UI_PRINT_LOG("nwk_send_info_request: Sending NWK_INFO_REQ__INIT");
	
	len = nwk_zigbee_nwk_info_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id = NWK_MGR_CMD_ID_T__NWK_ZIGBEE_NWK_INFO_REQ;

		nwk_zigbee_nwk_info_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&nwk_process_info_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("nwk_send_info_request: Error: Could not send msg");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("nwk_send_info_request: Error: Could not pack msg");
	}
}
