/*******************************************************************************
 Filename:      state_reflector.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:	Inquire current state of devices after sending a request to change state


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

#include "types.h"
#include "data_structures.h"
#include "state_reflector.h"
#include "attribute_engine.h"
#include "gateway.pb-c.h"
#include "user_interface.h"
#include "timer_utils.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table that holds all pending attributes */ 
static pending_attribute_info_t pending_attribs[MAX_PENDING_ATTRIBUTES] = 
{
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
  {false},
};

static tu_timer_t aging_timer = TIMER_RESET_VALUE;

static void aging_engine(void * arg);

/*******************************************************************************
 * Functions
 ******************************************************************************/
 
void sr_register_attribute_read_request(uint16_t seq_num, zb_addr_t * addr, uint16_t cluster_id, uint16_t * attr_ids, uint8_t attr_num)
{
	//Loop through the table to find an invalid entry and add entry there
	int i, j;

	UI_PRINT_LOG("sr_register_attribute_read_request: Received register read request for %d attributes.", attr_num);

	bool match = false; //Found a matching attribute
	bool found = false; //Found a free entry 
	int free_index = -1;

	for (i=0; i < MAX_PENDING_ATTRIBUTES; i++)
	{

		if ((free_index == -1) && (pending_attribs[i].valid == false))
		{
			free_index = i;
			found = true;
		}

		if ((pending_attribs[i].valid == true) 
			&& (pending_attribs[i].ieee_addr == addr->ieee_addr) 
			&& (pending_attribs[i].endpoint_id == addr->endpoint)
			&& (pending_attribs[i].cluster_id == cluster_id)
			&& (pending_attribs[i].num_attributes == attr_num))
		{
			match = true;
			
			for (j = 0; j < attr_num; j++)
			{
				if (pending_attribs[i].attr_id[j] != attr_ids[j]) match = false;
			}

			if (match)
			{
				pending_attribs[i].sequence_num = seq_num;
				pending_attribs[i].timer_val = READ_ATTR_TIMEOUT_VAL;
				found = true;
				break;
			}
		}
	}

	if (found)
	{
		if (match == false)
		{
			pending_attribs[free_index].valid = true;
			pending_attribs[free_index].sequence_num = seq_num;
			pending_attribs[free_index].ieee_addr = addr->ieee_addr;
			pending_attribs[free_index].endpoint_id = addr->endpoint;
			pending_attribs[free_index].cluster_id = cluster_id;
			pending_attribs[free_index].num_attributes = attr_num;
			pending_attribs[free_index].timer_val = READ_ATTR_TIMEOUT_VAL;

			for (j = 0; j < attr_num; j++)
			{
				pending_attribs[free_index].attr_id[j] = attr_ids[j];
				UI_PRINT_LOG("sr_register_attribute_read_request: Adding attribute read for attr_id 0x%x", pending_attribs[i].attr_id[j]);
			}

			ds_network_status.num_pending_attribs++;

			if (ds_network_status.num_pending_attribs == 1)
			{
				tu_set_timer(&aging_timer, TIMER_CHECK_VAL * 1000, true, &aging_engine, 
				NULL); 
			}
		}

		UI_PRINT_LOG("sr_register_attribute_read_request: Total pending attributes %d.", ds_network_status.num_pending_attribs);
	}
	else
	{
		UI_PRINT_LOG("sr_register_attribute_read_request: Error: Could not add to pending attribute table.");
	}
}

void sr_process_generic_response_indication(pkt_buf_t * pkt)
{
	//Sends out attribute read request
	GwZigbeeGenericRspInd *msg = NULL;
	int i, count;
	zb_addr_t addr;
	int num_deleted = 0;

	if (pkt->header.cmd_id != GW_CMD_ID_T__ZIGBEE_GENERIC_RSP_IND)
	{
		return;
	}

	UI_PRINT_LOG("sr_process_generic_response_indication: Received GW ZIGBEE_GENERIC_RSP_IND.");

	msg = gw_zigbee_generic_rsp_ind__unpack(NULL, pkt->header.len, 
	pkt->packed_protobuf_packet); 

	if (msg)
	{
		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("sr_process_generic_response_indication: Status SUCCESS.");
			
			//Search for the sequence number in the table, and when found delete the entry
			UI_PRINT_LOG("sr_process_generic_response_indication: searching for seq_num=%d", msg->sequencenumber);

			for (i=0, count=0 ; (i < MAX_PENDING_ATTRIBUTES) && (count < ds_network_status.num_pending_attribs); i++)
			{
				UI_PRINT_LOG("sr_process_generic_response_indication: pending_attribs[%d] vld=%d seq_num=%d", i, pending_attribs[i].valid, pending_attribs[i].sequence_num);

				if (pending_attribs[i].valid)
				{
					if (pending_attribs[i].sequence_num == msg->sequencenumber)
					{
						UI_PRINT_LOG("sr_process_generic_response_indication: Found a match in the pending attributes table..");

						addr.ieee_addr = pending_attribs[i].ieee_addr;
						addr.endpoint = pending_attribs[i].endpoint_id;

						UI_PRINT_LOG("sr_process_generic_response_indication: Sending attribute read request for %d attributes.", pending_attribs[i].num_attributes); 

						attr_send_read_attribute_request(&addr, 
							(uint16_t) pending_attribs[i].cluster_id, 
							pending_attribs[i].num_attributes, 
							pending_attribs[i].attr_id);

						//Delete entry from table
						pending_attribs[i].valid = false;

						num_deleted++;
					}

					count++;
				}
			}

			ds_network_status.num_pending_attribs -= num_deleted;

			UI_PRINT_LOG("sr_process_generic_response_indication: Total pending attributes %d.", ds_network_status.num_pending_attribs);

			if (ds_network_status.num_pending_attribs == 0)
			{
				//Delete timer when all attributes have been read
				tu_kill_timer(&aging_timer);	
			}
		}
		else
		{
			UI_PRINT_LOG("sr_process_generic_response_indication: Status FAILURE (%d)", msg->status);
		}

		gw_zigbee_generic_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("sr_process_generic_response_indication: Error Could not unpack msg.");
	}
}

static void aging_engine(void * arg)
{
	int i, count;
	int num_deleted = 0;

	for (i=0, count=0 ; (i < MAX_PENDING_ATTRIBUTES) &&	(count < ds_network_status.num_pending_attribs); i++)
	{
		/* Every second decrement the timeout val for all "valid" entries */
		if (pending_attribs[i].valid)
		{
			if (pending_attribs[i].timer_val != 0)
			{
				pending_attribs[i].timer_val--;
			}
			else
			{
				/* delete entries that have timed out */ 
				pending_attribs[i].valid = false;
				num_deleted++;
				UI_PRINT_LOG("aging_engine: Attr read from device addr 0x%Lx, timed out waiting for response", pending_attribs[i].ieee_addr);
			}
			
			count++;
		}
	}

	ds_network_status.num_pending_attribs -= num_deleted;

	if (ds_network_status.num_pending_attribs == 0)
	{
		//Delete timer when all attributes have been read
		tu_kill_timer(&aging_timer);	
		UI_PRINT_LOG("aging_engine: Killing timer.");
	}
}
