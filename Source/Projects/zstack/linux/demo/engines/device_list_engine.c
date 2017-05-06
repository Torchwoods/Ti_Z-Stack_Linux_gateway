/*******************************************************************************
 Filename:       device_list_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    Device List Engine manages updating the list of devices in the network.


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
 * Include
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "data_structures.h"
#include "socket_interface.h"
#include "device_list_engine.h"
#include "nwkmgr.pb-c.h"
#include "user_interface.h" 

/******************************************************************************
 * Constants
 *****************************************************************************/

#define GATEWAY_MANAGEMENT_ENDPOINT 2

/******************************************************************************
 * Global Variables
 *****************************************************************************/

zb_addr_t gateway_self_addr = {0, 0, 0};

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
static inline bool update_device_table_entry(device_info_t * entry,	NwkDeviceInfoT * dev_info);
static inline bool update_endpoint_entry(endpoint_info_t * ep_info, NwkSimpleDescriptorT *pbuf_simple_desc);

/******************************************************************************
 * Function
 *****************************************************************************/

void device_invalidate_entry(device_info_t * entry)
{
	memset(entry, 0, sizeof(*entry));
}

void device_process_list_response(pkt_buf_t * pkt, void * cbarg)
{
	NwkGetDeviceListCnf *msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_GET_DEVICE_LIST_CNF)
	{
		return;
	}
	
	UI_PRINT_LOG("device_process_list_response: Received NWK_GET_DEVICE_LIST_CNF");

	msg = nwk_get_device_list_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("device_process_list_response: Status SUCCESS.");

			int i;

			ds_devices_total = msg->n_devicelist + 1;

			if (ds_devices_total > MAX_DEVICE_LIST)
			{
				UI_PRINT_LOG("device_process_list_response: Error: Number of devices exceed MAX"); 
				ds_devices_total = MAX_DEVICE_LIST;
			}

			UI_PRINT_LOG("device_process_list_response: Total Devices %d", ds_devices_total);

			for (i = 1; i < MAX_DEVICE_LIST; i++)
			{
				device_invalidate_entry(&ds_device_table[i]);
			}

			/* Updated ds_device_table with information returned from this call */
			for (i = 1; i < ds_devices_total; i++)
			{
				if (false == update_device_table_entry(&ds_device_table[i], msg->devicelist[i - 1]))
				{
					device_invalidate_entry(&ds_device_table[i]);
					UI_PRINT_LOG("device_process_list_response: Error populating Device Table for device 0x%Lx", msg->devicelist[i - 1]->ieeeaddress);
				}
			}

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("device_process_list_response: Error: Status FAILURE.");
		}
		
		nwk_get_device_list_cnf__free_unpacked(msg, NULL);  
	}
	else
	{
		UI_PRINT_LOG("device_process_list_response: Error Could not unpack msg");
	}
}

void device_process_local_info_response(pkt_buf_t * pkt, void * cbarg)
{
	NwkGetLocalDeviceInfoCnf *msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_GET_LOCAL_DEVICE_INFO_CNF)
	{
		return;
	}
	
	UI_PRINT_LOG("device_process_local_info_response: Received NWK_GET_LOCAL_DEVICE_INFO_CNF");

	msg = nwk_get_local_device_info_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		gateway_self_addr.ieee_addr = msg->deviceinfolist->ieeeaddress;
		gateway_self_addr.endpoint = GATEWAY_MANAGEMENT_ENDPOINT;
		
		device_invalidate_entry(&ds_device_table[0]); //TBD: maybe not required
		
		if (false == update_device_table_entry(&ds_device_table[0], msg->deviceinfolist))
		{
			device_invalidate_entry(&ds_device_table[0]);
			UI_PRINT_LOG("device_process_local_info_response: Error populating Device Table for device 0x%Lx", msg->deviceinfolist->ieeeaddress);
		}

		ui_refresh_display();
		
		nwk_get_local_device_info_cnf__free_unpacked(msg, NULL);  
	}
	else
	{
		UI_PRINT_LOG("device_process_local_info_response: Error Could not unpack msg");
	}
}

void device_process_change_indication(pkt_buf_t * pkt)
{
	NwkZigbeeDeviceInd *msg = NULL;
	int free_index = -1;
	int removed_index = -1;
	int i = 0;
	bool found = false;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__NWK_ZIGBEE_DEVICE_IND)
	{
		return;
	}

	UI_PRINT_LOG("device_process_change_indication: Received NWK_ZIGBEE_DEVICE_IND");

	msg = nwk_zigbee_device_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		/* Update device info in the device list */

		//Loop through the table to find a non-valid entry
		for (i= 0; i < MAX_DEVICE_LIST; i++)
		{
			//Found a free index location
			if ((free_index == -1) && (!ds_device_table[i].valid))
			{
				free_index = i;
			}
			else if ((removed_index == -1) && (ds_device_table[i].device_status == NWK_DEVICE_STATUS_T__DEVICE_REMOVED))
			{
				removed_index = i;
			}

			//Check to see if there is a match (ieee_addr) first with a 
			//valid/non-valid entry
			if (ds_device_table[i].ieee_addr == msg->deviceinfo->ieeeaddress)
			{
				found = true;
				break; 
			} 
		}

		if (found)
		{
			//Update same entry as before.
			UI_PRINT_LOG("device_process_change_indication: Found existing entry at index %d", i);

			if (msg->deviceinfo->devicestatus == NWK_DEVICE_STATUS_T__DEVICE_REMOVED)
			{
				device_invalidate_entry(&ds_device_table[i]);
				UI_PRINT_LOG("device_process_list_response: Device removed");
			}
			else
			{
				if (false == update_device_table_entry(&ds_device_table[i], msg->deviceinfo))
				{
					device_invalidate_entry(&ds_device_table[i]);
					UI_PRINT_LOG("device_process_list_response: Error populating Device Table, skipping device 0x%Lx", msg->deviceinfo->ieeeaddress);
				}
				else
				{
					//If valid flag is false, then set to true and increment ds_devices_total
					if (!ds_device_table[i].valid)
					{
						ds_device_table[i].valid = true;
						ds_devices_total++;
					}
				}
			}

			ui_refresh_display();
		}
		else
		{
			if (free_index != -1)
			{

				UI_PRINT_LOG("device_process_change_indication: Adding new entry at index %d", free_index);

				//Use free_index location for new entry and increment ds_devices_total
				if (false == update_device_table_entry(&ds_device_table[free_index], msg->deviceinfo))
				{
					device_invalidate_entry(&ds_device_table[free_index]);
					UI_PRINT_LOG("device_process_change_indication: Error populating Device Table, skipping device 0x%Lx", msg->deviceinfo->ieeeaddress);
				}
				else
				{
					ds_devices_total++;
					ui_refresh_display();
				}
			}
			else if (removed_index != -1)
			{
				UI_PRINT_LOG("device_process_change_indication: Adding new entry at index %d", removed_index);

				if ( false == update_device_table_entry(&ds_device_table[removed_index], msg->deviceinfo))
				{
					device_invalidate_entry(&ds_device_table[removed_index]);
					UI_PRINT_LOG("device_process_change_indication: Error populating Device Table, skipping device 0x%Lx.", msg->deviceinfo->ieeeaddress);
				}
				else
				{
					ui_refresh_display();
				}
			}
			else
			{
				//Error out: Not configured for these many entries !!
				UI_PRINT_LOG("device_process_change_indication: ERROR Device Table Full");
			}
		}

		nwk_zigbee_device_ind__free_unpacked(msg, NULL);  
	}
}

void device_send_list_request(void)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkGetDeviceListReq msg = NWK_GET_DEVICE_LIST_REQ__INIT;

	UI_PRINT_LOG("device_send_list_request: Sending NWK_GET_DEVICE_LIST_REQ");

	len = nwk_get_device_list_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id = NWK_MGR_CMD_ID_T__NWK_GET_DEVICE_LIST_REQ;

		nwk_get_device_list_req__pack(&msg, pkt->packed_protobuf_packet);
		
		if (si_send_packet(pkt,	(confirmation_processing_cb_t)&device_process_list_response, NULL) != 0)
		{
			UI_PRINT_LOG("device_send_list_request: Error: Could not send msg");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("device_send_list_request: Error: Could not pack msg");
	}
}

void device_send_local_info_request(void)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	NwkGetLocalDeviceInfoReq msg = NWK_GET_LOCAL_DEVICE_INFO_REQ__INIT;

	UI_PRINT_LOG("device_send_local_device_info_request: Sending NWK_GET_LOCAL_DEVICE_INFO_REQ");

	len = nwk_get_local_device_info_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_NWK_MGR_SYS_ID_T__RPC_SYS_PB_NWK_MGR;
		pkt->header.cmd_id = NWK_MGR_CMD_ID_T__NWK_GET_LOCAL_DEVICE_INFO_REQ;

		nwk_get_local_device_info_req__pack(&msg, pkt->packed_protobuf_packet);
		
		if (si_send_packet(pkt,	(confirmation_processing_cb_t)&device_process_local_info_response, NULL) != 0)
		{
			UI_PRINT_LOG("device_send_local_device_info_request: Error: Could not send msg");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("device_send_local_device_info_request: Error: Could not pack msg");
	}
}

static inline bool update_device_table_entry(device_info_t * entry,	NwkDeviceInfoT * dev_info)
{
	int j;

	entry->valid = true;
	entry->nwk_addr = dev_info->networkaddress;
	entry->ieee_addr = dev_info->ieeeaddress;
	entry->manufacturer_id = dev_info->manufacturerid;
	entry->device_status = dev_info->devicestatus;
	entry->num_endpoints = dev_info->n_simpledesclist;

	if (entry->num_endpoints > MAX_ENDPOINTS)
	{
		UI_PRINT_LOG("update_device_table_entry: Error: Number of endpoints exceed MAX value configured (types.h: MAX_ENDPOINTS)."); 
		
		return false;
	}

	UI_PRINT_LOG("update_device_table_entry: Adding/Updating enry info nwk_addr 0x%x ieeeaddr 0x%Lx manu id 0x%x status %d num endpoints %d",
		entry->nwk_addr, entry->ieee_addr, entry->manufacturer_id,
		entry->device_status, entry->num_endpoints);

	for (j = 0; j < entry->num_endpoints; j++)
	{
		if (false == update_endpoint_entry(&(entry->ep_list[j]), dev_info->simpledesclist[j]))
		{
			return false;
		}
	}
	
	return true;
}

static inline bool update_endpoint_entry(endpoint_info_t * ep_info, 	NwkSimpleDescriptorT *pbuf_desc)
{
	int k = 0;
	int l = 0;

	ep_info->endpoint_id = pbuf_desc->endpointid;
	ep_info->profile_id = pbuf_desc->profileid;
	ep_info->device_id = pbuf_desc->deviceid;
	ep_info->num_ip_clusters = pbuf_desc->n_inputclusters;
	ep_info->num_op_clusters = pbuf_desc->n_outputclusters;

	if ((ep_info->num_ip_clusters > MAX_CLUSTERS_ON_EP) || 	(ep_info->num_op_clusters > MAX_CLUSTERS_ON_EP))
	{
		UI_PRINT_LOG("update_endpoint_entry: Error: Number of clusters exceed MAX value configured (types.h: MAX_CLUSTERS_ON_EP)."); 
		
		return false; 
	}

	for (k = 0; k < ep_info->num_ip_clusters; k++)
	{
		ep_info->ip_cluster_list[k].cluster_id = pbuf_desc->inputclusters[k];

		ep_info->ip_cluster_list[k].num_attributes = 0;

		for (l = 0; l < MAX_ATTRIBUTES; l++)
		{
			ep_info->ip_cluster_list[k].attribute_list[l].valid = false; 
		}
	}

	for (k = 0; k < ep_info->num_op_clusters; k++)
	{
		ep_info->op_cluster_list[k].cluster_id = pbuf_desc->outputclusters[k];
		ep_info->op_cluster_list[k].num_attributes = 0;

		for (l = 0; l < MAX_ATTRIBUTES; l++)
		{
			ep_info->op_cluster_list[k].attribute_list[l].valid = false; 
		}
	}
	
	return true;
}
