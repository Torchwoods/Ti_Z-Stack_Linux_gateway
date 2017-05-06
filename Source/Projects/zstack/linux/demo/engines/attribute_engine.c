/*******************************************************************************
 Filename:      attribute_engine.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:	 Engine to send attribute read requests and handle responses.


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

#include "nwkmgr.pb-c.h"
#include "data_structures.h"
#include "socket_interface.h"
#include "attribute_engine.h"
#include "gateway.pb-c.h"
#include "user_interface.h"
#include "group_scene_engine.h"
#include "commissioning_engine.h"

/******************************************************************************
 * Functional Macros
 *****************************************************************************/
#define TABLE_ENTRY(dev)		ds_device_table[dev]
#define TABLE_NUM_ENDPOINTS(dev)	ds_device_table[dev].num_endpoints
#define TABLE_ENDPOINT(dev,ep)  	ds_device_table[dev].ep_list[ep]
	
#define TABLE_NUM_OP_CLUSTERS(dev, ep)  ds_device_table[dev].ep_list[ep].num_op_clusters
#define TABLE_NUM_IP_CLUSTERS(dev, ep)  ds_device_table[dev].ep_list[ep].num_ip_clusters
#define TABLE_OP_CLUSTER(dev, ep, cid)  ds_device_table[dev].ep_list[ep].op_cluster_list[cid]  
#define TABLE_IP_CLUSTER(dev, ep, cid)  ds_device_table[dev].ep_list[ep].ip_cluster_list[cid]  
	
#define TABLE_OP_NUM_ATTRS(dev, ep, cid)  ds_device_table[dev].ep_list[ep].op_cluster_list[cid].num_attributes;
#define TABLE_IP_NUM_ATTRS(dev, ep, cid)  ds_device_table[dev].ep_list[ep].ip_cluster_list[cid].num_attributes;
#define TABLE_OP_ATTR(dev, ep, cid, attr)  ds_device_table[dev].ep_list[ep].op_cluster_list[cid].attribute_list[attr]
#define TABLE_IP_ATTR(dev, ep, cid, attr)  ds_device_table[dev].ep_list[ep].ip_cluster_list[cid].attribute_list[attr]
	
/*******************************************************************************
 * Types
 ******************************************************************************/

typedef struct
{
	zb_addr_t reporting_source_addr;
	zb_addr_t reporting_destination_addr;
	uint16_t interval_seconds;
	uint32_t cluster_id;
	uint32_t attributeid;
	GwZclAttributeDataTypesT attributetype;
} reporting_request_t;

/*******************************************************************************
 * Functions
 ******************************************************************************/
	
void attr_read_confirm_handler (pkt_buf_t *pkt, void *cbArg) 
{
	NwkZigbeeGenericCnf * msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("attr_read_confirm_handler: Received ZIGBEE_GENERIC_CNF");

	msg = nwk_zigbee_generic_cnf__unpack(NULL, pkt->header.len, 
	pkt->packed_protobuf_packet);	

	if (msg )
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attr_read_confirm_handler: Status SUCCESS.");
		}
		else
		{
			UI_PRINT_LOG("attr_read_confirm_handler:  Status FAILURE.");
		}


		nwk_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attr_read_confirm_handler: Error Could not unpack msg.");
	}
}

void attr_process_read_attribute_response(pkt_buf_t * pkt)
{
	GwReadDeviceAttributeRspInd *msg = NULL;
	int i;
	attribute_info_t attr_recvd[MAX_ATTRIBUTES];

	if (pkt->header.cmd_id != GW_CMD_ID_T__GW_READ_DEVICE_ATTRIBUTE_RSP_IND)
	{
		return;
	}

	msg = gw_read_device_attribute_rsp_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{

		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attr_process_read_attribute_response: Status SUCCESS.");

			UI_PRINT_LOG("addr=%s", ui_make_string_GwAddrStructT(msg->srcaddress));
			UI_PRINT_LOG("msg->n_attributerecordlist = %d", msg->attributerecordlist);
			UI_PRINT_LOG("msg->attributerecordcount = %d", msg->n_attributerecordlist);

			for (i = 0; i < msg->n_attributerecordlist; i++)
			{
				UI_PRINT_LOG("attr_id = %d", msg->attributerecordlist[i]->attributeid);
				UI_PRINT_LOG("attr_type = %d", msg->attributerecordlist[i]->attributetype);
				UI_PRINT_LOG("len = %d", msg->attributerecordlist[i]->attributevalue.len);

				attr_recvd[i].valid = true;
				attr_recvd[i].attr_id = msg->attributerecordlist[i]->attributeid;
				attr_recvd[i].attr_type = msg->attributerecordlist[i]->attributetype;

				memcpy(attr_recvd[i].attr_val, 
					msg->attributerecordlist[i]->attributevalue.data,
					msg->attributerecordlist[i]->attributevalue.len);
			}

			attr_update_attribute_in_dev_table(msg->srcaddress->ieeeaddr, 
				msg->srcaddress->endpointid, msg->clusterid, 
				msg->n_attributerecordlist, attr_recvd);  

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("attr_process_read_attribute_response: Error: Status FAILURE.");
		}

		gw_read_device_attribute_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attr_process_read_attribute_response: Error: Could not unpack msg.");
	}
}

void attr_send_read_attribute_request(zb_addr_t *addr, uint32_t cluster_id, uint16_t attr_count, uint32_t * attr_id)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwReadDeviceAttributeReq msg = GW_READ_DEVICE_ATTRIBUTE_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	dstaddr.addresstype = GW_ADDRESS_TYPE_T__UNICAST;
	dstaddr.has_ieeeaddr = true;
	dstaddr.ieeeaddr = addr->ieee_addr;
	dstaddr.has_endpointid = true;
	dstaddr.endpointid = addr->endpoint;

	msg.dstaddress = &dstaddr;
	msg.clusterid = cluster_id;
	msg.n_attributelist = attr_count;
	//msg.n_attributelist = msg.attributecount;
	msg.attributelist = attr_id;

	len = gw_read_device_attribute_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_READ_DEVICE_ATTRIBUTE_REQ;

		gw_read_device_attribute_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)attr_read_confirm_handler, NULL) != 0)
		{
			UI_PRINT_LOG("attr_send_read_attribute_request: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("attr_send_read_attribute_request: Error: Could not unpack msg.");
	}
}

bool attr_update_attribute_in_dev_table(uint64_t ieee_address, uint8_t endpoint_id,	uint32_t cluster_id, uint32_t num_attrs, attribute_info_t *attr_list)
{
	int i,j,k,l,m;
	bool found;
	bool error = false;
	int freeEntry;
	char log_string[5 * UI_LOG_LINE_LENGTH];

	STRING_START(log_string, "attr_update_attribute_in_dev_table, IEEE=");
	STRING_ADD_64BIT_HEX(log_string, ieee_address);
	STRING_ADD(log_string, " ep=%02X, clstr=%04x", endpoint_id, cluster_id);
	
	for (i = 0; i < num_attrs; i++)
	{
		STRING_ADD(log_string, "\n vld=%d, id=%d, tp=%d, val=%08X", attr_list[i].valid, attr_list[i].attr_id, attr_list[i].attr_type, LITTLE_ENDIAN_TO_INT32(attr_list[i].attr_val));
	}

	UI_PRINT_LOG(log_string);

	for (i = 0; i < MAX_DEVICE_LIST; i++)
	{
		if ((ds_device_table[i].valid) 
			&& (ds_device_table[i].device_status != GW_DEVICE_STATUS_T__DEVICE_REMOVED) 
			&& (ds_device_table[i].ieee_addr == ieee_address))
		{
			for (j = 0; j < TABLE_NUM_ENDPOINTS(i); j++)
			{
				if (TABLE_ENDPOINT(i,j).endpoint_id == endpoint_id)
				{
					for (k = 0; k < TABLE_NUM_IP_CLUSTERS(i,j); k++)
					{
						if (TABLE_IP_CLUSTER(i,j,k).cluster_id == cluster_id)
						{
							/* Update Attributes. Loop through the attribs returned */
							for (l = 0; l < num_attrs; l++)
							{
								found = false;
								freeEntry = -1;

								/* Find a attributeid entry in the existing table */
								for (m = 0; m < MAX_ATTRIBUTES; m++)
								{
									/* Found a match, update the attr_val */
									if ((TABLE_IP_ATTR(i,j,k,m).valid == true) && (attr_list[l].attr_id == TABLE_IP_ATTR(i,j,k,m).attr_id))
									{

										memcpy(TABLE_IP_ATTR(i,j,k,m).attr_val, &attr_list[l].attr_val, MAX_ATTRIBUTE_SIZE);

										UI_PRINT_LOG("attr_update_attribute_in_dev_table: Updated existing attribute - index = %d.", m);

										found = true;

										break;
									}
									else
									{
										if ((freeEntry == -1) && (TABLE_IP_ATTR(i,j,k,m).valid == false))
										{
											freeEntry = m;
										}
									}
								}

								if (!found)
								{
									if  (freeEntry != -1)
									{
										TABLE_IP_ATTR(i,j,k,freeEntry).valid = true;
										TABLE_IP_ATTR(i,j,k,freeEntry).attr_id = attr_list[l].attr_id;
										TABLE_IP_ATTR(i,j,k,freeEntry).attr_type = 
										attr_list[l].attr_type;
										memcpy(TABLE_IP_ATTR(i,j,k,freeEntry).attr_val, &attr_list[l].attr_val, MAX_ATTRIBUTE_SIZE);
										UI_PRINT_LOG("attr_update_attribute_in_dev_table: Adding new attribute - index = %d.", freeEntry);
									}
									else
									{
										UI_PRINT_LOG("attr_update_attribute_in_dev_table: Error: No space for attribute entry in device_info table.");
										error = true;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	
	return error;
}

void attr_process_set_attribute_reporting_rsp_ind(pkt_buf_t * pkt)
{
	GwSetAttributeReportingRspInd *msg = NULL;
	int i;

	if (pkt->header.cmd_id != GW_CMD_ID_T__GW_SET_ATTRIBUTE_REPORTING_RSP_IND)
	{
		return;
	}

	msg = gw_set_attribute_reporting_rsp_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{

		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attr_process_set_attribute_reporting_rsp_ind: Status SUCCESS.");

		}
		else
		{
			UI_PRINT_LOG("attr_process_set_attribute_reporting_rsp_ind: WARNING: Status FAILURE msg->status = %d",msg->status);
		}

		for (i = 0; i < msg->n_attributereportconfiglist; i++)
		{
			UI_PRINT_LOG("Attribute could NOT be configured clusted id =0x%x attribute id=%d", msg->clusterid, msg->attributereportconfiglist[i]->attributeid);
		}


		gw_set_attribute_reporting_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attr_process_set_attribute_reporting_rsp_ind: ERROR: Could not unpack msg.");
	}
}

void attr_set_attribute_report_req (zb_addr_t *addr, uint32_t count, uint32_t cluster_id,  uint32_t minreportinterval,  uint32_t maxreportinterval, uint32_t* attributeid,GwZclAttributeDataTypesT* attributetype )
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwSetAttributeReportingReq msg = GW_SET_ATTRIBUTE_REPORTING_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;
	GwAttributeReportT **ppAttributeReportList = NULL ;
	GwAttributeReportT *pAttributeReportList = NULL;

	si_compose_address(addr, &dstaddr);
 
	msg.n_attributereportlist = count;

	ppAttributeReportList =  malloc( sizeof( GwAttributeReportT * ) * msg.n_attributereportlist );
	// Ensure memory has been allocated
	if( ppAttributeReportList == NULL )
	{
	    return;
	}

	// Allocate memory for protobuf structures
	pAttributeReportList =  malloc( sizeof( GwAttributeReportT ) * msg.n_attributereportlist );
	// Ensure memory has been allocated
	if( pAttributeReportList == NULL)
	{
		free( ppAttributeReportList );
		return;
	}

	msg.attributereportlist = ppAttributeReportList;

	for(int i = 0; i < count; i++ )
	{
		ppAttributeReportList[i] = &pAttributeReportList[i];
		gw_attribute_report_t__init( &pAttributeReportList[i] );

		pAttributeReportList[i].attributeid = attributeid[i];

		pAttributeReportList[i].attributetype = attributetype[i];

		pAttributeReportList[i].minreportinterval = minreportinterval;
		pAttributeReportList[i].maxreportinterval = maxreportinterval;
		pAttributeReportList[i].has_reportablechange = true;
		pAttributeReportList[i].reportablechange = 1;
	}

	msg.dstaddress = &dstaddr;
	msg.n_attributereportlist = count;
	msg.clusterid = cluster_id ;

	len = gw_set_attribute_reporting_req__get_packed_size(&msg);

	pkt = malloc(sizeof(pkt_buf_hdr_t) + len);

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW;
		pkt->header.cmd_id = GW_CMD_ID_T__GW_SET_ATTRIBUTE_REPORTING_REQ;

		gw_set_attribute_reporting_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, attr_read_confirm_handler,NULL) != 0)
		{
			UI_PRINT_LOG("attr_set_attribute_report_req: Error: Could not send msg.");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("attr_set_attribute_report_req: Error: Could not create pkt.");
	}

	if (pAttributeReportList != NULL)
	{
		free( pAttributeReportList );
	}

	if (ppAttributeReportList != NULL)
	{
		free( ppAttributeReportList );
	}
}

void attr_process_attribute_report_ind(pkt_buf_t * pkt)
{
	GwAttributeReportingInd *msg = NULL;
	int i;
	attribute_info_t attr_recvd[MAX_ATTRIBUTES];

	if (pkt->header.cmd_id != GW_CMD_ID_T__GW_ATTRIBUTE_REPORTING_IND)
	{
		return;
	}

	msg = gw_attribute_reporting_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{

		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attr_process_attribute_report_ind: Status SUCCESS.");

			UI_PRINT_LOG("addr=%s", ui_make_string_GwAddrStructT(msg->srcaddress));
			UI_PRINT_LOG("msg->n_attributerecordlist = %d", msg->n_attributerecordlist);

			for (i = 0; i < msg->n_attributerecordlist; i++)
			{
				UI_PRINT_LOG("attr_id = %d", msg->attributerecordlist[i]->attributeid);
				UI_PRINT_LOG("attr_type = %d", msg->attributerecordlist[i]->attributetype);
				UI_PRINT_LOG("len = %d", msg->attributerecordlist[i]->attributevalue.len);

				attr_recvd[i].valid = true;
				attr_recvd[i].attr_id = msg->attributerecordlist[i]->attributeid;
				attr_recvd[i].attr_type = msg->attributerecordlist[i]->attributetype;

				memcpy(attr_recvd[i].attr_val, 
					msg->attributerecordlist[i]->attributevalue.data,
					msg->attributerecordlist[i]->attributevalue.len);
			}

			attr_update_attribute_in_dev_table(msg->srcaddress->ieeeaddr, 
				msg->srcaddress->endpointid, msg->clusterid, 
				msg->n_attributerecordlist, attr_recvd);  

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("attr_process_attribute_report_ind: Error: Status FAILURE.");
		}

		gw_attribute_reporting_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attr_process_attribute_report_ind: Error: Could not unpack msg.");
	}
}

bool attr_set_attribute_report_req_state_machine(bool timed_out, void * arg)
{
	static int state = 0;
	reporting_request_t * reporting_request = (reporting_request_t *)arg;
	bool rerun_state_machine = true;

	if (si_is_server_ready(SI_SERVER_ID_GATEWAY) == false)
	{
		UI_PRINT_LOG("attr_set_attribute_report_req_state_machine: ERROR: GATEWAY server not connected");
		state = 5;
	}
	else if ((!timed_out) || (state == 0))
	{
		state++;
	}
	
	while (rerun_state_machine)
	{
		UI_PRINT_LOG("Reporting Register state %d", state);
		
		rerun_state_machine = false;
		switch (state)
		{
			case 1:
				if (si_is_waiting_for_confirmation() == false)
				{
					state++;
					rerun_state_machine = true;
				}
 				break;
			case 2:
				comm_device_binding_entry_request(&reporting_request->reporting_source_addr, &reporting_request->reporting_destination_addr, reporting_request->cluster_id, BINDING_MODE_BIND);
				break;
			case 3:
				attr_set_attribute_report_req(&reporting_request->reporting_source_addr, 1, reporting_request->cluster_id, reporting_request->interval_seconds, reporting_request->interval_seconds, &reporting_request->attributeid, &reporting_request->attributetype);
				break;
			case 4:
				si_delay_next_idle_state_machine_state(5000);
				break;
			case 5:
				state = 0;
				free(arg);
				si_unregister_idle_callback(); //must be called last, since may recursively call attr_set_attribute_report_req_state_machine if it is registered again next in the queue
				break;
			default:
				break;
		}
	}

	return true;
}

void attr_configure_reporting(zb_addr_t reporting_source_addr, zb_addr_t reporting_destination_addr, uint16_t interval_seconds, uint32_t cluster_id, uint32_t attributeid, GwZclAttributeDataTypesT attributetype )
{
	reporting_request_t * reporting_request = malloc(sizeof(reporting_request_t));
	
	if (reporting_request == NULL)
	{
		UI_PRINT_LOG("ERROR: Reporting Register state machine cannot be initialized");
	}

	reporting_request->attributeid = attributeid;
	reporting_request->attributetype = attributetype;
	reporting_request->cluster_id = cluster_id;
	reporting_request->interval_seconds = interval_seconds;
	reporting_request->reporting_destination_addr = reporting_destination_addr;
	reporting_request->reporting_source_addr = reporting_source_addr;

	if (si_register_idle_callback((si_idle_calback_t)attr_set_attribute_report_req_state_machine, reporting_request) != 0)
	{
		UI_PRINT_LOG("WARNING: Reporting Register state machine cannot be initialized");
		free(reporting_request);
	}

	si_initiate_idle_callback();
}

void attribute_list_confirm_handler (pkt_buf_t *pkt, void *cbArg) 
{
	NwkZigbeeGenericCnf * msg = NULL;

	if (pkt->header.cmd_id != NWK_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("attribute_list_confirm_handler: Received ZIGBEE_GENERIC_CNF");

	msg = nwk_zigbee_generic_cnf__unpack(NULL, pkt->header.len, 
	pkt->packed_protobuf_packet);	

	if (msg )
	{
		if (msg->status == NWK_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attribute_list_confirm_handler: Status SUCCESS.");
		}
		else
		{
			UI_PRINT_LOG("attribute_list_confirm_handler:  Status FAILURE.");
		}


		nwk_zigbee_generic_cnf__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attribute_list_confirm_handler: Error Could not unpack msg.");
	}
}

void attr_get_attribute_list(zb_addr_t addr)
{
	pkt_buf_t * pkt = NULL;
	uint8_t len = 0;
	GwGetDeviceAttributeListReq msg = GW_GET_DEVICE_ATTRIBUTE_LIST_REQ__INIT;
	GwAddressStructT dstaddr = GW_ADDRESS_STRUCT_T__INIT;

	dstaddr.addresstype = GW_ADDRESS_TYPE_T__UNICAST;
	dstaddr.has_ieeeaddr = true;
	dstaddr.ieeeaddr = addr.ieee_addr;
	dstaddr.has_endpointid = true;
	dstaddr.endpointid = addr.endpoint;

	msg.dstaddress = &dstaddr;

	len = gw_get_device_attribute_list_req__get_packed_size(&msg);
	pkt = malloc(sizeof(pkt_buf_hdr_t) + len); 
	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = Z_STACK_GW_SYS_ID_T__RPC_SYS_PB_GW; //gateway
		pkt->header.cmd_id = GW_CMD_ID_T__GW_GET_DEVICE_ATTRIBUTE_LIST_REQ;

		gw_get_device_attribute_list_req__pack(&msg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, attribute_list_confirm_handler, NULL) != 0)
		{
			UI_PRINT_LOG("attr_get_attribute_list: Error: Could not send msg.");
		}
		
		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("attr_get_attribute_list: Error: Could not pack msg.");
	}
}

void attr_process_attribute_list_rsp_ind(pkt_buf_t * pkt)
{
	GwGetDeviceAttributeListRspInd *msg = NULL;
	int cluster;
	int attrib;

	if (pkt->header.cmd_id != GW_CMD_ID_T__GW_GET_DEVICE_ATTRIBUTE_LIST_RSP_IND)
	{
		return;
	}

	msg = gw_get_device_attribute_list_rsp_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{

		if (msg->status == GW_STATUS_T__STATUS_SUCCESS)
		{
			UI_PRINT_LOG("attr_process_attribute_list_rsp_ind: Status SUCCESS.");

			UI_PRINT_LOG("addr=%s", ui_make_string_GwAddrStructT(msg->srcaddress));
			UI_PRINT_LOG("msg->n_clusterlist = %d", msg->n_clusterlist);

			for (cluster = 0; cluster < msg->n_clusterlist; cluster++)
			{
				UI_PRINT_LOG("clusterid = %d", msg->clusterlist[cluster]->clusterid);
				UI_PRINT_LOG("  n_attributelist = %d", msg->clusterlist[cluster]->n_attributelist);

				for (attrib = 0; attrib < msg->clusterlist[cluster]->n_attributelist; attrib++)
				{
					UI_PRINT_LOG("  attribute = %d", msg->clusterlist[cluster]->attributelist[attrib]);
				}

			}
		}
		else
		{
			UI_PRINT_LOG("attr_process_attribute_list_rsp_ind: Error: Status FAILURE.");
		}

		gw_get_device_attribute_list_rsp_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("attr_process_attribute_list_rsp_ind: Error: Could not unpack msg.");
	}
}


