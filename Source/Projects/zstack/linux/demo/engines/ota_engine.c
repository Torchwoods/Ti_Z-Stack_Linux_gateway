/*******************************************************************************
 Filename:       ota_engine.c
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
#include <string.h>

#include "data_structures.h"
#include "socket_interface.h"
#include "otasrvr.pb-c.h"
#include "user_interface.h"
#include "ota_engine.h"

upgrade_info_t upgrade_file_status[MAX_UPGRADE_FILES];

int upgrade_num_files = 0;

static void register_single_file(int i);
static void parse_file_to_list (char * fileName);

static void upgrade_process_regis_cnf(pkt_buf_t * pkt, void * cbarg);
static void upgrade_apply_image_cnf(pkt_buf_t * pkt, void * cbarg);
static void enable_ota_upgrade(void);
void ota_register(char * fileName);
static void update_enable_cnf(pkt_buf_t * pkt, void * cbarg);


/*******************************************************************************
* Functions
******************************************************************************/

void ota_enable_state_machine(bool timed_out, void * arg)
{
	static int state = 0;
	static int current_ota_file_index;
	bool rerun_state_machine = true;

	if ((arg == NULL) && (state != 0))
	{
		ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "OTA-Enable already in progress");
		return;
	}
		
	if (si_is_server_ready(SI_SERVER_ID_OTA) == false)
	{
		UI_PRINT_LOG("ota_enable_state_machine: ERROR: OTA server not connected");
		state = 6;
	}
	else if ((!timed_out) || (state == 0))
	{
		state++;
	}
	
	while (rerun_state_machine)
	{
		UI_PRINT_LOG("OTA-Enable state %d", state);
		
		rerun_state_machine = false;
		switch (state)
		{
			case 1:
				if (si_register_idle_callback(ota_enable_state_machine, NOT_NULL) == 0)
				{
					if (si_is_waiting_for_confirmation() == false)
					{
						state++;
						rerun_state_machine = true;
					}
				}
				else
				{
					state = 0;
					UI_PRINT_LOG("WARNING: OTA-Enable state machine cannot be initialized at this time");
				}
				break;
			case 2:
				enable_ota_upgrade();
				break;
			case 3:
				ota_register("./sample_app_ota.cfg");
				current_ota_file_index = 0;
				state++;
				rerun_state_machine = true;
				break;
			case 4:
				register_single_file(current_ota_file_index);
				break;
			case 5:
				current_ota_file_index++;
				if (current_ota_file_index < upgrade_num_files)
				{
					state--;
				}
				else
				{
					state++;
				}
				rerun_state_machine = true;
				break;
			case 6:
				si_unregister_idle_callback();
				state = 0;
				break;
			default:
				break;
		}
	}
}

static void enable_ota_upgrade(void)
{
	OtaUpdateEnableReq pMsg = OTA_UPDATE_ENABLE_REQ__INIT;

	uint8_t len;
	pkt_buf_t * pkt = NULL;

	pMsg.mode = OTA_ENABLE_MODES__DOWNLOAD_ENABLE;

	len = ota_update_enable_req__get_packed_size(&pMsg);

	pkt = malloc(len + sizeof(pkt_buf_hdr_t)); 

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR;
		pkt->header.cmd_id = OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_REQ;

		ota_update_enable_req__pack(&pMsg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&update_enable_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("enable_ota_upgrade: Error: Could not send msg");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("enable_ota_upgrade: Error: Could not pack msg");
	}
}

static void update_enable_cnf(pkt_buf_t * pkt, void * cbarg)
{
	OtaUpdateEnableCnf *msg = NULL;

	if (pkt->header.cmd_id != OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF)
	{
		UI_PRINT_LOG("update_enable_cnf: Received bad header: Not OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF");
		return;
	}

	UI_PRINT_LOG("update_enable_cnf: Received OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF");

	msg = ota_update_enable_cnf__unpack(NULL, pkt->header.len, 
	pkt->packed_protobuf_packet);

	if (msg)
	{
		UI_PRINT_LOG("msg->status = %d", msg->status);

		/* */
		if (msg->status != true)
		{
			//ERROR 
			UI_PRINT_LOG("msg->status = %d", msg->status);
		}
	}
	else
	{
		UI_PRINT_LOG("update_enable_cnf: Error: Could not unpack msg");
	}
}


/* Init function for this engine */
void ota_register(char * fileName)
{
	int i;
	/* Initialize upgrade_file_status */
	upgrade_num_files = 0;

	for (i = 0; i <  MAX_UPGRADE_FILES; i++)
	{
		//upgrade_file_status[i].fileLoc = NULL;  
		upgrade_file_status[i].fileStatus = 0;
		upgrade_file_status[i].valid = false;
		upgrade_file_status[i].numDevices = 0;
	}

	/* Open file and read all contents
	Full-path to fileName, Number of devices, List of device ieee addresses
	*/
	/* Store in internal information structure until register call */
	parse_file_to_list(fileName);
}

/* Apply image to device at given address */
void upgrade_apply_image(zb_addr_t * addr)
{
	OtaUpdateApplyImageReq pMsg = OTA_UPDATE_APPLY_IMAGE_REQ__INIT;
	uint8_t len;
	AddressStruct address = ADDRESS_STRUCT__INIT;
	pkt_buf_t * pkt = NULL;

	if (addr->ieee_addr != 0)
	{
		address.addrmode = ADDRESS_MODE__UNICAST;
		address.has_ieeeaddr = true;
		address.ieeeaddr = addr->ieee_addr; 
	}
	else
	{
		address.addrmode = ADDRESS_MODE__BROADCAST;
		address.has_broadcaseaddr = true;
		address.broadcaseaddr = 0xFFFF;
	}

	pMsg.address = &address;

	len = ota_update_apply_image_req__get_packed_size(&pMsg);
	pkt = malloc(len + sizeof(pkt_buf_hdr_t)); 

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR;
		pkt->header.cmd_id = OTA_MGR_CMD_ID_T__OTA_UPDATE_APPLY_IMAGE_REQ;

		ota_update_apply_image_req__pack(&pMsg, pkt->packed_protobuf_packet);

		if (si_send_packet(pkt, 
			(confirmation_processing_cb_t)&upgrade_apply_image_cnf, NULL) != 0)
		{
			UI_PRINT_LOG("upgrade_apply_image: Error: Could not send msg");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("upgrade_apply_image: Error: Could not pack msg");
	}
}

void ota_process_download_finished_indication(pkt_buf_t * pkt)
{
	OtaUpdateDlFinishedInd *msg = NULL;

	if (pkt->header.cmd_id != OTA_MGR_CMD_ID_T__OTA_UPDATE_DL_FINISHED_IND)
	{
		return;
	}

	msg = ota_update_dl_finished_ind__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		if (msg->status == OTA_STATUS__OTA_SUCCESS)
		{
			UI_PRINT_LOG("ota_process_download_finished_indication: Status SUCCESS.");

			UI_PRINT_LOG("addr=%s", ui_make_string_OtaAddrStruct(msg->address));

			ui_refresh_display();
		}
		else
		{
			UI_PRINT_LOG("ota_process_download_finished_indication: FAILURE (%d)", msg->status);
		}

		ota_update_dl_finished_ind__free_unpacked(msg, NULL);
	}
	else
	{
		UI_PRINT_LOG("ota_process_download_finished_indication: Error Could not unpack msg");
	}
}

static void upgrade_apply_image_cnf(pkt_buf_t * pkt, void * cbarg)
{
	OtaZigbeeGenericCnf *msg = NULL;
	int index = (int )cbarg; 

	if (pkt->header.cmd_id != OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("upgrade_apply_image_cnf: Received OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF");

	msg = ota_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		UI_PRINT_LOG("msg->status = %d", msg->status);

		/* */
		if (msg->status == GENERIC_STATUS__SUCCESS)
		{
			upgrade_file_status[index -1].fileStatus = 2;
		}
		else
		{
			//ERROR 
			upgrade_file_status[index -1].fileStatus = -2;
		}

		ui_refresh_display();
		ota_zigbee_generic_cnf__free_unpacked(msg, NULL);	

	}
	else
	{
		UI_PRINT_LOG("upgrade_apply_image_cnf: Error: Could not unpack msg");
	}
}

static void upgrade_process_regis_cnf(pkt_buf_t * pkt, void * cbarg)
{
	OtaZigbeeGenericCnf *msg = NULL;
	int index = (int )cbarg; 

	if (pkt->header.cmd_id != OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF)
	{
		return;
	}

	UI_PRINT_LOG("upgrade_process_regis_cnf: Received IMAGE REGIS OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF for index %d", index);

	msg = ota_zigbee_generic_cnf__unpack(NULL, pkt->header.len, pkt->packed_protobuf_packet);

	if (msg)
	{
		UI_PRINT_LOG("msg->status = %d", msg->status);

		/* */
		if (msg->status == GENERIC_STATUS__SUCCESS)
		{
			UI_PRINT_LOG("OTA Server successfully registered file #%d", index);
			upgrade_file_status[index].fileStatus = 1;
		}
		else
		{
			//ERROR 
			UI_PRINT_LOG("ERROR: OTA Server could not register file #%d", index);
			upgrade_file_status[index].fileStatus = -1;
		}

		ui_refresh_display(); //???
		ota_zigbee_generic_cnf__free_unpacked(msg, NULL);	
	}
	else
	{
		UI_PRINT_LOG("upgrade_process_regis_cnf: Error: Could not unpack msg");
	}
}

static void parse_file_to_list (char * fileName)
{
	FILE * pfile = NULL;
	char temp[512];
	char *line_tok;
	uint8_t extAddr[8];
	int i, j;

	pfile = fopen(fileName, "rb");
	if (NULL == pfile)
	{
		return;
	}

	while (fgets(temp, 512, pfile) != NULL)
	{
		//printf("Read out %s\n", temp);

		strtok(temp, " ");

		strcpy(upgrade_file_status[upgrade_num_files].fileLoc, temp);
		line_tok = (char *)strtok(NULL, " "); 

		upgrade_file_status[upgrade_num_files].numDevices = atoi(line_tok);
		upgrade_file_status[upgrade_num_files].valid = true;

		for (i = 0; i < upgrade_file_status[upgrade_num_files].numDevices; i++)
		{
			line_tok = (char *)strtok(NULL, " "); 
			sscanf(line_tok, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
				&extAddr[7], &extAddr[6], &extAddr[5], &extAddr[4],
				&extAddr[3], &extAddr[2], &extAddr[1], &extAddr[0]);
			memcpy(&upgrade_file_status[upgrade_num_files].deviceList[i], &extAddr, 8);
		}

		upgrade_num_files++; 
	}

	UI_PRINT_LOG("Found %d file listings", upgrade_num_files);
	for (i = 0; i < upgrade_num_files; i++)
	{
		UI_PRINT_LOG("%d) Name %s", i, upgrade_file_status[i].fileLoc);
		for (j = 0; j < upgrade_file_status[i].numDevices; j++)
		{
			UI_PRINT_LOG("\tDevice %d: 0x%Lx",j, 
			upgrade_file_status[i].deviceList[j]);
		}
	}
}

static void register_single_file(int i)
{
	OtaUpdateImageRegisterationReq pMsg = 
	OTA_UPDATE_IMAGE_REGISTERATION_REQ__INIT;
	uint8_t len;
	pkt_buf_t * pkt = NULL;


	pMsg.imagepath = upgrade_file_status[i].fileLoc;
	pMsg.registerunregister = true;
	pMsg.executetiming = OTA_EXECUTE_TYPE__IMMEDIATE;
	pMsg.notification = OTA_NOTIFICATION_TYPE__UNICAST_NOT;
	pMsg.n_supporteddevicelist = upgrade_file_status[i].numDevices;
	pMsg.supporteddevicelist = upgrade_file_status[i].deviceList;
	pMsg.updatesupporteddevicelist = true;

	UI_PRINT_LOG("Registering %s for %d devices", 
	upgrade_file_status[i].fileLoc,
	upgrade_file_status[i].numDevices);

	len = ota_update_image_registeration_req__get_packed_size(&pMsg);
	pkt = malloc(len + sizeof(pkt_buf_hdr_t)); 

	if (pkt)
	{
		pkt->header.len = len;
		pkt->header.subsystem = ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR;
		pkt->header.cmd_id = OTA_MGR_CMD_ID_T__OTA_UPDATE_IMAGE_REGISTERATION_REQ; 

		ota_update_image_registeration_req__pack(&pMsg, 
		pkt->packed_protobuf_packet);
		UI_PRINT_LOG("Sending packet len %x subsystem %d cmdid %d", len, pkt->header.subsystem, pkt->header.cmd_id);

		if (si_send_packet(pkt, (confirmation_processing_cb_t)&upgrade_process_regis_cnf, (void *)i) != 0)
		{
			UI_PRINT_LOG("register_single_file: Error: Could not send msg");
		}

		free(pkt);
	}
	else
	{
		UI_PRINT_LOG("register_single_file: Error: Could not pack msg");
	}
}

