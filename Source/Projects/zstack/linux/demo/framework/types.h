/*******************************************************************************
  Filename:       types.h
  Revised:        $Date$
  Revision:       $Revision$

  Description:   This file contains type definitions for the HA Gateway Demo.


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
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#ifndef TYPES_H
#define TYPES_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "stdint.h"
#include "stdbool.h"

/******************************************************************************
 * Constants
 *****************************************************************************/
#define MAX_ENDPOINTS		20
#define MAX_CLUSTERS_ON_EP	20
#define MAX_ATTRIBUTES		10
#define MAX_ATTRIBUTE_SIZE	4
#define MAX_PENDING_ATTRIBUTES 16
#define MAX_DEVICES_PER_FILE   10

#define ZIGBEE_NETWORK_STATE_UNAVAILABLE  0
#define ZIGBEE_NETWORK_STATE_INITIALIZING 1
#define ZIGBEE_NETWORK_STATE_READY        2

#define NOT_NULL	((void *)1)

/******************************************************************************
 * Constants - Clusters and their associated attributes
 *****************************************************************************/
#define ZCL_CLUSTER_ID_SE_SIMPLE_METERING		0x0702
#define ATTRID_SE_METERING_INSTANTANEOUS_DEMAND		0x0400
#define ATTRID_SE_METERING_FORMATTING_UNIT_OF_MEASURE 0x0300
#define ATTRID_SE_METERING_FORMATTING_MULTIPLIER 0x0301
#define ATTRID_SE_METERING_FORMATTING_DIVISOR 0x0302
#define ATTRID_SE_METERING_FORMATTING_SUMMATION_FORMATING 0x0303

#define ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT		0x0402
#define ATTRID_MS_TEMPERATURE_MEASURED_VALUE		0x0000

#define ZCL_CLUSTER_ID_GEN_ON_OFF		0x0006
#define ATTRID_ON_OFF		0x0000

#define ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL		0x0008
#define ATTRID_LEVEL_CURRENT_LEVEL                        0x0000
#define ATTRID_LEVEL_ON_OFF_TRANSITION_TIME               0x0010

#define ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL		0x0300
#define ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_HUE		0x0000
#define ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_SATURATION		0x0001

#define ATTRID_OCCUPANCY		0x0000

#define ATTRID_HUMDITY			0x0000	

#define ATTRID_TEMPERATURE		0x0000

/******************************************************************************
 * Types
 *****************************************************************************/
typedef struct
{
	uint16_t len;
	uint8_t subsystem;
	uint8_t cmd_id;
} pkt_buf_hdr_t;

typedef struct
{
	pkt_buf_hdr_t header;
	uint8_t packed_protobuf_packet[];
} pkt_buf_t;

typedef struct
{
	uint64_t ieee_addr;
	uint8_t endpoint;
	uint32_t groupaddr;
} zb_addr_t;

/* Attribute tuple */
typedef struct
{
	bool valid;
	uint16_t attr_id;
	uint8_t attr_type;
	uint8_t attr_val[MAX_ATTRIBUTE_SIZE];
} attribute_info_t;

/* Attribute information for a particular Cluster */
typedef struct
{
	uint16_t cluster_id;
	uint8_t	num_attributes;
	attribute_info_t attribute_list[MAX_ATTRIBUTES];
} cluster_info_t;

/* Cluster and Attribute information for a particular Endpoint */
typedef struct 
{
	uint8_t	endpoint_id;
	uint16_t profile_id;
	uint16_t device_id;
	uint8_t num_ip_clusters;
	cluster_info_t ip_cluster_list[MAX_CLUSTERS_ON_EP];
	uint8_t	num_op_clusters;
	cluster_info_t op_cluster_list[MAX_CLUSTERS_ON_EP];
} endpoint_info_t; 

typedef enum
{
	NONE = 0,
	BINDSELECT
} bind_info_t;

typedef enum
{
	BINDING_MODE_BIND = 0,
	BINDING_MODE_UNBIND
} binding_mode_t;


/* Display-related information for a Device in the network */
typedef struct
{
	endpoint_info_t ep_list[MAX_ENDPOINTS];
	uint64_t ieee_addr;
	uint16_t nwk_addr;
	uint16_t manufacturer_id;
	bool valid;
	bool selected;
	bool selected_as_bind_destination;
	uint8_t device_status;
	uint8_t num_endpoints;
	uint8_t selected_endpoint_index;
} device_info_t;

/* Information for pending attribute read commands */
typedef struct
{
	bool valid;
	uint16_t sequence_num;
	uint64_t ieee_addr;
	uint8_t endpoint_id;
	uint16_t cluster_id;
	uint8_t  num_attributes;
	uint32_t attr_id[MAX_ATTRIBUTES];
	uint8_t  timer_val;
} pending_attribute_info_t;

/* Network Info to be displayed */
typedef struct {
	int state;
	uint32_t nwk_channel;
	uint32_t pan_id;
	uint64_t ext_pan_id;
	uint8_t permit_remaining_time;
	uint8_t num_pending_attribs;
} network_info_t;

typedef struct {
  char fileLoc[512];
  int numDevices;
  uint64_t deviceList[MAX_DEVICES_PER_FILE];
  bool valid;
  int fileStatus; // 0 - init, 1 - registered, 2- downloaded, 3 - abort
		  //-1 - regis. error
	          //??-2 - apply image error.
} upgrade_info_t; 

#endif /* TYPES_H */
