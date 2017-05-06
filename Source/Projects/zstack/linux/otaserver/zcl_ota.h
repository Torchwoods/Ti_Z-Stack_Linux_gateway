/******************************************************************************
  Filename:       zcl_ota.h
  Revised:        $Date: 2014-03-12 19:44:01 -0700 (Wed, 12 Mar 2014) $
  Revision:       $Revision: 37654 $

  Description:    ZCL Over-the-Air Upgrade Cluster definitions.


  Copyright 2010-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/

#ifndef ZCL_OTA_H
#define ZCL_OTA_H

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "ota_common.h"

/******************************************************************************
 * CONSTANTS
 */
#define ZCL_SE_PROFILE_ID                             0x0109
#define ZCL_HA_PROFILE_ID                             0x0104

#define ZCL_SE_DEVICEID_PHYSICAL                      0x0507

// Simple descriptor values
#define ZCL_OTA_ENDPOINT                              14
#ifdef OTA_HA
#define ZCL_OTA_PROFILE_ID						ZCL_HA_PROFILE_ID
#define ZCL_OTA_DEVICEID                       0
#else
#define ZCL_OTA_PROFILE_ID                     ZCL_SE_PROFILE_ID
#define ZCL_OTA_DEVICEID                       ZCL_SE_DEVICEID_PHYSICAL
#endif
#define ZCL_OTA_DEVICE_VERSION                        0
#define ZCL_OTA_FLAGS                                 0

// OTA Device ID values

// OTA Header Version
#define OTA_HDR_VERSION                               0x0100

#define OTA_MAX_MTU                                   32

// OTA Header Field Control Bits
#define OTA_HDR_FC_SEC_CRED_PRESENT                   0x01
#define OTA_HDR_FC_DEV_SPEC_FILE                      0x02
#define OTA_HDR_FC_HW_VER_PRESENT                     0x04

// OTA Header Manufacturer ID 'Match All'
#define OTA_HDR_MFG_MATCH_ALL                         0xFFFF

// OTA Header Image Type
#define OTA_HDR_IMAGE_SEC_CRED                        0xFFC0
#define OTA_HDR_IMAGE_CONFIG                          0xFFC1
#define OTA_HDR_IMAGE_LOG                             0xFFC2
#define OTA_HDR_IMAGE_MATCH_ALL                       0xFFFF

// OTA Header File Version 'Match All'
#define OTA_HDR_FILE_VER_MATCH_ALL                    0xFFFFFFFF

// OTA ZigBee Stack Version
#define OTA_STACK_VER_2006                            0x0000
#define OTA_STACK_VER_2007                            0x0001
#define OTA_STACK_VER_PRO                             0x0002
#define OTA_STACK_VER_IP                              0x0003

// OTA Security Credential Version
#define OTA_SEC_CRED_VER_10                           0x00
#define OTA_SEC_CRED_VER_1X                           0x01
#define OTA_SEC_CRED_VER_20                           0x02

// OTA Attribute IDs
#define ATTRID_UPGRADE_SERVER_ID                      0x0000
#define ATTRID_FILE_OFFSET                            0x0001
#define ATTRID_CURRENT_FILE_VERSION                   0x0002
#define ATTRID_CURRENT_ZIGBEE_STACK_VERSION           0x0003
#define ATTRID_DOWNLOADED_FILE_VERSION                0x0004
#define ATTRID_DOWNLOADED_ZIGBEE_STACK_VERSION        0x0005
#define ATTRID_IMAGE_UPGRADE_STATUS                   0x0006
#define ATTRID_MANUFACTURER_ID                        0x0007  // UINT16, R, O
#define ATTRID_IMAGE_TYPE_ID                          0x0008  // UINT16, R, O
#define ATTRID_MINIMUM_BLOCK_REQ_DELAY                0x0009  // UINT16, R, O
#define ATTRID_IMAGE_STAMP                            0x000A  // UINT32, R, O

// OTA Upgrade Status
#define OTA_STATUS_NORMAL                             0x00
#define OTA_STATUS_IN_PROGRESS                        0x01
#define OTA_STATUS_COMPLETE                           0x02
#define OTA_STATUS_UPGRADE_WAIT                       0x03
#define OTA_STATUS_COUNTDOWN                          0x04
#define OTA_STATUS_WAIT_FOR_MORE                      0x05

// OTA Upgrade Time 'wait for upgrade'
#define OTA_UPGRADE_TIME_WAIT                         0xFFFFFFFF

// OTA Cluster Command Frames
#define COMMAND_IMAGE_NOTIFY                          0x00
#define COMMAND_QUERY_NEXT_IMAGE_REQ                  0x01
#define COMMAND_QUERY_NEXT_IMAGE_RSP                  0x02
#define COMMAND_IMAGE_BLOCK_REQ                       0x03
#define COMMAND_IMAGE_PAGE_REQ                        0x04
#define COMMAND_IMAGE_BLOCK_RSP                       0x05
#define COMMAND_UPGRADE_END_REQ                       0x06
#define COMMAND_UPGRADE_END_RSP                       0x07
#define COMMAND_QUERY_SPECIFIC_FILE_REQ               0x08
#define COMMAND_QUERY_SPECIFIC_FILE_RSP               0x09

// OTA Cluster Command Frame Payload Lengths
#define PAYLOAD_MAX_LEN_IMAGE_NOTIFY                  10
#define PAYLOAD_MAX_LEN_QUERY_NEXT_IMAGE_REQ          11
#define PAYLOAD_MAX_LEN_QUERY_NEXT_IMAGE_RSP          13
#define PAYLOAD_MAX_LEN_IMAGE_BLOCK_REQ               24
#define PAYLOAD_MAX_LEN_IMAGE_PAGE_REQ                26
#define PAYLOAD_MAX_LEN_IMAGE_BLOCK_RSP               14
#define PAYLOAD_MAX_LEN_UPGRADE_END_REQ               9
#define PAYLOAD_MAX_LEN_UPGRADE_END_RSP               16
#define PAYLOAD_MAX_LEN_QUERY_SPECIFIC_FILE_REQ       18
#define PAYLOAD_MAX_LEN_QUERY_SPECIFIC_FILE_RSP       13

#define PAYLOAD_MIN_LEN_IMAGE_NOTIFY                  2
#define PAYLOAD_MIN_LEN_QUERY_NEXT_IMAGE_REQ          9
#define PAYLOAD_MIN_LEN_QUERY_NEXT_IMAGE_RSP          1
#define PAYLOAD_MIN_LEN_IMAGE_BLOCK_REQ               14
#define PAYLOAD_MIN_LEN_IMAGE_PAGE_REQ                18
#define PAYLOAD_MIN_LEN_IMAGE_BLOCK_RSP               14
#define PAYLOAD_MIN_LEN_IMAGE_BLOCK_WAIT              11
#define PAYLOAD_MIN_LEN_UPGRADE_END_REQ               1
#define PAYLOAD_MIN_LEN_UPGRADE_END_RSP               16
#define PAYLOAD_MIN_LEN_QUERY_SPECIFIC_FILE_REQ       18
#define PAYLOAD_MIN_LEN_QUERY_SPECIFIC_FILE_RSP       1

// Image Block Request Field Control Bitmask
#define OTA_BLOCK_FC_GENERIC                          0x00
#define OTA_BLOCK_FC_NODES_IEEE_PRESENT               0x01
#define OTA_BLOCK_FC_REQ_DELAY_PRESENT                0x02

//  Image Notify Command Payload Type
#define NOTIFY_PAYLOAD_JITTER                         0x00
#define NOTIFY_PAYLOAD_JITTER_MFG                     0x01
#define NOTIFY_PAYLOAD_JITTER_MFG_TYPE                0x02
#define NOTIFY_PAYLOAD_JITTER_MFG_TYPE_VERS           0x03


/******************************************************************************
 * TYPEDEFS
 */
// Message Parameter Structures
typedef struct
{
  uint8 payloadType;
  uint8 queryJitter;
  zclOTA_FileID_t fileId;
} zclOTA_ImageNotifyParams_t;

typedef struct
{
  uint8 fieldControl;
  zclOTA_FileID_t fileId;
  uint16 hardwareVersion;
} zclOTA_QueryNextImageReqParams_t;

typedef struct
{
  uint8 status;
  zclOTA_FileID_t fileId;
  uint32 imageSize;
} zclOTA_QueryImageRspParams_t;

typedef struct
{
  uint8 fieldControl;
  zclOTA_FileID_t fileId;
  uint32 fileOffset;
  uint8 maxDataSize;
  uint8 nodeAddr[Z_EXTADDR_LEN];
  uint16 blockReqDelay;
} zclOTA_ImageBlockReqParams_t;

typedef struct
{
  uint8 fieldControl;
  zclOTA_FileID_t fileId;
  uint32 fileOffset;
  uint8 maxDataSize;
  uint16 pageSize;
  uint16 responseSpacing;
  uint8 nodeAddr[Z_EXTADDR_LEN];
} zclOTA_ImagePageReqParams_t;

typedef struct
{
  zclOTA_FileID_t fileId;
  uint32 fileOffset;
  uint8 dataSize;
  uint8 *pData;
} imageBlockRspSuccess_t;

typedef struct
{
  uint32 currentTime;
  uint32 requestTime;
  uint16 blockReqDelay;
} imageBlockRspWait_t;

typedef union
{
  imageBlockRspSuccess_t success;
  imageBlockRspWait_t wait;
} imageBlockRsp_t;

typedef struct
{
  uint8 status;
  imageBlockRsp_t rsp;
} zclOTA_ImageBlockRspParams_t;

typedef struct
{
  uint8 status;
  zclOTA_FileID_t fileId;
} zclOTA_UpgradeEndReqParams_t;

typedef struct
{
  zclOTA_FileID_t fileId;
  uint32 currentTime;
  uint32 upgradeTime;
} zclOTA_UpgradeEndRspParams_t;

typedef struct
{
  uint8 nodeAddr[Z_EXTADDR_LEN];
  zclOTA_FileID_t fileId;
  uint16 stackVersion;
} zclOTA_QuerySpecificFileReqParams_t;

typedef union
{
  zclOTA_QueryImageRspParams_t  queryNextImageRsp;
  zclOTA_ImageBlockRspParams_t  imageBlockRsp;
  zclOTA_UpgradeEndRspParams_t  upgradeEndRsp;
  zclOTA_QueryImageRspParams_t  querySpecificFileRsp;
} zclOTA_RspParams_t;

typedef struct
{
  osal_event_hdr_t hdr;
  uint8 ota_event;
} zclOTA_CallbackMsg_t;

/******************************************************************************
 * FUNCTIONS
 */


#if defined(OTA_SERVER) && (OTA_SERVER == TRUE)
/******************************************************************************
 * @fn      zclOTA_SendImageNotify
 *
 * @brief   Called by a server to send an Image Notify to an OTA client.
 *
 * @param   dstAddr - Short address of the client
 * @param   pParams - Parameters of the Image Notify message
 *
 * @return  ZStatus_t
 */
extern ZStatus_t zclOTA_SendImageNotify(afAddrType_t *dstAddr, zclOTA_ImageNotifyParams_t *pParams, uint8 zclSeqNo);

#if 0
extern ZStatus_t zclOTA_SendReadImageStatus( afAddrType_t *dstAddr, void *pParams, uint8 zclSeqNo); 
#endif

#endif

#if (defined OTA_CLIENT) && (OTA_CLIENT == TRUE)

extern ZStatus_t zclOTA_SendQueryNextImageReq( afAddrType_t *dstAddr, zclOTA_QueryNextImageReqParams_t *pParams , uint8 zclSeqNo);
extern ZStatus_t zclOTA_SendImageBlockReq( afAddrType_t *dstAddr, zclOTA_ImageBlockReqParams_t *pParams , uint8 zclSeqNo);
extern ZStatus_t zclOTA_SendUpgradeEndReq( afAddrType_t *dstAddr, zclOTA_UpgradeEndReqParams_t *pParams , uint8 zclSeqNo);

#endif // OTA_CLIENT

#if (defined OTA_SERVER) && (OTA_SERVER == TRUE)
extern ZStatus_t zclOTA_SendQueryNextImageRsp( afAddrType_t *dstAddr, zclOTA_QueryImageRspParams_t *pParams , uint8 zclSeqNo);
extern ZStatus_t zclOTA_SendImageBlockRsp( afAddrType_t *dstAddr, zclOTA_ImageBlockRspParams_t *pParams, uint8 zclSeqNo );
extern ZStatus_t zclOTA_SendUpgradeEndRsp( afAddrType_t *dstAddr, zclOTA_UpgradeEndRspParams_t *pParams, uint8 zclSeqNo );
ZStatus_t zclOTA_SendQuerySpecificFileRsp( afAddrType_t *dstAddr, zclOTA_QueryImageRspParams_t *pParams, uint8 zclSeqNo );
#endif // OTA_SERVER


#ifdef __cplusplus
}
#endif

#endif /* ZCL_OTA_H */
