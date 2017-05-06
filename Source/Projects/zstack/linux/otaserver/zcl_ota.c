/******************************************************************************
  Filename:       zcl_ota.c
  Revised:        $Date: 2014-03-12 19:44:01 -0700 (Wed, 12 Mar 2014) $
  Revision:       $Revision: 37654 $

  Description:    Zigbee Cluster Library - Over-the-Air Upgrade Cluster ( OTA )


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

/******************************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ota.h"

#if defined ( INTER_PAN )
  #include "stub_aps.h"
#endif

#if defined OTA_MMO_SIGN
#include "ota_signature.h"
#endif

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * LOCAL FUNCTIONS
 */


#if (defined OTA_SERVER) && (OTA_SERVER == TRUE)
/******************************************************************************
 * @fn      zclOTA_SendImageNotify
 *
 * @brief   Send an OTA Image Notify message.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendImageNotify( afAddrType_t *dstAddr,
                                  zclOTA_ImageNotifyParams_t *pParams,
                                  uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_IMAGE_NOTIFY];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->payloadType;
  *pBuf++ = pParams->queryJitter;
  if (pParams->payloadType >= NOTIFY_PAYLOAD_JITTER_MFG)
  {
    *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
    *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
  }
  if (pParams->payloadType >= NOTIFY_PAYLOAD_JITTER_MFG_TYPE)
  {
    *pBuf++ = LO_UINT16(pParams->fileId.type);
    *pBuf++ = HI_UINT16(pParams->fileId.type);
  }
  if (pParams->payloadType == NOTIFY_PAYLOAD_JITTER_MFG_TYPE_VERS)
  {
    pBuf = (uint8 *) zcl_buffer_uint32((uint8 *)pBuf, (uint8)(pParams->fileId.version));
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_IMAGE_NOTIFY, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendQueryNextImageRsp
 *
 * @brief   Send an OTA Query Next Image Response message.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendQueryNextImageRsp( afAddrType_t *dstAddr,
                                        zclOTA_QueryImageRspParams_t *pParams,
                                        uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_QUERY_NEXT_IMAGE_RSP];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->status;
  if (pParams->status == ZCL_STATUS_SUCCESS)
  {
    *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
    *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
    *pBuf++ = LO_UINT16(pParams->fileId.type);
    *pBuf++ = HI_UINT16(pParams->fileId.type);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, (pParams->fileId.version));
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, (pParams->imageSize));
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_QUERY_NEXT_IMAGE_RSP, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendImageBlockRsp
 *
 * @brief   Send an OTA Image Block Response mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendImageBlockRsp( afAddrType_t *dstAddr,
                                    zclOTA_ImageBlockRspParams_t *pParams,
                                    uint8 zclSeqNo)
{
  uint8 *buf;
  uint8 *pBuf;
  ZStatus_t status;
  uint8 len;

  if (pParams->status == ZCL_STATUS_SUCCESS)
  {
    len = PAYLOAD_MAX_LEN_IMAGE_BLOCK_RSP + pParams->rsp.success.dataSize;
  }
  else if (pParams->status == ZCL_STATUS_WAIT_FOR_DATA)
  {
    len = PAYLOAD_MIN_LEN_IMAGE_BLOCK_WAIT;
  }
  else
  {
    len = 1;
  }

  buf = zcl_mem_alloc( len );

  if ( buf == NULL )
  {
    return (ZMemError);
  }

  pBuf = buf;
  *pBuf++ = pParams->status;

  if (pParams->status == ZCL_STATUS_SUCCESS)
  {
    *pBuf++ = LO_UINT16(pParams->rsp.success.fileId.manufacturer);
    *pBuf++ = HI_UINT16(pParams->rsp.success.fileId.manufacturer);
    *pBuf++ = LO_UINT16(pParams->rsp.success.fileId.type);
    *pBuf++ = HI_UINT16(pParams->rsp.success.fileId.type);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->rsp.success.fileId.version);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->rsp.success.fileOffset);
    *pBuf++ = pParams->rsp.success.dataSize;
    zcl_memcpy(pBuf, pParams->rsp.success.pData, pParams->rsp.success.dataSize);
  }
  else if (pParams->status == ZCL_STATUS_WAIT_FOR_DATA)
  {
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->rsp.wait.currentTime);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->rsp.wait.requestTime);
    *pBuf++ = LO_UINT16(pParams->rsp.wait.blockReqDelay);
    *pBuf++ = HI_UINT16(pParams->rsp.wait.blockReqDelay);
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_IMAGE_BLOCK_RSP, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0,
                            zclSeqNo, len, buf );

  zcl_mem_free(buf);

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendUpgradeEndRsp
 *
 * @brief   Send an OTA Upgrade End Response mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendUpgradeEndRsp( afAddrType_t *dstAddr,
                                    zclOTA_UpgradeEndRspParams_t *pParams,
                                    uint8 zclSeqNo )
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_UPGRADE_END_RSP];
  uint8 *pBuf = buf;

  *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = LO_UINT16(pParams->fileId.type);
  *pBuf++ = HI_UINT16(pParams->fileId.type);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileId.version);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->currentTime);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->upgradeTime);

  /* Set Default Response to TRUE. Server needs to report status of cmd */
  
  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_UPGRADE_END_RSP, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0,
                            zclSeqNo, PAYLOAD_MAX_LEN_UPGRADE_END_RSP, buf );

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendQuerySpecificFileRsp
 *
 * @brief   Send an OTA Query Specific File Response mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendQuerySpecificFileRsp( afAddrType_t *dstAddr,
                                           zclOTA_QueryImageRspParams_t *pParams
                                           , uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_QUERY_SPECIFIC_FILE_RSP];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->status;
  if (pParams->status == ZCL_STATUS_SUCCESS)
  {
    *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
    *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
    *pBuf++ = LO_UINT16(pParams->fileId.type);
    *pBuf++ = HI_UINT16(pParams->fileId.type);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileId.version);
    pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->imageSize);
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_QUERY_SPECIFIC_FILE_RSP, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}


#endif // OTA_SERVER

#if (defined OTA_CLIENT) && (OTA_CLIENT == TRUE)
/******************************************************************************
 * @fn      zclOTA_SendQueryNextImageReq
 *
 * @brief   Send an OTA Query Next Image Request mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendQueryNextImageReq( afAddrType_t *dstAddr,
                                        zclOTA_QueryNextImageReqParams_t 
                                        *pParams, uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_QUERY_NEXT_IMAGE_REQ];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->fieldControl;
  *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = LO_UINT16(pParams->fileId.type);
  *pBuf++ = HI_UINT16(pParams->fileId.type);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileId.version);
  if (pParams->fieldControl == 1)
  {
    *pBuf++ = LO_UINT16(pParams->hardwareVersion);
    *pBuf++ = HI_UINT16(pParams->hardwareVersion);
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_QUERY_NEXT_IMAGE_REQ, TRUE,
                            ZCL_FRAME_CLIENT_SERVER_DIR, FALSE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendImageBlockReq
 *
 * @brief   Send an OTA Image Block Request mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendImageBlockReq( afAddrType_t *dstAddr,
                                    zclOTA_ImageBlockReqParams_t *pParams,
                                    uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_IMAGE_BLOCK_REQ];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->fieldControl;
  *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = LO_UINT16(pParams->fileId.type);
  *pBuf++ = HI_UINT16(pParams->fileId.type);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileId.version);
  pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileOffset);
  *pBuf++ = pParams->maxDataSize;

  if ( ( pParams->fieldControl & OTA_BLOCK_FC_NODES_IEEE_PRESENT ) != 0 )
  {
    zcl_cpyExtAddr(pBuf, pParams->nodeAddr);
    pBuf += Z_EXTADDR_LEN;
  }

  if ( ( pParams->fieldControl & OTA_BLOCK_FC_REQ_DELAY_PRESENT ) != 0 )
  {
    *pBuf++ = LO_UINT16(pParams->blockReqDelay);
    *pBuf++ = HI_UINT16(pParams->blockReqDelay);
  }

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_IMAGE_BLOCK_REQ, TRUE,
                            ZCL_FRAME_CLIENT_SERVER_DIR, FALSE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}

/******************************************************************************
 * @fn      zclOTA_SendUpgradeEndReq
 *
 * @brief   Send an OTA Upgrade End Request mesage.
 *
 * @param   dstAddr - where you want the message to go
 * @param   pParams - message parameters
 *
 * @return  ZStatus_t
 */
ZStatus_t zclOTA_SendUpgradeEndReq( afAddrType_t *dstAddr,
                                    zclOTA_UpgradeEndReqParams_t *pParams,
                                    uint8 zclSeqNo)
{
  ZStatus_t status;
  uint8 buf[PAYLOAD_MAX_LEN_UPGRADE_END_REQ];
  uint8 *pBuf = buf;

  *pBuf++ = pParams->status;
  *pBuf++ = LO_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = HI_UINT16(pParams->fileId.manufacturer);
  *pBuf++ = LO_UINT16(pParams->fileId.type);
  *pBuf++ = HI_UINT16(pParams->fileId.type);
   pBuf = (uint8 *)zcl_buffer_uint32(pBuf, pParams->fileId.version);

  status = zcl_SendCommand( ZCL_OTA_ENDPOINT, dstAddr, ZCL_CLUSTER_ID_OTA,
                            COMMAND_UPGRADE_END_REQ, TRUE,
                            ZCL_FRAME_CLIENT_SERVER_DIR, FALSE, 0,
                            zclSeqNo, (uint16) (pBuf - buf), buf );

  return status;
}

#endif // OTA_CLIENT
