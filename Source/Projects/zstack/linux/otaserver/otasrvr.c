/******************************************************************************
 Filename:       otasrvr.c
 Revised:        $Date$
 Revision:       $Revision: 29217 $

 Description:    OTA Server Protobuf Interface


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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "otasrvr.pb-c.h"
#include "api_server.h"
#include "otasrvr.h"
#include "hal_rpc.h"
#include "utils.h"

#include "OtaServer.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
int appConnectionHandle = -1;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 zclOTA_SeqNo;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint8 zclOTA_Srv_GetActiveDownloads();

extern void zclOTA_PermitOta(OtaEnableModes permit);

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
/* Process new Image Registration request */
static void processImageRegReq(int connection, 
	OtaUpdateImageRegisterationReq* pMsg);

/* Process Update Enable mode change */
static void processUpdateEnable(int connection, OtaUpdateEnableReq * pMsg);

/* Process a request to apply a particular image  */
static void processApplyImage(int connection, OtaUpdateApplyImageReq * pMsg);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void processApplyImageCnf(int connection, bool status);

/*********************************************************************
 * @fn      otasrvrpbHandlePbCb
 *
 * @brief   Handles incoming OTA Server Protobuf messages from Linux Gateway.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to message structure
 *
 * @return  none
 */
void otasrvrHandlePbCb (int connection, uint8 subSys, uint8 cmdId, uint16 len, 
    uint8 *pData, uint8 type)
{
  if (type == SERVER_CONNECT) {
    if (appConnectionHandle == -1) appConnectionHandle = connection;
    return;
  }
  else if (type  == SERVER_DISCONNECT) {
    return;
  }

  if ((subSys & RPC_SUBSYSTEM_MASK) != ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR ) {
    return;
  }

  uiPrintfEx(trDEBUG, "\notasrvrHandlePbCb: subsystemID:%x, cmdId:%x\n", subSys,
    cmdId);


  switch (cmdId) {
    case OTA_MGR_CMD_ID_T__OTA_UPDATE_IMAGE_REGISTERATION_REQ:
      {
        /* Declare pointer to message */
        OtaUpdateImageRegisterationReq* pUpdateReq;

        /* Unpack pMsg->len, pMsg->Data */
        pUpdateReq = ota_update_image_registeration_req__unpack(NULL, len, 
		pData );

        if (pUpdateReq )
        {
          /* If Unpack successful, process message */
          processImageRegReq(connection, pUpdateReq);
          /* Free unpacked message */
          ota_update_image_registeration_req__free_unpacked(pUpdateReq, NULL);
        }
      }		
      break;

    case OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_REQ:
      {
        OtaUpdateEnableReq  *pUpdateEnable;
        pUpdateEnable =  ota_update_enable_req__unpack(NULL, len, pData);
        if (pUpdateEnable) 
        {
          processUpdateEnable(connection, pUpdateEnable);

          ota_update_enable_req__free_unpacked(pUpdateEnable, NULL);
        }
      }
      break;

    case OTA_MGR_CMD_ID_T__OTA_UPDATE_APPLY_IMAGE_REQ:
      {

        OtaUpdateApplyImageReq *pApplyImage;
            pApplyImage =  ota_update_apply_image_req__unpack(NULL, len, pData);
                
        if (pApplyImage) 
        {
          processApplyImage(connection, pApplyImage);

          ota_update_apply_image_req__free_unpacked(pApplyImage, NULL); 
        }
      }
      break;

    default:
            uiPrintfEx(trDEBUG, "\notasrvrHandlePbCb Recvd- Unknown: %d\n", cmdId );
            break;
  }
}

/*********************************************************************
 * @fn       	processImageRegReq
 *
 * @brief  	Process a request to register an image.	 
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to OtaUpdateReq msg. 
 *
 * @return  none
 */
static void processImageRegReq(int connection, 
	OtaUpdateImageRegisterationReq* pMsg)
{
  int len;
  uint8 *pBuf = NULL;
  //afAddrType_t dstAddr;
  uint8 osStatus;
  uint32_t executiondelay = -1;
  uint32_t executiontime = -1;

  /* Declare a variable of type response to above msg */
  OtaZigbeeGenericCnf irRsp = OTA_ZIGBEE_GENERIC_CNF__INIT;

  /* 1. Process the message pMsg */

  if (pMsg->registerunregister) {

    if (pMsg->has_executiondelay) executiondelay = pMsg->executiondelay;

    if (pMsg->has_executiontime) executiontime = pMsg->executiontime;	

    /* Call OtaServer_AddImage */
    osStatus = OtaServer_AddImage((char *)(pMsg->imagepath),pMsg->executetiming, 		executiondelay, executiontime, pMsg->updatesupporteddevicelist, 
		pMsg->n_supporteddevicelist, pMsg->supporteddevicelist, 
		pMsg->notification, connection);

  }
  else {
    /* Call OtaServer_DeleteImage */
    osStatus = OtaServer_DeleteImage((char *)(pMsg->imagepath));
  }

  if (osStatus == OTASERVER_SUCCESS) {
    irRsp.status = GENERIC_STATUS__SUCCESS;
  }
  else {
    irRsp.status = GENERIC_STATUS__FAILURE;
  }

  irRsp.has_sequencenumber = FALSE;
  len = ota_zigbee_generic_cnf__get_packed_size(&irRsp);
  pBuf = malloc ( len );
  if (pBuf) {
    
    ota_zigbee_generic_cnf__pack(&irRsp, pBuf);

    /* Construct a synch confirmation msg and send it using: */
    APIS_SendData(connection, ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR, FALSE, 
            OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF, len, pBuf);

    free(pBuf);
  }
}

/*********************************************************************
 * @fn       	processUpdateEnable
 *
 * @brief  	Process a request to change Update Enable mode. 
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to OtaUpdateEnableReq msg. 
 *
 * @return  none
 */
static void processUpdateEnable(int connection, OtaUpdateEnableReq * pMsg)
{
  int len;
  uint8 *pBuf;

  /* Declare a variable of type response to above msg */
  OtaUpdateEnableCnf upRsp = OTA_UPDATE_ENABLE_CNF__INIT;

  //Process the  permit flag
  zclOTA_PermitOta(pMsg->mode);

  upRsp.cmdid = OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF;

  //Success
  upRsp.status = 0;

  len = ota_update_enable_cnf__get_packed_size(&upRsp);
  pBuf = malloc ( len );
  if (pBuf) {
    
    ota_update_enable_cnf__pack(&upRsp, pBuf);

    /* Construct a synchronous confirmation message and send it using: */
    APIS_SendData(connection, ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR, FALSE, 
            OTA_MGR_CMD_ID_T__OTA_UPDATE_ENABLE_CNF, len, pBuf);

    free(pBuf);
  }
}

/*********************************************************************
 * @fn       	processApplyImage
 *
 * @brief  	Process a request to apply a particular image 
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to OtaUpdateApplyImageReq msg. 
 *
 * @return  none
 */
static void processApplyImage(int connection, OtaUpdateApplyImageReq * pMsg)
{

  zclOTA_UpgradeEndRspParams_t rspParms;
  afAddrType_t srcAddr, *pSrcAddr = &srcAddr;
  ZStatus_t status;
  OtaZigbeeGenericCnf irRsp = OTA_ZIGBEE_GENERIC_CNF__INIT;
  int len;
  uint8 *pBuf = NULL;

  rspParms.fileId.manufacturer = 0xFFFF;
  rspParms.fileId.type = 0xFFFF;
  rspParms.fileId.version = 0xFFFFFFFF;

  //Apply immediately 
  rspParms.currentTime = 0x0;
  rspParms.upgradeTime = 0x0;  

  if (pMsg->address->addrmode == ADDRESS_MODE__BROADCAST) {

    /* Need to send generic confirmation without sequence number */

    pSrcAddr->addrMode = afAddrBroadcast;
    pSrcAddr->addr.shortAddr = pMsg->address->broadcaseaddr;

  }
  else if (pMsg->address->addrmode == ADDRESS_MODE__GROUPCAST) {

    /* Need to send generic confirmation without sequence number */

    /* Convert broadcast or groupcast address to afAddr */
    pSrcAddr->addrMode = afAddrGroup;
    pSrcAddr->addr.shortAddr = pMsg->address->groupaddr;

  }
  else if (pMsg->address->addrmode == ADDRESS_MODE__UNICAST) {

    /* Send generic confirmation with sequence number */

    if (pMsg->address->has_ieeeaddr) {
      pSrcAddr->addrMode = afAddr64Bit;
      memcpy(pSrcAddr->addr.extAddr, &pMsg->address->ieeeaddr, Z_EXTADDR_LEN);
    }
    else {
    //GUNDEV Error      
    }
  }

  if (pMsg->address->has_endpointid) {
    pSrcAddr->endPoint = pMsg->address->endpointid; 
  }
  else pSrcAddr->endPoint = 0xFF;

  status = zclOTA_SendUpgradeEndRsp(pSrcAddr, &rspParms, ++zclOTA_SeqNo);


  if (status == ZSuccess) {
    irRsp.status = GENERIC_STATUS__SUCCESS;
  }
  else {
    irRsp.status = GENERIC_STATUS__FAILURE;
  }

  irRsp.has_sequencenumber = FALSE;
  len = ota_zigbee_generic_cnf__get_packed_size(&irRsp);
  pBuf = malloc ( len );
  if (pBuf) {
    
    ota_zigbee_generic_cnf__pack(&irRsp, pBuf);

    /* Construct a synch confirmation msg and send it using: */
    APIS_SendData(connection, ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR, FALSE, 
            OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF, len, pBuf);

    free(pBuf);
  }
}

void processApplyImageCnf(int connection, bool status)
{

  int len;
  uint8 *pBuf;
  OtaZigbeeGenericCnf imgCnf = OTA_ZIGBEE_GENERIC_CNF__INIT;

  imgCnf.has_sequencenumber = false;

  if (status == TRUE) {
    imgCnf.status = GENERIC_STATUS__SUCCESS;
  }
  else imgCnf.status = GENERIC_STATUS__FAILURE;

  //Check transaction id here and send respone appropriately

  len = ota_zigbee_generic_cnf__get_packed_size(&imgCnf);
  pBuf = malloc ( len );
  if (pBuf) {
      
    ota_zigbee_generic_cnf__pack(&imgCnf, pBuf);
  
    /* Construct a asynch response message and send it using: */
    APIS_SendData(connection, ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR, FALSE,
    OTA_MGR_CMD_ID_T__ZIGBEE_GENERIC_CNF, len, pBuf);

    free(pBuf);
  }
}
