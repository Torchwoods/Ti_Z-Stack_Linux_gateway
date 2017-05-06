/**************************************************************************************************
 Filename:       znp_misc.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    A place to put porting ZNP variables and functions.


 Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

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
 **************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ZComDef.h"
#include "AF.h"
#include "ZDObject.h"
#include "ZDApp.h"
#include "MT.h"

#include "api_client.h"
#include "zstackpb.h"
#include "trace.h"

/*********************************************************************
 * Constants
 */

/*********************************************************************
 * Typedefs
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Place to put the extended address
uint8 extAddr[8] = { 0 };
uint8 has_extAddr = FALSE;

// ZGlobals variables - ZStackServerZNP doesn't have the ZGlobals module
uint8 zgPreConfigKeys = FALSE;
uint8 zgSecurityMode = ZG_SECURITY_MODE;
uint8 zgUseDefaultTCLK;
uint16 zgPollRate = POLL_RATE;
uint16 zgQueuedPollRate = QUEUED_POLL_RATE;
uint16 zgResponsePollRate = RESPONSE_POLL_RATE;
uint16 zgRejoinPollRate = REJOIN_POLL_RATE;
uint16 zgApscAckWaitDurationPolled = APSC_ACK_WAIT_DURATION_POLLED;
uint16 zgApsDefaultMaxBindingTime = APS_DEFAULT_MAXBINDING_TIME;
uint16 zgConfigPANID = ZDAPP_CONFIG_PAN_ID;
uint8 zgMaxPollFailureRetries = MAX_POLL_FAILURE_RETRIES;
uint8 zgIndirectMsgTimeout = NWK_INDIRECT_MSG_TIMEOUT;
uint8 zgMaxBcastRetires = MAX_BCAST_RETRIES;
uint8 zgPassiveAckTimeout = PASSIVE_ACK_TIMEOUT;
uint8 zgBcastDeliveryTime = BCAST_DELIVERY_TIME;
uint8 zgRouteExpiryTime = ROUTE_EXPIRY_TIME;
uint32 zgDefaultChannelList = DEFAULT_CHANLIST;
uint8 zgApsNonMemberRadius = APS_DEFAULT_NONMEMBER_RADIUS;
uint8 zgApsUseExtendedPANID[Z_EXTADDR_LEN] = {00,00,00,00,00,00,00,00};
uint8 zgApscMaxFrameRetries = APSC_MAX_FRAME_RETRIES;
uint8 zgConcentratorEnable = CONCENTRATOR_ENABLE;
uint8 zgConcentratorDiscoveryTime = CONCENTRATOR_DISCOVERY_TIME;

extern apicHandle_t zmainClientHandle; // ZStack API client handle
#define ZNP_API_CLIENT  zmainClientHandle

/*********************************************************************
 * External and Callback functions in zstackpb.c
 *********************************************************************/
extern void *zdoConcentratorIndCB( void *pStr );
extern void *zdoSrcRtgCB( void *pStr );
extern void *zdoJoinCnfCB( void *pStr );
extern void *zdoLeaveIndCB( void *pStr );
extern void sendDeviceAnnounce( uint16 srcAddr, ZDO_DeviceAnnce_t *pDevAnn );
extern void sendSysJammerInd( uint8 jammerInd );
extern void zstackpbReestablishEPs( void );
extern void znpInit( uint8 triggerStartEvent );

extern void zdoExtNwkInfo( nwkIB_t *pNib );
extern void *sendTcDeviceInd( void *params );
extern void *sendDevPermitJoinInd( void *params );

/*********************************************************************
 * local prototypes
 *********************************************************************/
static void processZDOLeaveInd( uint16 len, uint8 *pMsg );
static void processZDODeviceAnnounceInd( uint16 len, uint8 *pMsg );
static void processZDOTCDeviceInd( uint16 len, uint8 *pMsg );
static void processZDOPermitJoinInd( uint16 len, uint8 *pMsg );
static void processZDOJoinConfirmInd( uint16 len, uint8 *pMsg );
static void processZDOSourceRouteInd( uint16 len, uint8 *pMsg );
static void processZDOConcentratorInd( uint16 len, uint8 *pMsg );
static void processAFIncomingMsgInd( uint8 cmdID, uint16 len, uint8 *pMsg );
static void processAFDataConfirmInd( uint16 len, uint8 *pMsg );
static void processZDOStateChangeInd( uint8 status );
static void processZDOMsgCB( uint16 len, uint8 *pMsg );
static uint8 sendNPIExpectDefaultStatusSYS( int cmdID, int len, uint8 *pData );

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/**************************************************************************************************
 *
 * @fn          sendAPICZNP
 *
 * @brief       Send a request message with no response expected
 *
 * @param       subSys - messages sub-system ID
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      none
 *
 **************************************************************************************************/
void sendAPICZNP( int subSys, int cmdID, int len, uint8 *pData )
{
  uiPrintfEx(trINFO, "znp_misc sendAPICZNP: subSys:%x, cmdID:%x, len:%d\n", subSys, cmdID, len );

  // send serialized request to API Client synchronously
  apicSendAsynchData( ZNP_API_CLIENT, subSys, cmdID, len, pData );
}

/**************************************************************************************************
 *
 * @fn          sendNPIExpectDefaultStatusZNP
 *
 * @brief       Send a request message and expects the normal "default" response
 *
 * @param       subSys - messages sub-system ID
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      synchronous return status
 *
 **************************************************************************************************/
uint8 sendNPIExpectDefaultStatusZNP( int subSys, int cmdID, int len,
                                     uint8 *pData )
{
  uint8 *pRsp, rspcmdid;
  uint16 rsplen;
  uint8 status = 0;

  uiPrintfEx(trINFO, "znp_misc sendNPIExpectDefaultStatusZNP: subSys:%x, cmdID:%x, len:%d\n",
      subSys, cmdID, len );

  // send serialized request to API Client synchronously with expected response
  pRsp = apicSendSynchData( ZNP_API_CLIENT, subSys, cmdID, len, pData, NULL,
      &rspcmdid, &rsplen );
  if ( pRsp )
  {

    if ( (cmdID == rspcmdid) && (rsplen > 0) )
    {
      // Process the immediate response
      status = pRsp[0];
      if ( status != 0 )
      {
        uiPrintfEx(trDEBUG, "--> status failed: cmdID:%x, status: %d\n",
            cmdID, status );
      }
    }
    apicFreeSynchData( pRsp );
  }

  // return the status
  return (status);
}

/**************************************************************************************************
 *
 * @fn          sendNPIExpectDefaultStatusSYS
 *
 * @brief       Send a request message and expect the normal "default" response (MacDefaultRsp)
 *
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      synchronous return status
 *
 **************************************************************************************************/
static uint8 sendNPIExpectDefaultStatusSYS( int cmdID, int len, uint8 *pData )
{
  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_SYS, cmdID, len, pData ));
}

/**************************************************************************************************
 *
 * @fn          handleAsyncMsgs
 *
 * @brief       Handles incoming API Client messages (from ZNP)
 *
 * @param       handle - TCP Client connection handle
 * @param       subSys - incoming message subsystem ID
 * @param       cmdID - incoming message command ID
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
void handleAsyncMsgs( apicHandle_t handle, uint8 subSys, uint8 cmdID,
                      uint16 len, uint8 *pMsg )
{
  subSys &= MT_RPC_SUBSYSTEM_MASK;

  if ( subSys == MT_RPC_SYS_ZDO )
  {
    switch ( cmdID )
    {
      case MT_ZDO_MSG_CB_INCOMING:
        processZDOMsgCB( len, pMsg );
        break;

      case MT_ZDO_STATE_CHANGE_IND:
        processZDOStateChangeInd( *pMsg );
        break;

      case MT_ZDO_CONCENTRATOR_IND_CB:
        processZDOConcentratorInd( len, pMsg );
        break;

      case MT_ZDO_SRC_RTG_IND:
        processZDOSourceRouteInd( len, pMsg );
        break;

      case MT_ZDO_JOIN_CNF:
        processZDOJoinConfirmInd( len, pMsg );
        break;

      case MT_ZDO_LEAVE_IND:
        processZDOLeaveInd( len, pMsg );
        break;

      case MT_ZDO_BEACON_NOTIFY_IND:
      case MT_ZDO_NWK_DISCOVERY_CNF:
        // These cause problems
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: not implemented cmdID:%x\n", cmdID );
        break;

      case MT_ZDO_END_DEVICE_ANNCE_IND:
        processZDODeviceAnnounceInd( len, pMsg );
        break;

      case MT_ZDO_TC_DEVICE_IND:
        processZDOTCDeviceInd( len, pMsg );
        break;

      case MT_ZDO_PERMIT_JOIN_IND:
        processZDOPermitJoinInd( len, pMsg );
        break;

      case MT_ZDO_MATCH_DESC_RSP_SENT:
      case MT_ZDO_STATUS_ERROR_RSP:
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Un-handled cmdID:%x\n", cmdID );
        break;

      default:
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Unrecognized cmdID:%x\n", cmdID );
        break;
    }
  }
  else if ( subSys == MT_RPC_SYS_AF )
  {
    switch ( cmdID )
    {
      case MT_AF_DATA_CONFIRM:
        processAFDataConfirmInd( len, pMsg );
        break;

      case MT_AF_INCOMING_MSG_EXT:
      case MT_AF_INCOMING_MSG:
        processAFIncomingMsgInd( cmdID, len, pMsg );
        break;

      case MT_AF_REFLECT_ERROR:
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Un-handled cmdID:%x\n", cmdID );
        break;

      default:
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Unrecognized cmdID:%x\n", cmdID );
        break;
    }
  }
  else if ( subSys == MT_RPC_SYS_SYS )
  {
    switch ( cmdID )
    {
      case MT_SYS_RESET_IND:
        {
          // ZNP Reset, so it lost the MSG callbacks, and EP descriptors
          uiPrintfEx(trINFO, "znp_misc handleAsyncMsgs: Reset Indication\n" );

          // Register for ZDO Rsp messages
          ZDO_RegisterForZDOMsg( zspbTaskID, ZDO_ALL_MSGS_CLUSTERID );

          // Restore config information
          znpInit( TRUE );

          // Re-establish the EP descriptors
          zstackpbReestablishEPs();
        }
        break;

      case MT_SYS_JAMMER_IND:
        sendSysJammerInd( *pMsg );
        break;

      default:
        uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Unrecognized cmdID:%x\n", cmdID );
        break;
    }
  }
  else
  {
    uiPrintfEx(trDEBUG, "znp_misc handleAsyncMsgs: Unrecognized subSys: %x, cmdID:%x\n", subSys, cmdID );
  }
}

/**************************************************************************************************
 *
 * @fn          processZDOLeaveInd
 *
 * @brief       processes incoming ZNP ZDO Leave Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOLeaveInd( uint16 len, uint8 *pMsg )
{
  NLME_LeaveInd_t leaveInd;

  leaveInd.srcAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  memcpy( leaveInd.extAddr, pMsg, 8 );
  pMsg += 8;

  leaveInd.request = *pMsg++;
  leaveInd.removeChildren = *pMsg++;
  leaveInd.rejoin = *pMsg;

  (void) zdoLeaveIndCB( (void *) &leaveInd );
}

/**************************************************************************************************
 *
 * @fn          processZDODeviceAnnounceInd
 *
 * @brief       processes incoming ZNP ZDO Device Announce Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDODeviceAnnounceInd( uint16 len, uint8 *pMsg )
{
  ZDO_DeviceAnnce_t devAnnounce;
  uint16 srcAddr;

  srcAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  devAnnounce.nwkAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  memcpy( devAnnounce.extAddr, pMsg, 8 );
  pMsg += 8;

  devAnnounce.capabilities = *pMsg++;

  sendDeviceAnnounce( srcAddr, &devAnnounce );
}

/**************************************************************************************************
 *
 * @fn          processZDOTCDeviceInd
 *
 * @brief       processes incoming ZNP TC Device Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOTCDeviceInd( uint16 len, uint8 *pMsg )
{
  ZDO_TC_Device_t dev;

  dev.nwkAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  memcpy( dev.extAddr, pMsg, 8 );
  pMsg += 8;

  dev.parentAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  sendTcDeviceInd( (void *) &dev );
}

/**************************************************************************************************
 *
 * @fn          processZDOPermitJoinInd
 *
 * @brief       processes incoming ZNP TC Device Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOPermitJoinInd( uint16 len, uint8 *pMsg )
{
  sendDevPermitJoinInd( (void *) pMsg );
}

/**************************************************************************************************
 *
 * @fn          processZDOJoinConfirmInd
 *
 * @brief       processes incoming ZNP ZDO Join Confirm Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOJoinConfirmInd( uint16 len, uint8 *pMsg )
{
  zdoJoinCnf_t joinCnf;

  joinCnf.status = *pMsg++;
  joinCnf.deviceAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;
  joinCnf.parentAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  (void) zdoJoinCnfCB( (void *) &joinCnf );
}

/**************************************************************************************************
 *
 * @fn          processZDOSourceRouteInd
 *
 * @brief       processes incoming ZNP ZDO Source Route Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOSourceRouteInd( uint16 len, uint8 *pMsg )
{
  zdoSrcRtg_t srcRtg;

  memset( &srcRtg, 0, sizeof(zdoSrcRtg_t) );

  srcRtg.srcAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  srcRtg.relayCnt = *pMsg++;

  if ( srcRtg.relayCnt )
  {
    srcRtg.pRelayList = (uint16 *) malloc( sizeof(uint16) * srcRtg.relayCnt );
    if ( srcRtg.pRelayList )
    {
      int x;

      for ( x = 0; x < srcRtg.relayCnt; x++ )
      {
        srcRtg.pRelayList[x] = BUILD_UINT16( pMsg[0], pMsg[1] );
        pMsg += 2;
      }
    }
  }

  (void) zdoSrcRtgCB( (void *) &srcRtg );

  if ( srcRtg.pRelayList )
  {
    free( srcRtg.pRelayList );
  }

}

/**************************************************************************************************
 *
 * @fn          processZDOConcentratorInd
 *
 * @brief       processes incoming ZNP ZDO Concentrator Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOConcentratorInd( uint16 len, uint8 *pMsg )
{
  zdoConcentratorInd_t concInd;

  concInd.nwkAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
  pMsg += 2;

  concInd.extAddr = pMsg;
  pMsg += 8;

  concInd.pktCost = *pMsg;

  (void) zdoConcentratorIndCB( (void *) &concInd );
}

/**************************************************************************************************
 *
 * @fn          processAFIncomingMsgInd
 *
 * @brief       processes incoming ZNP ZDO Concentrator Indication
 *
 * @param       cmdID - command ID, either MT_AF_INCOMING_MSG_EXT or MT_AF_INCOMING_MSG
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processAFIncomingMsgInd( uint8 cmdID, uint16 len, uint8 *pMsg )
{
  afIncomingMSGPacket_t *pBuf;
  uint16 asduLen;

  if ( cmdID == MT_AF_INCOMING_MSG_EXT )
  {
    asduLen = len - 27;
  }
  else
  {
    asduLen = len - 17;
  }
  pBuf = (afIncomingMSGPacket_t *) osal_msg_allocate(
      (sizeof(afIncomingMSGPacket_t) + 4) + asduLen );
  if ( pBuf )
  {
    pBuf->hdr.event = AF_INCOMING_MSG_CMD;
    pBuf->hdr.status = 0;

    pBuf->groupId = BUILD_UINT16( pMsg[0], pMsg[1] );
    pMsg += 2;

    pBuf->clusterId = BUILD_UINT16( pMsg[0], pMsg[1] );
    pMsg += 2;

    if ( cmdID == MT_AF_INCOMING_MSG_EXT )
    {
      pBuf->srcAddr.addrMode = *pMsg++;
      if ( pBuf->srcAddr.addrMode == afAddr64Bit )
      {
        memcpy( pBuf->srcAddr.addr.extAddr, pMsg, 8 );
      }
      else
      {
        pBuf->srcAddr.addr.shortAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
      }
      pMsg += 8;

      pBuf->srcAddr.endPoint = *pMsg++;
      pBuf->srcAddr.panId = BUILD_UINT16( pMsg[0], pMsg[1] );
      pMsg += 2;
    }
    else
    {
      pBuf->srcAddr.addrMode = afAddr16Bit;
      pBuf->srcAddr.addr.shortAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
      pMsg += 2;
      pBuf->srcAddr.endPoint = *pMsg++;
    }

    pBuf->endPoint = *pMsg++;
    pBuf->wasBroadcast = *pMsg++;
    pBuf->LinkQuality = *pMsg++;
    pBuf->SecurityUse = *pMsg++;

    pBuf->timestamp = BUILD_UINT32( pMsg[0], pMsg[1], pMsg[2], pMsg[3] );
    pMsg += 4;

    if ( cmdID == MT_AF_INCOMING_MSG_EXT )
    {
      pMsg++; // TODO - workaround Z-Tool shortcoming; should be: = pMsg->cmd.TransSeqNumber;
      pBuf->cmd.DataLength = BUILD_UINT16( pMsg[0], pMsg[1] );
      pMsg += 2;
    }
    else
    {
      pBuf->cmd.TransSeqNumber = *pMsg++;
      pBuf->cmd.DataLength = *pMsg++;
    }

    if ( cmdID == MT_AF_INCOMING_MSG_EXT )
    {
      // This comes in in multiple messages, TBD
    }
    else
    {
      uint32 leftover;
      uint8 *pPtr = (uint8 *) (pBuf + 1);

     // Adjust pointers
      leftover = (uint32)pPtr % 4;
      pPtr += leftover;
      pBuf->cmd.Data = (uint8 *)pPtr;
      memcpy( pBuf->cmd.Data, pMsg, pBuf->cmd.DataLength )    ;
    }

    if ( osal_msg_send( zspbTaskID, (uint8 *)pBuf ) != SUCCESS )
    {
      // Error, message wasn't sent
      osal_msg_deallocate( (uint8 *)pBuf );
    }
  }
}

/**************************************************************************************************
 *
 * @fn          processAFDataConfirmInd
 *
 * @brief       processes incoming ZNP AF Data Confirm Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processAFDataConfirmInd( uint16 len, uint8 *pMsg )
{
  afDataConfirm_t *pBuf;

  pBuf = (afDataConfirm_t *) osal_msg_allocate( sizeof(afDataConfirm_t) );
  if ( pBuf )
  {
    pBuf->hdr.event = AF_DATA_CONFIRM_CMD;
    pBuf->hdr.status = *pMsg++;
    pBuf->endpoint = *pMsg++;
    pBuf->transID = *pMsg;

    if ( osal_msg_send( zspbTaskID, (uint8 *) pBuf ) != SUCCESS )
    {
      // Error, message wasn't sent
      osal_msg_deallocate( (uint8 *) pBuf );
    }
  }
}

/**************************************************************************************************
 *
 * @fn          processZDOStateChangeInd
 *
 * @brief       processes incoming ZNP ZDO State Change Indication
 *
 * @param       status - new state
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOStateChangeInd( uint8 status )
{
  osal_event_hdr_t *pBuf;

  pBuf = (osal_event_hdr_t *) osal_msg_allocate( sizeof(osal_event_hdr_t) );
  if ( pBuf )
  {
    pBuf->event = ZDO_STATE_CHANGE;
    pBuf->status = status;
    if ( osal_msg_send( zspbTaskID, (uint8 *) pBuf ) != SUCCESS )
    {
      // Error, message wasn't sent
      osal_msg_deallocate( (uint8 *) pBuf );
    }
  }
}

/**************************************************************************************************
 *
 * @fn          processZDOMsgCB
 *
 * @brief       processes incoming ZNP ZDO Response Message Callback Indication
 *
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 **************************************************************************************************/
static void processZDOMsgCB( uint16 len, uint8 *pMsg )
{
  zdoIncomingMsg_t *pBuf;
  uint16 asduLen = len - 9;

  pBuf = (zdoIncomingMsg_t *) osal_msg_allocate(
      sizeof(zdoIncomingMsg_t) + asduLen );
  if ( pBuf )
  {
    pBuf->srcAddr.addrMode = Addr16Bit;
    pBuf->srcAddr.addr.shortAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
    pMsg += 2;

    pBuf->wasBroadcast = *pMsg++;

    pBuf->clusterID = BUILD_UINT16( pMsg[0], pMsg[1] );
    pMsg += 2;

    pBuf->SecurityUse = *pMsg++;
    pBuf->TransSeq = *pMsg++;

    pBuf->macDestAddr = BUILD_UINT16( pMsg[0], pMsg[1] );
    pMsg += 2;

    pBuf->asdu = (byte*) (((byte*) pBuf) + sizeof(zdoIncomingMsg_t));

    pBuf->asduLen = asduLen;
    memcpy( pBuf->asdu, pMsg, asduLen );

    pBuf->hdr.event = ZDO_CB_MSG;
    if ( osal_msg_send( zspbTaskID, (uint8 *) pBuf ) != SUCCESS )
    {
      // Error, message wasn't sent
      osal_msg_deallocate( (uint8 *) pBuf );
    }
  }
}

/**************************************************************************************************
 * @fn      osal_nv_read
 *
 * @brief   Converts function call to ZNP OSAL NV Read Message
 *
 * @param   id  - Valid NV item Id.
 * @param   ndx - Index offset into item
 * @param   len - Length of data to read.
 * @param  *buf - Data is read into this buffer.
 *
 * @return  SUCCESS if NV data was copied to the parameter 'buf'.
 *          Otherwise, NV_OPER_FAILED for failure.
 *
 **************************************************************************************************/
uint8 osal_nv_read( uint16 id, uint16 ndx, uint16 len, void *buf )
{
  uint8 *pRsp;
  uint8 pData[3];
  uint8 status = 0;

  // serialize the request
  pData[0] = LO_UINT16( id );
  pData[1] = HI_UINT16( id );
  pData[2] = ndx;

  uiPrintfEx(trINFO, "znp_misc osal_nv_read: id:%x, len:%d\n", id, len );

  // send serialized request to NPI synchronously
  pRsp = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_SYS, MT_SYS_OSAL_NV_READ,
      3, pData, NULL, NULL, NULL );

  if ( pRsp )
  {
    // Process the immediate response
    status = pRsp[0];
    if ( status == ZSuccess )
    {
      // Copy the NV value
      memcpy( buf, &pRsp[2], pRsp[1] );
    }
    else
    {
      uiPrintfEx(trDEBUG, "--> NV Read Failed: %d\n", status );
    }
    apicFreeSynchData( pRsp );
  }

  // return the status
  return (status);
}

/**************************************************************************************************
 * @fn      osal_nv_write
 *
 * @brief   Converts function call to ZNP OSAL NV Write Message
 *
 * @param   id  - Valid NV item Id.
 * @param   ndx - Index offset into item
 * @param   len - Length of data to write.
 * @param  *buf - Data to write.
 *
 * @return  SUCCESS if successful, NV_ITEM_UNINIT if item did not
 *          exist in NV and offset is non-zero, NV_OPER_FAILED if failure.
 *
 **************************************************************************************************/
uint8 osal_nv_write( uint16 id, uint16 ndx, uint16 len, void *buf )
{
  ZStatus_t status;
  uint8 *pBuf;

  pBuf = malloc( len + 4 );
  if ( pBuf )
  {
    uint8 *pPtr = pBuf;
    *pPtr++ = LO_UINT16( id );
    *pPtr++ = HI_UINT16( id );
    *pPtr++ = ndx;
    *pPtr++ = len;
    memcpy( pPtr, buf, len );

    status = sendNPIExpectDefaultStatusSYS( MT_SYS_OSAL_NV_WRITE, (len + 4),
        pBuf );

    if ( (status == SUCCESS) && (id == ZCD_NV_EXTADDR) )
    {
      memcpy( extAddr, buf, 8 );
      has_extAddr = TRUE;
    }

    free( pBuf );
  }
  else
  {
    status = ZMemError;
  }

  return (status);
}

/**************************************************************************************************
 * @fn          NLME_GetExtAddr
 *
 * @brief       This function will return a pointer to the device's
 *              IEEE 64 bit address.
 *
 * @param       none
 *
 * @return      pointer to 64 bit address
 **************************************************************************************************/
uint8 *NLME_GetExtAddr( void )
{
  if ( has_extAddr == FALSE )
  {
    if ( osal_nv_read( ZCD_NV_EXTADDR, 0, 8, extAddr ) == SUCCESS )
    {
      has_extAddr = TRUE;
    }
  }

  return (extAddr);
}

/*********************************************************************
 * @fn          NLME_GetShortAddr
 *
 * @brief       Return this device's 16-bit short address.
 *
 * @param       none
 *
 * @return      Network short address of this device.
 */
uint16 NLME_GetShortAddr( void )
{
  nwkIB_t NIB = {0};

  zdoExtNwkInfo( &NIB );

  return (NIB.nwkDevAddress);
}
/**************************************************************************************************
 * @fn          znpSetTxPower
 *
 * @brief       This function Sets the Tx Power in a ZNP.
 *
 * @param       txPwr - new tx Power
 *
 * @return      status
 **************************************************************************************************/
uint8 znpSetTxPower( uint8 txPwr )
{
  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_SYS, MT_SYS_SET_TX_POWER, 1,
      &txPwr ));

}

/**************************************************************************************************
 * @fn          znpReset
 *
 * @brief       This function sends a reset message to the ZNP.
 *
 * @param       mode - type of reset: 0 - hard, 1 - soft
 * @param       nvReset - TRUE to bypass the NV network state on reset
 * @param       shutdown - TRUE to shutdown the ZNP, the ZNP will require a reset to restart
 *
 * @return      status
 **************************************************************************************************/
void znpReset( uint8 mode, uint8 nvReset, uint8 shutdown )
{
  uint8 parameter;

  uiPrintfEx(trINFO, "znp_misc znpReset: mode:%d, nvReset:%d, shutdown:%d\n", mode, nvReset, shutdown );

  if ( nvReset )
  {
    uint8 nvStartup = ZCD_STARTOPT_DEFAULT_NETWORK_STATE;
    osal_nv_write( ZCD_NV_STARTUP_OPTION, 0, 1, &nvStartup );
  }

  if ( shutdown )
  {
    parameter = MT_SYS_RESET_SHUTDOWN; // Shutdown value
  }
  else
  {
    parameter = mode;
  }

  uiPrintfEx(trINFO, "znp_misc znpReset: parameter:%d\n", parameter );

  sendAPICZNP( MT_RPC_SYS_SYS, MT_SYS_RESET_REQ, 1, &parameter );
}

/**************************************************************************************************
 * @fn          znpSniffer
 *
 * @brief       This function sends a reset message to the ZNP.
 *
 * @param       mode - MT_SYS_SNIFFER_DISABLE - Disable sniffer I/O
 *                     MT_SYS_SNIFFER_ENABLE - Enable sniffer I/O
 *                     MT_SYS_SNIFFER_GET_SETTING - read sniffer setting (enable or disable)
 *
 * @return      status for a write (disable/enable), setting (0/1) for a get
 **************************************************************************************************/
uint8 znpSniffer( uint8 mode )
{
  uiPrintfEx(trINFO, "znp_misc znpSniffer: mode:%d\n", mode );

  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_SYS,
      MT_SYS_SNIFFER_PARAMETERS, 1, &mode ));
}

/**************************************************************************************************
 * @fn          znpZDOSetParam
 *
 * @brief       This function sends a ZDO Ext Set Parameter message.
 *
 * @param       has_useMultiCast - True to change the nwkUseMultiCast option
 * @param       useMultiCast - True to turn on multicast option, false to turn off
 *
 * @return      status for a write (disable/enable), setting (0/1) for a get
 **************************************************************************************************/
uint8 znpZDOSetParam( uint8 has_useMultiCast, uint8 useMultiCast )
{
  uint8 buf[1];
  uint8 *pBuf = buf;

  //uiPrintfEx(trDEBUG, "znp_misc znpZDOSetParam: has_useMultiCast:%d, useMultiCast:%d\n", has_useMultiCast, useMultiCast );

  if ( has_useMultiCast )
  {
    *pBuf++ = useMultiCast | 0x80;
  }
  else
  {
    *pBuf++ = 0;
  }

  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_ZDO, MT_ZDO_EXT_SET_PARAMS,
      1, buf ));
}

/**************************************************************************************************
 *
 * @fn          ZMacSetJammerParameters
 *
 * @brief       Set the Jammer detect parameters
 *
 * @param       contEvents - Number of continuous events needed to detect a jam
 * @param       highNoiseLevel - Noise level needed to be jammed
 * @param       detectPeriod - The Time between each noise level reading
 *
 * @return      status
 *
 **************************************************************************************************/
uint8 ZMacSetJammerParameters( uint16 contEvents, int8 highNoiseLevel,
                               uint32 detectPeriod )
{
  uint8 status;
  uint8 buffer[7];
  uint8 *pBuf;

  pBuf = buffer;

  *pBuf++ = LO_UINT16( contEvents );
  *pBuf++ = HI_UINT16( contEvents );
  *pBuf++ = highNoiseLevel;
  *pBuf++ = BREAK_UINT32( detectPeriod, 0 );
  *pBuf++ = BREAK_UINT32( detectPeriod, 1 );
  *pBuf++ = BREAK_UINT32( detectPeriod, 2 );
  *pBuf++ = BREAK_UINT32( detectPeriod, 3 );

  status = sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_SYS,
      MT_SYS_JAMMER_PARAMETERS, 7, buffer );

  return (status);
}

/*********************************************************************
 *********************************************************************/
