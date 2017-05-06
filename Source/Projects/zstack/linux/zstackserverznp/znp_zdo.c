/**************************************************************************************************
 Filename:       znp_zdo.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:

 This file declares the ZNP Application Processor proxy ZDO API functions.


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
 **************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api_client.h"

#include "ZComDef.h"
#include "OSAL.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDSecMgr.h"

#include "MT.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint16 ami;
  uint16 keyNvId;   // index to the Link Key table in NV
  ZDSecMgr_Authentication_Option authenticateOption;
} ZDSecMgrEntry_t;

typedef enum
{
  RTG_SUCCESS, RTG_FAIL,
} RTG_Status_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */
uint8 ZDP_TransID = 0;
#if defined( HOLD_AUTO_START )
devStates_t devState = DEV_HOLD;
#else
devStates_t devState = DEV_INIT;
#endif

static ZDSecMgrEntry_t fixedEntry =
{ 0 };
static aps_Group_t dummyGroup =
{ 0 };

extern apicHandle_t zmainClientHandle;
#define ZNP_API_CLIENT zmainClientHandle

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
extern uint8 sendAPICZNP( int subSys, int cmdID, int len, uint8 *pData );
extern uint8 sendNPIExpectDefaultStatusZNP( int subSys, int cmdID, int len,
                                            uint8 *pData );

static uint8 sendNPIZDO( int cmdID, int len, uint8 *pData );
static uint8 sendNPIExpectDefaultStatusZDO( int cmdID, int len, uint8 *pData );

/**************************************************************************************************
 *
 * @fn          sendNPIZDO
 *
 * @brief       Send a request message and no expect the normal "default" response (MacDefaultRsp)
 *
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      synchronous return status
 *
 **************************************************************************************************/
static uint8 sendNPIZDO( int cmdID, int len, uint8 *pData )
{
  return (sendAPICZNP( MT_RPC_SYS_ZDO, cmdID, len, pData ));
}

/**************************************************************************************************
 *
 * @fn          sendNPIExpectDefaultStatusZDO
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
static uint8 sendNPIExpectDefaultStatusZDO( int cmdID, int len, uint8 *pData )
{
  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_ZDO, cmdID, len, pData ));
}

/*********************************************************************
 * @fn          ZDO_RegisterForZdoCB
 *
 * @brief       Call this function to register the higher layer (for
 *              example, the Application layer or MT layer) with ZDO
 *              callbacks to get notified of some ZDO indication like
 *              existence of a concentrator or receipt of a source
 *              route record.
 *
 * @param       indID - ZDO Indication ID
 * @param       pFn   - Callback function pointer
 *
 * @return      ZSuccess - successful, ZInvalidParameter if not
 */
ZStatus_t ZDO_RegisterForZdoCB( uint8 indID, pfnZdoCb pFn )
{
  // This function isn't needed with the ZNP, all callbacks are
  // subscribed to.
  return (ZSuccess);
}

/*********************************************************************
 * @fn          ZDO_RegisterForZDOMsg
 *
 * @brief       Call this function to register of an incoming over
 *              the air ZDO message - probably a response message
 *              but requests can also be received.
 *              Messages are delivered to the task with ZDO_CB_MSG
 *              as the message ID.
 *
 * @param       taskID - Where you would like the message delivered
 * @param       clusterID - What message?
 *
 * @return      ZSuccess - successful, ZMemError if not
 */
ZStatus_t ZDO_RegisterForZDOMsg( uint8 taskID, uint16 clusterID )
{
  ZStatus_t status;
  uint8 buffer[2];

  buffer[0] = LO_UINT16( clusterID );
  buffer[1] = HI_UINT16( clusterID );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MSG_CB_REGISTER, 2, buffer );
  return (status);
}

/*********************************************************************
 * @fn          ZDP_MatchDescReq
 *
 * @brief       This builds and send a Match_Desc_req message.  This
 *              function sends a broadcast or unicast message
 *              requesting the list of endpoint/interfaces that
 *              match profile ID and cluster IDs.
 *
 * @param       dstAddr - destination address
 * @param       nwkAddr - network address of interest
 * @param       ProfileID - Profile ID
 * @param       NumInClusters - number of input clusters
 * @param       InClusterList - input cluster ID list
 * @param       NumOutClusters - number of output clusters
 * @param       OutClusterList - output cluster ID list
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MatchDescReq( zAddrType_t *dstAddr, uint16 nwkAddr,
                             uint16 ProfileID, byte NumInClusters,
                             cId_t *InClusterList, byte NumOutClusters,
                             cId_t *OutClusterList, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 *pBuf;
  uint8 len = (8 + (NumInClusters * 2) + (NumOutClusters * 2));

  pBuf = malloc( len );
  if ( pBuf )
  {
    uint8 x;
    uint8 *pPtr = pBuf;

    *pPtr++ = LO_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = HI_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = LO_UINT16( nwkAddr );
    *pPtr++ = HI_UINT16( nwkAddr );
    *pPtr++ = LO_UINT16( ProfileID );
    *pPtr++ = HI_UINT16( ProfileID );
    *pPtr++ = NumInClusters;
    for ( x = 0; x < NumInClusters; x++ )
    {
      *pPtr++ = LO_UINT16( InClusterList[x] );
      *pPtr++ = HI_UINT16( InClusterList[x] );
    }
    *pPtr++ = NumOutClusters;
    for ( x = 0; x < NumOutClusters; x++ )
    {
      *pPtr++ = LO_UINT16( OutClusterList[x] );
      *pPtr++ = HI_UINT16( OutClusterList[x] );
    }

    status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MATCH_DESC_REQ, len, pBuf );

    free( pBuf );
  }
  else
  {
    status = ZMemError;
  }

  return (status);
}

/*********************************************************************
 * @fn          ZDP_NwkAddrReq
 *
 * @brief       This builds and send a NWK_addr_req message.  This
 *              function sends a broadcast message looking for a 16
 *              bit address with a 64 bit address as bait.
 *
 * @param       IEEEAddress - looking for this device
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_NwkAddrReq( uint8 *IEEEAddress, byte ReqType, byte StartIndex,
                           byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[10];

  memcpy( buf, IEEEAddress, 8 );

  buf[8] = ReqType;
  buf[9] = StartIndex;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_NWK_ADDR_REQ, 10, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_IEEEAddrReq
 *
 * @brief       This builds and send a IEEE_addr_req message.  This
 *              function sends a unicast message looking for a 64
 *              bit IEEE address with a 16 bit address as bait.
 *
 * @param       ReqType - ZDP_IEEEADDR_REQTYPE_SINGLE or
 *                        ZDP_IEEEADDR_REQTYPE_EXTENDED
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_IEEEAddrReq( uint16 shortAddr, byte ReqType, byte StartIndex,
                            byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[4];

  buf[0] = LO_UINT16( shortAddr );
  buf[1] = HI_UINT16( shortAddr );

  buf[2] = ReqType;
  buf[3] = StartIndex;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_IEEE_ADDR_REQ, 4, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_NWKAddrOfInterestReq
 *
 * @brief       This builds and send a request message that has
 *              NWKAddrOfInterest as its only parameter.
 *
 * @param       dstAddr - destination address
 * @param       nwkAddr - 16 bit address
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_NWKAddrOfInterestReq( zAddrType_t *dstAddr, uint16 nwkAddr,
                                     byte cmd, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[5];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  buf[2] = LO_UINT16( nwkAddr );
  buf[3] = HI_UINT16( nwkAddr );

  buf[4] = cmd;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_NWK_ADDR_OF_INTEREST_REQ, 5,
      buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_SimpleDescReq
 *
 * @brief       This builds and send a NWK_Simple_Desc_req
 *              message.  This function sends unicast message to the
 *              destination device.
 *
 * @param       dstAddr - destination address
 * @param       nwkAddr - 16 bit address
 * @param       epIntf - endpoint/interface
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_SimpleDescReq( zAddrType_t *dstAddr, uint16 nwkAddr,
                              byte endPoint, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[5];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  buf[2] = LO_UINT16( nwkAddr );
  buf[3] = HI_UINT16( nwkAddr );

  buf[4] = endPoint;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_SIMPLE_DESC_REQ, 5, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_UserDescSet
 *
 * @brief       This builds and send a User_Desc_set message to set
 *              the user descriptor.  This function sends unicast
 *              message to the destination device.
 *
 * @param       dstAddr - destination address
 * @param       nwkAddr - 16 bit address
 * @param       UserDescriptor - user descriptor
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_UserDescSet( zAddrType_t *dstAddr, uint16 nwkAddr,
                            UserDescriptorFormat_t *UserDescriptor,
                            byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[21];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  buf[2] = LO_UINT16( nwkAddr );
  buf[3] = HI_UINT16( nwkAddr );

  buf[4] = UserDescriptor->len;
  memcpy( &buf[5], UserDescriptor->desc, 16 );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_USER_DESC_SET, 21, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_ServerDiscReq
 *
 * @brief       Build and send a Server_Discovery_req request message.
 *
 * @param       serverMask - 16-bit bit-mask of server services being sought.
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_ServerDiscReq( uint16 serverMask, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[2];

  buf[0] = LO_UINT16( serverMask );
  buf[1] = HI_UINT16( serverMask );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_SERVICE_DISC_REQ, 2, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_EndDeviceBindReq
 *
 * @brief       This builds and sends a End_Device_Bind_req message.
 *              This function sends a unicast message.
 *
 * @param       dstAddr - destination address
 * @param       LocalCoordinator - short address of local coordinator
 * @param       epIntf - Endpoint/Interface of Simple Desc
 * @param       ProfileID - Profile ID
 *
 *   The Input cluster list is the opposite of what you would think.
 *   This is the output cluster list of this device
 * @param       NumInClusters - number of input clusters
 * @param       InClusterList - input cluster ID list
 *
 *   The Output cluster list is the opposite of what you would think.
 *   This is the input cluster list of this device
 * @param       NumOutClusters - number of output clusters
 * @param       OutClusterList - output cluster ID list
 *
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_EndDeviceBindReq( zAddrType_t *dstAddr, uint16 LocalCoordinator,
                                 byte endPoint, uint16 ProfileID,
                                 byte NumInClusters, cId_t *InClusterList,
                                 byte NumOutClusters, cId_t *OutClusterList,
                                 byte SecurityEnable )
{
  ZStatus_t status;
  uint8 *pBuf;
  uint8 len = (17 + (NumInClusters * 2) + (NumOutClusters * 2));

  pBuf = malloc( len );
  if ( pBuf )
  {
    uint8 x;
    uint8 *pPtr = pBuf;

    *pPtr++ = LO_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = HI_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = LO_UINT16( LocalCoordinator );
    *pPtr++ = HI_UINT16( LocalCoordinator );

    // Skip extended address (in MT definition)
    memset( pPtr, 0, 8 );
    pPtr += 8;

    *pPtr++ = endPoint;

    *pPtr++ = LO_UINT16( ProfileID );
    *pPtr++ = HI_UINT16( ProfileID );

    *pPtr++ = NumInClusters;
    for ( x = 0; x < NumInClusters; x++ )
    {
      *pPtr++ = LO_UINT16( InClusterList[x] );
      *pPtr++ = HI_UINT16( InClusterList[x] );
    }

    *pPtr++ = NumOutClusters;
    for ( x = 0; x < NumOutClusters; x++ )
    {
      *pPtr++ = LO_UINT16( OutClusterList[x] );
      *pPtr++ = HI_UINT16( OutClusterList[x] );
    }

    status = sendNPIExpectDefaultStatusZDO( MT_ZDO_END_DEV_BIND_REQ, len,
        pBuf );

    free( pBuf );
  }
  else
  {
    status = ZMemError;
  }

  return (status);
}

/*********************************************************************
 * @fn          ZDP_BindUnbindReq
 *
 * @brief       This builds and send a Bind_req or Unbind_req message
 *              Depending on the ClusterID. This function
 *              sends a unicast message to the local coordinator.
 *
 * @param       BindOrUnbind - either Bind_req or Unbind_req
 * @param       dstAddr - destination address of the message
 * @param       SourceAddr - source 64 bit address of the binding
 * @param       SrcEPIntf - Source endpoint/interface
 * @param       ClusterID - Binding cluster ID
 * @param       DestinationAddr - destination 64 bit addr of binding
 * @param       DstEPIntf - destination endpoint/interface
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_BindUnbindReq( uint16 BindOrUnbind, zAddrType_t *dstAddr,
                              uint8 *SourceAddr, byte SrcEndPoint,
                              cId_t ClusterID, zAddrType_t *destinationAddr,
                              byte DstEndPoint, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 *pBuf;
  uint8 len = 15;

  if ( destinationAddr->addrMode == Addr64Bit )
  {
    len += 8;
  }
  else
  {
    len += 2;
  }

  pBuf = malloc( len );
  if ( pBuf )
  {
    int cmdID;
    uint8 *pPtr = pBuf;

    *pPtr++ = LO_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = HI_UINT16( dstAddr->addr.shortAddr );

    memcpy( pPtr, SourceAddr, 8 );
    pPtr += 8;

    *pPtr++ = SrcEndPoint;

    *pPtr++ = LO_UINT16( ClusterID );
    *pPtr++ = HI_UINT16( ClusterID );

    *pPtr++ = destinationAddr->addrMode;

    if ( destinationAddr->addrMode == Addr64Bit )
    {
      memcpy( pPtr, destinationAddr->addr.extAddr, 8 );
      pPtr += 8;
    }
    else
    {
      *pPtr++ = LO_UINT16( destinationAddr->addr.shortAddr );
      *pPtr++ = HI_UINT16( destinationAddr->addr.shortAddr );
    }

    *pPtr = DstEndPoint;

    if ( BindOrUnbind == Bind_req )
    {
      cmdID = MT_ZDO_BIND_REQ;
    }
    else
    {
      cmdID = MT_ZDO_UNBIND_REQ;
    }

    status = sendNPIExpectDefaultStatusZDO( cmdID, len, pBuf );

    free( pBuf );
  }
  else
  {
    status = ZMemError;
  }

  return (status);
}

/*********************************************************************
 * @fn          ZDP_MgmtPermitJoinReq
 *
 * @brief       This builds and send a Mgmt_Permit_Join_req message.
 *
 * @param       dstAddr - destination address of the message
 * @param       duration - Permit duration
 * @param       TcSignificance - Trust Center Significance
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MgmtPermitJoinReq( zAddrType_t *dstAddr, byte duration,
                                  byte TcSignificance, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[5];

  buf[0] = Addr16Bit;
  buf[1] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[2] = HI_UINT16( dstAddr->addr.shortAddr );

  buf[3] = duration;
  buf[4] = TcSignificance;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MGMT_PERMIT_JOIN_REQ, 5, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_MgmtLeaveReq
 *
 * @brief       This builds and send a Mgmt_Leave_req message.
 *
 * @param       dstAddr - destination address of the message
 *              IEEEAddr - IEEE adddress of device that is removed
 *              RemoveChildren - set to 1 to remove the children of the
 *                                device as well. 0 otherwise.
 *              Rejoin - set to 1 if the removed device should rejoin
 afterwards. 0 otherwise.
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MgmtLeaveReq( zAddrType_t *dstAddr, uint8 *IEEEAddr,
                             uint8 RemoveChildren, uint8 Rejoin,
                             uint8 SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[11];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  memcpy( &buf[2], IEEEAddr, 8 );

  buf[10] = 0;
  if ( Rejoin )
  {
    buf[10] |= 0x1;
  }
  if ( RemoveChildren )
  {
    buf[10] |= 0x02;
  }

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MGMT_LEAVE_REQ, 11, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_MgmtNwkDiscReq
 *
 * @brief       This builds and send a Mgmt_NWK_Disc_req message. This
 *              function sends a unicast message.
 *
 * @param       dstAddr - destination address of the message
 * @param       ScanChannels - 32 bit address bit map
 * @param       StartIndex - Starting index within the reporting network
 *                           list
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MgmtNwkDiscReq( zAddrType_t *dstAddr, uint32 ScanChannels,
                               byte ScanDuration, byte StartIndex,
                               byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[8];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  buf[2] = BREAK_UINT32( ScanChannels, 0 );
  buf[3] = BREAK_UINT32( ScanChannels, 1 );
  buf[4] = BREAK_UINT32( ScanChannels, 2 );
  buf[5] = BREAK_UINT32( ScanChannels, 3 );

  buf[6] = ScanDuration;
  buf[7] = StartIndex;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MGMT_NWKDISC_REQ, 8, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_SendData
 *
 * @brief       This builds and send a request message that has
 *              NWKAddrOfInterest as its only parameter.
 *
 * @param       dstAddr - destination address
 * @param       cmd - clusterID
 * @param       dataLen - number of bytes of data
 * @param       data - pointer to the data
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_SendData( uint8 *TransSeq, zAddrType_t *dstAddr, uint16 cmd,
                         byte len, uint8 *buf, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 *pBuf;
  uint8 bufLen = (6 + len);

  pBuf = malloc( bufLen );
  if ( pBuf )
  {
    uint8 *pPtr = pBuf;

    *pPtr++ = LO_UINT16( dstAddr->addr.shortAddr );
    *pPtr++ = HI_UINT16( dstAddr->addr.shortAddr );

    *pPtr++ = *TransSeq;
    *TransSeq += 1;

    *pPtr++ = LO_UINT16( cmd );
    *pPtr++ = HI_UINT16( cmd );
    *pPtr++ = len;

    memcpy( pPtr, buf, len );

    status = sendNPIExpectDefaultStatusZDO( MT_ZDO_SEND_DATA, bufLen, pBuf );

    free( pBuf );
  }
  else
  {
    status = ZMemError;
  }

  return (status);
}

/*********************************************************************
 * @fn          ZDP_MgmtNwkUpdateReq
 *
 * @brief       This builds and send a Mgmt_NWK_Update_req message. This
 *              function sends a unicast or broadcast message.
 *
 * @param       dstAddr - destination address of the message
 * @param       ChannelMask - 32 bit address bit map
 * @param       ScanDuration - length of time to spend scanning each channel
 * @param       ScanCount - number of energy scans to be conducted
 * @param       NwkUpdateId - NWk Update Id value
 * @param       NwkManagerAddr - NWK address for device with Network Manager
 *                               bit set in its Node Descriptor
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MgmtNwkUpdateReq( zAddrType_t *dstAddr, uint32 ChannelMask,
                                 uint8 ScanDuration, uint8 ScanCount,
                                 uint8 NwkUpdateId, uint16 NwkManagerAddr )
{
  ZStatus_t status;
  uint8 buf[11];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );
  buf[2] = dstAddr->addrMode;

  buf[3] = BREAK_UINT32( ChannelMask, 0 );
  buf[4] = BREAK_UINT32( ChannelMask, 1 );
  buf[5] = BREAK_UINT32( ChannelMask, 2 );
  buf[6] = BREAK_UINT32( ChannelMask, 3 );

  buf[7] = ScanDuration;
  buf[8] = ScanCount;

  buf[9] = LO_UINT16( NwkManagerAddr );
  buf[10] = HI_UINT16( NwkManagerAddr );

  // NWKUpdateId is ignored, the ZNP will use the _NIB.nwkUpdateId+1

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MGMT_NWK_UPDATE_REQ, 11, buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_MgmtDirectJoinReq
 *
 * @brief       This builds and send a Mgmt_Direct_Join_req message. This
 *              function sends a unicast message.
 *
 * @param       dstAddr - destination address of the message
 * @param       deviceAddr - 64 bit IEEE Address
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_MgmtDirectJoinReq( zAddrType_t *dstAddr, uint8 *deviceAddr,
                                  byte capInfo, byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[11];

  buf[0] = LO_UINT16( dstAddr->addr.shortAddr );
  buf[1] = HI_UINT16( dstAddr->addr.shortAddr );

  memcpy( &buf[2], deviceAddr, 8 );

  buf[10] = capInfo;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_MGMT_DIRECT_JOIN_REQ, 11,
      buf );

  return (status);
}

/*********************************************************************
 * @fn          ZDP_DeviceAnnce
 *
 * @brief       This builds and send a Device_Annce message.  This
 *              function sends a broadcast message.
 *
 * @param       nwkAddr - 16 bit address of the device
 * @param       IEEEAddr - 64 bit address of the device
 * @param       capabilities - device capabilities.  This field is only
 *                 sent for v1.1 networks.
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_DeviceAnnce( uint16 nwkAddr, uint8 *IEEEAddr, byte capabilities,
                            byte SecurityEnable )
{
  ZStatus_t status;
  uint8 buf[11];

  buf[0] = LO_UINT16( nwkAddr );
  buf[1] = HI_UINT16( nwkAddr );

  memcpy( &buf[2], IEEEAddr, 8 );

  buf[10] = capabilities;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_END_DEV_ANNCE, 11, buf );

  return (status);
}

/*********************************************************************
 * @fn      ZDApp_StartJoiningCycle()
 *
 * @brief   Starts the joining cycle of a device.  This will only
 *          continue an already started (or stopped) joining cycle.
 *
 * @param   none
 *
 * @return  TRUE if joining stopped, FALSE if joining or rejoining
 */
uint8 ZDApp_StartJoiningCycle( void )
{
  sendNPIZDO( MT_ZDO_STARTUP_FROM_APP, 0, NULL );

  // always return joining for ZNP
  return (0);
}

/* ------------------------------------------------------------------------------------------------
 * ZDO Parsing Functions
 * ------------------------------------------------------------------------------------------------
 */

/*********************************************************************
 * @fn          ZDO_ConvertOTAClusters
 *
 * @brief       This function will convert the over-the-air cluster list
 *              format to an internal format.
 *
 * @param       inMsg  - incoming message (request)
 *
 * @return      pointer to incremented inBuf
 */
uint8 *ZDO_ConvertOTAClusters( uint8 cnt, uint8 *inBuf, uint16 *outList )
{
  uint8 x;

  for ( x = 0; x < cnt; x++ )
  {
    // convert ota format to internal
    outList[x] = BUILD_UINT16( inBuf[0], inBuf[1] );
    inBuf += sizeof(uint16);
  }
  return (inBuf);
}

/*********************************************************************
 * @fn          ZDO_ParseEndDeviceBindReq
 *
 * @brief       This function parses the End_Device_Bind_req message.
 *
 *     NOTE:  The clusters lists in bindReq are allocated in this
 *            function and must be freed by that calling function.
 *
 * @param       inMsg  - incoming message (request)
 * @param       bindReq - pointer to place to parse message to
 *
 * @return      none
 */
void ZDO_ParseEndDeviceBindReq( zdoIncomingMsg_t *inMsg,
                                ZDEndDeviceBind_t *bindReq )
{
  uint8 *msg;

  // Parse the message
  bindReq->TransSeq = inMsg->TransSeq;
  bindReq->srcAddr = inMsg->srcAddr.addr.shortAddr;
  bindReq->SecurityUse = inMsg->SecurityUse;
  msg = inMsg->asdu;

  bindReq->localCoordinator = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;

  osal_cpyExtAddr( bindReq->ieeeAddr, msg );
  msg += Z_EXTADDR_LEN;

  bindReq->endpoint = *msg++;
  bindReq->profileID = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;

  bindReq->inClusters = NULL;
  bindReq->outClusters = NULL;

  if ( (bindReq->numInClusters = *msg++) && (bindReq->inClusters =
      (uint16*) osal_mem_alloc( (bindReq->numInClusters * sizeof(uint16)) )) )
  {
    msg = ZDO_ConvertOTAClusters( bindReq->numInClusters, msg,
        bindReq->inClusters );
  }
  else
  {
    bindReq->numInClusters = 0;
  }

  if ( (bindReq->numOutClusters = *msg++) && (bindReq->outClusters =
      (uint16*) osal_mem_alloc( (bindReq->numOutClusters * sizeof(uint16)) )) )
  {
    msg = ZDO_ConvertOTAClusters( bindReq->numOutClusters, msg,
        bindReq->outClusters );
  }
  else
  {
    bindReq->numOutClusters = 0;
  }
}

/*********************************************************************
 * @fn          ZDO_ParseBindUnbindReq
 *
 * @brief       This function parses the Bind_req or Unbind_req message.
 *
 * @param       inMsg  - incoming message (request)
 * @param       pReq - place to put parsed information
 *
 * @return      none
 */
void ZDO_ParseBindUnbindReq( zdoIncomingMsg_t *inMsg,
                             ZDO_BindUnbindReq_t *pReq )
{
  uint8 *msg;

  msg = inMsg->asdu;
  osal_cpyExtAddr( pReq->srcAddress, msg );
  msg += Z_EXTADDR_LEN;
  pReq->srcEndpoint = *msg++;
  pReq->clusterID = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  pReq->dstAddress.addrMode = *msg++;
  if ( pReq->dstAddress.addrMode == Addr64Bit )
  {
    osal_cpyExtAddr( pReq->dstAddress.addr.extAddr, msg );
    msg += Z_EXTADDR_LEN;
    pReq->dstEndpoint = *msg;
  }
  else
  {
    // copy group address
    pReq->dstAddress.addr.shortAddr = BUILD_UINT16( msg[0], msg[1] );
  }
}

/*********************************************************************
 * @fn      ZDO_ParseAddrRsp
 *
 * @brief   Turns the inMsg (incoming message) into the out parsed
 *          structure.
 *
 * @param   inMsg - incoming message
 *
 * @return  pointer to parsed structures.  This structure was
 *          allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_NwkIEEEAddrResp_t *ZDO_ParseAddrRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_NwkIEEEAddrResp_t *rsp;
  uint8 *msg;
  byte cnt = 0;

  // Calculate the number of items in the list
  if ( inMsg->asduLen > (1 + Z_EXTADDR_LEN + 2) )
  {
    cnt = inMsg->asdu[1 + Z_EXTADDR_LEN + 2];
  }
  else
  {
    cnt = 0;
  }

  // Make buffer
  rsp = (ZDO_NwkIEEEAddrResp_t *) osal_mem_alloc(
      sizeof(ZDO_NwkIEEEAddrResp_t) + (cnt * sizeof(uint16)) );

  if ( rsp )
  {
    msg = inMsg->asdu;

    rsp->status = *msg++;
    if ( rsp->status == ZDO_SUCCESS )
    {
      osal_cpyExtAddr( rsp->extAddr, msg );
      msg += Z_EXTADDR_LEN;
      rsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );

      msg += 2;
      rsp->numAssocDevs = 0;

      // StartIndex field is only present if NumAssocDev field is non-zero.
      if ( cnt > 0 )
      {
        uint16 *pList = &(rsp->devList[0]);
        byte n = cnt;

        rsp->numAssocDevs = *msg++;
        rsp->startIndex = *msg++;

        while ( n != 0 )
        {
          *pList++ = BUILD_UINT16( msg[0], msg[1] );
          msg += sizeof(uint16);
          n--;
        }
      }
    }
  }

  return (rsp);
}

/*********************************************************************
 * @fn          ZDO_ParseNodeDescRsp
 *
 * @brief       This function parses the Node_Desc_rsp message.
 *
 * @param       inMsg - incoming message
 * @param       pNDRsp - place to parse the message into
 *
 * @return      none
 */
void ZDO_ParseNodeDescRsp( zdoIncomingMsg_t *inMsg, ZDO_NodeDescRsp_t *pNDRsp )
{
  uint8 *msg;

  msg = inMsg->asdu;

  pNDRsp->status = *msg++;
  pNDRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );

  if ( pNDRsp->status == ZDP_SUCCESS )
  {
    msg += 2;
    pNDRsp->nodeDesc.LogicalType = *msg & 0x07;

    pNDRsp->nodeDesc.ComplexDescAvail = (*msg & 0x08) >> 3;
    pNDRsp->nodeDesc.UserDescAvail = (*msg & 0x10) >> 4;

    msg++;  // Reserved bits.
    pNDRsp->nodeDesc.FrequencyBand = (*msg >> 3) & 0x1f;
    pNDRsp->nodeDesc.APSFlags = *msg++ & 0x07;
    pNDRsp->nodeDesc.CapabilityFlags = *msg++;
    pNDRsp->nodeDesc.ManufacturerCode[0] = *msg++;
    pNDRsp->nodeDesc.ManufacturerCode[1] = *msg++;
    pNDRsp->nodeDesc.MaxBufferSize = *msg++;
    pNDRsp->nodeDesc.MaxInTransferSize[0] = *msg++;
    pNDRsp->nodeDesc.MaxInTransferSize[1] = *msg++;
    pNDRsp->nodeDesc.ServerMask = BUILD_UINT16( msg[0], msg[1] );
    msg += 2;
    pNDRsp->nodeDesc.MaxOutTransferSize[0] = *msg++;
    pNDRsp->nodeDesc.MaxOutTransferSize[1] = *msg++;
    pNDRsp->nodeDesc.DescriptorCapability = *msg;
  }
}

/*********************************************************************
 * @fn          ZDO_ParsePowerDescRsp
 *
 * @brief       This function parses the Power_Desc_rsp message.
 *
 * @param       inMsg  - incoming message
 * @param       pNPRsp - place to parse the message into
 *
 * @return      none
 */
void ZDO_ParsePowerDescRsp( zdoIncomingMsg_t *inMsg, ZDO_PowerRsp_t *pNPRsp )
{
  uint8 *msg;

  msg = inMsg->asdu;
  pNPRsp->status = *msg++;
  pNPRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );

  if ( pNPRsp->status == ZDP_SUCCESS )
  {
    msg += 2;
    pNPRsp->pwrDesc.AvailablePowerSources = *msg >> 4;
    pNPRsp->pwrDesc.PowerMode = *msg++ & 0x0F;
    pNPRsp->pwrDesc.CurrentPowerSourceLevel = *msg >> 4;
    pNPRsp->pwrDesc.CurrentPowerSource = *msg++ & 0x0F;
  }
}

/*********************************************************************
 * @fn          ZDO_ParseSimpleDescRsp
 *
 * @brief       This function parse the Simple_Desc_rsp message.
 *
 *   NOTE: The pAppInClusterList and pAppOutClusterList fields
 *         in the SimpleDescriptionFormat_t structure are allocated
 *         and the calling function needs to free [osal_msg_free()]
 *         these buffers.
 *
 * @param       inMsg  - incoming message
 * @param       pSimpleDescRsp - place to parse the message into
 *
 * @return      none
 */
void ZDO_ParseSimpleDescRsp( zdoIncomingMsg_t *inMsg,
                             ZDO_SimpleDescRsp_t *pSimpleDescRsp )
{
  uint8 *msg;

  msg = inMsg->asdu;
  pSimpleDescRsp->status = *msg++;
  pSimpleDescRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += sizeof(uint16);
  msg++; // Skip past the length field.

  if ( pSimpleDescRsp->status == ZDP_SUCCESS )
  {
    ZDO_ParseSimpleDescBuf( msg, &(pSimpleDescRsp->simpleDesc) );
  }
}

/*********************************************************************
 * @fn          ZDO_ParseEPListRsp
 *
 * @brief       This parse the Active_EP_rsp or Match_Desc_rsp message.
 *
 * @param       inMsg  - incoming message
 *
 * @return      none
 */
ZDO_ActiveEndpointRsp_t *ZDO_ParseEPListRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_ActiveEndpointRsp_t *pRsp;
  uint8 *msg;
  uint8 Status;
  uint8 cnt;

  msg = inMsg->asdu;
  Status = *msg++;
  cnt = msg[2];

  pRsp = (ZDO_ActiveEndpointRsp_t *) osal_mem_alloc(
      sizeof(ZDO_ActiveEndpointRsp_t) + cnt );
  if ( pRsp )
  {
    pRsp->status = Status;
    pRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
    msg += sizeof(uint16);
    pRsp->cnt = cnt;
    msg++; // pass cnt
    osal_memcpy( pRsp->epList, msg, cnt );
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseServerDiscRsp
 *
 * @brief       Parse the Server_Discovery_rsp message.
 *
 * @param       inMsg - incoming message.
 * @param       pRsp - place to put the parsed information.
 *
 * @return      none
 */
void ZDO_ParseServerDiscRsp( zdoIncomingMsg_t *inMsg,
                             ZDO_ServerDiscRsp_t *pRsp )
{
  pRsp->status = inMsg->asdu[0];
  pRsp->serverMask = BUILD_UINT16( inMsg->asdu[1], inMsg->asdu[2] );
}

/*********************************************************************
 * @fn          ZDO_ParseMgmtLqiRsp
 *
 * @brief       This function parses the incoming Management
 *              LQI response
 *
 * @param       inMsg - incoming message
 *
 * @return      a pointer to parsed response structure (NULL if not allocated).
 *          This structure was allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_MgmtLqiRsp_t *ZDO_ParseMgmtLqiRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmtLqiRsp_t *pRsp;
  uint8 status;
  uint8 startIndex = 0;
  uint8 neighborLqiCount = 0;
  uint8 neighborLqiEntries = 0;
  uint8 *msg;

  msg = inMsg->asdu;

  status = *msg++;
  if ( status == ZSuccess )
  {
    neighborLqiEntries = *msg++;
    startIndex = *msg++;
    neighborLqiCount = *msg++;
  }

  // Allocate a buffer big enough to handle the list.
  pRsp = (ZDO_MgmtLqiRsp_t *) osal_mem_alloc(
      sizeof(ZDO_MgmtLqiRsp_t)
          + (neighborLqiCount * sizeof(ZDP_MgmtLqiItem_t)) );
  if ( pRsp )
  {
    uint8 x;
    ZDP_MgmtLqiItem_t *pList = pRsp->list;
    pRsp->status = status;
    pRsp->neighborLqiEntries = neighborLqiEntries;
    pRsp->startIndex = startIndex;
    pRsp->neighborLqiCount = neighborLqiCount;

    for ( x = 0; x < neighborLqiCount; x++ )
    {
      uint8 tmp;

      pList->panID = 0; // This isn't in the record, why is it in the structure?
      osal_cpyExtAddr( pList->extPanID, msg );   //Copy extended PAN ID
      msg += Z_EXTADDR_LEN;

      osal_cpyExtAddr( pList->extAddr, msg );   //Copy extended address
      msg += Z_EXTADDR_LEN;

      pList->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
      msg += 2;

      tmp = *msg++;
      pList->devType = tmp & 0x03;
      pList->rxOnIdle = (tmp >> 2) & 0x03;
      pList->relation = (tmp >> 4) & 0x07;

      pList->permit = (*msg++) & 0x03;

      pList->depth = *msg++;

      pList->lqi = *msg++;
      pList++;
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseMgmNwkDiscRsp
 *
 * @brief       This function parses the incoming Management
 *              Network Discover response.
 *
 * @param       inMsg - incoming message
 *
 * @return      pointer to parsed response.  This structure was
 *          allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_MgmNwkDiscRsp_t *ZDO_ParseMgmNwkDiscRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmNwkDiscRsp_t *pRsp;
  uint8 status;
  uint8 networkCount = 0;
  uint8 startIndex = 0;
  uint8 networkListCount = 0;
  uint8 *msg;

  msg = inMsg->asdu;
  status = *msg++;

  if ( status == ZSuccess )
  {
    networkCount = *msg++;
    startIndex = *msg++;
    networkListCount = *msg++;
  }

  // Allocate a buffer big enough to handle the list.
  pRsp = (ZDO_MgmNwkDiscRsp_t *) osal_mem_alloc(
      sizeof(ZDO_MgmNwkDiscRsp_t)
          + (networkListCount * sizeof(mgmtNwkDiscItem_t)) );
  if ( pRsp )
  {
    uint8 x;
    mgmtNwkDiscItem_t *pList;

    pRsp->status = status;
    pRsp->networkCount = networkCount;
    pRsp->startIndex = startIndex;
    pRsp->networkListCount = networkListCount;
    pList = pRsp->list;

    for ( x = 0; x < networkListCount; x++ )
    {
      osal_cpyExtAddr( pList->extendedPANID, msg );   //Copy extended PAN ID
      pList->PANId = BUILD_UINT16( msg[0], msg[1] );
      msg += Z_EXTADDR_LEN;

      pList->logicalChannel = *msg++;
      pList->stackProfile = (*msg) & 0x0F;
      pList->version = (*msg++ >> 4) & 0x0F;
      pList->beaconOrder = (*msg) & 0x0F;
      pList->superFrameOrder = (*msg++ >> 4) & 0x0F;
      pList->permitJoining = *msg++;
      pList++;
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseMgmtRtgRsp
 *
 * @brief       This function parses the incoming Management
 *              Routing response.
 *
 * @param       inMsg - incoming message
 *
 * @return      a pointer to parsed response structure (NULL if not allocated).
 *          This structure was allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_MgmtRtgRsp_t *ZDO_ParseMgmtRtgRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmtRtgRsp_t *pRsp;
  uint8 status;
  uint8 rtgCount = 0;
  uint8 startIndex = 0;
  uint8 rtgListCount = 0;
  uint8 *msg;

  msg = inMsg->asdu;

  status = *msg++;
  if ( status == ZSuccess )
  {
    rtgCount = *msg++;
    startIndex = *msg++;
    rtgListCount = *msg++;
  }

  // Allocate a buffer big enough to handle the list
  pRsp = (ZDO_MgmtRtgRsp_t *) osal_mem_alloc(
      sizeof(ZDO_MgmtRtgRsp_t) + (rtgListCount * sizeof(rtgItem_t)) );
  if ( pRsp )
  {
    uint8 x;
    rtgItem_t *pList = pRsp->list;
    pRsp->status = status;
    pRsp->rtgCount = rtgCount;
    pRsp->startIndex = startIndex;
    pRsp->rtgListCount = rtgListCount;

    for ( x = 0; x < rtgListCount; x++ )
    {
      uint8 statOpt;

      pList->dstAddress = BUILD_UINT16( msg[0], msg[1] );
      msg += 2;
      statOpt = *msg++;
      pList->status = (statOpt & 0x07);
      pList->options = ((statOpt >> 3) & 0x07);
      pList->nextHopAddress = BUILD_UINT16( msg[0], msg[1] );
      msg += 2;
      pList++;
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseMgmtBindRsp
 *
 * @brief       This function parses the incoming Management
 *              Binding response.
 *
 * @param       inMsg - pointer to message to parse
 *
 * @return      a pointer to parsed response structure (NULL if not allocated).
 *          This structure was allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_MgmtBindRsp_t *ZDO_ParseMgmtBindRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_MgmtBindRsp_t *pRsp;
  uint8 status;
  uint8 bindingCount = 0;
  uint8 startIndex = 0;
  uint8 bindingListCount = 0;
  uint8 *msg;

  msg = inMsg->asdu;

  status = *msg++;
  if ( status == ZSuccess )
  {
    bindingCount = *msg++;
    startIndex = *msg++;
    bindingListCount = *msg++;
  }

  // Allocate a buffer big enough to handle the list
  pRsp =
      (ZDO_MgmtBindRsp_t *) osal_mem_alloc(
          (sizeof(ZDO_MgmtBindRsp_t)
              + (bindingListCount * sizeof(apsBindingItem_t))) );
  if ( pRsp )
  {
    uint8 x;
    apsBindingItem_t *pList = pRsp->list;
    pRsp->status = status;
    pRsp->bindingCount = bindingCount;
    pRsp->startIndex = startIndex;
    pRsp->bindingListCount = bindingListCount;

    for ( x = 0; x < bindingListCount; x++ )
    {
      osal_cpyExtAddr( pList->srcAddr, msg );
      msg += Z_EXTADDR_LEN;
      pList->srcEP = *msg++;

      // Get the Cluster ID

      pList->clusterID = BUILD_UINT16( msg[0], msg[1] );
      msg += 2;
      pList->dstAddr.addrMode = *msg++;
      if ( pList->dstAddr.addrMode == Addr64Bit )
      {
        osal_cpyExtAddr( pList->dstAddr.addr.extAddr, msg );
        msg += Z_EXTADDR_LEN;
        pList->dstEP = *msg++;
      }
      else
      {
        pList->dstAddr.addr.shortAddr = BUILD_UINT16( msg[0], msg[1] );
        msg += 2;
      }

      pList++;
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseUserDescRsp
 *
 * @brief       This function parses the incoming User
 *              Descriptor Response.
 *
 * @param       inMsg - incoming response message
 *
 * @return      a pointer to parsed response structure (NULL if not allocated).
 *          This structure was allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_UserDescRsp_t *ZDO_ParseUserDescRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_UserDescRsp_t *pRsp;
  uint8 *msg;
  uint8 descLen = 0;

  msg = inMsg->asdu;

  if ( msg[0] == ZSuccess )
  {
    descLen = msg[3];
  }

  pRsp = (ZDO_UserDescRsp_t *) osal_mem_alloc(
      sizeof(ZDO_UserDescRsp_t) + descLen );
  if ( pRsp )
  {
    pRsp->status = msg[0];
    pRsp->nwkAddr = BUILD_UINT16( msg[1], msg[2] );
    pRsp->length = descLen;
    if ( descLen )
    {
      osal_memcpy( pRsp->desc, &msg[4], descLen );
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseSimpleDescBuf
 *
 * @brief       Parse a byte sequence representation of a Simple Descriptor.
 *
 * @param       buf  - pointer to a byte array representing a Simple Desc.
 * @param       desc - SimpleDescriptionFormat_t *
 *
 *              This routine allocates storage for the cluster IDs because
 *              they are 16-bit and need to be aligned to be properly processed.
 *              This routine returns non-zero if an allocation fails.
 *
 *              NOTE: This means that the caller or user of the input structure
 *                    is responsible for freeing the memory
 *
 * @return      0: success
 *              1: failure due to malloc failure.
 */
uint8 ZDO_ParseSimpleDescBuf( uint8 *buf, SimpleDescriptionFormat_t *desc )
{
  uint8 num, i;

  desc->EndPoint = *buf++;
  desc->AppProfId = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;
  desc->AppDeviceId = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;
  desc->AppDevVer = *buf >> 4;

  desc->Reserved = 0;
  buf++;

  // move in input cluster list (if any). allocate aligned memory.
  num = desc->AppNumInClusters = *buf++;
  if ( num )
  {
    if ( !(desc->pAppInClusterList = (uint16 *) osal_mem_alloc(
        num * sizeof(uint16) )) )
    {
      // malloc failed. we're done.
      return 1;
    }
    for ( i = 0; i < num; ++i )
    {
      desc->pAppInClusterList[i] = BUILD_UINT16( buf[0], buf[1] );
      buf += 2;
    }
  }

  // move in output cluster list (if any). allocate aligned memory.
  num = desc->AppNumOutClusters = *buf++;
  if ( num )
  {
    if ( !(desc->pAppOutClusterList = (uint16 *) osal_mem_alloc(
        num * sizeof(uint16) )) )
    {
      // malloc failed. free input cluster list memory if there is any
      if ( desc->pAppInClusterList != NULL )
      {
        osal_mem_free( desc->pAppInClusterList );

        desc->pAppInClusterList = NULL;
      }
      return 1;
    }
    for ( i = 0; i < num; ++i )
    {
      desc->pAppOutClusterList[i] = BUILD_UINT16( buf[0], buf[1] );
      buf += 2;
    }
  }
  return 0;
}

/*********************************************************************
 * @fn          ZDO_ParseDeviceAnnce
 *
 * @brief       Parse a Device Announce message.
 *
 * @param       inMsg - Incoming message
 * @param       pAnnce - place to put the parsed information
 *
 * @return      none
 */
void ZDO_ParseDeviceAnnce( zdoIncomingMsg_t *inMsg, ZDO_DeviceAnnce_t *pAnnce )
{
  uint8 *msg;

  // Parse incoming message
  msg = inMsg->asdu;
  pAnnce->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  osal_cpyExtAddr( pAnnce->extAddr, msg );
  msg += Z_EXTADDR_LEN;
  pAnnce->capabilities = *msg;
}

/*********************************************************************
 * @fn          ZDO_ParseMgmtNwkUpdateNotify
 *
 * @brief       This function handles parsing of the incoming Management
 *              Network Update notify.
 *
 * @param       inMsg - incoming message (request)
 *
 * @return      a pointer to parsed response structure (NULL if not allocated).
 *          This structure was allocated using osal_mem_alloc, so it must be freed
 *          by the calling function [osal_mem_free()].
 */
ZDO_MgmtNwkUpdateNotify_t *ZDO_ParseMgmtNwkUpdateNotify(
    zdoIncomingMsg_t *inMsg )
{
  uint8 status;
  uint32 scannedChannels = 0;
  uint16 totalTransmissions = 0;
  uint16 transmissionFailures = 0;
  uint8 listCount = 0;
  uint8 *msg = inMsg->asdu;
  ZDO_MgmtNwkUpdateNotify_t *pRsp;

  status = *msg++;
  if ( status == ZSuccess )
  {
    scannedChannels = osal_build_uint32( msg, 4 );
    msg += 4;
    totalTransmissions = BUILD_UINT16( msg[0], msg[1] );
    msg += 2;
    transmissionFailures = BUILD_UINT16( msg[0], msg[1] );
    msg += 2;
    listCount = *msg++;
  }

  pRsp = (ZDO_MgmtNwkUpdateNotify_t *) osal_mem_alloc(
      sizeof(ZDO_MgmtNwkUpdateNotify_t) + listCount );

  if ( pRsp )
  {
    pRsp->status = status;
    pRsp->scannedChannels = scannedChannels;
    pRsp->totalTransmissions = totalTransmissions;
    pRsp->transmissionFailures = transmissionFailures;
    pRsp->listCount = listCount;

    // Allocate a buffer big enough to handle the list.
    if ( listCount > 0 )
    {
      osal_memcpy( pRsp->energyValues, msg, listCount );
    }
  }

  return (pRsp);
}

/*********************************************************************
 * @fn          ZDO_ParseMgmtNwkUpdateReq
 *
 * @brief       This function handles parsing the incoming Management
 *              Network Update request and starts the request (if needed).
 *
 * @param       inMsg - incoming message (request)
 * @param       pReq - pointer to place to parse message to
 *
 * @return      none
 */
void ZDO_ParseMgmtNwkUpdateReq( zdoIncomingMsg_t *inMsg,
                                ZDO_MgmtNwkUpdateReq_t *pReq )
{
  uint8 *msg = inMsg->asdu;

  pReq->channelMask = osal_build_uint32( msg, 4 );
  msg += 4;
  pReq->scanDuration = *msg++;

  if ( pReq->scanDuration <= 0x05 )
  {
    // Request is to scan over channelMask
    pReq->scanCount = *msg;
  }
  else if ( (pReq->scanDuration == 0xFE) || (pReq->scanDuration == 0xFF) )
  {
    // Request is to change Channel (0xFE) or apsChannelMask and NwkManagerAddr (0xFF)
    pReq->nwkUpdateId = *msg++;

    if ( pReq->scanDuration == 0xFF )
    {
      pReq->nwkManagerAddr = BUILD_UINT16( msg[0], msg[1] );
    }
  }
}

/*********************************************************************
 * @fn          ZDO_ParseEndDeviceTimeoutRsp
 *
 * @brief       Parse the End_Device_Timeout_rsp message.
 *
 * @param       inMsg - incoming message.
 * @param       pRsp - place to put the parsed information.
 *
 * @return      none
 */
void ZDO_ParseEndDeviceTimeoutRsp( zdoIncomingMsg_t *inMsg, uint16 *pRsp )
{
  *pRsp = inMsg->asdu[0];
}

/******************************************************************************
 * @fn          ZDSecMgrEntryLookupExt
 *
 * @brief       Lookup entry index using specified EXT address.
 *
 * @param       extAddr - [in] EXT address
 * @param       entry   - [out] valid entry
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrEntryLookupExt( uint8* extAddr, ZDSecMgrEntry_t** entry )
{
  ZStatus_t status = ZFailure;
  uint8 *pRsp, rspcmdid;

  *entry = NULL;

  memset( &fixedEntry, 0, sizeof(ZDSecMgrEntry_t) );

  // send serialized request to NPI synchronously
  pRsp = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_ZDO,
      MT_ZDO_SEC_ENTRY_LOOKUP_EXT, 8, extAddr, NULL, &rspcmdid, NULL );

  if ( pRsp )
  {
    if ( rspcmdid == MT_ZDO_SEC_ENTRY_LOOKUP_EXT )
    {
      status = pRsp[0];
      // Process the immediate response
      if ( status == ZSuccess )
      {
        fixedEntry.ami = BUILD_UINT16( pRsp[1], pRsp[2] );
        fixedEntry.keyNvId = BUILD_UINT16( pRsp[3], pRsp[4] );
        fixedEntry.authenticateOption =
            (ZDSecMgr_Authentication_Option) pRsp[5];
        *entry = &fixedEntry;
      }
    }

    apicFreeSynchData( pRsp );
  }
  return (status);
}

/******************************************************************************
 * @fn          ZDSecMgrAddLinkKey
 *
 * @brief       Add the application link key to ZDSecMgr. Also mark the device
 *              as authenticated in the authenticateOption. Note that this function
 *              is hardwared to CBKE right now.
 *
 * @param       shortAddr - short address of the partner device
 * @param       extAddr - extended address of the partner device
 * @param       key - link key
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrAddLinkKey( uint16 shortAddr, uint8 *extAddr, uint8 *key )
{
  ZStatus_t status;
  uint8 buf[26];

  buf[0] = LO_UINT16( shortAddr );
  buf[1] = HI_UINT16( shortAddr );
  memcpy( &buf[2], extAddr, 8 );
  memcpy( &buf[10], key, 16 );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_SEC_ADD_LINK_KEY, 26, buf );

  return (status);
}

/******************************************************************************
 * @fn          ZDSecMgrDeviceRemoveByExtAddr
 *
 * @brief       Remove device entry by its ext address.
 *
 * @param       pAddr - pointer to the extended address
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrDeviceRemoveByExtAddr( uint8 *pAddr )
{
  ZStatus_t status;
  uint8 buf[8];

  memcpy( buf, pAddr, 8 );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_SEC_DEVICE_REMOVE, 8, buf );

  return (status);
}

/******************************************************************************
 * @fn          ZDSecMgrUpdateNwkKey
 *
 * @brief       Load a new NWK key and trigger a network wide update.
 *
 * @param       key       - [in] new NWK key
 * @param       keySeqNum - [in] new NWK key sequence number
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrUpdateNwkKey( uint8* key, uint8 keySeqNum, uint16 dstAddr )
{
  ZStatus_t status;
  uint8 buf[19];

  buf[0] = LO_UINT16( dstAddr );
  buf[1] = HI_UINT16( dstAddr );
  buf[2] = keySeqNum;
  memcpy( &buf[3], key, 16 );

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_UPDATE_NWK_KEY, 19, buf );

  return (status);
}

/******************************************************************************
 * @fn          ZDSecMgrSwitchNwkKey
 *
 * @brief       Causes the NWK key to switch via a network wide command.
 *
 * @param       keySeqNum - [in] new NWK key sequence number
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrSwitchNwkKey( uint8 keySeqNum, uint16 dstAddr )
{
  ZStatus_t status;
  uint8 buf[3];

  buf[0] = LO_UINT16( dstAddr );
  buf[1] = HI_UINT16( dstAddr );
  buf[2] = keySeqNum;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_SWITCH_NWK_KEY, 3, buf );

  return (status);
}

/*********************************************************************
 * @fn          NLME_RouteDiscoveryRequest
 *
 * @brief       This function initiates a route discovery
 *              for the destination node.
 *
 * @param       DstAddress      - destination address
 * @param       options         - route options (repair, mcast, mto, etc)
 *
 * @return      ZStatus_t
 */
ZStatus_t NLME_RouteDiscoveryRequest( uint16 DstAddress, byte options,
                                      uint8 radius )
{
  ZStatus_t status;
  uint8 buf[4];

  buf[0] = LO_UINT16( DstAddress );
  buf[1] = HI_UINT16( DstAddress );
  buf[2] = options;
  buf[3] = radius;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_ROUTE_DISC, 4, buf );

  return (status);
}

/*********************************************************************
 * @fn          RTG_CheckRtStatus
 *
 * @brief       This function checks status of a routing table entry
 *
 * @param
 *
 * @return      None
 */
RTG_Status_t RTG_CheckRtStatus( uint16 DstAddress, uint8 RtStatus,
                                uint8 options )
{
  RTG_Status_t status;
  uint8 buf[4];

  buf[0] = LO_UINT16( DstAddress );
  buf[1] = HI_UINT16( DstAddress );
  buf[2] = RtStatus;
  buf[3] = options;

  status = sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_ROUTE_CHECK, 4, buf );

  return (status);
}

/*********************************************************************
 * @fn      aps_RemoveGroup
 *
 * @brief   Remove a group with endpoint and groupID
 *
 * @param   endpoint -
 * @param   groupID - ID to look forw group
 *
 * @return  TRUE if removed, FALSE if not found
 */
uint8 aps_RemoveGroup( uint8 endpoint, uint16 groupID )
{
  uint8 status;
  uint8 buf[3];

  buf[0] = endpoint;
  buf[1] = LO_UINT16( groupID );
  buf[2] = HI_UINT16( groupID );

  status = (uint8) sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_REMOVE_GROUP, 3,
      buf );

  return (status);
}

/*********************************************************************
 * @fn      aps_RemoveAllGroup
 *
 * @brief   Remove a groups with an endpoint
 *
 * @param   endpoint -
 * @param   groupID - ID to look for group
 *
 * @return  none
 */
void aps_RemoveAllGroup( uint8 endpoint )
{
  sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_REMOVE_ALL_GROUP, 1, &endpoint );
}

/*********************************************************************
 * @fn      aps_FindAllGroupsForEndpoint
 *
 * @brief   Find all the groups with endpoint
 *
 * @param   endpoint - endpoint to look for
 * @param   groupList - List to hold group IDs (should hold APS_MAX_GROUPS entries)
 *
 * @return  number of groups copied to groupList
 */
uint8 aps_FindAllGroupsForEndpoint( uint8 endpoint, uint16 *groupList )
{
  uint8 *pRsp, rspcmdid;
  uint8 numGroups = 0;

  // send serialized request to NPI synchronously
  pRsp = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_ZDO,
      MT_ZDO_EXT_FIND_ALL_GROUPS_ENDPOINT, 1, &endpoint, NULL, &rspcmdid,
      NULL );

  if ( pRsp )
  {
    if ( rspcmdid == MT_ZDO_EXT_FIND_ALL_GROUPS_ENDPOINT )
    {
      numGroups = pRsp[0];
      // Process the immediate response
      if ( numGroups > 0 )
      {
        int x;
        uint8 *pBuf = &(pRsp[1]);
        for ( x = 0; x < numGroups; x++ )
        {
          groupList[x] = BUILD_UINT16( pBuf[0], pBuf[1] );
          pBuf += 2;
        }
      }
    }

    apicFreeSynchData( pRsp );
  }
  return (numGroups);
}

/*********************************************************************
 * @fn      aps_FindGroup
 *
 * @brief   Find a group with endpoint and groupID
 *
 * @param   endpoint -
 * @param   groupID - ID to look forw group
 *
 * @return  a pointer to the group information, NULL if not found
 */
aps_Group_t *aps_FindGroup( uint8 endpoint, uint16 groupID )
{
  uint8 *pRsp, pData[3], rspcmdid;
  uint8 status;

  // serialize the request
  pData[0] = endpoint;
  pData[1] = LO_UINT16( groupID );
  pData[2] = HI_UINT16( groupID );

  // send serialized request to NPI synchronously
  pRsp = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_ZDO,
      MT_ZDO_EXT_FIND_GROUP, 3, pData, NULL, &rspcmdid, NULL );

  if ( pRsp )
  {
    if ( rspcmdid == MT_ZDO_EXT_FIND_GROUP )
    {
      status = pRsp[0];
      // Process the immediate response
      if ( status == ZSuccess )
      {
        dummyGroup.ID = BUILD_UINT16( pRsp[1], pRsp[2] );
        dummyGroup.name[0] = pRsp[3];
        if ( dummyGroup.name[0] > 0 )
        {
          memcpy( &dummyGroup.name[1], &pRsp[4], dummyGroup.name[0] );
        }

        apicFreeSynchData( pRsp );
        return (&dummyGroup);
      }
    }

    apicFreeSynchData( pRsp );
  }
  return (NULL);
}

/*********************************************************************
 * @fn      aps_AddGroup
 *
 * @brief   Add a group for an endpoint
 *
 * @param   endpoint -
 * @param   group - new group
 *
 * @return  ZStatus_t
 */
ZStatus_t aps_AddGroup( uint8 endpoint, aps_Group_t *group )
{
  uint8 status;
  uint8 buf[1 + 2 + APS_GROUP_NAME_LEN];

  buf[0] = endpoint;
  buf[1] = LO_UINT16( group->ID );
  buf[2] = HI_UINT16( group->ID );
  buf[3] = group->name[0];
  memcpy( &buf[4], &group->name[1], buf[3] );

  status = (uint8) sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_ADD_GROUP,
      (1 + 2 + APS_GROUP_NAME_LEN), buf );

  return (status);
}

/*********************************************************************
 * @fn      aps_CountAllGroups
 *
 * @brief   Count the total number of groups
 *
 * @param   none
 *
 * @return  number of groups
 */
uint8 aps_CountAllGroups( void )
{
  uint8 status;

  status = (uint8) sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_COUNT_ALL_GROUPS,
      0, NULL );

  return (status);
}

/*********************************************************************
 * @fn      zdoExtRxIdle
 *
 * @brief   Read or set the RxOnIdle value in the MAC
 *
 * @param   setFlag - TRUE to set the value in setValue, FALSE to return the value
 * @param   setValue - if setFlag is true
 *
 * @return  the value of RxOnIdle if setFlag is FALSE, ignore if not
 */
uint8 zdoExtRxIdle( uint8 setFlag, uint8 setValue )
{
  uint8 status;
  uint8 buf[2];

  buf[0] = setFlag;
  buf[1] = setValue;

  status = (uint8) sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_RX_IDLE, 2, buf );

  return (status);
}

/******************************************************************************
 * @fn          ZDSecMgrAPSRemove
 *
 * @brief       Remove device from network.
 *
 * @param       nwkAddr - device's NWK address
 * @param       extAddr - device's Extended address
 * @param       parentAddr - parent's NWK address
 *
 * @return      ZStatus_t
 */
ZStatus_t ZDSecMgrAPSRemove( uint16 nwkAddr, uint8 *extAddr, uint16 parentAddr )
{
  uint8 status;
  uint8 buf[12];

  buf[0] = LO_UINT16( parentAddr );
  buf[1] = HI_UINT16( parentAddr );
  buf[2] = LO_UINT16( nwkAddr );
  buf[3] = HI_UINT16( nwkAddr );
  osal_memcpy( &buf[4], extAddr, 8 );

  status = (uint8) sendNPIExpectDefaultStatusZDO( MT_ZDO_EXT_SEC_APS_REMOVE_REQ,
      12, buf );

  return (status);
}

/******************************************************************************
 * @fn          zdoExtNwkInfo
 *
 * @brief       Get the NIB info fields.
 *
 */
void zdoExtNwkInfo( nwkIB_t *pNib )
{
  uint8 *pBuf, rspcmdid;

  // send request to NPI synchronously
  pBuf = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_ZDO, MT_ZDO_EXT_NWK_INFO,
      0, NULL, NULL, &rspcmdid, NULL );

  if ( rspcmdid == MT_ZDO_EXT_NWK_INFO )
  {
    pNib->nwkDevAddress = BUILD_UINT16( pBuf[0], pBuf[1] );
    pBuf += 2;
    pNib->nwkState = *pBuf++;
    pNib->nwkPanId = BUILD_UINT16( pBuf[0], pBuf[1] );
    pBuf += 2;
    pNib->nwkCoordAddress = BUILD_UINT16( pBuf[0], pBuf[1] );
    pBuf += 2;
    memcpy( pNib->extendedPANID, pBuf, 8 );
    pBuf += 8;
    memcpy( pNib->nwkCoordExtAddress, pBuf, 8 );
    pBuf += 8;
    pNib->nwkLogicalChannel = *pBuf;
  }
}

/*********************************************************************
 * @fn      ZDApp_ForceConcentratorChange()
 *
 * @brief   Force a network concentrator change by resetting
 *          zgConcentratorEnable and zgConcentratorDiscoveryTime
 *          from NV and set nwk event.
 *
 * @param   none
 *
 * @return  none
 */
void ZDApp_ForceConcentratorChange( void )
{
  sendNPIExpectDefaultStatusZDO( MT_ZDO_FORCE_CONCENTRATOR_CHANGE, 0, NULL );
}

/**************************************************************************************************
 */

