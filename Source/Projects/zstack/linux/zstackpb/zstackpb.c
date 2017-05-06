/******************************************************************************
 Filename:       zstackpb.c
 Revised:        $Date$
 Revision:       $Revision: 29217 $

 Description:    ZStack Protobuf interface


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
 ******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_rpc.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#include "ZGlobals.h"
#include "AF.h"
#include "ZDProfile.h"
#include "ZDObject.h"
#include "ZDApp.h"
#include "ZDSecMgr.h"
#include "rtg.h"
#include "aps_groups.h"

#include "zstack.pb-c.h"
#include "api_server.h"
#include "zstackpb.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if !defined ( ZDSECMGR_TC_DEVICE_MAX )
#define ZDSECMGR_TC_DEVICE_MAX 1
#endif

#if !defined ( MAX_SUPPORTED_ENDPOINTS )
#define MAX_SUPPORTED_ENDPOINTS 20
#endif

#define ZS_ZDO_SRC_RTG_IND_CBID         0x0001
#define ZS_ZDO_CONCENTRATOR_IND_CBID    0x0002
#define ZS_ZDO_NWK_DISCOVERY_CNF_CBID   0x0004
#define ZS_ZDO_BEACON_NOTIFY_IND_CBID   0x0008
#define ZS_ZDO_JOIN_CNF_CBID            0x0010
#define ZS_ZDO_LEAVE_CNF_CBID           0x0020
#define ZS_ZDO_LEAVE_IND_CBID           0x0040

#define ZS_ZDO_NWK_ADDR_RSP_CDID            0x00000001
#define ZS_ZDO_IEEE_ADDR_RSP_CDID           0x00000002
#define ZS_ZDO_NODE_DESC_RSP_CDID           0x00000004
#define ZS_ZDO_POWER_DESC_RSP_CDID          0x00000008
#define ZS_ZDO_SIMPLE_DESC_RSP_CDID         0x00000010
#define ZS_ZDO_ACTIVE_EP_RSP_CDID           0x00000020
#define ZS_ZDO_MATCH_DESC_RSP_CDID          0x00000040
#define ZS_ZDO_COMPLEX_DESC_RSP_CDID        0x00000080
#define ZS_ZDO_USER_DESC_RSP_CDID           0x00000100
#define ZS_ZDO_DISCOVERY_CACHE_RSP_CDID     0x00000200
#define ZS_ZDO_USER_DESC_CONF_CDID          0x00000400
#define ZS_ZDO_SERVER_DISCOVERY_RSP_CDID    0x00000800
#define ZS_ZDO_END_DEVICE_TIMEOUT_RSP_CDID  0x00001000
#define ZS_ZDO_BIND_RSP_CDID                0x00002000
#define ZS_ZDO_END_DEVICE_BIND_RSP_CDID     0x00004000
#define ZS_ZDO_UNBIND_RSP_CDID              0x00008000
#define ZS_ZDO_MGMT_NWK_DISC_RSP_CDID       0x00010000
#define ZS_ZDO_MGMT_LQI_RSP_CDID            0x00020000
#define ZS_ZDO_MGMT_RTG_RSP_CDID            0x00040000
#define ZS_ZDO_MGMT_BIND_RSP_CDID           0x00080000
#define ZS_ZDO_MGMT_LEAVE_RSP_CDID          0x00100000
#define ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID    0x00200000
#define ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID    0x00400000
#define ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID  0x00800000
#define ZS_ZDO_DEVICE_ANNOUNCE_CDID         0x01000000
#define ZS_DEV_STATE_CHANGE_CDID            0x02000000
#define ZS_DEV_JAMMER_IND_CDID              0x04000000
#define ZS_TC_DEVICE_IND_CDID               0x08000000
#define ZS_DEV_PERMIT_JOIN_IND_CDID         0x10000000

/* Capability Information */
#define CAPABLE_PAN_COORD       0x01  /* Device is capable of becoming a PAN coordinator */
#define CAPABLE_FFD             0x02  /* Device is an FFD */
#define CAPABLE_MAINS_POWER     0x04  /* Device is mains powered rather than battery powered */
#define CAPABLE_RX_ON_IDLE      0x08  /* Device has its receiver on when idle */
#define CAPABLE_SECURITY        0x40  /* Device is capable of sending and receiving secured frames */
#define CAPABLE_ALLOC_ADDR      0x80  /* Request allocation of a short address in the associate procedure */

#define MTO_ROUTE           0x01       // Used in option of NLME_RouteDiscoveryRequest()
#define NO_ROUTE_CACHE      0x02       // Used in option of NLME_RouteDiscoveryRequest()
#define MULTICAST_ROUTE     0x40       // Used in all three places
#define RT_ACTIVE     1

#define JAMMER_DETECT_CONTINUOUS_EVENTS          150
#define JAMMER_DETECT_PERIOD_TIME                100  // In milliseconds
#define JAMMER_HIGH_NOISE_LEVEL                  60

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  void *next;                // Next in the link List
  int connection;
  uint16 zdoCBs;
  uint32 zdoRsps;
  endPointDesc_t epDesc;
} epItem_t;

// This is also defined in ZDSecMgr.c
typedef struct
{
  uint16 ami;
  uint16 keyNvId;   // index to the Link Key table in NV
  ZDSecMgr_Authentication_Option authenticateOption;
} ZDSecMgrEntry_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 zspbPermitJoin = TRUE;
uint8 nwkUseMultiCast = TRUE;

uint16 jammerContinuousEvents = JAMMER_DETECT_CONTINUOUS_EVENTS;
uint8 jammerHighNoiseLevel = JAMMER_HIGH_NOISE_LEVEL;
uint32 jammerDetectPeriodTime = JAMMER_DETECT_PERIOD_TIME;
uint8 maxSupportedEndpoints = MAX_SUPPORTED_ENDPOINTS;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern nwkIB_t _NIB;

#if defined ( LINUX_ZNP )
extern uint8 deviceType;
#endif

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern ZStatus_t ZDSecMgrEntryLookupExt( uint8* extAddr,
                                         ZDSecMgrEntry_t** entry );
extern void ZMacSetJammerParameters( uint16 contEvents, int8 highNoiseLevel,
                                     uint16 detectPeriod );

#if defined ( LINUX_ZNP )
extern uint8 znpSetTxPower( uint8 txPwr );
extern void znpReset( uint8 mode, uint8 nvReset, uint8 shutdown );
extern uint8 znpSniffer( uint8 mode );
extern uint8 zdoExtRxIdle( uint8 setFlag, uint8 setValue );
void zdoExtNwkInfo( nwkIB_t *pNib );
extern uint8 znpZDOSetParam( uint8 has_useMultiCast, uint8 useMultiCast );
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 zspbTaskID;   // Task ID for task/event processing

#if defined( HOLD_AUTO_START )
static devStates_t newDevState = DEV_HOLD;
#else
static devStates_t newDevState = DEV_INIT;
#endif

static epItem_t *pEpTableHdr = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void *zdoConcentratorIndCB( void *pStr );
void *zdoSrcRtgCB( void *pStr );
void *zdoJoinCnfCB( void *pStr );
void *zdoLeaveIndCB( void *pStr );
void *sendDevPermitJoinInd( void *params );
void *sendTcDeviceInd( void *params );
void sendDeviceAnnounce( uint16 srcAddr, ZDO_DeviceAnnce_t *pDevAnn );
void *zdoNwkDiscCnfCB( void *pStr );
void *zdoBeaconNotifyIndCB( void *pStr );
void *zdoLeaveCnfCB( void *pStr );

static void zspbProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void processAfDataConfirm( afDataConfirm_t *pkt );
static void processDevStateChangeInd( uint8 state );
static void processAfIncomingMsgInd( afIncomingMSGPacket_t *pkt );
static void sendDefaultRsp( int connection, uint8 cmdId, ZStatusValues status );

static void processSysResetReq( int connection, SysResetReq *pMsg );
static void processDevStartReq( int connection, DevStartReq *pMsg );
static void processSysSetTxPowerReq( int connection, SysSetTxPowerReq *pMsg );
static void processApsRemoveGroup( int connection, ApsRemoveGroup *pMsg );
static void processApsRemoveAllGroups( int connection,
                                       ApsRemoveAllGroups *pMsg );
static void processApsFindAllGroupsReq( int connection,
                                        ApsFindAllGroupsReq *pMsg );
static void processApsFindGroupReq( int connection, ApsFindGroupReq *pMsg );
static void processApsAddGroup( int connection, ApsAddGroup *pMsg );
static void processApsCountAllGroups( int connection );

static void processSecApsRemoveReq( int connection, SecApsRemoveReq *pMsg );
static void processSecNwkKeySetReq( int connection, SecNwkKeySetReq *pMsg );
static void processSecNwkKeyGetReq( int connection, SecNwkKeyGetReq *pMsg );
static void processSecNwkKeyUpdateReq( int connection,
                                       SecNwkKeyUpdateReq *pMsg );
static void processSecNwkKeySwitchReq( int connection,
                                       SecNwkKeySwitchReq *pMsg );
static void processSecApsLinkKeyGetReq( int connection,
                                        SecApsLinkKeyGetReq *pMsg );
static void processSecApsLinkKeySetReq( int connection,
                                        SecApsLinkKeySetReq *pMsg );
static void processSecApsLinkKeyRemoveReq( int connection,
                                           SecApsLinkKeyRemoveReq *pMsg );
static void processDevNwkRouteReq( int connection, DevNwkRouteReq *pMsg );
static void processDevNwkCheckRouteReq( int connection,
                                        DevNwkCheckRouteReq *pMsg );
static void processSysConfigReadReq( int connection, SysConfigReadReq *pMsg );
static void processSysConfigWriteReq( int connection, SysConfigWriteReq *pMsg );
static void processAfRegisterReq( int connection, AfRegisterReq *pAfRegReq );
static void processAfUnRegisterReq( int connection, AfUnRegisterReq *pMsg );
static void processAfConfigGetReq( int connection, AfConfigGetReq *pMsg );
static void processAfConfigSetReq( int connection, AfConfigSetReq *pMsg );
static void processAfDataReq( int connection, AfDataReq *pMsg );
static void processZdoMatchDescReq( int connection, ZdoMatchDescReq *pMsg );
static void processZdoNwkAddrReq( int connection, ZdoNwkAddrReq *pMsg );
static void processZdoIeeeAddrReq( int connection, ZdoIeeeAddrReq *pMsg );
static void processZdoNodeDescReq( int connection, ZdoNodeDescReq *pMsg );
static void processZdoPowerDescReq( int connection, ZdoPowerDescReq *pMsg );
static void processZdoSimpleDescReq( int connection, ZdoSimpleDescReq *pMsg );
static void processZdoActiveEndpointsReq( int connection,
                                          ZdoActiveEndpointReq *pMsg );
static void processZdoComplexDescReq( int connection, ZdoComplexDescReq *pMsg );
static void processZdoUserDescSetReq( int connection, ZdoUserDescSetReq *pMsg );
static void processZdoMgmtPermitJoinReq( int connection,
                                         ZdoMgmtPermitJoinReq *pMsg );
static void processZdoMgmtLeaveReq( int connection, ZdoMgmtLeaveReq *pMsg );
static void processZdoMgmtNwkDiscReq( int connection, ZdoMgmtNwkDiscReq *pMsg );
static void processZdoMgmtLqiReq( int connection, ZdoMgmtLqiReq *pMsg );
static void processZdoMgmtRtgReq( int connection, ZdoMgmtRtgReq *pMsg );
static void processZdoMgmtBindReq( int connection, ZdoMgmtBindReq *pMsg );
static void processZdoMgmtNwkUpdateReq( int connection,
                                        ZdoMgmtNwkUpdateReq *pMsg );
static void processZdoMgmtDirectJoinReq( int connection,
                                         ZdoMgmtDirectJoinReq *pMsg );
static void processZdoDeviceAnnounceReq( int connection,
                                         ZdoDeviceAnnounceReq *pMsg );

static void processZdoServerDiscReq( int connection, ZdoServerDiscReq *pMsg );
static void processZdoEndDeviceBindReq( int connection,
                                        ZdoEndDeviceBindReq *pMsg );
static void processZdoBindReq( int connection, ZdoBindReq *pMsg );
static void processZdoUnbindReq( int connection, ZdoUnbindReq *pMsg );
static void processZdoUserDescReq( int connection, ZdoUserDescReq *pMsg );
static void processDevZDOCBReq( int connection, DevZDOCBReq *pMsg );
static void sendMsgToAllCBs( int len, uint8 *pBuf, uint16 cbMask, uint8 cmdId );
static void sendMsgToAllCBMsgs( int len, uint8 *pBuf, uint32 cbMask,
                                uint8 cmdId );
static void sendNwkAddrRsp( ZDO_NwkIEEEAddrResp_t *pAddrRsp );
static void sendIeeeAddrRsp( ZDO_NwkIEEEAddrResp_t *pAddrRsp );
static void sendNodeDescRsp( uint16 srcAddr, ZDO_NodeDescRsp_t *pNdRsp );
static void sendPowerDescRsp( uint16 srcAddr, ZDO_PowerRsp_t *pPowerRsp );
static void sendSimpleDescRsp( uint16 srcAddr, ZDO_SimpleDescRsp_t *pSimpleRsp );
static void sendActiveEPRsp( uint16 srcAddr,
                             ZDO_ActiveEndpointRsp_t *pActiveEPRsp );
static void sendMatchDescRsp( uint16 srcAddr,
                              ZDO_ActiveEndpointRsp_t *pActiveEPRsp );
static void sendUserDescRsp( uint16 srcAddr, ZDO_UserDescRsp_t *pUdRsp );
static void sendServerDiscRsp( uint16 srcAddr, ZDO_ServerDiscRsp_t *pSdRsp );
static void sendEndDeviceTimeoutRsp( uint16 srcAddr, uint16 timeout );
static void sendBindRsp( uint16 srcAddr, uint8 result );
static void sendEndDeviceBindRsp( uint16 srcAddr, uint8 result );
static void sendUnbindRsp( uint16 srcAddr, uint8 result );
void sendDeviceAnnounce( uint16 srcAddr, ZDO_DeviceAnnce_t *pDevAnn );
static void sendNwkInfoRsp( int connection );
static void sendMgmtNwkDiscRsp( uint16 srcAddr,
                                ZDO_MgmNwkDiscRsp_t *pNwkDiscRsp );
static void sendMgmtLqiRsp( uint16 srcAddr, ZDO_MgmtLqiRsp_t *pLqiRsp );
static void sendMgmtRtgRsp( uint16 srcAddr, ZDO_MgmtRtgRsp_t *pRtgRsp );
static void sendMgmtBindRsp( uint16 srcAddr, ZDO_MgmtBindRsp_t *pBindRsp );
static void sendMgmtLeaveRsp( uint16 srcAddr, uint8 result );
static void sendMgmtDirectJoinRsp( uint16 srcAddr, uint8 result );
static void sendMgmtPermitJoinRsp( uint16 srcAddr, uint8 result );
static void sendMgmtNwkUpdateNotify( uint16 srcAddr,
                                     ZDO_MgmtNwkUpdateNotify_t *pNotify );

static uint8 epTableAddNewEntry( epItem_t *newEntry );
static epItem_t *epTableFindEntryEP( uint8 ep );
static epItem_t *epTableFindEntryConnection( int connection );
static void epTableRemoveEntry( epItem_t *entry );
static uint8 epTableNumEntries( void );
static void freeEpItem( epItem_t *pItem );
static void buildPBcapInfo( uint8 cInfo, CapabilityInfo *pPBcapInfo );
static void buildPBserverCap( uint16 sInfo, ServerCapabilities *pPBsrvCap );
static void buildPBpowerSource( uint8 pInfo, PowerSource *pPBpwrSrc );
static uint8 convertPBTransOptions( TransOptions *pOptions );
static uint16 convertPBServerCapabilities( ServerCapabilities *pSrvCap );
uint8 convertCapabilityInfo( CapabilityInfo *pPBcapInfo );
void znpInit( uint8 triggerStartEvent );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      zspbInit
 *
 * @brief   Initialization function for the ZStack Protobuf Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void zspbInit( uint8 task_id )
{
  zspbTaskID = task_id;

  // Register for ZDO Function Callbacks
  ZDO_RegisterForZdoCB( ZDO_SRC_RTG_IND_CBID, zdoSrcRtgCB );
  ZDO_RegisterForZdoCB( ZDO_CONCENTRATOR_IND_CBID, zdoConcentratorIndCB );
  ZDO_RegisterForZdoCB( ZDO_JOIN_CNF_CBID, zdoJoinCnfCB );
  ZDO_RegisterForZdoCB( ZDO_LEAVE_IND_CBID, zdoLeaveIndCB );
  ZDO_RegisterForZdoCB( ZDO_PERMIT_JOIN_CBID, sendDevPermitJoinInd );
  ZDO_RegisterForZdoCB( ZDO_TC_DEVICE_CBID, sendTcDeviceInd );

  // TBD: these either have a return parameter that needs to be evaluated
  //      or they interrupt the normal processing of these indications/confirms.
  //ZDO_RegisterForZdoCB( ZDO_NWK_DISCOVERY_CNF_CBID, zdoNwkDiscCnfCB );
  //ZDO_RegisterForZdoCB( ZDO_BEACON_NOTIFY_IND_CBID, zdoBeaconNotifyIndCB );
  //ZDO_RegisterForZdoCB( ZDO_LEAVE_CNF_CBID, zdoLeaveCnfCB );

  // Register for ZDO Rsp messages
  ZDO_RegisterForZDOMsg( zspbTaskID, ZDO_ALL_MSGS_CLUSTERID );

#if defined ( LINUX_ZNP )
#if defined ( HOLD_AUTO_START )
  znpInit( FALSE );
#else
  znpInit( TRUE );
#endif
#endif

  // Update Jammer parameters
  ZMacSetJammerParameters( jammerContinuousEvents, (jammerHighNoiseLevel * -1),
      jammerDetectPeriodTime );
}

#if defined ( LINUX_ZNP )
/*********************************************************************
 * @fn      znpInit
 *
 * @brief   Initialize the ZNP
 *
 * @param   triggerStartEvent - set the Start Event
 *
 * @return  none
 */
void znpInit( uint8 triggerStartEvent )
{
  uiPrintfEx(trINFO, "zstackpb config: defaultChannelList:%x, configPANID:%x, deviceType:%d\n",
      zgDefaultChannelList, zgConfigPANID, deviceType );

  osal_nv_write( ZCD_NV_CHANLIST, 0, sizeof ( zgDefaultChannelList ), &zgDefaultChannelList );
  osal_nv_write( ZCD_NV_PANID, 0, sizeof ( zgConfigPANID ), &zgConfigPANID );
  osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0, sizeof ( deviceType ), &deviceType );

  if ( triggerStartEvent )
  {
    osal_set_event( zspbTaskID, ZSPB_START_EVENT );
  }
}
#endif

/*********************************************************************
 * @fn      zspbProcessEvent
 *
 * @brief   ZStack Protobuf Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 zspbProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt = (afIncomingMSGPacket_t *) osal_msg_receive(
        zspbTaskID );
    if ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          zspbProcessZDOMsgs( (zdoIncomingMsg_t *) MSGpkt );
          break;

        case AF_DATA_CONFIRM_CMD:
          processAfDataConfirm( (afDataConfirm_t *) MSGpkt );
          break;

        case AF_INCOMING_MSG_CMD:
          processAfIncomingMsgInd( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          {
            if ( (newDevState != MSGpkt->hdr.status)
                && ((MSGpkt->hdr.status == DEV_ZB_COORD)
                    || (MSGpkt->hdr.status == DEV_ROUTER)
                    || (MSGpkt->hdr.status == DEV_END_DEVICE)) )
            {
              zAddrType_t dstAddr;
              uint8 duration = 0;

              if ( zspbPermitJoin )
              {
                duration = 0xFF;
              }

              dstAddr.addrMode = Addr16Bit;
              dstAddr.addr.shortAddr = NLME_GetShortAddr();

              ZDP_MgmtPermitJoinReq( &dstAddr, duration, 0, 0 );
            }

            newDevState = MSGpkt->hdr.status;
            processDevStateChangeInd( newDevState );

#if defined ( LINUX_ZNP )
            znpZDOSetParam( TRUE, nwkUseMultiCast );
#else
            _NIB.nwkUseMultiCast = nwkUseMultiCast;
#endif
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *) MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

#if defined ( LINUX_ZNP )
  if ( events & ZSPB_START_EVENT )
  {
    ZDApp_StartJoiningCycle();

    // return unprocessed events
    return (events ^ ZSPB_START_EVENT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zspbHandlePbCb
 *
 * @brief   Handles incoming ZStack Protobuf messages.
 *
 * @param   connection - connection handle (tcp)
 * @param   subSys - subsystem ID of the incoming message
 * @param   cmdId - command ID of the incoming message
 * @param   len - length in bytes of the incoming message payload
 * @param   pData - pointer to the message payload
 * @param   type - reason for the callback (SERVER_DATA, SERVER_CONNECT, or SERVER_DISCONNECT)
 *
 * @return  none
 */
void zspbHandlePbCb( int connection, uint8 subSys, uint8 cmdId, uint16 len,
                     uint8 *pData, uint8 type )
{
  if ( type == SERVER_CONNECT )
  {
    uiPrintfEx(trINFO, "zstackpb zspbHandlePbCb - connected: %d\n", connection );
  }
  else if ( type == SERVER_DISCONNECT )
  {
    epItem_t *pEp;
    // Connection was removed, remove all endpoints associated with connection
    do
    {
      pEp = epTableFindEntryConnection( connection );
      if ( pEp )
      {
        // Unregister the Endpoint
        afDelete( pEp->epDesc.endPoint );

        // Remove from connection table
        epTableRemoveEntry( pEp );
      }
    } while ( pEp );
    return;
  }
  else if ( type == SERVER_DATA )
  {
    uiPrintfEx(trINFO, "zstackpb zspbHandlePbCb: subsystemID:%x, cmdId:%x\n", subSys, cmdId );

    // Filter for only ZStack Protobuf messages
    if ( (subSys & RPC_SUBSYSTEM_MASK) != ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF )
    {
      return;
    }

    switch ( cmdId )
    {
      case ZSTACK_CMD_IDS__SYS_VERSION_REQ:
      case ZSTACK_CMD_IDS__SYS_VERSION_RSP:
      case ZSTACK_CMD_IDS__DEV_NWK_DISCOVERY_REQ:
      case ZSTACK_CMD_IDS__DEV_JOIN_REQ:
      case ZSTACK_CMD_IDS__DEV_REJOIN_REQ:
      case ZSTACK_CMD_IDS__AF_INTERPAN_CTL_REQ:
      default:
        {
          uiPrintfEx(trINFO, "zstackpb Rcvd - Unknown: %d\n", cmdId );
          sendDefaultRsp( connection, cmdId, ZSTATUS_VALUES__ZUnsupportedMode );
        }
        break;

      case ZSTACK_CMD_IDS__SYS_RESET_REQ:
        {
          SysResetReq *pResetReq;
          pResetReq = sys_reset_req__unpack( NULL, len, pData );
          if ( pResetReq )
          {
            processSysResetReq( connection, pResetReq );
            sys_reset_req__free_unpacked( pResetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__DEV_START_REQ:
        {
          DevStartReq *pStartReq;
          pStartReq = dev_start_req__unpack( NULL, len, pData );
          if ( pStartReq )
          {
            processDevStartReq( connection, pStartReq );
            dev_start_req__free_unpacked( pStartReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SYS_SET_TX_POWER_REQ:
        {
          SysSetTxPowerReq *pTxReq;
          pTxReq = sys_set_tx_power_req__unpack( NULL, len, pData );
          if ( pTxReq )
          {
            processSysSetTxPowerReq( connection, pTxReq );
            sys_set_tx_power_req__free_unpacked( pTxReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_REMOVE_GROUP:
        {
          ApsRemoveGroup *pRemoveGroup;
          pRemoveGroup = aps_remove_group__unpack( NULL, len, pData );
          if ( pRemoveGroup )
          {
            processApsRemoveGroup( connection, pRemoveGroup );
            aps_remove_group__free_unpacked( pRemoveGroup, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_REMOVE_ALL_GROUPS:
        {
          ApsRemoveAllGroups *pRemoveAllGroups;
          pRemoveAllGroups = aps_remove_all_groups__unpack( NULL, len, pData );
          if ( pRemoveAllGroups )
          {
            processApsRemoveAllGroups( connection, pRemoveAllGroups );
            aps_remove_all_groups__free_unpacked( pRemoveAllGroups, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_FIND_ALL_GROUPS_REQ:
        {
          ApsFindAllGroupsReq *pFindAllGroupsReq;
          pFindAllGroupsReq = aps_find_all_groups_req__unpack( NULL, len, pData );
          if ( pFindAllGroupsReq )
          {
            processApsFindAllGroupsReq( connection, pFindAllGroupsReq );
            aps_find_all_groups_req__free_unpacked( pFindAllGroupsReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_FIND_GROUP_REQ:
        {
          ApsFindGroupReq *pFindGroupReq;
          pFindGroupReq = aps_find_group_req__unpack( NULL, len, pData );
          if ( pFindGroupReq )
          {
            processApsFindGroupReq( connection, pFindGroupReq );
            aps_find_group_req__free_unpacked( pFindGroupReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_ADD_GROUP:
        {
          ApsAddGroup *pAddReq;
          pAddReq = aps_add_group__unpack( NULL, len, pData );
          if ( pAddReq )
          {
            processApsAddGroup( connection, pAddReq );
            aps_add_group__free_unpacked( pAddReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__APS_COUNT_ALL_GROUPS:
        processApsCountAllGroups( connection );
        break;

      case ZSTACK_CMD_IDS__SEC_APS_REMOVE_REQ:
        {
          SecApsRemoveReq *pRemoveReq;
          pRemoveReq = sec_aps_remove_req__unpack( NULL, len, pData );
          if ( pRemoveReq )
          {
            processSecApsRemoveReq( connection, pRemoveReq );
            sec_aps_remove_req__free_unpacked( pRemoveReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_NWK_KEY_UPDATE_REQ:
        {
          SecNwkKeyUpdateReq *pUpdateReq;
          pUpdateReq = sec_nwk_key_update_req__unpack( NULL, len, pData );
          if ( pUpdateReq )
          {
            processSecNwkKeyUpdateReq( connection, pUpdateReq );
            sec_nwk_key_update_req__free_unpacked( pUpdateReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_NWK_KEY_SWITCH_REQ:
        {
          SecNwkKeySwitchReq *pSwitchReq;
          pSwitchReq = sec_nwk_key_switch_req__unpack( NULL, len, pData );
          if ( pSwitchReq )
          {
            processSecNwkKeySwitchReq( connection, pSwitchReq );
            sec_nwk_key_switch_req__free_unpacked( pSwitchReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_NWK_KEY_SET_REQ:
        {
          SecNwkKeySetReq *pSetReq;
          pSetReq = sec_nwk_key_set_req__unpack( NULL, len, pData );
          if ( pSetReq )
          {
            processSecNwkKeySetReq( connection, pSetReq );
            sec_nwk_key_set_req__free_unpacked( pSetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_NWK_KEY_GET_REQ:
        {
          SecNwkKeyGetReq *pGetReq;
          pGetReq = sec_nwk_key_get_req__unpack( NULL, len, pData );
          if ( pGetReq )
          {
            processSecNwkKeyGetReq( connection, pGetReq );
            sec_nwk_key_get_req__free_unpacked( pGetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_APS_LINKKEY_GET_REQ:
        {
          SecApsLinkKeyGetReq *pGetReq;
          pGetReq = sec_aps_link_key_get_req__unpack( NULL, len, pData );
          if ( pGetReq )
          {
            processSecApsLinkKeyGetReq( connection, pGetReq );
            sec_aps_link_key_get_req__free_unpacked( pGetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_APS_LINKKEY_SET_REQ:
        {
          SecApsLinkKeySetReq *pSetReq;
          pSetReq = sec_aps_link_key_set_req__unpack( NULL, len, pData );
          if ( pSetReq )
          {
            processSecApsLinkKeySetReq( connection, pSetReq );
            sec_aps_link_key_set_req__free_unpacked( pSetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SEC_APS_LINKKEY_REMOVE_REQ:
        {
          SecApsLinkKeyRemoveReq *pRemoveReq;
          pRemoveReq = sec_aps_link_key_remove_req__unpack( NULL, len, pData );
          if ( pRemoveReq )
          {
            processSecApsLinkKeyRemoveReq( connection, pRemoveReq );
            sec_aps_link_key_remove_req__free_unpacked( pRemoveReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__DEV_NWK_ROUTE_REQ:
        {
          DevNwkRouteReq *pRouteReq;
          pRouteReq = dev_nwk_route_req__unpack( NULL, len, pData );
          if ( pRouteReq )
          {
            processDevNwkRouteReq( connection, pRouteReq );
            dev_nwk_route_req__free_unpacked( pRouteReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__DEV_NWK_CHECK_ROUTE_REQ:
        {
          DevNwkCheckRouteReq *pCheckRouteReq;
          pCheckRouteReq = dev_nwk_check_route_req__unpack( NULL, len, pData );
          if ( pCheckRouteReq )
          {
            processDevNwkCheckRouteReq( connection, pCheckRouteReq );
            dev_nwk_check_route_req__free_unpacked( pCheckRouteReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SYS_CONFIG_READ_REQ:
        {
          SysConfigReadReq *pSysConfigReadReq;
          pSysConfigReadReq = sys_config_read_req__unpack( NULL, len, pData );
          if ( pSysConfigReadReq )
          {
            processSysConfigReadReq( connection, pSysConfigReadReq );
            sys_config_read_req__free_unpacked( pSysConfigReadReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SYS_CONFIG_WRITE_REQ:
        {
          SysConfigWriteReq *pSysConfigWriteReq;
          pSysConfigWriteReq = sys_config_write_req__unpack( NULL, len, pData );
          if ( pSysConfigWriteReq )
          {
            processSysConfigWriteReq( connection, pSysConfigWriteReq );
            sys_config_write_req__free_unpacked( pSysConfigWriteReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_REQ:
        sendNwkInfoRsp( connection );
        break;

      case ZSTACK_CMD_IDS__AF_REGISTER_REQ:
        {
          AfRegisterReq *pAfRegReq;
          pAfRegReq = af_register_req__unpack( NULL, len, pData );
          if ( pAfRegReq )
          {
            processAfRegisterReq( connection, pAfRegReq );
            af_register_req__free_unpacked( pAfRegReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__AF_UNREGISTER_REQ:
        {
          AfUnRegisterReq *pAfUnRegReq;
          pAfUnRegReq = af_un_register_req__unpack( NULL, len, pData );
          if ( pAfUnRegReq )
          {
            processAfUnRegisterReq( connection, pAfUnRegReq );
            af_un_register_req__free_unpacked( pAfUnRegReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__AF_CONFIG_GET_REQ:
        {
          AfConfigGetReq *pAConfigGetReq;
          pAConfigGetReq = af_config_get_req__unpack( NULL, len, pData );
          if ( pAConfigGetReq )
          {
            processAfConfigGetReq( connection, pAConfigGetReq );
            af_config_get_req__free_unpacked( pAConfigGetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__AF_CONFIG_SET_REQ:
        {
          AfConfigSetReq *pAConfigSetReq;
          pAConfigSetReq = af_config_set_req__unpack( NULL, len, pData );
          if ( pAConfigSetReq )
          {
            processAfConfigSetReq( connection, pAConfigSetReq );
            af_config_set_req__free_unpacked( pAConfigSetReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__DEV_ZDO_CBS_REQ:
        {
          DevZDOCBReq *pDevZDOCBReq;
          pDevZDOCBReq = dev_zdocbreq__unpack( NULL, len, pData );
          if ( pDevZDOCBReq )
          {
            processDevZDOCBReq( connection, pDevZDOCBReq );
            dev_zdocbreq__free_unpacked( pDevZDOCBReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__AF_DATA_REQ:
        {
          AfDataReq *pAfDataReq;
          pAfDataReq = af_data_req__unpack( NULL, len, pData );
          if ( pAfDataReq )
          {
            processAfDataReq( connection, pAfDataReq );
            af_data_req__free_unpacked( pAfDataReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MATCH_DESC_REQ:
        {
          ZdoMatchDescReq *pReq;
          pReq = zdo_match_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMatchDescReq( connection, pReq );
            zdo_match_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_NWK_ADDR_REQ:
        {
          ZdoNwkAddrReq *pReq;
          pReq = zdo_nwk_addr_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoNwkAddrReq( connection, pReq );
            zdo_nwk_addr_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_IEEE_ADDR_REQ:
        {
          ZdoIeeeAddrReq *pReq;
          pReq = zdo_ieee_addr_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoIeeeAddrReq( connection, pReq );
            zdo_ieee_addr_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_NODE_DESC_REQ:
        {
          ZdoNodeDescReq *pReq;
          pReq = zdo_node_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoNodeDescReq( connection, pReq );
            zdo_node_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_POWER_DESC_REQ:
        {
          ZdoPowerDescReq *pReq;
          pReq = zdo_power_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoPowerDescReq( connection, pReq );
            zdo_power_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_SIMPLE_DESC_REQ:
        {
          ZdoSimpleDescReq *pReq;
          pReq = zdo_simple_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoSimpleDescReq( connection, pReq );
            zdo_simple_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_ACTIVE_ENDPOINT_REQ:
        {
          ZdoActiveEndpointReq *pReq;
          pReq = zdo_active_endpoint_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoActiveEndpointsReq( connection, pReq );
            zdo_active_endpoint_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_COMPLEX_DESC_REQ:
        {
          ZdoComplexDescReq *pReq;
          pReq = zdo_complex_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoComplexDescReq( connection, pReq );
            zdo_complex_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_USER_DESCR_SET_REQ:
        {
          ZdoUserDescSetReq *pReq;
          pReq = zdo_user_desc_set_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoUserDescSetReq( connection, pReq );
            zdo_user_desc_set_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_SERVER_DISC_REQ:
        {
          ZdoServerDiscReq *pReq;
          pReq = zdo_server_disc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoServerDiscReq( connection, pReq );
            zdo_server_disc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_END_DEVICE_BIND_REQ:
        {
          ZdoEndDeviceBindReq *pReq;
          pReq = zdo_end_device_bind_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoEndDeviceBindReq( connection, pReq );
            zdo_end_device_bind_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_BIND_REQ:
        {
          ZdoBindReq *pReq;
          pReq = zdo_bind_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoBindReq( connection, pReq );
            zdo_bind_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_UNBIND_REQ:
        {
          ZdoUnbindReq *pReq;
          pReq = zdo_unbind_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoUnbindReq( connection, pReq );
            zdo_unbind_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_USER_DESC_REQ:
        {
          ZdoUserDescReq *pReq;
          pReq = zdo_user_desc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoUserDescReq( connection, pReq );
            zdo_user_desc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_PERMIT_JOIN_REQ:
        {
          ZdoMgmtPermitJoinReq *pReq;
          pReq = zdo_mgmt_permit_join_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtPermitJoinReq( connection, pReq );
            zdo_mgmt_permit_join_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_LEAVE_REQ:
        {
          ZdoMgmtLeaveReq *pReq;
          pReq = zdo_mgmt_leave_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtLeaveReq( connection, pReq );
            zdo_mgmt_leave_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_NWK_DISC_REQ:
        {
          ZdoMgmtNwkDiscReq *pReq;
          pReq = zdo_mgmt_nwk_disc_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtNwkDiscReq( connection, pReq );
            zdo_mgmt_nwk_disc_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_LQI_REQ:
        {
          ZdoMgmtLqiReq *pReq;
          pReq = zdo_mgmt_lqi_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtLqiReq( connection, pReq );
            zdo_mgmt_lqi_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_RTG_REQ:
        {
          ZdoMgmtRtgReq *pReq;
          pReq = zdo_mgmt_rtg_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtRtgReq( connection, pReq );
            zdo_mgmt_rtg_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_BIND_REQ:
        {
          ZdoMgmtBindReq *pReq;
          pReq = zdo_mgmt_bind_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtBindReq( connection, pReq );
            zdo_mgmt_bind_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_NWK_UPDATE_REQ:
        {
          ZdoMgmtNwkUpdateReq *pReq;
          pReq = zdo_mgmt_nwk_update_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtNwkUpdateReq( connection, pReq );
            zdo_mgmt_nwk_update_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_MGMT_DIRECT_JOIN_REQ:
        {
          ZdoMgmtDirectJoinReq *pReq;
          pReq = zdo_mgmt_direct_join_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoMgmtDirectJoinReq( connection, pReq );
            zdo_mgmt_direct_join_req__free_unpacked( pReq, NULL );
          }
        }
        break;

      case ZSTACK_CMD_IDS__ZDO_DEVICE_ANNOUNCE_REQ:
        {
          ZdoDeviceAnnounceReq *pReq;
          pReq = zdo_device_announce_req__unpack( NULL, len, pData );
          if ( pReq )
          {
            processZdoDeviceAnnounceReq( connection, pReq );
            zdo_device_announce_req__free_unpacked( pReq, NULL );
          }
        }
        break;
    }
  }
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      zspbProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void zspbProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  uiPrintfEx(trINFO, "zstackpb Sending ZDO Rsp/Ind Msg - clusterID: 0x%.4X\n", inMsg->clusterID );

  switch ( inMsg->clusterID )
  {
    case Device_annce:
      {
        ZDO_DeviceAnnce_t devAnn;

        memset( &devAnn, 0, sizeof(ZDO_DeviceAnnce_t) );

        ZDO_ParseDeviceAnnce( inMsg, &devAnn );
        sendDeviceAnnounce( inMsg->srcAddr.addr.shortAddr, &devAnn );
      }
      break;

    case NWK_addr_rsp:
      {
        ZDO_NwkIEEEAddrResp_t *pAddrRsp = ZDO_ParseAddrRsp( inMsg );
        if ( pAddrRsp )
        {
          sendNwkAddrRsp( pAddrRsp );

          osal_mem_free( pAddrRsp );
        }
      }
      break;

    case IEEE_addr_rsp:
      {
        ZDO_NwkIEEEAddrResp_t *pAddrRsp = ZDO_ParseAddrRsp( inMsg );
        if ( pAddrRsp )
        {
          sendIeeeAddrRsp( pAddrRsp );

          osal_mem_free( pAddrRsp );
        }
      }
      break;

    case Node_Desc_rsp:
      {
        ZDO_NodeDescRsp_t ndRsp;

        memset( &ndRsp, 0, sizeof(ZDO_NodeDescRsp_t) );

        ZDO_ParseNodeDescRsp( inMsg, &ndRsp );
        sendNodeDescRsp( inMsg->srcAddr.addr.shortAddr, &ndRsp );
      }
      break;

    case Power_Desc_rsp:
      {
        ZDO_PowerRsp_t powerRsp;

        memset( &powerRsp, 0, sizeof(ZDO_PowerRsp_t) );

        ZDO_ParsePowerDescRsp( inMsg, &powerRsp );
        sendPowerDescRsp( inMsg->srcAddr.addr.shortAddr, &powerRsp );
      }
      break;

    case Simple_Desc_rsp:
      {
        ZDO_SimpleDescRsp_t simpleRsp;

        memset( &simpleRsp, 0, sizeof(ZDO_SimpleDescRsp_t) );

        ZDO_ParseSimpleDescRsp( inMsg, &simpleRsp );
        sendSimpleDescRsp( inMsg->srcAddr.addr.shortAddr, &simpleRsp );
      }
      break;

    case Active_EP_rsp:
      {
        ZDO_ActiveEndpointRsp_t *epRsp = ZDO_ParseEPListRsp( inMsg );
        if ( epRsp )
        {
          sendActiveEPRsp( inMsg->srcAddr.addr.shortAddr, epRsp );

          osal_mem_free( epRsp );
        }
      }
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *epRsp = ZDO_ParseEPListRsp( inMsg );
        if ( epRsp )
        {
          sendMatchDescRsp( inMsg->srcAddr.addr.shortAddr, epRsp );

          osal_mem_free( epRsp );
        }
      }
      break;

    case User_Desc_rsp:
      {
        ZDO_UserDescRsp_t *udRsp = ZDO_ParseUserDescRsp( inMsg );
        if ( udRsp )
        {
          sendUserDescRsp( inMsg->srcAddr.addr.shortAddr, udRsp );

          osal_mem_free( udRsp );
        }
      }
      break;

    case Server_Discovery_rsp:
      {
        ZDO_ServerDiscRsp_t sdRsp;

        memset( &sdRsp, 0, sizeof(ZDO_ServerDiscRsp_t) );

        ZDO_ParseServerDiscRsp( inMsg, &sdRsp );
        sendServerDiscRsp( inMsg->srcAddr.addr.shortAddr, &sdRsp );
      }
      break;

    case End_Device_Timeout_rsp:
      {
        uint16 timeout;
        ZDO_ParseEndDeviceTimeoutRsp( inMsg, &timeout );
        sendEndDeviceTimeoutRsp( inMsg->srcAddr.addr.shortAddr, timeout );
      }
      break;

    case Bind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendBindRsp( inMsg->srcAddr.addr.shortAddr, result );
      }
      break;

    case End_Device_Bind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendEndDeviceBindRsp( inMsg->srcAddr.addr.shortAddr, result );
      }
      break;

    case Unbind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendUnbindRsp( inMsg->srcAddr.addr.shortAddr, result );
      }
      break;

    case Mgmt_NWK_Disc_rsp:
      {
        ZDO_MgmNwkDiscRsp_t *pNwkDiscRsp = ZDO_ParseMgmNwkDiscRsp( inMsg );
        if ( pNwkDiscRsp )
        {
          sendMgmtNwkDiscRsp( inMsg->srcAddr.addr.shortAddr, pNwkDiscRsp );

          osal_mem_free( pNwkDiscRsp );
        }
      }
      break;

    case Mgmt_Lqi_rsp:
      {
        ZDO_MgmtLqiRsp_t *pLqiRsp = ZDO_ParseMgmtLqiRsp( inMsg );
        if ( pLqiRsp )
        {
          sendMgmtLqiRsp( inMsg->srcAddr.addr.shortAddr, pLqiRsp );

          osal_mem_free( pLqiRsp );
        }
      }
      break;

    case Mgmt_Rtg_rsp:
      {
        ZDO_MgmtRtgRsp_t *pRtgRsp = ZDO_ParseMgmtRtgRsp( inMsg );
        if ( pRtgRsp )
        {
          sendMgmtRtgRsp( inMsg->srcAddr.addr.shortAddr, pRtgRsp );

          osal_mem_free( pRtgRsp );
        }
      }
      break;

    case Mgmt_Bind_rsp:
      {
        ZDO_MgmtBindRsp_t *pBindRsp = ZDO_ParseMgmtBindRsp( inMsg );
        if ( pBindRsp )
        {
          sendMgmtBindRsp( inMsg->srcAddr.addr.shortAddr, pBindRsp );

          osal_mem_free( pBindRsp );
        }
      }
      break;

    case Mgmt_Leave_rsp:
      sendMgmtLeaveRsp( inMsg->srcAddr.addr.shortAddr,
          ZDO_ParseMgmtLeaveRsp( inMsg ) );
      break;

    case Mgmt_Direct_Join_rsp:
      sendMgmtDirectJoinRsp( inMsg->srcAddr.addr.shortAddr,
          ZDO_ParseMgmtDirectJoinRsp( inMsg ) );
      break;

    case Mgmt_Permit_Join_rsp:
      sendMgmtPermitJoinRsp( inMsg->srcAddr.addr.shortAddr,
          ZDO_ParseMgmtDirectJoinRsp( inMsg ) );
      break;

    case Mgmt_NWK_Update_notify:
      {
        ZDO_MgmtNwkUpdateNotify_t *pNwkUpdateNotifyRsp =
            ZDO_ParseMgmtNwkUpdateNotify( inMsg );
        if ( pNwkUpdateNotifyRsp )
        {
          sendMgmtNwkUpdateNotify( inMsg->srcAddr.addr.shortAddr,
              pNwkUpdateNotifyRsp );

          osal_mem_free( pNwkUpdateNotifyRsp );
        }
      }
      break;

    // TBD: Not implemented yet
    case Complex_Desc_rsp:
    case Discovery_Cache_rsp:
    default:
      break;
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      processAfDataConfirm
 *
 * @brief   process incoming AF Data Confirm
 *
 * @param   pkt - pointer to data confirm message
 *
 * @return  none
 */
static void processAfDataConfirm( afDataConfirm_t *pkt )
{
  int len;
  uint8 *pBuf;
  AfDataConfirmInd inMsg = AF_DATA_CONFIRM_IND__INIT;

  inMsg.cmdid = ZSTACK_CMD_IDS__AF_DATA_CONFIRM_IND;
  inMsg.endpoint = pkt->endpoint;
  inMsg.status = pkt->hdr.status;
  inMsg.transid = pkt->transID;

  len = af_data_confirm_ind__get_packed_size( &inMsg );
  pBuf = malloc( len );
  if ( pBuf )
  {
    epItem_t *pItem;

    af_data_confirm_ind__pack( &inMsg, pBuf );

    pItem = epTableFindEntryEP( pkt->endpoint );
    if ( pItem )
    {
      APIS_SendData( pItem->connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, TRUE,
          ZSTACK_CMD_IDS__AF_DATA_CONFIRM_IND, len, pBuf );
    }

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processDevStateChangeInd
 *
 * @brief   process incoming AF Data Confirm
 *
 * @param   state - new state
 *
 * @return  none
 */
static void processDevStateChangeInd( uint8 state )
{
  int len;
  uint8 *pBuf;
  DevStateChangeInd stateChg = DEV_STATE_CHANGE_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb: DevStateChangeInd: state:%d\n", state );

  stateChg.cmdid = ZSTACK_CMD_IDS__DEV_STATE_CHANGE_IND;
  stateChg.state = state;

  len = dev_state_change_ind__get_packed_size( &stateChg );
  pBuf = malloc( len );
  if ( pBuf )
  {
    dev_state_change_ind__pack( &stateChg, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_DEV_STATE_CHANGE_CDID,
        ZSTACK_CMD_IDS__DEV_STATE_CHANGE_IND );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processAfIncomingMsgInd
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void processAfIncomingMsgInd( afIncomingMSGPacket_t *pkt )
{
  int len;
  uint8 *pBuf;
  AFAddr srcAddr = AFADDR__INIT;
  AfIncomingMsgInd inMsg = AF_INCOMING_MSG_IND__INIT;

  inMsg.cmdid = ZSTACK_CMD_IDS__AF_INCOMING_MSG_IND;

  srcAddr.addrmode = pkt->srcAddr.addrMode;
  if ( (pkt->srcAddr.addrMode == afAddr16Bit)
      || (pkt->srcAddr.addrMode == afAddrGroup)
      || (pkt->srcAddr.addrMode == afAddrBroadcast) )
  {
    srcAddr.has_shortaddr = TRUE;
    srcAddr.shortaddr = pkt->srcAddr.addr.shortAddr;
  }
  else if ( pkt->srcAddr.addrMode == afAddr64Bit )
  {
    srcAddr.has_extaddr = TRUE;
    memcpy( &srcAddr.extaddr, pkt->srcAddr.addr.extAddr, 8 );
  }
  srcAddr.has_endpoint = TRUE;
  srcAddr.endpoint = pkt->srcAddr.endPoint;
  srcAddr.has_panid = TRUE;
  srcAddr.panid = pkt->srcAddr.panId;
  inMsg.srcaddr = &srcAddr;

  inMsg.clusterid = pkt->clusterId;
  inMsg.correlation = pkt->correlation;
  inMsg.endpoint = pkt->endPoint;
  inMsg.groupid = pkt->groupId;
  inMsg.linkquality = pkt->LinkQuality;
  inMsg.macdestaddr = pkt->macDestAddr;
  inMsg.macsrcaddr = pkt->macSrcAddr;
  inMsg.nwkseqnum = pkt->nwkSeqNum;
  inMsg.rssi = pkt->rssi;
  inMsg.timestamp = pkt->timestamp;
  inMsg.transseqnum = pkt->cmd.TransSeqNumber;

  if ( pkt->wasBroadcast )
  {
    inMsg.wasbroadcast = TRUE;
  }
  else
  {
    inMsg.wasbroadcast = FALSE;
  }

  if ( pkt->SecurityUse )
  {
    inMsg.securityuse = TRUE;
  }
  else
  {
    inMsg.securityuse = FALSE;
  }

  inMsg.payload.len = pkt->cmd.DataLength;
  inMsg.payload.data = malloc( pkt->cmd.DataLength );
  if ( inMsg.payload.data )
  {
    memcpy( inMsg.payload.data, pkt->cmd.Data, pkt->cmd.DataLength );
  }

  len = af_incoming_msg_ind__get_packed_size( &inMsg );
  pBuf = malloc( len );
  if ( pBuf )
  {
    epItem_t *pItem;

    af_incoming_msg_ind__pack( &inMsg, pBuf );

    pItem = epTableFindEntryEP( pkt->endPoint );
    if ( pItem )
    {
      APIS_SendData( pItem->connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, TRUE,
          ZSTACK_CMD_IDS__AF_INCOMING_MSG_IND, len, pBuf );
    }

    free( pBuf );
  }

  if ( inMsg.payload.data )
  {
    free( inMsg.payload.data );
  }
}

/***************************************************************************************************
 * @fn          sendDefaultRsp
 *
 * @brief       Encode and send the default response to the message
 *
 * @param       connection - TCP connection handle
 * @param       cmdId - outgoing command ID
 * @param       status - outgoing status field value
 *
 * @return      None
 ***************************************************************************************************/
static void sendDefaultRsp( int connection, uint8 cmdId, ZStatusValues status )
{
  int len;
  uint8 *pBuf;
  ZstackDefaultRsp defaultRsp = ZSTACK_DEFAULT_RSP__INIT;

  defaultRsp.cmdid = cmdId;
  defaultRsp.status = status;

  len = zstack_default_rsp__get_packed_size( &defaultRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zstack_default_rsp__pack( &defaultRsp, pBuf );

    // Send the response message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE, cmdId,
        len, pBuf );

    free( pBuf );
  }
  else
  {
    uiPrintfEx(trINFO, "zstackpb sendDefaultRsp: mem alloc error\n" );
  }
}

/*********************************************************************
 * @fn      processSysResetReq
 *
 * @brief   Process incoming System Reset Request message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSysResetReq( int connection, SysResetReq *pMsg )
{
  uint8 status = ZSTATUS_VALUES__ZSuccess;
#if defined ( LINUX_ZNP )
  uint8 shutdown = 0;

  if ( pMsg->has_shutdown )
  {
    shutdown = pMsg->shutdown;
  }

  znpReset( 1, pMsg->newnwkstate, shutdown );
#else
  status = ZSTATUS_VALUES__ZUnsupportedMode;
#endif

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processDevStartReq
 *
 * @brief   Process incoming Dev Start Request message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processDevStartReq( int connection, DevStartReq *pMsg )
{
  uint8 status = ZSTATUS_VALUES__ZSuccess;

#if defined ( HOLD_AUTO_START )
#if defined ( LINUX_ZNP )
  osal_set_event( zspbTaskID, ZSPB_START_EVENT );
#else
  ZDOInitDevice( (uint16)pMsg->startdelay );
#endif
#else
  status = ZSTATUS_VALUES__ZUnsupportedMode;
#endif

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processSysSetTxPowerReq
 *
 * @brief   Process incoming System Set Tx Power message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSysSetTxPowerReq( int connection, SysSetTxPowerReq *pMsg )
{
  int len;
  uint8 *pBuf;
  SysSetTxPowerRsp txPowerRsp = SYS_SET_TX_POWER_RSP__INIT;
  int8 txPwr;

  txPwr = (int8) pMsg->requestedtxpower;

  txPowerRsp.cmdid = ZSTACK_CMD_IDS__SYS_SET_TX_POWER_RSP;

#if defined ( LINUX_ZNP )
  txPowerRsp.txpower = znpSetTxPower( (uint8)txPwr );
#else
  (void) ZMacSetTransmitPower( (ZMacTransmitPower_t) txPwr );
  txPowerRsp.txpower = txPwr; // We don't have a corrected value, so use the requested
#endif

  len = sys_set_tx_power_rsp__get_packed_size( &txPowerRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    sys_set_tx_power_rsp__pack( &txPowerRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__SYS_SET_TX_POWER_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processApsRemoveGroup
 *
 * @brief   Process incoming APS Remove Group message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processApsRemoveGroup( int connection, ApsRemoveGroup *pMsg )
{
  ZStatusValues status = ZSTATUS_VALUES__ZInvalidParameter;

  if ( aps_RemoveGroup( pMsg->endpoint, pMsg->groupid ) )
  {
    status = ZSTATUS_VALUES__ZSuccess;
  }

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processApsRemoveAllGroups
 *
 * @brief   Process incoming APS Remove All Groups message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processApsRemoveAllGroups( int connection,
                                       ApsRemoveAllGroups *pMsg )
{
  aps_RemoveAllGroup( pMsg->endpoint );

  sendDefaultRsp( connection, pMsg->cmdid, ZSTATUS_VALUES__ZSuccess );
}

/*********************************************************************
 * @fn      processApsFindAllGroupsReq
 *
 * @brief   Process incoming APS Find All Groups Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processApsFindAllGroupsReq( int connection,
                                        ApsFindAllGroupsReq *pMsg )
{
  int len;
  uint8 *pBuf;
  ApsFindAllGroupsRsp apsGroupsRsp = APS_FIND_ALL_GROUPS_RSP__INIT;
  uint8 groups = 0;
  uint16 groupList[APS_MAX_GROUPS] =  { 0 };
  uint32_t *pGroups = NULL;

  apsGroupsRsp.cmdid = ZSTACK_CMD_IDS__APS_FIND_ALL_GROUPS_RSP;

  groups = aps_FindAllGroupsForEndpoint( (uint8) pMsg->endpoint, groupList );
  if ( groups )
  {
    pGroups = malloc( sizeof(uint32_t) * groups );
    if ( pGroups )
    {
      int x;
      apsGroupsRsp.n_grouplist = groups;
      for ( x = 0; x < groups; x++ )
      {
        pGroups[x] = groupList[x];
      }
      apsGroupsRsp.grouplist = pGroups;
    }
  }

  len = aps_find_all_groups_rsp__get_packed_size( &apsGroupsRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_find_all_groups_rsp__pack( &apsGroupsRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__APS_FIND_ALL_GROUPS_RSP, len, pBuf );

    free( pBuf );
  }

  if ( pGroups )
  {
    free( pGroups );
  }
}

/*********************************************************************
 * @fn      processApsFindGroupReq
 *
 * @brief   Process incoming APS Find Group Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processApsFindGroupReq( int connection, ApsFindGroupReq *pMsg )
{
  int len;
  uint8 *pBuf;
  ApsFindGroupRsp apsGroupRsp = APS_FIND_GROUP_RSP__INIT;
  aps_Group_t *pGroup;

  apsGroupRsp.cmdid = ZSTACK_CMD_IDS__APS_FIND_GROUP_RSP;

  pGroup = aps_FindGroup( (uint8) pMsg->endpoint, (uint16) pMsg->groupid );
  if ( pGroup )
  {
    apsGroupRsp.has_groupid = TRUE;
    apsGroupRsp.groupid = pGroup->ID;
    if ( pGroup->name[0] > 0 )
    {
      apsGroupRsp.has_name = TRUE;
      apsGroupRsp.name.len = pGroup->name[0];
      apsGroupRsp.name.data = &(pGroup->name[1]);
    }
  }

  len = aps_find_group_rsp__get_packed_size( &apsGroupRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_find_group_rsp__pack( &apsGroupRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__APS_FIND_GROUP_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processApsAddGroup
 *
 * @brief   Process incoming APS Add Group message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processApsAddGroup( int connection, ApsAddGroup *pMsg )
{
  ZStatusValues status = ZSTATUS_VALUES__ZInvalidParameter;
  aps_Group_t group =  { 0 };

  group.ID = (uint16) pMsg->groupid;
  if ( pMsg->has_name )
  {
    uint8 nameLen = pMsg->name.len;
    if ( nameLen > (APS_GROUP_NAME_LEN - 1) )
    {
      nameLen = (APS_GROUP_NAME_LEN - 1);
    }
    memcpy( &(group.name[1]), pMsg->name.data, nameLen );
    group.name[0] = nameLen;
  }

  status = aps_AddGroup( pMsg->endpoint, &group );

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processApsCountAllGroups
 *
 * @brief   Process incoming APS Add Group message.
 *
 * @param   connection - connection handle (tcp)
 *
 * @return  none
 */
static void processApsCountAllGroups( int connection )
{
  ZStatusValues status = ZSTATUS_VALUES__ZInvalidParameter;

  status = (ZStatusValues) aps_CountAllGroups();

  sendDefaultRsp( connection, ZSTACK_CMD_IDS__APS_COUNT_ALL_GROUPS, status );
}

/*********************************************************************
 * @fn      processSecApsRemoveReq
 *
 * @brief   Process incoming Security Manager APS Remove Request message.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSecApsRemoveReq( int connection, SecApsRemoveReq *pMsg )
{
  ZStatusValues status;
  uint16 nwkAddr;
  uint8 extAddr[8];
  uint16 parentAddr;

  nwkAddr = pMsg->nwkaddr;
  memcpy( extAddr, &(pMsg->extaddr), 8 );
  parentAddr = pMsg->parentaddr;

#if defined ( ZDO_COORDINATOR )
  status = ZDSecMgrAPSRemove( nwkAddr, extAddr, parentAddr );
#else
  status = ZSTATUS_VALUES__ZUnsupportedMode;
#endif

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processSecNwkKeyUpdateReq
 *
 * @brief   Process incoming Security Update Network Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSecNwkKeyUpdateReq( int connection,
                                       SecNwkKeyUpdateReq *pMsg )
{
  ZStatusValues status = ZSTATUS_VALUES__ZInvalidParameter;

#if ( ZG_BUILD_COORDINATOR_TYPE )
  nwkKeyDesc nwkKey =  { 0};
  uint16 nvID = 0;
  uint8 done = FALSE;

  while ( !done )
  {
    if ( nvID == 0 )
    {
      nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
    }
    else
    {
      nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
      done = TRUE;
    }

    osal_nv_read( nvID, 0, sizeof(nwkKeyDesc), &nwkKey );

    if ( nwkKey.keySeqNum == (uint8)pMsg->seqnum )
    {
      done = TRUE;
      status = ZSTATUS_VALUES__ZSuccess;
    }
  }

  if ( status == ZSTATUS_VALUES__ZSuccess )
  {
    status = ZDSecMgrUpdateNwkKey( nwkKey.key, (uint8)pMsg->seqnum, (uint16)pMsg->dstaddr );
  }
#else
  status = ZSTATUS_VALUES__ZUnsupportedMode;
#endif

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processSecNwkKeySwitchReq
 *
 * @brief   Process incoming Security Switch Network Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSecNwkKeySwitchReq( int connection,
                                       SecNwkKeySwitchReq *pMsg )
{
  ZStatusValues status;

#if ( ZG_BUILD_COORDINATOR_TYPE )
  status = ZDSecMgrSwitchNwkKey( (uint8)pMsg->seqnum, (uint16)pMsg->dstaddr );
#else
  status = ZSTATUS_VALUES__ZUnsupportedMode;
#endif

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processSecNwkKeySetReq
 *
 * @brief   Process incoming Security Set Network Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSecNwkKeySetReq( int connection, SecNwkKeySetReq *pMsg )
{
  uint8 status;
  nwkKeyDesc nwkKey;
  uint16 nvID;

  if ( pMsg->activekey )
  {
    nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
  }
  else
  {
    nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
  }

  nwkKey.keySeqNum = pMsg->seqnum;
  if ( pMsg->has_key )
  {
    memcpy( nwkKey.key, pMsg->key.data, 16 );
  }
  else
  {
    // optional key isn't included, so generate the key
    int x;
    for ( x = 0; x < 16; x++ )
    {
      nwkKey.key[x] = osal_rand();
    }
  }

  status = osal_nv_write( nvID, 0, sizeof(nwkKeyDesc), &nwkKey );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processSecNwkKeyGetReq
 *
 * @brief   Process incoming Security Get Network Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Network Key Get Request
 *
 * @return  none
 */
static void processSecNwkKeyGetReq( int connection, SecNwkKeyGetReq *pMsg )
{
  int len;
  uint8 *pBuf;
  SecNwkKeyGetRsp nwkKeyRsp = SEC_NWK_KEY_GET_RSP__INIT;
  nwkKeyDesc nwkKey;
  uint16 nvID;

  nwkKeyRsp.cmdid = ZSTACK_CMD_IDS__SEC_NWK_KEY_GET_RSP;

  if ( pMsg->activekey )
  {
    nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
  }
  else
  {
    nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
  }

  nwkKeyRsp.status = osal_nv_read( nvID, 0, sizeof(nwkKeyDesc), &nwkKey );

  if ( nwkKeyRsp.status == ZSTATUS_VALUES__ZSuccess )
  {
    nwkKeyRsp.activekey = pMsg->activekey;
    nwkKeyRsp.seqnum = nwkKey.keySeqNum;
    nwkKeyRsp.key.len = 16;
    nwkKeyRsp.key.data = nwkKey.key;
  }

  len = sec_nwk_key_get_rsp__get_packed_size( &nwkKeyRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    sec_nwk_key_get_rsp__pack( &nwkKeyRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__SEC_NWK_KEY_GET_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processSecApsLinkKeyGetReq
 *
 * @brief   Process incoming Security Get APS Link Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processSecApsLinkKeyGetReq( int connection,
                                        SecApsLinkKeyGetReq *pMsg )
{
  int len;
  uint8 *pBuf;
  SecApsLinkKeyGetRsp apsKeyRsp = SEC_APS_LINK_KEY_GET_RSP__INIT;

  apsKeyRsp.cmdid = ZSTACK_CMD_IDS__SEC_APS_LINKKEY_GET_RSP;
  apsKeyRsp.tclinkkey = pMsg->tclinkkey;
  apsKeyRsp.ieeeaddr = pMsg->ieeeaddr;

  if ( pMsg->tclinkkey )
  {
    int x;
    APSME_TCLinkKey_t tcLinkKey;

    for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
    {
      apsKeyRsp.status = osal_nv_read( (ZCD_NV_TCLK_TABLE_START + x), 0,
          sizeof(APSME_TCLinkKey_t), &tcLinkKey );
      if ( apsKeyRsp.status == ZSTATUS_VALUES__ZSuccess )
      {
        if ( memcmp( &(pMsg->ieeeaddr), tcLinkKey.extAddr, 8 ) == 0 )
        {
          apsKeyRsp.status = ZSTATUS_VALUES__ZSuccess;
          apsKeyRsp.rxfrmcntr = tcLinkKey.rxFrmCntr;
          apsKeyRsp.txfrmcntr = tcLinkKey.txFrmCntr;
          apsKeyRsp.key.len = 16;
          apsKeyRsp.key.data = tcLinkKey.key;
          break;
        }
      }
    }

    if ( x == ZDSECMGR_TC_DEVICE_MAX )
    {
      apsKeyRsp.status = ZSTATUS_VALUES__ZSecNoKey;
    }
  }
  else
  {
    ZDSecMgrEntry_t *pEntry;
    uint8 addr[8];

    memcpy( addr, &(pMsg->ieeeaddr), 8 );

    apsKeyRsp.status = ZDSecMgrEntryLookupExt( addr, &pEntry );
    if ( apsKeyRsp.status == ZSTATUS_VALUES__ZSuccess )
    {
      APSME_LinkKeyData_t linkKey;

      apsKeyRsp.status = osal_nv_read( pEntry->keyNvId, 0,
          sizeof(APSME_LinkKeyData_t), &linkKey );
      if ( apsKeyRsp.status == ZSTATUS_VALUES__ZSuccess )
      {
        apsKeyRsp.rxfrmcntr = linkKey.rxFrmCntr;
        apsKeyRsp.txfrmcntr = linkKey.txFrmCntr;
        apsKeyRsp.key.len = 16;
        apsKeyRsp.key.data = linkKey.key;
      }
    }
  }

  len = sec_aps_link_key_get_rsp__get_packed_size( &apsKeyRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    sec_aps_link_key_get_rsp__pack( &apsKeyRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__SEC_APS_LINKKEY_GET_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processSecApsLinkKeySetReq
 *
 * @brief   Process incoming Security Set APS Link Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *l
 * @return  none
 */
static void processSecApsLinkKeySetReq( int connection,
                                        SecApsLinkKeySetReq *pMsg )
{
  ZStatusValues status;

  if ( pMsg->tclinkkey )
  {
    int y = 0;
    int x;
    APSME_TCLinkKey_t tcLinkKey;
    uint8 dummyKey[] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
    {
      status = osal_nv_read( (ZCD_NV_TCLK_TABLE_START + x), 0,
          sizeof(APSME_TCLinkKey_t), &tcLinkKey );
      if ( status == ZSTATUS_VALUES__ZSuccess )
      {
        if ( memcmp( &(pMsg->ieeeaddr), tcLinkKey.extAddr, 8 ) == 0 )
        {
          y = x;
          break;
        }
        else if ( (y == 0) && (memcmp( &(pMsg->ieeeaddr), dummyKey, 8 ) == 0) )
        {
          // First empty spot
          y = x;
        }
      }
    }

    if ( y > 0 )
    {
      memcpy( tcLinkKey.extAddr, &(pMsg->ieeeaddr), 8 );
      memcpy( tcLinkKey.key, pMsg->key.data, SEC_KEY_LEN );
      tcLinkKey.txFrmCntr = (pMsg->has_txfrmcntr) ? pMsg->txfrmcntr : 0;
      tcLinkKey.rxFrmCntr = (pMsg->has_rxfrmcntr) ? pMsg->rxfrmcntr : 0;

      status = osal_nv_write( (ZCD_NV_TCLK_TABLE_START + x), 0,
          sizeof(APSME_TCLinkKey_t), &tcLinkKey );
    }
  }
  else
  {
    ZDSecMgrEntry_t *pEntry;
    APSME_LinkKeyData_t linkKey;

    memcpy( linkKey.key, pMsg->key.data, SEC_KEY_LEN );
    linkKey.txFrmCntr = (pMsg->has_txfrmcntr) ? pMsg->txfrmcntr : 0;
    linkKey.rxFrmCntr = (pMsg->has_rxfrmcntr) ? pMsg->rxfrmcntr : 0;

    status = ZDSecMgrEntryLookupExt( (uint8 *) &(pMsg->ieeeaddr), &pEntry );
    if ( status == ZSTATUS_VALUES__ZSuccess )
    {
      status = osal_nv_write( pEntry->keyNvId, 0, sizeof(APSME_LinkKeyData_t),
          &linkKey );
    }
    else
    {
      if ( pMsg->has_shortaddr )
      {
        status = ZDSecMgrAddLinkKey( (uint16) pMsg->shortaddr,
            (uint8 *) &(pMsg->ieeeaddr), linkKey.key );
      }
      else
      {
        status = ZSTATUS_VALUES__ZInvalidParameter;
      }
    }
  }

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processSecApsLinkKeyRemoveReq
 *
 * @brief   Process incoming Security Remove APS Link Key Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *l
 * @return  none
 */
static void processSecApsLinkKeyRemoveReq( int connection,
                                           SecApsLinkKeyRemoveReq *pMsg )
{
  ZStatusValues status;

  if ( pMsg->tclinkkey )
  {
    int x;
    APSME_TCLinkKey_t tcLinkKey;

    for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
    {
      status = osal_nv_read( (ZCD_NV_TCLK_TABLE_START + x), 0,
          sizeof(APSME_TCLinkKey_t), &tcLinkKey );
      if ( status == ZSTATUS_VALUES__ZSuccess )
      {
        if ( memcmp( &(pMsg->ieeeaddr), tcLinkKey.extAddr, 8 ) == 0 )
        {
          memset( &tcLinkKey, 0, sizeof(APSME_TCLinkKey_t) );

          status = osal_nv_write( (ZCD_NV_TCLK_TABLE_START + x), 0,
              sizeof(APSME_TCLinkKey_t), &tcLinkKey );
          break;
        }
      }
    }
  }
  else
  {
    status = ZDSecMgrDeviceRemoveByExtAddr( (uint8 *) &(pMsg->ieeeaddr) );
  }

  sendDefaultRsp( connection, pMsg->cmdid, status );
}

/*********************************************************************
 * @fn      processDevNwkRouteReq
 *
 * @brief   Process incoming Device Network Route Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Nwk Route Request
 *
 * @return  none
 */
static void processDevNwkRouteReq( int connection, DevNwkRouteReq *pMsg )
{
  uint8 status;
  uint8 options = 0;

  if ( pMsg->has_mtoroute && pMsg->mtoroute )
  {
    options |= MTO_ROUTE;

    if ( pMsg->has_mtonocache && pMsg->mtonocache )
    {
      options |= NO_ROUTE_CACHE;
    }
  }

  if ( pMsg->has_multicast && pMsg->multicast )
  {
    options |= MULTICAST_ROUTE;
  }

  status = NLME_RouteDiscoveryRequest( (uint16) pMsg->dstaddr, options,
      (uint8) pMsg->radius );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processDevNwkCheckRouteReq
 *
 * @brief   Process incoming Device Network Check Route Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Nwk Check Route Request
 *
 * @return  none
 */
static void processDevNwkCheckRouteReq( int connection,
                                        DevNwkCheckRouteReq *pMsg )
{
  uint8 status;
  uint8 options = 0;

  if ( pMsg->mtoroute )
  {
    options |= MTO_ROUTE;
  }

  status = RTG_CheckRtStatus( (uint16) pMsg->dstaddr, RT_ACTIVE, options );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processSysConfigReadReq
 *
 * @brief   Process incoming Sys Config Read Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Sys Config Read Request
 *
 * @return  none
 */
static void processSysConfigReadReq( int connection, SysConfigReadReq *pMsg )
{
  int len;
  uint8 *pBuf;
  SysConfigReadRsp scRsp = SYS_CONFIG_READ_RSP__INIT;
  UserDescriptorFormat_t userDesc;
  uint8 preConfigKey[16];

  scRsp.cmdid = ZSTACK_CMD_IDS__SYS_CONFIG_READ_RSP;

  if ( pMsg->preconfigkeyenable )
  {
    scRsp.has_preconfigkeyenable = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_PRECFGKEYS_ENABLE, 0, sizeof(zgPreConfigKeys), &zgPreConfigKeys );
#endif
    scRsp.preconfigkeyenable = zgPreConfigKeys;
  }

  if ( pMsg->securitymodeenable )
  {
    scRsp.has_securitymodeenable = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_SECURITY_MODE, 0, sizeof(zgSecurityMode), &zgSecurityMode );
#endif
    scRsp.securitymodeenable = zgSecurityMode;
  }

  if ( pMsg->usedefaulttclk )
  {
    scRsp.has_usedefaulttclk = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_USE_DEFAULT_TCLK, 0, sizeof(zgUseDefaultTCLK), &zgUseDefaultTCLK );
#endif
    scRsp.usedefaulttclk = zgUseDefaultTCLK;
  }

  if ( pMsg->pollrate )
  {
    scRsp.has_pollrate = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_POLL_RATE, 0, sizeof(zgPollRate), &zgPollRate );
#endif
    scRsp.pollrate = zgPollRate;
  }

  if ( pMsg->queuedpollrate )
  {
    scRsp.has_queuedpollrate = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_QUEUED_POLL_RATE, 0, sizeof(zgQueuedPollRate), &zgQueuedPollRate );
#endif
    scRsp.queuedpollrate = zgQueuedPollRate;
  }

  if ( pMsg->responsepollrate )
  {
    scRsp.has_responsepollrate = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_RESPONSE_POLL_RATE, 0, sizeof(zgResponsePollRate), &zgResponsePollRate );
#endif
    scRsp.responsepollrate = zgResponsePollRate;
  }

  if ( pMsg->apsackwaitduration )
  {
    scRsp.has_apsackwaitduration = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_APS_ACK_WAIT_DURATION, 0, sizeof(zgApscAckWaitDurationPolled), &zgApscAckWaitDurationPolled );
#endif
    scRsp.apsackwaitduration = zgApscAckWaitDurationPolled;
  }

  if ( pMsg->bindingtime )
  {
    scRsp.has_bindingtime = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_BINDING_TIME, 0, sizeof(zgApsDefaultMaxBindingTime), &zgApsDefaultMaxBindingTime );
#endif
    scRsp.bindingtime = zgApsDefaultMaxBindingTime;
  }

  if ( pMsg->panid )
  {
    scRsp.has_panid = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
#endif
    scRsp.panid = zgConfigPANID;
  }

  if ( pMsg->pollfailureretries )
  {
    scRsp.has_pollfailureretries = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_POLL_FAILURE_RETRIES, 0, sizeof(zgMaxPollFailureRetries), &zgMaxPollFailureRetries );
#endif
    scRsp.pollfailureretries = zgMaxPollFailureRetries;
  }

  if ( pMsg->indirectmsgtimeout )
  {
    scRsp.has_indirectmsgtimeout = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_INDIRECT_MSG_TIMEOUT, 0, sizeof(zgIndirectMsgTimeout), &zgIndirectMsgTimeout );
#endif
    scRsp.indirectmsgtimeout = zgIndirectMsgTimeout;
  }

  if ( pMsg->apsframeretries )
  {
    scRsp.has_apsframeretries = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_APS_FRAME_RETRIES, 0, sizeof(zgApscMaxFrameRetries), &zgApscMaxFrameRetries );
#endif
    scRsp.apsframeretries = zgApscMaxFrameRetries;
  }

  if ( pMsg->bcastretries )
  {
    scRsp.has_bcastretries = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_BCAST_RETRIES, 0, sizeof(zgMaxBcastRetires), &zgMaxBcastRetires );
#endif
    scRsp.bcastretries = zgMaxBcastRetires;
  }

  if ( pMsg->passiveacktimeout )
  {
    scRsp.has_passiveacktimeout = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_PASSIVE_ACK_TIMEOUT, 0, sizeof(zgPassiveAckTimeout), &zgPassiveAckTimeout );
#endif
    scRsp.passiveacktimeout = zgPassiveAckTimeout;
  }

  if ( pMsg->bcastdeliverytime )
  {
    scRsp.has_bcastdeliverytime = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_BCAST_DELIVERY_TIME, 0, sizeof(zgBcastDeliveryTime), &zgBcastDeliveryTime );
#endif
    scRsp.bcastdeliverytime = zgBcastDeliveryTime;
  }

  if ( pMsg->routeexpirytime )
  {
    scRsp.has_routeexpirytime = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_ROUTE_EXPIRY_TIME, 0, sizeof(zgRouteExpiryTime), &zgRouteExpiryTime );
#endif
    scRsp.routeexpirytime = zgRouteExpiryTime;
  }

  if ( pMsg->userdesc )
  {
    scRsp.has_userdesc = TRUE;

    osal_nv_read( ZCD_NV_USERDESC, 0, sizeof(UserDescriptorFormat_t),
        &userDesc );

    scRsp.userdesc.len = userDesc.len;
    scRsp.userdesc.data = userDesc.desc;
  }

  if ( pMsg->preconfigkey )
  {
    scRsp.has_preconfigkey = TRUE;

    osal_nv_read( ZCD_NV_PRECFGKEY, 0, 16, preConfigKey );

    scRsp.preconfigkey.len = 16;
    scRsp.preconfigkey.data = preConfigKey;
  }

  if ( pMsg->chanlist )
  {
    scRsp.has_chanlist = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_CHANLIST, 0, sizeof(zgDefaultChannelList), &zgDefaultChannelList );
#endif
    scRsp.chanlist = zgDefaultChannelList;
  }

  if ( pMsg->multicastradius )
  {
    scRsp.has_multicastradius = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_APS_NONMEMBER_RADIUS, 0, sizeof(zgApsNonMemberRadius), &zgApsNonMemberRadius );
#endif
    scRsp.multicastradius = zgApsNonMemberRadius;
  }

  if ( pMsg->extendedpanid )
  {
    scRsp.has_extendedpanid = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_APS_USE_EXT_PANID, 0, sizeof(zgApsUseExtendedPANID), &zgApsUseExtendedPANID );
#endif
    memcpy( &scRsp.extendedpanid, zgApsUseExtendedPANID, 8 );
  }

  if ( pMsg->ieeeaddr )
  {
    scRsp.has_ieeeaddr = TRUE;
    memcpy( &scRsp.ieeeaddr, NLME_GetExtAddr(), 8 );
  }

  if ( pMsg->macrxonidle )
  {
    uint8 x;
#if defined ( LINUX_ZNP )
    x = zdoExtRxIdle( FALSE, 0 );
#else
    ZMacGetReq( ZMacRxOnIdle, &x );
#endif

    scRsp.has_macrxonidle = TRUE;
    if ( x )
    {
      scRsp.macrxonidle = TRUE;
    }
    else
    {
      scRsp.macrxonidle = FALSE;
    }
  }

  if ( pMsg->snifferfeature )
  {
    scRsp.has_snifferfeature = TRUE;
#if defined ( LINUX_ZNP )
    scRsp.snifferfeature = znpSniffer( 2 );
#endif
  }

  if ( pMsg->concentratorenable )
  {
    scRsp.has_concentratorenable = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_CONCENTRATOR_ENABLE, 0, sizeof(zgConcentratorEnable), &zgConcentratorEnable );
#endif
    scRsp.concentratorenable = zgConcentratorEnable;
  }

  if ( pMsg->concentratordisctime )
  {
    scRsp.has_concentratordisctime = TRUE;
#if defined ( LINUX_ZNP )
    osal_nv_read( ZCD_NV_CONCENTRATOR_DISCOVERY, 0, sizeof(zgConcentratorDiscoveryTime), &zgConcentratorDiscoveryTime );
#endif
    scRsp.concentratorenable = zgConcentratorDiscoveryTime;
  }

  len = sys_config_read_rsp__get_packed_size( &scRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    sys_config_read_rsp__pack( &scRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__SYS_CONFIG_READ_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processSysConfigWriteReq
 *
 * @brief   Process incoming Sys Config Write Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Sys Config Write Request
 *
 * @return  none
 */
static void processSysConfigWriteReq( int connection, SysConfigWriteReq *pMsg )
{
  uint8 status = ZSUCCESS;

  if ( pMsg->has_preconfigkeyenable )
  {
    if ( pMsg->preconfigkeyenable )
    {
      zgPreConfigKeys = TRUE;
    }
    else
    {
      zgPreConfigKeys = FALSE;
    }
    osal_nv_write( ZCD_NV_PRECFGKEYS_ENABLE, 0, sizeof(zgPreConfigKeys),
        &zgPreConfigKeys );
  }

  if ( pMsg->has_securitymodeenable )
  {
    if ( pMsg->securitymodeenable )
    {
      zgSecurityMode = TRUE;
    }
    else
    {
      zgSecurityMode = FALSE;
    }
    osal_nv_write( ZCD_NV_SECURITY_MODE, 0, sizeof(zgSecurityMode),
        &zgSecurityMode );
  }

  if ( pMsg->has_usedefaulttclk )
  {
    if ( pMsg->usedefaulttclk )
    {
      zgUseDefaultTCLK = TRUE;
    }
    else
    {
      zgUseDefaultTCLK = FALSE;
    }
    osal_nv_write( ZCD_NV_USE_DEFAULT_TCLK, 0, sizeof(zgUseDefaultTCLK),
        &zgUseDefaultTCLK );
  }

  if ( pMsg->has_pollrate )
  {
    zgPollRate = (uint16) pMsg->pollrate;
    osal_nv_write( ZCD_NV_POLL_RATE, 0, sizeof(zgPollRate), &zgPollRate );
  }

  if ( pMsg->has_queuedpollrate )
  {
    zgQueuedPollRate = (uint16) pMsg->queuedpollrate;
    osal_nv_write( ZCD_NV_QUEUED_POLL_RATE, 0, sizeof(zgQueuedPollRate),
        &zgQueuedPollRate );
  }

  if ( pMsg->has_responsepollrate )
  {
    zgResponsePollRate = (uint16) pMsg->responsepollrate;
    osal_nv_write( ZCD_NV_RESPONSE_POLL_RATE, 0, sizeof(zgResponsePollRate),
        &zgResponsePollRate );
  }

  if ( pMsg->has_apsackwaitduration )
  {
    zgApscAckWaitDurationPolled = (uint16) pMsg->apsackwaitduration;
    osal_nv_write( ZCD_NV_APS_ACK_WAIT_DURATION, 0,
        sizeof(zgApscAckWaitDurationPolled), &zgApscAckWaitDurationPolled );
  }

  if ( pMsg->has_bindingtime )
  {
    zgApsDefaultMaxBindingTime = (uint16) pMsg->bindingtime;
    osal_nv_write( ZCD_NV_BINDING_TIME, 0, sizeof(zgApsDefaultMaxBindingTime),
        &zgApsDefaultMaxBindingTime );
  }

  if ( pMsg->has_panid )
  {
    zgConfigPANID = (uint16) pMsg->panid;
    osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
  }

  if ( pMsg->has_pollfailureretries )
  {
    zgMaxPollFailureRetries = (uint8) pMsg->pollfailureretries;
    osal_nv_write( ZCD_NV_POLL_FAILURE_RETRIES, 0,
        sizeof(zgMaxPollFailureRetries), &zgMaxPollFailureRetries );
  }

  if ( pMsg->has_indirectmsgtimeout )
  {
    zgIndirectMsgTimeout = (uint8) pMsg->indirectmsgtimeout;
    osal_nv_write( ZCD_NV_INDIRECT_MSG_TIMEOUT, 0, sizeof(zgIndirectMsgTimeout),
        &zgIndirectMsgTimeout );
  }

  if ( pMsg->has_apsframeretries )
  {
    zgApscMaxFrameRetries = (uint8) pMsg->apsframeretries;
    osal_nv_write( ZCD_NV_APS_FRAME_RETRIES, 0, sizeof(zgApscMaxFrameRetries),
        &zgApscMaxFrameRetries );
  }

  if ( pMsg->has_bcastretries )
  {
    zgMaxBcastRetires = (uint8) pMsg->bcastretries;
    osal_nv_write( ZCD_NV_BCAST_RETRIES, 0, sizeof(zgMaxBcastRetires),
        &zgMaxBcastRetires );
  }

  if ( pMsg->has_passiveacktimeout )
  {
    zgPassiveAckTimeout = (uint8) pMsg->passiveacktimeout;
    osal_nv_write( ZCD_NV_PASSIVE_ACK_TIMEOUT, 0, sizeof(zgPassiveAckTimeout),
        &zgPassiveAckTimeout );
  }

  if ( pMsg->has_bcastdeliverytime )
  {
    zgBcastDeliveryTime = (uint8) pMsg->bcastdeliverytime;
    osal_nv_write( ZCD_NV_BCAST_DELIVERY_TIME, 0, sizeof(zgBcastDeliveryTime),
        &zgBcastDeliveryTime );
  }

  if ( pMsg->has_routeexpirytime )
  {
    zgRouteExpiryTime = (uint8) pMsg->routeexpirytime;
    osal_nv_write( ZCD_NV_ROUTE_EXPIRY_TIME, 0, sizeof(zgRouteExpiryTime),
        &zgRouteExpiryTime );
  }

  if ( pMsg->has_userdesc )
  {
    UserDescriptorFormat_t userDesc;

    userDesc.len = pMsg->userdesc.len;
    if ( userDesc.len > AF_MAX_USER_DESCRIPTOR_LEN )
    {
      userDesc.len = AF_MAX_USER_DESCRIPTOR_LEN;
    }

    memcpy( userDesc.desc, pMsg->userdesc.data, userDesc.len );
    osal_nv_write( ZCD_NV_USERDESC, 0, sizeof(UserDescriptorFormat_t),
        &userDesc );
  }

  if ( pMsg->has_preconfigkey )
  {
    int len;

    len = pMsg->preconfigkey.len;
    if ( len > 16 )
    {
      len = 16;
    }

    osal_nv_write( ZCD_NV_PRECFGKEY, 0, len, pMsg->preconfigkey.data );
  }

  if ( pMsg->has_chanlist )
  {
    zgDefaultChannelList = (uint32) pMsg->chanlist;
    osal_nv_write( ZCD_NV_CHANLIST, 0, sizeof(zgDefaultChannelList),
        &zgDefaultChannelList );
  }

  if ( pMsg->has_multicastradius )
  {
    zgApsNonMemberRadius = (uint8) pMsg->multicastradius;
    osal_nv_write( ZCD_NV_APS_NONMEMBER_RADIUS, 0, sizeof(zgApsNonMemberRadius),
        &zgApsNonMemberRadius );
  }

  if ( pMsg->has_extendedpanid )
  {
    memcpy( zgApsUseExtendedPANID, &(pMsg->extendedpanid), 8 );
    osal_nv_write( ZCD_NV_APS_USE_EXT_PANID, 0, 8, zgApsUseExtendedPANID );
  }

  if ( pMsg->has_ieeeaddr )
  {
    osal_nv_write( ZCD_NV_EXTADDR, 0, 8, &(pMsg->ieeeaddr) );
  }

  if ( pMsg->has_macrxonidle )
  {
    uint8 x = pMsg->macrxonidle;
#if defined ( LINUX_ZNP )
    zdoExtRxIdle( TRUE, x );
#else
    ZMacSetReq( ZMacRxOnIdle, &x );
#endif
  }

  if ( pMsg->has_snifferfeature )
  {
#if defined ( LINUX_ZNP )
    znpSniffer( pMsg->snifferfeature );
#else
#endif
  }

  if ( pMsg->has_concentratordisctime )
  {
    zgConcentratorDiscoveryTime = (uint8) pMsg->concentratordisctime;
    osal_nv_write( ZCD_NV_CONCENTRATOR_DISCOVERY, 0,
        sizeof(zgConcentratorDiscoveryTime), &zgConcentratorDiscoveryTime );
  }

  if ( pMsg->has_concentratorenable )
  {
    zgConcentratorEnable = (uint8) pMsg->concentratorenable;
    osal_nv_write( ZCD_NV_CONCENTRATOR_ENABLE, 0, sizeof(zgConcentratorEnable),
        &zgConcentratorEnable );

    ZDApp_ForceConcentratorChange();
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processAfRegisterReq
 *
 * @brief   Process incoming AF Register Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to AF Register Request
 *
 * @return  none
 */
static void processAfRegisterReq( int connection, AfRegisterReq *pMsg )
{
  uint8 status = ZSTATUS_VALUES__ZSuccess;
  epItem_t *pItem;

  if ( epTableFindEntryEP( (uint8) pMsg->endpoint ) )
  {
    uiPrintfEx(trINFO, "zstackpb afRegister fail: endpoint:%d\n", pMsg->endpoint );
    status = ZSTATUS_VALUES__ZAfDuplicateEndpoint;
  }
  else if ( epTableNumEntries() >= maxSupportedEndpoints )
  {
    status = ZSTATUS_VALUES__ZAfEndpointMax;
  }
  else
  {
    uint8 allocated = FALSE;

    // See if there is an EP item that hasn't defined an endpointl
    pItem = epTableFindEntryConnection( connection );
    if ( (pItem == NULL) || (pItem->epDesc.endPoint != 0) )
    {
      // Allocate a new epItem_t if one doesn't existed or an endpoint has already been
      // defined.
      pItem = (epItem_t *) malloc( sizeof(epItem_t) );
      if ( pItem )
      {
        allocated = TRUE;
        memset( pItem, 0, sizeof(epItem_t) );
        pItem->connection = connection;
      }
    }

    if ( pItem )
    {
      pItem->epDesc.endPoint = pMsg->endpoint;
      pItem->epDesc.latencyReq = pMsg->latencyreq;
      pItem->epDesc.task_id = &zspbTaskID;

      pItem->epDesc.simpleDesc = (SimpleDescriptionFormat_t *) malloc(
          sizeof(SimpleDescriptionFormat_t) );
      if ( pItem->epDesc.simpleDesc )
      {
        int i;
        memset( pItem->epDesc.simpleDesc, 0,
            sizeof(SimpleDescriptionFormat_t) );
        pItem->epDesc.simpleDesc->AppDevVer = pMsg->simpledesc->devicever;
        pItem->epDesc.simpleDesc->AppDeviceId = pMsg->simpledesc->deviceid;
        pItem->epDesc.simpleDesc->EndPoint = pMsg->simpledesc->endpoint;
        pItem->epDesc.simpleDesc->AppNumInClusters =
            pMsg->simpledesc->n_inputclusters;
        pItem->epDesc.simpleDesc->AppNumOutClusters =
            pMsg->simpledesc->n_outputclusters;
        pItem->epDesc.simpleDesc->AppProfId = pMsg->simpledesc->profileid;
        pItem->epDesc.simpleDesc->pAppInClusterList = (cId_t *) malloc(
            sizeof(cId_t) * pItem->epDesc.simpleDesc->AppNumInClusters );
        if ( pItem->epDesc.simpleDesc->pAppInClusterList )
        {
          for ( i = 0; i < pItem->epDesc.simpleDesc->AppNumInClusters; i++ )
          {
            pItem->epDesc.simpleDesc->pAppInClusterList[i] =
                pMsg->simpledesc->inputclusters[i];
          }
        }
        pItem->epDesc.simpleDesc->pAppOutClusterList = (cId_t *) malloc(
            sizeof(cId_t) * pItem->epDesc.simpleDesc->AppNumOutClusters );
        if ( pItem->epDesc.simpleDesc->pAppOutClusterList )
        {
          for ( i = 0; i < pItem->epDesc.simpleDesc->AppNumOutClusters; i++ )
          {
            pItem->epDesc.simpleDesc->pAppOutClusterList[i] =
                pMsg->simpledesc->outputclusters[i];
          }
        }
      }

      uiPrintfEx(trINFO, "zstackpb afRegister: profileID:%x\n", pItem->epDesc.simpleDesc->AppProfId );

      status = afRegister( &(pItem->epDesc) );

      if ( (status == afStatus_INVALID_PARAMETER)
          || (status == ZApsDuplicateEntry) )
      {
        afDelete( pItem->epDesc.endPoint );
        status = afRegister( &(pItem->epDesc) );
      }

      if ( status == ZSuccess )
      {
        if ( allocated == TRUE )
        {
          if ( epTableAddNewEntry( pItem ) == FALSE )
          {
            freeEpItem( pItem );
          }
        }
      }
      else
      {
        if ( allocated == TRUE )
        {
          freeEpItem( pItem );
        }
        else
        {
          // AF registration failed clear the end point descriptor
          if ( pItem->epDesc.simpleDesc )
          {
            if ( pItem->epDesc.simpleDesc->pAppInClusterList )
            {
              free( pItem->epDesc.simpleDesc->pAppInClusterList );
            }
            if ( pItem->epDesc.simpleDesc->pAppInClusterList )
            {
              free( pItem->epDesc.simpleDesc->pAppInClusterList );
            }
            free( pItem->epDesc.simpleDesc );
          }

          memset( &(pItem->epDesc), 0, sizeof(endPointDesc_t) );
        }
      }

      // If status is still bad
      if ( (status == afStatus_INVALID_PARAMETER)
          || (status == ZApsDuplicateEntry) )
      {
        status = ZSTATUS_VALUES__ZAfDuplicateEndpoint;
      }
    }
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processAfUnRegisterReq
 *
 * @brief   Process incoming AF UnRegister Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to AF Un-Register Request
 *
 * @return  none
 */
static void processAfUnRegisterReq( int connection, AfUnRegisterReq *pMsg )
{
  uint8 status;
  epItem_t *pItem;

  pItem = epTableFindEntryEP( (uint8) pMsg->endpoint );
  if ( pItem == NULL )
  {
    status = ZSTATUS_VALUES__ZInvalidParameter;
  }
  else
  {
    status = afDelete( pItem->epDesc.endPoint );

    if ( status == ZSuccess )
    {
      epTableRemoveEntry( pItem );
    }
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processAfConfigGetReq
 *
 * @brief   Process incoming AF Config Get Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to AF Config Get Request
 *
 * @return  none
 */
static void processAfConfigGetReq( int connection, AfConfigGetReq *pMsg )
{
  int len;
  uint8 *pBuf;
  AfConfigGetRsp cgRsp = AF_CONFIG_GET_RSP__INIT;
  afAPSF_Config_t config;

  afAPSF_ConfigGet( pMsg->endpoint, &config );

  cgRsp.cmdid = ZSTACK_CMD_IDS__AF_CONFIG_GET_RSP;
  cgRsp.endpoint = pMsg->endpoint;
  cgRsp.framedelay = config.frameDelay;
  cgRsp.windowsize = config.windowSize;

  len = af_config_get_rsp__get_packed_size( &cgRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    af_config_get_rsp__pack( &cgRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__AF_CONFIG_GET_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn      processAfConfigSetReq
 *
 * @brief   Process incoming AF Config Set Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to AF Config Set Request
 *
 * @return  none
 */
static void processAfConfigSetReq( int connection, AfConfigSetReq *pMsg )
{
  uint8 status;
  afAPSF_Config_t config;

  config.frameDelay = (uint8) pMsg->framedelay;
  config.windowSize = (uint8) pMsg->windowsize;

  status = afAPSF_ConfigSet( (uint8) pMsg->endpoint, &config );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processAfDataReq
 *
 * @brief   Process incoming AF Data Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to AF Data Request
 *
 * @return  none
 */
static void processAfDataReq( int connection, AfDataReq *pMsg )
{
  uint8 status;

  if ( epTableFindEntryEP( (uint8) pMsg->srcendpoint ) == NULL )
  {
    status = ZSTATUS_VALUES__ZInvalidParameter;
  }
  else
  {
    endPointDesc_t *pEPDesc = NULL;
    afAddrType_t dstAddr;
    uint8 transId = pMsg->transid;
    uint8 txOptions = convertPBTransOptions( pMsg->options );

    uiPrintfEx(trINFO, "zstackpb Rcvd AF Data Request: addrMode: %d, dstAddr:%x, dep:%d, sep:%d\n",
        pMsg->dstaddr->addrmode, pMsg->dstaddr->shortaddr, pMsg->dstaddr->endpoint, pMsg->srcendpoint );

    pEPDesc = afFindEndPointDesc( (uint8) pMsg->srcendpoint );
    if ( pEPDesc )
    {
      dstAddr.endPoint = pMsg->dstaddr->endpoint;
      dstAddr.panId = pMsg->dstaddr->panid;
      dstAddr.addrMode = pMsg->dstaddr->addrmode;
      if ( (dstAddr.addrMode == afAddr16Bit)
          || (dstAddr.addrMode == afAddrGroup)
          || (dstAddr.addrMode == afAddrBroadcast) )
      {
        dstAddr.addr.shortAddr = pMsg->dstaddr->shortaddr;
      }
      else if ( dstAddr.addrMode == afAddr64Bit )
      {
        memcpy( dstAddr.addr.extAddr, &(pMsg->dstaddr->extaddr), 8 );
      }

      if ( pMsg->n_relaylist )
      {
        uint16 *pList = malloc( sizeof(uint16) * pMsg->n_relaylist );
        if ( pList )
        {
          int i;
          for ( i = 0; i < pMsg->n_relaylist; i++ )
          {
            pList[i] = (uint16) pMsg->relaylist[i];
          }

          status = AF_DataRequestSrcRtg( &dstAddr, pEPDesc,
              (uint16) pMsg->clusterid, (uint16) pMsg->payload.len,
              (uint8 *) pMsg->payload.data, &transId, txOptions,
              (uint8) pMsg->radius, (uint8) pMsg->n_relaylist, pList );

          free( pList );
        }
        else
        {
          status = ZSTATUS_VALUES__ZMemError;
        }
      }
      else
      {
        status = AF_DataRequest( &dstAddr, pEPDesc, (uint16) pMsg->clusterid,
            (uint16) pMsg->payload.len, (uint8 *) pMsg->payload.data, &transId,
            txOptions, (uint8) pMsg->radius );
      }
    }
    else
    {
      uiPrintfEx(trINFO, "zstackpb Rcvd AF Data Request: ep not found2\n" );
      status = ZSTATUS_VALUES__ZInvalidParameter;
    }
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMatchDescReq
 *
 * @brief   Process incoming ZDO Match Desc Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMatchDescReq( int connection, ZdoMatchDescReq *pMsg )
{
  int i;
  uint8 status;
  zAddrType_t dstAddr;
  uint16 *inClusters = NULL;
  uint16 *outClusters = NULL;

  uiPrintfEx(trINFO, "zstackpb Rcvd Match Descriptor Request\n" );

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  if ( pMsg->n_inputclusters )
  {
    inClusters = (uint16 *) malloc( sizeof(uint16) * pMsg->n_inputclusters );
    if ( inClusters )
    {
      for ( i = 0; i < pMsg->n_inputclusters; i++ )
      {
        inClusters[i] = pMsg->inputclusters[i];
      }
    }
  }

  if ( pMsg->n_outputclusters )
  {
    outClusters = (uint16 *) malloc( sizeof(uint16) * pMsg->n_outputclusters );
    if ( outClusters )
    {
      for ( i = 0; i < pMsg->n_outputclusters; i++ )
      {
        outClusters[i] = pMsg->outputclusters[i];
      }
    }
  }

  status = ZDP_MatchDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest,
      (uint16) pMsg->profileid, (uint8) pMsg->n_inputclusters, inClusters,
      (uint8) pMsg->n_outputclusters, outClusters, FALSE );

  if ( inClusters )
  {
    free( inClusters );
  }

  if ( outClusters )
  {
    free( outClusters );
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoNwkAddrReq
 *
 * @brief   Process incoming ZDO Nwk Address Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoNwkAddrReq( int connection, ZdoNwkAddrReq *pMsg )
{
  uint8 status;
  uint8 extAddr[8];

  memcpy( extAddr, &pMsg->ieeeaddr, 8 );

  status = ZDP_NwkAddrReq( extAddr, (uint8) pMsg->type,
      (uint8) pMsg->startindex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoIeeeAddrReq
 *
 * @brief   Process incoming ZDO IEEE Address Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoIeeeAddrReq( int connection, ZdoIeeeAddrReq *pMsg )
{
  uint8 status;

  status = ZDP_IEEEAddrReq( (uint16) pMsg->nwkaddr, (uint8) pMsg->type,
      (uint8) pMsg->startindex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoNodeDescReq
 *
 * @brief   Process incoming ZDO Node Descriptor Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoNodeDescReq( int connection, ZdoNodeDescReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_NodeDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoPowerDescReq
 *
 * @brief   Process incoming ZDO Power Descriptor Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoPowerDescReq( int connection, ZdoPowerDescReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_PowerDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest,
      FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoSimpleDescReq
 *
 * @brief   Process incoming ZDO Simple Descriptor Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoSimpleDescReq( int connection, ZdoSimpleDescReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_SimpleDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest,
      (uint8) pMsg->endpoint, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoActiveEndpointsReq
 *
 * @brief   Process incoming ZDO Active Endpoints Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoActiveEndpointsReq( int connection,
                                          ZdoActiveEndpointReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_ActiveEPReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoComplexDescReq
 *
 * @brief   Process incoming ZDO Complex Descriptor Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoComplexDescReq( int connection, ZdoComplexDescReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_ComplexDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest,
      FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoUserDescSetReq
 *
 * @brief   Process incoming ZDO User Descriptor Set Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoUserDescSetReq( int connection, ZdoUserDescSetReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  UserDescriptorFormat_t userDesc;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  userDesc.len = pMsg->userdescriptor.len;
  if ( userDesc.len > AF_MAX_USER_DESCRIPTOR_LEN )
  {
    userDesc.len = AF_MAX_USER_DESCRIPTOR_LEN;
  }
  memcpy( userDesc.desc, pMsg->userdescriptor.data, userDesc.len );

  status = ZDP_UserDescSet( &dstAddr, (uint16) pMsg->nwkaddrofinterest,
      &userDesc, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoServerDiscReq
 *
 * @brief   Process incoming ZDO Server Discovery Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoServerDiscReq( int connection, ZdoServerDiscReq *pMsg )
{
  uint8 status;
  uint16 serverMask;

  serverMask = convertPBServerCapabilities( pMsg->servermask );

  status = ZDP_ServerDiscReq( serverMask, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoEndDeviceBindReq
 *
 * @brief   Process incoming ZDO End Device Bind Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoEndDeviceBindReq( int connection,
                                        ZdoEndDeviceBindReq *pMsg )
{
  int i;
  uint8 status;
  zAddrType_t dstAddr;
  cId_t *inList = NULL;
  cId_t *outList = NULL;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  if ( pMsg->n_inputclusters )
  {
    inList = (cId_t *) malloc( sizeof(cId_t) * pMsg->n_inputclusters );
    if ( inList )
    {
      for ( i = 0; i < pMsg->n_inputclusters; i++ )
      {
        inList[i] = pMsg->inputclusters[i];
      }
    }
  }

  if ( pMsg->n_outputclusters )
  {
    outList = (cId_t *) malloc( sizeof(cId_t) * pMsg->n_outputclusters );
    if ( outList )
    {
      for ( i = 0; i < pMsg->n_outputclusters; i++ )
      {
        outList[i] = pMsg->outputclusters[i];
      }
    }
  }

  status = ZDP_EndDeviceBindReq( &dstAddr, (uint16) pMsg->localcoordinator,
      (uint8) pMsg->endpoint, (uint16) pMsg->profileid,
      (uint8) pMsg->n_inputclusters, inList, (uint8) pMsg->n_outputclusters,
      outList, FALSE );

  if ( pMsg->n_inputclusters )
  {
    free( inList );
  }

  if ( pMsg->n_outputclusters )
  {
    free( outList );
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoBindReq
 *
 * @brief   Process incoming ZDO Bind Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoBindReq( int connection, ZdoBindReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 srcAddr[8];
  zAddrType_t destinationAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;

  memcpy( srcAddr, &pMsg->bindinfo->srcaddr, 8 );

  destinationAddr.addrMode = pMsg->bindinfo->dstaddr->addrmode;
  if ( (destinationAddr.addrMode == afAddr16Bit)
      || (destinationAddr.addrMode == afAddrGroup)
      || (destinationAddr.addrMode == afAddrBroadcast) )
  {
    destinationAddr.addr.shortAddr = pMsg->bindinfo->dstaddr->shortaddr;
  }
  else if ( destinationAddr.addrMode == afAddr64Bit )
  {
    memcpy( destinationAddr.addr.extAddr, &(pMsg->bindinfo->dstaddr->extaddr),
        8 );
  }

  status = ZDP_BindUnbindReq( Bind_req, &dstAddr, srcAddr,
      (uint8) pMsg->bindinfo->srcendpoint, (cId_t) pMsg->bindinfo->clusterid,
      &destinationAddr, (uint8) pMsg->bindinfo->dstaddr->endpoint, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoUnbindReq
 *
 * @brief   Process incoming ZDO Unbind Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoUnbindReq( int connection, ZdoUnbindReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 srcAddr[8];
  zAddrType_t destinationAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;

  memcpy( srcAddr, &pMsg->bindinfo->srcaddr, 8 );

  destinationAddr.addrMode = pMsg->bindinfo->dstaddr->addrmode;
  if ( (destinationAddr.addrMode == afAddr16Bit)
      || (destinationAddr.addrMode == afAddrGroup)
      || (destinationAddr.addrMode == afAddrBroadcast) )
  {
    destinationAddr.addr.shortAddr = pMsg->bindinfo->dstaddr->shortaddr;
  }
  else if ( destinationAddr.addrMode == afAddr64Bit )
  {
    memcpy( destinationAddr.addr.extAddr, &(pMsg->bindinfo->dstaddr->extaddr),
        8 );
  }

  status = ZDP_BindUnbindReq( Unbind_req, &dstAddr, srcAddr,
      (uint8) pMsg->bindinfo->srcendpoint, (cId_t) pMsg->bindinfo->clusterid,
      &destinationAddr, (uint8) pMsg->bindinfo->dstaddr->endpoint, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoUserDescReq
 *
 * @brief   Process incoming ZDO User Descriptor Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoUserDescReq( int connection, ZdoUserDescReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = ZDP_UserDescReq( &dstAddr, (uint16) pMsg->nwkaddrofinterest, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtPermitJoinReq
 *
 * @brief   Process incoming ZDO Management Permit Join Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtPermitJoinReq( int connection,
                                         ZdoMgmtPermitJoinReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;

  // Check for broadcast message
  if ( (pMsg->nwkaddr == 0xFFFF) || (pMsg->nwkaddr == 0xFFFD)
      || (pMsg->nwkaddr == 0xFFFC) )
  {
    // Send to self first
    dstAddr.addr.shortAddr = NLME_GetShortAddr();
    status = (uint8) ZDP_MgmtPermitJoinReq( &dstAddr, (uint8) pMsg->duration,
        (uint8) pMsg->tcsignificance, FALSE );
  }

  dstAddr.addr.shortAddr = pMsg->nwkaddr;

  status = (uint8) ZDP_MgmtPermitJoinReq( &dstAddr, (uint8) pMsg->duration,
      (uint8) pMsg->tcsignificance, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtLeaveReq
 *
 * @brief   Process incoming ZDO Management Leave Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtLeaveReq( int connection, ZdoMgmtLeaveReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 ieeeAddr[8];

  dstAddr.addrMode = Addr16Bit;
  memcpy( ieeeAddr, &(pMsg->deviceaddress), 8 );

  // Check for broadcast message
  if ( (pMsg->nwkaddr == 0xFFFF) || (pMsg->nwkaddr == 0xFFFD)
      || (pMsg->nwkaddr == 0xFFFC) )
  {
    // Send to self first
    dstAddr.addr.shortAddr = NLME_GetShortAddr();
    status = (uint8) ZDP_MgmtLeaveReq( &dstAddr, ieeeAddr,
        pMsg->options->removechildren, pMsg->options->rejoin, FALSE );
  }

  dstAddr.addr.shortAddr = pMsg->nwkaddr;

  status = (uint8) ZDP_MgmtLeaveReq( &dstAddr, ieeeAddr,
      pMsg->options->removechildren, pMsg->options->rejoin, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtNwkDiscReq
 *
 * @brief   Process incoming ZDO Management Network Discovery Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtNwkDiscReq( int connection, ZdoMgmtNwkDiscReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;

  status = (uint8) ZDP_MgmtNwkDiscReq( &dstAddr, pMsg->scanchannels,
      (uint8) pMsg->scanduration, (uint8) pMsg->startindex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtLqiReq
 *
 * @brief   Process incoming ZDO Management LQI Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtLqiReq( int connection, ZdoMgmtLqiReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 startIndex;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;
  startIndex = (uint8) pMsg->startindex;

  status = (uint8) ZDP_MgmtLqiReq( &dstAddr, startIndex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtRtgReq
 *
 * @brief   Process incoming ZDO Management Routing Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtRtgReq( int connection, ZdoMgmtRtgReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 startIndex;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;
  startIndex = (uint8) pMsg->startindex;

  status = (uint8) ZDP_MgmtRtgReq( &dstAddr, startIndex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtBindReq
 *
 * @brief   Process incoming ZDO Management Binding Table Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtBindReq( int connection, ZdoMgmtBindReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 startIndex;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;
  startIndex = (uint8) pMsg->startindex;

  status = (uint8) ZDP_MgmtBindReq( &dstAddr, startIndex, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtNwkUpdateReq
 *
 * @brief   Process incoming ZDO Management Network Update Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtNwkUpdateReq( int connection,
                                        ZdoMgmtNwkUpdateReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->dstaddr;

  status = (uint8) ZDP_MgmtNwkUpdateReq( &dstAddr, pMsg->channelmask,
      (uint8) pMsg->scanduration, (uint8) pMsg->scancount,
      (uint8) pMsg->nwkupdateid, (uint16) pMsg->nwkmgraddr );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoMgmtDirectJoinReq
 *
 * @brief   Process incoming ZDO Management Direct Join Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoMgmtDirectJoinReq( int connection,
                                         ZdoMgmtDirectJoinReq *pMsg )
{
  uint8 status;
  zAddrType_t dstAddr;
  uint8 cInfo;
  uint8 ieeeAddr[8];

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = pMsg->nwkaddr;
  memcpy( ieeeAddr, &(pMsg->deviceaddress), 8 );
  cInfo = convertCapabilityInfo( pMsg->capinfo );

  status = (uint8) ZDP_MgmtDirectJoinReq( &dstAddr, ieeeAddr, cInfo, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processZdoDeviceAnnounceReq
 *
 * @brief   Process incoming ZDO Device Announce Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Request
 *
 * @return  none
 */
static void processZdoDeviceAnnounceReq( int connection,
                                         ZdoDeviceAnnounceReq *pMsg )
{
  uint8 status;
  uint8 cInfo;
  uint8 ieeeAddr[8];

  memcpy( ieeeAddr, &(pMsg->ieeeaddr), 8 );
  cInfo = convertCapabilityInfo( pMsg->capabilities );

  status = (uint8) ZDP_DeviceAnnce( pMsg->nwkaddr, ieeeAddr, cInfo, FALSE );

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn      processDevZDOCBReq
 *
 * @brief   Process incoming Device ZDO Callback Request.
 *
 * @param   connection - connection handle (tcp)
 * @param   pMsg - pointer to Device ZDO Callback Request
 *
 * @return  none
 */
static void processDevZDOCBReq( int connection, DevZDOCBReq *pMsg )
{
  uint8 status = ZSTATUS_VALUES__ZSuccess;
  epItem_t *pItem;

  pItem = epTableFindEntryConnection( connection );
  if ( pItem == NULL )
  {
    // EP entry wasn't found, so create one with empty EP descriptor
    pItem = (epItem_t *) malloc( sizeof(epItem_t) );
    if ( pItem )
    {
      memset( pItem, 0, sizeof(epItem_t) );
      pItem->connection = connection;
      if ( epTableAddNewEntry( pItem ) == FALSE )
      {
        free( pItem );
        pItem = NULL;
        status = ZSTATUS_VALUES__ZAfEndpointMax;
      }
    }
  }

  if ( pItem != NULL )
  {
    if ( pMsg->has_srcrtgindcb )
    {
      if ( pMsg->srcrtgindcb )
      {
        pItem->zdoCBs |= ZS_ZDO_SRC_RTG_IND_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_SRC_RTG_IND_CBID;
      }
    }

    if ( pMsg->has_concentratorindcb )
    {
      if ( pMsg->concentratorindcb )
      {
        pItem->zdoCBs |= ZS_ZDO_CONCENTRATOR_IND_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_CONCENTRATOR_IND_CBID;
      }
    }

    if ( pMsg->has_nwkdisccnfcb )
    {
      if ( pMsg->nwkdisccnfcb )
      {
        pItem->zdoCBs |= ZS_ZDO_NWK_DISCOVERY_CNF_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_NWK_DISCOVERY_CNF_CBID;
      }
    }

    if ( pMsg->has_beaconnotindcb )
    {
      if ( pMsg->beaconnotindcb )
      {
        pItem->zdoCBs |= ZS_ZDO_BEACON_NOTIFY_IND_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_BEACON_NOTIFY_IND_CBID;
      }
    }

    if ( pMsg->has_joincnfcb )
    {
      if ( pMsg->joincnfcb )
      {
        pItem->zdoCBs |= ZS_ZDO_JOIN_CNF_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_JOIN_CNF_CBID;
      }
    }

    if ( pMsg->has_leavecnfcb )
    {
      if ( pMsg->leavecnfcb )
      {
        pItem->zdoCBs |= ZS_ZDO_LEAVE_CNF_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_LEAVE_CNF_CBID;
      }
    }

    if ( pMsg->has_leaveindcb )
    {
      if ( pMsg->leaveindcb )
      {
        pItem->zdoCBs |= ZS_ZDO_LEAVE_IND_CBID;
      }
      else
      {
        pItem->zdoCBs &= ~ZS_ZDO_LEAVE_IND_CBID;
      }
    }

    if ( pMsg->has_nwkaddrrsp )
    {
      if ( pMsg->nwkaddrrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_NWK_ADDR_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_NWK_ADDR_RSP_CDID;
      }
    }

    if ( pMsg->has_ieeeaddrrsp )
    {
      if ( pMsg->ieeeaddrrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_IEEE_ADDR_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_IEEE_ADDR_RSP_CDID;
      }
    }

    if ( pMsg->has_nodedescrsp )
    {
      if ( pMsg->nodedescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_NODE_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_NODE_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_powerdescrsp )
    {
      if ( pMsg->powerdescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_POWER_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_POWER_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_simpledescrsp )
    {
      if ( pMsg->simpledescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_SIMPLE_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_SIMPLE_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_activeendpointrsp )
    {
      if ( pMsg->activeendpointrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_ACTIVE_EP_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_ACTIVE_EP_RSP_CDID;
      }
    }

    if ( pMsg->has_matchdescrsp )
    {
      if ( pMsg->matchdescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MATCH_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MATCH_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_complexdescrsp )
    {
      if ( pMsg->complexdescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_COMPLEX_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_COMPLEX_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_userdescrsp )
    {
      if ( pMsg->userdescrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_USER_DESC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_USER_DESC_RSP_CDID;
      }
    }

    if ( pMsg->has_discoverycachersp )
    {
      if ( pMsg->discoverycachersp )
      {
        pItem->zdoRsps |= ZS_ZDO_DISCOVERY_CACHE_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_DISCOVERY_CACHE_RSP_CDID;
      }
    }

    if ( pMsg->has_userdesccnf )
    {
      if ( pMsg->userdesccnf )
      {
        pItem->zdoRsps |= ZS_ZDO_USER_DESC_CONF_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_USER_DESC_CONF_CDID;
      }
    }

    if ( pMsg->has_serverdiscoveryrsp )
    {
      if ( pMsg->serverdiscoveryrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_SERVER_DISCOVERY_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_SERVER_DISCOVERY_RSP_CDID;
      }
    }

    if ( pMsg->has_enddevicetimeoutrsp )
    {
      if ( pMsg->enddevicetimeoutrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_END_DEVICE_TIMEOUT_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_END_DEVICE_TIMEOUT_RSP_CDID;
      }
    }

    if ( pMsg->has_bindrsp )
    {
      if ( pMsg->bindrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_BIND_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_BIND_RSP_CDID;
      }
    }

    if ( pMsg->has_enddevicebindrsp )
    {
      if ( pMsg->enddevicebindrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_END_DEVICE_BIND_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_END_DEVICE_BIND_RSP_CDID;
      }
    }

    if ( pMsg->has_unbindrsp )
    {
      if ( pMsg->unbindrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_UNBIND_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_UNBIND_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtnwkdiscrsp )
    {
      if ( pMsg->mgmtnwkdiscrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_NWK_DISC_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_NWK_DISC_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtlqirsp )
    {
      if ( pMsg->mgmtlqirsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_LQI_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_LQI_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtrtgrsp )
    {
      if ( pMsg->mgmtrtgrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_RTG_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_RTG_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtbindrsp )
    {
      if ( pMsg->mgmtbindrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_BIND_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_BIND_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtleaversp )
    {
      if ( pMsg->mgmtleaversp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_LEAVE_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_LEAVE_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtdirectjoinrsp )
    {
      if ( pMsg->mgmtdirectjoinrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtpermitjoinrsp )
    {
      if ( pMsg->mgmtpermitjoinrsp )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID;
      }
    }

    if ( pMsg->has_mgmtnwkupdatenotify )
    {
      if ( pMsg->mgmtnwkupdatenotify )
      {
        pItem->zdoRsps |= ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID;
      }
    }

    if ( pMsg->has_deviceannounce )
    {
      if ( pMsg->deviceannounce )
      {
        pItem->zdoRsps |= ZS_ZDO_DEVICE_ANNOUNCE_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_ZDO_DEVICE_ANNOUNCE_CDID;
      }
    }

    if ( pMsg->has_devstatechange )
    {
      if ( pMsg->devstatechange )
      {
        pItem->zdoRsps |= ZS_DEV_STATE_CHANGE_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_DEV_STATE_CHANGE_CDID;
      }
    }

    if ( pMsg->has_devjammerind )
    {
      if ( pMsg->devjammerind )
      {
        pItem->zdoRsps |= ZS_DEV_JAMMER_IND_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_DEV_JAMMER_IND_CDID;
      }
    }

    if ( pMsg->has_tcdeviceind )
    {
      if ( pMsg->tcdeviceind )
      {
        pItem->zdoRsps |= ZS_TC_DEVICE_IND_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_TC_DEVICE_IND_CDID;
      }
    }

    if ( pMsg->has_devpermitjoinind )
    {
      if ( pMsg->devpermitjoinind )
      {
        pItem->zdoRsps |= ZS_DEV_PERMIT_JOIN_IND_CDID;
      }
      else
      {
        pItem->zdoRsps &= ~ZS_DEV_PERMIT_JOIN_IND_CDID;
      }
    }
  }

  sendDefaultRsp( connection, pMsg->cmdid, (ZStatusValues) status );
}

/*********************************************************************
 * @fn          sendMsgToAllCBs
 *
 * @brief       Send a message to all that is subscribed to ZDO CB functions
 *
 * @param       pStr - pointer to concentrator information
 *
 * @return      none
 */
static void sendMsgToAllCBs( int len, uint8 *pBuf, uint16 cbMask, uint8 cmdId )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( (srch->zdoCBs & cbMask) == cbMask )
    {
      // Send the AREQ message
      APIS_SendData( srch->connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, TRUE,
          cmdId, len, pBuf );
    }

    srch = srch->next;
  }
}

/*********************************************************************
 * @fn          sendMsgToAllCBMsgs
 *
 * @brief       Send a message to all that is subscribed to ZDO CB Messages
 *
 * @param       pStr - pointer to concentrator information
 *
 * @return      none
 */
static void sendMsgToAllCBMsgs( int len, uint8 *pBuf, uint32 cbMask,
                                uint8 cmdId )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( (srch->zdoRsps & cbMask) == cbMask )
    {
      // Send the AREQ message
      APIS_SendData( srch->connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, TRUE,
          cmdId, len, pBuf );
    }

    srch = srch->next;
  }
}

/*********************************************************************
 * @fn          zdoSrcRtgCB
 *
 * @brief       callback function to handle Source Route indication
 *
 * @param       pStr - pointer to source route information
 *
 * @return      NULL
 */
void *zdoSrcRtgCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoSrcRtgInd srcRtgInd = ZDO_SRC_RTG_IND__INIT;
  zdoSrcRtg_t *pSrcRtg = (zdoSrcRtg_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd Source Route Ind: srcAddr:%.4X, relayCnt:%d\n",
      pSrcRtg->srcAddr, pSrcRtg->relayCnt );

  srcRtgInd.cmdid = ZSTACK_CMD_IDS__ZDO_SRC_RTG_IND;
  srcRtgInd.n_relay = pSrcRtg->relayCnt;
  srcRtgInd.srcaddr = pSrcRtg->srcAddr;
  if ( pSrcRtg->relayCnt )
  {
    int i;
    srcRtgInd.relay = (uint32_t *) malloc(
        sizeof(uint32_t) * pSrcRtg->relayCnt );
    if ( srcRtgInd.relay == NULL )
    {
      return (NULL);
    }

    for ( i = 0; i < pSrcRtg->relayCnt; i++ )
    {
      srcRtgInd.relay[i] = pSrcRtg->pRelayList[i];
    }
  }

  len = zdo_src_rtg_ind__get_packed_size( &srcRtgInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_src_rtg_ind__pack( &srcRtgInd, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_SRC_RTG_IND_CBID,
        ZSTACK_CMD_IDS__ZDO_SRC_RTG_IND );

    free( pBuf );
  }

  if ( srcRtgInd.relay )
  {
    free( srcRtgInd.relay );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zdoConcentratorIndCB
 *
 * @brief       callback function to handle Concentrator indication
 *
 * @param       pStr - pointer to concentrator information
 *
 * @return      NULL
 */
void *zdoConcentratorIndCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoConcentratorInd conInd = ZDO_CONCENTRATOR_IND__INIT;
  zdoConcentratorInd_t *pConInd = (zdoConcentratorInd_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd ZDO Concentrator Ind\n" );

  conInd.cmdid = ZSTACK_CMD_IDS__ZDO_CONCENTRATOR_IND;
  conInd.nwkaddr = pConInd->nwkAddr;
  conInd.pktcost = pConInd->pktCost;
  memcpy( &conInd.ieeeaddr, pConInd->extAddr, 8 );

  len = zdo_concentrator_ind__get_packed_size( &conInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_concentrator_ind__pack( &conInd, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_CONCENTRATOR_IND_CBID,
        ZSTACK_CMD_IDS__ZDO_CONCENTRATOR_IND );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zdoNwkDiscCnfCB
 *
 * @brief       callback function to handle Network Discovery Confirm
 *
 * @param       pStr - pointer to Network Discovery Confirm status
 *
 * @return      NULL
 */
void *zdoNwkDiscCnfCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoNwkDiscCnf nwkDiscCnf = ZDO_NWK_DISC_CNF__INIT;
  uint8 *pStatus = (uint8 *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd Network Discovery Confirm\n" );

  nwkDiscCnf.cmdid = ZSTACK_CMD_IDS__ZDO_NWK_DISC_CNF;
  nwkDiscCnf.status = *pStatus;

  len = zdo_nwk_disc_cnf__get_packed_size( &nwkDiscCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_nwk_disc_cnf__pack( &nwkDiscCnf, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_NWK_DISCOVERY_CNF_CBID,
        ZSTACK_CMD_IDS__ZDO_NWK_DISC_CNF );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zdoBeaconNotifyIndCB
 *
 * @brief       callback function to handle beacon notify indication
 *
 * @param       pStr - pointer to beacon information
 *
 * @return      NULL
 */
void *zdoBeaconNotifyIndCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoBeaconNotifyInd beaconInd = ZDO_BEACON_NOTIFY_IND__INIT;
  NLME_beaconInd_t *pBeaconInd = (NLME_beaconInd_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd Beacon Notify Ind 1\n" );

  beaconInd.cmdid = ZSTACK_CMD_IDS__ZDO_BEACON_NOTIFY_IND;
  beaconInd.sourceaddr = pBeaconInd->sourceAddr;
  beaconInd.panid = pBeaconInd->panID;
  beaconInd.logicalchannel = pBeaconInd->logicalChannel;

  beaconInd.permitjoining = pBeaconInd->permitJoining;
  beaconInd.routercapacity = pBeaconInd->routerCapacity;
  beaconInd.devicecapacity = pBeaconInd->deviceCapacity;

  beaconInd.protocolversion = pBeaconInd->protocolVersion;
  beaconInd.stackprofile = pBeaconInd->stackProfile;
  beaconInd.lqi = pBeaconInd->LQI;
  beaconInd.depth = pBeaconInd->depth;
  beaconInd.updateid = pBeaconInd->updateID;
  memcpy( &beaconInd.extendedpanid, pBeaconInd->extendedPanID, 8 );

  len = zdo_beacon_notify_ind__get_packed_size( &beaconInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_beacon_notify_ind__pack( &beaconInd, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_BEACON_NOTIFY_IND_CBID,
        ZSTACK_CMD_IDS__ZDO_BEACON_NOTIFY_IND );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zdoJoinCnfCB
 *
 * @brief       callback function to handle Join Confirm
 *
 * @param       pStr - pointer to join confirm information
 *
 * @return      NULL
 */
void *zdoJoinCnfCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoJoinCnf joinCnf = ZDO_JOIN_CNF__INIT;
  zdoJoinCnf_t *pJoinCnf = (zdoJoinCnf_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd Join Confirm\n" );

  joinCnf.cmdid = ZSTACK_CMD_IDS__ZDO_JOIN_CNF;
  joinCnf.status = pJoinCnf->status;
  joinCnf.devaddr = pJoinCnf->deviceAddr;
  joinCnf.parentaddr = pJoinCnf->parentAddr;

  len = zdo_join_cnf__get_packed_size( &joinCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_join_cnf__pack( &joinCnf, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_JOIN_CNF_CBID,
        ZSTACK_CMD_IDS__ZDO_JOIN_CNF );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zdoLeaveCnfCB
 *
 * @brief       callback function to handle Leave Confirm
 *
 * @param       pStr - pointer to leave confirm information
 *
 * @return      NULL
 */
void *zdoLeaveCnfCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoLeaveCnf leaveCnf = ZDO_LEAVE_CNF__INIT;
  NLME_LeaveCnf_t *pLeaveCnf = (NLME_LeaveCnf_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd ZDO Leave Confirm\n" );

  leaveCnf.cmdid = ZSTACK_CMD_IDS__ZDO_LEAVE_CNF;
  leaveCnf.dstaddr = pLeaveCnf->dstAddr;
  leaveCnf.rejoin = pLeaveCnf->rejoin;
  leaveCnf.removechildren = pLeaveCnf->removeChildren;
  memcpy( &leaveCnf.extendedaddr, pLeaveCnf->extAddr, 8 );

  len = zdo_leave_cnf__get_packed_size( &leaveCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_leave_cnf__pack( &leaveCnf, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_LEAVE_CNF_CBID,
        ZSTACK_CMD_IDS__ZDO_LEAVE_CNF );

    free( pBuf );
  }

// TBD: what to return
  return (NULL);
}

/*********************************************************************
 * @fn          zdoLeaveIndCB
 *
 * @brief       callback function to handle Leave indication
 *
 * @param       pStr - pointer to leave information
 *
 * @return      NULL
 */
void *zdoLeaveIndCB( void *pStr )
{
  int len;
  uint8 *pBuf;
  ZdoLeaveInd leaveInd = ZDO_LEAVE_IND__INIT;
  NLME_LeaveInd_t *pLeaveInd = (NLME_LeaveInd_t *) pStr;

  uiPrintfEx(trINFO, "zstackpb Rcvd ZDO Leave Ind\n" );

  leaveInd.cmdid = ZSTACK_CMD_IDS__ZDO_LEAVE_IND;
  leaveInd.srcaddr = pLeaveInd->srcAddr;
  leaveInd.request = pLeaveInd->request;
  leaveInd.rejoin = pLeaveInd->rejoin;
  leaveInd.removechildren = pLeaveInd->removeChildren;
  memcpy( &leaveInd.extendedaddr, pLeaveInd->extAddr, 8 );

  len = zdo_leave_ind__get_packed_size( &leaveInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_leave_ind__pack( &leaveInd, pBuf );

    sendMsgToAllCBs( len, pBuf, ZS_ZDO_LEAVE_IND_CBID,
        ZSTACK_CMD_IDS__ZDO_LEAVE_IND );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          sendNwkAddrRsp
 *
 * @brief       Send a Nwk Addr Rsp indication
 *
 * @param       pAddrRsp - pointer to nwk addr rsp message
 *
 * @return      NULL
 */
static void sendNwkAddrRsp( ZDO_NwkIEEEAddrResp_t *pAddrRsp )
{
  int len;
  uint8 *pBuf;
  ZdoNwkAddrRspInd nwkAddrRsp = ZDO_NWK_ADDR_RSP_IND__INIT;

  nwkAddrRsp.cmdid = ZSTACK_CMD_IDS__ZDO_NWK_ADDR_RSP;
  nwkAddrRsp.status = pAddrRsp->status;
  nwkAddrRsp.nwkaddr = pAddrRsp->nwkAddr;
  memcpy( &nwkAddrRsp.ieeeaddr, pAddrRsp->extAddr, 8 );
  nwkAddrRsp.n_assocdevlist = pAddrRsp->numAssocDevs;
  if ( nwkAddrRsp.n_assocdevlist )
  {
    nwkAddrRsp.assocdevlist = (uint32_t *) malloc(
        sizeof(uint32_t) * pAddrRsp->numAssocDevs );
    if ( nwkAddrRsp.assocdevlist )
    {
      int i;
      for ( i = 0; i < nwkAddrRsp.n_assocdevlist; i++ )
      {
        nwkAddrRsp.assocdevlist[i] = pAddrRsp->devList[i];
      }
    }
    else
    {
      nwkAddrRsp.n_assocdevlist = 0;
    }
  }

  len = zdo_nwk_addr_rsp_ind__get_packed_size( &nwkAddrRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_nwk_addr_rsp_ind__pack( &nwkAddrRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_NWK_ADDR_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_NWK_ADDR_RSP );

    free( pBuf );
  }

  if ( nwkAddrRsp.assocdevlist )
  {
    free( nwkAddrRsp.assocdevlist );
  }
}

/*********************************************************************
 * @fn          sendIeeeAddrRsp
 *
 * @brief       Send a IEEE Addr Rsp indication
 *
 * @param       pAddrRsp - pointer to IEEE addr rsp message
 *
 * @return      NULL
 */
static void sendIeeeAddrRsp( ZDO_NwkIEEEAddrResp_t *pAddrRsp )
{
  int len;
  uint8 *pBuf;
  ZdoIeeeAddrRspInd ieeeAddrRsp = ZDO_IEEE_ADDR_RSP_IND__INIT;

  ieeeAddrRsp.cmdid = ZSTACK_CMD_IDS__ZDO_IEEE_ADDR_RSP;
  ieeeAddrRsp.status = pAddrRsp->status;
  ieeeAddrRsp.nwkaddr = pAddrRsp->nwkAddr;
  memcpy( &ieeeAddrRsp.ieeeaddr, pAddrRsp->extAddr, 8 );
  ieeeAddrRsp.n_assocdevlist = pAddrRsp->numAssocDevs;
  if ( ieeeAddrRsp.n_assocdevlist )
  {
    ieeeAddrRsp.assocdevlist = (uint32_t *) malloc(
        sizeof(uint32_t) * pAddrRsp->numAssocDevs );
    if ( ieeeAddrRsp.assocdevlist )
    {
      int i;
      for ( i = 0; i < ieeeAddrRsp.n_assocdevlist; i++ )
      {
        ieeeAddrRsp.assocdevlist[i] = pAddrRsp->devList[i];
      }
    }
    else
    {
      ieeeAddrRsp.n_assocdevlist = 0;
    }
  }

  len = zdo_ieee_addr_rsp_ind__get_packed_size( &ieeeAddrRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_ieee_addr_rsp_ind__pack( &ieeeAddrRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_IEEE_ADDR_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_IEEE_ADDR_RSP );

    free( pBuf );
  }

  if ( ieeeAddrRsp.assocdevlist )
  {
    free( ieeeAddrRsp.assocdevlist );
  }
}

/*********************************************************************
 * @fn          sendNodeDescRsp
 *
 * @brief       Send a Node Descriptor Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pNdRsp - pointer to Node Desc rsp message
 *
 * @return      NULL
 */
static void sendNodeDescRsp( uint16 srcAddr, ZDO_NodeDescRsp_t *pNdRsp )
{
  int len;
  uint8 *pBuf;
  ServerCapabilities srvCap = SERVER_CAPABILITIES__INIT;
  CapabilityInfo capInfo = CAPABILITY_INFO__INIT;
  NodeDescriptor nodeDesc = NODE_DESCRIPTOR__INIT;
  ZdoNodeDescRspInd ndRsp = ZDO_NODE_DESC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Node Descriptor Response Ind\n" );

  ndRsp.cmdid = ZSTACK_CMD_IDS__ZDO_NODE_DESC_RSP;
  ndRsp.status = pNdRsp->status;
  ndRsp.srcaddr = srcAddr;
  ndRsp.nwkaddrofinterest = pNdRsp->nwkAddr;

  buildPBcapInfo( pNdRsp->nodeDesc.CapabilityFlags, &capInfo );
  buildPBserverCap( pNdRsp->nodeDesc.ServerMask, &srvCap );

  nodeDesc.apsflags = pNdRsp->nodeDesc.APSFlags;
  nodeDesc.complexdescavail = pNdRsp->nodeDesc.ComplexDescAvail;
  nodeDesc.desccap = pNdRsp->nodeDesc.DescriptorCapability;
  nodeDesc.freqband = pNdRsp->nodeDesc.FrequencyBand;
  nodeDesc.logicaltype = pNdRsp->nodeDesc.LogicalType;
  nodeDesc.manufacturercode = BUILD_UINT16(
      pNdRsp->nodeDesc.ManufacturerCode[0],
      pNdRsp->nodeDesc.ManufacturerCode[1] );
  nodeDesc.maxbuffersize = pNdRsp->nodeDesc.MaxBufferSize;
  nodeDesc.maxintransfersize = BUILD_UINT16(
      pNdRsp->nodeDesc.MaxInTransferSize[0],
      pNdRsp->nodeDesc.MaxInTransferSize[1] );
  nodeDesc.maxouttransfersize = BUILD_UINT16(
      pNdRsp->nodeDesc.MaxOutTransferSize[0],
      pNdRsp->nodeDesc.MaxOutTransferSize[1] );
  nodeDesc.userdescavail = pNdRsp->nodeDesc.UserDescAvail;

  nodeDesc.capinfo = &capInfo;
  nodeDesc.servermask = &srvCap;
  ndRsp.nodedesc = &nodeDesc;

  len = zdo_node_desc_rsp_ind__get_packed_size( &ndRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_node_desc_rsp_ind__pack( &ndRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_NODE_DESC_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_NODE_DESC_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendPowerDescRsp
 *
 * @brief       Send a Power Descriptor Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pPowerRsp - pointer to Power Desc rsp message
 *
 * @return      NULL
 */
static void sendPowerDescRsp( uint16 srcAddr, ZDO_PowerRsp_t *pPowerRsp )
{
  int len;
  uint8 *pBuf;
  PowerSource availPwr = POWER_SOURCE__INIT;
  PowerSource currentPwr = POWER_SOURCE__INIT;
  PowerDescriptor pwrDesc = POWER_DESCRIPTOR__INIT;
  ZdoPowerDescRspInd pwrRsp = ZDO_POWER_DESC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Power Descriptor Response Ind\n" );

  pwrRsp.cmdid = ZSTACK_CMD_IDS__ZDO_POWER_DESC_RSP;
  pwrRsp.status = pPowerRsp->status;
  pwrRsp.srcaddr = srcAddr;
  pwrRsp.nwkaddrofinterest = pPowerRsp->nwkAddr;

  buildPBpowerSource( pPowerRsp->pwrDesc.AvailablePowerSources, &availPwr );
  buildPBpowerSource( pPowerRsp->pwrDesc.CurrentPowerSource, &currentPwr );
  pwrDesc.availpowersource = &availPwr;
  pwrDesc.currentpowersource = &currentPwr;

  pwrDesc.currentpowerlevel = pPowerRsp->pwrDesc.CurrentPowerSourceLevel;
  pwrDesc.powermode = pPowerRsp->pwrDesc.PowerMode;

  pwrRsp.powerdesc = &pwrDesc;

  len = zdo_power_desc_rsp_ind__get_packed_size( &pwrRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_power_desc_rsp_ind__pack( &pwrRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_POWER_DESC_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_POWER_DESC_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendSimpleDescRsp
 *
 * @brief       Send a Simple Descriptor Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pSimpleRsp - pointer to Simple Desc rsp message
 *
 * @return      NULL
 */
static void sendSimpleDescRsp( uint16 srcAddr, ZDO_SimpleDescRsp_t *pSimpleRsp )
{
  int i;
  int len;
  uint8 *pBuf;
  SimpleDescriptor simpleDesc = SIMPLE_DESCRIPTOR__INIT;
  ZdoSimpleDescRspInd simpleRsp = ZDO_SIMPLE_DESC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Simple Descriptor Response Ind\n" );

  simpleRsp.cmdid = ZSTACK_CMD_IDS__ZDO_SIMPLE_DESC_RSP;
  simpleRsp.status = pSimpleRsp->status;
  simpleRsp.srcaddr = srcAddr;
  simpleRsp.nwkaddrofinterest = pSimpleRsp->nwkAddr;

  simpleDesc.deviceid = pSimpleRsp->simpleDesc.AppDeviceId;
  simpleDesc.devicever = pSimpleRsp->simpleDesc.AppDevVer;
  simpleDesc.endpoint = pSimpleRsp->simpleDesc.EndPoint;
  simpleDesc.profileid = pSimpleRsp->simpleDesc.AppProfId;
  simpleDesc.n_inputclusters = pSimpleRsp->simpleDesc.AppNumInClusters;
  simpleDesc.n_outputclusters = pSimpleRsp->simpleDesc.AppNumOutClusters;

  simpleDesc.inputclusters = (uint32_t *) malloc(
      sizeof(uint32_t) * pSimpleRsp->simpleDesc.AppNumInClusters );
  if ( simpleDesc.inputclusters )
  {
    for ( i = 0; i < simpleDesc.n_inputclusters; i++ )
    {
      simpleDesc.inputclusters[i] = pSimpleRsp->simpleDesc.pAppInClusterList[i];
    }
  }

  simpleDesc.outputclusters = (uint32_t *) malloc(
      sizeof(uint32_t) * pSimpleRsp->simpleDesc.AppNumOutClusters );
  if ( simpleDesc.outputclusters )
  {
    for ( i = 0; i < simpleDesc.n_outputclusters; i++ )
    {
      simpleDesc.outputclusters[i] =
          pSimpleRsp->simpleDesc.pAppOutClusterList[i];
    }
  }

  simpleRsp.simpledesc = &simpleDesc;

  len = zdo_simple_desc_rsp_ind__get_packed_size( &simpleRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_simple_desc_rsp_ind__pack( &simpleRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_SIMPLE_DESC_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_SIMPLE_DESC_RSP );

    free( pBuf );
  }

  if ( simpleDesc.inputclusters )
  {
    free( simpleDesc.inputclusters );
  }

  if ( simpleDesc.outputclusters )
  {
    free( simpleDesc.outputclusters );
  }
}

/*********************************************************************
 * @fn          sendActiveEPRsp
 *
 * @brief       Send a Active Endpoint Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pActiveEPRsp - pointer to Active EP rsp message
 *
 * @return      NULL
 */
static void sendActiveEPRsp( uint16 srcAddr,
                             ZDO_ActiveEndpointRsp_t *pActiveEPRsp )
{
  int len;
  uint8 *pBuf;
  ZdoActiveEndpointsRspInd epRsp = ZDO_ACTIVE_ENDPOINTS_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Active Endpoints Response Ind\n" );

  epRsp.cmdid = ZSTACK_CMD_IDS__ZDO_ACTIVE_EP_RSP;
  epRsp.status = pActiveEPRsp->status;
  epRsp.srcaddr = srcAddr;
  epRsp.nwkaddrofinterest = pActiveEPRsp->nwkAddr;

  epRsp.n_activeeplist = pActiveEPRsp->cnt;

  epRsp.activeeplist = (uint32_t *) malloc(
      sizeof(uint32_t) * pActiveEPRsp->cnt );
  if ( epRsp.activeeplist )
  {
    int i;
    for ( i = 0; i < epRsp.n_activeeplist; i++ )
    {
      epRsp.activeeplist[i] = pActiveEPRsp->epList[i];
    }
  }

  len = zdo_active_endpoints_rsp_ind__get_packed_size( &epRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_active_endpoints_rsp_ind__pack( &epRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_ACTIVE_EP_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_ACTIVE_EP_RSP );

    free( pBuf );
  }

  if ( epRsp.activeeplist )
  {
    free( epRsp.activeeplist );
  }
}

/*********************************************************************
 * @fn          sendMatchDescRsp
 *
 * @brief       Send a MatchDesc Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pActiveEPRsp - pointer to Active EP rsp message
 *
 * @return      NULL
 */
static void sendMatchDescRsp( uint16 srcAddr,
                              ZDO_ActiveEndpointRsp_t *pActiveEPRsp )
{
  int len;
  uint8 *pBuf;
  ZdoMatchDescRspInd mdRsp = ZDO_MATCH_DESC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Match Descriptor Response Ind\n" );

  mdRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MATCH_DESC_RSP;
  mdRsp.status = pActiveEPRsp->status;
  mdRsp.srcaddr = srcAddr;
  mdRsp.nwkaddrofinterest = pActiveEPRsp->nwkAddr;

  mdRsp.n_matchlist = pActiveEPRsp->cnt;

  mdRsp.matchlist = (uint32_t *) malloc( sizeof(uint32_t) * pActiveEPRsp->cnt );
  if ( mdRsp.matchlist )
  {
    int i;
    for ( i = 0; i < mdRsp.n_matchlist; i++ )
    {
      mdRsp.matchlist[i] = pActiveEPRsp->epList[i];
    }
  }

  len = zdo_match_desc_rsp_ind__get_packed_size( &mdRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_match_desc_rsp_ind__pack( &mdRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MATCH_DESC_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_MATCH_DESC_RSP );

    free( pBuf );
  }

  if ( mdRsp.matchlist )
  {
    free( mdRsp.matchlist );
  }
}

/*********************************************************************
 * @fn          sendUserDescRsp
 *
 * @brief       Send a User Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pUdRsp - pointer to User Descriptor rsp message
 *
 * @return      NULL
 */
static void sendUserDescRsp( uint16 srcAddr, ZDO_UserDescRsp_t *pUdRsp )
{
  int len;
  uint8 *pBuf;
  ZdoUserDescRspInd udRsp = ZDO_USER_DESC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending User Descriptor Response Ind\n" );

  udRsp.cmdid = ZSTACK_CMD_IDS__ZDO_USER_DESC_RSP;
  udRsp.status = pUdRsp->status;
  udRsp.srcaddr = srcAddr;
  udRsp.nwkaddrofinterest = pUdRsp->nwkAddr;

  udRsp.desc.len = pUdRsp->length;

  udRsp.desc.data = malloc( pUdRsp->length );
  if ( udRsp.desc.data )
  {
    memcpy( udRsp.desc.data, pUdRsp->desc, pUdRsp->length );
  }

  len = zdo_user_desc_rsp_ind__get_packed_size( &udRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_user_desc_rsp_ind__pack( &udRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_USER_DESC_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_USER_DESC_RSP );

    free( pBuf );
  }

  if ( udRsp.desc.data )
  {
    free( udRsp.desc.data );
  }
}

/*********************************************************************
 * @fn          sendServerDiscRsp
 *
 * @brief       Send a Server Discovery Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pSdRsp - pointer to Server Discovery rsp message
 *
 * @return      NULL
 */
static void sendServerDiscRsp( uint16 srcAddr, ZDO_ServerDiscRsp_t *pSdRsp )
{
  int len;
  uint8 *pBuf;
  ServerCapabilities srvCap = SERVER_CAPABILITIES__INIT;
  ZdoServerDiscoveryRspInd sdRsp = ZDO_SERVER_DISCOVERY_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Server Discovery Response Ind\n" );

  sdRsp.cmdid = ZSTACK_CMD_IDS__ZDO_SERVER_DISC_RSP;
  sdRsp.status = pSdRsp->status;
  sdRsp.srcaddr = srcAddr;

  buildPBserverCap( pSdRsp->serverMask, &srvCap );
  sdRsp.servercap = &srvCap;

  len = zdo_server_discovery_rsp_ind__get_packed_size( &sdRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_server_discovery_rsp_ind__pack( &sdRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_SERVER_DISCOVERY_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_SERVER_DISC_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendEndDeviceTimeoutRsp
 *
 * @brief       Send a End Device Timeout Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       timeout - timeout value
 *
 * @return      NULL
 */
static void sendEndDeviceTimeoutRsp( uint16 srcAddr, uint16 timeout )
{
  int len;
  uint8 *pBuf;
  ZdoEndDeviceTimeoutRspInd edtRsp = ZDO_END_DEVICE_TIMEOUT_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending End Device Timeout Response Ind\n" );

  edtRsp.cmdid = ZSTACK_CMD_IDS__ZDO_END_DEVICE_TIMEOUT_RSP;
  edtRsp.status = ZSTATUS_VALUES__ZSuccess;
  edtRsp.srcaddr = srcAddr;
  edtRsp.timeout = timeout;

  len = zdo_end_device_timeout_rsp_ind__get_packed_size( &edtRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_end_device_timeout_rsp_ind__pack( &edtRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_END_DEVICE_TIMEOUT_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_END_DEVICE_TIMEOUT_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendBindRsp
 *
 * @brief       Send a Bind Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - bind result
 *
 * @return      NULL
 */
static void sendBindRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoBindRspInd bindRsp = ZDO_BIND_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Bind Response Ind\n" );

  bindRsp.cmdid = ZSTACK_CMD_IDS__ZDO_BIND_RSP;
  bindRsp.status = result;
  bindRsp.srcaddr = srcAddr;

  len = zdo_bind_rsp_ind__get_packed_size( &bindRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_bind_rsp_ind__pack( &bindRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_BIND_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_BIND_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendEndDeviceBindRsp
 *
 * @brief       Send a End Device Bind Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - end device bind result
 *
 * @return      NULL
 */
static void sendEndDeviceBindRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoEndDeviceBindRspInd bindRsp = ZDO_END_DEVICE_BIND_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending End Device Bind Response Ind\n" );

  bindRsp.cmdid = ZSTACK_CMD_IDS__ZDO_END_DEVICE_BIND_RSP;
  bindRsp.status = result;
  bindRsp.srcaddr = srcAddr;

  len = zdo_end_device_bind_rsp_ind__get_packed_size( &bindRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_end_device_bind_rsp_ind__pack( &bindRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_END_DEVICE_BIND_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_END_DEVICE_BIND_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendUnbindRsp
 *
 * @brief       Send a Unbind Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - unbind result
 *
 * @return      NULL
 */
static void sendUnbindRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoUnbindRspInd unbindRsp = ZDO_UNBIND_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Unbind Response Ind\n" );

  unbindRsp.cmdid = ZSTACK_CMD_IDS__ZDO_UNBIND_RSP;
  unbindRsp.status = result;
  unbindRsp.srcaddr = srcAddr;

  len = zdo_unbind_rsp_ind__get_packed_size( &unbindRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_unbind_rsp_ind__pack( &unbindRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_UNBIND_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_UNBIND_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendMgmtNwkDiscRsp
 *
 * @brief       Send a Management Network Discovery Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pNwkDiscRsp - Mgmt Network Disc Rsp
 *
 * @return      NULL
 */
static void sendMgmtNwkDiscRsp( uint16 srcAddr,
                                ZDO_MgmNwkDiscRsp_t *pNwkDiscRsp )
{
  uint8 *pBuf;
  ZdoMgmtNwkDiscRspInd ndRsp = ZDO_MGMT_NWK_DISC_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Network Discovery Response Ind\n" );

  ndRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_NWK_DISC_RSP;
  ndRsp.status = pNwkDiscRsp->status;
  ndRsp.srcaddr = srcAddr;
  ndRsp.networkcount = pNwkDiscRsp->networkCount;
  ndRsp.startindex = pNwkDiscRsp->startIndex;
  ndRsp.n_netlist = pNwkDiscRsp->networkListCount;
  ndRsp.netlist = (NwkDiscItem **) malloc(
      sizeof(NwkDiscItem *) * ndRsp.n_netlist );
  if ( ndRsp.netlist )
  {
    int len;
    int x;
    for ( x = 0; x < ndRsp.n_netlist; x++ )
    {
      ndRsp.netlist[x] = (NwkDiscItem *) malloc( sizeof(NwkDiscItem) );
      nwk_disc_item__init( ndRsp.netlist[x] );
      memcpy( &(ndRsp.netlist[x]->extendedpanid),
          pNwkDiscRsp->list[x].extendedPANID, 8 );
      ndRsp.netlist[x]->logicalchan = pNwkDiscRsp->list[x].logicalChannel;
      ndRsp.netlist[x]->stackprofile = pNwkDiscRsp->list[x].stackProfile;
      ndRsp.netlist[x]->version = pNwkDiscRsp->list[x].version;
      ndRsp.netlist[x]->beaconorder = pNwkDiscRsp->list[x].beaconOrder;
      ndRsp.netlist[x]->superframeorder = pNwkDiscRsp->list[x].superFrameOrder;
      ndRsp.netlist[x]->permitjoin = pNwkDiscRsp->list[x].permitJoining;
    }

    len = zdo_mgmt_nwk_disc_rsp_ind__get_packed_size( &ndRsp );
    pBuf = malloc( len );
    if ( pBuf )
    {
      zdo_mgmt_nwk_disc_rsp_ind__pack( &ndRsp, pBuf );

      sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_NWK_DISC_RSP_CDID,
          ZSTACK_CMD_IDS__ZDO_MGMT_NWK_DISC_RSP );

      free( pBuf );
    }

    for ( x = 0; x < ndRsp.n_netlist; x++ )
    {
      free( ndRsp.netlist[x] );
    }
    free( ndRsp.netlist );
  }
}

/*********************************************************************
 * @fn          sendMgmtLqiRsp
 *
 * @brief       Send a Management LQI Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pLqiRsp - Mgmt LQI Rsp
 *
 * @return      NULL
 */
static void sendMgmtLqiRsp( uint16 srcAddr, ZDO_MgmtLqiRsp_t *pLqiRsp )
{
  uint8 *pBuf;
  ZdoMgmtLqiRspInd lqiRsp = ZDO_MGMT_LQI_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt LQI Response Ind\n" );

  lqiRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_LQI_RSP;
  lqiRsp.status = pLqiRsp->status;
  lqiRsp.srcaddr = srcAddr;
  lqiRsp.neighborlqientries = pLqiRsp->neighborLqiEntries;
  lqiRsp.startindex = pLqiRsp->startIndex;
  lqiRsp.n_lqilist = pLqiRsp->neighborLqiCount;
  lqiRsp.lqilist = (NeighborLqiItem**) malloc(
      sizeof(NeighborLqiItem*) * lqiRsp.n_lqilist );
  if ( lqiRsp.lqilist )
  {
    int len;
    int x;
    for ( x = 0; x < lqiRsp.n_lqilist; x++ )
    {
      lqiRsp.lqilist[x] = (NeighborLqiItem *) malloc( sizeof(NeighborLqiItem) );
      neighbor_lqi_item__init( lqiRsp.lqilist[x] );

      memcpy( &(lqiRsp.lqilist[x]->extendedpanid), pLqiRsp->list[x].extPanID,
          8 );
      memcpy( &(lqiRsp.lqilist[x]->extendedaddr), pLqiRsp->list[x].extAddr, 8 );
      lqiRsp.lqilist[x]->nwkaddr = pLqiRsp->list[x].nwkAddr;

      lqiRsp.lqilist[x]->devicetype = pLqiRsp->list[x].devType;
      lqiRsp.lqilist[x]->rxonwhenidle = pLqiRsp->list[x].rxOnIdle;
      lqiRsp.lqilist[x]->relationship = pLqiRsp->list[x].relation;
      lqiRsp.lqilist[x]->permit = pLqiRsp->list[x].permit;
      lqiRsp.lqilist[x]->depth = pLqiRsp->list[x].depth;
      lqiRsp.lqilist[x]->lqi = pLqiRsp->list[x].lqi;
    }

    len = zdo_mgmt_lqi_rsp_ind__get_packed_size( &lqiRsp );
    pBuf = malloc( len );
    if ( pBuf )
    {
      zdo_mgmt_lqi_rsp_ind__pack( &lqiRsp, pBuf );

      sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_LQI_RSP_CDID,
          ZSTACK_CMD_IDS__ZDO_MGMT_LQI_RSP );

      free( pBuf );
    }

    for ( x = 0; x < lqiRsp.n_lqilist; x++ )
    {
      free( lqiRsp.lqilist[x] );
    }
    free( lqiRsp.lqilist );
  }
}

/*********************************************************************
 * @fn          sendMgmtRtgRsp
 *
 * @brief       Send a Management Routing Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pRtgRsp - Mgmt Rtg Rsp
 *
 * @return      NULL
 */
static void sendMgmtRtgRsp( uint16 srcAddr, ZDO_MgmtRtgRsp_t *pRtgRsp )
{
  uint8 *pBuf;
  ZdoMgmtRtgRspInd rtgRsp = ZDO_MGMT_RTG_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Routing Response Ind\n" );

  rtgRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_RTG_RSP;
  rtgRsp.status = pRtgRsp->status;
  rtgRsp.srcaddr = srcAddr;
  rtgRsp.rtgentries = pRtgRsp->rtgCount;
  rtgRsp.startindex = pRtgRsp->startIndex;
  rtgRsp.n_rtglist = pRtgRsp->rtgListCount;
  rtgRsp.rtglist = (RtgItem**) malloc( sizeof(RtgItem*) * rtgRsp.n_rtglist );
  if ( rtgRsp.rtglist )
  {
    int len;
    int x;
    for ( x = 0; x < rtgRsp.n_rtglist; x++ )
    {
      rtgRsp.rtglist[x] = (RtgItem *) malloc( sizeof(RtgItem) );
      rtg_item__init( rtgRsp.rtglist[x] );

      rtgRsp.rtglist[x]->dstaddr = pRtgRsp->list[x].dstAddress;
      rtgRsp.rtglist[x]->nexthop = pRtgRsp->list[x].nextHopAddress;
      rtgRsp.rtglist[x]->status = pRtgRsp->list[x].status;

      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_MANYTOONE )
      {
        rtgRsp.rtglist[x]->manytoone = TRUE;
      }
      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_ROUTE_RECORD_REQUIRED )
      {
        rtgRsp.rtglist[x]->routerecordrequired = TRUE;
      }

      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_MEMORY_CONSTRAINED )
      {
        rtgRsp.rtglist[x]->memoryconstrained = TRUE;
      }
    }

    len = zdo_mgmt_rtg_rsp_ind__get_packed_size( &rtgRsp );
    pBuf = malloc( len );
    if ( pBuf )
    {
      zdo_mgmt_rtg_rsp_ind__pack( &rtgRsp, pBuf );

      sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_RTG_RSP_CDID,
          ZSTACK_CMD_IDS__ZDO_MGMT_RTG_RSP );

      free( pBuf );
    }

    for ( x = 0; x < rtgRsp.n_rtglist; x++ )
    {
      free( rtgRsp.rtglist[x] );
    }
    free( rtgRsp.rtglist );
  }
}

/*********************************************************************
 * @fn          sendMgmtBindRsp
 *
 * @brief       Send a Management Binding Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       pBindRsp - Mgmt Binding Rsp
 *
 * @return      NULL
 */
static void sendMgmtBindRsp( uint16 srcAddr, ZDO_MgmtBindRsp_t *pBindRsp )
{
  uint8 *pBuf;
  ZdoMgmtBindRspInd bindRsp = ZDO_MGMT_BIND_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Bind Response Ind\n" );

  bindRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_BIND_RSP;
  bindRsp.status = pBindRsp->status;
  bindRsp.srcaddr = srcAddr;
  bindRsp.bindentries = pBindRsp->bindingCount;
  bindRsp.startindex = pBindRsp->startIndex;
  bindRsp.n_bindlist = pBindRsp->bindingListCount;

  bindRsp.bindlist = (BindItem**) malloc(
      sizeof(BindItem*) * bindRsp.n_bindlist );
  if ( bindRsp.bindlist )
  {
    int len;
    int x;
    for ( x = 0; x < bindRsp.n_bindlist; x++ )
    {
      bindRsp.bindlist[x] = (BindItem*) malloc( sizeof(BindItem) );
      bind_item__init( bindRsp.bindlist[x] );

      memcpy( &(bindRsp.bindlist[x]->srcaddr), pBindRsp->list[x].srcAddr, 8 );
      bindRsp.bindlist[x]->srcendpoint = pBindRsp->list[x].srcEP;
      bindRsp.bindlist[x]->clustedid = pBindRsp->list[x].clusterID;

      bindRsp.bindlist[x]->dstaddr = (AFAddr*) malloc( sizeof(AFAddr) );
      afaddr__init( bindRsp.bindlist[x]->dstaddr );

      bindRsp.bindlist[x]->dstaddr->addrmode =
          pBindRsp->list[x].dstAddr.addrMode;
      if ( bindRsp.bindlist[x]->dstaddr->addrmode == AFADDR_MODE__EXT )
      {
        bindRsp.bindlist[x]->dstaddr->has_extaddr = TRUE;
        memcpy( &(bindRsp.bindlist[x]->dstaddr->extaddr),
            pBindRsp->list[x].dstAddr.addr.extAddr, 8 );
      }
      else
      {
        bindRsp.bindlist[x]->dstaddr->has_shortaddr = TRUE;
        bindRsp.bindlist[x]->dstaddr->shortaddr =
            pBindRsp->list[x].dstAddr.addr.shortAddr;
      }
      bindRsp.bindlist[x]->dstaddr->has_endpoint = TRUE;
      bindRsp.bindlist[x]->dstaddr->endpoint = pBindRsp->list[x].dstEP;
    }

    len = zdo_mgmt_bind_rsp_ind__get_packed_size( &bindRsp );
    pBuf = malloc( len );
    if ( pBuf )
    {
      zdo_mgmt_bind_rsp_ind__pack( &bindRsp, pBuf );

      sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_BIND_RSP_CDID,
          ZSTACK_CMD_IDS__ZDO_MGMT_BIND_RSP );

      free( pBuf );
    }

    for ( x = 0; x < bindRsp.n_bindlist; x++ )
    {
      free( bindRsp.bindlist[x]->dstaddr );
      free( bindRsp.bindlist[x] );
    }
    free( bindRsp.bindlist );
  }
}

/*********************************************************************
 * @fn          sendMgmtLeaveRsp
 *
 * @brief       Send a ZDO Management Leave Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - leave result
 *
 * @return      NULL
 */
static void sendMgmtLeaveRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoMgmtLeaveRspInd leaveRsp = ZDO_MGMT_LEAVE_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Leave Response Ind\n" );

  leaveRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_LEAVE_RSP;
  leaveRsp.status = result;
  leaveRsp.srcaddr = srcAddr;

  len = zdo_mgmt_leave_rsp_ind__get_packed_size( &leaveRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_mgmt_leave_rsp_ind__pack( &leaveRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_LEAVE_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_MGMT_LEAVE_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendMgmtDirectJoinRsp
 *
 * @brief       Send a ZDO Management Direct Join Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - direct join result
 *
 * @return      NULL
 */
static void sendMgmtDirectJoinRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoMgmtDirectJoinRspInd djRsp = ZDO_MGMT_DIRECT_JOIN_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Direct Join Response Ind\n" );

  djRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_DIRECT_JOIN_RSP;
  djRsp.status = result;
  djRsp.srcaddr = srcAddr;

  len = zdo_mgmt_direct_join_rsp_ind__get_packed_size( &djRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_mgmt_direct_join_rsp_ind__pack( &djRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_MGMT_DIRECT_JOIN_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendMgmtPermitJoinRsp
 *
 * @brief       Send a ZDO Management Permit Join Rsp indication
 *
 * @param       srcAddr - source address of message
 * @param       result - permit join result
 *
 * @return      NULL
 */
static void sendMgmtPermitJoinRsp( uint16 srcAddr, uint8 result )
{
  int len;
  uint8 *pBuf;
  ZdoMgmtPermitJoinRspInd pjRsp = ZDO_MGMT_PERMIT_JOIN_RSP_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Permit Join Response Ind\n" );

  pjRsp.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_PERMIT_JOIN_RSP;
  pjRsp.status = result;
  pjRsp.srcaddr = srcAddr;

  len = zdo_mgmt_permit_join_rsp_ind__get_packed_size( &pjRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_mgmt_permit_join_rsp_ind__pack( &pjRsp, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID,
        ZSTACK_CMD_IDS__ZDO_MGMT_PERMIT_JOIN_RSP );

    free( pBuf );
  }
}

/*********************************************************************
 * @fn          sendMgmtNwkUpdateNotify
 *
 * @brief       Send a Management Network Update Notify indication
 *
 * @param       srcAddr - source address of message
 * @param       pNUNotify - Mgmt Nwk Update Notify
 *
 * @return      NULL
 */
static void sendMgmtNwkUpdateNotify( uint16 srcAddr,
                                     ZDO_MgmtNwkUpdateNotify_t *pNotify )
{
  uint8 *pBuf;
  ZdoMgmtNwkUpdateNotifyInd nuNot = ZDO_MGMT_NWK_UPDATE_NOTIFY_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Mgmt Update Notify Response Ind\n" );

  nuNot.cmdid = ZSTACK_CMD_IDS__ZDO_MGMT_NWK_UPDATE_NOTIFY;
  nuNot.status = pNotify->status;
  nuNot.srcaddr = srcAddr;
  nuNot.scannedchannels = pNotify->scannedChannels;
  nuNot.totaltrans = pNotify->totalTransmissions;
  nuNot.transfails = pNotify->transmissionFailures;
  nuNot.n_energyvalueslist = pNotify->listCount;

  nuNot.energyvalueslist = malloc(
      sizeof(uint32_t) * nuNot.n_energyvalueslist );
  if ( nuNot.energyvalueslist )
  {
    int len;
    int x;
    for ( x = 0; x < nuNot.n_energyvalueslist; x++ )
    {
      nuNot.energyvalueslist[x] = pNotify->energyValues[x];
    }

    len = zdo_mgmt_nwk_update_notify_ind__get_packed_size( &nuNot );
    pBuf = malloc( len );
    if ( pBuf )
    {
      zdo_mgmt_nwk_update_notify_ind__pack( &nuNot, pBuf );

      sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID,
          ZSTACK_CMD_IDS__ZDO_MGMT_NWK_UPDATE_NOTIFY );

      free( pBuf );
    }

    free( nuNot.energyvalueslist );
  }
}

/**********************************************************************
 * @fn          sendDeviceAnnounce
 *
 * @brief       Send a ZDO Device Announce indication
 *
 * @param       srcAddr - source address of message
 * @param       pDevAnn - pointer to the device announce message
 *
 * @return      NULL
 */
void sendDeviceAnnounce( uint16 srcAddr, ZDO_DeviceAnnce_t *pDevAnn )
{
  int len;
  uint8 *pBuf;
  CapabilityInfo capInfo = CAPABILITY_INFO__INIT;
  ZdoDeviceAnnounceInd devInd = ZDO_DEVICE_ANNOUNCE_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Device Announce Ind\n" );

  devInd.cmdid = ZSTACK_CMD_IDS__ZDO_DEVICE_ANNOUNCE;
  devInd.srcaddr = srcAddr;
  devInd.devaddr = pDevAnn->nwkAddr;

  memcpy( &devInd.devextaddr, pDevAnn->extAddr, 8 );
  buildPBcapInfo( pDevAnn->capabilities, &capInfo );

  devInd.capinfo = &capInfo;

  len = zdo_device_announce_ind__get_packed_size( &devInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_device_announce_ind__pack( &devInd, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_ZDO_DEVICE_ANNOUNCE_CDID,
        ZSTACK_CMD_IDS__ZDO_DEVICE_ANNOUNCE );

    free( pBuf );
  }
}

/**********************************************************************
 * @fn          sendSysJammerInd
 *
 * @brief       Send a Sys Jammer indication
 *
 * @param       jammerInd - TRUE if transition from undetected to detected
 *                          FALSE if transition from detected to undetected
 *
 * @return      NULL
 */
void sendSysJammerInd( uint8 jammerInd )
{
  int len;
  uint8 *pBuf;
  DevJammerInd jamInd = DEV_JAMMER_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Device Jammer Ind: %d\n", jammerInd );

  jamInd.cmdid = ZSTACK_CMD_IDS__DEV_JAMMER_IND;
  if ( jammerInd )
  {
    jamInd.jammed = TRUE;
  }

  len = dev_jammer_ind__get_packed_size( &jamInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    dev_jammer_ind__pack( &jamInd, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_DEV_JAMMER_IND_CDID,
        ZSTACK_CMD_IDS__DEV_JAMMER_IND );

    free( pBuf );
  }
}

/**********************************************************************
 * @fn          sendTcDeviceInd
 *
 * @brief       Send a ZDO TC Device indication
 *
 * @param       params - pointer to device information
 *
 * @return      NULL
 */
void *sendTcDeviceInd( void *params )
{
  int len;
  uint8 *pBuf;
  ZDO_TC_Device_t *pDev = (ZDO_TC_Device_t *) params;
  ZdoTcDeviceInd devInd = ZDO_TC_DEVICE_IND__INIT;

  devInd.cmdid = ZSTACK_CMD_IDS__ZDO_TC_DEVICE_IND;
  devInd.nwkaddr = pDev->nwkAddr;
  memcpy( &devInd.extendedaddr, pDev->extAddr, 8 );
  devInd.parentaddr = pDev->parentAddr;

  uiPrintfEx(trINFO, "zstackpb Sending TC Device Ind: nwkAddr:0x%.4X, extAddr:0x%.16llX, parentAddr:0x%.4X\n",
      devInd.nwkaddr, devInd.extendedaddr, devInd.parentaddr );

  len = zdo_tc_device_ind__get_packed_size( &devInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    zdo_tc_device_ind__pack( &devInd, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_TC_DEVICE_IND_CDID,
        ZSTACK_CMD_IDS__ZDO_TC_DEVICE_IND );

    free( pBuf );
  }

  return (NULL);
}

/**********************************************************************
 * @fn          sendDevPermitJoinInd
 *
 * @brief       Send a Device Permit Join indication
 *
 * @param       params - pointer to permit join duration
 *
 * @return      NULL
 */
void *sendDevPermitJoinInd( void *params )
{
  uint8 duration = *((uint8 *) params);
  int len;
  uint8 *pBuf;
  DevPermitJoinInd devInd = DEV_PERMIT_JOIN_IND__INIT;

  uiPrintfEx(trINFO, "zstackpb Sending Dev Permit Join Ind: 0x%.2X\n", duration );

  devInd.cmdid = ZSTACK_CMD_IDS__DEV_PERMIT_JOIN_IND;
  devInd.duration = duration;

  len = dev_permit_join_ind__get_packed_size( &devInd );
  pBuf = malloc( len );
  if ( pBuf )
  {
    dev_permit_join_ind__pack( &devInd, pBuf );

    sendMsgToAllCBMsgs( len, pBuf, ZS_DEV_PERMIT_JOIN_IND_CDID,
        ZSTACK_CMD_IDS__DEV_PERMIT_JOIN_IND );

    free( pBuf );
  }

  return (NULL);
}

/*********************************************************************
 * @fn          sendNwkInfoRsp
 *
 * @brief       Send a SYS Netork Info Response
 *
 * @param       connection - tcp connection to send response
 *
 * @return      NULL
 */
static void sendNwkInfoRsp( int connection )
{
  uint8 *pIeeeAddr;
  int len;
  uint8 *pBuf;
  DeviceTypes devtypes = DEVICE_TYPES__INIT;
  SysNwkInfoReadRsp niRsp = SYS_NWK_INFO_READ_RSP__INIT;
  uint8 deviceType = 0;
#if defined ( LINUX_ZNP )
  nwkIB_t _NIB =  { 0};

  zdoExtNwkInfo( &_NIB );
  devState = _NIB.nwkState;

  deviceType = DEVICE_BUILD_COORDINATOR; // force coordinator only
#else
  deviceType = ZSTACK_DEVICE_BUILD;
#endif

  uiPrintfEx(trINFO, "zstackpb Sending Network Info Response\n" );
  niRsp.cmdid = ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_RSP;

  niRsp.nwkaddr = _NIB.nwkDevAddress;
  niRsp.devstate = devState;
  niRsp.panid = _NIB.nwkPanId;
  niRsp.coordaddr = _NIB.nwkCoordAddress;
  pIeeeAddr = NLME_GetExtAddr();
  memcpy( &niRsp.extendedpanid, _NIB.extendedPANID, 8 );
  memcpy( &niRsp.ieeeaddr, pIeeeAddr, 8 );
  memcpy( &niRsp.coordextaddr, _NIB.nwkCoordExtAddress, 8 );
  niRsp.logicalchannel = _NIB.nwkLogicalChannel;

  if ( deviceType & DEVICE_BUILD_COORDINATOR )
  {
    devtypes.coodinator = TRUE;
  }
  if ( deviceType & DEVICE_BUILD_ROUTER )
  {
    devtypes.router = TRUE;
  }
  if ( deviceType & DEVICE_BUILD_ENDDEVICE )
  {
    devtypes.enddevice = TRUE;
  }

  niRsp.devtypes = &devtypes;

  len = sys_nwk_info_read_rsp__get_packed_size( &niRsp );
  pBuf = malloc( len );
  if ( pBuf )
  {
    sys_nwk_info_read_rsp__pack( &niRsp, pBuf );

    // Send the AREQ message
    APIS_SendData( connection, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, FALSE,
        ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_RSP, len, pBuf );

    free( pBuf );
  }
}

/*********************************************************************
 * Endpoint Table Functions
 */

/*********************************************************************
 * @fn          epTableAddNewEntry
 *
 * @brief       Add entry to the list.  It will add to the end of the list.
 *
 * @param       newEntry - pointer to the entry
 *
 * @return      TRUE if added, FALSE if MAX reached
 */
static uint8 epTableAddNewEntry( epItem_t *newEntry )
{
  epItem_t *srch;

  if ( epTableNumEntries() > maxSupportedEndpoints )
  {
    return (FALSE);
  }

  srch = pEpTableHdr;

  while ( srch && srch->next )
  {
    srch = srch->next;
  }

  newEntry->next = (epItem_t *) NULL;

  if ( srch )
  {
    srch->next = newEntry;
  }
  else
  {
    pEpTableHdr = newEntry;
  }

  return (TRUE);
}

/*********************************************************************
 * @fn          epTableFindEntryEP
 *
 * @brief       Search list for endpoint and return pointer to record
 *
 * @param       ep - endpoint to find
 *
 * @return      pointer to entry record, NULL if not found
 */
static epItem_t *epTableFindEntryEP( uint8 ep )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( srch->epDesc.endPoint == ep )
    {
      return (srch);
    }

    srch = srch->next;
  }

  return (NULL);
}

/*********************************************************************
 * @fn          epTableFindEntryConnection
 *
 * @brief       Search list for connection and return pointer to record
 *
 * @param       connection - connection to find
 *
 * @return      pointer to entry record, NULL if not found
 */
static epItem_t *epTableFindEntryConnection( int connection )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( srch->connection == connection )
    {
      return (srch);
    }

    srch = srch->next;
  }

  return (NULL);
}

/*********************************************************************
 * @fn          zstackpbReestablishEPs
 *
 * @brief       Reregister all endpoints
 *
 * @param       none
 *
 * @return      none
 */
void zstackpbReestablishEPs( void )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( srch->epDesc.endPoint != 0 )
    {
      uint8 status = afRegister( &(srch->epDesc) );
      if ( status == afStatus_INVALID_PARAMETER )
      {
        afDelete( srch->epDesc.endPoint );
        afRegister( &(srch->epDesc) );
      }
    }

    srch = srch->next;
  }
}

/*********************************************************************
 * @fn          epTableRemoveEntry
 *
 * @brief       Remove an entry from the list and release its memory.
 *
 * @param       entry - pointer to entry
 *
 * @return      none
 */
static void epTableRemoveEntry( epItem_t *entry )
{
  epItem_t *srch;
  epItem_t *prev;

// Is this a real entry and is there anything in the list
  if ( entry && pEpTableHdr )
  {
    srch = pEpTableHdr;
    prev = (epItem_t *) NULL;

    while ( srch && srch != entry )
    {
      prev = srch;
      srch = srch->next;
    }

    // Found?
    if ( srch == entry )
    {
      // Update the list
      if ( entry == pEpTableHdr )
      {
        // First entry
        pEpTableHdr = entry->next;
      }
      else
      {
        prev->next = entry->next;
      }

      // Release the entry's memory
      freeEpItem( entry );
    }
  }
}

/*********************************************************************
 * @fn          epTableNumEntries
 *
 * @brief       Get the number of entries currently in the list.
 *
 * @param       none
 *
 * @return      number of entries
 */
static uint8 epTableNumEntries( void )
{
  epItem_t *srch;
  uint8 cnt = 0;

  srch = pEpTableHdr;

  while ( srch )
  {
    cnt++;
    srch = srch->next;
  }

  return (cnt);
}

/*********************************************************************
 * @fn          freeEpItem
 *
 * @brief       Free memory used in an endpoint entry.
 *
 * @param       pItem - pointer to the endpoint entry
 *
 * @return      none
 */
static void freeEpItem( epItem_t *pItem )
{
  if ( pItem )
  {
    if ( pItem->epDesc.simpleDesc )
    {
      if ( pItem->epDesc.simpleDesc->pAppInClusterList )
      {
        free( pItem->epDesc.simpleDesc->pAppInClusterList );
      }
      if ( pItem->epDesc.simpleDesc->pAppOutClusterList )
      {
        free( pItem->epDesc.simpleDesc->pAppOutClusterList );
      }
      free( pItem->epDesc.simpleDesc );
    }
    free( pItem );
  }
}

/***************************************************************************************************
 * @fn          buildPBcapInfo
 *
 * @brief       Convert from bitmask byte to PB capInfo
 *
 * @param       cInfo - source
 * @param       pPBcapInfo - destination
 *
 * @return      None
 ***************************************************************************************************/
static void buildPBcapInfo( uint8 cInfo, CapabilityInfo *pPBcapInfo )
{
  if ( cInfo & CAPABLE_PAN_COORD )
  {
    pPBcapInfo->pancoord = 1;
  }

  if ( cInfo & CAPABLE_FFD )
  {
    pPBcapInfo->ffd = 1;
  }

  if ( cInfo & CAPABLE_MAINS_POWER )
  {
    pPBcapInfo->mainspower = 1;
  }

  if ( cInfo & CAPABLE_RX_ON_IDLE )
  {
    pPBcapInfo->rxonwhenidle = 1;
  }

  if ( cInfo & CAPABLE_SECURITY )
  {
    pPBcapInfo->security = 1;
  }
}

/***************************************************************************************************
 * @fn          buildPBserverCap
 *
 * @brief       Convert from bitmask byte to PB ServerCapabilities
 *
 * @param       cInfo - source
 * @param       pPBsrvCap - destination
 *
 * @return      None
 ***************************************************************************************************/
static void buildPBserverCap( uint16 sInfo, ServerCapabilities *pPBsrvCap )
{
  if ( sInfo & PRIM_TRUST_CENTER )
  {
    pPBsrvCap->primarytrustcenter = 1;
  }

  if ( sInfo & BKUP_TRUST_CENTER )
  {
    pPBsrvCap->backuptrustcenter = 1;
  }

  if ( sInfo & PRIM_BIND_TABLE )
  {
    pPBsrvCap->primarybindingtablecache = 1;
  }

  if ( sInfo & BKUP_BIND_TABLE )
  {
    pPBsrvCap->backupbindingtablecache = 1;
  }

  if ( sInfo & PRIM_DISC_TABLE )
  {
    pPBsrvCap->primarydiscoverycache = 1;
  }

  if ( sInfo & BKUP_DISC_TABLE )
  {
    pPBsrvCap->backupdiscoverycache = 1;
  }

  if ( sInfo & NETWORK_MANAGER )
  {
    pPBsrvCap->networkmanager = 1;
  }
}

/***************************************************************************************************
 * @fn          buildPBpowerSource
 *
 * @brief       Convert from bitmask byte to PB PowerSource
 *
 * @param       cInfo - source
 * @param       pPBsrvCap - destination
 *
 * @return      None
 ***************************************************************************************************/
static void buildPBpowerSource( uint8 pInfo, PowerSource *pPBpwrSrc )
{
  if ( pInfo & NODEAVAILPWR_MAINS )
  {
    pPBpwrSrc->mains = 1;
  }

  if ( pInfo & NODEAVAILPWR_RECHARGE )
  {
    pPBpwrSrc->recharge = 1;
  }

  if ( pInfo & NODEAVAILPWR_DISPOSE )
  {
    pPBpwrSrc->dispose = 1;
  }
}

/***************************************************************************************************
 * @fn          convertPBTransOptions
 *
 * @brief       Convert PB TransOptions data type to uint8 txOptions
 *
 * @param       pOptions - TransOptions pointer
 *
 * @return      txOptions
 ***************************************************************************************************/
static uint8 convertPBTransOptions( TransOptions *pOptions )
{
  uint8 options = 0;

  if ( pOptions->has_wildcardprofileid && pOptions->wildcardprofileid )
  {
    options |= AF_WILDCARD_PROFILEID;
  }

  if ( pOptions->has_ackrequest && pOptions->ackrequest )
  {
    options |= AF_ACK_REQUEST;
  }

  if ( pOptions->has_limitconcentrator && pOptions->limitconcentrator )
  {
    options |= AF_LIMIT_CONCENTRATOR;
  }

  if ( pOptions->has_suppressroutedisc && pOptions->suppressroutedisc )
  {
    options |= AF_SUPRESS_ROUTE_DISC_NETWORK;
  }

  if ( pOptions->has_apssecurity && pOptions->apssecurity )
  {
    options |= AF_EN_SECURITY;
  }

  if ( pOptions->has_skiprouting && pOptions->skiprouting )
  {
    options |= AF_SKIP_ROUTING;
  }

  return (options);
}

/***************************************************************************************************
 * @fn          convertPBServerCapabilities
 *
 * @brief       Convert PB ServerCapabilities data type to uint8 txOptions
 *
 * @param       pOptions - TransOptions pointer
 *
 * @return      txOptions
 ***************************************************************************************************/
static uint16 convertPBServerCapabilities( ServerCapabilities *pSrvCap )
{
  uint16 mask = 0;

  if ( pSrvCap->primarytrustcenter )
  {
    mask |= PRIM_TRUST_CENTER;
  }

  if ( pSrvCap->backuptrustcenter )
  {
    mask |= BKUP_TRUST_CENTER;
  }

  if ( pSrvCap->primarybindingtablecache )
  {
    mask |= PRIM_BIND_TABLE;
  }

  if ( pSrvCap->backupbindingtablecache )
  {
    mask |= BKUP_BIND_TABLE;
  }

  if ( pSrvCap->primarydiscoverycache )
  {
    mask |= PRIM_DISC_TABLE;
  }

  if ( pSrvCap->backupdiscoverycache )
  {
    mask |= BKUP_DISC_TABLE;
  }

  if ( pSrvCap->networkmanager )
  {
    mask |= NETWORK_MANAGER;
  }

  return (mask);
}

/***************************************************************************************************
 * @fn          convertCapabilityInfo
 *
 * @brief       Convert PB CapabilityInfo data type to uint8 capInfo
 *
 * @param       pPBcapInfo - CapabilityInfo pointer
 *
 * @return      capInfo
 ***************************************************************************************************/
uint8 convertCapabilityInfo( CapabilityInfo *pPBcapInfo )
{
  uint8 capInfo = 0;

  if ( pPBcapInfo->pancoord )
  {
    capInfo |= CAPABLE_PAN_COORD;
  }

  if ( pPBcapInfo->ffd )
  {
    capInfo |= CAPABLE_FFD;
  }

  if ( pPBcapInfo->mainspower )
  {
    capInfo |= CAPABLE_MAINS_POWER;
  }

  if ( pPBcapInfo->rxonwhenidle )
  {
    capInfo |= CAPABLE_RX_ON_IDLE;
  }

  if ( pPBcapInfo->security )
  {
    capInfo |= CAPABLE_SECURITY;
  }

  return (capInfo);
}

/*********************************************************************
 */
