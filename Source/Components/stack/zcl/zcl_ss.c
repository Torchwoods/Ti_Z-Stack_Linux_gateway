/**************************************************************************************************
  Filename:       zcl_ss.c
  Revised:        $Date: 2014-01-20 14:17:28 -0800 (Mon, 20 Jan 2014) $
  Revision:       $Revision: 36892 $

  Description:    Zigbee Cluster Library - Security and Safety ( SS )


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

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

/*******************************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ss.h"

#if defined ( INTER_PAN )
  #include "stub_aps.h"
#endif

/*******************************************************************************
 * MACROS
 */
#define zclSS_ZoneTypeSupported( a ) ( (a) == SS_IAS_ZONE_TYPE_STANDARD_CIE              || \
                                       (a) == SS_IAS_ZONE_TYPE_MOTION_SENSOR             || \
                                       (a) == SS_IAS_ZONE_TYPE_CONTACT_SWITCH            || \
                                       (a) == SS_IAS_ZONE_TYPE_FIRE_SENSOR               || \
                                       (a) == SS_IAS_ZONE_TYPE_WATER_SENSOR              || \
                                       (a) == SS_IAS_ZONE_TYPE_GAS_SENSOR                || \
                                       (a) == SS_IAS_ZONE_TYPE_PERSONAL_EMERGENCY_DEVICE || \
                                       (a) == SS_IAS_ZONE_TYPE_VIBRATION_MOVEMENT_SENSOR || \
                                       (a) == SS_IAS_ZONE_TYPE_REMOTE_CONTROL            || \
                                       (a) == SS_IAS_ZONE_TYPE_KEY_FOB                   || \
                                       (a) == SS_IAS_ZONE_TYPE_KEYPAD                    || \
                                       (a) == SS_IAS_ZONE_TYPE_STANDARD_WARNING_DEVICE )

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */
typedef struct zclSSCBRec
{
  struct zclSSCBRec       *next;
  uint8                   endpoint; // Used to link it into the endpoint descriptor
  zclSS_AppCallbacks_t    *CBs;     // Pointer to Callback function
} zclSSCBRec_t;

typedef struct zclSS_ZoneItem
{
  struct zclSS_ZoneItem   *next;
  uint8                   endpoint; // Used to link it into the endpoint descriptor
  IAS_ACE_ZoneTable_t     zone;     // Zone info
} zclSS_ZoneItem_t;

/*******************************************************************************
 * GLOBAL VARIABLES
 */
const uint8 zclSS_UknownIeeeAddress[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/*******************************************************************************
 * GLOBAL FUNCTIONS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
static zclSSCBRec_t *zclSSCBs = (zclSSCBRec_t *)NULL;
static uint8 zclSSPluginRegisted = FALSE;

#if defined(ZCL_ZONE) || defined(ZCL_ACE)
static zclSS_ZoneItem_t *zclSS_ZoneTable = (zclSS_ZoneItem_t *)NULL;
#endif // ZCL_ZONE || ZCL_ACE

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static ZStatus_t zclSS_HdlIncoming( zclIncoming_t *pInHdlrMsg );
static ZStatus_t zclSS_HdlInSpecificCommands( zclIncoming_t *pInMsg );
static zclSS_AppCallbacks_t *zclSS_FindCallbacks( uint8 endpoint );

#ifdef ZCL_ZONE
static ZStatus_t zclSS_ProcessInZoneStatusCmdsServer( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInZoneStatusCmdsClient( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );

static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_ChangeNotification( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_EnrollRequest( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationModeResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitTestModeResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_EnrollResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationMode( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitTestMode( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
#endif // ZCL_ZONE

#ifdef ZCL_ACE
static ZStatus_t zclSS_ProcessInACECmdsServer( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInACECmdsClient( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );

static ZStatus_t zclSS_ProcessInCmd_ACE_Arm( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_Bypass( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_Emergency( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_Fire( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_Panic( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneIDMap( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneInformation( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_ZoneStatusChanged( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_PanelStatusChanged( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_ArmResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneIDMapResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneInformationResponse( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
#endif // ZCL_ACE

#ifdef ZCL_WD
static ZStatus_t zclSS_ProcessInWDCmds( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );

static ZStatus_t zclSS_ProcessInCmd_WD_StartWarning( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
static ZStatus_t zclSS_ProcessInCmd_WD_Squawk( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs );
#endif // ZCL_WD

#ifdef ZCL_ZONE
static uint8 zclSS_GetNextFreeZoneID( void );
static ZStatus_t zclSS_AddZone( uint8 endpoint, IAS_ACE_ZoneTable_t *zone );
static uint8 zclSS_CountAllZones( void );
static uint8 zclSS_ZoneIDAvailable( uint8 zoneID );
#endif // ZCL_ZONE

#if defined(ZCL_ZONE) || defined(ZCL_ACE)
static IAS_ACE_ZoneTable_t *zclSS_FindZone( uint8 endpoint, uint8 zoneID );
#endif // ZCL_ZONE || ZCL_ACE

/******************************************************************************
 * @fn      zclSS_RegisterCmdCallbacks
 *
 * @brief   Register an applications command callbacks
 *
 * @param   endpoint - application's endpoint
 * @param   callbacks - pointer to the callback record.
 *
 * @return  ZMemError if not able to allocate
 */
ZStatus_t zclSS_RegisterCmdCallbacks( uint8 endpoint, zclSS_AppCallbacks_t *callbacks )
{
  zclSSCBRec_t *pNewItem;
  zclSSCBRec_t *pLoop;

  // Register as a ZCL Plugin
  if ( !zclSSPluginRegisted )
  {
    zcl_registerPlugin( ZCL_CLUSTER_ID_SS_IAS_ZONE,
                        ZCL_CLUSTER_ID_SS_IAS_WD,
                        zclSS_HdlIncoming );
    zclSSPluginRegisted = TRUE;
  }

  // Fill in the new profile list
  pNewItem = zcl_mem_alloc( sizeof( zclSSCBRec_t ) );
  if ( pNewItem == NULL )
  {
    return ( ZMemError ); // memory error
  }

  pNewItem->next = (zclSSCBRec_t *)NULL;
  pNewItem->endpoint = endpoint;
  pNewItem->CBs = callbacks;

  // Find spot in list
  if ( zclSSCBs == NULL )
  {
    zclSSCBs = pNewItem;
  }
  else
  {
    // Look for end of list
    pLoop = zclSSCBs;
    while ( pLoop->next != NULL )
    {
      pLoop = pLoop->next;
    }

    // Put new item at end of list
    pLoop->next = pNewItem;
  }
  return ( ZSuccess );
}

#ifdef ZCL_ZONE
/*******************************************************************************
 * @fn      zclSS_Send_IAS_ZoneStatusChangeNotificationCmd
 *
 * @brief   Call to send out a Change Notification Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneStatus - current zone status - bit map
 * @param   extendedStatus - bit map, currently set to All zeros ( reserved)
 * @param   zoneID - allocated zone ID
 * @param   delay - delay from change in ZoneStatus attr to transmission of change notification cmd
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_IAS_Send_ZoneStatusChangeNotificationCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                          uint16 zoneStatus, uint8 extendedStatus,
                                                          uint8 zoneID, uint16 delay,
                                                          uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PAYLOAD_LEN_ZONE_STATUS_CHANGE_NOTIFICATION];

  buf[0] = LO_UINT16( zoneStatus );
  buf[1] = HI_UINT16( zoneStatus );
  buf[2] = extendedStatus;
  buf[3] = zoneID;
  buf[4] = LO_UINT16( delay );
  buf[5] = HI_UINT16( delay );

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ZONE,
                          COMMAND_SS_IAS_ZONE_STATUS_CHANGE_NOTIFICATION, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, PAYLOAD_LEN_ZONE_STATUS_CHANGE_NOTIFICATION, buf );
}

/*******************************************************************************
 * @fn      zclSS_Send_IAS_ZoneStatusEnrollRequestCmd
 *
 * @brief   Call to send out a Enroll Request Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneType - 	  current value of Zone Type attribute
 * @param   manufacturerCode - manuf. code from node descriptor for the device
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_IAS_Send_ZoneStatusEnrollRequestCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                     uint16 zoneType, uint16 manufacturerCode,
                                                     uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PAYLOAD_LEN_ZONE_ENROLL_REQUEST];

  buf[0] = LO_UINT16( zoneType );
  buf[1] = HI_UINT16( zoneType );
  buf[2] = LO_UINT16( manufacturerCode );
  buf[3] = HI_UINT16( manufacturerCode );

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ZONE,
                          COMMAND_SS_IAS_ZONE_STATUS_ENROLL_REQUEST, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, PAYLOAD_LEN_ZONE_ENROLL_REQUEST, buf );
}

/*******************************************************************************
 * @fn      zclSS_IAS_Send_ZoneStatusEnrollResponseCmd
 *
 * @brief   Call to send out a Enroll Response Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   responseCode -  value of  response Code
 * @param   zoneID  - index to the zone table of the CIE
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_IAS_Send_ZoneStatusEnrollResponseCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint8 responseCode, uint8 zoneID,
                                                      uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PAYLOAD_LEN_ZONE_STATUS_ENROLL_RSP];

  buf[0] = responseCode;
  buf[1] = zoneID;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ZONE,
                          COMMAND_SS_IAS_ZONE_STATUS_ENROLL_RESPONSE, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0,
                          seqNum, PAYLOAD_LEN_ZONE_STATUS_ENROLL_RSP, buf );
}

/*******************************************************************************
 * @fn      zclSS_IAS_Send_ZoneStatusInitTestModeCmd
 *
 * @brief   Call to send out a Initiate Test Mode Command
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   pCmd -  pointer to command structure
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_IAS_Send_ZoneStatusInitTestModeCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                    zclZoneInitTestMode_t *pCmd,
                                                    uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[PAYLOAD_LEN_ZONE_STATUS_INIT_TEST_MODE];

  buf[0] = pCmd->testModeDuration;
  buf[1] = pCmd->currZoneSensitivityLevel;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ZONE,
                          COMMAND_SS_IAS_ZONE_STATUS_INIT_TEST_MODE, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0,
                          seqNum, PAYLOAD_LEN_ZONE_STATUS_INIT_TEST_MODE, buf );
}

#endif // ZCL_ZONE

#ifdef ZCL_ACE
/*******************************************************************************
 * @fn      zclSS_Send_IAS_ACE_ArmCmd
 *
 * @brief   Call to send out a Arm  Command  ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   armMode -  value of armMode
 * @param   pArmDisarmCode - (64 bits) arm/disarm code for the system
 * @param   zoneID - index to the zone table of the CIE
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_ArmCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                     uint8 armMode, uint8 *pArmDisarmCode, uint8 zoneID,
                                     uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 i;
  uint8 *pBuf;
  uint8 *pOutBuf;
  uint8 len = sizeof(armMode) + ARM_DISARM_CODE_LEN + sizeof(zoneID);
  ZStatus_t stat;

  pBuf = zcl_mem_alloc( len );
  if ( pBuf )
  {
    pOutBuf = pBuf;

    *pOutBuf++ = armMode;

    for( i = 0; i < ARM_DISARM_CODE_LEN; i++ )
    {
      *pOutBuf++ = *pArmDisarmCode++;
    }

    *pOutBuf++ = zoneID;

    stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                           COMMAND_SS_IAS_ACE_ARM, TRUE,
                           ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, len, pBuf );

    zcl_mem_free( pBuf );
  }
  else
  {
    stat = ZMemError;   // memory error
  }

  return( stat );
}

/*********************************************************************
 * @fn      zclSS_Send_IAS_ACE_BypassCmd
 *
 * @brief   Call to send out a Bypass Command  ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   numberOfZones - one byte
 * @param   bypassBuf - zone IDs array of 256 entries one byte each
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_BypassCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                        uint8 numberOfZones, uint8 *bypassBuf,
                                        uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pBuf;
  uint8 len = 1 + numberOfZones;
  ZStatus_t stat;

  buf = zcl_mem_alloc( len );
  if ( buf )
  {
    pBuf = buf;

    *pBuf++ = numberOfZones;
    zcl_memcpy( pBuf, bypassBuf, numberOfZones );

    stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                            COMMAND_SS_IAS_ACE_BYPASS, TRUE,
                            ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, len, buf );
    zcl_mem_free( buf );
  }
  else
  {
    stat = ZMemError; // memory error
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSS_Send_IAS_ACE_GetZoneInformationCmd
 *
 * @brief   Call to send out a Get Zone Information Command ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneID - 8 bit value from 0 to 255
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_GetZoneInformationCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                    uint8 zoneID, uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[1];

  buf[0] = zoneID;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                          COMMAND_SS_IAS_ACE_GET_ZONE_INFORMATION, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 1, buf );
}

/*******************************************************************************
 * @fn      zclSS_Send_IAS_ACE_ArmResponse
 *
 * @brief   Call to send out a Arm Response Command ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   armNotification - notification parameter
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_ArmResponse( uint8 srcEP, afAddrType_t *dstAddr,
                                         uint8 armNotification, uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[1];

  buf[0] = armNotification;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                          COMMAND_SS_IAS_ACE_ARM_RESPONSE, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, 1, buf );
}

/*********************************************************************
 * @fn      zclSS_Send_IAS_ACE_GetZoneIDMapResponseCmd
 *
 * @brief   Call to send out a Get Zone ID Map Response Cmd  ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneIDMap - pointer to an array of 16 uint16
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_GetZoneIDMapResponseCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                      uint16 *zoneIDMap, uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 *pIndex;
  uint8 j,len = ( ZONE_ID_MAP_ARRAY_SIZE * sizeof( uint16 ) );
  ZStatus_t stat;

  buf = zcl_mem_alloc( len );

  if ( buf )
  {
    pIndex = buf;

    for( j = 0; j < ZONE_ID_MAP_ARRAY_SIZE; j++ )
    {
      *pIndex++  = LO_UINT16( *zoneIDMap   );
      *pIndex++  = HI_UINT16( *zoneIDMap++ );
    }

    stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                            COMMAND_SS_IAS_ACE_GET_ZONE_ID_MAP_RESPONSE, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, len, buf );
    zcl_mem_free( buf );
  }
  else
  {
    stat = ZMemError; // memory error
  }

  return ( stat );

}

/*******************************************************************************
 * @fn      zclSS_Send_IAS_ACE_GetZoneInformationResponseCmd
 *
 * @brief   Call to send out Get Zone Information Response Cmd (IAS ACE Cluster)
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneID - 8 bit value from 0 to 255
 * @param   zoneType - 16 bit
 * @param   ieeeAddress - pointer to 64 bit address ( 8bytes*8)
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_GetZoneInformationResponseCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                            uint8 zoneID, uint16 zoneType, uint8 *ieeeAddress,
                                                            uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 *buf;
  uint8 len = 11; // zoneID (1) + zoneType (2) + zoneAddress (8)
  ZStatus_t stat;

  buf = zcl_mem_alloc( len );

  if ( buf )
  {
    buf[0] = zoneID;
    buf[1] = LO_UINT16( zoneType);
    buf[2] = HI_UINT16( zoneType);
    zcl_cpyExtAddr( &buf[3], ieeeAddress );

    stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                            COMMAND_SS_IAS_ACE_GET_ZONE_INFORMATION_RESPONSE, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, len, buf );
    zcl_mem_free( buf );
  }
  else
  {
    stat = ZMemError; // memory error
  }

  return ( stat );
}

/*******************************************************************************
 * @fn      zclSS_Send_IAS_ACE_ZoneStatusChangedCmd
 *
 * @brief   Call to send out a Zone Status Changed Command  ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   zoneID - index to the zone table of the CIE
 * @param   zoneStatus - current value of the attribute
 * @param   pZoneLabel - the first 16 bytes of attribute programmed into the AES client
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_ZoneStatusChangedCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                   uint8 zoneID, uint16 zoneStatus, uint8 *pZoneLabel,
                                                   uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 i;
  uint8 *pBuf;
  uint8 *pOutBuf;
  uint8 len = sizeof(zoneID) + sizeof(zoneStatus) + 16;  // size of zoneLabel is 16 bytes
  ZStatus_t stat;

  pBuf = zcl_mem_alloc( len );
  if ( pBuf )
  {
    pOutBuf = pBuf;

    *pOutBuf++ = zoneID;

    *pOutBuf++ = LO_UINT16(zoneStatus);
    *pOutBuf++ = HI_UINT16(zoneStatus);

    for( i = 0; i < 16; i++ )
    {
      *pOutBuf++ = *pZoneLabel++;
    }

    stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                            COMMAND_SS_IAS_ACE_ZONE_STATUS_CHANGED, TRUE,
                            ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, len, pBuf );

    zcl_mem_free( pBuf );
  }
  else
  {
    stat = ZMemError; // memory error
  }

  return( stat );
}

/*******************************************************************************
 * @fn      zclSS_Send_IAS_ACE_PanelStatusChangedCmd
 *
 * @brief   Call to send out a Arm  Command  ( IAS ACE Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   panelStatus - current value of the attribute
 * @param   secondsRemaining - time left for server to be in state indicated by panelStatus
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_ACE_PanelStatusChangedCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                                    uint8 panelStatus, uint8 secondsRemaining,
                                                    uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[2];

  buf[0] = panelStatus;
  buf[1] = secondsRemaining;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_ACE,
                          COMMAND_SS_IAS_ACE_PANEL_STATUS_CHANGED, TRUE,
                          ZCL_FRAME_SERVER_CLIENT_DIR, disableDefaultRsp, 0, seqNum, sizeof(buf), buf );
}
#endif // ZCL_ACE

#ifdef ZCL_WD
/*******************************************************************************
 * @fn      zclSS_Send_IAS_WD_StartWarningCmd
 *
 * @brief   Call to send out a Start Warning  Command (IAS WD Cluster)
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   pWarning - 3 bytes  of type	zclCmdSSWDStartWarningPayload_t
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_WD_StartWarningCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                             zclCmdSSWDStartWarningPayload_t *pWarning,
                                             uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[5]; // [warningMode + strobe + sirenLevel] + warningDuration + strobeDutyCycle + strobeLevel
  ZStatus_t stat;

  buf[0] = pWarning->warningmessage.warningbyte;
  buf[1] = LO_UINT16( pWarning->warningDuration );
  buf[2] = HI_UINT16( pWarning->warningDuration );
  buf[3] = pWarning->strobeDutyCycle;
  buf[4] = pWarning->strobeLevel;

  stat = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_WD,
                          COMMAND_SS_IAS_WD_START_WARNING, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 5, buf );

  return( stat );
}

/******************************************************************************
 * @fn      zclSS_Send_IAS_WD_StartWarningCmd
 *
 * @brief   Call to send out a Squawk Command  ( IAS WD Cluster )
 *
 * @param   srcEP - Sending application's endpoint
 * @param   dstAddr - where you want the message to go
 * @param   squawk - one byte of type zclCmdSSWDSquawkPayload_t
 * @param   disableDefaultRsp - toggle for enabling/disabling default response
 * @param   seqNum - command sequence number
 *
 * @return  ZStatus_t
 */
ZStatus_t zclSS_Send_IAS_WD_SquawkCmd( uint8 srcEP, afAddrType_t *dstAddr,
                                       zclCmdSSWDSquawkPayload_t *squawk,
                                       uint8 disableDefaultRsp, uint8 seqNum )
{
  uint8 buf[1];
  buf[0] = squawk->squawkbyte;

  return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_SS_IAS_WD,
                          COMMAND_SS_IAS_WD_SQUAWK, TRUE,
                          ZCL_FRAME_CLIENT_SERVER_DIR, disableDefaultRsp, 0, seqNum, 1, buf);
}
#endif // ZCL_WD

/*********************************************************************
 * @fn      zclSS_FindCallbacks
 *
 * @brief   Find the callbacks for an endpoint
 *
 * @param   endpoint
 *
 * @return  pointer to the callbacks
 */
static zclSS_AppCallbacks_t *zclSS_FindCallbacks( uint8 endpoint )
{
  zclSSCBRec_t *pCBs;

  pCBs = zclSSCBs;
  while ( pCBs )
  {
    if ( pCBs->endpoint == endpoint )
    {
      return ( pCBs->CBs );
    }
    pCBs = pCBs->next;
  }
  return ( (zclSS_AppCallbacks_t *)NULL );
}

/*********************************************************************
 * @fn      zclSS_HdlIncoming
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library or Profile commands for attributes
 *          that aren't in the attribute list
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_HdlIncoming( zclIncoming_t *pInMsg )
{
  ZStatus_t stat = ZSuccess;

#if defined ( INTER_PAN )
  if ( StubAPS_InterPan( pInMsg->msg->srcAddr.panId, pInMsg->msg->srcAddr.endPoint ) )
  {
    return ( stat ); // Cluster not supported thru Inter-PAN
  }
#endif
  if ( zcl_ClusterCmd( pInMsg->hdr.fc.type ) )
  {
    // Is this a manufacturer specific command?
    if ( pInMsg->hdr.fc.manuSpecific == 0 )
    {
      stat = zclSS_HdlInSpecificCommands( pInMsg );
    }
    else
    {
      // We don't support any manufacturer specific command -- ignore it.
      stat = ZFailure;
    }
  }
  else
  {
    // Handle all the normal (Read, Write...) commands
    stat = ZFailure;
  }
  return ( stat );
}

/*********************************************************************
 * @fn      zclSS_HdlInSpecificCommands
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_HdlInSpecificCommands( zclIncoming_t *pInMsg )
{
  ZStatus_t stat;
  zclSS_AppCallbacks_t *pCBs;

  // make sure endpoint exists
  pCBs = (void*)zclSS_FindCallbacks( pInMsg->msg->endPoint );
  if ( pCBs == NULL )
  {
    return ( ZFailure );
  }

  switch ( pInMsg->msg->clusterId )
  {
#ifdef ZCL_ZONE
    case ZCL_CLUSTER_ID_SS_IAS_ZONE:
      if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
      {
        stat = zclSS_ProcessInZoneStatusCmdsServer( pInMsg, pCBs );
      }
      else
      {
        stat = zclSS_ProcessInZoneStatusCmdsClient( pInMsg, pCBs );
      }
      break;
#endif // ZCL_ZONE

#ifdef ZCL_ACE
    case ZCL_CLUSTER_ID_SS_IAS_ACE:
      if ( zcl_ServerCmd( pInMsg->hdr.fc.direction ) )
      {
        stat = zclSS_ProcessInACECmdsServer( pInMsg, pCBs );
      }
      else
      {
        stat = zclSS_ProcessInACECmdsClient( pInMsg, pCBs );
      }
      break;
#endif // ZCL_ACE

#ifdef ZCL_WD
    case ZCL_CLUSTER_ID_SS_IAS_WD:
      stat = zclSS_ProcessInWDCmds( pInMsg, pCBs );
      break;
#endif // ZCL_WD

    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}

#ifdef ZCL_ZONE
/*********************************************************************
 * @fn      zclSS_ProcessInZoneStatusCmdsServer
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInZoneStatusCmdsServer( zclIncoming_t *pInMsg,
                                                      zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat;

  switch ( pInMsg->hdr.commandID )
  {
    case COMMAND_SS_IAS_ZONE_STATUS_ENROLL_RESPONSE:
      stat = zclSS_ProcessInCmd_ZoneStatus_EnrollResponse( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ZONE_STATUS_INIT_NORMAL_OP_MODE:
      stat = zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationMode( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ZONE_STATUS_INIT_TEST_MODE:
      stat = zclSS_ProcessInCmd_ZoneStatus_InitTestMode( pInMsg, pCBs );
      break;

  default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSS_ProcessInZoneStatusCmdsClient
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInZoneStatusCmdsClient( zclIncoming_t *pInMsg,
                                                      zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat;

  switch ( pInMsg->hdr.commandID )
  {
    case COMMAND_SS_IAS_ZONE_STATUS_CHANGE_NOTIFICATION:
      stat = zclSS_ProcessInCmd_ZoneStatus_ChangeNotification( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ZONE_STATUS_ENROLL_REQUEST:
      stat = zclSS_ProcessInCmd_ZoneStatus_EnrollRequest( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ZONE_STATUS_INIT_NORMAL_OP_MODE_RSP:
      stat = zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationModeResponse( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ZONE_STATUS_INIT_TEST_MODE_RSP:
      stat = zclSS_ProcessInCmd_ZoneStatus_InitTestModeResponse( pInMsg, pCBs );
      break;

    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}
#endif // ZCL_ZONE

#ifdef ZCL_ACE
/*********************************************************************
 * @fn      zclSS_ProcessInACECmdsServer
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInACECmdsServer( zclIncoming_t *pInMsg,
                                               zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat;

  switch ( pInMsg->hdr.commandID )
  {
    case COMMAND_SS_IAS_ACE_ARM:
      stat = zclSS_ProcessInCmd_ACE_Arm( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_BYPASS:
      stat = zclSS_ProcessInCmd_ACE_Bypass( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_EMERGENCY:
      stat = zclSS_ProcessInCmd_ACE_Emergency( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_FIRE:
      stat = zclSS_ProcessInCmd_ACE_Fire( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_PANIC:
      stat = zclSS_ProcessInCmd_ACE_Panic( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_GET_ZONE_ID_MAP:
      stat = zclSS_ProcessInCmd_ACE_GetZoneIDMap( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_GET_ZONE_INFORMATION:
      stat = zclSS_ProcessInCmd_ACE_GetZoneInformation( pInMsg, pCBs );
      break;

    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSS_ProcessInACECmdsClient
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInACECmdsClient( zclIncoming_t *pInMsg,
                                               zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat;

  switch ( pInMsg->hdr.commandID )
  {
    case COMMAND_SS_IAS_ACE_ARM_RESPONSE:
      stat = zclSS_ProcessInCmd_ACE_ArmResponse( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_GET_ZONE_ID_MAP_RESPONSE:
      stat = zclSS_ProcessInCmd_ACE_GetZoneIDMapResponse( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_GET_ZONE_INFORMATION_RESPONSE:
      stat = zclSS_ProcessInCmd_ACE_GetZoneInformationResponse( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_ZONE_STATUS_CHANGED:
      stat = zclSS_ProcessInCmd_ACE_ZoneStatusChanged( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_ACE_PANEL_STATUS_CHANGED:
      stat = zclSS_ProcessInCmd_ACE_PanelStatusChanged( pInMsg, pCBs );
      break;

    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}
#endif // ZCL_ACE

#ifdef ZCL_ZONE
/*********************************************************************
 * @fn      zclSS_AddZone
 *
 * @brief   Add a zone for an endpoint
 *
 * @param   endpoint - endpoint of new zone
 * @param   zone - new zone item
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_AddZone( uint8 endpoint, IAS_ACE_ZoneTable_t *zone )
{
  zclSS_ZoneItem_t *pNewItem;
  zclSS_ZoneItem_t *pLoop;

  // Fill in the new profile list
  pNewItem = zcl_mem_alloc( sizeof( zclSS_ZoneItem_t ) );
  if ( pNewItem == NULL )
  {
    return ( ZMemError );   // memory error
  }

  // Fill in the plugin record.
  pNewItem->next = (zclSS_ZoneItem_t *)NULL;
  pNewItem->endpoint = endpoint;
  zcl_memcpy( (uint8*)&(pNewItem->zone), (uint8*)zone, sizeof ( IAS_ACE_ZoneTable_t ));

  // Find spot in list
  if (  zclSS_ZoneTable == NULL )
  {
    zclSS_ZoneTable = pNewItem;
  }
  else
  {
    // Look for end of list
    pLoop = zclSS_ZoneTable;
    while ( pLoop->next != NULL )
    {
      pLoop = pLoop->next;
    }

    // Put new item at end of list
    pLoop->next = pNewItem;
  }

  return ( ZSuccess );
}

/*********************************************************************
 * @fn      zclSS_CountAllZones
 *
 * @brief   Count the total number of zones
 *
 * @param   none
 *
 * @return  number of zones
 */
uint8 zclSS_CountAllZones( void )
{
  zclSS_ZoneItem_t *pLoop;
  uint8 cnt = 0;

  // Look for end of list
  pLoop = zclSS_ZoneTable;
  while ( pLoop )
  {
    cnt++;
    pLoop = pLoop->next;
  }
  return ( cnt );
}

/*********************************************************************
 * @fn      zclSS_GetNextFreeZoneID
 *
 * @brief   Get the next free zone ID
 *
 * @param   none
 *
 * @return  free zone ID (0-ZCL_SS_MAX_ZONE_ID) ,
 *          (ZCL_SS_MAX_ZONE_ID + 1) if none is found (0xFF)
 */
static uint8 zclSS_GetNextFreeZoneID( void )
{
  static uint8 nextAvailZoneID = 0;

  if ( zclSS_ZoneIDAvailable( nextAvailZoneID ) == FALSE )
  {
    uint8 zoneID = nextAvailZoneID;

    // Look for next available zone ID
    do
    {
      if ( ++zoneID > ZCL_SS_MAX_ZONE_ID )
      {
        zoneID = 0; // roll over
      }
    } while ( (zoneID != nextAvailZoneID) && (zclSS_ZoneIDAvailable( zoneID ) == FALSE) );

    // Did we found a free zone ID?
    if ( zoneID != nextAvailZoneID )
    {
      nextAvailZoneID = zoneID;
    }
    else
    {
      return ( ZCL_SS_MAX_ZONE_ID + 1 );
    }
  }

  return ( nextAvailZoneID );
}


/*********************************************************************
 * @fn      zclSS_ZoneIDAvailable
 *
 * @brief   Check to see whether zoneID is available for use
 *
 * @param   zoneID - ID to look for zone
 *
 * @return  TRUE if zoneID is available, FALSE otherwise
 */
static uint8 zclSS_ZoneIDAvailable( uint8 zoneID )
{
  zclSS_ZoneItem_t *pLoop;

  if ( zoneID < ZCL_SS_MAX_ZONE_ID )
  {
    pLoop = zclSS_ZoneTable;
    while ( pLoop )
    {
      if ( pLoop->zone.zoneID == zoneID  )
      {
        return ( FALSE );
      }
      pLoop = pLoop->next;
    }

    // Zone ID not in use
    return ( TRUE );
  }

  return ( FALSE );
}
#endif // ZCL_ZONE

#if defined(ZCL_ZONE) || defined(ZCL_ACE)
/*********************************************************************
 * @fn      zclSS_FindZone
 *
 * @brief   Find a zone with endpoint and ZoneID
 *
 * @param   endpoint -
 * @param   zoneID - ID to look for zone
 *
 * @return  a pointer to the zone information, NULL if not found
 */
static IAS_ACE_ZoneTable_t *zclSS_FindZone( uint8 endpoint, uint8 zoneID )
{
  zclSS_ZoneItem_t *pLoop;

  // Look for end of list
  pLoop = zclSS_ZoneTable;
  while ( pLoop )
  {
    if ( ( pLoop->endpoint == endpoint ) && ( pLoop->zone.zoneID == zoneID )  )
    {
      return ( &(pLoop->zone) );
    }
    pLoop = pLoop->next;
  }

  return ( (IAS_ACE_ZoneTable_t *)NULL );
}

/*********************************************************************
 * @fn      zclSS_RemoveZone
 *
 * @brief   Remove a zone with endpoint and zoneID
 *
 * @param   endpoint - endpoint of zone to be removed
 * @param   zoneID - ID to look for zone
 *
 * @return  TRUE if removed, FALSE if not found
 */
uint8 zclSS_RemoveZone( uint8 endpoint, uint8 zoneID )
{
  zclSS_ZoneItem_t *pLoop;
  zclSS_ZoneItem_t *pPrev;

  // Look for end of list
  pLoop = zclSS_ZoneTable;
  pPrev = NULL;
  while ( pLoop )
  {
    if ( pLoop->endpoint == endpoint && pLoop->zone.zoneID == zoneID )
    {
      if ( pPrev == NULL )
      {
        zclSS_ZoneTable = pLoop->next;
      }
      else
      {
        pPrev->next = pLoop->next;
      }

      // Free the memory
      zcl_mem_free( pLoop );

      return ( TRUE );
    }
    pPrev = pLoop;
    pLoop = pLoop->next;
  }

  return ( FALSE );
}

/*********************************************************************
 * @fn      zclSS_UpdateZoneAddress
 *
 * @brief   Update Zone Address for zoneID
 *
 * @param   endpoint - endpoint of zone
 * @param   zoneID - ID to look for zone
 * @param   ieeeAddr - Device IEEE Address
 *
 * @return  none
 */
void zclSS_UpdateZoneAddress( uint8 endpoint, uint8 zoneID, uint8 *ieeeAddr )
{
  IAS_ACE_ZoneTable_t *pZone;

  pZone = zclSS_FindZone( endpoint, zoneID );

  if ( pZone != NULL )
  {
    // Update the zone address
    zcl_cpyExtAddr( pZone->zoneAddress, ieeeAddr );
  }
}
#endif // ZCL_ZONE || ZCL_ACE

#ifdef ZCL_ZONE
/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_ChangeNotification
 *
 * @brief   Process in the received StatusChangeNotification Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_ChangeNotification( zclIncoming_t *pInMsg,
                                                                   zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnChangeNotification )
  {
    zclZoneChangeNotif_t cmd;

    cmd.zoneStatus = BUILD_UINT16( pInMsg->pData[0], pInMsg->pData[1] );
    cmd.extendedStatus = pInMsg->pData[2];
    cmd.zoneID = pInMsg->pData[3];
    cmd.delay = BUILD_UINT16( pInMsg->pData[4], pInMsg->pData[5] );

    return ( pCBs->pfnChangeNotification( &cmd, &(pInMsg->msg->srcAddr) ) );
  }

  return ( ZFailure );
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_EnrollRequest
 *
 * @brief   Process in the received StatusEnrollRequest Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_EnrollRequest( zclIncoming_t *pInMsg,
                                                              zclSS_AppCallbacks_t *pCBs )
{
  IAS_ACE_ZoneTable_t zone;
  ZStatus_t stat = ZFailure;
  uint16 zoneType;
  uint16 manuCode;
  uint8 responseCode;
  uint8 zoneID;

  zoneType = BUILD_UINT16( pInMsg->pData[0], pInMsg->pData[1] );
  manuCode = BUILD_UINT16( pInMsg->pData[2], pInMsg->pData[3] );

  if ( zclSS_ZoneTypeSupported( zoneType ) )
  {
    // Add zone to the table if space is available
    if ( ( zclSS_CountAllZones() < ZCL_SS_MAX_ZONES-1 ) &&
       ( ( zoneID = zclSS_GetNextFreeZoneID() ) <= ZCL_SS_MAX_ZONE_ID ) )
    {
      zone.zoneID = zoneID;
      zone.zoneType = zoneType;

      // The application will fill in the right IEEE Address later
      zcl_cpyExtAddr( zone.zoneAddress, (void *)zclSS_UknownIeeeAddress );

      if ( zclSS_AddZone( pInMsg->msg->endPoint, &zone ) == ZSuccess )
      {
        responseCode = ZSuccess;
      }
      else
      {
        // CIE does not permit new zones to enroll at this time
        responseCode = SS_IAS_ZONE_STATUS_ENROLL_RESPONSE_CODE_NO_ENROLL_PERMIT;
      }
    }
    else
    {
      // CIE reached its limit of number of enrolled zones
      responseCode = SS_IAS_ZONE_STATUS_ENROLL_RESPONSE_CODE_TOO_MANY_ZONES;
    }
  }
  else
  {
    // Zone type is not known to CIE and is not supported
    responseCode = SS_IAS_ZONE_STATUS_ENROLL_RESPONSE_CODE_NOT_SUPPORTED;
  }

  // Callback the application so it can fill in the Device IEEE Address
  if ( pCBs->pfnEnrollRequest )
  {
    zclZoneEnrollReq_t req;

    req.srcAddr = &(pInMsg->msg->srcAddr);
    req.zoneID = zoneID;
    req.zoneType = zoneType;
    req.manufacturerCode = manuCode;

    stat = pCBs->pfnEnrollRequest( &req, pInMsg->msg->endPoint );
  }

  if ( stat == ZSuccess )
  {
    // Send a response back
    stat = zclSS_IAS_Send_ZoneStatusEnrollResponseCmd( pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
                                                       responseCode, zoneID, true, pInMsg->hdr.transSeqNum );

    return ( ZCL_STATUS_CMD_HAS_RSP );
  }
  else
  {
    return ( stat );
  }
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationModeResponse
 *
 * @brief   Process in the received Initiate Normal Operation Mode Response Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationModeResponse( zclIncoming_t *pInMsg,
                                                                                zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnInitNormalOpModeResponse )
  {
    pCBs->pfnInitNormalOpModeResponse( pInMsg );

    return ( ZSuccess  );
  }

  return ( ZFailure );
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_InitTestModeResponse
 *
 * @brief   Process in the received Initiate Test Mode Response Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitTestModeResponse( zclIncoming_t *pInMsg,
                                                                     zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnInitTestModeResponse )
  {
    pCBs->pfnInitTestModeResponse( pInMsg );

    return ( ZSuccess  );
  }

  return ( ZFailure );
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_EnrollResponse
 *
 * @brief   Process in the received Zone Enroll Response Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_EnrollResponse( zclIncoming_t *pInMsg,
                                                               zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnEnrollResponse )
  {
    zclZoneEnrollRsp_t rsp;

    rsp.responseCode = pInMsg->pData[0];
    rsp.zoneID = pInMsg->pData[1];

    return ( pCBs->pfnEnrollResponse( &rsp ) );
  }

  return ( ZFailure );
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationMode
 *
 * @brief   Process in the received Initiate Normal Operation Mode Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitNormalOperationMode( zclIncoming_t *pInMsg,
                                                                        zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnInitNormalOpMode )
  {
    pCBs->pfnInitNormalOpMode( pInMsg );

    return ( ZCL_STATUS_CMD_HAS_RSP  );
  }

  return ( ZFailure );
}

/*******************************************************************************
 * @fn      zclSS_ProcessInCmd_ZoneStatus_InitTestMode
 *
 * @brief   Process in the received Initiate Test Mode Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ZoneStatus_InitTestMode( zclIncoming_t *pInMsg,
                                                             zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnInitTestMode )
  {
    zclZoneInitTestMode_t cmd;

    cmd.testModeDuration =  pInMsg->pData[0];
    cmd.currZoneSensitivityLevel = pInMsg->pData[1];

    pCBs->pfnInitTestMode( &cmd, pInMsg );

    return ( ZCL_STATUS_CMD_HAS_RSP  );
  }

  return ( ZFailure );
}
#endif // ZCL_ZONE

#ifdef ZCL_ACE
/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_Arm
 *
 * @brief   Process in the received Arm Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_Arm( zclIncoming_t *pInMsg,
                                             zclSS_AppCallbacks_t *pCBs )
{
  uint8 i;
  uint8 codeLen = 8;  // intended size for UTF-8 String
  uint8 armNotification = 0xFF;
  ZStatus_t stat = ZFailure;
  zclCmdSSIASACEArmPayload_t cmd;

  if ( pCBs->pfnACE_Arm )
  {
    cmd.pArmDisarmCode = zcl_mem_alloc( codeLen );

    if ( cmd.pArmDisarmCode )
    {
      cmd.armMode =  pInMsg->pData[0];

      for ( i = 0; i < 8; i++ )
      {
        cmd.pArmDisarmCode[i] = pInMsg->pData[1 + i];
      }

      cmd.zoneID =  pInMsg->pData[10];

      if ( pCBs->pfnACE_Arm )
      {
        armNotification = pCBs->pfnACE_Arm( &cmd );

        if ( armNotification != 0xFF )
        {
          stat = ZSuccess;
        }
        else
        {
          stat = ZCL_STATUS_INVALID_VALUE;
        }
      }

      zcl_mem_free( cmd.pArmDisarmCode );
    }
    else
    {
      stat = ZMemError;
    }
  }

  if ( stat == ZSuccess  )
  {
    // Send a response back
    zclSS_Send_IAS_ACE_ArmResponse( pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
                                    armNotification, true, pInMsg->hdr.transSeqNum );

    return ( ZCL_STATUS_CMD_HAS_RSP );
  }
  else
  {
    return ( stat );
  }
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_Bypass
 *
 * @brief   Process in the received Bypass Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_Bypass( zclIncoming_t *pInMsg,
                                                zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_Bypass )
  {
    zclACEBypass_t cmd;

    cmd.numberOfZones = pInMsg->pData[0];
    cmd.bypassBuf = &(pInMsg->pData[1]);

    return ( pCBs->pfnACE_Bypass( &cmd ) );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_Emergency
 *
 * @brief   Process in the received Emergency Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_Emergency( zclIncoming_t *pInMsg,
                                                   zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_Emergency )
  {
    return ( pCBs->pfnACE_Emergency() );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_Fire
 *
 * @brief   Process in the received Fire Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_Fire( zclIncoming_t *pInMsg,
                                              zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_Fire )
  {
    return ( pCBs->pfnACE_Fire() );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_Panic
 *
 * @brief   Process in the received Panic Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_Panic( zclIncoming_t *pInMsg,
                                               zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_Panic )
  {
    return ( pCBs->pfnACE_Panic() );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_GetZoneIDMap
 *
 * @brief   Process in the received GetZoneIDMap Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneIDMap( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZFailure;
  uint16 zoneIDMap[16];
  uint16 mapSection;
  uint8 zoneID;
  uint8 i, j;

  for ( i = 0; i < 16; i++ )
  {
    mapSection = 0;

    // Find out Zone IDs that are allocated for this map section
    for ( j = 0; j < 16; j++ )
    {
      zoneID = 16 * i + j;
      if ( zclSS_FindZone( pInMsg->msg->endPoint, zoneID ) != NULL )
      {
        // Set the corresponding bit
        mapSection |= ( 0x01 << j );
      }
    }
    zoneIDMap[i] = mapSection;
  }

  if ( pCBs->pfnACE_GetZoneIDMap )
  {
    stat = pCBs->pfnACE_GetZoneIDMap( );
  }

  if ( stat == ZSuccess )
  {
    // Send a response back
    zclSS_Send_IAS_ACE_GetZoneIDMapResponseCmd( pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
                                                zoneIDMap, true, pInMsg->hdr.transSeqNum );

    return ( ZCL_STATUS_CMD_HAS_RSP );
  }
  else
  {
    return ( stat );
  }
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_GetZoneInformation
 *
 * @brief   Process in the received GetZoneInformation Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneInformation( zclIncoming_t *pInMsg, zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat = ZFailure;
  IAS_ACE_ZoneTable_t zone;
  IAS_ACE_ZoneTable_t *pZone;

  pZone = zclSS_FindZone( pInMsg->msg->endPoint, pInMsg->pData[0] );
  if ( pZone == NULL )
  {
    // Zone not found
    pZone = &zone;
    pZone->zoneID = pInMsg->pData[0];
    pZone->zoneType = SS_IAS_ZONE_TYPE_INVALID_ZONE_TYPE;
    zcl_cpyExtAddr( pZone->zoneAddress, (void *)zclSS_UknownIeeeAddress );
  }

  if ( pCBs->pfnACE_GetZoneInformation )
  {
    stat = pCBs->pfnACE_GetZoneInformation( pZone->zoneID );
  }

  if ( stat == ZSuccess )
  {
    // Send a response back
    zclSS_Send_IAS_ACE_GetZoneInformationResponseCmd( pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
                                                      pZone->zoneID, pZone->zoneType,
                                                      pZone->zoneAddress, true, pInMsg->hdr.transSeqNum );

    return ( ZCL_STATUS_CMD_HAS_RSP );
  }
  else
  {
    return ( stat );
  }
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_ArmResponse
 *
 * @brief   Process in the received Arm Response Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_ArmResponse( zclIncoming_t *pInMsg,
                                                     zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_ArmResponse )
  {
    return ( pCBs->pfnACE_ArmResponse(pInMsg->pData[0]) );
  }

  return ( ZFailure );
}


/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_GetZoneIDMapResponse
 *
 * @brief   Process in the received GetZoneIDMapResponse Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneIDMapResponse( zclIncoming_t *pInMsg,
                                                              zclSS_AppCallbacks_t *pCBs )
{
  uint16 *buf;
  uint16 *pIndex;
  uint8 *pData;
  uint8 i, len = 32; // 16 fields of 2 octets

  buf = zcl_mem_alloc( len );

  if ( buf )
  {
    pIndex = buf;
    pData = pInMsg->pData;

    for ( i = 0; i < 16; i++ )
    {
      *pIndex++ = BUILD_UINT16( pData[0], pData[1] );
      pData += 2;
    }

    zcl_mem_free( buf );

    if ( pCBs->pfnACE_GetZoneIDMapResponse )
    {
      return ( pCBs->pfnACE_GetZoneIDMapResponse( buf ) );
    }

   return ( ZFailure );
  }
  else
  {
    return ( ZMemError );   // Memory failure
  }
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_GetZoneInformationResponse
 *
 * @brief   Process in the received GetZoneInformationResponse Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_GetZoneInformationResponse( zclIncoming_t *pInMsg,
                                                                    zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnACE_GetZoneInformationResponse )
  {
    zclACEGetZoneInfoRsp_t rsp;

    rsp.zoneID = pInMsg->pData[0];
    rsp.zoneType = BUILD_UINT16( pInMsg->pData[1], pInMsg->pData[2] );
    rsp.ieeeAddr = &(pInMsg->pData[3]);

    return( pCBs->pfnACE_GetZoneInformationResponse( &rsp ) );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_ZoneStatusChanged
 *
 * @brief   Process in the received Zone Status Changed Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_ZoneStatusChanged( zclIncoming_t *pInMsg,
                                                           zclSS_AppCallbacks_t *pCBs )
{
  uint8 i;
  uint8 arraySize = 16; // size of 16 byte array (UTF-8 String)
  zclCmdSSIASACEZoneStatusChangedPayload_t cmd;

  if ( pCBs->pfnACE_ZoneStatusChanged )
  {
    cmd.pZoneLabel = zcl_mem_alloc( arraySize );

    if( cmd.pZoneLabel )
    {
      cmd.zoneID =  pInMsg->pData[0];
      cmd.zoneStatus =  BUILD_UINT16( pInMsg->pData[1], pInMsg->pData[2] );


      for( i = 0; i < arraySize; i++ )
      {
        cmd.pZoneLabel[i] = pInMsg->pData[3 + i];
      }

      zcl_mem_free( cmd.pZoneLabel );

      return ( pCBs->pfnACE_ZoneStatusChanged( &cmd ) );
    }
    else
    {
      return ( ZMemError );   // memory error
    }
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_ACE_PanelStatusChanged
 *
 * @brief   Process in the received Arm Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_ACE_PanelStatusChanged( zclIncoming_t *pInMsg,
                                                           zclSS_AppCallbacks_t *pCBs )
{
  zclCmdSSIASACEPanelStatusChangedPayload_t cmd;

  if ( pCBs->pfnACE_PanelStatusChanged )
  {
      cmd.panelStatus =  pInMsg->pData[0];
      cmd.secondsRemaining =  pInMsg->pData[1];

      return ( pCBs->pfnACE_PanelStatusChanged( &cmd ) );
  }

  return ( ZFailure );
}
#endif // ZCL_ACE

#ifdef ZCL_WD
/*********************************************************************
 * @fn      zclSS_ProcessInWDCmds
 *
 * @brief   Callback from ZCL to process incoming Commands specific
 *          to this cluster library on a command ID basis
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclSS_ProcessInWDCmds( zclIncoming_t *pInMsg,
                                        zclSS_AppCallbacks_t *pCBs )
{
  ZStatus_t stat;

  switch ( pInMsg->hdr.commandID )
  {
    case COMMAND_SS_IAS_WD_START_WARNING:
      stat = zclSS_ProcessInCmd_WD_StartWarning( pInMsg, pCBs );
      break;

    case COMMAND_SS_IAS_WD_SQUAWK:
      stat = zclSS_ProcessInCmd_WD_Squawk( pInMsg, pCBs );
      break;

    default:
      stat = ZFailure;
      break;
  }

  return ( stat );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_WD_StartWarning
 *
 * @brief   Process in the received StartWarning Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_WD_StartWarning( zclIncoming_t *pInMsg,
                                                     zclSS_AppCallbacks_t *pCBs )
{
  if ( pCBs->pfnWD_StartWarning )
  {
    zclWDStartWarning_t cmd;

    cmd.warnings.warningbyte = pInMsg->pData[0];
    cmd.duration = BUILD_UINT16( pInMsg->pData[1], pInMsg->pData[2] );

    return ( pCBs->pfnWD_StartWarning( &cmd ) );
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn      zclSS_ProcessInCmd_WD_Squawk
 *
 * @brief   Process in the received Squawk Command.
 *
 * @param   pInMsg - pointer to the incoming message
 * @param   pCBs - pointer to callback functions
 *
 * @return   ZStatus_t
 */
static ZStatus_t zclSS_ProcessInCmd_WD_Squawk( zclIncoming_t *pInMsg,
                                              zclSS_AppCallbacks_t *pCBs )
{
  zclCmdSSWDSquawkPayload_t cmd;

  if ( pCBs->pfnWD_Squawk )
  {
    cmd.squawkbyte = pInMsg->pData[0];

    return ( pCBs->pfnWD_Squawk( &cmd ) );
  }

  return ( ZFailure );
}
#endif // ZCL_WD

/*******************************************************************************
*******************************************************************************/

