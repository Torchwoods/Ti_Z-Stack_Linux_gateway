/**************************************************************************************************
  Filename:       gatewayservices.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    Handles services required by the gateway server.


  Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zstack.pb-c.h"
#include "gateway.pb-c.h"
#include "gatewaysrvr.h"
#include "gatewayservices.h"
#include "serverep.h"
#include "zcl.h"
#include "zcl_poll_control.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define POLLCONTROL_STATE_NONE      0 // state machine not started
#define POLLCONTROL_STATE_READY     1 // Success/complete

/*********************************************************************
 * TYPEDEFS
 */
typedef enum gwServices_State_tag
{
  gwServices_State_Available_c,   // must be first
  gwServices_State_GettingAttrListRsp_c,
  gwServices_State_Finish_c
} gwServices_State_t;

// Add device attribute list state machine
typedef struct gwServices_StateMachine_AttrList_tag
{
  gwServices_State_t        state;
  uint16                    appTransId;
  uint64_t                  ieeeAddr;       // IEEE address of the remote device
  uint8                     endpointId;     // endpoint ID of the attributes
  int                       connection;     // tcp connection
  sGwServices_AttrList_t   *pHead;
  sGwServices_AttrList_t   *pAttrList;      // attribute List record
  uint8                     clusterCount;   // number of server-side clusters
  uint16                   *pServerClusterList;   // list of server-side clusters
  uint8                     clusterIndex;   // current cluster index
  uint16                    startAttr;
} gwServices_StateMachine_AttrList_t;

typedef struct gwServices_PollControl_PacketPending_tag
{
  uint64_t          ieeeAddr;
  bool      packetPending;
  uint8_t   state;
  void             *pNext;
} gwServices_PollControl_PacketPending_t;

typedef struct gwServices_PacketPendingQueue_tag
{
  gwServices_PollControl_PacketPending_t *pHead;
  gwServices_PollControl_PacketPending_t *pQueueList;
} gwServices_PacketPendingQueue_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
int giGwServices_AttrListTableCount = 0;
int giGwServices_RetryTableCount = 0;
int giGwServices_TransTableCount = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
gwServices_PacketPendingQueue_t gGwService_PacketPendingQueue = { NULL, NULL };

gwServices_StateMachine_AttrList_t *gpGwServices_StateMachine_AttrList = NULL;

gsGwMsgRetryTable_t *gpGwServices_RetryTable = NULL;

gsGwMsgTransTable_t *gpGwServices_TransTable = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
bool gwServices_SetPacketQueueEntry( uint64_t ieeeAddr );
bool gwServices_FindPacketQueueEntry( uint64_t ieeeAddr );
bool gwServices_FindPacketPendingDeviceEntry( uint64_t ieeeAddr );
static gwServices_PollControl_PacketPending_t * gwServices_GetPacketQueueEntry( uint64_t ieeeAddr );
bool gwServices_PollControlStart( afAddrType_t *pSrcAddr, uint64_t ieeeAddr );
bool gwServices_AddPacketPendingDevice( uint64_t ieeeAddr );
static gwServices_PollControl_PacketPending_t * gwServices_AddPacketQueueEntry( void );
void gwServices_ClearPacketQueueEntry( uint64_t ieeeAddr );
uint8 gwServices_StartStateMachine_DeviceAttrList( int connection, SrvrDeviceInfoT *pDeviceInfo, 
                                                   GwGetDeviceAttributeListReq *pGetAttrListReq );
uint8 gwServices_UpdateStateMachine_DeviceAttrList( GwGetDeviceAttributeListRspInd *pGetAttrListRsp, 
                                                    uint16 appTransId, uint16 inClusterId, 
                                                    zclDiscoverAttrsRspCmd_t *pDiscAttrsRsp );                                    
static uint8 gwServices_AddAttrList( gwServices_StateMachine_AttrList_t *pState, uint16 clusterId, 
                                     zclDiscoverAttrsRspCmd_t *pDiscAttrRsp );
static gwServices_StateMachine_AttrList_t * gwServices_FindStateMachine( uint64_t ieeeAddr );
static gwServices_StateMachine_AttrList_t * gwServices_GetStateMachine( void );
static bool gwServices_FindClustersOnEndpoint( SrvrDeviceInfoT *pDeviceInfo, 
                                               gwServices_StateMachine_AttrList_t *pState );
static void gwServices_FinishStateMachine( gwServices_StateMachine_AttrList_t *pState );
static void gwServices_FreeStateMachine( gwServices_StateMachine_AttrList_t *pState );
static void gwServices_FreeAttrList( sGwServices_AttrList_t *pAttrListRecord );
static GwClusterListT * gwServices_ConvertDiscAttrsRspPb( uint16 clusterId, zclDiscoverAttrsRspCmd_t *pDiscAttrRsp, uint16 startAttr, GwClusterListT  *pClusterList );
void gwServices_StateMachineTimeout_DeviceAttrList( uint64_t ieeeAddr );
GwClusterListT ** gwServices_ConvertPbClusterList( int attrCount, zclAttrRec_t *pAttrRecord,
                                                   uint8 inClusterCount, uint32 *pInClusterList,
                                                   uint32 *pClusterListCount );
void gwServices_FreePbClusterList( int clusterListCount, GwClusterListT **ppClusterList );
                                               
gwServices_StateMachine_AttrList_t * gwServices_UpdateAttrListTable( void );
gwServices_StateMachine_AttrList_t * gwServices_GetAttrListEntry( void );
static uint8 gwServices_AddAttrListTable( void );
static uint8 gwServices_CheckAttrListTableSize( void ); 
static void gwServices_FreeAttrListTable( gwServices_StateMachine_AttrList_t *pTableEntry ); 
gsGwMsgRetryTable_t * gwServices_UpdateRetryTable( void );     
static gsGwMsgRetryTable_t * gwServices_GetRetryTableEntry( void );  
gsGwMsgRetryTable_t * gwServices_GetDeviceInRetryTable( uint64_t ieeeAddr ); 
static uint8 gwServices_AddRetryTable( void );  
static uint8 gwServices_CheckRetryTableSize( void );  
static void gwServices_FreeRetryTable( gsGwMsgRetryTable_t *pAttrList );
gsGwMsgTransTable_t * gwServices_UpdateTransTable( void );
static gsGwMsgTransTable_t * gwServices_GetTransTableEntry( void ); 
static uint8 gwServices_AddTransTable( void ); 
static uint8 gwServices_CheckTransTableSize( void );    
static void gwServices_FreeTransTable( gsGwMsgTransTable_t *pTransTable );              

/**************************************************************************************************
 **************************************************************************************************/

/*********************************************************************
 * @fn      gwServices_SetPacketQueueEntry
 *
 * @brief   Finds a device in queue and sets packet pending bit
 *
 * @param   ieeeAddr - device's IEEE address
 *
 * @return  TRUE if found, FALSE if not found
 */
bool gwServices_SetPacketQueueEntry( uint64_t ieeeAddr )
{  
  // Look for device
  if ( !gwServices_FindPacketQueueEntry( ieeeAddr ) )
  {
    // Add device to list
    gwServices_AddPacketPendingDevice( ieeeAddr );
  }
  
  if ( gGwService_PacketPendingQueue.pHead )
  {
    // Start at head of list
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;
    
    while ( gGwService_PacketPendingQueue.pQueueList )
{
      if ( gGwService_PacketPendingQueue.pQueueList->ieeeAddr == ieeeAddr )
  {
        // set packet pending bit
        gGwService_PacketPendingQueue.pQueueList->packetPending = TRUE;
        
    return TRUE;
  }
  
      gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
    }
  }
  
  return FALSE; // failure
}

/*********************************************************************
 * @fn      gwServices_FindPacketQueueEntry
 *
 * @brief   Finds a device in queue
 *
 * @param   ieeeAddr - device's IEEE address
 *
 * @return  TRUE if found, FALSE if not found
 */
bool gwServices_FindPacketQueueEntry( uint64_t ieeeAddr )
{  
  if ( gGwService_PacketPendingQueue.pHead )
  {
    // Start at head of list
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;
    
    while ( gGwService_PacketPendingQueue.pQueueList )
    {
      if ( gGwService_PacketPendingQueue.pQueueList->ieeeAddr == ieeeAddr )
      {
        return TRUE;
      }
      
      gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
    }
  }
  
  return FALSE;
}

/*********************************************************************
 * @fn      gwServices_FindPacketPendingDeviceEntry
 *
 * @brief   Finds a device in queue with packet pending
 *
 * @param   ieeeAddr - device's IEEE address
 *
 * @return  TRUE if found, FALSE if not found
 */
bool gwServices_FindPacketPendingDeviceEntry( uint64_t ieeeAddr )
{  
  if ( gGwService_PacketPendingQueue.pHead )
  {
    // Start at head of list
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;
    
    while ( gGwService_PacketPendingQueue.pQueueList )
    {
      if ( (gGwService_PacketPendingQueue.pQueueList->ieeeAddr == ieeeAddr) &&
           (gGwService_PacketPendingQueue.pQueueList->packetPending == TRUE) )
      {        
        return TRUE;
      }
      
      gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
    }
  }
  
  return FALSE;
}

/*********************************************************************
 * @fn      gwServices_GetPacketQueueEntry
 *
 * @brief   Gets a device in packet pending queue
 *
 * @param   ieeeAddr - device's IEEE address
 *
 * @return  Pointer to device information, NULL if failure
 */
static gwServices_PollControl_PacketPending_t * gwServices_GetPacketQueueEntry( uint64_t ieeeAddr )
{  
  if ( gGwService_PacketPendingQueue.pHead )
  {
    // Start at head of list
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;
    
    while ( gGwService_PacketPendingQueue.pQueueList )
{
      if ( gGwService_PacketPendingQueue.pQueueList->ieeeAddr == ieeeAddr )
      {
        return (gGwService_PacketPendingQueue.pQueueList);
      }
      
      gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
    }
  }
  
  return NULL;  // no entry found
}

/*********************************************************************
 * @fn      gwServices_PollControlStart
 *
 * @brief   Handles the configuring of a Poll Control Server.
 *
 * @param   pSrcAddr - device's source address information
 * @param   ieeeAddr - device's IEEE address information
 *
 * @return  TRUE device is ready, FALSE if device is not ready or failure
 */
bool gwServices_PollControlStart( afAddrType_t *pSrcAddr, uint64_t ieeeAddr )
{  
  ZStatus_t status;
  gwServices_PollControl_PacketPending_t *pEntry;
  
  // Get device information
  pEntry = gwServices_GetPacketQueueEntry( ieeeAddr );
  
  if ( !pEntry )
  {
    return FALSE;
  }
  
  if ( pEntry->state == POLLCONTROL_STATE_READY )
  {
    // nothing to do, Poll Control Server is ready
    return TRUE;
  }
  
  // Send check-in response for fast polling mode
  status = zclPollControl_Send_CheckInRsp( GW_EP, pSrcAddr,
                                           TRUE, FAST_POLL_TIMEOUT,
                                           TRUE, zcl_TransID );
  
  if ( status == ZSuccess )
  {
    // Set Long Poll Interval, assume success
    zclPollControl_Send_SetLongPollInterval( GW_EP, pSrcAddr,
                                             giLongPollInterval,
                                             FALSE, zcl_TransID );
                                                              
    // Set Short Poll Interval, assume success
    zclPollControl_Send_SetShortPollInterval( GW_EP, pSrcAddr,
                                              giShortPollInterval,
                                              FALSE, zcl_TransID );

    pEntry->state = POLLCONTROL_STATE_READY;    
    
    return TRUE;                                        
  }
  
  return FALSE; // failed to send check-in rsp
}

/*********************************************************************
 * @fn      gwServices_AddPacketPendingDevice
 *
 * @brief   Adds entry in packet pending que for Sleepy Device
 *          Packet Pending Request
 *
 * @param   ieeeAddr - IEEE address of sleepy device
 *
 * @return  TRUE for success or FALSE for memory failure
 */
bool gwServices_AddPacketPendingDevice( uint64_t ieeeAddr )
{
  int count = 1;
  gwServices_PollControl_PacketPending_t *pNewEntry;
  
  // Check to see if first entry
  if ( !gGwService_PacketPendingQueue.pHead )
  {
    pNewEntry = gwServices_AddPacketQueueEntry();
    
    gGwService_PacketPendingQueue.pHead = pNewEntry;
  }
  else
  {
    // Start at head of list
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;
    
    pNewEntry = gwServices_AddPacketQueueEntry();
    
    // Find end of list
    while ( gGwService_PacketPendingQueue.pQueueList )
    {
      count++; // add to entry count
       
      if ( !(gGwService_PacketPendingQueue.pQueueList->pNext) )
      {
        gGwService_PacketPendingQueue.pQueueList->pNext = pNewEntry; // link in new entry
        
        break;
      }
      
      gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
    }
  }
  
  if ( !pNewEntry )
  {
    return FALSE;  // memory error
  }
  
  uiPrintfEx(trDEBUG, "Packet Pending Device Queue Count: %d\n", count );
  
  pNewEntry->ieeeAddr = ieeeAddr; // store device's IEEE address
  pNewEntry->state = POLLCONTROL_STATE_NONE;
  pNewEntry->packetPending = FALSE;
  
  return TRUE;  // success
}

/*********************************************************************
 * @fn      gwServices_AddPacketQueueEntry
 *
 * @brief   Allocates new packet queue entry
 *
 * @param   none
 *
 * @return  pointer to new entry, NULL for memory error
 */
static gwServices_PollControl_PacketPending_t * gwServices_AddPacketQueueEntry( void )
{
  gwServices_PollControl_PacketPending_t *pNewEntry;
  
  pNewEntry = malloc( sizeof( gwServices_PollControl_PacketPending_t ) );
  if ( !pNewEntry )
  {
    return NULL;
  }
  
  pNewEntry->packetPending = FALSE;
  pNewEntry->state = POLLCONTROL_STATE_NONE;
  pNewEntry->pNext = NULL;  // mark as last entry
  
  return pNewEntry;
}

/*********************************************************************
 * @fn      gwServices_ClearPacketQueueEntry
 *
 * @brief   Remove packet pending queue list
 *
 * @param   ieeeAddr - device's IEEE address
 *
 * @return  none
 */
void gwServices_ClearPacketQueueEntry( uint64_t ieeeAddr )
{
  // Start at head of list
  gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pHead;

  while ( gGwService_PacketPendingQueue.pQueueList )
  {
    if ( gGwService_PacketPendingQueue.pQueueList->ieeeAddr == ieeeAddr )
    {
      gGwService_PacketPendingQueue.pQueueList->packetPending = FALSE;
      }
      
    gGwService_PacketPendingQueue.pQueueList = gGwService_PacketPendingQueue.pQueueList->pNext;
  }
}

/*********************************************************************
 * @fn      gwServices_StartStateMachine_DeviceAttrList
 *
 * @brief   Starts state machine for Get Device Attribute List Request
 *
 * @param   connection - connection handle (tcp)
 * @param   pDeviceInfo - pointer to remote device's database information
 * @param   pGetAttrListReq - pointer to request command structure
 *
 * @return  GW_SERVICES_BUSY, GW_SERVICES_SUCCESS, or GW_SERVICES_FAILURE
 */
uint8 gwServices_StartStateMachine_DeviceAttrList( int connection, SrvrDeviceInfoT *pDeviceInfo, 
                                                   GwGetDeviceAttributeListReq *pGetAttrListReq )
{
  gwServices_StateMachine_AttrList_t *pAttrListState;
  ZStatus_t status;
  
  uiPrintfEx(trDEBUG, "Starting Attribute List State Machine - IeeeAddr: %016llX",  
                   pGetAttrListReq->dstaddress->ieeeaddr );
  
  // Check to see if state machine is already in use for this IEEE addresss
  if ( gwServices_FindStateMachine( pGetAttrListReq->dstaddress->ieeeaddr ) )
  {
    uiPrintf( "\nState Machine Error - duplicate state machine\n" );
    
    return GW_SERVICES_BUSY;
  }
  
  // Get available state machine
  pAttrListState = gwServices_UpdateAttrListTable();
  if ( !pAttrListState )
  {
    uiPrintf( "\nState Machine Error - unable to allocate more state machines\n" );
    
    return GW_SERVICES_FAILURE;
  }
  
  // Fill in state machine information
  pAttrListState->state = gwServices_State_GettingAttrListRsp_c;
  pAttrListState->ieeeAddr = pGetAttrListReq->dstaddress->ieeeaddr;
  pAttrListState->endpointId = pGetAttrListReq->dstaddress->endpointid;
  pAttrListState->connection = connection;
  pAttrListState->pHead = NULL;
  pAttrListState->pAttrList = NULL;
  pAttrListState->clusterCount = 0;
  pAttrListState->clusterIndex = 0;
  pAttrListState->startAttr = 0x0000;
  
  // Find cluster list
  if ( gwServices_FindClustersOnEndpoint( pDeviceInfo, pAttrListState ) )
  {
    // Send out first attribute list request
    status = sendGwGetDeviceAttributeListReq( connection, pGetAttrListReq, 
                                              pAttrListState->pServerClusterList[0], pAttrListState->startAttr );
  
    if ( status == ZSuccess )
    {     
      return GW_SERVICES_SUCCESS;
    }                            
    else
    {
      uiPrintf( "\nState Machine Error - ZCL send status: %d\n", status );
      
      return GW_SERVICES_FAILURE;
    }     
  }
  else
  {
    // Unable to find clusters on matching endpoint
    uiPrintf( "\nState Machine Error - Unable to find matching clusters on endpoint\n" );
    
    // Clear state machine
    pAttrListState->state = gwServices_State_Available_c;
  
    return GW_SERVICES_FAILURE;
  }              
}

/*********************************************************************
 * @fn      gwServices_UpdateStateMachine_DeviceAttrList
 *
 * @brief   Called upon receiving response for Get Device Attribute List 
 *          Request. Handles updating state machine and any further 
 *          action required for command.
 *
 * @param   pGetAttrListRsp - pointer to response command structure
 * @param   appTransId - application transaction ID
 * @param   inClusterId - responding cluster ID
 * @param   pDiscAttrsRsp - pointer to OTA response structure
 *
 * @return  GW_SERVICES_SUCCESS, GW_SERVICES_FAILURE (unknown device), or
 *          GW_SERVICES_MEM_ERROR
 */
uint8 gwServices_UpdateStateMachine_DeviceAttrList( GwGetDeviceAttributeListRspInd *pGetAttrListRsp, 
                                                    uint16 appTransId, uint16 inClusterId, 
                                                    zclDiscoverAttrsRspCmd_t *pDiscAttrsRsp )
{
  gwServices_StateMachine_AttrList_t *pState;
  
  // Check to see if state machine is already in use for this IEEE addresss
  pState = gwServices_FindStateMachine( pGetAttrListRsp->srcaddress->ieeeaddr );
  if ( !pState )
  {
    uiPrintf( "\nState Machine: Unable to update device - unknown IEEE address\n" );
    
    return GW_SERVICES_FAILURE; // unknown remote device
  }
  
  if ( (pState->clusterIndex == 0) && (pState->startAttr = 0x0000) )
  {
    // Store app sequence number on first response
    pState->appTransId = appTransId;
  }

  // Fill in attribute information
  if ( !gwServices_AddAttrList( pState, inClusterId, pDiscAttrsRsp ) )
  {
    // Free up data
    gwServices_FreeStateMachine( pState );
    
    return GW_SERVICES_MEM_ERROR;
  }
  
  // Check to see if all messages have been sent
  if ( pState->clusterIndex < pState->clusterCount )
  {
    uint16 nextClusterId;
    
    GwGetDeviceAttributeListReq getAttrListReq = GW_GET_DEVICE_ATTRIBUTE_LIST_REQ__INIT;
    
    nextClusterId = pState->pServerClusterList[pState->clusterIndex]; // store next cluster ID
    
    getAttrListReq.dstaddress = pGetAttrListRsp->srcaddress;
    
    // Send out message to ZigBee
    if ( ZSuccess == sendGwGetDeviceAttributeListReq( pState->connection, &getAttrListReq, nextClusterId, pState->startAttr ) )
    {
      return GW_SERVICES_PROCESSING;
    }
  }
  else
  {
    pState->state = gwServices_State_Finish_c;
    
    // If all messages have been sent, send response to app
    gwServices_FinishStateMachine( pState );
    
    return GW_SERVICES_SUCCESS;
  }
  
  return GW_SERVICES_FAILURE;
}

/*********************************************************************
 * @fn      gwServices_FindStateMachine
 *
 * @brief   Finds state machine already in progress
 *
 * @param   ieeeAddr - IEEE address of device
 *
 * @return  pointer to valid state machine, NULL if not found
 */
static gwServices_StateMachine_AttrList_t * gwServices_FindStateMachine( uint64_t ieeeAddr )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giGwServices_AttrListTableCount; ++i )
  {
    if ( (gpGwServices_StateMachine_AttrList[i].state != gwServices_State_Available_c) && 
         (gpGwServices_StateMachine_AttrList[i].ieeeAddr == ieeeAddr) )
    {
      return &gpGwServices_StateMachine_AttrList[i];
    }
  }

  return NULL;  // not found
}

/*********************************************************************
 * @fn      gwServices_GetStateMachine
 *
 * @brief   Finds available state machine
 *
 * @param   none
 *
 * @return  pointer to available state machine, NULL if none available
 */
static gwServices_StateMachine_AttrList_t * gwServices_GetStateMachine( void )
{
  uint8 status;
  gwServices_StateMachine_AttrList_t *pNewStateMachine;
  
  // Get new state machine
  pNewStateMachine = gwServices_GetAttrListEntry();

  // Not free state machine
  if ( !pNewStateMachine )
  {
    // Table is full, allocate more space
    status = gwServices_AddAttrListTable();
    
    if ( status == GW_SERVICES_SUCCESS )
    {
      // Get new state machine
      pNewStateMachine = gwServices_GetAttrListEntry();
    }
  }
  
  if ( pNewStateMachine )
  {
    return pNewStateMachine;
  }
  else
  {
    uiPrintfEx(trDEBUG, "Attribute list: Error - Can't allocate more state machines\n" );
      
    return NULL;
  }
}

/*********************************************************************
 * @fn      gwServices_FindClustersOnEndpoint
 *
 * @brief   Finds server clusters listed on a given endpoint in the database
 *
 * @param   pDeviceInfo - pointer to remote device's database information
 * @param   pState - pointer to state machine
 *
 * @return  TRUE for success, FALSE for failure
 */
static bool gwServices_FindClustersOnEndpoint( SrvrDeviceInfoT *pDeviceInfo, 
                                               gwServices_StateMachine_AttrList_t *pState )
{
  uint8 i;
  uint8 clusterCount;
  uint32 *pInputClusters;
  
  // Search for Cluster ID in database simple descriptor list
  for ( i = 0; i < pDeviceInfo->n_simpledesclist; i++ )
  {
    if ( pDeviceInfo->simpledesclist[i]->endpointid == pState->endpointId )
    {
      clusterCount = pDeviceInfo->simpledesclist[i]->n_inputclusters;
      
      pInputClusters = pDeviceInfo->simpledesclist[i]->inputclusters;
      
      break;
    }
  }
  
  pState->clusterCount = clusterCount;  // store cluster count
  
  if ( clusterCount )
  {
    // Allocate memory
    pState->pServerClusterList = malloc( sizeof( uint16 ) * clusterCount );
    if ( !pState->pServerClusterList )
    {
      uiPrintf( "\nFind Clusters On Endpoint: Memory Error\n" );
      
      return FALSE; // memory error
    }
    
    // Store cluster information
    for ( i = 0; i < clusterCount; i++ )
    {
      pState->pServerClusterList[i] = pInputClusters[i];
    }
    
    return TRUE;
  }
  
  return FALSE;
}

/*********************************************************************
 * @fn      gwServices_AddAttrList
 *
 * @brief   Add new attribute information to list
 *
 * @param   pState - pointer to state machine
 * @param   clusterId - attribute list cluster ID
 * @param   pDiscAttrRsp - pointer to discover attributes response payload
 *
 * @return  Returns TRUE if successful, FALSE for memory failure
 */
static uint8 gwServices_AddAttrList( gwServices_StateMachine_AttrList_t *pState, uint16 clusterId, 
                                     zclDiscoverAttrsRspCmd_t *pDiscAttrRsp )
{
  sGwServices_AttrList_t *pNewAttrListRecord;
  GwClusterListT *pClusterList;
  
  if ( pState == NULL )
  {
    return FALSE; // unexpected error
  }

  if (pState->startAttr == 0x0000) //if this is the first chunk of attributes reported for the current cluster
  {
  // Allocate memory
  pNewAttrListRecord = malloc( sizeof( sGwServices_AttrList_t ) );
  if ( !pNewAttrListRecord )
  {
    return FALSE; // memory error
  }
  
  pNewAttrListRecord->pNext = NULL;
  
    if ( pState->pHead == NULL)
  {
    // Mark head of record
    pState->pHead = pNewAttrListRecord;
    pState->pAttrList = pNewAttrListRecord;
  }
    else if (pState->pAttrList != NULL)
    {
      // Find the end of the allocated records
      while ( pState->pAttrList->pNext != NULL )
      {
        pState->pAttrList = pState->pAttrList->pNext;
      }

      pState->pAttrList->pNext = pNewAttrListRecord;   // move pointer to next allocated structure
    }
  else
  {
      return FALSE; // unexpected error
    }
  }  
  else
    {
    if ( pState->pAttrList == NULL )
      {
      return FALSE; // unexpected error
    }
        
    pNewAttrListRecord = pState->pAttrList->pNext;
      }
      
  
  // Store ZCL message in protobuf structure
  pClusterList = gwServices_ConvertDiscAttrsRspPb( clusterId, pDiscAttrRsp, pState->startAttr, pNewAttrListRecord->pClusterList );
  
  if ( pClusterList == NULL )
  {
    return FALSE; // memory error
  }
  
  pNewAttrListRecord->pClusterList = pClusterList;
  
  // Check if response on cluster is complete
  if ( pDiscAttrRsp->discComplete )
  {
    pState->clusterIndex++;
    pState->startAttr = 0x0000;
  }
  else
  {
    //request the next chunk of attributes of the current cluster
    pState->startAttr = pDiscAttrRsp->attrList[pDiscAttrRsp->numAttr - 1].attrID + 1;
  }
  
  uiPrintfEx(trDEBUG, "Update State Machine: Cluster Index: %d, Remaining Clusters: %d\n", 
                   pState->clusterIndex, pState->clusterCount );
  
  return TRUE;
}

/*********************************************************************
 * @fn      gwServices_FinishStateMachine
 *
 * @brief   Free memory and state machine, send response indication 
 *          to the application
 *
 * @param   pState - pointer to state machine
 *
 * @return  none
 */
static void gwServices_FinishStateMachine( gwServices_StateMachine_AttrList_t *pState )
{
  uint8 i;
  GwClusterListT **ppClusterList;
  GwAddressStructT srcAddr = GW_ADDRESS_STRUCT_T__INIT;
  GwGetDeviceAttributeListRspInd getAttrListRsp = GW_GET_DEVICE_ATTRIBUTE_LIST_RSP_IND__INIT;
  
  // Configure protobuf address structure
  srcAddr.addresstype = GW_ADDRESS_TYPE_T__UNICAST;
  srcAddr.has_ieeeaddr = TRUE;
  srcAddr.ieeeaddr = pState->ieeeAddr;
  srcAddr.has_endpointid = TRUE;
  srcAddr.endpointid = pState->endpointId;
  
  // Prepare message to call on app sending function
  getAttrListRsp.sequencenumber = pState->appTransId;
  
  if ( pState->clusterIndex == pState->clusterCount )
  {
    getAttrListRsp.status = GW_STATUS_T__STATUS_SUCCESS;
  }
  else
  {
    getAttrListRsp.status = GW_STATUS_T__STATUS_FAILURE;
    getAttrListRsp.n_clusterlist = pState->clusterCount;
    getAttrListRsp.clusterlist = NULL;
    
    sendGwGetDeviceAttributeListRspInd( pState->connection, &getAttrListRsp );
  }
  
  getAttrListRsp.srcaddress = &srcAddr;
  getAttrListRsp.n_clusterlist = pState->clusterCount;
  
  // Store cluster attribute lists
  ppClusterList = malloc( sizeof( GwClusterListT * ) * pState->clusterCount );
  if ( !ppClusterList )
  {
    gwServices_FreeStateMachine( pState );
    
    return; // error
  }
  
  getAttrListRsp.clusterlist = ppClusterList;
  
  pState->pAttrList = pState->pHead;  // make sure attribute list is at the head
  
  // Go through server-side cluster list and consolidate cluster lists
  for ( i = 0; i < pState->clusterCount; i++ )
  {
    // Search for clusters in linked list
    while ( pState->pAttrList )
    {
      // Find matching cluster IDs
      if ( pState->pServerClusterList[i] == pState->pAttrList->pClusterList->clusterid )
      {
        // Store cluster list pointer address to pointer structure
        ppClusterList[i] = pState->pAttrList->pClusterList;
        
        break;
      }
      
      pState->pAttrList = pState->pAttrList->pNext; // move to next list
    }
  }
  
  sendGwGetDeviceAttributeListRspInd( pState->connection, &getAttrListRsp );
  
  free( ppClusterList );
  
  // Free state machine
  gwServices_FreeStateMachine( pState );
}

/*********************************************************************
 * @fn      gwServices_FreeStateMachine
 *
 * @brief   Calls on function to free allocate memory for the Device 
 *          Attribute List and frees the state machine
 *
 * @param   pState - pointer to state machine
 *
 * @return  none
 */
static void gwServices_FreeStateMachine( gwServices_StateMachine_AttrList_t *pState )
{
  // Free attribute list memory
  gwServices_FreeAttrList( pState->pHead );
  
  if ( pState->clusterCount )
  {
    free( pState->pServerClusterList );
  }
  
  // Update state
  pState->state = gwServices_State_Available_c;
}

/*********************************************************************
 * @fn      gwServices_FreeAttrList
 *
 * @brief   Frees all allocated memory for the Device Attribute List
 *
 * @param   pAttrListRecord - pointer to allocated attribute data structure
 *
 * @return  none
 */
static void gwServices_FreeAttrList( sGwServices_AttrList_t *pAttrListRecord )
{
  sGwServices_AttrList_t *pFreeAttrList;
  
  // Assumes pAttrListRecord is at the head
  
  while ( pAttrListRecord )
  {
    pFreeAttrList = pAttrListRecord;  // store pointer address  
    pAttrListRecord = pAttrListRecord->pNext; // move to next address
    
    // Free memory
    if ( pFreeAttrList->pClusterList->n_attributelist )
    {
      free( pFreeAttrList->pClusterList->attributelist );
    }
    
    free( pFreeAttrList->pClusterList );
    free( pFreeAttrList );  
  }
}

/*********************************************************************
 * @fn      gwServices_ConvertDiscAttrsRspPb
 *
 * @brief   Allocates and stores ZCL Discover Attributes Response 
 *          in Protobuf native structure
 *
 * @param   clusterId - cluster ID for attribute list
 * @param   pDiscAttrRsp - pointer to OTA discover attributes 
 *                         response structure
 *
 * @return  Pointer to cluster attribute list protobuf structure
 */
static GwClusterListT * gwServices_ConvertDiscAttrsRspPb( uint16 clusterId, zclDiscoverAttrsRspCmd_t *pDiscAttrRsp, uint16 startAttr, GwClusterListT  *pClusterList )
{
  uint8 i;
  uint32 *pAttrList;
  GwClusterListT *pNewClusterList;
  
  // Allocate memory, will be freed later
  if (startAttr == 0x0000)
  {
  pNewClusterList = malloc( sizeof( GwClusterListT ) );
    if ( pNewClusterList == NULL )
  {
    return NULL;  // memory error
  }
  
  gw_cluster_list_t__init( pNewClusterList );
  
    pNewClusterList->n_attributelist = 0;
  pNewClusterList->clusterid = clusterId;
  
    pAttrList = malloc( pDiscAttrRsp->numAttr * sizeof( uint32 ) );

    if ( pAttrList == NULL )
{
      free( pNewClusterList );
  
      return NULL;  // memory error
    }
  }
  else
  {
    pNewClusterList = pClusterList;
    
    if ( pNewClusterList == NULL )
      {
      return NULL;  // unexpected error
      }
      
    pAttrList = realloc(pNewClusterList->attributelist, (pNewClusterList->n_attributelist + pDiscAttrRsp->numAttr) * sizeof( uint32 ));
    
    if ( pAttrList == NULL )
    {
      pNewClusterList->attributelist = NULL;
      pNewClusterList->n_attributelist = 0;
    
      return NULL;	// memory error
  }
}

  pNewClusterList->attributelist = pAttrList;
  
  for ( i = 0; i < pDiscAttrRsp->numAttr; i++ )
  {
    pAttrList[pNewClusterList->n_attributelist + i] = pDiscAttrRsp->attrList[i].attrID;
  }
  
  pNewClusterList->n_attributelist += pDiscAttrRsp->numAttr;
    
  return pNewClusterList;
}

/*********************************************************************
 * @fn      gwServices_StateMachineTimeout_DeviceAttrList
 *
 * @brief   Called when message response times out. Frees up 
 *          device's state machine
 *
 * @param   ieeeAddr - IEEE address of device
 *
 * @return  none
 */
void gwServices_StateMachineTimeout_DeviceAttrList( uint64_t ieeeAddr )
{
  gwServices_StateMachine_AttrList_t *pState;
  
  pState = gwServices_FindStateMachine( ieeeAddr );
  
  if ( !pState )
  {    
    return; // unknown remote device
  }
  
  // Free state machine
  gwServices_FreeStateMachine( pState );
}

/*********************************************************************
 * @fn      gwServices_ConvertPbClusterList
 *
 * @brief   Converts local attribute record to protobuf cluster list
 *          native form
 *
 * @param   attrCount - total number of local attributes
 * @param   pAttrRecord - pointer to local attribute record
 * @param   inClusterCount - total input clusters registered to provided endpoint
 * @param   pInClusterList - pointer to input cluster list
 * @param   pClusterListCount - returned count of pointer list
 *
 * @return  pointer to cluster list pointers, NULL if failure
 */
GwClusterListT ** gwServices_ConvertPbClusterList( int attrCount, zclAttrRec_t *pAttrRecord,
                                                   uint8 inClusterCount, uint32 *pInClusterList,
                                                   uint32 *pClusterListCount )
{
  uint8 count = 0;
  uint8 clusterMatchCount;
  int i;
  int j;
  uint32 *pNewAttrRecord;
  GwClusterListT *pClusterList;
  GwClusterListT **ppClusterList;
  
  for ( i = 0; i < inClusterCount; i++ )
  {
    // Look for all attributes matching cluster ID
    for ( j = 0; j < attrCount; j++ )
    {
      if ( pInClusterList[i] == pAttrRecord[j].clusterID )
      {
        clusterMatchCount++;
      }
    }
  }
  
  if ( clusterMatchCount == 0 )
  {
    return NULL; // no clusters found
  }
  
  *pClusterListCount = clusterMatchCount;
  
  // Allocate memory
  ppClusterList = malloc( sizeof( GwClusterListT * ) * clusterMatchCount );
  if ( !ppClusterList )
  {
    return NULL; // memory error
  }
  
  pClusterList = malloc( sizeof( GwClusterListT ) * clusterMatchCount );
  if ( !pClusterList )
  {
    free( ppClusterList );
    
    return NULL; // memory error
  }
  
  pNewAttrRecord = malloc( sizeof( uint32 ) * clusterMatchCount );
  if ( !pNewAttrRecord )
  {
    free( pClusterList );
    free( ppClusterList );
    
    return NULL; // memory error
  }
  
  for ( i = 0; i < inClusterCount; i++ )
  {          
    // Look for all attributes matching cluster ID
    for ( j = 0; j < attrCount; j++ )
    {
      if ( pInClusterList[i] == pAttrRecord[j].clusterID &&
           count <= clusterMatchCount )
      {
        gw_cluster_list_t__init( pClusterList );
    
        // Store cluster ID
        pClusterList->clusterid = pInClusterList[i];
        pClusterList->n_attributelist++;
        pClusterList->attributelist = pNewAttrRecord;
        *pNewAttrRecord++ = pAttrRecord[j].attr.attrId;
        
        ppClusterList[count++] = pClusterList++;
      }
    }
  }
  
  return ppClusterList;
}

/*********************************************************************
 * @fn      gwServices_FreePbClusterList
 *
 * @brief   Frees allocated protobuf cluster list structure
 *
 * @param   clusterListCount - total number of cluster lists
 * @param   ppClusterList - pointer to list of cluster list pointers
 *
 * @return  pointer to list of allocated cluster list pointers
 */
void gwServices_FreePbClusterList( int clusterListCount, GwClusterListT **ppClusterList )
{
  free( ppClusterList[0]->attributelist );
  free( ppClusterList[0] );
  free( ppClusterList );
}

/*********************************************************************
 * @fn      gwServices_UpdateAttrListTable
 *
 * @brief   New information needs to be added to the attribute list table
 *
 * @param   none
 *
 * @return  pointer to available table entry
 */
gwServices_StateMachine_AttrList_t * gwServices_UpdateAttrListTable( void )
{
  uint8 status;
  
  // Check current memory usage, allocate more if necessary
  status = gwServices_CheckAttrListTableSize();
  
  if ( status == GW_SERVICES_BUSY )
  {
    return NULL;
  }
  
  // Find available table entry
  return gwServices_GetStateMachine();
}

/*********************************************************************
 * @fn      gwServices_GetAttrListEntry
 *
 * @brief   Find available entry in table
 *
 * @param   none
 *
 * @return  pointer to table entry, NULL if memory error
 */
gwServices_StateMachine_AttrList_t * gwServices_GetAttrListEntry( void )
{
  int i;
  
  for ( i = 0; i < giGwServices_AttrListTableCount; i++ )
  {
    if ( gpGwServices_StateMachine_AttrList[i].state == gwServices_State_Available_c )
    {
      return &gpGwServices_StateMachine_AttrList[i];
    }
  }
  
  return NULL;  // no available entry's found
}

/*********************************************************************
 * @fn      gwServices_AddAttrListTable
 *
 * @brief   Allocate more memory to table
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_AddAttrListTable( void )
{
  uint64_t iNewTableSize;
  gwServices_StateMachine_AttrList_t *pNewAttrList;
  
  iNewTableSize = (sizeof( gwServices_StateMachine_AttrList_t ) * 
                  (giGwServices_AttrListTableCount + GW_ATTR_LIST_TABLE_COUNT));
  
  pNewAttrList = malloc( iNewTableSize );
  if ( !pNewAttrList )
  {
    return GW_SERVICES_BUSY;  // memory error, allocate later
  }
  
  // Clear out new table
  memset( pNewAttrList, 0, iNewTableSize );
  
  if ( gpGwServices_StateMachine_AttrList )
  {
    // Copy old table to new table
    memcpy( pNewAttrList, gpGwServices_StateMachine_AttrList, 
            (sizeof( gwServices_StateMachine_AttrList_t ) * 
            giGwServices_AttrListTableCount) );
    
    // Free old table
    gwServices_FreeAttrListTable( gpGwServices_StateMachine_AttrList );
  }
  
  // Increase count
  giGwServices_AttrListTableCount += GW_ATTR_LIST_TABLE_COUNT;
    
  // Point global to new table
  gpGwServices_StateMachine_AttrList = pNewAttrList;
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_CheckAttrListTableSize
 *
 * @brief   Check current table size, allocate more space if necessary
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_CheckAttrListTableSize( void )
{
  int i;
  int count = 0;
  
  for ( i = 0; i < giGwServices_AttrListTableCount; i++ )
  {
    if ( gpGwServices_StateMachine_AttrList[i].state != gwServices_State_Available_c )
    {
      count++;
    }
  }
  
  uiPrintfEx(trDEBUG, "Checked attribute list table size: %d\n", giGwServices_AttrListTableCount );
  uiPrintfEx(trDEBUG, "Checked attribute list number of entries: %d\n", count );
  
  if ( count == giGwServices_AttrListTableCount )
  {
    // Table is full, allocate more space
    return gwServices_AddAttrListTable();
  }
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_FreeAttrListTable
 *
 * @brief   Free old table
 *
 * @param   pAttrList - pointer to old table entry
 *
 * @return  none
 */
static void gwServices_FreeAttrListTable( gwServices_StateMachine_AttrList_t *pAttrList )
{
  free( pAttrList );
}

/*********************************************************************
 * @fn      gwServices_UpdateRetryTable
 *
 * @brief   New information needs to be added to the retry table
 *
 * @param   none
 *
 * @return  pointer to available table entry
 */
gsGwMsgRetryTable_t * gwServices_UpdateRetryTable( void )
{
  uint8 status;
  
  // Check current memory usage, allocate more if necessary
  status = gwServices_CheckRetryTableSize();
  
  if ( status == GW_SERVICES_BUSY )
  {
    return NULL;
  }
  
  // Find available table entry
  return gwServices_GetRetryTableEntry();
}

/*********************************************************************
 * @fn      gwServices_GetRetryTableEntry
 *
 * @brief   Find available entry
 *
 * @param   none
 *
 * @return  pointer to table entry, NULL if memory error
 */
static gsGwMsgRetryTable_t * gwServices_GetRetryTableEntry( void )
{
  int i;
  
  for ( i = 0; i < giGwServices_RetryTableCount; i++ )
  {
    if ( !(gpGwServices_RetryTable[i].inUse) )
    {
      return &gpGwServices_RetryTable[i];
    }
  }
  
  return NULL;  // no available entry's found
}

/**************************************************************************************************
 *
 * @fn          gwServices_GetDeviceInRetryTable
 *
 * @brief       Finds device's entry in the retry table
 *
 * @param       ieeeAddr - IEEE address of device
 *
 * @return      pointer to device's entry, NULL if no entry found
 *
 **************************************************************************************************/
gsGwMsgRetryTable_t * gwServices_GetDeviceInRetryTable( uint64_t ieeeAddr )
{
  int i;
  
  for ( i = 0; i < giGwServices_RetryTableCount; i++ )
  {
    if ( (gpGwServices_RetryTable[i].ieeeAddr == ieeeAddr) && 
         (gpGwServices_RetryTable[i].inUse == TRUE) )
    {
      return &gpGwServices_RetryTable[i];
    }         
  }
    
  return NULL; // no entry found
}

/*********************************************************************
 * @fn      gwServices_AddRetryTable
 *
 * @brief   Allocate more memory to table
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_AddRetryTable( void )
{
  uint64_t iNewTableSize;
  gsGwMsgRetryTable_t *pNewAttrList;
  
  iNewTableSize = (sizeof( gsGwMsgRetryTable_t ) * 
                  (giGwServices_RetryTableCount + GW_RETRY_TABLE_COUNT));
  
  pNewAttrList = malloc( iNewTableSize );
  if ( !pNewAttrList )
  {
    return GW_SERVICES_BUSY;  // memory error, allocate later
  }
  
  // Clear out new table
  memset( pNewAttrList, 0, iNewTableSize );
  
  if ( gpGwServices_RetryTable )
  {
    // Copy old table to new table
    memcpy( pNewAttrList, gpGwServices_RetryTable, 
            (sizeof( gsGwMsgRetryTable_t ) * 
            giGwServices_RetryTableCount) );
    
    // Free old table
    gwServices_FreeRetryTable( gpGwServices_RetryTable );
  }
  
  // Increase count
  giGwServices_RetryTableCount += GW_RETRY_TABLE_COUNT;
    
  // Point global to new table
  gpGwServices_RetryTable = pNewAttrList;
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_CheckRetryTableSize
 *
 * @brief   Check current table size, allocate more space if necessary
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_CheckRetryTableSize( void )
{
  int i;
  int count = 0;
  
  for ( i = 0; i < giGwServices_RetryTableCount; i++ )
  {
    if ( gpGwServices_RetryTable[i].inUse )
    {
      count++;
    }
  }
  
  uiPrintfEx(trDEBUG, "Checked retry table size: %d\n", giGwServices_RetryTableCount );
  uiPrintfEx(trDEBUG, "Checked retry table number of entries: %d\n", count );
  
  if ( count == giGwServices_RetryTableCount )
  {
    // Table is full, allocate more space
    return gwServices_AddRetryTable();
  }
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_FreeRetryTable
 *
 * @brief   Free old table
 *
 * @param   pRetryTable - pointer to old table
 *
 * @return  none
 */
static void gwServices_FreeRetryTable( gsGwMsgRetryTable_t *pRetryTable )
{
  free( pRetryTable );
}

/*********************************************************************
 * @fn      gwServices_UpdateTransTable
 *
 * @brief   New information needs to be added to the transaction table
 *
 * @param   none
 *
 * @return  pointer to available table entry
 */
gsGwMsgTransTable_t * gwServices_UpdateTransTable( void )
{
  uint8 status;
  
  // Check current memory usage, allocate more if necessary
  status = gwServices_CheckTransTableSize();
  
  if ( status == GW_SERVICES_BUSY )
  {
    return NULL;
  }
  
  // Find available table entry
  return gwServices_GetTransTableEntry();
}

/*********************************************************************
 * @fn      gwServices_GetTransTableEntry
 *
 * @brief   Find available entry
 *
 * @param   none
 *
 * @return  pointer to table entry, NULL if memory error
 */
static gsGwMsgTransTable_t * gwServices_GetTransTableEntry( void )
{
  int i;
  
  for ( i = 0; i < giGwServices_TransTableCount; i++ )
  {
    if ( !(gpGwServices_TransTable[i].inUse) )
    {
      return &gpGwServices_TransTable[i];
    }
  }
  
  return NULL;  // no available entry's found
}

/*********************************************************************
 * @fn      gwServices_AddTransTable
 *
 * @brief   Allocate more memory to table
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_AddTransTable( void )
{
  uint64_t iNewTableSize;
  gsGwMsgTransTable_t *pNewAttrList;
  
  iNewTableSize = (sizeof( gsGwMsgTransTable_t ) * 
                  (giGwServices_TransTableCount + GW_TRANS_TABLE_COUNT));
  
  pNewAttrList = malloc( iNewTableSize );
  if ( !pNewAttrList )
  {
    return GW_SERVICES_BUSY;  // memory error, allocate later
  }
  
  // Clear out new table
  memset( pNewAttrList, 0, iNewTableSize );
  
  if ( gpGwServices_TransTable )
  {
    // Copy old table to new table
    memcpy( pNewAttrList, gpGwServices_TransTable, 
            (sizeof( gsGwMsgTransTable_t ) * 
            giGwServices_TransTableCount) );
    
    // Free old table
    gwServices_FreeTransTable( gpGwServices_TransTable );
  }
  
  // Increase count
  giGwServices_TransTableCount += GW_TRANS_TABLE_COUNT;
    
  // Point global to new table
  gpGwServices_TransTable = pNewAttrList;
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_CheckTransTableSize
 *
 * @brief   Check current table size, allocate more space if necessary
 *
 * @param   none
 *
 * @return  GW_SERVICES_BUSY if unable to allocate memory or GW_SERVICES_SUCCESS
 */
static uint8 gwServices_CheckTransTableSize( void )
{
  int i;
  int count = 0;
  
  for ( i = 0; i < giGwServices_TransTableCount; i++ )
  {
    if ( gpGwServices_TransTable[i].inUse )
    {
      count++;
    }
  }
  
  uiPrintfEx(trDEBUG, "Checked transaction table size: %d\n", giGwServices_TransTableCount );
  uiPrintfEx(trDEBUG, "Checked transaction table number of entries: %d\n", count );
  
  if ( count == giGwServices_TransTableCount )
  {
    // Table is full, allocate more space
    return gwServices_AddTransTable();
  }
  
  return GW_SERVICES_SUCCESS;
}

/*********************************************************************
 * @fn      gwServices_FreeTransTable
 *
 * @brief   Free old table
 *
 * @param   pTransTable - pointer to old table
 *
 * @return  none
 */
static void gwServices_FreeTransTable( gsGwMsgTransTable_t *pTransTable )
{
  free( pTransTable );
}


