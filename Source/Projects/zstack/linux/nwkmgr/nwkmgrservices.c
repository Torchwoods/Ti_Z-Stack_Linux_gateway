/**************************************************************************************************
 Filename:       zigbeeservices.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    This file contains the database server shared by the Gateway and Network servers.


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
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include "trace.h"

#include "zstack.pb-c.h"
#include "nwkmgrsrv.h"
#include "nwkmgrservices.h"
#include "nwkmgrdatabase.h"

/**********************************
  Local Types and Defines
***********************************/

typedef enum zNwkSrv_AD_State_tag
{
  zNwkSrv_AD_State_Available_c = 0,       // must be 0
  zNwkSrv_AD_State_Waiting_c,             // used to wait before getting the parent
  zNwkSrv_AD_State_GettingParentAddr_c,   // the parent address must be available from ZdoTcDeviceInd
  zNwkSrv_AD_State_GettingNwkAddr_c,
  zNwkSrv_AD_State_GettingNodeInfo_c,
  zNwkSrv_AD_State_GettingActiveEndpoints_c,
  zNwkSrv_AD_State_GettingSimpleDesc_c,
  zNwkSrv_AD_State_WrapUp_c   // wrap up the state machine (free the memory and set state to available)
} zNwkSrv_AD_State_t;

// add device state machine
typedef struct zNwkSrv_AD_StateMachine_tag
{
  zNwkSrv_AD_State_t       state;
  uint16                   appSeq;
  uint16                   parentNwkAddr;
  sNwkMgrDb_DeviceInfo_t  *pDeviceInfo;  // allocated by the state machine, must be freed.
  pfnZNwkSrvAddDeviceCB_t  pfnZNwkSrvAddDeviceCB;
  uint8                    ep;           // which endpoint index are we getting info on? (SimpleDesc)
  int                      tries;        // up to 3 tries per state before timing out
  int                      timeout;      // ms before timeout
} zNwkSrv_AD_StateMachine_t;

// for timeleft
#define NM_WAITING_TICKS    (7000 / 250)            // 7 seconds

int giNwkSrv_RetryTableCount = 0;
gsNmMsgRetryTable_t *gpNwkSrv_RetryTable = NULL;

/**********************************
  Local Prototypes
***********************************/
#ifdef NWKMGR_SRVR
static void zNwkSrv_AD_FreeStateMachine( zNwkSrv_AD_StateMachine_t *pState );
static void zNwkSrv_AD_WrapUp( zNwkSrv_AD_StateMachine_t *pState );
static zNwkSrv_AD_StateMachine_t * zNwkSrv_AD_ReuseStateMachine( uint64_t ieeeAddr );
static zNwkSrv_AD_StateMachine_t * zNwkSrv_AD_GetStateMachine( void );
static void zNwkSrv_AD_ProcessNwkAddrRsp( ZdoNwkAddrRspInd *pNwkAddrRsp );
static void zNwkSrv_AD_ProcessIeeeAddrRsp( ZdoIeeeAddrRspInd *pNwkAddrRsp );
static void zNwkSrv_AD_ProcessNodeDescRsp( ZdoNodeDescRspInd *pNodeDescRsp );
static void zNwkSrv_AD_ProcessActiveEpRsp( ZdoActiveEndpointsRspInd *pActiveEpRsp );
static void zNwkSrv_AD_ProcessSimpleDescRsp( ZdoSimpleDescRspInd *pSimpleDescRsp );
static void zNwkSrv_TimerCallback( zNwkSrv_AD_StateMachine_t *pState );
void zNwkSrv_ResetTime(zNwkSrv_AD_StateMachine_t *pState);
gsNmMsgRetryTable_t * zNwkSrv_UpdateRetryTable( void );     
static gsNmMsgRetryTable_t * zNwkSrv_GetRetryTableEntry( void );  
gsNmMsgRetryTable_t * zNwkSrv_GetDeviceInRetryTable( uint64_t ieeeAddr ); 
static uint8 zNwkSrv_AddRetryTable( void );  
static uint8 zNwkSrv_CheckRetryTableSize( void );  
static void zNwkSrv_FreeRetryTable( gsNmMsgRetryTable_t *pAttrList );
#endif  // NWKMGR_SRVR

/**********************************
  Data
***********************************/

// state machines
zNwkSrv_AD_StateMachine_t *gNwkSrv_AD_StateMachine;
static int giNwkSrv_AD_NumStateMachines = 0;


/**********************************
  Code
***********************************/

#ifdef NWKMGR_SRVR

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AddDevice
 *
 * @brief   Adds a device to the database. Will make a series of ZigBee calls to find out all of
 *          the device information.
 * 
 * @param   ieeeAddr     address of the ndoe
 * @param   tcInfo       pointer to ZStack ZdoTcDeviceInd, or NULL if Maintenance Request
 * @param   appSeq       application sequence # (if required) for sending final CB
 * @param   pfnZNwkSrvAddDeviceCB   called once the process is complete (or failed)
 *
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_AddDevice( uint64_t ieeeAddr, ZdoTcDeviceInd *tcInfo, uint16 appSeq, pfnZNwkSrvAddDeviceCB_t pfnZNwkSrvAddDeviceCB )
{
  zNwkSrv_AD_StateMachine_t *pState;
  
  uiPrintfEx(trINFO, "\nNwkMgr AddDevice State Machine Started on 0x%016llx\n", ieeeAddr );
  
  if(tcInfo)
  {
    uiPrintfEx(trINFO, "  nwkAddr %04X, parent %04X\n", tcInfo->nwkaddr, tcInfo->parentaddr );
  }
  else
  {
    uiPrintfEx(trINFO, "  refresh\n" );
  }

  // if we already have a state machine, reuse it, if not, get a new one
  pState = zNwkSrv_AD_ReuseStateMachine( ieeeAddr );
  if ( !pState ) 
  {
    pState = zNwkSrv_AD_GetStateMachine();
  }

  // no state machines, give up!
  if ( !pState || !pState->pDeviceInfo )
  {
    // tell user if requested
    if ( pfnZNwkSrvAddDeviceCB )
    {
      (*pfnZNwkSrvAddDeviceCB)( appSeq, NULL, NS_ADS_NO_MEM );
    }
    if(pState)
    {
      pState->state = zNwkSrv_AD_State_Available_c;
    }
    return;
  }

  // set up state machine common items
  pState->pDeviceInfo->ieeeAddr = ieeeAddr;
  pState->pDeviceInfo->status = devInfoFlags_OnLine_c;   // assume online, it will go off-line if it times out
  pState->appSeq = appSeq;
  pState->pfnZNwkSrvAddDeviceCB = pfnZNwkSrvAddDeviceCB;
  pState->tries = MAX_DEVICE_FAILED_ATTEMPTS - 1;

  // getting parent IEEE address (we have the short address)
  if(tcInfo)
  {
    pState->parentNwkAddr = tcInfo->parentaddr;
    pState->pDeviceInfo->nwkAddr = tcInfo->nwkaddr;

    // using TcDeviceIndication, get parent IEEE addr of device
    pState->state = zNwkSrv_AD_State_Waiting_c;
    pState->timeout = NM_WAITING_TICKS;
  }

  // no parent info or NWK info, just try to get NWK info
  else
  {
    // assume same parent as in the database, if found in the database (otherwise it will be 0x00s)
    nwkMgrDb_GetParentAddr( ieeeAddr, &pState->pDeviceInfo->parentAddr );

    // use standard timeout
    zNwkSrv_ResetTime( pState );

    // have ieeeAddr, get NwkAddr
    pState->state = zNwkSrv_AD_State_GettingNwkAddr_c;
    sendZdoNwkAddrReq( ieeeAddr );
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_ResetTime
 *
 * @brief   Starting up or got a successful response, reset the timer for next step
 *
 * @param   pState - pointer to state machine
 * 
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_ResetTime( zNwkSrv_AD_StateMachine_t *pState )
{
  pState->timeout = giNmStateTicks;
}

/*********************************************************************
 * @fn      zNwkSrv_UpdateRetryTable
 *
 * @brief   New information needs to be added to the retry table
 *
 * @param   none
 *
 * @return  pointer to available table entry
 *********************************************************************/
gsNmMsgRetryTable_t * zNwkSrv_UpdateRetryTable( void )
{
  uint8 status;
  
  // Check current memory usage, allocate more if necessary
  status = zNwkSrv_CheckRetryTableSize();
  
  if ( status == NS_ADS_BUSY )
  {
    return NULL;
  }
  
  // Find available table entry
  return zNwkSrv_GetRetryTableEntry();
}

/*********************************************************************
 * @fn      zNwkSrv_GetRetryTableEntry
 *
 * @brief   Find available entry
 *
 * @param   none
 *
 * @return  pointer to table entry, NULL if memory error
 *********************************************************************/
static gsNmMsgRetryTable_t * zNwkSrv_GetRetryTableEntry( void )
{
  int i;
  
  for ( i = 0; i < giNwkSrv_RetryTableCount; i++ )
  {
    if ( !(gpNwkSrv_RetryTable[i].inUse) )
    {
      return &gpNwkSrv_RetryTable[i];
    }
  }
  
  return NULL;  // no available entry's found
}

/**************************************************************************************************
 *
 * @fn          zNwkSrv_GetDeviceInRetryTable
 *
 * @brief       Finds device's entry in the retry table
 *
 * @param       ieeeAddr - IEEE address of device
 *
 * @return      pointer to device's entry, NULL if no entry found
 *
 **************************************************************************************************/
gsNmMsgRetryTable_t * zNwkSrv_GetDeviceInRetryTable( uint64_t ieeeAddr )
{
  int i;
  
  for ( i = 0; i < giNwkSrv_RetryTableCount; i++ )
  {
    if ( (gpNwkSrv_RetryTable[i].ieeeAddr == ieeeAddr) && 
         (gpNwkSrv_RetryTable[i].inUse == TRUE) )
    {
      return &gpNwkSrv_RetryTable[i];
    }         
  }
    
  return NULL; // no entry found
}

/*********************************************************************
 * @fn      zNwkSrv_AddRetryTable
 *
 * @brief   Allocate more memory to table
 *
 * @param   none
 *
 * @return  NS_ADS_BUSY if unable to allocate memory or NS_ADS_OK
 */
static uint8 zNwkSrv_AddRetryTable( void )
{
  uint64_t iNewTableSize;
  gsNmMsgRetryTable_t *pNewAttrList;
  
  iNewTableSize = (sizeof( gsNmMsgRetryTable_t ) * 
                  (giNwkSrv_RetryTableCount + NWKSRV_RETRY_TABLE_COUNT));
  
  pNewAttrList = malloc( iNewTableSize );
  if ( !pNewAttrList )
  {
    return NS_ADS_BUSY;  // memory error, allocate later
  }
  
  // Clear out new table
  memset( pNewAttrList, 0, iNewTableSize );
  
  if ( gpNwkSrv_RetryTable )
  {
    // Copy old table to new table
    memcpy( pNewAttrList, gpNwkSrv_RetryTable, 
            (sizeof( gsNmMsgRetryTable_t ) * 
            giNwkSrv_RetryTableCount) );
    
    // Free old table
    zNwkSrv_FreeRetryTable( gpNwkSrv_RetryTable );
  }
  
  // Increase count
  giNwkSrv_RetryTableCount += NWKSRV_RETRY_TABLE_COUNT;
    
  // Point global to new table
  gpNwkSrv_RetryTable = pNewAttrList;
  
  return NS_ADS_OK;
}

/*********************************************************************
 * @fn      zNwkSrv_CheckRetryTableSize
 *
 * @brief   Check current table size, allocate more space if necessary
 *
 * @param   none
 *
 * @return  NS_ADS_BUSY if unable to allocate memory or NS_ADS_OK
 */
static uint8 zNwkSrv_CheckRetryTableSize( void )
{
  int i;
  int count = 0;
  
  for ( i = 0; i < giNwkSrv_RetryTableCount; i++ )
  {
    if ( gpNwkSrv_RetryTable[i].inUse )
    {
      count++;
    }
  }
  
  uiPrintfEx(trINFO, "\nChecked retry table size: %d\n", giNwkSrv_RetryTableCount );
  uiPrintfEx(trINFO, "\nChecked retry table number of entries: %d\n", count );
  
  if ( count == giNwkSrv_RetryTableCount )
  {
    // Table is full, allocate more space
    return zNwkSrv_AddRetryTable();
  }
  
  return NS_ADS_OK;
}

/*********************************************************************
 * @fn      zNwkSrv_FreeRetryTable
 *
 * @brief   Free old table
 *
 * @param   pRetryTable - pointer to old table
 *
 * @return  none
 */
static void zNwkSrv_FreeRetryTable( gsNmMsgRetryTable_t *pRetryTable )
{
  free( pRetryTable );
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_UpdateTimers
 *
 * @brief   Update timer ticks
 *
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_UpdateTimers( void )
{
  int i;
  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // if active and with a timeout
    if( gNwkSrv_AD_StateMachine[i].state )
    {
      if( gNwkSrv_AD_StateMachine[i].timeout)
      {
        --gNwkSrv_AD_StateMachine[i].timeout;
        if( 0 == gNwkSrv_AD_StateMachine[i].timeout )
        {
          zNwkSrv_TimerCallback( &gNwkSrv_AD_StateMachine[i] );
        }
      }
    }
  } // end for()
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_Timeout
 *
 * @brief   The current state has timed out. Either retry, or exit state machine with error
 *
 * @param   pState - pointer to state machine
 * 
 * @return  none
 *
 **************************************************************************************************/
static void zNwkSrv_TimerCallback( zNwkSrv_AD_StateMachine_t *pState )
{
  // invalid or already free
  if( !pState || pState->state == zNwkSrv_AD_State_Available_c )
    return;

  uiPrintfEx(trINFO, "\nAddDevice: Retrying state %d, remaining tries: %d\n", pState->state, pState->tries );
  
  // state machine sanity check
  if ( pState->tries )
  {
    // we've tried one more time
    --pState->tries;

    // restart the time
    zNwkSrv_ResetTime( pState );

    switch ( pState->state )
    {
      case zNwkSrv_AD_State_Waiting_c:
        // go on to getting parent address
        pState->state = zNwkSrv_AD_State_GettingParentAddr_c;
        sendZdoIeeeAddrReq( pState->parentNwkAddr );
        break;

      case zNwkSrv_AD_State_GettingParentAddr_c:
        sendUnicastRouteReq( pState->parentNwkAddr );
        sendZdoIeeeAddrReq( pState->parentNwkAddr );
        break;

      case zNwkSrv_AD_State_GettingNwkAddr_c:
        sendZdoNwkAddrReq( pState->pDeviceInfo->ieeeAddr ); // broadcast
        break;
        
      case zNwkSrv_AD_State_GettingNodeInfo_c:
        sendUnicastRouteReq( pState->pDeviceInfo->nwkAddr );
        sendZdoNodeDescReq( pState->pDeviceInfo->nwkAddr );
        break;
        
      case zNwkSrv_AD_State_GettingActiveEndpoints_c:
        sendUnicastRouteReq( pState->pDeviceInfo->nwkAddr );
        sendZdoActiveEndpointReq( pState->pDeviceInfo->nwkAddr );
        break;
        
      case zNwkSrv_AD_State_GettingSimpleDesc_c:
        sendUnicastRouteReq( pState->pDeviceInfo->nwkAddr );
        sendSimpleDescReq( pState->pDeviceInfo->nwkAddr, pState->pDeviceInfo->aEndpoint[pState->ep].endpointId );
        break;
      
      default:
        break;
    }
  }

  // give up, no tries left
  else
  {
    uiPrintfEx(trINFO, "\nAddDevice: Failed to get response from target node, state %d", pState->state );
    if( pState->pDeviceInfo )
    {
      uiPrintfEx(trINFO," device %016llX", pState->pDeviceInfo->ieeeAddr );
    }
    uiPrintfEx(trINFO,"\n");

    if ( pState->pfnZNwkSrvAddDeviceCB )
    {
      // call on the Network Server to let it know we're done
      pState->pDeviceInfo->status = devInfoFlags_OffLine_c;
      pState->pfnZNwkSrvAddDeviceCB( pState->appSeq, pState->pDeviceInfo, NS_ADS_NO_RSP );
    }

    // free the state machine
    zNwkSrv_AD_FreeStateMachine( pState );
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_UpdateAddDeviceStateMachine
 *
 * @brief   Received an incoming indication from the ZStack Server, and updates the state machine
 *
 * @param   cmdId - ZDO command ID
 * @param   pNativePbData - pointer to converted native protobuf incoming data
 * 
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_UpdateAddDeviceStateMachine( ZStackCmdIDs cmdId, void *pNativePbData )
{  
  // got a NwkAddr Rsp. See if it's one we care about.
  if ( cmdId == ZSTACK_CMD_IDS__ZDO_NWK_ADDR_RSP )
  {
    zNwkSrv_AD_ProcessNwkAddrRsp( (ZdoNwkAddrRspInd *)pNativePbData );
  }

  if ( cmdId == ZSTACK_CMD_IDS__ZDO_IEEE_ADDR_RSP )
  {
    zNwkSrv_AD_ProcessIeeeAddrRsp( (ZdoIeeeAddrRspInd *)pNativePbData );
  }

  if ( cmdId == ZSTACK_CMD_IDS__ZDO_NODE_DESC_RSP )
  {
    zNwkSrv_AD_ProcessNodeDescRsp( (ZdoNodeDescRspInd *)pNativePbData );
  }

  if ( cmdId == ZSTACK_CMD_IDS__ZDO_SIMPLE_DESC_RSP )
  {
    zNwkSrv_AD_ProcessSimpleDescRsp( (ZdoSimpleDescRspInd *)pNativePbData );
  }

  if ( cmdId == ZSTACK_CMD_IDS__ZDO_ACTIVE_EP_RSP )
  {
    zNwkSrv_AD_ProcessActiveEpRsp( (ZdoActiveEndpointsRspInd *)pNativePbData );
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_GetStateMachine
 *
 * @brief   Allocates a state machine entry, including the Device Info.
 *
 * @return  pointer to state machine, ready to be activated.
 *
 **************************************************************************************************/
static zNwkSrv_AD_StateMachine_t * zNwkSrv_AD_GetStateMachine( void )
{
  int i;
  int newCount;
  zNwkSrv_AD_StateMachine_t *pNewEntries;
  
  // will exit either with no memory (via return) or found state machine
  while( 1 )
  {

    // find a free entry
    for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
    {
      if ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_Available_c )
      {
        uiPrintfEx(trINFO, "\nAddDevice: Initiated new state machine\n" );
        break;
      }
    }

    // found an entry, use it
    if ( i < giNwkSrv_AD_NumStateMachines )
    {
      break;  // from while loop
    }

    // no free state machine, try adding more
    else
    {
      newCount = giNwkSrv_AD_NumStateMachines + NWKSRV_ADD_STATE_MACHINES;
      uiPrintfEx(trINFO, "\nAddDevice: No state machines, extending list to %d\n", newCount );
      pNewEntries = malloc( newCount * sizeof(zNwkSrv_AD_StateMachine_t) );
      if(!pNewEntries)
      {
        uiPrintfEx(trINFO, "\nAddDevice: Error - State machines full, no memory\n" );
        return NULL;
      }
      
      // clear all entries
      memset( pNewEntries, 0, newCount * sizeof(zNwkSrv_AD_StateMachine_t) );

      // copy old to new, and free old
      if(giNwkSrv_AD_NumStateMachines && gNwkSrv_AD_StateMachine)
      {
        memcpy( pNewEntries, gNwkSrv_AD_StateMachine, giNwkSrv_AD_NumStateMachines * sizeof(zNwkSrv_AD_StateMachine_t) );
        free( gNwkSrv_AD_StateMachine );
      }
      
      // now have a new count
      giNwkSrv_AD_NumStateMachines = newCount;
      gNwkSrv_AD_StateMachine = pNewEntries;
    }
  } // end while

  // found a free state machine. also, allocate a device info for this entry
  gNwkSrv_AD_StateMachine[i].pDeviceInfo = malloc( sizeof( sNwkMgrDb_DeviceInfo_t ) );
  if ( !gNwkSrv_AD_StateMachine[i].pDeviceInfo )
  {
    gNwkSrv_AD_StateMachine[i].state = zNwkSrv_AD_State_Available_c;
    return NULL;
  }
  // make sure to fill with 00s so things like simple descriptor pointers are NULL
  memset( gNwkSrv_AD_StateMachine[i].pDeviceInfo, 0, sizeof( sNwkMgrDb_DeviceInfo_t ) );

  return &gNwkSrv_AD_StateMachine[i];
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_UpdateCapInfo
 *
 * @brief   Update the capability info for this state machine. To be stored with the record.
 *
 * @param   ieeeaddr - address of device
 * @param   capInfo  - capability info
 *
 * @return  pointer to state machine, or NULL if not found
 *
 **************************************************************************************************/
bool zNwkSrv_AD_UpdateCapInfo( uint64_t ieeeAddr, uint8 capInfo )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    if ( gNwkSrv_AD_StateMachine[i].state != zNwkSrv_AD_State_Available_c && 
         gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == ieeeAddr)
    {
      gNwkSrv_AD_StateMachine[i].pDeviceInfo->capInfo = capInfo;
      return TRUE;
    }
  }
  
  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_StateMachineExists
 *
 * @brief   find and reuse the state machine by ieeeaddr
 *
 * @param   ieeeaddr - address of device
 *
 * @return  pointer to state machine, or NULL if not found
 *
 **************************************************************************************************/
bool zNwkSrv_AD_StateMachineExists( uint64_t ieeeAddr )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    if ( gNwkSrv_AD_StateMachine[i].state != zNwkSrv_AD_State_Available_c && 
         gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == ieeeAddr)
    {
      return TRUE;
    }
  }
  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ReuseStateMachine
 *
 * @brief   find and reuse the state machine by ieeeaddr
 *
 * @param   ieeeaddr - address of device
 *
 * @return  pointer to state machine, or NULL if not found
 *
 **************************************************************************************************/
static zNwkSrv_AD_StateMachine_t * zNwkSrv_AD_ReuseStateMachine( uint64_t ieeeAddr )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    if ( gNwkSrv_AD_StateMachine[i].state != zNwkSrv_AD_State_Available_c && 
         gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == ieeeAddr)
    {
      uiPrintfEx(trINFO, "\n- Restarting State Machine IeeeAddr: %016llX", gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr );
      
      // free the state machine
      zNwkSrv_AD_FreeStateMachine( &gNwkSrv_AD_StateMachine[i] );

      // add a new one (just like from scratch)
      return zNwkSrv_AD_GetStateMachine();
    }
  }

  return NULL;  // not found
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_HaltStateMachine
 *
 * @brief   finds the state machine by ieeeaddr and frees it
 *
 * @param   ieeeaddr - address of device
 *
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_AD_HaltStateMachine( uint64_t ieeeAddr )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    if ( (gNwkSrv_AD_StateMachine[i].state != zNwkSrv_AD_State_Available_c) && 
         (gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == ieeeAddr) )
    {
      uiPrintfEx(trINFO, "\n- Halting State Machine IeeeAddr: %016llX", gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr );
      
      // free the state machine
      zNwkSrv_AD_FreeStateMachine( &gNwkSrv_AD_StateMachine[i] );
      
      break;
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessDeviceAnnounce
 *
 * @brief   finds the state machine by ieeeaddr and clear timeout entry
 *
 * @param   ieeeaddr - address of device
 *
 * @return  none
 *
 **************************************************************************************************/
void zNwkSrv_AD_ProcessDeviceAnnounce( uint64_t ieeeAddr )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    if ( (gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_Waiting_c) && 
         (gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == ieeeAddr) )
    {
      // Update state of state machine
      zNwkSrv_TimerCallback(&gNwkSrv_AD_StateMachine[i]);
      break;
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_FreeStateMachine
 *
 * @brief   free the state machine (and the pDeviceInfo)
 *
 * @param   pState - pointer to response (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void zNwkSrv_AD_FreeStateMachine( zNwkSrv_AD_StateMachine_t *pState )
{
  if ( pState )
  {
    if ( pState->pDeviceInfo )
    {
      uiPrintfEx(trINFO, "\nAddDevice: State machine freed\n" );
      
      nwkMgrDb_FreeDeviceInfo( pState->pDeviceInfo );
    }

    // reset state machine to scratch
    memset( pState, 0, sizeof(zNwkSrv_AD_StateMachine_t) );
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessNwkAddrRsp
 *
 * @brief   process an nwk addr response. Fill in a DeviceInfo structure.
 *
 * @param   pNwkAddrRsp - pointer to response (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void  zNwkSrv_AD_ProcessNwkAddrRsp( ZdoNwkAddrRspInd *pNwkAddrRsp )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // find the local state machine
    if ( ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_GettingNwkAddr_c ) && 
         ( gNwkSrv_AD_StateMachine[i].pDeviceInfo->ieeeAddr == pNwkAddrRsp->ieeeaddr ) )
    {
      uiPrintfEx(trINFO, "\nAddDevice: Received NwkAddrRsp %04X\n", pNwkAddrRsp->nwkaddr );

      // reset the timer, we got the response
      zNwkSrv_ResetTime( &gNwkSrv_AD_StateMachine[i] );
      
      // add in the nwkAddr
      gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr = pNwkAddrRsp->nwkaddr;

      // move on to getting NodeInfo
      gNwkSrv_AD_StateMachine[i].state = zNwkSrv_AD_State_GettingNodeInfo_c;
      sendZdoNodeDescReq( pNwkAddrRsp->nwkaddr );
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessIeeeAddrRsp
 *
 * @brief   process an ieee addr response. Fill in a DeviceInfo structure.
 *
 * @param   pIeeeAddrRsp - pointer to response (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void  zNwkSrv_AD_ProcessIeeeAddrRsp( ZdoIeeeAddrRspInd *pIeeeAddrRsp )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // find the right state machine
    if ( ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_GettingParentAddr_c ) && 
         ( gNwkSrv_AD_StateMachine[i].parentNwkAddr == pIeeeAddrRsp->nwkaddr ) )
    {
      uiPrintfEx(trINFO, "\nAddDevice: Received Parent IeeeAddrRsp %04X\n", pIeeeAddrRsp->nwkaddr );
      
      // reset the timer, we got the response
      zNwkSrv_ResetTime( &gNwkSrv_AD_StateMachine[i] );

      // add in the parrent address
      gNwkSrv_AD_StateMachine[i].pDeviceInfo->parentAddr = pIeeeAddrRsp->ieeeaddr;

      // move on to getting NodeInfo
      gNwkSrv_AD_StateMachine[i].state = zNwkSrv_AD_State_GettingNodeInfo_c;
      sendZdoNodeDescReq( gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr );
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessNodeDescRsp
 *
 * @brief   process a node descriptor response. Fill in a DeviceInfo structure.
 *
 * @param   pNodeDescRsp - pointer to response (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void  zNwkSrv_AD_ProcessNodeDescRsp( ZdoNodeDescRspInd *pNodeDescRsp )
{
  int i;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // find the right state machine
    if( ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_GettingNodeInfo_c ) && 
        ( gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr == pNodeDescRsp->srcaddr ) )
    {
      uiPrintfEx(trINFO, "\nAddDevice: Received NodeDescriptorRsp\n" );
      
      // stop the timer, we got the response
      zNwkSrv_ResetTime( &gNwkSrv_AD_StateMachine[i] );

      // add in the mfgCode
      gNwkSrv_AD_StateMachine[i].pDeviceInfo->manufacturerId = pNodeDescRsp->nodedesc->manufacturercode;

      // move on to getting Active endpoints
      gNwkSrv_AD_StateMachine[i].state = zNwkSrv_AD_State_GettingActiveEndpoints_c;
      sendZdoActiveEndpointReq( gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr );
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessActiveEpRsp
 *
 * @brief   process an active endpoint response. Fill in a DeviceInfo structure.
 *
 * @param   pActiveEpRsp - pointer to response (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void zNwkSrv_AD_ProcessActiveEpRsp( ZdoActiveEndpointsRspInd *pActiveEpRsp )
{
  int i;
  int ep;
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // find the right state machine
    if( ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_GettingActiveEndpoints_c ) && 
        ( gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr == pActiveEpRsp->srcaddr ) )
    {
      uiPrintfEx(trINFO, "\nAddDevice: Received Active Endpoint Info Response\n" );

      // restart timeout, we got the response
      zNwkSrv_ResetTime( &gNwkSrv_AD_StateMachine[i] );

      // convenience variable
      pDeviceInfo = gNwkSrv_AD_StateMachine[i].pDeviceInfo;

      // add in active endpoints to this device
      pDeviceInfo->endpointCount = pActiveEpRsp->n_activeeplist;
      
      if( pDeviceInfo->endpointCount )
      {
        pDeviceInfo->aEndpoint = malloc( pDeviceInfo->endpointCount * sizeof(sNwkMgrDb_Endpoint_t) );
        if( !pDeviceInfo->aEndpoint )
        {
          // cannot continue processing if no memory
          break;
        }

        memset(pDeviceInfo->aEndpoint, 0, pDeviceInfo->endpointCount * sizeof(sNwkMgrDb_Endpoint_t));

        for ( ep = 0; ep < pDeviceInfo->endpointCount; ++ep )
        {
          pDeviceInfo->aEndpoint[ep].endpointId = pActiveEpRsp->activeeplist[ep];
        }

        // move on to getting Simple descriptors
        gNwkSrv_AD_StateMachine[i].ep = 0;    // for counting endpoints in next state
        gNwkSrv_AD_StateMachine[i].state = zNwkSrv_AD_State_GettingSimpleDesc_c;

        // try to get the first one (timeouts will cause retries)
        sendSimpleDescReq( pDeviceInfo->nwkAddr, pDeviceInfo->aEndpoint[0].endpointId );
      }
      
      // no active endpoints, go to done
      else
      {
        pDeviceInfo->aEndpoint = NULL;  // no endpoints
        zNwkSrv_AD_WrapUp( &gNwkSrv_AD_StateMachine[i] );
      }
    }
  }
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_ProcessSimpleDescRsp
 *
 * @brief   process a simple descriptor response. Fill in a DeviceInfo structure.
 *
 * @param   pSimpleDescRsp - pointer to simple descriptor (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void zNwkSrv_AD_ProcessSimpleDescRsp( ZdoSimpleDescRspInd *pSimpleDescRsp )
{
  int i;
  uint8 epIndex;
  uint8 ep;
  int cluster = 0;
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;
  sNwkMgrDb_Endpoint_t *pEndpoint;

  // find the state machine based on the ieeeAddr
  for ( i = 0; i < giNwkSrv_AD_NumStateMachines; ++i )
  {
    // find the record state machine entry
    if( ( gNwkSrv_AD_StateMachine[i].state == zNwkSrv_AD_State_GettingSimpleDesc_c ) && 
        ( gNwkSrv_AD_StateMachine[i].pDeviceInfo->nwkAddr == pSimpleDescRsp->srcaddr ) )
    {
      uiPrintfEx(trINFO, "\nAddDevice: Received Simple Descriptor Rsp node %04X, status %d\n",
        pSimpleDescRsp->srcaddr, pSimpleDescRsp->status );

      // reset the timer, we got the response
      zNwkSrv_ResetTime( &gNwkSrv_AD_StateMachine[i] );

      // convenience variable
      pDeviceInfo = gNwkSrv_AD_StateMachine[i].pDeviceInfo;
      epIndex = gNwkSrv_AD_StateMachine[i].ep;      // endpoint index

      uiPrintfEx(trINFO, "\nAddDevice: ep %d, epIndex %d, epCount %d\n", pSimpleDescRsp->simpledesc->endpoint, epIndex, pDeviceInfo->endpointCount);

      pEndpoint = &(pDeviceInfo->aEndpoint[epIndex]);
      
      // there was an error (no simple descriptor), reduce # of enpoints by 1
      if( pSimpleDescRsp->status != 0 )
      {
        // remove the active endpoint (copy all the other endpoint numbers down)
        uiPrintfEx(trINFO, "\nAddDevice: removing inactive endpoint %d\n", pDeviceInfo->aEndpoint[ep].endpointId );
        for(ep = epIndex; ep < pDeviceInfo->endpointCount-1; ++ep)
        {
          pDeviceInfo->aEndpoint[ep].endpointId = pDeviceInfo->aEndpoint[ep+1].endpointId;
        }
        --pDeviceInfo->endpointCount; // one less endpoint
      }

      // simple descriptor is OK, use it
      else
      {
        // fill in endpoint from response
        pEndpoint->profileId = (uint16)(pSimpleDescRsp->simpledesc->profileid);
        pEndpoint->deviceId  = (uint16)(pSimpleDescRsp->simpledesc->deviceid);
        pEndpoint->deviceVer = (uint8)(pSimpleDescRsp->simpledesc->devicever);
        pEndpoint->inputClusterCount = (uint16)(pSimpleDescRsp->simpledesc->n_inputclusters);
        pEndpoint->outputClusterCount = (uint16)(pSimpleDescRsp->simpledesc->n_outputclusters);

        uiPrintfEx(trINFO, "\nAddDevice: inClusters %d, outClusters %d\n", pEndpoint->inputClusterCount, pEndpoint->outputClusterCount );      

        // allocate memory for the input clusters
        pEndpoint->inputClusters = malloc( pEndpoint->inputClusterCount * sizeof(uint16) );
        pEndpoint->outputClusters = malloc( pEndpoint->outputClusterCount * sizeof(uint16) );
        if(!pEndpoint->inputClusters || !pEndpoint->outputClusters)
        {
          uiPrintfEx(trINFO, "- Failed to allocate clusters\n");
          break;  // cannot continue processing, no memory
        }

        for ( cluster = 0; cluster<pEndpoint->inputClusterCount; ++cluster )
        {
          pEndpoint->inputClusters[cluster] = (uint16_t)(pSimpleDescRsp->simpledesc->inputclusters[cluster]);
        }

        for ( cluster = 0; cluster<pEndpoint->outputClusterCount; ++cluster )
        {
          pEndpoint->outputClusters[cluster] = (uint16_t)(pSimpleDescRsp->simpledesc->outputclusters[cluster]);
        }

        // on to next endpoint
        (gNwkSrv_AD_StateMachine[i].ep)++;
      }
      
      // if more endpoints to get, get them now
      if ( gNwkSrv_AD_StateMachine[i].ep < pDeviceInfo->endpointCount )
      {
        // get the next endpoint info (Simple Descriptor)
        sendSimpleDescReq( pDeviceInfo->nwkAddr, pDeviceInfo->aEndpoint[gNwkSrv_AD_StateMachine[i].ep].endpointId );
      }

      // done, got all the info, tell the app we're ready
      else
      {
        zNwkSrv_AD_WrapUp( &gNwkSrv_AD_StateMachine[i] );
      }
    } // end if
  } // end for
}

/**************************************************************************************************
 *
 * @fn      zNwkSrv_AD_WrapUp
 *
 * @brief   Wrap up. We're done gather info about the state machine.
 *
 * @param   pSimpleDescRsp - pointer to simple descriptor (input)
 *
 * @return  none
 *
 **************************************************************************************************/
static void zNwkSrv_AD_WrapUp( zNwkSrv_AD_StateMachine_t *pState )
{
  pState->state = zNwkSrv_AD_State_WrapUp_c;
  pState->pDeviceInfo->status = devInfoFlags_OnLine_c;  // 

  if ( pState->pfnZNwkSrvAddDeviceCB )
  {
    uiPrintfEx(trINFO, "\nAddDevice: Service discovery complete, notify app\n" );
    
    // call on the Network Server to let it know we're done
    pState->pfnZNwkSrvAddDeviceCB( pState->appSeq, pState->pDeviceInfo, NS_ADS_OK );
  }

  // free the state machine
  zNwkSrv_AD_FreeStateMachine( pState );
}

#endif // NWKMGR_SRVR
