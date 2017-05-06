/**************************************************************************************************
  Filename:       nwkmgrp2p.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the peer-to-peer commands for the 
                  Network Manager server.

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

#include "server.pb-c.h"
#include "api_server.h"
#include "nwkmgrdatabase.h"
#include "nwkmgrservices.h"
#include "trace.h"

/**************************************************************************************************
 * Constants
 **************************************************************************************************/

/**************************************************************************************************
 * Typedefs
 **************************************************************************************************/

/**************************************************************************************************
 * Function Prototypes
 **************************************************************************************************/
void nmPb_processSrvrGetIeeeAddressReq( int connection, SrvrGetIeeeAddressReq *pIeeeAddrReq );
void nmPb_processSrvrGetShortAddressReq( int connection, SrvrGetShortAddressReq *pShortAddrReq );
void nmPb_processSrvrGetDeviceInfoReq( int connection, SrvrGetDeviceInfoReq *pDeviceInfoReq );
void nmPb_processSrvrGetDeviceStatusReq( int connection, SrvrGetDeviceStatusReq *pDeviceStatusReq );
void nmPb_processSrvrSetDeviceStatusReq( int connection, SrvrSetDeviceStatusReq *pDeviceStatusReq );

static void nmPb_sendSrvrGetIeeeAddressCnf( int connection, SrvrGetIeeeAddressCnf *pCnf );
static void nmPb_sendSrvrGetShortAddressReq( int connection, SrvrGetShortAddressCnf *pCnf );
static void nmPb_sendSrvrGetDeviceInfoCnf( int connection, SrvrGetDeviceInfoCnf *pCnf );
static void nmPb_sendSrvrGetDeviceStatusCnf( int connection, SrvrGetDeviceStatusCnf *pCnf );
static void nmPb_sendSrvrSetDeviceStatusCnf( int connection, SrvrSetDeviceStatusCnf *pCnf );

static SrvrSimpleDescriptorT ** nmPb_HandleServerSimpleDescPb( int simpleDescCount, 
                                                               sNwkMgrDb_Endpoint_t *pEpInfo );
static void nmPb_FreeServerSimpleDescPb( SrvrSimpleDescriptorT **ppSimpleDesc );

/**************************************************************************************************
 * Locals and Globals
 **************************************************************************************************/

/**************************************************************************************************
 * Code
 **************************************************************************************************/

/**************************************************************************************************
 * @fn          nmPb_processSrvrGetIeeeAddressReq
 *
 * @brief       Handles incoming request from Gateway/OTA Server.
 *              Gets the IEEE address of a device listed in the database.
 *
 * @param       connection    - tcp connection
 * @param       pIeeeAddrReq  - pointer to request structure
 *
 * @return      none
 **************************************************************************************************/
void nmPb_processSrvrGetIeeeAddressReq( int connection, SrvrGetIeeeAddressReq *pIeeeAddrReq )
{
  uint64_t ieeeAddr;
  SrvrGetIeeeAddressCnf ieeeAddrCnf = SRVR_GET_IEEE_ADDRESS_CNF__INIT;
  
  uiPrintf( "\nSRVR_GET_IEEE_ADDRESS_REQ: shortAddr: %04X\n", pIeeeAddrReq->shortaddress );
  
  if ( nwkMgrDb_GetIeeeAddr( pIeeeAddrReq->shortaddress, &ieeeAddr ) )
  {
    if( zNwkSrv_AD_StateMachineExists( ieeeAddr ) )
      ieeeAddrCnf.status = SRVR_STATUS_T__STATUS_BUSY;    // must stay awake to complete state machine
    else
      ieeeAddrCnf.status = SRVR_STATUS_T__STATUS_SUCCESS; // can sleep immediately

    ieeeAddrCnf.ieeeaddress = ieeeAddr;
  }
  else
  {
    ieeeAddrCnf. status = SRVR_STATUS_T__STATUS_FAILURE;  // not in database
  }
  
  // Send response back to server
  nmPb_sendSrvrGetIeeeAddressCnf( connection, &ieeeAddrCnf );
}

/**************************************************************************************************
 * @fn          nmPb_sendSrvrGetIeeeAddressCnf
 *
 * @brief       Sends the confirmation back to the requesting server.
 *
 * @param       connection    - tcp connection
 * @param       pCnf  - pointer to confirmation structure
 *
 * @return      none
 **************************************************************************************************/
static void nmPb_sendSrvrGetIeeeAddressCnf( int connection, SrvrGetIeeeAddressCnf *pCnf )
{
  int len;
  uint8 *pBuf;
  
  len = srvr_get_ieee_address_cnf__get_packed_size( pCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    srvr_get_ieee_address_cnf__pack( pCnf, pBuf );

    // Send response back to app (asynchronous data)
    APIS_SendData( connection, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, FALSE,
                   SRVR_CMD_ID_T__SRVR_GET_IEEE_ADDRESS_CNF, len, pBuf );
    
    free( pBuf );
  }
}

/**************************************************************************************************
 * @fn          nmPb_processSrvrGetShortAddressReq
 *
 * @brief       Handles incoming request from Gateway/OTA Server.
 *              Gets the short address of a device listed in the database.
 *
 * @param       connection    - tcp connection
 * @param       pShortAddrReq  - pointer to request structure
 *
 * @return      none
 **************************************************************************************************/
void nmPb_processSrvrGetShortAddressReq( int connection, SrvrGetShortAddressReq *pShortAddrReq )
{
  uint16 shortAddr;
  SrvrGetShortAddressCnf shortAddrCnf = SRVR_GET_SHORT_ADDRESS_CNF__INIT;
  
  uiPrintf( "\nSRVR_GET_SHORT_ADDRESS_REQ: ieeeAddr: %016llX\n", pShortAddrReq->ieeeaddress );
  
  if ( nwkMgrDb_GetShortAddr( pShortAddrReq->ieeeaddress, &shortAddr ) )
  {
    shortAddrCnf.status = SRVR_STATUS_T__STATUS_SUCCESS;
    
    shortAddrCnf.shortaddress = shortAddr;
  }
  else
  {
    shortAddrCnf. status = SRVR_STATUS_T__STATUS_FAILURE;
  }
  
  // Send response back to server
  nmPb_sendSrvrGetShortAddressReq( connection, &shortAddrCnf );
}

/**************************************************************************************************
 * @fn          nmPb_sendSrvrGetShortAddressReq
 *
 * @brief       Sends the confirmation back to the requesting server.
 *
 * @param       connection    - tcp connection
 * @param       pCnf  - pointer to confirmation structure
 *
 * @return      none
 **************************************************************************************************/
static void nmPb_sendSrvrGetShortAddressReq( int connection, SrvrGetShortAddressCnf *pCnf )
{
  int len;
  uint8 *pBuf;
  
  len = srvr_get_short_address_cnf__get_packed_size( pCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    srvr_get_short_address_cnf__pack( pCnf, pBuf );

    // Send response back to app (asynchronous data)
    APIS_SendData( connection, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, FALSE,
                   SRVR_CMD_ID_T__SRVR_GET_SHORT_ADDRESS_CNF, len, pBuf );
    
    free( pBuf );
  }
}

/**************************************************************************************************
 * @fn          nmPb_processSrvrGetDeviceInfoReq
 *
 * @brief       Handles incoming request from Gateway/OTA Server.
 *              Gets the device information of a device listed in the database.
 *
 * @param       connection    - tcp connection
 * @param       pDeviceInfoReq  - pointer to request structure
 *
 * @return      none
 **************************************************************************************************/
void nmPb_processSrvrGetDeviceInfoReq( int connection, SrvrGetDeviceInfoReq *pDeviceInfoReq )
{
  SrvrGetDeviceInfoCnf deviceInfoCnf = SRVR_GET_DEVICE_INFO_CNF__INIT;
  SrvrDeviceInfoT deviceInfo = SRVR_DEVICE_INFO_T__INIT;
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;
  
  uiPrintf( "\nSRVR_GET_DEVICE_INFO_REQ: ieeeAddr: %016llX\n", pDeviceInfoReq->ieeeaddress );
  
  pDeviceInfo = nwkMgrDb_GetDeviceInfo( pDeviceInfoReq->ieeeaddress );
  
  deviceInfoCnf.deviceinfo = &deviceInfo;
  
  if ( pDeviceInfo )
  {
    deviceInfoCnf.status = SRVR_STATUS_T__STATUS_SUCCESS;
    
    deviceInfo.networkaddress = pDeviceInfo->nwkAddr;
    deviceInfo.ieeeaddress = pDeviceInfo->ieeeAddr;
    deviceInfo.has_parentieeeaddress = TRUE;
    deviceInfo.parentieeeaddress = pDeviceInfo->parentAddr;
    deviceInfo.manufacturerid = pDeviceInfo->manufacturerId;
    deviceInfo.n_simpledesclist = pDeviceInfo->endpointCount;
    
    if ( pDeviceInfo->endpointCount )
    {
      deviceInfo.simpledesclist = nmPb_HandleServerSimpleDescPb( pDeviceInfo->endpointCount,
                                                                 pDeviceInfo->aEndpoint );
                                                                 
      if ( !deviceInfo.simpledesclist )
      {
        nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
        
        return; // memory failure
      }                                                                 
    }
    
    deviceInfo.devicestatus = pDeviceInfo->status;
  }
  else
  {
    deviceInfoCnf.status = SRVR_STATUS_T__STATUS_FAILURE;
  }
  
  // Send response back to server
  nmPb_sendSrvrGetDeviceInfoCnf( connection, &deviceInfoCnf );
  
  if ( deviceInfo.n_simpledesclist )
  {
    nmPb_FreeServerSimpleDescPb( deviceInfo.simpledesclist );
  }
  
  if ( pDeviceInfo )
  {
    nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
  }
}

/**************************************************************************************************
 * @fn          nmPb_sendSrvrGetDeviceInfoCnf
 *
 * @brief       Sends the confirmation back to the requesting server.
 *
 * @param       connection    - tcp connection
 * @param       pCnf  - pointer to confirmation structure
 *
 * @return      none
 **************************************************************************************************/
static void nmPb_sendSrvrGetDeviceInfoCnf( int connection, SrvrGetDeviceInfoCnf *pCnf )
{
  int len;
  uint8 *pBuf;
  
  len = srvr_get_device_info_cnf__get_packed_size( pCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    srvr_get_device_info_cnf__pack( pCnf, pBuf );

    // Send response back to app (asynchronous data)
    APIS_SendData( connection, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, FALSE,
                   SRVR_CMD_ID_T__SRVR_GET_DEVICE_INFO_CNF, len, pBuf );
    
    free( pBuf );
  }
}

/**************************************************************************************************
 * @fn          nmPb_processSrvrGetDeviceStatusReq
 *
 * @brief       Handles incoming request from Gateway/OTA Server.
 *              Gets the device status from the database.
 *
 * @param       connection    - tcp connection
 * @param       pDeviceStatusReq  - pointer to request structure
 *
 * @return      none
 **************************************************************************************************/
void nmPb_processSrvrGetDeviceStatusReq( int connection, SrvrGetDeviceStatusReq *pDeviceStatusReq )
{
  uint8 status;
  SrvrGetDeviceStatusCnf deviceStatusCnf = SRVR_GET_DEVICE_STATUS_CNF__INIT;
  
  uiPrintf( "\nSRVR_GET_DEVICE_STATUS_REQ: ieeeAddr: %016llX\n", pDeviceStatusReq->ieeeaddress );
  
  if ( nwkMgrDb_GetDeviceStatus( pDeviceStatusReq->ieeeaddress, &status ) )
  {
    deviceStatusCnf.status = SRVR_STATUS_T__STATUS_SUCCESS;
    deviceStatusCnf.devicestatus = status;
  }
  else
  {
    deviceStatusCnf.status = SRVR_STATUS_T__STATUS_FAILURE;
  }
  
  // Send response back to server
  nmPb_sendSrvrGetDeviceStatusCnf( connection, &deviceStatusCnf );
}

/**************************************************************************************************
 * @fn          nmPb_sendSrvrGetDeviceStatusCnf
 *
 * @brief       Sends the confirmation back to the requesting server.
 *
 * @param       connection    - tcp connection
 * @param       pCnf  - pointer to confirmation structure
 *
 * @return      none
 **************************************************************************************************/
static void nmPb_sendSrvrGetDeviceStatusCnf( int connection, SrvrGetDeviceStatusCnf *pCnf )
{
  int len;
  uint8 *pBuf;
  
  len = srvr_get_device_status_cnf__get_packed_size( pCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    srvr_get_device_status_cnf__pack( pCnf, pBuf );

    // Send response back to app (asynchronous data)
    APIS_SendData( connection, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, FALSE,
                   SRVR_CMD_ID_T__SRVR_GET_DEVICE_STATUS_CNF, len, pBuf );
    
    free( pBuf );
  }
}

/**************************************************************************************************
 * @fn          nmPb_processSrvrSetDeviceStatusReq
 *
 * @brief       Handles incoming request from Gateway/OTA Server.
 *              Sets the device status from the database.
 *
 * @param       connection    - tcp connection
 * @param       pDeviceStatusReq  - pointer to request structure
 *
 * @return      none
 **************************************************************************************************/
void nmPb_processSrvrSetDeviceStatusReq( int connection, SrvrSetDeviceStatusReq *pDeviceStatusReq )
{
  SrvrSetDeviceStatusCnf deviceStatusCnf = SRVR_SET_DEVICE_STATUS_CNF__INIT;
  
  uiPrintf( "\nSRVR_SET_DEVICE_STATUS_REQ: ieeeAddr: %016llX\n", pDeviceStatusReq->ieeeaddress );
  
  if ( nwkMgrDb_SetDeviceStatus( pDeviceStatusReq->ieeeaddress, (uint8)pDeviceStatusReq->devicestatus ) )
  {
    deviceStatusCnf.status = SRVR_STATUS_T__STATUS_SUCCESS;
  }
  else
  {
    deviceStatusCnf.status = SRVR_STATUS_T__STATUS_FAILURE;
  }
  
  // Send response back to server
  nmPb_sendSrvrSetDeviceStatusCnf( connection, &deviceStatusCnf );
}

/**************************************************************************************************
 * @fn          nmPb_sendSrvrSetDeviceStatusCnf
 *
 * @brief       Sends the confirmation back to the requesting server.
 *
 * @param       connection    - tcp connection
 * @param       pCnf  - pointer to confirmation structure
 *
 * @return      none
 **************************************************************************************************/
static void nmPb_sendSrvrSetDeviceStatusCnf( int connection, SrvrSetDeviceStatusCnf *pCnf )
{
  int len;
  uint8 *pBuf;
  
  len = srvr_set_device_status_cnf__get_packed_size( pCnf );
  pBuf = malloc( len );
  if ( pBuf )
  {
    srvr_set_device_status_cnf__pack( pCnf, pBuf );

    // Send response back to app (asynchronous data)
    APIS_SendData( connection, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, FALSE,
                   SRVR_CMD_ID_T__SRVR_SET_DEVICE_STATUS_CNF, len, pBuf );
    
    free( pBuf );
  }
}

/*********************************************************************
 * @fn      nmPb_HandleServerSimpleDescPb
 *
 * @brief   Stores the received simple descriptor information from the 
 *          database.
 *
 * @param   simpleDescCount - count of received simple descriptor information
 * @param   pEpInfo - pointer to array of simple descriptor information
 *
 * @return  TRUE if found, FALSE if failed
 */
static SrvrSimpleDescriptorT ** nmPb_HandleServerSimpleDescPb( int simpleDescCount, 
                                                               sNwkMgrDb_Endpoint_t *pEpInfo )
{
  int i;
  int j;
  int clusterCount = 0;  // total number of input/output clusters
  uint32 *pClusters;
  SrvrSimpleDescriptorT **ppSimpleDesc;
  SrvrSimpleDescriptorT  *pSimpleDesc;
  
  if ( !simpleDescCount )
  {
    return NULL;
  }
  
  // Count number of input/output clusters to be allocated
  for ( i = 0; i < simpleDescCount; i++ )
  {
    clusterCount += pEpInfo[i].inputClusterCount;
    clusterCount += pEpInfo[i].outputClusterCount;
  }
  
  // Allocate memory
  ppSimpleDesc = malloc( simpleDescCount * sizeof( SrvrSimpleDescriptorT * ) );
  if ( !ppSimpleDesc )
  {
    return NULL;  // memory error
  }
  
  pSimpleDesc = malloc( simpleDescCount * sizeof( SrvrSimpleDescriptorT ) );
  if ( !pSimpleDesc )
  {
    free( ppSimpleDesc );
    
    return NULL;  // memory error
  }
  
  pClusters = malloc( clusterCount * sizeof( uint32 ) );
  if ( !pClusters )
  {
    free( ppSimpleDesc );
    free( pSimpleDesc );
    
    return NULL;  // memory error
  }
  
  for ( i = 0; i < simpleDescCount; i++ )
  {
    ppSimpleDesc[i] = pSimpleDesc;
    
    srvr_simple_descriptor_t__init( pSimpleDesc );
    
    pSimpleDesc->endpointid = pEpInfo[i].endpointId;
    pSimpleDesc->profileid = pEpInfo[i].profileId;
    pSimpleDesc->deviceid = pEpInfo[i].deviceId;
    pSimpleDesc->devicever = pEpInfo[i].deviceVer;
    pSimpleDesc->n_inputclusters = pEpInfo[i].inputClusterCount;
    pSimpleDesc->inputclusters = pClusters;
    
    for ( j = 0; j < pEpInfo[i].inputClusterCount; j++ )
    {
      *pClusters++ = pEpInfo[i].inputClusters[j];
    }
    
    pSimpleDesc->n_outputclusters = pEpInfo[i].outputClusterCount;
    pSimpleDesc->outputclusters = pClusters;
    
    for ( j = 0; j < pEpInfo[i].outputClusterCount; j++ )
    {
      *pClusters++ = pEpInfo[i].outputClusters[j];
    }
    
    pSimpleDesc++;
  }
  
  return ppSimpleDesc;
}

/*********************************************************************
 * @fn      nmPb_FreeServerSimpleDescPb
 *
 * @brief   Frees the allocated simple descriptor information.
 *
 * @param   ppSimpleDesc - pointer to list of simple descriptor information
 *                         pointers
 *
 * @return  none
 */
static void nmPb_FreeServerSimpleDescPb( SrvrSimpleDescriptorT **ppSimpleDesc )
{
  if ( ppSimpleDesc[0]->n_inputclusters )
  {
    free( ppSimpleDesc[0]->inputclusters );
  }
  else if ( ppSimpleDesc[0]->n_outputclusters )
  {
    free( ppSimpleDesc[0]->outputclusters );
  }
  
  free( ppSimpleDesc[0] );
  free( ppSimpleDesc );
}


