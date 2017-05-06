/**************************************************************************************************
  Filename:       gatewayp2p.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the peer-to-peer commands for the 
                  Gateway or other servers.

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
#include "zstack.pb-c.h"
#include "api_client.h"
#include "gatewayp2p.h"
#include "trace.h"

/**************************************************************************************************
 * Constants
 **************************************************************************************************/

#define GW_NWKMGR_HANDLE  giNwkMgrHandle


/**************************************************************************************************
 * Typedefs
 **************************************************************************************************/

/**************************************************************************************************
 * Function Prototypes
 **************************************************************************************************/
bool gwPb_SrvrGetIeeeAddress( uint16 shortAddr, uint64_t *pIeeeAddr );

bool gwPb_SrvrGetShortAddress( uint64_t ieeeAddr, uint16 *pShortAddr );

SrvrDeviceInfoT * gwPb_SrvrGetDeviceInfoReq( uint64_t ieeeAddr );
void gwPb_FreeSrvrGetDeviceInfo( SrvrDeviceInfoT *pDeviceInfo );
static SrvrSimpleDescriptorT ** gwPb_HandleServerSimpleDescPb( int simpleDescCount, 
                                                               SrvrSimpleDescriptorT **ppInSimpleDesc );
static void gwPb_FreeServerSimpleDescPb( SrvrSimpleDescriptorT **ppSimpleDesc );  

bool gwPb_SrvrGetDeviceStatus( uint64_t ieeeAddr, uint8 *pStatus );

bool gwPb_SrvrSetDeviceStatus( uint64_t ieeeAddr, uint8 status );                                                       
                                                        
/**************************************************************************************************
 * Locals and Globals
 **************************************************************************************************/

/**************************************************************************************************
 * Code
 **************************************************************************************************/

/*********************************************************************
 * @fn      gwPb_SrvrGetIeeeAddress
 *
 * @brief   Gets the IEEE address of a device in the database from the
 *          Network Manager Server.
 *
 * @param   shortAddr - short address of the device
 * @param   pIeeeAddr - short address of the device
 *
 * @return  TRUE if found, FALSE if not found/failure
 */
bool gwPb_SrvrGetIeeeAddress( uint16 shortAddr, uint64_t *pIeeeAddr )
{
  bool isSuccess;
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;
  int len;
  uint16 rspLen;
  SrvrGetIeeeAddressReq ieeeAddrReq = SRVR_GET_IEEE_ADDRESS_REQ__INIT;
  SrvrGetIeeeAddressCnf *pIeeeAddrCnf;
  
  ieeeAddrReq.shortaddress = shortAddr;
  
  len = srvr_get_ieee_address_req__get_packed_size( &ieeeAddrReq );
  pBuf = malloc( len );
  if ( !pBuf )
  {
    uiPrintf( "\nP2P: Get IEEE Address Failed - Memory Error\n" );
    
    return FALSE; // memory error
  }
  
  srvr_get_ieee_address_req__pack( &ieeeAddrReq, pBuf );
  
  // Send message to nwkmgr server
  pRsp = apicSendSynchData( GW_NWKMGR_HANDLE, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, 
                            SRVR_CMD_ID_T__SRVR_GET_IEEE_ADDRESS_REQ,
                            len, pBuf, NULL, &rspCmdId, &rspLen );
             
  free( pBuf );             
                            
  if ( pRsp )
  {
    if ( (rspLen > 0) && (rspCmdId == SRVR_CMD_ID_T__SRVR_GET_IEEE_ADDRESS_CNF) )
    {
      pIeeeAddrCnf = srvr_get_ieee_address_cnf__unpack( NULL, rspLen, pRsp );
      if ( pIeeeAddrCnf )
      {
        if ( pIeeeAddrCnf->status != SRVR_STATUS_T__STATUS_SUCCESS )
        {
          uiPrintf( "\nP2P: Get IEEE Address Failed, status: %d\n", pIeeeAddrCnf->status );
        
          isSuccess = FALSE;
        }
        else
        {
          *pIeeeAddr = pIeeeAddrCnf->ieeeaddress;
          
          isSuccess = TRUE;
        }
        
        srvr_get_ieee_address_cnf__free_unpacked( pIeeeAddrCnf, NULL );
      }
    }
    else
    {
      uiPrintf( "\nP2P: Get IEEE Address Failed\n" );
      uiPrintf( "\nExpected Get IEEE Address Confirmation, got CmdId: %d\n", rspCmdId );
    
      return FALSE;
    }  
    
    apicFreeSynchData( pRsp );
  }
      
  return isSuccess;                          
}

/*********************************************************************
 * @fn      gwPb_SrvrGetShortAddress
 *
 * @brief   Gets the Short address of a device in the database from the
 *          Network Manager Server.
 *
 * @param   ieeeAddr - IEEE address of the device
 * @param   pShortAddr - pointer to short address
 *
 * @return  TRUE if successful, FALSE if not found/failure
 */
bool gwPb_SrvrGetShortAddress( uint64_t ieeeAddr, uint16 *pShortAddr )
{
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;
  int len;
  uint16 rspLen;
  SrvrGetShortAddressReq shortAddrReq = SRVR_GET_SHORT_ADDRESS_REQ__INIT;
  SrvrGetShortAddressCnf *pShortAddrCnf;
  
  shortAddrReq.ieeeaddress = ieeeAddr;
  
  len = srvr_get_short_address_req__get_packed_size( &shortAddrReq );
  pBuf = malloc( len );
  if ( !pBuf )
  {
    uiPrintf( "\nP2P: Get Short Address Failed - Memory Error\n" );
    
    return FALSE; // memory error
  }
  
  srvr_get_short_address_req__pack( &shortAddrReq, pBuf );
  
  // Send message to nwkmgr server
  pRsp = apicSendSynchData( GW_NWKMGR_HANDLE, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, 
                            SRVR_CMD_ID_T__SRVR_GET_SHORT_ADDRESS_REQ,
                            len, pBuf, NULL, &rspCmdId, &rspLen );
                            
  free(pBuf);

  if ( pRsp )
  {
    if ( (rspLen > 0) && (rspCmdId == SRVR_CMD_ID_T__SRVR_GET_SHORT_ADDRESS_CNF) )
    {
      pShortAddrCnf = srvr_get_short_address_cnf__unpack( NULL, rspLen, pRsp );
      if ( pShortAddrCnf )
      {
        if ( pShortAddrCnf->status != SRVR_STATUS_T__STATUS_SUCCESS )
        {
          uiPrintf( "\nP2P: Get Short Address Failed, status: %d\n", pShortAddrCnf->status );
        }
        else
        {
          *pShortAddr = pShortAddrCnf->shortaddress;
        }
        
        srvr_get_short_address_cnf__free_unpacked( pShortAddrCnf, NULL );
      }
    }
    else
    {
      uiPrintf( "\nP2P: Get Short Address Failed\n" );
      uiPrintf( "\nExpected Get Short Address Confirmation, got CmdId: %d\n", rspCmdId );
    
      return FALSE;
    }  
    
    apicFreeSynchData( pRsp );
  }
      
  return TRUE;                          
}

/*********************************************************************
 * @fn      gwPb_SrvrGetDeviceInfoReq
 *
 * @brief   Gets the device information of a device in the database from the
 *          Network Manager Server. Must be freed by 
 *          gwPb_FreeSrvrGetDeviceInfo
 *
 * @param   ieeeAddr - IEEE address of the device
 *
 * @return  Pointer to device info structure, NULL if failure
 */
SrvrDeviceInfoT * gwPb_SrvrGetDeviceInfoReq( uint64_t ieeeAddr )
{
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;
  int len;
  uint16 rspLen;
  SrvrDeviceInfoT *pDeviceInfo = NULL;
  SrvrGetDeviceInfoReq deviceInfoReq = SRVR_GET_DEVICE_INFO_REQ__INIT;
  SrvrGetDeviceInfoCnf *pDeviceInfoCnf;
  
  deviceInfoReq.ieeeaddress = ieeeAddr;
  
  len = srvr_get_device_info_req__get_packed_size( &deviceInfoReq );
  pBuf = malloc( len );
  if ( !pBuf )
  {
    uiPrintf( "\nP2P: Get Device Information Failed - Memory Error\n" );
    
    return NULL; // memory error
  }
  
  srvr_get_device_info_req__pack( &deviceInfoReq, pBuf );
  
  // Send message to nwkmgr server
  pRsp = apicSendSynchData( GW_NWKMGR_HANDLE, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, 
                            SRVR_CMD_ID_T__SRVR_GET_DEVICE_INFO_REQ,
                            len, pBuf, NULL, &rspCmdId, &rspLen );
                            
  free( pBuf ); 
                   
  if ( pRsp )
  {
    if ( (rspLen > 0) && (rspCmdId == SRVR_CMD_ID_T__SRVR_GET_DEVICE_INFO_CNF) )
    {
      pDeviceInfoCnf = srvr_get_device_info_cnf__unpack( NULL, rspLen, pRsp );
      if ( pDeviceInfoCnf )
      {
        // Check the return status
        if ( pDeviceInfoCnf->status != SRVR_STATUS_T__STATUS_SUCCESS )
        {
          uiPrintf( "\nP2P: Get Device Information Failed, status: %d\n", pDeviceInfoCnf->status );
        }
        else
        {
          pDeviceInfo = malloc( sizeof( SrvrDeviceInfoT ) );
          if ( !pDeviceInfo )
          {
            return NULL;  // memory error
          }  
      
          srvr_device_info_t__init( pDeviceInfo );
      
          // Fill in device information
          pDeviceInfo->networkaddress = pDeviceInfoCnf->deviceinfo->networkaddress;
          pDeviceInfo->ieeeaddress = pDeviceInfoCnf->deviceinfo->ieeeaddress;
          pDeviceInfo->has_parentieeeaddress = pDeviceInfoCnf->deviceinfo->has_parentieeeaddress;
          pDeviceInfo->parentieeeaddress = pDeviceInfoCnf->deviceinfo->parentieeeaddress;
          pDeviceInfo->manufacturerid = pDeviceInfoCnf->deviceinfo->manufacturerid;
          pDeviceInfo->n_simpledesclist = pDeviceInfoCnf->deviceinfo->n_simpledesclist;
          pDeviceInfo->devicestatus = pDeviceInfoCnf->deviceinfo->devicestatus;
          
          if ( pDeviceInfo->n_simpledesclist )
          {
            // Fill in simple descriptor
            pDeviceInfo->simpledesclist = gwPb_HandleServerSimpleDescPb( pDeviceInfoCnf->deviceinfo->n_simpledesclist,
                                                                         pDeviceInfoCnf->deviceinfo->simpledesclist );
              
            if ( !pDeviceInfo->simpledesclist )
            {
              srvr_get_device_info_cnf__free_unpacked( pDeviceInfoCnf, NULL );  // free response structure
              apicFreeSynchData( pRsp );  // free response message
              free( pDeviceInfo );  // free returned message
              
              uiPrintf( "\nP2P: Get Device Information Failed - Memory Error\n" );
              
              return NULL; // memory error
            }
          }
        }
        
        srvr_get_device_info_cnf__free_unpacked( pDeviceInfoCnf, NULL );  // free response structure
      }
      else
      {
        apicFreeSynchData( pRsp );  // free response message
        free( pDeviceInfo );  // free returned message
        
        uiPrintf( "\nP2P: Get Device Information Failed - Memory Error\n" );
              
        return NULL; // memory error
      }
    }
    else
    {
      apicFreeSynchData( pRsp );  // free response message
      
      uiPrintf( "\nP2P: Get Device Information Failed\n" );
      uiPrintf( "\nExpected Get Device Information Confirmation, got CmdId: %d\n", rspCmdId );

      return NULL;
    }  
    
    apicFreeSynchData( pRsp );  // free response message
  }
  else
  {
    return NULL;
  }
      
  return pDeviceInfo;                          
}

/*********************************************************************
 * @fn      gwPb_FreeSrvrGetDeviceInfo
 *
 * @brief   Frees the allocated device information.
 *
 * @param   pDeviceInfo - Pointer to device info structure
 *
 * @return  none
 */
void gwPb_FreeSrvrGetDeviceInfo( SrvrDeviceInfoT *pDeviceInfo )
{  
  if ( pDeviceInfo )
  {
    // Free the simple descriptor
    if ( pDeviceInfo->simpledesclist)
    {
      if( pDeviceInfo->n_simpledesclist == 0 )
      {
        uiPrintf(" n_simpledesclist mismatch\n");
      }

      gwPb_FreeServerSimpleDescPb( pDeviceInfo->simpledesclist );
    }
    free( pDeviceInfo );
  }
}

/*********************************************************************
 * @fn      gwPb_HandleServerSimpleDescPb
 *
 * @brief   Stores the received simple descriptor information from the 
 *          Network Manager Server.
 *
 * @param   simpleDescCount - count of received simple descriptor information
 * @param   pInSimpleDesc - pointer to list of received simple descriptor information
 *                          pointers
 *
 * @return  TRUE if found, FALSE if failed
 */
static SrvrSimpleDescriptorT ** gwPb_HandleServerSimpleDescPb( int simpleDescCount, 
                                                               SrvrSimpleDescriptorT **ppInSimpleDesc )
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
    clusterCount += ppInSimpleDesc[i]->n_inputclusters;
    clusterCount += ppInSimpleDesc[i]->n_outputclusters;
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
    
    pSimpleDesc->endpointid = ppInSimpleDesc[i]->endpointid;
    pSimpleDesc->profileid = ppInSimpleDesc[i]->profileid;
    pSimpleDesc->deviceid = ppInSimpleDesc[i]->deviceid;
    pSimpleDesc->devicever = ppInSimpleDesc[i]->devicever;
    pSimpleDesc->n_inputclusters = ppInSimpleDesc[i]->n_inputclusters;
    if (pSimpleDesc->n_inputclusters == 0)
    {
      pSimpleDesc->inputclusters = NULL;
    }
    else
    {
      pSimpleDesc->inputclusters = pClusters;
      for ( j = 0; j < ppInSimpleDesc[i]->n_inputclusters; j++ )
      {
        *pClusters++ = ppInSimpleDesc[i]->inputclusters[j];  
      }
    }
    pSimpleDesc->n_outputclusters = ppInSimpleDesc[i]->n_outputclusters;
    if (pSimpleDesc->n_outputclusters == 0)
    {
      pSimpleDesc->outputclusters = NULL;
    }
    else
    {
      pSimpleDesc->outputclusters = pClusters;

      for ( j = 0; j < ppInSimpleDesc[i]->n_outputclusters; j++ )
      {
        *pClusters++ = ppInSimpleDesc[i]->outputclusters[j];                      
      }
    }

    pSimpleDesc++;
  }
  
  return ppSimpleDesc;
}

/*********************************************************************
 * @fn      gwPb_FreeServerSimpleDescPb
 *
 * @brief   Frees the allocated simple descriptor information from the 
 *          Network Manager Server.
 *
 * @param   ppSimpleDesc - pointer to list of simple descriptor information
 *                         pointers
 *
 * @return  none
 */
static void gwPb_FreeServerSimpleDescPb( SrvrSimpleDescriptorT **ppSimpleDesc )
{
  
  // count { array of ptrs, pointers ptr to something }
  // 
  if( ppSimpleDesc && ppSimpleDesc[0] )
  {
    // always allocated in one large array for all endpoints.
    // No need to free individual pointers from the list of pointers
    if ( ppSimpleDesc[0]->inputclusters )
    {
      if( ppSimpleDesc[0]->n_inputclusters == 0)
        uiPrintf(" Mismatch n_inputclusters\n");
      free( ppSimpleDesc[0]->inputclusters );
    }
    else if ( ppSimpleDesc[0]->outputclusters )
    {
      if( ppSimpleDesc[0]->n_outputclusters == 0)
        uiPrintf(" Mismatch n_outputclusters\n");
      free( ppSimpleDesc[0]->outputclusters );
    }

    free( ppSimpleDesc[0] );
  }

  if( ppSimpleDesc )
  {
    free( ppSimpleDesc );
  }
}

/*********************************************************************
 * @fn      gwPb_SrvrGetDeviceStatus
 *
 * @brief   Gets device status from the database
 *
 * @param   ieeeAddr - IEEE address of device
 * @param   pStatus - pointer to the returned status
 *
 * @return  TRUE for success, FALSE for failure
 */
bool gwPb_SrvrGetDeviceStatus( uint64_t ieeeAddr, uint8 *pStatus )
{
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;
  int len;
  uint16 rspLen;
  SrvrGetDeviceStatusReq deviceStatusReq = SRVR_GET_DEVICE_STATUS_REQ__INIT;
  SrvrGetDeviceStatusCnf *pDeviceStatusCnf;
  
  deviceStatusReq.ieeeaddress = ieeeAddr;
  
  len = srvr_get_device_status_req__get_packed_size( &deviceStatusReq );
  pBuf = malloc( len );
  if ( !pBuf )
  {
    uiPrintf( "\nP2P: Get Device Status Failed - Memory Error\n" );
    
    return FALSE; // memory error
  }
  
  srvr_get_device_status_req__pack( &deviceStatusReq, pBuf );
  
  // Send message to nwkmgr server
  pRsp = apicSendSynchData( GW_NWKMGR_HANDLE, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, 
                            SRVR_CMD_ID_T__SRVR_GET_DEVICE_STATUS_REQ,
                            len, pBuf, NULL, &rspCmdId, &rspLen );
        
  free( pBuf );        
                            
  if ( pRsp )
  {
    if ( (rspLen > 0) && (rspCmdId == SRVR_CMD_ID_T__SRVR_GET_DEVICE_STATUS_CNF) )
    {
      pDeviceStatusCnf = srvr_get_device_status_cnf__unpack( NULL, rspLen, pRsp );
      if ( pDeviceStatusCnf )
      {
        if ( pDeviceStatusCnf->status != SRVR_STATUS_T__STATUS_SUCCESS )
        {
          uiPrintf( "\nP2P: Get Device Status Failed, status: %d\n", pDeviceStatusCnf->status );
        }
        else
        {
          *pStatus = pDeviceStatusCnf->devicestatus;
        }
        
        srvr_get_device_status_cnf__free_unpacked( pDeviceStatusCnf, NULL );
      }
    }
    else
    {
      uiPrintf( "\nP2P: Get Device Status Failed\n" );
      uiPrintf( "\nExpected Get Device Status Confirmation, got CmdId: %d\n", rspCmdId );
    
      return FALSE;
    }  
    
    apicFreeSynchData( pRsp );
  }
      
  return TRUE; 
}

/*********************************************************************
 * @fn      gwPb_SrvrSetDeviceStatus
 *
 * @brief   Sets device status from the database
 *
 * @param   ieeeAddr - IEEE address of device
 * @param   status - status of device
 *
 * @return  TRUE for success, FALSE for failure
 */
bool gwPb_SrvrSetDeviceStatus( uint64_t ieeeAddr, uint8 status )
{
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;
  int len;
  uint16 rspLen;
  SrvrSetDeviceStatusReq deviceStatusReq = SRVR_SET_DEVICE_STATUS_REQ__INIT;
  SrvrSetDeviceStatusCnf *pDeviceStatusCnf;
  
  deviceStatusReq.ieeeaddress = ieeeAddr;
  deviceStatusReq.devicestatus = status;
  
  len = srvr_set_device_status_req__get_packed_size( &deviceStatusReq );
  pBuf = malloc( len );
  if ( !pBuf )
  {
    uiPrintf( "\nP2P: Get Device Status Failed - Memory Error\n" );
    
    return FALSE; // memory error
  }
  
  srvr_set_device_status_req__pack( &deviceStatusReq, pBuf );
  
  // Send message to nwkmgr server
  pRsp = apicSendSynchData( GW_NWKMGR_HANDLE, Z_STACK_SERVER_SYS_ID_T__RPC_SYS_PB_SRVR, 
                            SRVR_CMD_ID_T__SRVR_SET_DEVICE_STATUS_REQ,
                            len, pBuf, NULL, &rspCmdId, &rspLen );
     
  free( pBuf );     
                            
  if ( pRsp )
  {
    if ( (rspLen > 0) && (rspCmdId == SRVR_CMD_ID_T__SRVR_SET_DEVICE_STATUS_CNF) )
    {
      pDeviceStatusCnf = srvr_set_device_status_cnf__unpack( NULL, rspLen, pRsp );
      if ( pDeviceStatusCnf )
      {
        if ( pDeviceStatusCnf->status != SRVR_STATUS_T__STATUS_SUCCESS )
        {
          uiPrintf( "\nP2P: Get Device Status Failed, status: %d\n", pDeviceStatusCnf->status );
        }
        
        srvr_set_device_status_cnf__free_unpacked( pDeviceStatusCnf, NULL );
      }
    }
    else
    {
      uiPrintf( "\nP2P: Get Device Status Failed\n" );
      uiPrintf( "\nExpected Get Device Status Confirmation, got CmdId: %d\n", rspCmdId );
    
      return FALSE;
    }  
    
    apicFreeSynchData( pRsp );
  }
      
  return TRUE;  
}

