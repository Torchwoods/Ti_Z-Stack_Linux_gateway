/**************************************************************************************************
 Filename:       nwkmgrdatabase.c
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

// enable unit test
//#define gNwkMgrDb_UnitTest_d    0

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <inttypes.h>

#include "hal_types.h"
#include "hal_defs.h"
#include "nwkmgr.pb-c.h"
#include "nwkmgrsrv.h"
#include "nwkmgrdatabase.h"
#include "SimpleDB.h"
#include "SimpleDBTxt.h"
#include "trace.h"

/*********************
  Types and Globals
**********************/

// for internal use
typedef struct sNwkMgrDb_IeeeAndEP_tag
{
  uint64_t  ieeeAddr;
  uint8     endpoint;
} sNwkMgrDb_IeeeAndEP_t;

// databases
void * gpNwkMgrDb_NetworkInfo;
void * gpNwkMgrDb_DeviceInfo;
void * gpNwkMgrDb_Endpoints;

// database headers
const char gszNwkMgrDb_DeviceInfoHeader[] =
  "Device Database\n"
  "IeeeAddress,NwkAddress,Status,MfgId,EP_Count,ParentIeeeAddress,CapInfo\n";
const char gszNwkMgrDb_EndpointsHeader[] =
  "Endpoint Database\n"
  "IeeeAddr,EndpointId,ProfileId,DeviceId,DeviceVer,NumIncluster,NumOutClusters,Clusters\n";

/*********************
  Local Prototypes
**********************/
static int nwkMgrDb_ComposeDeviceInfoRecord( char * pRec, sNwkMgrDb_DeviceInfo_t *pDeviceInfo );
static int nwmMgrDb_ComposeEndpointRecord( char *pRec, uint64_t ieeeAddr, sNwkMgrDb_Endpoint_t *pEndpoint );

static int nwkMgrDb_MatchDeviceIeeeAddr( void * record, void * key );
static int nwkMgrDb_MatchDeviceShortAddr( void * record, void * key );

static int nwkMgrDb_FillDevice( char *rec, sNwkMgrDb_DeviceInfo_t *pDeviceInfo );

/*********************
  Code
**********************/

/**************************************************************************************************
 *
 * @fn      nwkMgrDatabaseInit
 *
 * @brief   initialize the database for reading/writing. Database may already exist.
 *
 * @param   consolidate - consildate the databases.
 *
 * @return  TRUE for success, FALSE for failure
 *
 **************************************************************************************************/
bool nwkMgrDb_Init( bool consolidate )
{
  bool rc = TRUE;
  
  // open the Device Info database
  gpNwkMgrDb_DeviceInfo = sdb_init_db( (char *)gszDbDeviceInfoPath, 
                                       sdbtGetRecordSize, sdbtCheckDeleted, 
                                       sdbtCheckIgnored, sdbtMarkDeleted, 
                                       (consolidation_processing_f)sdbtErrorComment,
                                       SDB_TYPE_TEXT, strlen(gszNwkMgrDb_DeviceInfoHeader), 
                                       (char *)gszNwkMgrDb_DeviceInfoHeader);
                                       
  if ( gpNwkMgrDb_DeviceInfo )
  {
    uiPrintf( "Opened Database '%s', %d records\n", gszDbDeviceInfoPath, sdbtGetRecordCount( gpNwkMgrDb_DeviceInfo ) );
  }
  else
  {
    uiPrintf( "Failed to open Database '%s'\n", gszDbDeviceInfoPath );
    
    rc = FALSE;
  }
  
  // open the Endpoint Info database
  gpNwkMgrDb_Endpoints = sdb_init_db( (char *)gszDbEndpointsPath, 
                                      sdbtGetRecordSize, sdbtCheckDeleted, 
                                      sdbtCheckIgnored, sdbtMarkDeleted, 
                                      (consolidation_processing_f)sdbtErrorComment,
                                      SDB_TYPE_TEXT, strlen(gszNwkMgrDb_EndpointsHeader), 
                                      (char *)gszNwkMgrDb_EndpointsHeader);
                                      
  if ( gpNwkMgrDb_Endpoints )
  {
    uiPrintf( "Opened Database '%s', %d records\n", gszDbEndpointsPath, sdbtGetRecordCount( gpNwkMgrDb_Endpoints ) );
  }
  else
  {
    uiPrintf( "Failed to open Database '%s'\n",gszDbEndpointsPath );
    
    rc = FALSE;
  }

  // consolidate the databases
  if ( consolidate )
  {
    if ( (sdb_consolidate_db( &gpNwkMgrDb_DeviceInfo ) == FALSE) ||
         (sdb_consolidate_db( &gpNwkMgrDb_Endpoints ) == FALSE) )
    {
      uiPrintf( "\nFailed to consolidate Databases\n" );
      
      rc = FALSE;
    }
  }

  return rc;
}



/**************************************************************************************************
 *
 * @fn          nwkMgrDatabaseShutDown
 *
 * @brief       cleanly shut down the database.
 *
 * @return      none
 *
 **************************************************************************************************/
void nwkMgrDatabaseShutDown( void )
{
  sdb_release_db( &gpNwkMgrDb_DeviceInfo );
  sdb_release_db( &gpNwkMgrDb_Endpoints );
}

/**************************************************************************************************
 *
 * @fn          nwkMgrDatabaseReset
 *
 * @brief       Empty out the database.
 *
 * @return      none
 *
 **************************************************************************************************/
void nwkMgrDatabaseReset( void )
{
  int err;

  // Release database files
  nwkMgrDatabaseShutDown();
  
  err = remove( gszNwkMgrDb_DeviceInfo_c );
  if(err)
    uiPrintfEx(trDEBUG, "Failed to remove %s, %d\n", gszNwkMgrDb_DeviceInfo_c, err );
  err = remove( gszNwkMgrDb_Endpoints_c );
  if(err)
    uiPrintfEx(trDEBUG, "Failed to remove %s, %d\n", gszNwkMgrDb_Endpoints_c, err );
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_AddDevice
 *
 * @brief   add a device to the database. Must be filled out in the DeviceInfo structure. Also
 *          deletes old records (in case endpoints, etc... have changed)
 *
 * @param   pNwkInfo - pointer to a NwkInfo structure
 *
 * @return  TRUE if worked, FALSE if failed.
 *
 **************************************************************************************************/
bool nwkMgrDb_AddDevice( sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  int i;
  char szRecord[500];	// allows for maximum sized records
  bool worked;
  
  if(!pDeviceInfo)
    return FALSE;

  // mark old entry(ies) as deleted (in both databases)
  sdb_delete_records ( gpNwkMgrDb_DeviceInfo, &pDeviceInfo->ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );
  sdb_delete_records ( gpNwkMgrDb_Endpoints, &pDeviceInfo->ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );

  // add to Device Info Database
  nwkMgrDb_ComposeDeviceInfoRecord( szRecord, pDeviceInfo );
  worked = sdb_add_record( gpNwkMgrDb_DeviceInfo, szRecord );

  // add Endpoint(s) to Database
  for ( i = 0; worked && i < pDeviceInfo->endpointCount; ++i )
  {
    // add entry
    nwmMgrDb_ComposeEndpointRecord( szRecord, pDeviceInfo->ieeeAddr, &(pDeviceInfo->aEndpoint[i]) );
    worked = sdb_add_record( gpNwkMgrDb_Endpoints, szRecord );
  }

  return worked;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_RemoveDevice
 *
 * @brief   remove a device from the database
 *
 * @param   ieeeAddr - address of device to be removed
 *
 * @return  TRUE if worked, FALSE if failed.
 *
 **************************************************************************************************/
bool nwkMgrDb_RemoveDevice( uint64_t ieeeAddr )
{
  char * rec;
  bool found;

  // remove the device
  rec = sdb_delete_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );
  if ( !rec )
  {
    return FALSE;
  }

  // release the deleted DeviceInfo record
  sdb_release_record( (void **)(&rec) );

  do
  {
    found = FALSE;
    // delete all endpoints that match this ieeeAddr
    rec = sdb_delete_record( gpNwkMgrDb_Endpoints, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );
    
    if ( rec )
    {
      found = TRUE;
      sdb_release_record( (void **)(&rec) );
    }
  } while ( found );

  return TRUE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_FillDevice
 *
 * @brief   given a record that points to a device, find all endpoints records and fill it in.
 *          Assumes an uninitialized pDeviceInfo.
 *
 * @param   rec         - pointer to record containing device line from database
 * @param   pDeviceInfo - pointer to device record to be filled in
 *
 * @return  SDB_TXT_PARSER_RESULT_OK if worked, otherwise error
 *
 **************************************************************************************************/
static int nwkMgrDb_FillDevice( char *rec, sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  sNwkMgrDb_Endpoint_t *pEndpoint;
  uint8 ep;
  uint16 cluster;
  int i;
  int context;
  char *epRec;
  char *pBuf;
  uint64_t ieeeAddr;
  
  // return error if no record or device pointer
  if(!rec || !pDeviceInfo)
  {
    return SDB_TXT_PARSER_RESULT_REACHED_END_OF_RECORD;
  }
  
  // fill in device info
  // IeeeAddress,NwkAddress,Status,MfgId,EP_Count,ParentAddress
  sdb_txt_parser_get_uint64_field( &rec, &(pDeviceInfo->ieeeAddr), &parsingResult );
  sdb_txt_parser_get_numeric_field( &rec, &(pDeviceInfo->nwkAddr), 2, FALSE, &parsingResult );
  sdb_txt_parser_get_numeric_field( &rec, &(pDeviceInfo->status),  1, FALSE, &parsingResult );
  sdb_txt_parser_get_numeric_field( &rec, &(pDeviceInfo->manufacturerId), 2, FALSE, &parsingResult );
  sdb_txt_parser_get_numeric_field( &rec, &(pDeviceInfo->endpointCount),  1, FALSE, &parsingResult );
  sdb_txt_parser_get_uint64_field( &rec, &(pDeviceInfo->parentAddr), &parsingResult );
  
  // allocate endpoints, if we can't we can't continue parsing
  pDeviceInfo->aEndpoint = malloc(pDeviceInfo->endpointCount * sizeof( sNwkMgrDb_Endpoint_t ));
  if(!pDeviceInfo->aEndpoint)
  {
    return SDB_TXT_PARSER_RESULT_REACHED_END_OF_RECORD;
  }

  // fill in each endpoint
  context = 0;
  ieeeAddr = pDeviceInfo->ieeeAddr;
  for(ep = 0; ep < pDeviceInfo->endpointCount; ++ep)
  {
    // find the next EP record
    epRec = sdb_get_record( gpNwkMgrDb_Endpoints, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, &context );
    if( !epRec )
    {
      break;
    }

    // convenience variables
    pBuf = epRec;
    pEndpoint = &pDeviceInfo->aEndpoint[ep];

    // fill in endpoint info
    // IeeeAddr,EndpointId,ProfileId,DeviceId,DeviceVer,NumIncluster,NumOutClusters,Clusters
    sdb_txt_parser_get_uint64_field( &pBuf, NULL, &parsingResult );   // skip ieeeAddr
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->endpointId),  1, FALSE, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->profileId),   2, FALSE, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->deviceId),    2, FALSE, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->deviceVer),   1, FALSE, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->inputClusterCount),   1, FALSE, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, &(pEndpoint->outputClusterCount),  1, FALSE, &parsingResult );
    
    // allocate memory for input clusters. If it doesn't work, then we can't process this endpoint
    pEndpoint->inputClusters = malloc( pEndpoint->inputClusterCount * sizeof(uint16) );
    pEndpoint->outputClusters = malloc( pEndpoint->inputClusterCount * sizeof(uint16) );
    if(!pEndpoint->inputClusters || !pEndpoint->outputClusters)
    {
      break;
    }
      
    // get input clusters, if this list is short, then the parsing code will cause this to exit
    for ( i = 0; i < pEndpoint->inputClusterCount && !parsingResult.code; ++i )
    {
      sdb_txt_parser_get_numeric_field( &pBuf, &cluster, 2, FALSE, &parsingResult );
      pEndpoint->inputClusters[i] = cluster;
    }
    pEndpoint->inputClusterCount = i;   // in case loop broke early

    // get output clusters
    for ( i = 0; i < pEndpoint->outputClusterCount && !parsingResult.code; ++i )
    {
      sdb_txt_parser_get_numeric_field( &pBuf, &cluster, 2, FALSE, &parsingResult );
      pEndpoint->outputClusters[i] = cluster;
    }
    pEndpoint->outputClusterCount = i;  // in case loop broke early

    // done with record
    sdb_release_record( (void **)(&epRec) );

  } // end for loop
  
  pDeviceInfo->endpointCount = ep;  // just in case loop broke early
  
  if(parsingResult.code != 0)
  {
  }

  // return whether everything worked
  return parsingResult.code;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetDeviceInfo
 *
 * @brief   Allocate a device info from database
 *
 * @param   ieeeAddr    - address of device to get
 *
 * @return  allocated DeviceInfo, or NULL if can't find/allocate device
 *
 **************************************************************************************************/
sNwkMgrDb_DeviceInfo_t * nwkMgrDb_GetDeviceInfo( uint64_t ieeeAddr )
{
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;
  char * rec;
  int err;

  // allocate the structure
  pDeviceInfo = malloc( sizeof( sNwkMgrDb_DeviceInfo_t ) );
  if ( !pDeviceInfo )
  {
    return NULL;
  }
  
  // find the device record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, NULL );
  if ( !rec )
  {
    // free the device info
    free( pDeviceInfo );  
    return NULL;    // not found
  }

  // get the endpoints and fill in the device
  err = nwkMgrDb_FillDevice(rec, pDeviceInfo);
  sdb_release_record((void **)(&rec));
  
  if ( err )
  {
    // at least partially filled in, free substructures
    nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
    return NULL;
  }

  return pDeviceInfo;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_FreeDeviceInfo
 *
 * @brief   Free the device info substructures 
 *
 * @param   pDeviceInfo    - a device info structure
 *
 * @return  none
 *
 **************************************************************************************************/
static void nwkMgrDb_FreeDeviceInfoSubstructures( sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  int ep;

  // nothing to do, already free
  if( !pDeviceInfo )
    return;

  // free the endpoint/cluster lists
  if(pDeviceInfo->aEndpoint)
  {
    for(ep=0; ep<pDeviceInfo->endpointCount; ++ep)
    {
      if(pDeviceInfo->aEndpoint[ep].inputClusters)
      {
        free(pDeviceInfo->aEndpoint[ep].inputClusters);
      }
      if(pDeviceInfo->aEndpoint[ep].outputClusters)
      {
        free(pDeviceInfo->aEndpoint[ep].outputClusters);
      }
    }
    
    // free the endpoints
    free( pDeviceInfo->aEndpoint );
  }
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_FreeDeviceInfo
 *
 * @brief   Free the device info and its substructures 
 *
 * @param   pDeviceInfo    - a device info structure
 *
 * @return  none
 *
 **************************************************************************************************/
void nwkMgrDb_FreeDeviceInfo( sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  // nothing to do, already free
  if( !pDeviceInfo )
    return;

  nwkMgrDb_FreeDeviceInfoSubstructures( pDeviceInfo );
  free(pDeviceInfo);
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetAllDevices
 *
 * @brief   Get all devices from the device database
 *
 * @param   none
 *
 * @return  return a list to all devices. Returns NULL if failed.
 *
 **************************************************************************************************/
sNwkMgrDb_DeviceList_t * nwkMgrDb_GetAllDevices( void )
{
  int i;
  int err;
  char *rec;
  int context = 0;
  sNwkMgrDb_DeviceList_t *pDeviceList;
  
  pDeviceList = malloc( sizeof( sNwkMgrDb_DeviceList_t ) );
  if ( !pDeviceList )
  {
    return NULL;  // failed, not enough memory
  }

  // count # of valid records
  pDeviceList->count = sdbtGetRecordCount( gpNwkMgrDb_DeviceInfo );
  uiPrintf(" found %d Device Records\n", pDeviceList->count );
  if ( !pDeviceList->count )
  {
    pDeviceList->pDevices = NULL;
    return pDeviceList;
  }
  
  // allocate space for all the devices
  pDeviceList->pDevices = malloc( pDeviceList->count * sizeof( sNwkMgrDb_DeviceInfo_t ) );
  if ( !pDeviceList->pDevices )
  {
    free( pDeviceList );
    return NULL;  // failed, not enough memory
  }

  // populate with devices
  rec = sdb_get_first_record( gpNwkMgrDb_DeviceInfo, &context );
  err = SDB_TXT_PARSER_RESULT_OK;
  for ( i = 0; i < pDeviceList->count && rec && !err; ++i )
  {
    err = nwkMgrDb_FillDevice( rec, &(pDeviceList->pDevices[i]) );
    sdb_release_record( (void **)(&rec) );
    
    // get next device
    if ( !err )
    {
      rec = SDB_GET_NEXT_RECORD( gpNwkMgrDb_DeviceInfo, &context );
    }
  }
  
  // problem reading records, return NULL;
  if ( err )
  {
    free( pDeviceList->pDevices );
    free( pDeviceList );
  }

  return pDeviceList;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_FreeAllDevices
 *
 * @brief   Free the device list
 *
 * @param   pDeviceList - list of devices
 *
 * @return  none
 *
 **************************************************************************************************/
void nwkMgrDb_FreeAllDevices( sNwkMgrDb_DeviceList_t *pDeviceList )
{
  int i;
  if ( pDeviceList )
  {
    if ( pDeviceList->pDevices )
    {
      // free all devices
      for(i=0; i<pDeviceList->count; ++i)
      {
        nwkMgrDb_FreeDeviceInfoSubstructures( &pDeviceList->pDevices[i] );
      }
      // free the array of DeviceInfo
      free( pDeviceList->pDevices );
    }
    free( pDeviceList );
  }
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetShortAddr
 *
 * @brief   Get a short address, given an ieeeaddr
 *
 * @param   ieeeAddr - IEEE address of device
 * @param   pShortAddr - short address of device
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_GetShortAddr( uint64_t ieeeAddr, uint16 *pShortAddr )
{
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  char *rec;
  char *pBuf;

  // find the record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, NULL );
  if ( rec )
  {
    pBuf = rec;
    sdb_txt_parser_get_uint64_field ( &pBuf, NULL, &parsingResult );
    sdb_txt_parser_get_numeric_field( &pBuf, pShortAddr, 2, FALSE, &parsingResult );
    sdb_release_record( (void **)(&rec) );
    if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK )
    {
      return TRUE;
    }
    else
    {
      uiPrintf( "\n%s: Failed parsing the db record\n", __func__ );
    }
  }

  uiPrintf( "\n%s: cannot find matching db record\n", __func__ );
  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetParentAddr
 *
 * @brief   Get the parent, based on ieeeAddr
 *
 * @param   ieeeAddr - address of device
 * @param   pParentIeeeAddr - parent address of device
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_GetParentAddr( uint64_t ieeeAddr, uint64_t *pParentIeeeAddr )
{
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  char *rec;
  char *pBuf;

  // find the record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, NULL );
  if ( rec )
  {
    // "IeeeAddress,NwkAddress,Status,MfgId,EP_Count,ParentIeeeAddress\n"
    pBuf = rec;
    sdb_txt_parser_get_uint64_field ( &pBuf, NULL, &parsingResult );  // ieeeaddr
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 2, FALSE, &parsingResult );  // nwkaddr
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 1, FALSE, &parsingResult );  // status
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 2, FALSE, &parsingResult );  // mfgid
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 1, FALSE, &parsingResult );  // epcount
    sdb_txt_parser_get_uint64_field ( &pBuf, pParentIeeeAddr, &parsingResult);
    
    sdb_release_record( (void **)(&rec) );
    
    if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK )
    {
      uiPrintf( "Got parent %016llX of device %016llx\n", *pParentIeeeAddr, ieeeAddr );
      return TRUE;
    }
  }

  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetCapInfo
 *
 * @brief   Get capability information based on ieeeAddr
 *
 * @param   ieeeAddr - address of device
 * @param   pCapInfo - ptr to capability information to set
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_GetCapInfo( uint64_t ieeeAddr, uint8 *pCapInfo)
{
  // SJS_119, fix for Remove Device
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  char *rec;
  char *pBuf;

  // find the record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, NULL );
  if ( rec )
  {
    // "IeeeAddress,NwkAddress,Status,MfgId,EP_Count,ParentIeeeAddress\n"
    pBuf = rec;
    sdb_txt_parser_get_uint64_field ( &pBuf, NULL, &parsingResult );  // ieeeaddr
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 2, FALSE, &parsingResult );  // nwkaddr
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 1, FALSE, &parsingResult );  // status
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 2, FALSE, &parsingResult );  // mfgid
    sdb_txt_parser_get_numeric_field( &pBuf, NULL, 1, FALSE, &parsingResult );  // epcount
    sdb_txt_parser_get_uint64_field ( &pBuf, NULL, &parsingResult);       // parent
    sdb_txt_parser_get_numeric_field( &pBuf, pCapInfo, 1, FALSE, &parsingResult );  // capinfo
    
    sdb_release_record( (void **)(&rec) );
    
    if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK )
    {
      uiPrintf( "Got capinfo %02X of device %016llx\n", *pCapInfo, ieeeAddr );
      return TRUE;
    }
  }

  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_SetCapInfo
 *
 * @brief   Set the capability, based on ieeeAddr
 *
 * @param   ieeeAddr - address of device
 * @param   capInfo  - capability info
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_SetCapInfo( uint64_t ieeeAddr, uint8 capInfo)
{
  // SJS_119, fix for Remove Device
  bool worked;
  char szRecord[500];	// allows for maximum sized records
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;

  // retrieve all device information on this device
  pDeviceInfo = nwkMgrDb_GetDeviceInfo( ieeeAddr );
  
  // couldn't find or allocate record, do nothing
  if ( !pDeviceInfo )
  {
    return FALSE;
  }

  // update capability information
  pDeviceInfo->capInfo = capInfo;  

  // mark old entry as deleted
  sdb_delete_records ( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );

  // add to Device Info Database
  nwkMgrDb_ComposeDeviceInfoRecord( szRecord, pDeviceInfo );
  worked = sdb_add_record( gpNwkMgrDb_DeviceInfo, szRecord );

  nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
  
  return worked; 
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetIeeeAddr
 *
 * @brief   Get the ieeeAddr, based on shortAddr
 *
 * @param   shortAddr - short address of device
 * @param   pIeeeAddr - pointer to IEEE address of device
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_GetIeeeAddr( uint16 shortAddr, uint64_t *pIeeeAddr )
{
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  char *rec;
  char *pBuf;

  // find the record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &shortAddr, nwkMgrDb_MatchDeviceShortAddr, NULL );
  if ( rec )
  {
    pBuf = rec;
    sdb_txt_parser_get_uint64_field (&pBuf, pIeeeAddr, &parsingResult);

    sdb_release_record((void **)(&rec));
    if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK )
    {
      return TRUE;
    }
  }

  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_GetDeviceStatus
 *
 * @brief   Get the status, based on ieeeAddr
 *
 * @param   ieeeAddr - address of device
 * @param   pStatus - pointer to device status
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_GetDeviceStatus( uint64_t ieeeAddr, uint8 *pStatus )
{
  uint8 status;
  char *rec;
  char *pBuf;
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;

  // find the device record
  rec = sdb_get_record( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr, NULL );
  if ( !rec )
  {
    return FALSE;    // not found
  }

  // IeeeAddress,NwkAddress,Status
  pBuf = rec;
  sdb_txt_parser_get_uint64_field ( &pBuf, NULL, &parsingResult );  // ieeeaddr
  sdb_txt_parser_get_numeric_field( &pBuf, NULL, 2, FALSE, &parsingResult );  // nwkaddr
  sdb_txt_parser_get_numeric_field( &pBuf, &status, 1, FALSE, &parsingResult );  // status
  sdb_release_record( (void **)(&rec) );
  if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK )
  {
    uiPrintf( "Got status of device %016llX - status: %d\n", ieeeAddr, status );
    
    *pStatus = status;
    
    return TRUE;
  }

  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_SetDeviceStatus
 *
 * @brief   Set the status, based on ieeeAddr
 *
 * @param   ieeeAddr - address of device
 * @param   status - updated device status
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
bool nwkMgrDb_SetDeviceStatus( uint64_t ieeeAddr, uint8 status )
{
  bool worked;
  char szRecord[500];	// allows for maximum sized records
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;

  // retrieve all device information
  pDeviceInfo = nwkMgrDb_GetDeviceInfo( ieeeAddr );
  if( !pDeviceInfo )
  {
    return FALSE; // couldn't set
  }
  
  // check if status needs to be updated, if not, leave it alone
  if ( pDeviceInfo->status == status )
  {
    nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
    return TRUE;
  }

  pDeviceInfo->status = status;  // store updated status

  // mark old entry as deleted
  sdb_delete_records ( gpNwkMgrDb_DeviceInfo, &ieeeAddr, nwkMgrDb_MatchDeviceIeeeAddr );

  // add to Device Info Database
  nwkMgrDb_ComposeDeviceInfoRecord( szRecord, pDeviceInfo );
  worked = sdb_add_record( gpNwkMgrDb_DeviceInfo, szRecord );

  if ( worked )
  {
    NwkZigbeeDeviceInd devInd = NWK_ZIGBEE_DEVICE_IND__INIT;
    NwkDeviceInfoT *pDeviceInfoPb;

    uiPrintf( "Updated status of device %016llX to status: %d\n", ieeeAddr, status );

    // convert to protobuf native
    pDeviceInfoPb = allocPbDeviceInfoList( 1, pDeviceInfo );
    
    devInd.deviceinfo = pDeviceInfoPb;
    
    // notify apps up device status change
    sendNwkZigbeeDeviceInd( &devInd );
    
    freePbDeviceInfoList( 1, pDeviceInfoPb );
  }
  
  nwkMgrDb_FreeDeviceInfo( pDeviceInfo );
  
  return worked;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_MatchDeviceIeeeAddr
 *
 * @brief   Set the short address of a specific device
 *
 * @param   record - pointer to record
 * @param   key - pointer to IEEE address value to match
 *
 * @return  SDB_CHECK_KEY_EQUAL, or SDB_CHECK_KEY_NOT_EQUAL
 *
 **************************************************************************************************/
static int nwkMgrDb_MatchDeviceIeeeAddr( void * record, void * key )
{
  char * rec = record;
  uint64_t ieeeAddr = 0;
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;

  // IeeeAddress,NwkAddress,Status,MfgId,EP_Count
  sdb_txt_parser_get_uint64_field( &rec, &ieeeAddr, &parsingResult );

  if ( ieeeAddr == *(uint64_t *)key )  // matched
  {
    return SDB_CHECK_KEY_EQUAL;
  }

  return SDB_CHECK_KEY_NOT_EQUAL;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_MatchDeviceShortAddr
 *
 * @brief   Match the short address of a specific device
 *
 * @param   record - pointer to record
 * @param   key - pointer to short address value to match
 *
 * @return  SDB_CHECK_KEY_EQUAL, or SDB_CHECK_KEY_NOT_EQUAL
 *
 **************************************************************************************************/
static int nwkMgrDb_MatchDeviceShortAddr( void * record, void * key )
{
  char * rec = record;
  parsingResult_t parsingResult = SDB_INIT_PARSINGRESULT;
  uint16 shortAddr;
  uint16 *pShortAddrToMatch = key;

  // IeeeAddress,NwkAddress,Status,MfgId,EP_Count
  sdb_txt_parser_get_uint64_field( &rec, NULL, &parsingResult );
  sdb_txt_parser_get_numeric_field( &rec, &shortAddr, 2, FALSE, &parsingResult );

  if ( parsingResult.code == SDB_TXT_PARSER_RESULT_OK && (shortAddr == *pShortAddrToMatch) )
  {
    return SDB_CHECK_KEY_EQUAL;   // matched
  }
  
  return SDB_CHECK_KEY_NOT_EQUAL;
}

/**************************************************************************************************
 *
 * @fn      nwkMgrDb_ComposeDeviceInfoRecord
 *
 * @brief   Compose a text record from a DeviceInfo structure
 *
 * @param   pRec - pointer to text record buffer (output)
 * @param   pDeviceInfo - device structure (input)
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
static int nwkMgrDb_ComposeDeviceInfoRecord( char * pRec, sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  // find an delete the Device record if it currently exists

  // IeeeAddress,NwkAddress,Status,MfgId,EP_Count,ParentIeeeAddress
  sprintf( pRec, "        %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X , 0x%04X , 0x%02X, 0x%04X , %d, %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X, 0x%02X\n", 
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 7),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 6),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 5),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 4),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 3),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 2),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 1),
           BREAK_UINT64(pDeviceInfo->ieeeAddr, 0),
           pDeviceInfo->nwkAddr,
           pDeviceInfo->status,
           pDeviceInfo->manufacturerId,
           pDeviceInfo->endpointCount,
           BREAK_UINT64(pDeviceInfo->parentAddr, 7),
           BREAK_UINT64(pDeviceInfo->parentAddr, 6),
           BREAK_UINT64(pDeviceInfo->parentAddr, 5),
           BREAK_UINT64(pDeviceInfo->parentAddr, 4),
           BREAK_UINT64(pDeviceInfo->parentAddr, 3),
           BREAK_UINT64(pDeviceInfo->parentAddr, 2),
           BREAK_UINT64(pDeviceInfo->parentAddr, 1),
           BREAK_UINT64(pDeviceInfo->parentAddr, 0),
           pDeviceInfo->capInfo);

  return TRUE;
}

/**************************************************************************************************
 *
 * @fn      nwmMgrDb_ComposeEndpointRecord
 *
 * @brief   Compose a text record from an Endpoint structure
 *
 * @param   pRec - pointer to text record buffer (output)
 * @param   ieeeAddr - address of device (input)
 * @param   pEndpoint - endpoint structure (input)
 *
 * @return  TRUE if worked, FALSE if failed
 *
 **************************************************************************************************/
static int nwmMgrDb_ComposeEndpointRecord( char *pRec, uint64_t ieeeAddr, sNwkMgrDb_Endpoint_t *pEndpoint )
{
  int i;

  // IeeeAddr,EndpointId,ProfileId,DeviceId,DeviceVer,NumIncluster,NumOutClusters,Clusters
  pRec += sprintf( pRec, "        %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X , %d , 0x%04X, 0x%04X , %d , %d , %d",
                   BREAK_UINT64(ieeeAddr, 7),
                   BREAK_UINT64(ieeeAddr, 6),
                   BREAK_UINT64(ieeeAddr, 5),
                   BREAK_UINT64(ieeeAddr, 4),
                   BREAK_UINT64(ieeeAddr, 3),
                   BREAK_UINT64(ieeeAddr, 2),
                   BREAK_UINT64(ieeeAddr, 1),
                   BREAK_UINT64(ieeeAddr, 0),
                   pEndpoint->endpointId,
                   pEndpoint->profileId,
                   pEndpoint->deviceId,
                   pEndpoint->deviceVer,
                   pEndpoint->inputClusterCount,
                   pEndpoint->outputClusterCount );

  // add in each input clusters
  for ( i = 0; i < pEndpoint->inputClusterCount; ++i )
  {
    pRec += sprintf( pRec, " , 0x%04X", pEndpoint->inputClusters[i] );
  }

  // add in each output cluster
  for ( i = 0; i < pEndpoint->outputClusterCount; ++i )
  {
    pRec += sprintf( pRec, " , 0x%04X", pEndpoint->outputClusters[i] );
  }
  
  // finish the line
  sprintf( pRec, "\n" );

  return TRUE;
}

#if gNwkMgrDb_UnitTest_d


sNwkMgrDb_DeviceInfo_t aUnitTestDeviceGateway =
{
  0x00124B00012F7C88ULL,
  0x0000,
  devInfoFlags_OnLine_c | devInfoFlags_Local_c,
  0x1234,
  1,
  {
    {
      8,
      0x0104,
      0x0050,
      1,
      2,
      { 0x0000, 0x0003 },
      0,
      { 0 }
    },
    { 0 }, 
    { 0 }
  }
};

sNwkMgrDb_DeviceInfo_t aUnitTestDeviceOnOffSwitch =
{
  0x00124B0001020304ULL,
  0x1E53,
  devInfoFlags_OnLine_c,
  0x1234,
  1,
  {
    {
      8,
      0x0104,
      0x0103,
      1,
      2,
      { 0x0000, 0x0003 },
      1,
      { 0x0006 }
    },
    { 0 }, 
    { 0 }
  }
};

sNwkMgrDb_DeviceInfo_t aUnitTestDeviceOnOffLight =
{
  0x00124B00001020305ULL,
  0x230F,
  devInfoFlags_OnLine_c,
  0x1234,
  1,
  {
    {
      8,
      0x0104,
      0x0100,
      1,
      5,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006 },
      0,
      { 0 }
    },
    { 0 }, 
    { 0 }
  }
};

sNwkMgrDb_DeviceInfo_t aUnitTestDeviceDimmingLight =
{
  0x00124B00001020306ULL,
  0x5501,
  devInfoFlags_OffLine_c,
  0x1234,
  3,
  {
    {
      1,
      0x0104,
      0x0101,
      1,
      6,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006, 0x0008 },
      0,
      { 0 }
    },
    {
      2,
      0x0104,
      0x0101,
      1,
      6,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006, 0x0008 },
      0,
      { 0 }
    },
    {
      3,
      0x0104,
      0x0101,
      1,
      6,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006, 0x0008 },
      0,
      { 0 }
    }
  }

};

sNwkMgrDb_DeviceInfo_t aUnitTestDeviceColorLight =
{
  0x00124B00001020307ULL,
  0x4321,
  1,
  0x1234,
  devInfoFlags_OnLine_c,
  {
    {
      8,
      0x0104,
      0x0102,
      1,
      7,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006, 0x0800, 0x0300 },
      0,
      { 0 }
    },
    { 0 }, 
    { 0 }
  }
};

sNwkMgrDb_DeviceInfo_t aUnitTestDeviceTempSensor =
{
  0x00124B00001020308ULL,
  0x8E41,
  devInfoFlags_Sleepy_c,
  0x1e2f,
  1,
  {
    {
      8,
      0x0104,
      0x0302,
      1,
      5,
      { 0x0000, 0x0003, 0x0004, 0x0005, 0x0300 },
      2,
      { 0x0402, 0x0406 }
    },
    { 0 }, 
    { 0 }
  }
};

void UtExit( char *sz )
{
  uiPrintf( "%s\n",sz );
  exit( 1 );
}

void UtPrintDevice( sNwkMgrDb_DeviceInfo_t *pDeviceInfo )
{
  int ep;

  uiPrintf( "Found Device %016llX, %d endpoints: ", pDeviceInfo->ieeeAddr, pDeviceInfo->endpointCount );
  for( ep = 0; ep < pDeviceInfo->endpointCount; ++ep )
  {
    uiPrintf( "%d ", pDeviceInfo->aEndpoint[ep].endpointId );
  }
  
  uiPrintf( "\n" );
}

void NwkMgrDb_UnitTest( void )
{
  static const sNwkMgrDb_NetworkInfo_t sNwkInfo = { 1, 26, 0x1A3E, 0x1122334455667788ULL };
  sNwkMgrDb_DeviceList_t *pDeviceList;
  sNwkMgrDb_DeviceInfo_t *pDeviceInfo;
  int i;
  uint16 shortAddr;
  uint64_t ieeeAddr;

  uiPrintf( "---- Unit Test of Database Manager ----\n" );
  // open the database
  nwkMgrDb_Init( FALSE );
  
  if ( !nwkMgrDb_SetNetworkInfo( (sNwkMgrDb_NetworkInfo_t *)(&sNwkInfo) ) )
  {
    UtExit("Failed nwkMgrDb_SetNetworkInfo" );
  }
  
  uiPrintf( "Adding Gateway...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceGateway ) )
  {
    UtExit( "Error adding Gateway" );
  }
  
  uiPrintf( "Adding OnOffSwitch...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceOnOffSwitch ) )
  {
    UtExit( "Error adding OnOffSwitch" );
  }
  
  uiPrintf( "Adding OnOffLight...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceOnOffLight ) )
  {
    UtExit( "Error adding OnOffLight" );
  }
  
  uiPrintf( "Adding DimmingLight...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceDimmingLight ) )
  {
    UtExit( "Error Adding DimmingLight" );
  }
  
  uiPrintf( "Adding ColorLight...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceColorLight ) )
  {
    UtExit( "Error Adding ColorLight" );
  }
  
  uiPrintf( "Adding TempSensor...\n" );
  if ( !nwkMgrDb_AddDevice( &aUnitTestDeviceTempSensor ) )
  {
    UtExit( "Error adding TempSensor" );
  }  
  
  uiPrintf( "Consolidating NetworkInfo...\n" );
  if ( !sdb_consolidate_db( &gpNwkMgrDb_NetworkInfo ) )
  {
    UtExit( "Error consolidating NetworkInfo" );
  }

  uiPrintf( "Consolidating DeviceInfo...\n" );
  if ( !sdb_consolidate_db(&gpNwkMgrDb_DeviceInfo ) )
  {
    UtExit( "Error consolidating DeviceInfo" );
  }

  uiPrintf( "Consolidating Endpoints...\n" );
  if ( !sdb_consolidate_db(&gpNwkMgrDb_Endpoints ) )
  {
    UtExit( "Error consolidating Endpoints" );
  }
  
  // display all devices
  uiPrintf( "Getting All Devices...\n" );
  pDeviceList = nwkMgrDb_GetAllDevices();
  if ( !pDeviceList )
  {
    UtExit( "Error on nwkMgrDb_GetAllDevices" );
  }
  
  for ( i = 0; i < pDeviceList->count; ++i )
  {
    UtPrintDevice( &pDeviceList->pDevices[i] );
  }
  
  nwkMgrDb_FreeAllDevices( pDeviceList );
  
  uiPrintf( "Getting Device %016llx\n", aUnitTestDeviceColorLight.ieeeAddr );
  pDeviceInfo = nwkMgrDb_GetDeviceInfo( aUnitTestDeviceColorLight.ieeeAddr );
  if ( !pDeviceInfo )
  {
    UtExit( "Failed to Get Device\n" );
  }
  else
  {
    UtPrintDevice( pDeviceInfo );
  }
  
  nwkMgrDb_FreeDeviceInfo( pDeviceInfo );

  uiPrintf( "Getting non-existant device %016llX\n", 0x12345ULL );
  pDeviceInfo = nwkMgrDb_GetDeviceInfo( 0x12345ULL );
  if ( pDeviceInfo )
  {
    UtExit( "Should NOT have found device" );
  }
  
  uiPrintf( "Finding Short address of %016llX\n", aUnitTestDeviceTempSensor.ieeeAddr ); 
  if ( !nwkMgrDb_GetShortAddr( aUnitTestDeviceTempSensor.ieeeAddr, &shortAddr ) )
  {
    UtExit( "Failed to get shortAddr" );
  }
  else
  {
    uiPrintf( "shortAddr is %04X\n", shortAddr );
  }
  
  uiPrintf( "Finding ieeeAddr of %04X\n", aUnitTestDeviceDimmingLight.nwkAddr ); 
  if ( !nwkMgrDb_GetIeeeAddr( aUnitTestDeviceDimmingLight.nwkAddr, &ieeeAddr ) )
  {
    UtExit( "Failed to get ieeeAddr" );
  }
  else
  {
    uiPrintf( "ieeeAddr is %016llX\n", ieeeAddr );
  }
  
  // finds the local (gateway) device
  uiPrintf( "Finding local device\n" ); 
  if ( !nwkMgrDb_FindLocalDevice( &ieeeAddr, &shortAddr ) )
  {
    UtExit( "Failed to find local device" );
  }
  else
  {
    uiPrintf( "ieeeAddr is %016llX, shortAddr is %04X\n", ieeeAddr, shortAddr );
  }
  
  uiPrintf( "---- Unit Test Complete and Passed ----\n" );
}

int main ( int argc, char *argv[] )
{
  NwkMgrDb_UnitTest();
  return 0;
}

 
#endif // unit test

