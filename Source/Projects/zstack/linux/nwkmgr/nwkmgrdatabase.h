/**************************************************************************************************
 Filename:       nwkmgrdatabase.h
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
#ifndef _nwkmgrdatabase_h_
#define _nwkmgrdatabase_h_ 1


/*************************************
  Types and defines
**************************************/

// define the set of database files
#define gszNwkMgrDb_DeviceInfo_c     "DbDeviceInfo.csv"
#define gszNwkMgrDb_Endpoints_c      "DbEndpoints.csv"

// the database records get their ieeeAddr from the DeviceInfo
typedef struct sNwkMgrDb_Endpoint_tag
{
  uint8  endpointId;
  uint16 profileId;
  uint16 deviceId;
  uint8  deviceVer;
  uint8  inputClusterCount;
  uint16 *inputClusters;
  uint8  outputClusterCount;
  uint16 *outputClusters;
} sNwkMgrDb_Endpoint_t;

// must be either online or offline
typedef uint8 devInfoFlags_t;
#define devInfoFlags_OffLine_c 0x00
#define devInfoFlags_OnLine_c  0x01

typedef struct sNwkMgrDb_DeviceInfo_tag
{
  uint64_t ieeeAddr;
  uint16   nwkAddr;
  devInfoFlags_t status;
  uint16   manufacturerId;
  uint8    endpointCount;
  uint64_t parentAddr;
  uint8    capInfo;       // SJS_119, fix for RemoveDevice
  sNwkMgrDb_Endpoint_t *aEndpoint;
} sNwkMgrDb_DeviceInfo_t;

typedef struct sNwkMgrDb_DeviceList_tag
{
  uint16    count;
  sNwkMgrDb_DeviceInfo_t *pDevices;
} sNwkMgrDb_DeviceList_t;

// called once upon init for each app (only the Network Server should consolidate)
bool nwkMgrDb_Init( bool consolidate );

// cleanly shut down the database
void nwkMgrDatabaseShutDown( void );

// clears the database... now empty
void nwkMgrDatabaseReset( void );

// add a device to the datagbase. Must be filled out. Returns TRUE if added
bool nwkMgrDb_AddDevice( sNwkMgrDb_DeviceInfo_t *pDeviceInfo );

//  Remove a device from the database. Removes all of its endpoints, attributes and deviceinfo. Returns TRUE if removed.
bool nwkMgrDb_RemoveDevice( uint64_t ieeeAddr );

//  Allocates and returns DeviceInfo structure if found in the database. Returns NULL if not found.
sNwkMgrDb_DeviceInfo_t * nwkMgrDb_GetDeviceInfo( uint64_t ieeeAddr ); 
void nwkMgrDb_FreeDeviceInfo( sNwkMgrDb_DeviceInfo_t *pDeviceInfo );

// returns a pointer to the list of all devices in the database. Returns NULL if can't allocate memory.
sNwkMgrDb_DeviceList_t * nwkMgrDb_GetAllDevices( void );
void nwkMgrDb_FreeAllDevices( sNwkMgrDb_DeviceList_t *pDeviceList );

// Get the short address of a device from the database given its ieeeAddr. Returns TRUE if found.
bool nwkMgrDb_GetShortAddr( uint64_t ieeeAddr, uint16 *pShortAddr );

// Get the parent address of a device from the database given its ieeeAddr. Returns TRUE if found.
bool nwkMgrDb_GetParentAddr( uint64_t ieeeAddr, uint64_t *pParentIeeeAddr );

// returns the ieeeAddr of a device from the database given its short address. Returns TRUE if found.
bool nwkMgrDb_GetIeeeAddr( uint16 shortAddr, uint64_t *pIeeeAddr );

// Capability info
#define gCapInfo_Coord_c        0x01
#define gCapInfo_Router_c       0x02
#define gCapInfo_MainsPowered_c 0x04
#define gCapInfo_RxOnIdle_c     0x08
#define gCapInfo_Secure_c       0x40

// get capability info of an existing database record
bool nwkMgrDb_GetCapInfo( uint64_t ieeeAddr, uint8 *pCapInfo);

// set capability info for an existing database record
bool nwkMgrDb_SetCapInfo( uint64_t ieeeAddr, uint8 capInfo);

// gets the status of a device in the database
bool nwkMgrDb_GetDeviceStatus( uint64_t ieeeAddr, uint8 *pStatus );

// sets the status of a device in the database
bool nwkMgrDb_SetDeviceStatus( uint64_t ieeeAddr, uint8 status );

#endif // _nwkmgrdatabase_h_

