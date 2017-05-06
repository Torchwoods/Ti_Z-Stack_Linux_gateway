/**************************************************************************************************
 Filename:       zigbeenwkservices.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:    This file contains the public interface for the ZigBee Network Manager Services.

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
#ifndef _zigbeenwkservices_h_
#define _zigbeenwkservices_h_  1

// requires Network Manager Database types and calls
#include "nwkmgrdatabase.h"
#include "zstack.pb-c.h"

#define NWKSRV_RETRY_TABLE_COUNT   16  // number to increment retry table by if needed

// how many state machines can be in progress at once?
#define NWKSRV_ADD_STATE_MACHINES  16   // how many to add each time?

// state machine errors (for callback)
#define NS_ADS_OK       0
#define NS_ADS_NO_RSP   1   // one or more of the zigbee calls did not respond
#define NS_ADS_NO_MEM   2   // couldn't allocate structures or state machine
#define NS_ADS_BUSY     3   // already busy on this device ID

typedef struct gsNmMsgRetryTable_tag 
{
  bool      inUse;
  uint8     failedCount;  // used to count number of requests issued to remote device
  uint64_t  ieeeAddr;     // target IEEE address for the request command
} gsNmMsgRetryTable_t;

/***************************************************
  Network Manager Subsystem
****************************************************/

// function that will be called when the state machine is complete. After this function is called, pDeviceInfo is freed.
typedef void (*pfnZNwkSrvAddDeviceCB_t)( uint16 appSeq, sNwkMgrDb_DeviceInfo_t *pDeviceInfo, int err );

// add this device to the database, getting all info about the device (endpoints, etc...)
void zNwkSrv_AddDevice( uint64_t ieeeAddr, ZdoTcDeviceInd *tcInfo, uint16 appSeq, pfnZNwkSrvAddDeviceCB_t pfnZNwkSrvAddDeviceCB );

// will clean information for the state machine
void zNwkSrv_UpdateAddDeviceStateMachine( ZStackCmdIDs cmdId, void *pNativePbData );

// finds any running state machines for a given IeeeAddr and stops it
void zNwkSrv_AD_HaltStateMachine( uint64_t ieeeAddr );

// process device announce, update service discovery state machine
void zNwkSrv_AD_ProcessDeviceAnnounce( uint64_t ieeeAddr );

// updates any state machine timers
void zNwkSrv_UpdateTimers( void );

extern gsNmMsgRetryTable_t * zNwkSrv_UpdateRetryTable( void );

extern gsNmMsgRetryTable_t * zNwkSrv_GetDeviceInRetryTable( uint64_t ieeeAddr );

extern bool zNwkSrv_AD_StateMachineExists( uint64_t ieeeAddr );

// SJS_119, fix for RemoveDevice
extern bool zNwkSrv_AD_UpdateCapInfo( uint64_t ieeeAddr, uint8 capInfo );

#endif // _zigbeeservices_h_
