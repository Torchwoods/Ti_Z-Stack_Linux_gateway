/**************************************************************************************************
  Filename:       gatewaysrvr.h
  Revised:        $Date:  $
  Revision:       $Revision:  $

  Description:    This file contains the linux HA gateway server definitions


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

#ifndef GATEWAYSRVR_H
#define GATEWAYSRVR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "serverep.h"
#include "zstack.pb-c.h"
#include "gateway.pb-c.h"
#include "api_client.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_closures.h"
#include "zcl_poll_control.h"

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
extern apicHandle_t giZStackHandle;
#define GW_ZSTACK_HANDLE  giZStackHandle

extern apicHandle_t giNwkMgrHandle;
#define GW_NWKMGR_HANDLE  giNwkMgrHandle

#define GW_EP     2


// Poll Control Check-In Response global
#define FAST_POLL_TIMEOUT         120   // 30 second timeout (in quarterseconds)


#define ADDR_MODE_UNICAST     1
#define ADDR_MODE_GROUPCAST   2
#define ADDR_MODE_BROADCAST   4
#define ADDR_MODE_NONE        8 // used for self-addressing

#define NWK_ROUTE_RADIUS      5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern void zclProcessInCmds( zclIncoming_t *pCmd );

extern ZStatusValues sendAPICExpectDefaultStatus( int cmdID, int len, uint8 *pData );

extern ZStatus_t sendGwGetDeviceAttributeListReq( int connection, GwGetDeviceAttributeListReq *pGetDeviceAttrListReq, uint16 clusterId, uint16 startAttr );

extern void sendGwGetDeviceAttributeListRspInd( int connection, GwGetDeviceAttributeListRspInd *pGetAttrListRsp );

extern ZStatus_t gwGroupsClusterIndCB( zclGroupRsp_t *pRsp );

extern ZStatus_t gwScenesClusterIndCB( zclSceneRsp_t *pRsp );

extern ZStatus_t gwAlarmsClusterIndCB( zclAlarm_t *pAlarm );

extern ZStatus_t gwZclPollControlCheckInCB( zclPollControlCheckIn_t *pCmd );

extern ZStatus_t gwZclDoorLockRspCB( zclIncoming_t *pInMsg, uint8 status );

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATEWAYSRVR_H */
