/**************************************************************************************************
  Filename:       gatewayservices.h
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

#ifndef GATEWAYSERVICES_H
#define GATEWAYSERVICES_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "gateway.pb-c.h"
#include "server.pb-c.h"
#include "gatewaysrvr.h"

/*********************************************************************
 * MACROS
 */

#define GW_RETRY_TABLE_COUNT      10  // number to increment retry table by if needed
#define GW_TRANS_TABLE_COUNT      50  // number to increment transaction table by if needed
#define GW_ATTR_LIST_TABLE_COUNT  5   // increase table entry count by this amount each time
                                      // more space is needed

#define GW_SERVICES_SUCCESS         0   // state machine completed successfully
#define GW_SERVICES_FAILURE         1   // state machine had a failure
#define GW_SERVICES_MEM_ERROR       2   // state machine was not able to allocate memory
#define GW_SERVICES_BUSY            3   // state machine is busy with another process
#define GW_SERVICES_PROCESSING      4   // state machine is successfully working on next process
  
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

typedef struct gsGwMsgTransTable_tag 
{
  bool      inUse;
  bool      multRsp;     // checked when deleting a transaction expecting multiple responses
  int       rspCount;    // used to count responses on a transaction if multRsp is TRUE
  uint8     rspType;     // app response type: either GW_RSP_SPECIFIC or GW_RSP_GENERIC
  int       connection;  // app tcp connection value
  uint8     subSys;      // sub-system ID
  uint64_t  dstAddr;     // target IEEE address for the request command 
  uint8     cmdId;       // the command ID used for the transaction
  uint16    appTransId;  // App transaction ID
  uint8     zclTransId;  // ZCL transaction ID
  int       transTimer;  // countdown timer for removing entries
} gsGwMsgTransTable_t;

typedef struct gsGwMsgRetryTable_tag 
{
  bool      inUse;
  uint8     failedCount;  // used to count number of requests issued to remote device
  uint64_t  ieeeAddr;     // target IEEE address for the request command
} gsGwMsgRetryTable_t;

typedef struct sGwServices_AttrList_tag
{
  // May be more than one entry per cluster
  GwClusterListT  *pClusterList;  // structure that holds attribute list info
  void            *pNext;
} sGwServices_AttrList_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern int giGwServices_RetryTableCount;
extern int giGwServices_TransTableCount;
extern gsGwMsgTransTable_t *gpGwServices_TransTable;

/*********************************************************************
 * FUNCTIONS
 */

extern bool gwServices_FindPacketQueueEntry( uint64_t ieeeAddr );

extern bool gwServices_AddPacketPendingDevice( uint64_t ieeeAddr );

extern bool gwServices_FindPacketPendingDeviceEntry( uint64_t ieeeAddr );

extern bool gwServices_PollControlStart( afAddrType_t *pSrcAddr, uint64_t ieeeAddr );

extern bool gwServices_SetPacketQueueEntry( uint64_t ieeeAddr );

extern void gwServices_ClearPacketQueueEntry( uint64_t ieeeAddr );

extern uint8 gwServices_StartStateMachine_DeviceAttrList( int connection, SrvrDeviceInfoT *pDeviceInfo, 
                                                          GwGetDeviceAttributeListReq *pGetAttrListReq );
                                             
extern uint8 gwServices_UpdateStateMachine_DeviceAttrList( GwGetDeviceAttributeListRspInd *pGetAttrListRsp, 
                                                           uint16 appTransId, uint16 inClusterId, 
                                                           zclDiscoverAttrsRspCmd_t *pDiscAttrsRsp );  
                                                          
extern void gwServices_StateMachineTimeout_DeviceAttrList( uint64_t ieeeAddr ); 

extern GwClusterListT ** gwServices_ConvertPbClusterList( int attrCount, zclAttrRec_t *pAttrRecord,
                                                          uint8 inClusterCount, uint32 *pInClusterList,
                                                          uint32 *pClusterListCount );                                                                                            

extern void gwServices_FreePbClusterList( int clusterListCount, GwClusterListT **ppClusterList );

extern gsGwMsgRetryTable_t * gwServices_UpdateRetryTable( void );

extern gsGwMsgRetryTable_t * gwServices_GetDeviceInRetryTable( uint64_t ieeeAddr );

extern gsGwMsgTransTable_t * gwServices_UpdateTransTable( void );

/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* GATEWAYSERVICES_H */


