/**************************************************************************************************
  Filename:       aps_groups.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    Application Support Sub Layer group management functions.


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

  /*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zstack.pb-c.h"
#include "hal_rpc.h"
#include "aps_groups.h"
#include "api_client.h"
#include "gatewaysrvr.h"

#include "zcl.h"
#include "zcl_general.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
aps_Group_t gFindGroupRsp;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**************************************************************************************************
 **************************************************************************************************/

#ifdef ZCL_GROUPS
/*********************************************************************
 * @fn      aps_AddGroup
 *
 * @brief   Process in the received Add Group Command.
 *
 * @param   endpoint - application endpoint
 * @param   pGroup - includes group name and group ID
 *
 * @return  ZStatus_t
 */
ZStatus_t aps_AddGroup( uint8 endpoint, aps_Group_t *pGroup )
{
  int len;
  uint8 *pBuf;
  ZStatusValues status;
  ApsAddGroup addGroup = APS_ADD_GROUP__INIT;

  addGroup.endpoint = endpoint;
  addGroup.groupid = pGroup->ID;
  addGroup.has_name = FALSE;
    
  addGroup.name.data = malloc( pGroup->name[0] );
  if ( addGroup.name.data )
  {
    addGroup.has_name = TRUE;

    addGroup.name.len = pGroup->name[0];
    zcl_memcpy( addGroup.name.data, &(pGroup->name[1]), pGroup->name[0] );
  }
  
  len = aps_add_group__get_packed_size( &addGroup );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_add_group__pack( &addGroup, pBuf );

    // Send the NPI message
    status = sendAPICExpectDefaultStatus( ZSTACK_CMD_IDS__APS_ADD_GROUP, len, pBuf );  

    if ( status != ZSTATUS_VALUES__ZSuccess )
    {
      uiPrintfEx(trDEBUG, "APS Add Group bad status(%d).\n", status );
    }
    else
    {
      uiPrintfEx(trDEBUG, "APS Add Group Successful\n" );
    }

    free( pBuf );
  }

  if ( addGroup.has_name == TRUE )
  {
    free( addGroup.name.data );
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      aps_FindGroup
 *
 * @brief   Process in the received Find Group Command.
 *
 * @param   endpoint - application endpoint
 * @param   groupID - group ID
 *
 * @return  aps_Group_t
 */
aps_Group_t *aps_FindGroup( uint8 endpoint, uint16 groupID )
{
  uint8 rspCmdId;
  uint8 *pBuf;
  uint8 *pRsp;  
  int len;
  uint16 rspLen;
  ApsFindGroupReq findGroupReq = APS_FIND_GROUP_REQ__INIT;

  findGroupReq.endpoint = endpoint;
  findGroupReq.groupid = groupID;
  
  len = aps_find_group_req__get_packed_size( &findGroupReq );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_find_group_req__pack( &findGroupReq, pBuf );

    // send serialized request to API Client synchronously
    pRsp = apicSendSynchData( GW_ZSTACK_HANDLE, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, 
                              ZSTACK_CMD_IDS__APS_FIND_GROUP_REQ, len, pBuf,
                              NULL, &rspCmdId, &rspLen );

    if ( pRsp )
    {
      if ( (ZSTACK_CMD_IDS__APS_FIND_GROUP_RSP == rspCmdId) && (rspLen > 0) )
      {
        ApsFindGroupRsp *pFindGroupRsp;
        
        pFindGroupRsp = aps_find_group_rsp__unpack( NULL, rspLen, pRsp );
        if ( pFindGroupRsp )
        {                
          uiPrintfEx(trDEBUG, "APS Find Group Response Successful\n" );
          
          if ( pFindGroupRsp->has_groupid == TRUE )
          {
            gFindGroupRsp.ID = pFindGroupRsp->groupid;
          }
          else
          {
            gFindGroupRsp.ID = 0xFFFF;
          }
          
          if ( pFindGroupRsp->has_name == TRUE )
          {
            if ( pFindGroupRsp->name.len > (APS_GROUP_NAME_LEN - 1) )
            {
              gFindGroupRsp.name[0] = APS_GROUP_NAME_LEN;
            }
            else
            {
              gFindGroupRsp.name[0] = pFindGroupRsp->name.len;
              
              memcpy( &(gFindGroupRsp.name[1]), pFindGroupRsp->name.data, pFindGroupRsp->name.len ); 
            }
          }
          
          aps_find_group_rsp__free_unpacked( pFindGroupRsp, NULL );
        }
      }
      else
      {
        uiPrintfEx(trDEBUG, "Expected APS Find Group Response, got CmdId: %d\n", rspCmdId );
      }
      
      apicFreeSynchData( pRsp );
    }
    
    free( pBuf );
  }
  
  return ( &gFindGroupRsp );
}

/*********************************************************************
 * @fn      aps_FindGroupForEndpoint
 *
 * @brief   Process in the received Find Group For Endpoint Command.
 *
 * @param   groupID - application endpoint
 * @param   lastEP - last endpoint
 *
 * @return  returns endpoint if found, or 0xFF if not found
 */
uint8 aps_FindGroupForEndpoint( uint16 groupID, uint8 lastEP )
{
  // currently not supported
  return ( 0 );
}

/*********************************************************************
 * @fn      aps_FindAllGroupsForEndpoint
 *
 * @brief   Process in the received Find All Groups for Endpoint Command.
 *
 * @param   endpoint - application endpoint
 * @param   pGroupList - list of group IDs
 *
 * @return  returns number of groups copied to group list
 */
uint8 aps_FindAllGroupsForEndpoint( uint8 endpoint, uint16 *pGroupList )
{
  // currently not supported
  return ( 0 );
}

/*********************************************************************
 * @fn      aps_CountAllGroups
 *
 * @brief   Process in the received Count All Groups Command.
 *
 * @param   none
 *
 * @return  returns number of groups
 */
uint8 aps_CountAllGroups( void )
{
  // currently not supported
  return ( 0 );
}

/*********************************************************************
 * @fn      aps_RemoveGroup
 *
 * @brief   Process in the received Remove Group Command.
 *
 * @param   endpoint - application endpoint
 * @param   groupID - group ID
 *
 * @return  returns TRUE if removed, FALSE if not found
 */
uint8 aps_RemoveGroup( uint8 endpoint, uint16 groupID )
{
  int len;
  uint8 status;
  uint8 *pBuf;
  ApsRemoveGroup removeGroup = APS_REMOVE_GROUP__INIT;

  removeGroup.endpoint = endpoint;
  removeGroup.groupid = groupID;
  
  len = aps_remove_group__get_packed_size( &removeGroup );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_remove_group__pack( &removeGroup, pBuf );

    // Send the API Client message
    status = sendAPICExpectDefaultStatus( ZSTACK_CMD_IDS__APS_REMOVE_GROUP, len, pBuf );  

    if ( status != 0 )
    {
      uiPrintfEx(trDEBUG, "APS Remove Group bad status(%d).\n", status );
      
      status = FALSE;
    }
    else
    {
      uiPrintfEx(trDEBUG, "APS Remove Group Successful\n" );
      
      status = TRUE;
    }

    free( pBuf );
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      aps_RemoveAllGroup
 *
 * @brief   Process in the received Remove All Group Command.
 *
 * @param   endpoint - application endpoint
 *
 * @return  none
 */
void aps_RemoveAllGroup( uint8 endpoint )
{
  int len;
  uint8 *pBuf;
  ZStatusValues status;
  ApsRemoveAllGroups removeAllGroups = APS_REMOVE_ALL_GROUPS__INIT;

  removeAllGroups.endpoint = endpoint;
  
  len = aps_remove_all_groups__get_packed_size( &removeAllGroups );
  pBuf = malloc( len );
  if ( pBuf )
  {
    aps_remove_all_groups__pack( &removeAllGroups, pBuf );

    // Send the API Client message
    status = sendAPICExpectDefaultStatus( ZSTACK_CMD_IDS__APS_REMOVE_ALL_GROUPS, len, pBuf );  

    if ( status != ZSTATUS_VALUES__ZSuccess )
    {
      uiPrintfEx(trDEBUG, "APS Remove All Groups bad status(%d).\n", status );
    }
    else
    {
      uiPrintfEx(trDEBUG, "APS Remove All Groups Successful\n" );
    }

    free( pBuf );
  }
}

#endif  // ZCL_GROUPS
