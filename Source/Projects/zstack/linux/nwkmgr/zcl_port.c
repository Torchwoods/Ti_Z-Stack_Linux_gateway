/**************************************************************************************************
  Filename:       zcl_port.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the ZCL porting layer, 
                  Ported for Windows Test App.


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
#include "api_client.h"
#include "hal_rpc.h"

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

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
ZStatusValues sendAPICExpectDefaultStatus( int cmdID, int len, uint8 *pData );

/*********************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void convertTxOptions( TransOptions *pOptions, uint8 options );


/*********************************************************************
 * PUBLIC FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      zcl_HandleExternal
 *
 * @brief   
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  TRUE
 */
uint8 zcl_HandleExternal( zclIncoming_t *pInMsg )
{

  return ( TRUE );
}

/*********************************************************************
 * @fn          zcl_mem_alloc
 *
 * @brief       Abstraction function to allocate memory
 *
 * @param       size - size in bytes needed
 *
 * @return      pointer to allocated buffer, NULL if nothing allocated.
 */
void *zcl_mem_alloc( uint16 size )
{
  return ( (void *)malloc( size ) );
}

void *zcl_memset( void *dest, uint8 value, int len )
{
  return ( (void *)memset( dest, value, len ) );
}

void *zcl_memcpy( void *dst, void *src, unsigned int len )
{
  return ( (void *)memcpy( dst, src, len ) );
}

extern void *zcl_cpyExtAddr(uint8 * pDest, const uint8 * pSrc)
{
  return memcpy( pDest, pSrc, 8);
}

void zcl_mem_free(void *ptr)
{
  free( ptr );
}

uint8* zcl_buffer_uint32( uint8 *buf, uint32 val )
{
  *buf++ = BREAK_UINT32( val, 0 );
  *buf++ = BREAK_UINT32( val, 1 );
  *buf++ = BREAK_UINT32( val, 2 );
  *buf++ = BREAK_UINT32( val, 3 );

  return buf;
}

uint32 zcl_build_uint32( uint8 *swapped, uint8 len )
{
  if ( len == 2 )
    return ( BUILD_UINT32( swapped[0], swapped[1], 0L, 0L ) );
  else if ( len == 3 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], 0L ) );
  else if ( len == 4 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], swapped[3] ) );
  else
    return ( (uint32)swapped[0] );
}

uint8 zcl_nv_item_init( uint16 id, uint16 len, void *buf )
{
  return ( 0 );
}

uint8 zcl_nv_write( uint16 id, uint16 ndx, uint16 len, void *buf )
{
  return ( 0 );
}

uint8 zcl_nv_read( uint16 id, uint16 ndx, uint16 len, void *buf )
{
  return ( 0 );
}

/*********************************************************************
 * API Functions
 *********************************************************************/
extern endPointDesc_t zEpDesc;

endPointDesc_t *afFindEndPointDesc( uint8 EndPoint )
{
  if ( EndPoint == zEpDesc.endPoint )
  {
    return ( &zEpDesc );
  }
  else
  {
   return ( (endPointDesc_t *)NULL );
  }
}

void zclGeneral_ScenesInit( void )
{
}

void zclGeneral_ScenesSave( void )
{
}

uint8 zclGeneral_CountAllScenes( void )
{
  return ( 0 );
}

uint8 zclGeneral_FindAllScenesForGroup( uint8 endpoint, uint16 groupID, uint8 *sceneList )
{
  return ( 0 );
}

void zclGeneral_RemoveAllScenes( uint8 endpoint, uint16 groupID )
{
}

uint8 zclGeneral_RemoveScene( uint8 endpoint, uint16 groupID, uint8 sceneID )
{
  return ( TRUE );
}

zclGeneral_Scene_t *zclGeneral_FindScene( uint8 endpoint, uint16 groupID, uint8 sceneID )
{
  return ( (zclGeneral_Scene_t *)NULL );
}

ZStatus_t zclGeneral_AddScene( uint8 endpoint, zclGeneral_Scene_t *scene )
{
  return ( 0 );
}

/*********************************************************************
 * @fn      AF_DataRequest
 *
 * @brief   Common functionality for invoking APSDE_DataReq() for both
 *          SendMulti and MSG-Send.
 *
 *          NOTE: this is a conversion function
 *
 * input parameters
 *
 * @param  *dstAddr - Full ZB destination address: Nwk Addr + End Point.
 * @param  *srcEP - Origination (i.e. respond to or ack to) End Point Descr.
 * @param   cID - A valid cluster ID as specified by the Profile.
 * @param   bufLen - Number of bytes of data pointed to by next param.
 * @param  *buf - A pointer to the data bytes to send.
 * @param  *transID - A pointer to a byte which can be modified and which will
 *                    be used as the transaction sequence number of the msg.
 * @param   options - Valid bit mask of Tx options.
 * @param   radius - Normally set to AF_DEFAULT_RADIUS.
 *
 * output parameters
 *
 * @param  *transID - Incremented by one if the return value is success.
 *
 * @return  afStatus_t - See previous definition of afStatus_... types.
 */
afStatus_t AF_DataRequest( afAddrType_t *dstAddr, endPointDesc_t *srcEP,
                           uint16 cID, uint16 bufLen, uint8 *buf, uint8 *transID,
                           uint8 options, uint8 radius )
{
  afStatus_t status;
  int len;
  uint8 *pBuf;
  AFAddr dstaddr = AFADDR__INIT;
  TransOptions transOptions = TRANS_OPTIONS__INIT;
  AfDataReq dataReq = AF_DATA_REQ__INIT;

  uiPrintfEx(trINFO, "\nSending AF Data Request - dstAddr:%x, ep:%d, cID:%x, buLen:%d\n",
                   dstAddr->addr.shortAddr, dstAddr->endPoint, cID, bufLen );

  dataReq.cmdid = (ZStackCmdIDs) ZSTACK_CMD_IDS__AF_DATA_REQ;

  dstaddr.addrmode = dstAddr->addrMode;
  if ( dstaddr.addrmode == AFADDR_MODE__EXT )
  {
    dstaddr.has_extaddr = TRUE;
    memcpy( &dstaddr.extaddr, dstAddr->addr.extAddr, 8 );
  }
  else if ( dstaddr.addrmode != AFADDR_MODE__NONE )
  {
    dstaddr.has_shortaddr = TRUE;
    dstaddr.shortaddr = dstAddr->addr.shortAddr;
  }

  dstaddr.has_endpoint = TRUE;
  dstaddr.endpoint = dstAddr->endPoint;
  dstaddr.has_panid = TRUE;
  dstaddr.panid = dstAddr->panId;

  dataReq.dstaddr = &dstaddr;

  convertTxOptions( &transOptions, options );

  dataReq.options = &transOptions;

  dataReq.srcendpoint = srcEP->endPoint;
  dataReq.clusterid = cID;
  dataReq.transid = *transID;
  *transID += 1;
  dataReq.radius = radius;
  dataReq.payload.len = bufLen;
  dataReq.payload.data = buf;

  len = af_data_req__get_packed_size( &dataReq );
  pBuf = malloc( len );
  if ( pBuf )
  {
    af_data_req__pack( &dataReq, pBuf );

    // Send the API Client message
    status = (afStatus_t)sendAPICExpectDefaultStatus( ZSTACK_CMD_IDS__AF_DATA_REQ, len, pBuf );

    free( pBuf );
  }

  return ( status );
}

/***************************************************************************************************
 * @fn          convertTxOptions
 *
 * @brief       Convert uint8 txOptions into PB TransOptions data type
 *
 * @param       pOptions - TransOptions pointer
 * @param       options - txOptions
 *
 * @return      none
 ***************************************************************************************************/
static void convertTxOptions( TransOptions *pOptions, uint8 options )
{
  if ( options & AF_WILDCARD_PROFILEID )
  {
    pOptions->has_wildcardprofileid = TRUE;
    pOptions->wildcardprofileid = TRUE;
  }

  if ( options & AF_ACK_REQUEST )
  {
    pOptions->has_ackrequest = TRUE;
    pOptions->ackrequest = TRUE;
  }

  if ( options & AF_LIMIT_CONCENTRATOR )
  {
    pOptions->has_limitconcentrator = TRUE;
    pOptions->limitconcentrator = TRUE;
  }

  if ( options & AF_SUPRESS_ROUTE_DISC_NETWORK )
  {
    pOptions->has_suppressroutedisc = TRUE;
    pOptions->suppressroutedisc = TRUE;
  }

  if ( options & AF_EN_SECURITY )
  {
    pOptions->has_apssecurity = TRUE;
    pOptions->apssecurity = TRUE;
  }

  if ( options & AF_SKIP_ROUTING )
  {
    pOptions->has_skiprouting = TRUE;
    pOptions->skiprouting = TRUE;
  }
}

/*********************************************************************
*********************************************************************/

