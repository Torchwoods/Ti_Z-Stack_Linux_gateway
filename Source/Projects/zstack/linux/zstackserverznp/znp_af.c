/**************************************************************************************************
 Filename:       znp_af.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:

 This file declares the ZNP Application Processor proxy AF API functions.


 Copyright 2013 -2014 Texas Instruments Incorporated. All rights reserved.

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
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "api_client.h"

#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"

#include "MT.h"
#include "trace.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#define AF_DATA_STORE_MAX_LEN 247
#define AF_DATA_REQ_HDR_LEN 20

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */
extern apicHandle_t zmainClientHandle;
#define ZNP_API_CLIENT zmainClientHandle

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static endPointDesc_t EPDesc = {0};

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
extern uint8 sendNPIExpectDefaultStatusZNP( int subSys, int cmdID, int len,
                                            uint8 *pData );

static uint8 sendNPIExpectDefaultStatusAF( int cmdID, int len, uint8 *pData );
static afStatus_t sendAFDataStore( uint16 idx, uint8 len, uint8 *pData );

/**************************************************************************************************
 *
 * @fn          sendNPIExpectDefaultStatusAF
 *
 * @brief       Send a request message and expect the normal "default" response (MacDefaultRsp)
 *
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      synchronous return status
 *
 **************************************************************************************************/
static uint8 sendNPIExpectDefaultStatusAF( int cmdID, int len, uint8 *pData )
{
  return (sendNPIExpectDefaultStatusZNP( MT_RPC_SYS_AF, cmdID, len, pData ));
}

/*********************************************************************
 * @fn      afFindEndPointDesc
 *
 * @brief   Find the endpoint description entry from the endpoint
 *          number.
 *
 * @param   EndPoint - Application Endpoint to look for
 *
 * @return  the address to the endpoint/interface description entry
 */
endPointDesc_t *afFindEndPointDesc( uint8 EndPoint )
{
  // We always find an endpoint descriptor
  EPDesc.endPoint = EndPoint;
  return (&EPDesc);
}

/*********************************************************************
 * @fn      afRegister
 *
 * @brief   Register an Application's EndPoint description.
 *
 * @param   epDesc - pointer to the Application's endpoint descriptor.
 *
 * NOTE:  The memory that epDesc is pointing to must exist after this call.
 *
 * @return  afStatus_SUCCESS - Registered
 *          afStatus_MEM_FAIL - not enough memory to add descriptor
 *          afStatus_INVALID_PARAMETER - duplicate endpoint
 */
afStatus_t afRegister( endPointDesc_t *epDesc )
{
  afStatus_t status;
  const uint8 len = ((epDesc->simpleDesc->AppNumInClusters) * 2)
      + ((epDesc->simpleDesc->AppNumOutClusters) * 2) + 9;

  uint8 *pMsg = malloc( len );
  if ( pMsg )
  {
    uint8 *pBuf = pMsg;
    uint8 idx;

    *pBuf++ = (uint8)epDesc->endPoint;
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppProfId);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppProfId >> 8);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppDeviceId);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppDeviceId >> 8);
    *pBuf++ = epDesc->simpleDesc->AppDevVer;
    *pBuf++ = epDesc->latencyReq;
    *pBuf++ = epDesc->simpleDesc->AppNumInClusters;
    for ( idx = 0; idx < epDesc->simpleDesc->AppNumInClusters; idx++ )
    {
      *pBuf++ = LO_UINT16( epDesc->simpleDesc->pAppInClusterList[idx] );
      *pBuf++ = HI_UINT16( epDesc->simpleDesc->pAppInClusterList[idx] );
    }
    *pBuf++ = epDesc->simpleDesc->AppNumOutClusters;
    for ( idx = 0; idx < epDesc->simpleDesc->AppNumOutClusters; idx++ )
    {
      *pBuf++ = LO_UINT16( epDesc->simpleDesc->pAppOutClusterList[idx] );
      *pBuf++ = HI_UINT16( epDesc->simpleDesc->pAppOutClusterList[idx] );
    }

    status = sendNPIExpectDefaultStatusAF( MT_AF_REGISTER, len, pMsg );

    free( pMsg );
  }
  else
  {
    status = afStatus_MEM_FAIL;
  }

  return (status);
}

/*********************************************************************
 * @fn      afDelete
 *
 * @brief   Delete an Application's EndPoint descriptor and frees the memory
 *
 * @param   EndPoint - Application Endpoint to delete
 *
 * @return  afStatus_SUCCESS - endpoint deleted
 *          afStatus_INVALID_PARAMETER - endpoint not found
 *          afStatus_FAILED - endpoint list empty
 */
afStatus_t afDelete( uint8 EndPoint )
{
  ZStatus_t status;

  status = sendNPIExpectDefaultStatusAF( MT_AF_DELETE, 1, &EndPoint );

  return (status);
}

/**************************************************************************************************
 * @fn          afAPSF_ConfigGet
 *
 * @brief       This function ascertains the fragmentation configuration that corresponds to
 *              the specified EndPoint.
 *
 * @param       endPoint - The source EP of a Tx or destination EP of a Rx fragmented message.
 *
 * @param       pCfg - A pointer to an APSF configuration structure to fill with values.
 *
 * @return      None.
 */
void afAPSF_ConfigGet( uint8 endPoint, afAPSF_Config_t *pCfg )
{
  uint8 *pRsp, rspcmdid;

  // send request to NPI synchronously
  pRsp = apicSendSynchData( ZNP_API_CLIENT, MT_RPC_SYS_AF,
      MT_AF_APSF_CONFIG_GET, 1, &endPoint, NULL, &rspcmdid, NULL );

  if ( pRsp )
  {
    if ( rspcmdid == MT_AF_APSF_CONFIG_GET )
    {
      // Process the immediate response
      pCfg->frameDelay = pRsp[0];
      pCfg->windowSize = pRsp[1];
    }

    apicFreeSynchData( pRsp );
  }
}

/**************************************************************************************************
 * @fn          afAPSF_ConfigSet
 *
 * @brief       This function attempts to set the fragmentation configuration that corresponds to
 *              the specified EndPoint.
 *
 * @param       endPoint - The specific EndPoint for which to set the fragmentation configuration.
 *
 * @param       pCfg - A pointer to an APSF configuration structure to fill with values.
 *
 * @return      afStatus_SUCCESS for success.
 *              afStatus_INVALID_PARAMETER if the specified EndPoint is not registered.
 */
afStatus_t afAPSF_ConfigSet( uint8 endPoint, afAPSF_Config_t *pCfg )
{
  ZStatus_t status;
  uint8 buf[3];

  buf[0] = endPoint;
  buf[1] = pCfg->frameDelay;
  buf[2] = pCfg->windowSize;

  status = sendNPIExpectDefaultStatusAF( MT_AF_APSF_CONFIG_SET, 3, buf );

  return (status);
}

/*********************************************************************
 * @fn      AF_DataRequestSrcRtg
 *
 * @brief   Common functionality for invoking APSDE_DataReq() for both
 *          SendMulti and MSG-Send.
 *
 * @param  *dstAddr - Full ZB destination address: Nwk Addr + End Point.
 * @param  *srcEP - Origination (i.e. respond to or ack to) End Point Descr.
 * @param   cID - A valid cluster ID as specified by the Profile.
 * @param   len - Number of bytes of data pointed to by next param.
 * @param  *buf - A pointer to the data bytes to send.
 * @param  *transID - A pointer to a byte which can be modified and which will
 *                    be used as the transaction sequence number of the msg.
 * @param   options - Valid bit mask of Tx options.
 * @param   radius - Normally set to AF_DEFAULT_RADIUS.
 * @param   relayCnt - Number of devices in the relay list
 * @param   pRelayList - Pointer to the relay list
 *
 * @param  *transID - Incremented by one if the return value is success.
 *
 * @return  afStatus_t - See previous definition of afStatus_... types.
 */

afStatus_t AF_DataRequestSrcRtg( afAddrType_t *dstAddr, endPointDesc_t *srcEP,
                                 uint16 cID, uint16 len, uint8 *buf,
                                 uint8 *transID, uint8 options, uint8 radius,
                                 uint8 relayCnt, uint16* pRelayList )
{
  afStatus_t status;
  uint8 msgLen = (10 + (relayCnt * 2) + len);
  uint8 *pMsg = malloc( msgLen );
  if ( pMsg )
  {
    uint8 *pBuf = pMsg;
    uint8 idx;

    *pBuf++ = LO_UINT16( dstAddr->addr.shortAddr );
    *pBuf++ = HI_UINT16( dstAddr->addr.shortAddr );
    *pBuf++ = dstAddr->endPoint;

    *pBuf++ = LO_UINT16( cID );
    *pBuf++ = HI_UINT16( cID );

    *pBuf++ = *transID;
    *transID += 1;

    *pBuf++ = options;
    *pBuf++ = radius;

    *pBuf++ = relayCnt;

    for ( idx = 0; idx < relayCnt; idx++, pRelayList++ )
    {
      *pBuf++ = LO_UINT16( *pRelayList );
      *pBuf++ = HI_UINT16( *pRelayList );
    }

    *pBuf++ = len;
    memcpy( pBuf, buf, len );

    status = sendNPIExpectDefaultStatusAF( MT_AF_DATA_REQUEST_SRCRTG, msgLen,
        pMsg );

    free( pMsg );
  }
  else
  {
    status = afStatus_MEM_FAIL;
  }

  return (status);
}

/*********************************************************************
 * @fn      AF_DataRequest
 *
 * @brief   Common functionality for invoking APSDE_DataReq() for both
 *          SendMulti and MSG-Send.
 *
 * @param  *dstAddr - Full ZB destination address: Nwk Addr + End Point.
 * @param  *srcEP - Origination (i.e. respond to or ack to) End Point Descr.
 * @param   cID - A valid cluster ID as specified by the Profile.
 * @param   len - Number of bytes of data pointed to by next param.
 * @param  *buf - A pointer to the data bytes to send.
 * @param  *transID - A pointer to a byte which can be modified and which will
 *                    be used as the transaction sequence number of the msg.
 * @param   options - Valid bit mask of Tx options.
 * @param   radius - Normally set to AF_DEFAULT_RADIUS.
 *
 * @param  *transID - Incremented by one if the return value is success.
 *
 * @return  afStatus_t - See previous definition of afStatus_... types.
 */
afStatus_t AF_DataRequest( afAddrType_t *dstAddr, endPointDesc_t *srcEP,
                           uint16 cID, uint16 len, uint8 *buf, uint8 *transID,
                           uint8 options, uint8 radius )
{
  afStatus_t status;
  uint16 msgLen;
  uint8 *pMsg;
  uint16 dataLen = 0;
  uint8 sentFrags = 0;

  if ( len > (MT_RPC_DATA_MAX - AF_DATA_REQ_HDR_LEN) )
  {
    // First build and send the AF_DATA_REQ_EXT message (no data)
    // Then, send the data is fragments
    dataLen = len;
    msgLen = (AF_DATA_REQ_HDR_LEN);
  }
  else
  {
    // Send the data and head in the one packet
    msgLen = (AF_DATA_REQ_HDR_LEN + len);
  }

  pMsg = malloc( msgLen );
  if ( pMsg )
  {
    uint8 *pBuf = pMsg;

    *pBuf++ = dstAddr->addrMode;
    if ( dstAddr->addrMode == afAddr64Bit )
    {
      memcpy( pBuf, dstAddr->addr.extAddr, 8 );
      pBuf += 8;
    }
    else
    {
      *pBuf++ = LO_UINT16( dstAddr->addr.shortAddr );
      *pBuf++ = HI_UINT16( dstAddr->addr.shortAddr );
      pBuf += 6;
    }
    *pBuf++ = dstAddr->endPoint;

    *pBuf++ = LO_UINT16( dstAddr->panId );
    *pBuf++ = HI_UINT16( dstAddr->panId );

    *pBuf++ = srcEP->endPoint;

    *pBuf++ = LO_UINT16( cID );
    *pBuf++ = HI_UINT16( cID );

    *pBuf++ = *transID;
    *transID += 1;

    *pBuf++ = options;
    *pBuf++ = radius;

    *pBuf++ = LO_UINT16( len );
    *pBuf++ = HI_UINT16( len );

    if ( dataLen == 0 )
    {
      memcpy( pBuf, buf, len );
    }

    status = sendNPIExpectDefaultStatusAF( MT_AF_DATA_REQUEST_EXT, msgLen,
        pMsg );
    free( pMsg );

    while ( dataLen )
    {
      uint8 fragSize;

      // Calculate the frag size for this packet
      if ( (dataLen - (sentFrags * AF_DATA_STORE_MAX_LEN))
          > AF_DATA_STORE_MAX_LEN )
      {
        fragSize = AF_DATA_STORE_MAX_LEN;
      }
      else
      {
        fragSize = (dataLen - (sentFrags * AF_DATA_STORE_MAX_LEN));
        dataLen = 0;
      }

      status = sendAFDataStore( (sentFrags * AF_DATA_STORE_MAX_LEN), fragSize,
          &buf[sentFrags * AF_DATA_STORE_MAX_LEN] );

      if ( dataLen == 0 )
      {
        // Send the "special" message to end the fragmentation.
        status = sendAFDataStore(
            ((sentFrags * AF_DATA_STORE_MAX_LEN) + fragSize), 0, NULL );
      }

      sentFrags++;
    }
  }
  else
  {
    status = afStatus_MEM_FAIL;
  }

  return (status);
}

/*********************************************************************
 * @fn      sendAFDataStore
 *
 * @brief   Send the AF Data Store Message
 *
 * @param  idx - index into the data buffer
 * @param  len - length of data to send
 * @param  pData - pointer to data to send
 *
 * @return  afStatus_t - See previous definition of afStatus_... types.
 */
static afStatus_t sendAFDataStore( uint16 idx, uint8 len, uint8 *pData )
{
  afStatus_t status;
  uint8 *pMsg;

  pMsg = malloc( len + 3 );
  if ( pMsg )
  {
    uint8 *pBuf = pMsg;

    *pBuf++ = LO_UINT16( idx );
    *pBuf++ = HI_UINT16( idx );
    *pBuf++ = len;

    memcpy( pBuf, pData, len );

    status = sendNPIExpectDefaultStatusAF( MT_AF_DATA_STORE, (len + 3), pMsg );

    free( pMsg );
  }
  else
  {
    status = afStatus_MEM_FAIL;
  }

  return (status);
}

/**************************************************************************************************
 */

