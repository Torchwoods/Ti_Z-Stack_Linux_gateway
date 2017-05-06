/**************************************************************************************************
 Filename:       api_client.h
 Revised:        $Date: 2014-03-18 09:50:29 -0700 (Tue, 18 Mar 2014) $
 Revision:       $Revision: 37767 $

 Description:    This file contains API information for the NPI interface.

 Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef API_CLIENT_H
#define API_CLIENT_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------------------------------------
 * Includes
 * ------------------------------------------------------------------------------------------------
 */
#include <stdlib.h>

/* ------------------------------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined ( NULL )
#define NULL  0
#endif

#define TRUE 1
#define FALSE 0

// These came from npi_lnx.h
#define AP_MAX_BUF_LEN 256
#define APIC_PORT      "2533"

/* ------------------------------------------------------------------------------------------------
 * Typedefs
 * ------------------------------------------------------------------------------------------------
 */

// The data structure for header portion of
// 16 bit length MT frame
// Note that this data structure is used
// even if API_CLIENT_8BIT_LEN is defined.
typedef struct ATTR_PACKED
{
  uint8 lenL;
  uint8 lenH;
  uint8 subSys;
  uint8 cmdId;
} apic16BitLenMsgHdr_t;

#ifdef API_CLIENT_8BIT_LEN
// The data structure for header portion of
// 8 bit length MT frame
typedef struct ATTR_PACKED
{
  uint8 len;
  uint8 subSys;
  uint8 cmdId;
}apic8BitLenMsgHdr_t;
#endif // API_CLIENT_8BIT_LEN
typedef void *apicHandle_t;

typedef void (*pfnAsyncMsgCb)( apicHandle_t handle, uint8 subSys, uint8 cmdID,
                               uint16 len, uint8 *pData );

/**************************************************************************************************
 * Globals
 **************************************************************************************************/

// Size in bytes of the stack for each internal thread
// created by API client.
// Application may change the value before making apicInit() call
// to customize the stack size.
extern size_t apicThreadStackSize;

/* ------------------------------------------------------------------------------------------------
 * Functions
 * ------------------------------------------------------------------------------------------------
 */
extern void apicIgnoreSigPipe( void );
extern apicHandle_t apicInit( const char *srvAddr, bool getVer,
                              pfnAsyncMsgCb pFn );
extern void apicClose( apicHandle_t handle );
extern uint8 *apicSendSynchData( apicHandle_t handle, uint8 subSys, uint8 cmdId,
                                 uint16 len, const uint8 *pData,
                                 uint8 *pRxSubSys, uint8 *pRxCmdId,
                                 uint16 *pRxLen );
extern void apicFreeSynchData( uint8 *pData );
extern void apicSendAsynchData( apicHandle_t handle, uint8 subSys, uint8 cmdId,
                                uint16 len, uint8 *pData );
extern void apicReadVersionReq( apicHandle_t handle, uint8 *pValue );
extern void apicReadParamReq( apicHandle_t handle, uint8 paramId, uint8 len,
                              uint8 *pValue );

/**************************************************************************************************
 */

#ifdef __cplusplus
};
#endif

#endif /* API_CLIENT_H */
