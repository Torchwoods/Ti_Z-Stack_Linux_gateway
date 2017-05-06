/**************************************************************************************************
  Filename:       serverdefep.h
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains references to the hard-coded endpoint.
                  See serverep.c for the hard-coded attributes.

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
#ifndef _SERVER_DEF_EP_H_
#define _SERVER_DEF_EP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define ZDO_EP  0   // reserved for ZDO
#define NM_EP   1   // reserved for NWK Server
#define GW_EP   2   // reserved for Gateway Server
#define APP_EP  3   // defines the first endpoint for the application
#define APP_MAX_EP  254 // maximum endpoint ID


extern uint32_t srvDefaultInClusterList[];
extern int srvDefaultNumInClusters;
extern uint32_t srvDefaultOutClusterList[];
extern int srvDefaultNumOutClusters;
extern SimpleDescriptionFormat_t srvSimpleDesc;

#ifdef __cplusplus
}
#endif
#endif  // _SERVER_DEF_EP_H_
