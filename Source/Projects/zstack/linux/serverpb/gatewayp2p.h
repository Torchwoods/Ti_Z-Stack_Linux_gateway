/**************************************************************************************************
  Filename:       gatewayp2p.h
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the peer-to-peer commands for the 
                  Gateway or other servers.

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

#ifndef GATEWAYP2P_H
#define GATEWAYP2P_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "server.pb-c.h"

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
 * EXTERNAL TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern apicHandle_t giNwkMgrHandle;


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern bool gwPb_SrvrGetIeeeAddress( uint16 shortAddr, uint64_t *pIeeeAddr );
extern bool gwPb_SrvrGetShortAddress( uint64_t ieeeAddr, uint16 *pShortAddr );
extern SrvrDeviceInfoT * gwPb_SrvrGetDeviceInfoReq( uint64_t ieeeAddr );
extern void gwPb_FreeSrvrGetDeviceInfo( SrvrDeviceInfoT *pDeviceInfo );
extern bool gwPb_SrvrGetDeviceStatus( uint64_t ieeeAddr, uint8 *pStatus );
extern bool gwPb_SrvrSetDeviceStatus( uint64_t ieeeAddr, uint8 status );  

/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* GATEWAYP2P_H */


