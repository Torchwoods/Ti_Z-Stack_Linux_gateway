/**************************************************************************************************
  Filename:       nwkmgrsrv.h
  Revised:        $Date:  $
  Revision:       $Revision:  $

  Description:    This file contains the linux Network Manager server definitions


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

#ifndef NWKMGRSRV_H
#define NWKMGRSRV_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "zcl.h"
#include "nwkmgr.pb-c.h"
#include "zstack.pb-c.h"
#include "nwkmgrdatabase.h"

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

// the timer ticks once every 1/4 second
#define TIMER_WAIT_PERIOD            250000    // 1/4 second in usecs (must be < 1 sec)

#define MAX_DEVICE_FAILED_ATTEMPTS   5   // make number of tries to issue message to 
                                         // remote device without handling device status

#define NWK_ROUTE_RADIUS    5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern void sendNwkZigbeeDeviceInd( NwkZigbeeDeviceInd *pDeviceInd );

extern ZStatusValues sendSimpleDescReq( uint16 dstAddr, uint8 endpointId );
extern ZStatusValues sendZdoNodeDescReq( uint16 shortAddr );
extern ZStatusValues sendZdoNwkAddrReq( uint64_t ieeeAddr );
extern ZStatusValues sendZdoActiveEndpointReq( uint16 shortAddr );
extern ZStatusValues sendZdoIeeeAddrReq( uint16 nwkAddr );
extern ZStatusValues sendUnicastRouteReq( uint16 addr );

extern NwkDeviceInfoT *allocPbDeviceInfoList( int count, sNwkMgrDb_DeviceInfo_t *pInDeviceInfo );
extern void freePbDeviceInfoList( int count, NwkDeviceInfoT *pDeviceInfo );

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern char gszDbDeviceInfoPath[];
extern char gszDbEndpointsPath[];
extern int giNmDeviceTimeout;
extern int giNmStateTicks;

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* NWKMGRSRV_H */
