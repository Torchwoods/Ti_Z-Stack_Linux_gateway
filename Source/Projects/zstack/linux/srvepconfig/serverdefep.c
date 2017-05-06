/**************************************************************************************************
  Filename:       serverdefep.c
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the hard-coded endpoint.
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


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/file.h>

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "serverdefep.h"

/**************************************************************************************************
 * Constants
 **************************************************************************************************/

uint32_t srvDefaultInClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_ON_OFF,
  ZCL_CLUSTER_ID_SS_IAS_ACE
};
#define SRV_MAX_INCLUSTERS ((sizeof(srvDefaultInClusterList) / sizeof(srvDefaultInClusterList[0])))

uint32_t srvDefaultOutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF,
  ZCL_CLUSTER_ID_GEN_ALARMS,
  ZCL_CLUSTER_ID_GEN_POLL_CONTROL
};
#define SRV_MAX_OUTCLUSTERS ((sizeof(srvDefaultOutClusterList) / sizeof(srvDefaultOutClusterList[0])))

int srvDefaultNumInClusters = SRV_MAX_INCLUSTERS;
int srvDefaultNumOutClusters = SRV_MAX_OUTCLUSTERS;

#define SRV_DEVICE_VERSION         0
#define SRV_FLAGS                  0
#define SRV_HWVERSION              0
#define SRV_ZCLVERSION             0

// only needed for profile and version. cluster list taken care of separately
SimpleDescriptionFormat_t srvSimpleDesc =
{
  GW_EP,                              //  int    Endpoint
  ZCL_HA_PROFILE_ID,                  //  uint16 AppProfId[2]
  ZCL_HA_DEVICEID_COMBINED_INETRFACE, //  uint16 AppDeviceId[2]
  SRV_DEVICE_VERSION,                 //  int    AppDevVer:4
  SRV_FLAGS,                          //  int    AppFlags:4
  SRV_MAX_INCLUSTERS,                 //  uint8  AppNumInClusters
  (cId_t *)NULL,                      //  uint8 *pAppInClusterList (not used)
  SRV_MAX_OUTCLUSTERS,                //  uint8  AppNumInClusters
  (cId_t *)NULL                       //  uint8 *pAppInClusterList (not used)
};
