/**************************************************************************************************
 Filename:       OtaServer.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:    OTA Server.


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
#ifndef OTASERVER_H
#define OTASERVER_H

#include "ota_common.h"
#include "otasrvr.pb-c.h"

/* Types for OtaServer APIs return status */
#define OTASERVER_SUCCESS		0
#define OTASERVER_BADFILEFORMAT		1
#define OTASERVER_UNREGISTERFAILED	2
#define OTASERVER_OUTOFRESOURCES	3

#define OTASERVER_BADCOMMAND	 	4	
#define OTASERVER_IMG_DUPLICATE_ID	5 
#define OTASERVER_FOPENFAILED		6
#define OTASERVER_FILENOTFOUND		7

#define OTASERVER_FAILURE		8

/* Image Data information */
typedef struct
{
  OTA_ImageHeader_t header;
  uint32 length;
  uint8 *pData;
} OTA_ImageData_t;


/* Upgrade Node */
typedef struct OTA_UpgradeInstance
{
  OTA_ImageData_t * imageData;
  OtaExecuteType executionType;
  uint32 executionDelay;
  int32 executionTime;
  int connection; 		//Not sure
  struct OTA_UpgradeInstance * next;
  int deviceNumber;
  uint64_t * deviceList;
} OTA_UpgradeInstance_t;

#define OTA_UPGRADE_INSTANCE_INIT(node) {(node)->imageData = NULL; (node)->executionType = OTA_EXECUTE_TYPE__NO_CHANGE; (node)->executionDelay = 0; (node)->executionTime = 0; (node)->connection = 0; (node)->next = NULL; (node)->deviceNumber = 0; (node)->deviceList = NULL;}

#if 0
/* Sorted Image List Element */
typedef struct
{
  OTA_UpgradeInstance_t imageInstance;
  struct OTA_ImageList_t * next;
} OTA_ImageList_t;
#endif

uint8 OtaServer_AddImage(char *fileName, OtaExecuteType executionType, 
	uint32 executionDelay, uint32 executionTime, uint8 updateDeviceList, 
	uint8 deviceNumber, 
	uint64_t * deviceList, OtaNotificationType notification, int connection);

uint8 OtaServer_DeleteImage(char * fileName);

uint8 OtaServer_GetImage(zclOTA_FileID_t *pFileId, uint32 * imageSize, 
	uint16 hwVer, uint8 *ieee, uint8 options, afAddrType_t * addr);

uint32 OtaServer_GetImageBlock(zclOTA_FileID_t * pFileId, uint32 offset, 
	uint8 * pRsp, uint8 len);

uint32 OtaServer_GetImageSize(zclOTA_FileID_t * pFileId);
 
void OtaServer_PermitOta(uint8 permit);

OTA_UpgradeInstance_t * OtaServer_GetInstance(zclOTA_FileID_t *pFileId);

#endif /* OTASERVER_H */
