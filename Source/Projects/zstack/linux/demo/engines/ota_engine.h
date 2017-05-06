/*******************************************************************************
 Filename:       ota_engine.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:	 Handles the upgrade/downgrade of remote devices 


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
*******************************************************************************/

#ifndef OTA_ENGINE_H
#define OTA_ENGINE_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "types.h"

#define MAX_UPGRADE_FILES 10

extern int upgrade_num_files;
extern upgrade_info_t upgrade_file_status[];


/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

#if 0
/* Status of the download */
typedef enum upgrade_dl_status {
  UPGRADE_SUCCESS = 0,
  UPGRADE_INVALID_IMAGE,
  UPGRADE_REQ_MORE_IMAGE,
  UPGRADE_ABORT,
} upgrade_dl_status_t;

/* Callback fxn registered for informing caller of download status */
typedef void (* upgradeCBFxn)(zb_addr_t addr ,upgrade_dl_status_t status); 

void upgrade_init(char * fileName, upgradeCBFxn * fxn);

/* Process Download Status recvd from device */
void upgrade_process_download_status(pkt_buf_t * pkt);
#endif


/* Register all files, details in configFileName*/
void ota_enable_state_machine(bool timed_out, void * arg);

void ota_process_download_finished_indication(pkt_buf_t * pkt);


#endif /* OTA_ENGINE_H */
