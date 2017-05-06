/*******************************************************************************
 Filename:       group_scene_engine.h
 Revised:        $Date: 2014-06-03 19:06:22 -0700 (Tue, 03 Jun 2014) $
 Revision:       $Revision: 38790 $

 Description:     Handle Group and Scene related APIs


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

/******************************************************************************
 * Includes
 *****************************************************************************/
#ifndef GROUP_SCENE_ENGINE_H
#define GROUP_SCENE_ENGINE_H

#include "gateway.pb-c.h"

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void gs_add_group(zb_addr_t * addr, uint16_t groupid, char * groupname);
void gs_remove_from_group(zb_addr_t * addr, uint16_t groupid);
void gs_store_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid);
void gs_remove_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid);
void gs_recall_scene(zb_addr_t * addr, uint16_t groupid, uint32_t sceneid);
int convert_address(zb_addr_t * addr,	GwAddressStructT * dstaddr);

#endif /* GROUP_SCENE_ENGINE_H */
