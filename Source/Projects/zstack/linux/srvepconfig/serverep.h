/**************************************************************************************************
  Filename:       serverep.h
  Revised:        $Date$
  Revision:       $Revision$

  Description:    This file contains the Server Endpoint Configuration File
                  functions.

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

#ifndef SERVEREP_H
#define SERVEREP_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "zstack.pb-c.h"

/*********************************************************************
 * MACROS
 */

  
/*********************************************************************
 * CONSTANTS
 */

#define SRVEPERR_NONE     0   // success
#define SRVEPERR_NOFILE   1   // no file, hard-coded endpoint used instead
#define SRVEPERR_BADFILE  2   // syntax error in the file
#define SRVEPERR_MEMERROR 3   // syntax error in the file
#define SRVEPERR_NOSYMBOL 4   // referring to a symbol not there

#define MAX_CONFIG_STRING_LEN   180   // used for config file

/*********************************************************************
 * TYPEDEFS
 */

// parameters to zcl_registerAttrList( uint8 endpoint, uint8 numAttr, CONST zclAttrRec_t attrList[] );
typedef struct sSrvAttrList_tag
{
  uint8   endpoint;
  int     numAttr;            // # of attributes in the two parallel arrays
  zclAttrRec_t *pAttrRecords; // array of attributes records for registering
  void      **ppAttrPtrs;     // array of pointers to attribute variables (e.g. uint16, octet string)
} sSrvAttrList_t;

// definition of all endpoints
typedef struct sSrvEndpointDef_tag
{
  // endpoints (ready for registering with ZStack server)
  int            endpointCount;  // # of endpoints detected
  AfRegisterReq  **ppEndpoints;  // pointer to array of endpoint pointers (e.g. *pEndpoints[])
  sSrvAttrList_t  **ppAttrLists; // pointer to array of attribute list pointers (e.g. *pAttrLists[])
} sSrvEndpointDef_t;

/*********************************************************************
 * EXTERNAL TYPEDEFS
 */

extern SimpleDescriptionFormat_t srvSimpleDesc;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Initialize Poll Control fields to defaults
extern uint32 giLongPollInterval;
extern uint16 giShortPollInterval;

sSrvEndpointDef_t *pgSrvEndpointDefs;

extern CONST char gszConfigTlgFileName[];
extern char gszConfigTlgPath[];

/*********************************************************************
 * FUNCTIONS
 */
extern sSrvEndpointDef_t * srvReadEndpointConfigFile( int *pStatus );
extern zclAttrRec_t * srvGetServerAttrListOnEndpoint( uint8 endpointId, int *pAttrCount );
extern uint32 * srvGetInClusterListOnEndpoint( uint8 endpointId, uint8 *pClusterCount );
extern zclAttrRec_t * srvFindAttributeRec( uint8 endpointId, uint16 clusterId, uint16 attrId );
extern void * srvFindAttributePtr( uint8 endpointId, uint16 clusterId, uint16 attrId );

/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* SERVEREP_H */


