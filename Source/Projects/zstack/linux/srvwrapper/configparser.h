#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

/**************************************************************************************************
 Filename:       configParser.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    Function to parse a .ini file and return the value given a name.


 Copyright 2013 - 2014 Texas Instruments Incorporated. All rights reserved.

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

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
  TYPE_UINT8 = 0, TYPE_UINT16 = 1, TYPE_UINT32 = 2, TYPE_STRING = 3
} dataTypes_t;

typedef struct
{
  void * pAddr;       // The address of the variable that needs to be changed.
  char * pkeyword;    // Name of the variable that needs to be changed.
  dataTypes_t type;     // Type of variable that will be stored.
  uint8 size;         // The number of elements that go into the variable
} configTableItem_t;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      parseConfigFile
 *
 * @brief   Function to parse a .ini configuration file, and set the vaule of any keyword found
 * that is in both the .ini file, and the passed structure.
 *
 * @param   pFileName - File name of the file to search and its path.
 *
 * @param   pItems - Array of structues containing items that need to be set
 *
 * @param   numItems - The number of structures held in the pItems array.
 *
 * @return  int - Returns 0 if no errors occurred. Returns -1 otherwise.
 *
 * Example - 
 * 
 * configTableItem_t test[10];
 * parseConfigFile( "/home/user/worspace/CONFIG.ini", test, 10);
 *
 *********************************************************************/
int parseConfigFile( char *pFileName, configTableItem_t * pItems,
    uint8 numItems );

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // CONFIGPARSER_H
