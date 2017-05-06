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

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "configparser.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define MAX_STRING_LENGTH 180
#define MAX_UINT8_VALUE 255
#define MAX_UINT16_VALUE 65535
#define MAX_UINT32_VALUE 4294967295U
#define ERROR_FLAG -1
#define SUCCESS_FLAG 0

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Function Prototypes
 */
static char * buffCleaner( char * target );

static int arrayParse( char * pArrayList, uint8 size, int *result );

static int setArrayValue( uint8 size, uint32 *pLocation, int * pArrayOfNum,
                          uint8 type );

static void setValue( uint8 size, uint32 *pLocation, int newValue, uint8 type );

static void setStringValue( uint8 size, char * pLocation, char * pValue );

static void stringCopy( char * pCopy, char *pOriginal );

static int stringToInt( char * pValue );

/********************************************************************
 ********************************************************************/

/*********************************************************************
 * @fn      parseConfigFile
 *
 * @brief   Loop through the structure and set all the values.
 *
 * @param   pFileName - File name of the file to search
 *
 * @param   pItems - Array of structues containing items that need to be set
 *
 * @return  uint8 - Returns 0 if no errors occurred. Returns 1 otherwise.
 *
 *********************************************************************/
int parseConfigFile( char *pFileName, configTableItem_t * pItems,
                     uint8 numItems )
{
  int numericValue;
  char parseKey[MAX_STRING_LENGTH];
  char lineBuffer[MAX_STRING_LENGTH];

  // Open the file
  FILE *pOpenFile;
  pOpenFile = fopen( pFileName, "r" );

  if ( pOpenFile == NULL )
  {
    return ERROR_FLAG;
  }

  // Get line by line
  while ( fgets( lineBuffer, MAX_STRING_LENGTH, pOpenFile ) != NULL )
  {
    char *pUpdateVar;
    char *cleanedBuffer = buffCleaner( lineBuffer );

    // Check first character to see if the line is a comment ';'
    if ( lineBuffer[0] == ';' )
    {
      continue;
    }

    // Check to see if there is a keyword in the line by looking for "="
    pUpdateVar = strchr( cleanedBuffer, '=' );

    if ( pUpdateVar != NULL )
    {
      uint8 k;
      // Get a clean copy of the keyword
      pUpdateVar = buffCleaner( pUpdateVar );
      stringCopy( parseKey, cleanedBuffer );

      // Now search through the arrays for the pkeyword.
      for ( k = 0; k < numItems; ++k )
      {
        // If a keyword is a match convert it if needed, and set it into memory.
        if ( strcmp( pItems[k].pkeyword, parseKey ) == 0 )
        {
          // Value is a string
          if ( pItems[k].type == TYPE_STRING )
          {
            setStringValue( pItems[k].size, pItems[k].pAddr, pUpdateVar );
          }
          // Value is single numeric
          else
          {
            if ( pItems[k].size == 1 )
            {
              numericValue = stringToInt( pUpdateVar );

              if ( numericValue == ERROR_FLAG )
              {
                uiPrintf( "Value provided is not an uint type, skipping...\n" );
                break;
              }

              setValue( pItems[k].size, pItems[k].pAddr, numericValue,
                  pItems[k].type );
            }
            // Value is an array
            else
            {
              int errorCheck = 0;
              int arrayHolder[pItems[k].size];

              errorCheck = arrayParse( pUpdateVar, pItems[k].size,
                  arrayHolder );

              if ( errorCheck == ERROR_FLAG )
              {
                uiPrintfEx(trDEBUG, "Value provided is not an uint type, skipping...\n" );
                break;
              }

              errorCheck = setArrayValue( pItems[k].size, pItems[k].pAddr,
                  arrayHolder, pItems[k].type );

              if ( errorCheck == ERROR_FLAG )
              {
                uiPrintfEx(trDEBUG, "Error within array, skipping...\n" );
              }
            }
          }
        }
      }
    }
  }
  fclose( pOpenFile );
  return SUCCESS_FLAG;
}

/*********************************************************************
 * @fn      buffCleaner
 *
 * @brief   Strips whitespaces then an equal sign then any more whitespaces.
 *
 * @param   target - The char * that needs to be cleaned.
 *
 * @return  void
 *
 *********************************************************************/
static char * buffCleaner( char * pTarget )
{
  // Loop until no more white space, or equal signs.
  while ( pTarget[0] == ' ' || pTarget[0] == '=' )
  {
    ++pTarget;
  }
  return pTarget;
}

/*********************************************************************
 * @fn      arrayParse
 *
 * @brief   Converts a string that looks like an array into a numeric array.
 *
 * @param   pArrayList - The string that needs to be converted.
 *
 * @param   size - The number of elements in the array.
 *
 * @param   result - An array that the results will be stored in.
 *
 * @return  int - 0 for success, -1 for error
 *
 *********************************************************************/
static int arrayParse( char * pArrayList, uint8 size, int *pResult )
{
  uint8 i;
  uint32 converted;
  char *buff;

  buff = strtok( pArrayList, " {," );

  for ( i = 0; i < size; ++i )
  {
    // Convert the buffer depending on its type.
    converted = stringToInt( buff );

    if ( converted == -1 )
    {
      return ERROR_FLAG;
    }
    // Put the converted number into the result array. 
    pResult[i] = converted;
    buff = strtok( NULL, " {,}" );
  }
  return SUCCESS_FLAG;
}

/*********************************************************************
 * @fn      setArrayValue
 *
 * @brief   Sets the orignal array with the new variables from the configuration file.
 *
 * @param   size - The number of elements in the array
 *
 * @param   pLocation - The pLocation of the variable that needs to be set
 *
 * @param   pArrayOfNum - The array of numbers that will be set.
 *
 * @param   type - The type of variable to be set.
 *
 * @return  int - 0 for success, -1 for error
 *
 *********************************************************************/
static int setArrayValue( uint8 size, uint32 *pLocation, int * pArrayOfNum,
                          uint8 type )
{

  uint8 *type0;
  uint16 *type1;
  uint8 i;

  switch ( type )
  {
    case TYPE_UINT8:
      type0 = (uint8*) pLocation;
      // Check for errors.
      for ( i = 0; i < size; ++i )
      {
        if ( pArrayOfNum[i] > MAX_UINT8_VALUE )
        {
          uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
          return ERROR_FLAG;
        }
      }
      // Set the value
      for ( i = 0; i < size; ++i )
      {
        type0[i] = (uint8) pArrayOfNum[i];
      }
      break;

    case TYPE_UINT16:
      type1 = (uint16*) pLocation;
      // Check for errors
      for ( i = 0; i < size; ++i )
      {
        if ( pArrayOfNum[i] > MAX_UINT16_VALUE )
        {
          uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
          return ERROR_FLAG;
        }
      }
      // Set the value
      for ( i = 0; i < size; ++i )
      {
        type1[i] = (uint16) pArrayOfNum[i];
      }
      break;

    case TYPE_UINT32:
      // Check for errors
      for ( i = 0; i < size; ++i )
      {
        if ( pArrayOfNum[i] > MAX_UINT32_VALUE )
        {
          uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
          return ERROR_FLAG;
        }
      }
      // Set the value
      for ( i = 0; i < size; ++i )
      {
        pLocation[i] = (uint32) pArrayOfNum[i];
      }
      break;

    default:
      uiPrintfEx(trDEBUG, "Type provided is invalid\n" );
      break;
  }
  return SUCCESS_FLAG;
}

/*********************************************************************
 * @fn      setValue
 *
 * @brief   Sets the orignal variable with the updated information from the configuration file.
 *
 * @param   size - The number of elements in the array
 *
 * @param   pLocation - The pLocation of the variable that needs to be set
 *
 * @param   newValue - The new numeric value that will be set into the variable.
 * 
 * @param   type - The type of variable to be set.
 *
 * @return  void
 *
 *********************************************************************/
static void setValue( uint8 size, uint32 *pLocation, int newValue, uint8 type )
{
  switch ( type )
  {
    case TYPE_UINT8:
      // Check for errors
      if ( newValue > MAX_UINT8_VALUE )
      {
        uiPrintf( "Value provided is greater than variable can hold\n" );
        break;
      }
      // Set the value   
      *(uint8*) (pLocation) = (uint8) newValue;
      break;

    case TYPE_UINT16:
      // Check for errors
      if ( newValue > MAX_UINT16_VALUE )
      {
        uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
        break;
      }
      // Set the value
      *(uint16*) pLocation = (uint16) newValue;
      break;

    case TYPE_UINT32:
      // Check for errors
      if ( newValue > MAX_UINT32_VALUE )
      {
        uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
        break;
      }
      // Set the value
      *(uint32*) pLocation = (uint32) newValue;
      break;

    default:
      uiPrintfEx(trDEBUG,"Type provided is invalid\n" );
      break;
  }
}

/*********************************************************************
 * @fn      setStringValue
 *
 * @brief   Sets the orignal 
 *
 * @param   size - The number of elements in the array
 *
 * @param   pLocation - The pLocation of the variable that needs to be set
 *
 * @param   pValue - The value that will be set into the original variable.
 *
 * @return  void
 *
 *********************************************************************/
static void setStringValue( uint8 size, char * pLocation, char * pValue )
{
  uint8 i = 0;
  uint8 k = 0;
  uint8 totalSize = 0;
  uint8 sizeOfValue = 0;
  char *pStringTok;
  char stringTemp[MAX_STRING_LENGTH];

  pStringTok = strtok( pValue, " \n" );
  // Copy over all the tokens deleting any beginning, or trailing whitespace
  while ( pStringTok != NULL )
  {
    uint8 j = 0;
    sizeOfValue = strlen( pStringTok );

    if ( k != 0 )
    {
      stringTemp[i] = ' ';
      ++i;
      ++totalSize;
    }

    for ( j = 0; j < sizeOfValue; ++j )
    {
      if ( pStringTok[j] == ';' )
      {
        // Comment marker need to stop copying
        break;
      }
      stringTemp[i] = pStringTok[j];
      ++i;
      ++totalSize;
    }

    pStringTok = strtok( NULL, " \n" );
    ++k;
  }
  // Check for errors
  if ( size < totalSize )
  {
    uiPrintfEx(trDEBUG, "Value provided is greater than variable can hold\n" );
    return;
  }
  else if ( totalSize != 0 )
  {
    uint8 h;

    for ( i = 0; i < size; ++i )
    {
      pLocation[i] = '\0';
    }
    // Set the string into the memory address.

    for ( h = 0; h < totalSize; ++h )
    {
      pLocation[h] = stringTemp[h];
    }
  }
}

/*********************************************************************
 * @fn      stringToInt
 *
 * @brief   Converts a string into an integer.
 *
 * @param   pValue - The string that needs to be converted.
 *
 * @return  int - 0 for success, -1 for error.
 *
 *********************************************************************/
static int stringToInt( char * pValue )
{
  int result = 0;
  char *pEnd;
  char zeroCheck;

  if ( pValue[1] == 'x' )
  {
    zeroCheck = pValue[2];
    result = (int) strtol( (pValue + 2), &pEnd, 16 );
  }
  else
  {
    zeroCheck = pValue[0];
    result = (int) strtol( pValue, &pEnd, 10 );
  }
  // Check for errors
  if ( result == 0 )
  {
    if ( zeroCheck != '0' )
    {
      return ERROR_FLAG;
    }
  }
  else if ( errno != 0 )
  {
    errno = 0;
    return ERROR_FLAG;
  }

  return result;
}

/*********************************************************************
 * @fn      stringCopy
 *
 * @brief   Copies a string up untill an '=' or ' ' appears.
 *
 * @param   pOriginal - The string that needs to be converted.
 *
 * @param   pCopy - Where to store the copy of the stirng.
 *
 * @return  void
 *
 *********************************************************************/
static void stringCopy( char * pCopy, char *pOriginal )
{
  uint i = 0;

  while ( pOriginal[i] != '=' && pOriginal[i] != ' ' )
  {
    pCopy[i] = pOriginal[i];
    ++i;
  }
  pCopy[i] = '\0';
}

