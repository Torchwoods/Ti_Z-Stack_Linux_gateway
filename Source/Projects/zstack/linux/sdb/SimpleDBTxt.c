/**************************************************************************************************
 Filename:       SimpleDBTxt.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    This file is the text implementation of the simple Database engine. Can work with multiple files.


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
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "SimpleDBTxt.h"

uint32 sdbtGetRecordSize(void * record)
{
	return strlen(record);
}

bool sdbtCheckDeleted(void * record)
{
	return (((char *)record)[0] == SDBT_DELETED_LINE_CHARACTER);
}

bool sdbtCheckIgnored(void * record)
{
	return ((((char *)record)[0] == SDBT_COMMENT_CHARACTER) || (((char *)record)[0] == SDBT_PENDING_COMMENT_FORMAT_CHARACTER));
}

void sdbtMarkDeleted(void * record)
{
	((char *)record)[0] = SDBT_DELETED_LINE_CHARACTER;
}

int sdbtGetRecordCount(db_descriptor * db)
{  
  int recordCnt = 0;
  int context = 0;
  char * rec;
  
  rec = sdb_get_first_record(db, &context);
  
  while (rec != NULL)
  {
    // count and release the found record  
    sdb_release_record((void **)(&rec));
    recordCnt++;

    // find the next one
    rec = SDB_GET_NEXT_RECORD(db, &context);
  }
  
  return recordCnt;
}

/**************************************************************************************************
 *
 * @fn      sdbtErrorComment
 *
 * @brief   just copy the record, or copy it with an error line
 *
 * @param   db - database
 * @param   record - the record to copy or do error comment on
 *
 * @return  
 *
 **************************************************************************************************/
bool sdbtErrorComment(db_descriptor * db, char * record)
{
  bool rc;

  parsingResult_t parsingResult = {SDB_TXT_PARSER_RESULT_OK, 0};
  char * pBuf = record + 1;
  char comment[MAX_SUPPORTED_RECORD_SIZE];
  uint8_t errorCode;
  uint16_t errorOffset;
  char tempstr[3] = {SDBT_COMMENT_CHARACTER, '\n', '\0'};

  if (record[0] == SDBT_PENDING_COMMENT_FORMAT_CHARACTER)
  {
    record[0] = SDBT_COMMENT_CHARACTER;
    sprintf(comment,"%c-------\n", SDBT_COMMENT_CHARACTER);
    rc = sdb_add_record(db, comment) && sdb_add_record(db, record);
    
    if (rc )
    {
      sdb_txt_parser_get_numeric_field(&pBuf, &errorCode, 1, FALSE, &parsingResult);
      if (errorCode > SDB_TXT_PARSER_RESULT_MAX)
      {
        parsingResult.code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
        parsingResult.errorLocation = pBuf - 2;
      }
      else
      {
        sdb_txt_parser_get_numeric_field(&pBuf, (uint8 *)&errorOffset, 2, FALSE, &parsingResult);
        if (errorOffset >= strlen(record))
        {
          parsingResult.code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
          parsingResult.errorLocation = pBuf - 2;
        }
      }
      if (parsingResult.code == SDB_TXT_PARSER_RESULT_OK)
      {
        sprintf(comment,"%cERROR:%*s %s\n", SDBT_COMMENT_CHARACTER, errorOffset - 7 + 1, "^", parsingErrorStrings[errorCode]);
      }
      else
      {
        sprintf(comment,"%c%*s %s (%s)\n", SDBT_COMMENT_CHARACTER, (int)(parsingResult.errorLocation - record) - 1 + 1, "^", "BAD_ERROR_DESCRIPTION_HEADER", parsingErrorStrings[parsingResult.code]);
      }
      
      rc = sdb_add_record(db, comment) && sdb_add_record(db, tempstr);
    }
  }

  // deleted character
  else if( record[0] != SDBT_DELETED_LINE_CHARACTER )
  {
    rc = sdb_add_record(db, record);
  }
  
  return rc;
}


void sdbtMarkError(db_descriptor * db, char * record, parsingResult_t * parsingResult)
{
	//mark record as 'bad format'
	record[0] = SDBT_PENDING_COMMENT_FORMAT_CHARACTER;
	record[1] = '0' + parsingResult->code;
	record[2] = ',';
	sprintf(record + 3, "%3d", (int)(parsingResult->errorLocation - record));
	record[6] = ','; //must overwrite the '\0' from the above sprintf
	sdb_modify_last_accessed_record(db, record);
}


