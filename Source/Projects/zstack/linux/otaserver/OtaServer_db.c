/**************************************************************************************************
 * Filename:       OtaServer_db.c
 * Description:
 *
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "utils.h"
#include "OtaServer_db.h"
#include "SimpleDBTxt.h"
#include "trace.h"

/*********************************************************************
 * LOCAL VARIABLES
 */

static db_descriptor * db;


/*********************************************************************
 * TYPEDEFS
 */
 
/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

static char * otaListComposeRecord(fileDbInfo_t *info, char * record)
{
  int i;
  int index = 0;
  char * temp_ptr = record;

  index = sprintf(temp_ptr, "  \"%s\" , 0x%02X , 0x%08X , 0x%08X , 0x%02X", //leave a space at the beginning to mark this record as deleted if needed later, or as bad format (can happen if edited manually). Another space to write the reason of bad format. 
  	info->fileName,
	info->executionType,
	info->executionDelay,
	info->executionTime,
	info->deviceNumber);
  

  for (i = 0; i < info->deviceNumber; i++) {
    temp_ptr +=  index;
    index = sprintf(temp_ptr, ", %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X ",
	BREAK_UINT64(info->deviceList[i],7),
	BREAK_UINT64(info->deviceList[i],6),
	BREAK_UINT64(info->deviceList[i],5),
	BREAK_UINT64(info->deviceList[i],4),
	BREAK_UINT64(info->deviceList[i],3),
	BREAK_UINT64(info->deviceList[i],2),
	BREAK_UINT64(info->deviceList[i],1),
	BREAK_UINT64(info->deviceList[i],0)
	);
  }

  temp_ptr +=  index;
  sprintf(temp_ptr, "\n");

  return record;
}

void otaListAddEntry( fileDbInfo_t *epInfo)
{
  char rec[MAX_SUPPORTED_RECORD_SIZE];

  otaListComposeRecord(epInfo, rec);

  uiPrintfEx(trDEBUG,"otaListAddEntry: Adding new entry %s\n", rec);

  sdb_add_record(db, rec);
}


void otaListReleaseRecord(fileDbInfo_t * ptr)
{
  free(ptr->fileName);
  if (ptr->deviceNumber > 0) {
    free(ptr->deviceList);
  }
  free(ptr);
}

static fileDbInfo_t * otaListParseRecord(char * record)
{
  int i;

  char * pBuf = record + 1; //+1 is to ignore the 'for deletion' mark that may just be added to this record.
  fileDbInfo_t * info;

  //static char fileName[MAX_SUPPORTED_FILE_NAME_LENGTH + 1];
  char * fileName = NULL; //[MAX_SUPPORTED_FILE_NAME_LENGTH + 1];
  uint64_t * deviceList = NULL;
  parsingResult_t parsingResult = {SDB_TXT_PARSER_RESULT_OK, 0};

  if (record == NULL)
  {
    return NULL;
  }

  info = malloc(sizeof(fileDbInfo_t));
  if (NULL == info) {
    return NULL;
  }

  fileName = malloc(sizeof(char) * (MAX_SUPPORTED_FILE_NAME_LENGTH + 1));
  if (NULL == fileName) {
    free(info);
    return NULL;
  } 

  sdb_txt_parser_get_quoted_string(&pBuf, fileName, 
		MAX_SUPPORTED_FILE_NAME_LENGTH, &parsingResult);
  info->fileName = fileName;
  sdb_txt_parser_get_numeric_field(&pBuf, (uint8_t *)&info->executionType, 1, 
	false, &parsingResult);
  sdb_txt_parser_get_numeric_field(&pBuf, (uint32_t *)&info->executionDelay, 4, 
	false, &parsingResult);
  sdb_txt_parser_get_numeric_field(&pBuf, (uint32_t *)&info->executionTime, 4, 
	false, &parsingResult);
  sdb_txt_parser_get_numeric_field(&pBuf, (uint8_t *)&info->deviceNumber, 1, 
	false, &parsingResult);

  if (info->deviceNumber != 0) {
    //Who will free this memory :(
    deviceList = malloc(sizeof(uint64_t) * info->deviceNumber);

    for (i = 0; i < info->deviceNumber; i++) {
      sdb_txt_parser_get_uint64_field(&pBuf, (uint64_t *)&deviceList[i], 
	&parsingResult);
    }

    info->deviceList = deviceList;
  }
#if 0
	sdb_txt_parser_get_numeric_field(&pBuf, &info.endpoint, 1, false, &parsingResult);
	sdb_txt_parser_get_numeric_field(&pBuf, (uint8_t *)&info.profileID, 2, false, &parsingResult);
	sdb_txt_parser_get_numeric_field(&pBuf, (uint8_t *)&info.deviceID, 2, false, &parsingResult);
	sdb_txt_parser_get_numeric_field(&pBuf, &info.version, 1, false, &parsingResult);
	sdb_txt_parser_get_numeric_field(&pBuf, &info.status, 1, false, &parsingResult);
#endif

  if ((parsingResult.code != SDB_TXT_PARSER_RESULT_OK) && 
	(parsingResult.code != SDB_TXT_PARSER_RESULT_REACHED_END_OF_RECORD))
  {
    sdbtMarkError( db, record, &parsingResult);
    return NULL;
  }

  return info;
}

static int otaListCheckKeyFilename(char * record, char * key)
{
  fileDbInfo_t * epInfo;
  int result = SDB_CHECK_KEY_NOT_EQUAL;

  epInfo = otaListParseRecord(record);
  if (epInfo == NULL)
  {
    return SDB_CHECK_KEY_ERROR;
  }

  if (strcmp(epInfo->fileName, *(char **)key) == 0)
  {
    result = SDB_CHECK_KEY_EQUAL;
  }
 
  otaListReleaseRecord(epInfo);

  return result;
}

fileDbInfo_t * otaListRemoveEntryByFilename( char * fileName)
{
  char * key = fileName; 
  char * rec;
  fileDbInfo_t * fileDb;
  rec = sdb_delete_record(db, &key, (check_key_f)otaListCheckKeyFilename);
  
  fileDb = otaListParseRecord(sdb_delete_record(db, &key , 
	(check_key_f)otaListCheckKeyFilename));

  sdb_release_record((void **)&rec);
  return fileDb; 
}

fileDbInfo_t * otaListGetDeviceByFilename(char * fileName)
{
  char * rec;
  char * key = fileName;

  rec = SDB_GET_UNIQUE_RECORD(db, &key, (check_key_f)otaListCheckKeyFilename);

  if (rec == NULL)
  {
    return NULL;
  }

  return otaListParseRecord(rec);
}

uint32_t otaListNumEntries(void)
{  
  return sdbtGetRecordCount(db);
}


fileDbInfo_t * otaListGetNextEntry(int *context)
{
  char * rec;
  fileDbInfo_t *info;

  do
  {
    rec = SDB_GET_NEXT_RECORD(db,context);
    if (rec == NULL)
    {
      return NULL;
    }

    info = otaListParseRecord(rec);
  } while (info == NULL); //in case of a bad-format record - skip it and read the next one

  return info;
}


bool otaListInitDatabase( char * dbFilename )
{
  int context = 0;
	
  db = sdb_init_db(dbFilename, sdbtGetRecordSize, sdbtCheckDeleted, sdbtCheckIgnored, sdbtMarkDeleted, (consolidation_processing_f)sdbtErrorComment, SDB_TYPE_TEXT, 0, NULL);

  if (NULL == db) return FALSE;

  while (otaListGetNextEntry(&context) != NULL); //read all the db lines to check for errors
  sdb_consolidate_db(&db); //add comprehensive error comments where needed; delete lines that are marked for deletion

  return TRUE;
}


bool otaListCloseDatabase(void)
{
  return sdb_release_db(&db);
}
