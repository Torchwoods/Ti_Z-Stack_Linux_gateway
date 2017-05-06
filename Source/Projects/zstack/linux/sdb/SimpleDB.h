/**************************************************************************************************
 Filename:       SimpleDB.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    This file is a simple Database engine. Can work with multiple files.


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
#ifndef SIMPLE_DB_H
#define SIMPLE_DB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_types.h"

#define MAX_SUPPORTED_RECORD_SIZE 500
#define MAX_SUPPORTED_FILENAME    100


#define SDB_CHECK_KEY_EQUAL 0
#define SDB_CHECK_KEY_BIGGER 1
#define SDB_CHECK_KEY_SMALLER (-1)
#define SDB_CHECK_KEY_NOT_EQUAL 2
#define SDB_CHECK_KEY_ERROR 3

enum
{
	SDB_TYPE_TEXT,
	SDB_TYPE_BINARY
};

typedef void db_descriptor;

// compare the key against the record. Returns SDB_CHECK_KEY_EQUAL or one of the other CHECK_KEY status codes.
typedef int (* check_key_f)(void * record, void * key);

// get the size of the record
typedef uint32 (* get_record_size_f)(void * record);

// is the record deleted?
typedef bool (* check_deleted_f)(void * record);

// is the record a comment?
typedef bool (* check_ignore_f)(void * record);

// mark the record as deleted
typedef void (* mark_deleted_f)(void * record);

// consolidate (remove deleted records and check for record errors)
typedef bool (* consolidation_processing_f)(db_descriptor * db, void * record);

typedef struct
{
	char *  errorLocation;
	int     code;
	uint16  field;
} parsingResult_t;

// create the database (by the given name). A file header is optional
db_descriptor * sdb_init_db(char * name, get_record_size_f get_record_size, check_deleted_f check_deleted, check_ignore_f check_ignore, mark_deleted_f mark_deleted, consolidation_processing_f consolidation_processing, uint8 db_type, uint32 db_header_size, void *header);

// add a record to the database
bool sdb_add_record(db_descriptor * db, void * rec);

// delete a record from the database
void * sdb_delete_record(db_descriptor * db, void * key, check_key_f check_key);

// delete all records that match the key
void sdb_delete_records(db_descriptor * _db, void * key, check_key_f check_key);

// remove deleted records from the database. Note: database is closed an reopened.
bool sdb_consolidate_db(db_descriptor ** db);

// find a record in the database by key or contect (findfirst/next)
void * sdb_get_record(db_descriptor * db, void * key, check_key_f check_key, int * context);

// done with the record
bool sdb_release_record(void ** record);

// done with the database. Does not delete the database or contents, only closes it.
bool sdb_release_db(db_descriptor ** db);

// flush all data to the database file.
void sdb_flush_db(db_descriptor * db);

// modify a record (used internally, or if modifying a record to the same size, use delete/add if changing size).
uint8_t sdb_modify_last_accessed_record(db_descriptor * _db, void * record);

// get the first record
void * sdb_get_first_record(db_descriptor * db, int * context);

// get the next record
#define SDB_GET_NEXT_RECORD(_db, pContext) (sdb_get_record((_db), NULL, NULL, pContext))

// get a record that matches the key
#define SDB_GET_UNIQUE_RECORD(_db, _key, _check_key_func) (sdb_get_record((_db), (_key), (_check_key_func), NULL))

// error strings are placed into the file (just after the found record, but only on consolidate.
extern int sdbErrno;
extern const char * parsingErrorStrings[];

/* Macros for parsing records of a TEXT based database */
/* The following variables are expected to be defined (and initialized as specified):   */
/* bool end_of_record = false; */
/* char * pBuf = record; */

#define SDB_TXT_PARSER_RESULT_OK 0
#define SDB_TXT_PARSER_RESULT_REACHED_END_OF_RECORD 1
#define SDB_TXT_PARSER_RESULT_UNEXPECTED_CHARACTER_OR_TOO_LONG 2
#define SDB_TXT_PARSER_RESULT_FIELD_MISSING 3
#define SDB_TXT_PARSER_RESULT_HEX_UNEXPECTED_CHARACTER_OR_TOO_SHORT 4
#define SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE 5
#define SDB_TXT_PARSER_RESULT_MISSING_STARTING_QUOTE 6
#define SDB_TXT_PARSER_RESULT_MISSING_ENDING_QUOTE 7
#define SDB_TXT_PARSER_RESULT_STRING_TOO_LONG 8
#define SDB_TXT_PARSER_RESULT_MAX 8

#define SDB_INIT_PARSINGRESULT {SDB_TXT_PARSER_RESULT_OK, 0}

// get a hex field
void sdb_txt_parser_get_hex_field(char ** pBuf, uint8 * field, uint8 len, parsingResult_t * result);
void sdb_txt_parser_get_uint64_field(char ** pBuf, uint64_t * field, parsingResult_t * result);
void sdb_txt_parser_get_numeric_field(char ** pBuf, void * field, uint8 len, bool isSigned, parsingResult_t * result);
void sdb_txt_parser_get_quoted_string(char ** pBuf, char * field, uint8 size, parsingResult_t * result);

void sdb_txt_parser_move_to_next_field(char ** pBuf, parsingResult_t * result);

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_DB_H */

