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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <pthread.h>

#include "SimpleDB.h"
#include "trace.h"

/**************************************************************************************************
 * Defines and Structures
 **************************************************************************************************/

#define TEMP_FILENAME_EXTENTION_LENGTH 4

typedef struct
{
  char name[MAX_SUPPORTED_FILENAME + TEMP_FILENAME_EXTENTION_LENGTH + 1];
  FILE * file;
  long last_accessed_record_start_file_pointer;
  uint32 last_accessed_record_size;
  get_record_size_f get_record_size;
  check_deleted_f check_deleted;
  check_ignore_f check_ignore;
  mark_deleted_f mark_deleted;
  consolidation_processing_f consolidation_processing;
  uint8 type;
  uint32 db_header_size;
  void *header;
  pthread_mutex_t mutex;
} _db_descriptor;

/**************************************************************************************************
 * Globals
 **************************************************************************************************/

int sdbErrno;

/**************************************************************************************************
 * Function Prototypes
 **************************************************************************************************/

int sdb_lock_init( _db_descriptor * db );
int sdb_lock_destroy( _db_descriptor * db );
void sdb_lock( _db_descriptor * db );
void sdb_unlock( _db_descriptor * db );


/**************************************************************************************************
 * Code
 **************************************************************************************************/

int sdb_lock_init( _db_descriptor * db )
{
  return pthread_mutex_init( &db->mutex, NULL );
}

int sdb_lock_destroy( _db_descriptor * db )
{
  return pthread_mutex_destroy( &db->mutex );
}

void sdb_lock( _db_descriptor * db )
{
  pthread_mutex_lock( &db->mutex );
}

void sdb_unlock( _db_descriptor * db )
{
  pthread_mutex_unlock( &db->mutex );
}

/**************************************************************************************************
 *
 * @fn      sdb_init_db
 *
 * @brief   Open a database file and prepare for reading/writing records
 *
 * @param   name            - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   get_record_size - function that computes record size for this database
 * @param   check_deleted   - is this record deleted?
 * @param   check_ignore    - ingore this record (it's a comment)
 * @param   mark_deleted    - mark this record as deleted
 * @param   consolidation_processing - used during database garbage collection
 * @param   db_type         - always SDB_TYPE_TEXT
 * @param   db_header_size  - how much to skip at start of database (non-records)
 * @param   header          - pointer to the header
 *
 * @return  an allocate database descriptor
 *
 **************************************************************************************************/
db_descriptor * sdb_init_db(char * name, get_record_size_f get_record_size, check_deleted_f check_deleted, check_ignore_f check_ignore, mark_deleted_f mark_deleted, consolidation_processing_f consolidation_processing, uint8 db_type, uint32 db_header_size, void *header)
{
  _db_descriptor * db = NULL;

  // name too long
  if( strlen(name) > MAX_SUPPORTED_FILENAME )
    return NULL;

  db = malloc(sizeof(_db_descriptor));
  
  // can't allcoate memory
  if (db == NULL)
  {
    return NULL;
  }
  
  // need to be able to create the mutex
  if( sdb_lock_init( db ) != 0 )
  {
    free(db);
    return NULL;
  }

  strcpy(db->name, name);
      
  // first try to open for rd/wr, then try to create for rd/wr
  db->file = fopen(name,(db_type == SDB_TYPE_TEXT) ? "r+t" : "r+b");
  if ((db->file == NULL) && (errno == ENOENT))
  {
    // no database file, create it
    db->file = fopen(name,(db_type == SDB_TYPE_TEXT) ? "w+t" : "w+b");
    if(db->file != NULL)
    {
      // if creating, create the header
      sdb_lock( db );
      fwrite(header, db_header_size, 1, db->file);
      fflush( db->file );
      sdb_unlock( db );
    }
  }

  // if we can't create the file, give up
  if (db->file == NULL)
  {
    pthread_mutex_destroy( &db->mutex );
    free(db);
    return NULL;
  }

  // fill in database
  db->last_accessed_record_start_file_pointer = 0;
  db->last_accessed_record_size = 0;
  db->get_record_size = get_record_size;
  db->check_deleted = check_deleted;
  db->check_ignore = check_ignore;
  db->mark_deleted = mark_deleted;
  db->consolidation_processing = consolidation_processing;
  db->type = db_type;
  db->db_header_size = db_header_size;  // ignore header part of the file for records
  if(db_header_size)
  {
    db->header = malloc(db_header_size);
    if(db->header)
    {
      strncpy(db->header, header, db_header_size);
    }
  }

  return (db_descriptor *)db;
}

/**************************************************************************************************
 *
 * @fn      sdb_release_db
 *
 * @brief   Close the database, and free the descriptor. Doesn't change the contents of the
 *          database file.
 *
 * @param   db - pointer to the database descriptor (pointer is set to NULL on exit)
 *
 * @return  TRUE if file closed, FALSE if already closed.
 *
 **************************************************************************************************/
bool sdb_release_db(db_descriptor ** _db)
{
  _db_descriptor * db = *_db;
  
  if (db != NULL)
  {
    sdb_lock( db );
    fclose(db->file);
    sdb_unlock( db );
    sdb_lock_destroy( db );
    free(db);
    *_db = NULL;
    return TRUE;
  }
  return FALSE;
}

/**************************************************************************************************
 *
 * @fn      sdb_flush_db
 *
 * @brief   Flush all contents to the database. Does nothing as flushing is done at every change.
 *
 * @param   db - pointer to the database descriptor (pointer is set to NULL on exit)
 *
 * @return  nothing
 *
 **************************************************************************************************/
void sdb_flush_db(db_descriptor * db)
{
  // do nothing.
  // In the current implementation,  flushing is done automatically after every change
}

/**************************************************************************************************
 *
 * @fn      sdb_release_record
 *
 * @brief   Done with record, release it's contents.
 *
 * @param   ppRecord - pointer to the record pointer. 
 *
 * @return  record released and set to NULL
 *
 **************************************************************************************************/
bool sdb_release_record(void ** ppRecord)
{
  free(*ppRecord);
  *ppRecord = NULL;
  return TRUE;
}

/**************************************************************************************************
 *
 * @fn      sdb_add_record
 *
 * @brief   Add a record to the database.
 *
 * @param   db  - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   rec - pointer to the record 
 *
 * @return  returns TRUE if workd, false if something FAILED
 *
 **************************************************************************************************/

bool sdb_add_record(db_descriptor * _db, void * rec)
{
  _db_descriptor * db = _db;
  bool worked = TRUE;
  
  if(!db)
    return FALSE;

  sdb_lock( db );

  // seek to the end of the file
  if (fseek(db->file, 0, SEEK_END) != 0)
    worked = FALSE;

  if(worked)
  {
    db->last_accessed_record_start_file_pointer = ftell(db->file);
    db->last_accessed_record_size = db->get_record_size(rec);
  
    worked = ((fwrite(rec, db->get_record_size(rec), 1, db->file) == 1) &&
       (fflush(db->file) == 0));
  }
  
  sdb_unlock( db );

  return worked;
}

/**************************************************************************************************
 *
 * @fn      sdb_modify_last_accessed_record
 *
 * @brief   modify the last written record. Must be same size.
 *
 * @param   db  - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   rec - pointer to the record 
 *
 * @return  record released and set to NULL
 *
 **************************************************************************************************/
bool sdb_modify_last_accessed_record(db_descriptor * _db, void * record)
{
  _db_descriptor * db = _db;
  bool worked;
  
  if(!db)
    return FALSE;

  sdb_lock( db );
  worked = !((db->get_record_size(record) != db->last_accessed_record_size) ||
     ((fseek(db->file, db->last_accessed_record_start_file_pointer, SEEK_SET) != 0) ||
     (fwrite(record, db->last_accessed_record_size, 1, db->file) != 1) ||
     (fflush(db->file) != 0)));
  sdb_unlock( db );
  
  return worked;
}


/**************************************************************************************************
 *
 * @fn      sdb_delete_record
 *
 * @brief   mark a record as deleted
 *
 * @param   db  - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   key - pointer to the key of the record to delete
 * @param   check_key - function that compares each record to the key
 *
 * @return  returns pointer to deleted record, or NULL if something failed
 *
 **************************************************************************************************/
void * sdb_delete_record(db_descriptor * _db, void * key, check_key_f check_key)
{
  _db_descriptor * db = _db;
  void * rec;
  
  if(!db)
    return NULL;

  sdbErrno = 0;
  
  // get the record out of the database into local memory
  rec = sdb_get_record(db, key, check_key, NULL);

  if (rec != NULL)
  {
    db->mark_deleted(rec);
    
    if (!sdb_modify_last_accessed_record(db, rec))
    {
      sdbErrno = 1;
      sdb_release_record(&rec);
    }
  }

  return rec;
}

/**************************************************************************************************
 *
 * @fn      sdb_delete_records
 *
 * @brief   mark all record that match as deleted
 *
 * @param   db  - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   key - pointer to the key of the record to delete
 * @param   check_key - function that compares each record to the key
 *
 * @return  returns pointer to deleted record, or NULL if something failed
 *
 **************************************************************************************************/
void sdb_delete_records(db_descriptor * _db, void * key, check_key_f check_key)
{
  _db_descriptor * db = _db;
  void * rec;
  int _context = 0;
  int found;

  // no database  
  if(!db)
    return;

  // global error #
  sdbErrno = 0;

  do
  {
    found = FALSE;

    // get the record out of the database into local memory
    rec = sdb_get_record(db, key, check_key, &_context);

    // if found, mark as deleted
    if (rec != NULL)
    {
      found = TRUE;
      db->mark_deleted(rec);
    
      if (!sdb_modify_last_accessed_record(db, rec))
      {
        sdbErrno = 1;
      }

      sdb_release_record(&rec);
    }
  } while (found);  

}

/**************************************************************************************************
 *
 * @fn      sdb_rename_db
 *
 * @brief   used during consolidation
 *
 * @param   db  - pointer to the database descriptor (pointer is set to NULL on exit)
 * @param   newName - new name for the database
 *
 * @return  TRUE if worked, FALSE if failed.
 *
 **************************************************************************************************/
bool sdb_rename_db(db_descriptor * _db, char * newName)
{
  _db_descriptor * db = _db;
  int rc;
  
  sdb_lock( db );
  fclose(db->file);
  
  rc = rename(db->name, newName);
  if (rc != 0)
  {
    sdb_unlock( db );
    return FALSE;
  }
  
  strcpy(db->name, newName);
  
  db->file = fopen(db->name,(db->type == SDB_TYPE_TEXT) ? "r+t" : "r+b");	
  
  if (db->file == NULL)
  {
    sdb_unlock( db );
    return FALSE;
  }
  
  sdb_unlock( db );
  return TRUE;
}

/**************************************************************************************************
 *
 * @fn          sdb_consolidate_db
 *
 * @brief       Removes deleted entries, only done on startup
 *
 * @param       _db address of the pointer to the db_descriptor
 *
 * @return      TRUE if consolidation worked, or if something failed but the original DB can
 *              still be used.
 *
 **************************************************************************************************/
bool sdb_consolidate_db(db_descriptor ** _db)
{
  _db_descriptor * db = *_db;
  _db_descriptor * tempDb;
  void * rec;
  int context;
  int rc;
  static char tempfilename[MAX_SUPPORTED_FILENAME + TEMP_FILENAME_EXTENTION_LENGTH + 1];

  strcpy(tempfilename, db->name);
  strcat(tempfilename, ".tmp");
  
  // remove temp file if already there
  rc = remove(tempfilename);
  if ((rc != 0) && (errno != ENOENT))
  {
    uiPrintf( "%s: Warning: Failed to delete old temp db (errno=%d), %s\n", __func__, errno, tempfilename );
    return TRUE;
  }
    
  tempDb = sdb_init_db(tempfilename, db->get_record_size, db->check_deleted, db->check_ignore, db->mark_deleted,
             db->consolidation_processing, db->type, db->db_header_size, db->header);
  
  if (tempDb == NULL)
  {
    uiPrintf( "\n%s: Warning: Failed to initialize temp db, %s\n", __func__, tempfilename );
    return TRUE;
  }
  
  db->check_ignore = NULL;    //only deleted lines should be removed. Ignored lines should stay.
  
  rec = sdb_get_first_record(db, &context);
  rc = TRUE;
  while ((rec != NULL) && (rc == TRUE))
  {
    if (db->consolidation_processing != NULL)
    {
      rc = db->consolidation_processing(tempDb, rec);
    }
    else
    {
      rc = sdb_add_record(tempDb, rec);
    }
    sdb_release_record((void **)(&rec));
    
    rec = SDB_GET_NEXT_RECORD(db, &context);
  }
  
  if (rc != TRUE)
  {
    uiPrintf( "\n%s: Warning: Failed to rebuild db %s\n", __func__, tempfilename );
    
    sdb_release_db((db_descriptor **)&tempDb);
    
    rc = remove(tempfilename);
    
    if ((rc != 0) && (errno != ENOENT))
    {
      uiPrintf( "\n%s: Warning: Failed to delete temp db (errno=%d), %s\n", __func__, errno, tempfilename);
    }
    
    return TRUE;
  }
  
  // keep a copy of original name
  strcpy(tempfilename, db->name);
  
  sdb_release_db(_db);
  (*_db) = tempDb;

  // remove original file
  rc = remove(tempfilename);

  if (rc != 0)
  {
    uiPrintf( "\n%s: ERROR: Failed to rename temp db, %s\n", __func__, tempfilename );
  	return FALSE;
  }
  
  // rename new file to original name
  rc = sdb_rename_db(*_db, tempfilename);
  
  if (rc != TRUE)
  {
    uiPrintf( "\n%s: ERROR: Failed to rename temp db, %s\n", __func__, tempfilename);
    return FALSE;
  }
  
  // done
  return TRUE;  // TRUE if worked
}

/**************************************************************************************************
 *
 * @fn      sdb_get_record
 *
 * @brief   Get a record by one or more criteria
 *
 * @param   db        - database descriptor
 * @param   key       - the key to search for (or NULL if no key)
 * @param   check_key - the function to use to compare each record
 * @param   context   - file offset of the current record (set to 0 for first entry)
 *
 * @return  pointer to the allocated record
 *
 **************************************************************************************************/
void * sdb_get_record( db_descriptor * _db, void * key, check_key_f check_key, int * context )
{
  _db_descriptor * db = _db;
  static char rec[MAX_SUPPORTED_RECORD_SIZE];   // used temporarily to get the record (before it's copied and allocated)
  bool found = FALSE;
  int _context;
  void *pRec;
  
  // no database, no record
  if( db == NULL )
  {
    return NULL;
  }
  
  sdb_lock( db );

  if ( context != NULL )
  {
    _context = *context;    // start at last context
  }
  else
  {
    _context = 0; // start at beginning
  }

  if ( _context == 0 )
  {
    _context = db->db_header_size;  // skip header
  }
  
  if ( ftell( db->file ) != _context )
  {
    fseek( db->file, _context, SEEK_SET );
  }
  
  if ( db->type == SDB_TYPE_TEXT )
  {
    // find the first record that matches the criteria
    while ( (!found) && (fgets( rec, sizeof( rec ), db->file ) != NULL) )
    {
      db->last_accessed_record_start_file_pointer = _context;
      db->last_accessed_record_size = db->get_record_size( rec );
      _context = ftell( db->file );

      if ( rec[strlen( rec ) - 1] != '\n' )
      {
        sdb_unlock( db );
        return NULL;
      }

      // if deleted, ignore
      if ( db->check_deleted && db->check_deleted( rec ) )
      {
        continue;
      }

      // if commented out, ignore
      if ( db->check_ignore && db->check_ignore( rec ) )
      {
        continue;
      }

      // if checking for a key, and the key doesn't match, ignore
      if ( check_key && (check_key( rec, key ) != SDB_CHECK_KEY_EQUAL) ) 
      {
        continue;
      }

      // passed all criteria, found!
    	found = TRUE;
    }
    
    // if found, return a copy, so static "rec" buffer can be used for other purposes
    if(found)
    {
      pRec = malloc( strlen(rec) + 1 );
      if ( pRec )
      {
        strcpy( pRec, rec );
      }
    }
  }
  else //db->type == SDB_TYPE_BINARY
  {
    // Does not support binary files
  }
  
  if ( !found )
  {
    sdb_unlock( db );
  	return NULL;
  }
  
  if ( context != NULL )
  {
    *context = _context;	
  }
  
  sdb_unlock( db );
  return pRec;
}

// get the first record
void * sdb_get_first_record( db_descriptor * db, int * context )
{
  // get the first record
  *context = 0;
  return sdb_get_record(db, NULL, NULL, context);
}

// error strings for parsing
const char * parsingErrorStrings[] = 
{
  "SDB_TXT_PARSER_RESULT_OK",
  "SDB_TXT_PARSER_RESULT_REACHED_END_OF_RECORD",
  "SDB_TXT_PARSER_RESULT_UNEXPECTED_CHARACTER_OR_TOO_LONG",
  "SDB_TXT_PARSER_RESULT_FIELD_MISSING",
  "SDB_TXT_PARSER_RESULT_HEX_UNEXPECTED_CHARACTER_OR_TOO_SHORT",
  "SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE",
  "SDB_TXT_PARSER_RESULT_MISSING_STARTING_QUOTE",
  "SDB_TXT_PARSER_RESULT_MISSING_ENDING_QUOTE",
  "SDB_TXT_PARSER_RESULT_STRING_TOO_LONG",
};

/**************************************************************************************************
*
* @fn      sdb_txt_parser_move_to_next_field
*
* @brief   Moves to next field. Skips all white-space, including commas.
*          Gives error if unexpected character or end of file.
*
* @param   pBuf   - pointer to the beginning current position in the record
* @param   result - result of the parsing
*
* @return  error and location
*
**************************************************************************************************/
void sdb_txt_parser_move_to_next_field(char ** pBuf, parsingResult_t * result)
{
  // skip spaces, tabs, commas, linefeeds and carriage returns. Stop on NULL or next field.
  while ((result->code == SDB_TXT_PARSER_RESULT_OK) && (**pBuf != ',') && (**pBuf != '\0'))
  {
    if ((**pBuf != ' ') && (**pBuf != '\t') && (**pBuf != '\n')  && (**pBuf != '\r'))
    {
      result->code = SDB_TXT_PARSER_RESULT_UNEXPECTED_CHARACTER_OR_TOO_LONG;
      result->errorLocation = *pBuf;
    }
    else
    {
      (*pBuf)++;
    }
  }

  // at end of line  
  if (**pBuf != '\0')
  {
    (*pBuf)++;
  }

}

/**************************************************************************************************
*
* @fn      sdb_txt_parser_get_hex_field
*
* @brief   Get a hex number from (11:22:33:44) form to 11223344 (byte array) form
*
* @param   pBuf   - pointer to the field
* @param   field  - pointer to the output byte array
* @param   len    - length of byte array (up to 255 bytes)
* @param   result - results of the parsing
*
* @return  error and location
*
**************************************************************************************************/
void sdb_txt_parser_get_hex_field(char ** pBuf, uint8 * field, uint8 len, parsingResult_t * result)
{
  int i;
  unsigned long tempNum;
  
  if (**pBuf == '\0')
  {
    result->code = SDB_TXT_PARSER_RESULT_FIELD_MISSING;
    result->errorLocation = *pBuf;
  }
  else
  {
    for (i = 0; (result->code == SDB_TXT_PARSER_RESULT_OK) && (i < len); i++)
    {
      tempNum = strtoul(*pBuf, pBuf, 16);
      if ((errno == ERANGE) | (tempNum > 0xFF))
      {
        result->code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
        result->errorLocation = (*pBuf) - 1;
      }
      else
      {
        if(field)
        {
          field[(len - 1) - i] = (uint8)tempNum;
        }
        
        if (i < (len - 1))
        {
          if (**pBuf != ':')
          {
            result->code = SDB_TXT_PARSER_RESULT_HEX_UNEXPECTED_CHARACTER_OR_TOO_SHORT;
            result->errorLocation = *pBuf;
          }
          else
          {
            (*pBuf)++;
          }
        }
      }
    }
    
    if (result->code == SDB_TXT_PARSER_RESULT_OK)
    {
      result->field++;
      sdb_txt_parser_move_to_next_field(pBuf, result);
    }
  }
}

/**************************************************************************************************
*
* @fn      sdb_txt_parser_get_uint64_field
*
* @brief   Get a hex number from (11:22:33:44) form to 11223344 (byte array) form
*
* @param   pBuf   - pointer to the field
* @param   field  - pointer to the output byte array
* @param   len    - length of byte array
* @param   result - results of the parsing
*
* @return  error and location in result
*
**************************************************************************************************/
void sdb_txt_parser_get_uint64_field(char ** pBuf, uint64_t * field, parsingResult_t * result)
{
  int i;
  int len = 8;   // always 8 bytes for a uint64_t
  unsigned long tempNum;
  
  if (**pBuf == '\0')
  {
    result->code = SDB_TXT_PARSER_RESULT_FIELD_MISSING;
    result->errorLocation = *pBuf;
  }
  else
  {
    for (i = 0; (result->code == SDB_TXT_PARSER_RESULT_OK) && (i < len); i++)
    {
      tempNum = strtoul(*pBuf, pBuf, 16);   // will skip preceeding spaces
      if ((errno == ERANGE) | (tempNum > 0xFF))
      {
        result->code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
        result->errorLocation = (*pBuf) - 1;
      }
      else
      {
        // stored in database in big endian (human readable) form
        if(field)
        {
          *field = ((*field) * 256) + tempNum;
        }
        
        // expecting another colon
        if (i < (len - 1))
        {
          if (**pBuf != ':')
          {
            result->code = SDB_TXT_PARSER_RESULT_HEX_UNEXPECTED_CHARACTER_OR_TOO_SHORT;
            result->errorLocation = *pBuf;
          }
          else
          {
            (*pBuf)++;  // skip the colon
          }
        }
      }
    }
    
    if (result->code == SDB_TXT_PARSER_RESULT_OK)
    {
      result->field++;
      sdb_txt_parser_move_to_next_field(pBuf, result);
    }
  }
}

/**************************************************************************************************
 *
 * @fn      sdb_txt_parser_get_numeric_field
 *
 * @brief   Get a number. May be any normal C form (e.g. 0, 0x15, 0x12e5, 34561, -15 if isSigned is true)
 *
 * @param   pBuf     - pointer to the buffer
 * @param   field    - pointer to where the result will go (may be 8-32 bits, eg. uint8, int8, uint16, etc..),
 *                     may be null if just skipping field
 * @param   len      - length of field in bytes
 * @param   isSigned - must be set to allow for signed number. signed numbers cannot be hex.
 * @param   result   - file offset of the current record
 *
 * @return  error and location in result
 *
 **************************************************************************************************/
void sdb_txt_parser_get_numeric_field(char ** pBuf, void * field, uint8 len, bool isSigned, parsingResult_t * result)
{
  union
  {
  	signed long sNum;
  	unsigned long uNum;
  } temp;
  int base;
  
  if (**pBuf == '\0')
  {
    result->code = SDB_TXT_PARSER_RESULT_FIELD_MISSING;
    result->errorLocation = *pBuf;
  }
  else if (result->code == SDB_TXT_PARSER_RESULT_OK)
  {
    base = 10;

    while(base)
    {
      if (isSigned)
      {
        temp.sNum = strtol(*pBuf, pBuf, 0);
        
        if ((errno == ERANGE) || (temp.sNum > (((signed long)0x7FFFFFFF) << (8 * (4 -len)))) || 
           (temp.sNum < (((signed long)0x80000000) << (8 * (4 - len)))))
        {
          result->code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
          result->errorLocation = (*pBuf) - 1;
        }
      }
      else
      {
        temp.uNum = strtoul(*pBuf, pBuf, base);
        
        if ((errno == ERANGE) || (temp.uNum > (((unsigned long)0xFFFFFFFF) >> (8 * (4 - len)))))
        {
          result->code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
          result->errorLocation = (*pBuf) - 1;
        }
      }
      
      // if we found a 0x, then convert to base 16
      if(**pBuf == 'x')
      {
        base = 16;
        isSigned = FALSE;
        (*pBuf)++;
      }
      else
        base = 0;   // done
    }
    
    // From this point, only temp.uNum is used. It actually access temp.sNum, since temp is a union
    if (result->code == SDB_TXT_PARSER_RESULT_OK)
    {
      if(isSigned)
      {
        if( len == 1 && (temp.sNum >= -128 && temp.sNum <- 127) )
        {
          if(field)
            *(int8_t *)field = temp.sNum;
          temp.uNum = 0;
        }
        if( len == 2 )
        {
          if(field)
            *(int16_t *)field = temp.sNum;
          temp.uNum = 0;
        }
        if( len == 4 )
        {
          if(field)
            *(int32_t *)field = temp.sNum;
          temp.uNum = 0;
        }
      }
      else
      {
        if(len == 1 && temp.uNum <= 255 )
        {
          if(field)
            *(uint8_t *)field = temp.uNum;
          temp.uNum = 0;
        }
        if(len == 2 && temp.uNum <= 65535 )
        {
          if(field)
            *(uint16_t *)field = temp.uNum;
          temp.uNum = 0;
        }
        if(len == 4)
        {
          if(field)
            *(uint32_t *)field = temp.uNum;
          temp.uNum = 0;
        }
      }
      
      if (temp.uNum != 0)
      {
        result->code = SDB_TXT_PARSER_RESULT_VALUE_OUT_OF_RANGE;
        result->errorLocation = (*pBuf) - 1;
      }
      else
      {
        result->field++;
        sdb_txt_parser_move_to_next_field(pBuf, result);
      }
    }
  }
}

/**************************************************************************************************
 *
 * @fn      sdb_txt_parser_get_quoted_string
 *
 * @brief   Get a quoted string
 *
 * @param   pBuf     - pointer to the buffer
 * @param   field    - pointer to where the string output
 * @param   size     - length of field in bytes
 * @param   result   - file offset of the current record
 *
 * @return  error and location
 *
 **************************************************************************************************/
void sdb_txt_parser_get_quoted_string(char ** pBuf, char * field, uint8 size, parsingResult_t * result)
{
  char * tmpPtr;
  size_t stringLen;
  
  if (**pBuf == '\0')
  {
    result->code = SDB_TXT_PARSER_RESULT_FIELD_MISSING;
    result->errorLocation = *pBuf;
  }
  else
  {
    while ((result->code == SDB_TXT_PARSER_RESULT_OK) && (**pBuf != '\"') && (**pBuf != '\0'))
    {
      if ((**pBuf != ' ') && (**pBuf != '\t'))
      {
        result->code = SDB_TXT_PARSER_RESULT_MISSING_STARTING_QUOTE;
        result->errorLocation = *pBuf;
      }
      else
      {
        (*pBuf)++;
      }
    }
    
    if (result->code == SDB_TXT_PARSER_RESULT_OK)
    {
      (*pBuf)++;  // skip the open quote
      tmpPtr = *pBuf;
      while ((**pBuf != '\"') && (**pBuf != '\0'))
      {
        (*pBuf)++;
      }
      
      if (**pBuf != '\"')
      {
        result->code = SDB_TXT_PARSER_RESULT_MISSING_ENDING_QUOTE;
        result->errorLocation = *pBuf;
      }
      else
      {
        stringLen = *pBuf - tmpPtr;
        if (stringLen > size)
        {
          result->code = SDB_TXT_PARSER_RESULT_STRING_TOO_LONG;
          result->errorLocation = tmpPtr + size;
        }
      }
      
      if (result->code == SDB_TXT_PARSER_RESULT_OK)
      {
        (*pBuf)++;
        if(field)
        {
          memcpy(field, tmpPtr, stringLen);
          field[stringLen] = '\0';
        }
        
        result->field++;
        sdb_txt_parser_move_to_next_field(pBuf, result);
      }
    }
  }
}
