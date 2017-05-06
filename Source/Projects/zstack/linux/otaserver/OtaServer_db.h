/**************************************************************************************************
 * Filename:       OtaServer_db.h
 * Description:    Interface for saving OtaServer RAM contents to database file 
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

#ifndef OTASERVER_DB_H
#define OTASERVER_DB_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stdbool.h>

#include "OtaServer.h"

//states of db entries
#define FILEENTRY_STATE_NOT_ACTIVE    0
#define FILEENTRY_STATE_ACTIVE        1
#define MAX_SUPPORTED_FILE_NAME_LENGTH 128 

typedef struct {
  char * fileName;
  uint8 executionType;
  uint32 executionDelay;
  uint32 executionTime; 
  uint8 deviceNumber;
  uint64_t * deviceList;
} fileDbInfo_t;


/*
 * otaListAddEntry - create a device and add a rec to the list.
 */
void otaListAddEntry( fileDbInfo_t *epInfo);

/*
 * otaListRemoveEntryByNaEp - remove a device rec from the list.
fileDbInfo_t * otaListRemoveEntryByNaEp( uint16_t nwkAddr, uint8_t endpoint );
 */

/*
 * otaListNumEntries - get the number of devices in the list.
 */
uint32_t otaListNumEntries( void );

/*
 * otaListInitDatabase - restore device list from file.
 */
bool otaListInitDatabase( char * dbFilename );

fileDbInfo_t * otaListGetNextEntry(int *context);

fileDbInfo_t * otaListGetDeviceByFilename(char * fileName);

void otaListReleaseRecord (fileDbInfo_t * entry);

/*
fileDbInfo_t * otaListGetDeviceByNaEp( uint16_t nwkAddr, uint8_t endpoint );
*/

fileDbInfo_t * otaListRemoveEntryByFilename( char * fileName);

bool otaListCloseDatabase(void);

#ifdef __cplusplus
}
#endif

#endif /* OTASERVER_DB_H */
