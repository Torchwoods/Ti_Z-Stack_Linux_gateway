/*******************************************************************************
 Filename:       OtaServer.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    OTA Server.


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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h> 
#include "utils.h"
#include "OtaServer.h"
#include "otasrvr.pb-c.h"
#include "api_client.h"
#include "gatewayp2p.h"
#include "OtaServer_db.h"
#include "trace.h"

#include "AF.h"

/******************************************************************************
 * Constant
 *****************************************************************************/

#define IMAGEPTR_MANUFACTURER(imPtr) ((imPtr)->header.fileId.manufacturer)
#define IMAGEPTR_TYPE(imPtr) ((imPtr)->header.fileId.type)
#define IMAGEPTR_VERSION(imPtr) ((imPtr)->header.fileId.version)

/* Construct a hash out of the image header fields to sort the linked list */
#define IMAGEPTR_HASHFIELD(imPtr) (((uint64_t)((imPtr)->header.fileId.manufacturer) << 48) |  ((uint64_t)((imPtr)->header.fileId.type) << 32) | ((uint64_t)((imPtr)->header.fileId.version)))

#define EXACTMATCHPTR(imgPtr1, imgPtr2) ((IMAGEPTR_HASHFIELD(imgPtr1) == IMAGEPTR_HASHFIELD(imgPtr2)) ? TRUE : FALSE) 

/*******************************************************************************
 * Globals
 ******************************************************************************/
extern uint8 OTASERVER_QUERY_JITTER;
extern uint8 zclOTA_SeqNo;

/*******************************************************************************
 * Locals
 ******************************************************************************/
/* Pointer to the root of the Image List */
static OTA_UpgradeInstance_t * pUpgradeInstanceRoot = NULL; 

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/* Find next image that matches the fileId starting from the "start" point */
static OTA_UpgradeInstance_t* FindNextImage(zclOTA_FileID_t * pUpdateFileId, 
    OTA_UpgradeInstance_t * start, afAddrType_t * addr); 

/* Returns root of the image list */
static inline OTA_UpgradeInstance_t * GetRoot();

/* Insert image into sorted image linked list */
static OTA_UpgradeInstance_t * InsertImage(OTA_ImageData_t * pImage);

/* Insert corresponding attributes into sorted linked list */
static inline void InsertImageAttributes(OTA_UpgradeInstance_t *pItem, 
    OtaExecuteType executionType, uint32 executionDelay, 
    uint32 executionTime, int connection, uint8 updateDeviceList, 
    int numDevices, uint64_t * deviceList);

static inline OTA_UpgradeInstance_t* GetSibling(OTA_UpgradeInstance_t *);

/* Delete image from sorted linked list */
static void DeleteImage(OTA_UpgradeInstance_t *pItem);

static void addImageToFile(char * fileName, OTA_UpgradeInstance_t *pItem);

static int updateImageInFile( char * fileName, OTA_UpgradeInstance_t *pItem);

static int deleteImageFromFile(char * fileName);

static inline uint8 NetworkManager_getEndPointAddr(uint64_t ieee_addr)
{
  return 0xFF;
}

/*******************************************************************************
 *
 * @fn         OtaServer_GetImage 
 *
 * @brief      Returns an image from the database, corresponding to the fileId 
 * 	       and hwVer specified along with its size. 
 *
 * @param	pFileId - image info to match
 * @param	imageSize - return parameter corresponding to size of image
 * @param	hwVer - hwVer that should fall between min/max of image in db
 * @param	ieee - ieee addr of the sending device 
 * @param       options - attributes to be matched 
 *
 * @return      Status of image search. 
 *
 ******************************************************************************/
uint8 OtaServer_GetImage(zclOTA_FileID_t *pFileId, uint32 * imageSize, 
    uint16 hwVer, uint8 *ieee, uint8 options, afAddrType_t * addr)
{	//Equivalent of MT_OtaGetImage -> OTA_ProcessOta_GetImgReq
    // -> OTA_GetNextImage -> CFileList::getNextImage
    //CFileList::getNextImage(uint16 addr, zclOTA_FileID_t *pFileId,
        //         zclOTA_FileID_t *pUpdateFileId, uint8 options, 
    //	   uint16 hwVer, uint8* ieee)

  zclOTA_FileID_t pUpdateFileId;	
  OTA_UpgradeInstance_t *pImage = NULL;
  OTA_ImageData_t *pImageData = NULL;

  pUpdateFileId.manufacturer = pFileId->manufacturer;
  pUpdateFileId.type = pFileId->type;
  pUpdateFileId.version = 0xFFFFFFFF; //Get the latest version

  //Find an image based on manufacturer, type and version
  pImage = FindNextImage(&pUpdateFileId, NULL, addr); //Start from root

  while (pImage) {

    pImageData = pImage->imageData;
 
    if (pImageData) {
 
      // If a Hardware Version was specified, make sure it matches
      if (options & MT_OTA_HW_VER_PRESENT_OPTION) {
        if ((pImageData->header.fieldControl & OTA_FC_HWV_PRESENT) == 0 ||
            pImageData->header.minHwVer > hwVer ||
            pImageData->header.maxHwVer < hwVer)
        {
          //Move to the next item
          pImage = FindNextImage(&pUpdateFileId,pImage, addr);
          continue;
        }
      }

      // If an IEEE address was specified, make sure it matches
      if (options & MT_OTA_QUERY_SPECIFIC_OPTION) {
        if ((pImageData->header.fieldControl & OTA_FC_DSF_PRESENT) == 0 ||
            memcmp(pImageData->header.destIEEE, ieee, Z_EXTADDR_LEN))
        {
          //Move to the next item
          pImage = FindNextImage(&pUpdateFileId,pImage, addr);
          continue;
        }
      }
    }

    //If you found an image with correct version/destIEEE, exit loop
    break;
  }

#if 0 
  while(pImage) {

    if (pImage->imageData->header.fileId.version > pFileId->version) {
      if (pImage->imageData->header.fileId.version > pFileId->version)
        pBestImage = pImage;
    }
        
    pImage = FindNextImage(&pUpdateFileId, pImage);
  }
#endif

  if ((pImage != NULL) && (pImage->imageData != NULL)) {
    pFileId->version = pImage->imageData->header.fileId.version;
    *imageSize = pImage->imageData->length;
    return OTASERVER_SUCCESS;
  }
  else 
    return OTASERVER_FILENOTFOUND;
}

/*******************************************************************************
 *
 * @fn          OtaServer_GetImageBlock
 *
 * @brief       Returns a block of the image from the fileId and offset 
 * 		specified 
 *
 * @param	pFileId - fileId identifying the image
 * @param	offset  - offset from which to return the image
 * @param	pBuf	- Buffer to populate the image with
 * @param	len     - length of block requested 
 *
 * @return     	length of image returned (< or = len) 
 *
 ******************************************************************************/
uint32 OtaServer_GetImageBlock(zclOTA_FileID_t * pFileId, uint32 offset, 
    uint8 * pBuf, uint8 len)
{
  OTA_UpgradeInstance_t * pItem;

  pItem = FindNextImage(pFileId, NULL, NULL);

  if (pBuf && pItem) {

    OTA_ImageData_t *pImageData = pItem->imageData; 

    if (pImageData) {

      if (len > (pImageData->length - offset) ) {

        len = (uint8) (pImageData->length - offset);	

      }

      memcpy(pBuf, pImageData->pData + offset, len);

      return len;
    }
  }

  return 0;
}

/*******************************************************************************
 *
 * @fn  	OtaServer_GetImageSize        
 *
 * @brief      	Returns size of image
 *
 * @param	pFileId - identifier of image
 *
 * @return      size of image
 *
 ******************************************************************************/
uint32 OtaServer_GetImageSize(zclOTA_FileID_t * pFileId)
{
  OTA_UpgradeInstance_t * pItem;

  pItem = FindNextImage(pFileId, NULL, NULL);

  if (pItem) {

    OTA_ImageData_t *pData = pItem->imageData; 

    if (pData) return pData->length;
  }

  return 0;
}
 
/*******************************************************************************
 *
 * @fn          OtaServer_GetInstance
 *
 * @brief      	Returns the list "node" corresponding to the fileId 
 *
 * @param	pFileId - Identifier of the image
 *
 * @return      Pointer to the list "node".
 *
 ******************************************************************************/

OTA_UpgradeInstance_t * OtaServer_GetInstance(zclOTA_FileID_t *pFileId)
{

  return FindNextImage(pFileId, NULL, NULL); //Start from root
}

/*******************************************************************************
 *
 * @fn          OtaServer_AddImage
 *
 * @brief       Add image and attributes to the image database. If entry already
 *		exists, update the attributes of the image
 *
 * @param	fileName	- String name of the file
 * @param	executionType   - Information on when to update to image
 * @param	executionDelay	- Time Delay at which to update to image
 * @param	executionTime 	- Time at which to update to image
 * @param	deviceNumber	- Number of devices that should update
 * @param	deviceList[]	- List of devices (of size deviceNumber)
 * @param	notification	- mode of notification (broadcast, unicast, none
 * @param	connection	- connection id that made request.
 * 				  -1 implies, reading from database file
 *
 * @return     	status of image registration 
 *
 ******************************************************************************/
uint8 OtaServer_AddImage(char * fileName, OtaExecuteType executionType, 
    uint32 executionDelay, uint32 executionTime, uint8 updateDeviceList,
    uint8 deviceNumber, uint64_t deviceList[], OtaNotificationType notification,
    int connection)
{
  OTA_ImageData_t * pImageData = NULL;
  FILE * pfile = NULL;
  zclOTA_ImageNotifyParams_t notifyParams; //Alloc memory for Params
  uint8 status = OTASERVER_SUCCESS;
  OTA_UpgradeInstance_t * pItem;
  afAddrType_t dstAddr;
  zclOTA_FileID_t notifyFileId;
  pfile = fopen(fileName, "rb");

  pImageData = malloc(sizeof(OTA_ImageData_t));

  if (NULL == pImageData) return (OTASERVER_OUTOFRESOURCES);

  if (pfile) {

    int readLen;

    //Determine size of file
    fseek(pfile, 0L, SEEK_END);
    pImageData->length = ftell(pfile);
    fseek(pfile, 0L, SEEK_SET);

    //Also allocate space for pImage->pData
    pImageData->pData = malloc(pImageData->length);

    if (pImageData->pData == NULL) {
      free(pImageData);
      fclose(pfile);
      return OTASERVER_OUTOFRESOURCES;
    }

    // Read the image data
    readLen = (int) fread(pImageData->pData, 1, pImageData->length, pfile);

    if (readLen == pImageData->length) {
      // Parse the header
      OTA_ParseHeader(&(pImageData->header), pImageData->pData);

      if (pImageData->header.magicNumber == OTA_HDR_MAGIC_NUMBER)
      {

	//Save the fileID first
        notifyFileId = pImageData->header.fileId; 

        // See if the item is a duplicate
        pItem = FindNextImage(&(pImageData->header).fileId, NULL, NULL);

        if (pItem)
        {
          /* Found an existing file */

          free(pImageData->pData);
          free(pImageData);

          //If you find a duplicate, check and update the ExecuteTiming info
	  uiPrintfEx(trDEBUG,"OtaServer_AddImage: executionType passed %d\n",
		executionType);

          if (executionType == OTA_EXECUTE_TYPE__NO_CHANGE)
	  {
            executionType = pItem->executionType;
	    executionDelay = pItem->executionDelay;
	    executionTime = pItem->executionTime;
	  }

          //status = OTASERVER_IMG_DUPLICATE_ID;
          //Free malloc-ed stuff
          InsertImageAttributes(pItem, executionType, executionDelay, 
                executionTime, connection, updateDeviceList, deviceNumber, 
                deviceList);

          //Update the database
          if (0 != updateImageInFile(fileName, pItem)) 
          {
            uiPrintfEx(trDEBUG,"OtaServer_AddImage: Error adding %s to database "
                "file\n", fileName);
          }

        }
        else
        {

          /* No matching file found, create a new entry */
          if (executionType == OTA_EXECUTE_TYPE__NO_CHANGE)
          {
            uiPrintfEx(trDEBUG,"OtaServer_AddImage: No pre-registered image %s."
                "EXECUTE TYPE - NO CHANGE, invalid attribute\n", fileName);

            free(pImageData->pData);
            free(pImageData);
            fclose(pfile);
            return (OTASERVER_FAILURE);
          }

          pItem = InsertImage(pImageData);

          InsertImageAttributes(pItem, executionType, executionDelay,
                executionTime, connection, updateDeviceList, deviceNumber, 
                deviceList);

          //Add to the database
          if (connection != -1) addImageToFile(fileName, pItem);
        }

        //Don't notify if in recovery mode
        if (connection != -1) {

          notifyParams.fileId = notifyFileId; 
  
          notifyParams.payloadType = NOTIFY_PAYLOAD_JITTER_MFG_TYPE_VERS;
  
          notifyParams.queryJitter = OTASERVER_QUERY_JITTER;
  
          switch (notification) {
  
              case OTA_NOTIFICATION_TYPE__DO_NOT_SEND:
                  //Don't send notification
                  break;

              case OTA_NOTIFICATION_TYPE__BROADCAST_NOT:
                  //Broadcast notification
                  dstAddr.addrMode = afAddr16Bit;
                  dstAddr.addr.shortAddr = 0xFFFF;
                  dstAddr.endPoint = 0xFF;
                  zclOTA_SendImageNotify(&dstAddr, &notifyParams,
                    ++zclOTA_SeqNo);
                  break;

              case OTA_NOTIFICATION_TYPE__UNICAST_NOT:
                  {
                  //Unicast according to supported deviceList
                    int i;
                    //Not sure this is required. 
                    //if (pItem->deviceNumber != 0xFF)
                      for (i = 0; i < pItem->deviceNumber; i++) {

                        dstAddr.endPoint = NetworkManager_getEndPointAddr(
                            pItem->deviceList[i]);

                        dstAddr.addrMode = afAddr16Bit;

                        if (!(gwPb_SrvrGetShortAddress(pItem->deviceList[i],
                              &(dstAddr.addr.shortAddr)))) {

                          uiPrintfEx(trDEBUG,"OtaServer_AddImage: Error obtaining "
                              "short address from ieee address 0x%Lx\n",
                              pItem->deviceList[i]);

                          //Don't send ImageNotify
                          dstAddr.addr.shortAddr = 0xFFFF;
                          status = OTASERVER_FAILURE;
                        }
                        else 
                          zclOTA_SendImageNotify(&dstAddr, &notifyParams,
                                ++zclOTA_SeqNo);
                      }
                  }
                  break;

          default:
              status = OTASERVER_BADCOMMAND;
          }

        }
        //status = OTASERVER_SUCCESS;
      }
      else {
        //Send out some "bad header" error
        status = OTASERVER_BADFILEFORMAT;
      }	
    }
    else {
      //Send out some "could not read file" error
      status = OTASERVER_BADFILEFORMAT;
    }

    fclose(pfile);
  }
  else {
    //fopen failed 
    status = OTASERVER_OUTOFRESOURCES;
  }

  return (status);
}

/*******************************************************************************
 *
 * @fn		OtaServer_DeleteImage          
 *
 * @brief      	Deletes image from image database 
 *
 * @param	fileName - Unique name of image to be deleted.
 *
 * @return     	Status of image deletion 
 *
 ******************************************************************************/
uint8 OtaServer_DeleteImage(char * fileName) 
{
  OTA_ImageData_t * pImageData;
  FILE * pfile;
  uint8 status = OTASERVER_SUCCESS;
  OTA_UpgradeInstance_t * pItem;

  pfile = fopen(fileName, "rb");

  pImageData = malloc(sizeof(OTA_ImageData_t));

  if (NULL == pImageData) return (OTASERVER_OUTOFRESOURCES);

  if (pfile) {
    int readLen;

    //Determine size of file
    fseek(pfile, 0L, SEEK_END);
    pImageData->length = ftell(pfile);
    fseek(pfile, 0L, SEEK_SET);

    pImageData->pData = malloc(pImageData->length);

    if (pImageData->pData == NULL) {
      free(pImageData);
      fclose(pfile);
      return OTASERVER_OUTOFRESOURCES;
    }

    // Read the image data
    readLen = (int) fread(pImageData->pData, 1, pImageData->length, pfile);

    if (readLen == pImageData->length) {

      // Parse the header
      OTA_ParseHeader(&(pImageData->header), pImageData->pData);

      if (pImageData->header.magicNumber == OTA_HDR_MAGIC_NUMBER) {

        pItem = FindNextImage(&(pImageData->header).fileId, NULL, NULL);
        if (pItem) 
        {
          DeleteImage(pItem);

          //Update the database
          if (0 != deleteImageFromFile(fileName))
            uiPrintfEx(trDEBUG,"Error removing %s from database\n", fileName);

          status = OTASERVER_SUCCESS;
        }
        else {
          //File Not found !!
          status = OTASERVER_UNREGISTERFAILED;
        }
      }
      else {
        //Send out some "bad header" error
        status = OTASERVER_BADFILEFORMAT;
      }	
    }
    else {
      //Send out some "could not read file" error
      status = OTASERVER_BADFILEFORMAT;
    }

    free(pImageData->pData);
    free(pImageData);
    fclose(pfile);
  }
  else {
    //fopen failed 
    status = OTASERVER_OUTOFRESOURCES;
  }
        
  return (status);
}

/*******************************************************************************
 *
 * @fn       DeleteImage   
 *
 * @brief    Delete image node 
 *
 * @param	pItem - Node to be deleted
 *
 * @return      none
 *
 ******************************************************************************/
static void DeleteImage(OTA_UpgradeInstance_t *pItem)
{

  OTA_UpgradeInstance_t * pImage = NULL;
  OTA_ImageData_t *pImageData = NULL;
  OTA_UpgradeInstance_t * prevPtr;

  pImage = GetRoot();
  pImageData = pImage->imageData; 

  //Check for a match with root

  if (EXACTMATCHPTR(pImageData, pItem->imageData)) {
    //Update root
    pUpgradeInstanceRoot = pItem->next;
    //Free memory
    if ((pItem->deviceNumber > 0) && (pItem->deviceList != NULL))
    {
      free(pItem->deviceList);
    }
    free(pItem->imageData->pData);
    free(pItem->imageData);
    free(pItem);
    return;
  }

  /* Loop through tree to find a match for pImageData */
  do {
    prevPtr = pImage;
    pImage = pImage->next;

    //Extract imageData
    pImageData = pImage->imageData; 

    if (EXACTMATCHPTR(pImageData, pItem->imageData)) {
      prevPtr->next = pItem->next;
      if ((pItem->deviceNumber > 0) && (pItem->deviceList != NULL))
      {
        free(pItem->deviceList);
      }
      free(pItem->imageData->pData);
      free(pItem->imageData);
      free(pItem);
      return;
    }

  } while (pImage->next != NULL); 

  //assert!!      
}
/*******************************************************************************
 *
 * @fn 		FindNextImage
 *
 * @brief       Finds the next item that matches the given file ID 
 *
 * @param	pFileId - match characteristics of the file
 * @param	start - starting node from which to search.
 * @param	numDevices - Number of devices in filter list
 * @param	deviceList - List of devices to filter image against
 * @return     	pointer to the image node found 
 *
 ******************************************************************************/
static OTA_UpgradeInstance_t * FindNextImage(zclOTA_FileID_t * pFileId, 
    OTA_UpgradeInstance_t * start, afAddrType_t * address)
{

  OTA_UpgradeInstance_t * pImage = NULL;
  OTA_ImageData_t *pData = NULL;
  uint64_t client_addr =  0x0;

  if (NULL != address) {
    if (address->addrMode == afAddr64Bit ) {
      memcpy(&client_addr, &address->addr.extAddr, 8);
    }
    else if (address->addrMode == afAddr16Bit) {
      if (!(gwPb_SrvrGetIeeeAddress(address->addr.shortAddr, &client_addr))) {
        //Error in conversion
        client_addr = 0x0;
      }
    }
    else client_addr = 0x0;
  }

  if (start == NULL) {
    pImage = GetRoot();
  }
  else 
    pImage = GetSibling(start);

  /* Loop through tree to find a match for pData */
  while (pImage) {

    //Extract imageData
    pData = pImage->imageData; 
  
    //Check for a match against requested fields
    if (pData)

      if ((pFileId->type == 0xFFFF) || 
        (pFileId->type == pData->header.fileId.type))

        if ((pFileId->manufacturer == 0xFFFF) || 
        (pFileId->manufacturer == pData->header.fileId.manufacturer))

          if ( (pFileId->version == 0xFFFFFFFF) || 
            (pFileId->version == pData->header.fileId.version)) {

        int i;

        //No image list available, return this image
        if ((address == NULL) || (pImage->deviceNumber == 0)) 
        return (pImage);

        //Could not get a 64-bit address, this is an error situation
        if (client_addr == 0x0) return (NULL);

        for (i = 0; i < pImage->deviceNumber; i++) {
          if (pImage->deviceList[i] == client_addr) return (pImage); 
        }

        //If we return here, we disallow return of a lower version image
        //if a higher is present (same manu/type).Commenting this out
        //allows it.
            //return (NULL);
      }

    pImage = pImage->next;

  }

  return (NULL);
}

/* Returns the root (parent) of this tree */
static OTA_UpgradeInstance_t * GetRoot()
{
  return (pUpgradeInstanceRoot);
}


static OTA_UpgradeInstance_t * GetSibling(OTA_UpgradeInstance_t * start)
{
  return (start);
}

/*******************************************************************************
 *
 * @fn          InsertImage
 *
 * @brief      	Insert image into image linked list 
 *
 * @param	pImage - ImageData to be inserted
 *
 * @return      Pointer to new mode that was created.
 *
 ******************************************************************************/
static OTA_UpgradeInstance_t * InsertImage(OTA_ImageData_t * pImage)
{
  OTA_UpgradeInstance_t * newNode;
  OTA_UpgradeInstance_t * ptr;
  OTA_UpgradeInstance_t * prevPtr;
  bool found = FALSE;

  //Allocate a new node
  newNode = malloc(sizeof(OTA_UpgradeInstance_t));
  OTA_UPGRADE_INSTANCE_INIT(newNode);
 
  if (NULL == newNode) return NULL;

  newNode->imageData = pImage;
    
  //Insert new image, sorted on basis of manufacturer, image, version 
  ptr = GetRoot();

  if (NULL == ptr) {
    pUpgradeInstanceRoot  = newNode; 
    newNode->next = NULL;
    return newNode;
  }
  
  prevPtr = ptr;
  while ((ptr != NULL) && (found == 0)) {

    uiPrintfEx(trDEBUG,"Comparing new node 0x%Lx and existing node 0x%Lx\n",
    IMAGEPTR_HASHFIELD(newNode->imageData), 
    IMAGEPTR_HASHFIELD(ptr->imageData)); 

    if (IMAGEPTR_HASHFIELD(newNode->imageData) > 
        IMAGEPTR_HASHFIELD(ptr->imageData)) {
      //Newest version, insert here 
      found = 1;
      break;
    }
    prevPtr = ptr;
    ptr = ptr->next;	

  }

  if (found == 1) {
    //Found a position, insert at position of "ptr"	
    if (prevPtr == ptr) {
      //New head of list
      newNode->next = prevPtr;
      pUpgradeInstanceRoot = newNode;
    }
    else {
      //(prev > newNode) newNode is > ptr
      prevPtr->next = newNode;
      newNode->next = ptr; 
    }	
  }
  else {
    //Not found, so insert at end of list
    prevPtr->next = newNode;
    newNode->next = NULL;	
  }

  return newNode;
}

/*******************************************************************************
 *
 * @fn          InsertImageAttributes
 *
 * @brief      	Inserts other attributes of the image at node location 
 *
 * @param	pItem - Node at which to insert information
 * @param	executionType 
 * @param	executionDelay
 * @param	executionTime
 * @param	connection
 *
 * @return      
 *
 ******************************************************************************/
static inline void InsertImageAttributes(OTA_UpgradeInstance_t *pItem, 
    OtaExecuteType executionType, uint32 executionDelay, uint32 executionTime, 
    int connection, uint8 updateDeviceList, int deviceNumber,
    uint64_t *deviceList)
{
  if (pItem != NULL) {
  
    uiPrintfEx(trDEBUG,"InsertImageAttributes: Updating attributes type %d delay "
	"0x%x time 0x%x\n", executionType, executionDelay, executionTime);

    /* Always update the execution attributes */
    pItem->executionType = executionType;
    pItem->executionDelay = executionDelay;
    pItem->executionTime = executionTime;
    pItem->connection = connection;

    /* Check if deviceList needs to be updated */
    if (updateDeviceList) {

      if ((NULL == pItem->deviceList) && (0 == pItem->deviceNumber)) {

        /* Existing list is empty */
        int i;
        uiPrintfEx(trDEBUG,"InsertImageAttributes: Adding %d devices\n", 
        deviceNumber);

        /* Add new list here */
        pItem->deviceNumber = deviceNumber;
        if (0 != deviceNumber) {
          pItem->deviceList = malloc(sizeof(uint64_t) * deviceNumber);
        }
    
        for (i = 0; i < deviceNumber; i++) {
          pItem->deviceList[i] = deviceList[i];
        }
      }
      else {
  
        /* Override existing device list */
        uint64_t * newList = NULL;
        uiPrintfEx(trDEBUG,"InsertImageAttributes: Updating device list.\n");
  
        if (deviceNumber == 0) {
  
          /* Image now applies to all devices */
          pItem->deviceNumber = 0x0;
          free(pItem->deviceList);
          pItem->deviceList = NULL;
        }
        else {
  
          /* Image applies to new set of devices */
          int i;
          newList = malloc(sizeof(uint64_t) * deviceNumber);
  
          if (newList != 0x0) {
  
            for (i = 0; i < deviceNumber; i++) {
              newList[i] = deviceList[i];
            }
  
            free(pItem->deviceList);
            pItem->deviceNumber = deviceNumber;
            pItem->deviceList = newList;
          }
          else {
            uiPrintfEx(trDEBUG,"InsertImageAttributes: Allocation error, could not"
                " allocate memory for new list\n");
          }
        }
      }
    }
  }
}


static void addImageToFile(char * fileName, OTA_UpgradeInstance_t *pItem)
{
  fileDbInfo_t entry;

  entry.fileName = fileName;
  entry.executionType = (uint8)(pItem->executionType);
  entry.executionDelay = pItem->executionDelay;
  entry.executionTime = pItem->executionTime;
  entry.deviceNumber = pItem->deviceNumber;
  entry.deviceList = pItem->deviceList;
  
  otaListAddEntry(&entry);
}

static int updateImageInFile( char * fileName, OTA_UpgradeInstance_t *pItem)
{
  fileDbInfo_t * entry_ptr;
  fileDbInfo_t new_entry;
  //entry_ptr = otaListGetDeviceByFilename(fileName);
  entry_ptr = otaListRemoveEntryByFilename(fileName);
  if (entry_ptr != NULL)
  {
    new_entry.executionType = (uint8)(pItem->executionType);
    new_entry.executionDelay = pItem->executionDelay;
    new_entry.executionTime = pItem->executionTime;
    new_entry.deviceNumber = pItem->deviceNumber;
    new_entry.fileName = fileName;
    if (0 != new_entry.deviceNumber) {
      new_entry.deviceList = pItem->deviceList;
    }
    else new_entry.deviceList = NULL; 

    otaListReleaseRecord(entry_ptr);

    otaListAddEntry(&new_entry);
    
  }
  else
  {
    return 1;
  }
  return 0;
}

static int deleteImageFromFile(char * fileName)
{
  fileDbInfo_t * entry_ptr;
  entry_ptr = otaListRemoveEntryByFilename(fileName);

  if (entry_ptr != NULL) 
  {
    otaListReleaseRecord(entry_ptr);
    return 0;
  }
  return 1;
}
