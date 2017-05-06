/******************************************************************************
 Filename:       zcl_otaserver_lnx.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    This file contains the linux OTA Server application.


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
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>

#include "zstack.pb-c.h"
#include "hal_rpc.h"
#include "api_client.h"
#include "api_server.h"
#include "utils.h"
#include "otasrvr.h"
#include "OtaServer.h"
#include "otasrvr.pb-c.h"
#include "gatewayp2p.h"
#include "OtaServer_db.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ota.h"
#include "trace.h"

/******************************************************************************
 * Constant
 *****************************************************************************/

#define OTA_SERVER_DEFAULT_PORT 2525 

#define ZCL_OTA_MAX_INCLUSTERS       1
#define ZCL_OTA_MAX_OUTCLUSTERS      0
//#define zclOTA_OutClusterList         NULL

const cId_t zotaInClusterList[ZCL_OTA_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_OTA
};

/******************************************************************************
 * Typedefs
 ******************************************************************************/

/*****************************************************************************
 * Globals
 ******************************************************************************/
uint8 zclOTA_SeqNo = 0;

extern int appConnectionHandle;

//Configuration Parameters 
uint32 OTASERVER_CONFIG_PORT = OTA_SERVER_DEFAULT_PORT; 
uint8 OTASERVER_ENDPOINT = ZCL_OTA_ENDPOINT;
uint16 OTASERVER_PROFILEID = ZCL_HA_PROFILE_ID;
uint16 OTASERVER_DEVICEID = ZCL_OTA_DEVICEID;
uint8 OTASERVER_QUERY_JITTER = 100;
uint16 OTASERVER_MINBLOCKREQUEST_DELAY = 0;
char OTASERVER_DB[MAX_SUPPORTED_FILE_NAME_LENGTH] = {"./DbUpgradeList.csv"};
char OTASERVER_CTXT_INFO[MAX_SUPPORTED_FILE_NAME_LENGTH] = {"./DbUpgradeCtxt.csv"};
 

/* Call back function handles Linux Gateway messages from Application */
extern void otasrvrHandlePbCb (int connection, uint8 subSys, uint8 cmdid,
        uint16 len, uint8 *pData, uint8 type);

extern void processApplyImageCnf(int connection, bool status);


uint8 processIncomingZCLCommands( zclIncoming_t *pInMsg );

/*******************************************************************************
 * Locals
 ******************************************************************************/


/* Input Cluster list for OTA Server */
const cId_t zclOTA_InClusterList[ZCL_OTA_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_OTA
};

/* Download permit status for OTA Server */
//static OtaEnableModes zclOTA_Permit = OTA_ENABLE_MODES__DOWNLOAD_ENABLE;
static OtaEnableModes zclOTA_Permit = OTA_ENABLE_MODES__DOWNLOAD_DISABLE;

/* Network address information */
static SysNwkInfoReadRsp nwkInfoRsp = SYS_NWK_INFO_READ_RSP__INIT;

/* OTA Endpoint descriptor */
static endPointDesc_t zotaEpDesc;

/* Local copy of client handle to send requests */
apicHandle_t apiClientHandle;
apicHandle_t giNwkMgrHandle;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/* Get time in ms */
static uint32 gettimeinms();

/* Function to handle messages of interest from ZStackServer */ 
static void handleZstackMsgs( apicHandle_t handle, uint8 subSys, uint8 cmdID, 
    uint16 len, uint8 *pMsg );

/* Registers Endpoint Simple descriptor */
static bool zotaRegEndpoint(SimpleDescriptionFormat_t * simpleDescFmt);

/* Function to read network information */
static void sendNwkInfoReadReq( void );

/* Process Device Change Indication */
static void processDevStateChangeInd( DevStateChangeInd *pStateChng );

/* Process Incoming AF Message */
static void processAfIncomingMsgInd( AfIncomingMsgInd *pInMsg );

/* Process Data Confirm Message */
static void processAfDataConfirmInd( AfDataConfirmInd *pInMsg );

/* Send OTA Image Download Status to Gateway Application */
static void sendDownloadStatus(int connection, afAddrType_t * pSrcAddr,
    OtaStatus status);

/* Respond to user input */
static void getUserInput(void);

/* Build upgrade list info from database file */
static void buildUpgradeList(void);

/* Restore permit status info from database file */
static void restorePermitStatus(void);

/* Functions for ZCL Callbacks  */
static void basicResetCB( void );
static void identifyCB( zclIdentify_t *pCmd );
static void identifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );

/* Callback function to handle all incoming ZCL OTA commands */
static ZStatus_t zclOTA_ServerHdlIncoming( zclIncoming_t *pInMsg );

/* Functions to process ZCL OTA queries */ 
static ZStatus_t zclOTA_ProcessQueryNextImageReq( zclIncoming_t *pInMsg );
static ZStatus_t zclOTA_ProcessImageBlockReq( zclIncoming_t *pInMsg );
static ZStatus_t zclOTA_ProcessImagePageReq( zclIncoming_t *pInMsg );
static ZStatus_t zclOTA_ProcessUpgradeEndReq( zclIncoming_t *pInMsg );
static ZStatus_t zclOTA_ProcessQuerySpecificFileReq( zclIncoming_t *pInMsg );

/* Functions return appropriate ZCL OTA responses */
static ZStatus_t zclOTA_Srv_QueryNextImageReq(afAddrType_t *pSrcAddr, 
    zclOTA_QueryNextImageReqParams_t *pParam);
static ZStatus_t zclOTA_Srv_ImageBlockReq(afAddrType_t *pSrcAddr, 
    zclOTA_ImageBlockReqParams_t *pParam);
static ZStatus_t zclOTA_Srv_ImagePageReq(afAddrType_t *pSrcAddr, 
    zclOTA_ImagePageReqParams_t *pParam);
static ZStatus_t zclOTA_Srv_UpgradeEndReq(afAddrType_t *pSrcAddr, 
    zclOTA_UpgradeEndReqParams_t *pParam);
static ZStatus_t zclOTA_Srv_QuerySpecificFileReq(afAddrType_t *pSrcAddr, 
    zclOTA_QuerySpecificFileReqParams_t *pParam);

/* Function to send messages to API Client */
ZStatusValues sendAPICExpectDefaultStatus( apicHandle_t handle, int cmdID, int len, uint8 *pData );

extern apisSysParams_t sysParams;
/* Configuration parameters for this application */
configTableItem_t configItems[] =
{
  { &(sysParams.port), "OTA_SERVER_PORT", TYPE_UINT32, 1 },
  { &OTASERVER_ENDPOINT, "OTA_SERVER_ENDPOINT", TYPE_UINT8, 1 },
  { &OTASERVER_PROFILEID, "OTA_SERVER_PROFILEID", TYPE_UINT16, 1 },
  { &OTASERVER_DEVICEID, "OTA_SERVER_DEVICEID", TYPE_UINT16, 1 },
  { &OTASERVER_QUERY_JITTER, "OTA_SERVER_QUERY_JITTER", TYPE_UINT8, 1 },
  { &OTASERVER_MINBLOCKREQUEST_DELAY, "OTA_SERVER_MINBLOCKREQUEST_DELAY", 
    TYPE_UINT32, 1 },
  { OTASERVER_DB, "OTA_SERVER_DB", TYPE_STRING, MAX_SUPPORTED_FILE_NAME_LENGTH},
  { OTASERVER_CTXT_INFO, "OTA_CONTEXT_INFO", TYPE_STRING, MAX_SUPPORTED_FILE_NAME_LENGTH}
 };


/* System Parameters returned by appInit to API Server's main */
apisSysParams_t sysParams =
{
  OTA_SERVER_DEFAULT_PORT,
  TRUE,
  (configTableItem_t *)configItems,
  (sizeof ( configItems ) / sizeof (configTableItem_t)),
  2, //Number of API clients
  handleZstackMsgs,
  otasrvrHandlePbCb
};

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
SimpleDescriptionFormat_t zotaDongleSimpleDesc = 
{
  ZCL_OTA_ENDPOINT,
  ZCL_HA_PROFILE_ID,
  ZCL_OTA_DEVICEID,
  ZCL_OTA_DEVICE_VERSION,
  ZCL_OTA_FLAGS,
  ZCL_OTA_MAX_INCLUSTERS,
  (cId_t *) &zclOTA_InClusterList,
  ZCL_OTA_MAX_OUTCLUSTERS,
  NULL, 		//(cId_t *) &zclOTA_OutClusterList 
};

/******************************************************************************
 * ZCL Callback tables
 ******************************************************************************/
static zclGeneral_AppCallbacks_t cmdCallbacks =
{
  basicResetCB, 	// Basic Cluster Reset command
  identifyCB,           // Identify command
#ifdef ZCL_EZMODE
  NULL,                 // Identify EZ-Mode Invoke command
  NULL,                 // Identify Update Commission State command
#endif
  NULL,                 // Identify Trigger Effect command
  identifyQueryRspCB,   // Identify Query Response command
  NULL,                 // On/Off cluster commands
  NULL,                 // On/Off cluster enhanced command Off with Effect
  NULL,     // On/Off cluster enhanced command On with Recall Global Scene
  NULL,               	// On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                 // Level Control Move to Level command
  NULL,                 // Level Control Move command
  NULL,                 // Level Control Step command
  NULL,                 // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                 // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                 // Scene Store Request command
  NULL,                 // Scene Recall Request command
  NULL,                 // Scene Response command
#endif
#if ZCL_ALARMS
  NULL,                 // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                 // Get Event Log command
  NULL,                 // Publish Event Log command
#endif
  NULL,                 // RSSI Location command
  NULL                  // RSSI Location Response command
};

/******************************************************************************
 *****************************************************************************/

int appArgs(int *p_argc, char ***p_argv)
{
  int argc = *p_argc;
  char **argv = *p_argv;

  if (argc < (sysParams.numClients + 1)) 
  {
    uiPrintfEx(trUNMASKABLE, "\nUsage: %s zstackserveripaddr:port nwkmgripaddr:port [otaserver.ini]\n\n", argv[0]);

    exit (3);
  }

  return 0;
}

apisSysParams_t *appInit( void )
{
  return ( &sysParams );
}

int appMain(apicHandle_t *handles)
{
  //Usage message for key commands
  uiPrintfEx(trUNMASKABLE, "\n");
  uiPrintfEx(trUNMASKABLE, " ************************************************\n");
  uiPrintfEx(trUNMASKABLE, " *                OTA Upgrade Server 1.0.1      *\n");
  uiPrintfEx(trUNMASKABLE, " * The following are the avaible key commands:  *\n");
  uiPrintfEx(trUNMASKABLE, " * Exit Program.                         -  q   *\n");
  uiPrintfEx(trUNMASKABLE, " ************************************************\n\n");


  //Assuming we have 2 client handles
  apiClientHandle = handles[0];
  giNwkMgrHandle = handles[1];

  if (NULL == apiClientHandle) {
    //Error message
    uiPrintfEx(trUNMASKABLE, "\nError - No ZStack Server Detected. Exiting...\n\n");
    exit(4);
  }

  if (NULL == giNwkMgrHandle) {
    //Error message
    uiPrintfEx(trUNMASKABLE, "\nError - No Network Manager Server Detected. Exiting...\n\n");
    exit(4);
  }


  zotaDongleSimpleDesc.EndPoint = OTASERVER_ENDPOINT;
  zotaDongleSimpleDesc.AppProfId = OTASERVER_PROFILEID;
  zotaDongleSimpleDesc.AppDeviceId = OTASERVER_DEVICEID;
  zotaEpDesc.endPoint = OTASERVER_ENDPOINT;
  zotaEpDesc.task_id = 0;
  zotaEpDesc.simpleDesc = &zotaDongleSimpleDesc;
  zotaEpDesc.latencyReq = 0;

  uiPrintfEx(trDEBUG,"Registering OTA end point 0x%x, profile id 0x%x device id "
    "0x%x\n",OTASERVER_ENDPOINT, OTASERVER_PROFILEID, 
    OTASERVER_DEVICEID );

  //Register OTA Endpoint
  if (FALSE == zotaRegEndpoint(&zotaDongleSimpleDesc))
  {
    uiPrintfEx(trUNMASKABLE, "\nError - Could not register OTA endpoint with zstackserver."
        "Exiting...\n\n");
    exit(5);
  }

  // Register as a ZCL Plugin
  zcl_registerPlugin( ZCL_CLUSTER_ID_OTA,
                      ZCL_CLUSTER_ID_OTA,
                      zclOTA_ServerHdlIncoming );

  zclGeneral_RegisterCmdCallbacks( OTASERVER_ENDPOINT, &cmdCallbacks );

#ifdef __OTASERVER_DEBUG__
  sendNwkInfoReadReq();
#endif 

  //Initialize the network manager database, to prepare for future access.
  //uiprintfEx(trDEBUG,"Accessing nwkmgr's Device Database\n");
  //nwkMgrDb_Init(FALSE);

  buildUpgradeList();
  restorePermitStatus();

  //Accept key to exit
  getUserInput();

  exit (0);
}

static void restorePermitStatus()
{
  FILE * file_ptr = NULL;
  file_ptr = fopen(OTASERVER_CTXT_INFO, "r+b");
  if ((NULL == file_ptr) && (errno == ENOENT))
  {
    //File does not exist, creating it.
    uiPrintfEx(trDEBUG,"restorePermitStatus: Creating file %s to save PERMIT\n",
	OTASERVER_CTXT_INFO);
    file_ptr = fopen(OTASERVER_CTXT_INFO, "w+b");
    if (NULL != file_ptr)
    {
      uiPrintfEx(trDEBUG,"restorePermitStatus: PERMIT value %d\n", zclOTA_Permit);
      fwrite(&zclOTA_Permit, sizeof(zclOTA_Permit), 1, file_ptr);
      fflush(file_ptr);
      fclose(file_ptr);
    } 
    else
    {
      uiPrintfEx(trDEBUG,"restorePermitStatus: Error creating file %s, "
	"cannot save/restore PERMIT status\n", OTASERVER_CTXT_INFO);
    }
  }
  else if (file_ptr == NULL) 
  {
    uiPrintfEx(trDEBUG,"restorePermitStatus: Error reading file %s, "
	"cannot save/restore PERMIT status\n", OTASERVER_CTXT_INFO);
  }
  else {
    if (fread(&zclOTA_Permit, sizeof(zclOTA_Permit), 1, file_ptr) != 1) {
      uiPrintfEx(trDEBUG,"restorePermitStatus: Read error, resetting PERMIT\n"); 
      zclOTA_Permit = OTA_ENABLE_MODES__DOWNLOAD_DISABLE;
    }
    else {
      uiPrintfEx(trDEBUG,"restorePermitStatus: PERMIT value %d\n", zclOTA_Permit);
    }
    fclose(file_ptr);
  }
}

static void buildUpgradeList()
{
  int context = 0;

  fileDbInfo_t * temp_file_ptr = NULL;

  if (!(otaListInitDatabase(OTASERVER_DB))) {
    uiPrintfEx(trDEBUG,"buildUpgradeList: Issue creating database file to save "
    "upgrade info\n");
  }
  else
  while ((temp_file_ptr = otaListGetNextEntry(&context)) != NULL)
  {
    OtaExecuteType executeType = OTA_EXECUTE_TYPE__IMMEDIATE;
    switch(temp_file_ptr->executionType) {
      case 0:
        executeType = OTA_EXECUTE_TYPE__IMMEDIATE;
        break;
      case 1:
        executeType = OTA_EXECUTE_TYPE__DELAY;
        break;
      case 2:
        executeType = OTA_EXECUTE_TYPE__TIME;
        break;
      case 3:
        executeType = OTA_EXECUTE_TYPE__HOLD;
        break;
    }
    OtaServer_AddImage(temp_file_ptr->fileName, executeType,
        temp_file_ptr->executionDelay, temp_file_ptr->executionTime, 1,
        temp_file_ptr->deviceNumber, temp_file_ptr->deviceList, 
        OTA_NOTIFICATION_TYPE__DO_NOT_SEND , -1); 
  }

}

/*******************************************************************************
 *
 * @fn          getUserInput
 *
 * @brief       Get the user's input
 *
 * @return      none, actually doesn't return
 *
 ******************************************************************************/
static void getUserInput( void )
{
  int key = 0;

  //Debug usage message for key commands
  uiPrintf(" ************************************************\n");
  uiPrintf(" * The following are the avaible key commands:  *\n");
  uiPrintf(" * Exit Program.                         -  q   *\n");
  uiPrintf(" ************************************************\n");

  for ( ;; )
  {

    // Wait for a key press
    key = getchar();

    if ( key == 'q' ) {
      // Quit
      exit( 0 );
    }
    else if (key < 0) {
    	// stdin is unavailable. assume we are in background, pause()
	// until we get a signal (such as swapped to foreground)
	pause();
    }
    else {
    	; // ignoring this character
    }
  }
}

/*******************************************************************************
 *
 * @fn          handleZstackMsgs
 *
 * @brief       Receives incoming Asynchronous messages from the ZStack Server
 *
 * @param       cmdID - incoming message command ID
 * @param       len - length of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 *
 ******************************************************************************/
static void handleZstackMsgs(apicHandle_t handle, uint8 subSys, uint8 cmdID, 
    uint16 len, uint8 *pMsg )
{

  if ((pMsg == NULL) || (len == 0xFFFF) ) {
    // Connection terminated
    uiPrintf( "\nServer connection was terminated\n" );
    return;
  }

  if ( (subSys & RPC_SUBSYSTEM_MASK) != ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF ) 
    return;

  switch ( cmdID )
  {
    case ZSTACK_CMD_IDS__DEV_STATE_CHANGE_IND:
      {
        DevStateChangeInd *pStateChg;
        pStateChg = dev_state_change_ind__unpack( NULL, len, pMsg );
        if ( pStateChg )
        {
          processDevStateChangeInd( pStateChg );
          dev_state_change_ind__free_unpacked( pStateChg, NULL );
        }
      }
      break;

    case ZSTACK_CMD_IDS__AF_INCOMING_MSG_IND:
      {
        AfIncomingMsgInd *pInMsg;
        pInMsg = af_incoming_msg_ind__unpack( NULL, len, pMsg );
        if ( pInMsg )
        {
          processAfIncomingMsgInd( pInMsg );
          af_incoming_msg_ind__free_unpacked( pInMsg, NULL );
        }
      }
      break;

    case ZSTACK_CMD_IDS__AF_DATA_CONFIRM_IND:
      {
        AfDataConfirmInd *pInMsg;
        pInMsg = af_data_confirm_ind__unpack( NULL, len, pMsg );
        if ( pInMsg )
        {
          processAfDataConfirmInd( pInMsg );
          af_data_confirm_ind__free_unpacked( pInMsg, NULL );
        }
      }
      break;

    default:
      uiPrintf( "\nUndefined message received: %d\n\n", cmdID );
      break;
  }
}

/*******************************************************************************
 *
 * @fn          zotaRegEndpoint
 *
 * @brief       Send the AF Register Endpoint message
 *
 * @param       Simple Descriptor representation of the endpoint 
 *
 * @return      none
 *
 ******************************************************************************/
static bool zotaRegEndpoint(SimpleDescriptionFormat_t * simpleDescFmt )
{
  int i;
  int len;
  uint8 status;
  uint8 *pBuf;
  SimpleDescriptor simpleDesc = SIMPLE_DESCRIPTOR__INIT;
  AfRegisterReq afReq = AF_REGISTER_REQ__INIT;

  uiPrintfEx(trDEBUG, "Sending AF Register Request.\n" );

  afReq.cmdid = (ZStackCmdIDs) ZSTACK_CMD_IDS__AF_REGISTER_REQ;
  afReq.endpoint = simpleDescFmt->EndPoint;
  afReq.latencyreq = NETWORK_LATENCY__NO_LATENCY_REQS;

  simpleDesc.deviceid = simpleDescFmt->AppDeviceId;
  simpleDesc.devicever = simpleDescFmt->AppDevVer;
  simpleDesc.endpoint = simpleDescFmt->EndPoint;
  simpleDesc.n_inputclusters = simpleDescFmt->AppNumInClusters;

  if ( simpleDesc.n_inputclusters ) {
    simpleDesc.inputclusters = malloc( sizeof ( uint32_t ) * 
        simpleDesc.n_inputclusters );
    if ( simpleDesc.inputclusters ) {
      for ( i = 0; i < simpleDesc.n_inputclusters; i++ ) {
        simpleDesc.inputclusters[i] = simpleDescFmt->pAppInClusterList[i];
      }
    }
  }

  simpleDesc.n_outputclusters = simpleDescFmt->AppNumOutClusters;

  if ( simpleDesc.n_outputclusters ) {
    simpleDesc.outputclusters = malloc( sizeof ( uint32_t ) * 
        simpleDesc.n_outputclusters );
    if ( simpleDesc.outputclusters ) {
      for ( i = 0; i < simpleDesc.n_outputclusters; i++ ) {
        simpleDesc.outputclusters[i] = simpleDescFmt->pAppOutClusterList[i];
      }
    }
  }

  simpleDesc.profileid = simpleDescFmt->AppProfId;

  afReq.simpledesc = &simpleDesc;

  len = af_register_req__get_packed_size( &afReq );
  pBuf = malloc( len );

  if ( pBuf ) {
    af_register_req__pack( &afReq, pBuf );

    // Send the NPI message
    status = sendAPICExpectDefaultStatus( apiClientHandle, 
    ZSTACK_CMD_IDS__AF_REGISTER_REQ, len, pBuf );

    free( pBuf );
  }

  uiPrintfEx(trDEBUG,"AF Register Request status 0x%x\n", status);

  if (NULL != simpleDesc.outputclusters) free(simpleDesc.outputclusters);

  if (NULL != simpleDesc.inputclusters) free(simpleDesc.inputclusters);

  if (status != ZSuccess) {
    uiPrintfEx(trDEBUG,"*** Error doing AF Register , return status 0x%x\n", 
        status);
    return (FALSE);
  }

  return (TRUE);
}

/*******************************************************************************
 *
 * @fn          sendNwkInfoReadReq
 *
 * @brief       Send SYS Nwk Info Read Request message
 *
 * @param       none
 *
 * @return      none
 *
 ******************************************************************************/
static void sendNwkInfoReadReq( void )
{
  int len;
  uint8 *pBuf;
  uint8 rspcmdid;
  uint8 *pRsp;
  uint16 rsplen;

  SysNwkInfoReadReq niReq = SYS_NWK_INFO_READ_REQ__INIT;

  uiPrintfEx(trDEBUG, "Sending SYS Nwk Info Read Request.\n" );

  niReq.cmdid = (ZStackCmdIDs) ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_REQ;

  len = sys_nwk_info_read_req__get_packed_size( &niReq );
  pBuf = malloc( len );

  if ( pBuf ) {
    sys_nwk_info_read_req__pack( &niReq, pBuf );

    pRsp = apicSendSynchData( apiClientHandle, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF,
                              ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_REQ, len, pBuf,
                              NULL, &rspcmdid, &rsplen );

    if (pRsp) {
   
      if ( (ZSTACK_CMD_IDS__SYS_NWK_INFO_READ_RSP == rspcmdid) && (rsplen > 0) )
      {
        SysNwkInfoReadRsp *pNiRsp;
        pNiRsp = sys_nwk_info_read_rsp__unpack( NULL, rsplen, pRsp);

        if ( pNiRsp ) {
          memcpy( &nwkInfoRsp, pNiRsp, sizeof(SysNwkInfoReadRsp) );

          uiPrintfEx(trDEBUG, "nwkInfoReadRsp:");
	  uiPrintfEx(trDEBUG, "\t\t - nwkaddr:%x\n", nwkInfoRsp.nwkaddr);
	  uiPrintfEx(trDEBUG, "\t\t - ieeeaddr:%llx\n", nwkInfoRsp.ieeeaddr);
	  uiPrintfEx(trDEBUG, "\t\t - devstate:%d\n", nwkInfoRsp.devstate); 
          uiPrintfEx(trDEBUG, "\t\t - panid:%x\n", nwkInfoRsp.panid);
	  uiPrintfEx(trDEBUG, "\t\t - extendedpanid:%llx\n", nwkInfoRsp.extendedpanid );

          uiPrintfEx(trDEBUG, "\t\t - coordaddr:%x\n", nwkInfoRsp.coordaddr);
	  uiPrintfEx(trDEBUG, "\t\t - coordextaddr:%llx\n\n", nwkInfoRsp.coordextaddr );

          sys_nwk_info_read_rsp__free_unpacked( pNiRsp, NULL );
        }
      }
      else {
        uiPrintfEx(trDEBUG, "Expected NwkInfoReadRsp, got %d\n", rspcmdid);
      }
    }

    free( pBuf );
  }
}


/*******************************************************************************
 *
 * @fn          processDevStateChangeInd
 *
 * @brief       Process incoming Device State Change message
 *
 * @param       Msg of the type DevStateChangeInd 
 *
 * @return      none
 *
 ******************************************************************************/
static void processDevStateChangeInd( DevStateChangeInd *pStateChng )
{
  uiPrintf( "\n\nDevice State Change: state:%d\n\n", pStateChng->state );
  sendNwkInfoReadReq();
}

/*******************************************************************************
 *
 * @fn          processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param	Message of type AfIncomingMsgInd 
 *
 * @return      none
 *
 ******************************************************************************/
static void processAfIncomingMsgInd( AfIncomingMsgInd *pInMsg )
{
  afIncomingMSGPacket_t afMsg;

  /*
  uiPrintfEx(trDEBUG, "\nAfIncomingMsgInd: clusterID:%d\n", pInMsg->clusterid );
 */

  afMsg.groupId = pInMsg->groupid;
  afMsg.clusterId = pInMsg->clusterid;
  afMsg.srcAddr.endPoint = pInMsg->srcaddr->endpoint;
  afMsg.srcAddr.panId = pInMsg->srcaddr->panid;
  afMsg.srcAddr.addrMode = pInMsg->srcaddr->addrmode;
  if ( (afMsg.srcAddr.addrMode == afAddr16Bit) ||
       (afMsg.srcAddr.addrMode == afAddrGroup)
       || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
  {
    afMsg.srcAddr.addr.shortAddr =  pInMsg->srcaddr->shortaddr;
  }
  else if ( afMsg.srcAddr.addrMode == afAddr64Bit )
  {
    memcpy( afMsg.srcAddr.addr.extAddr, &(pInMsg->srcaddr->extaddr), 8 );
  }
  afMsg.macDestAddr = pInMsg->macdestaddr;
  afMsg.endPoint = pInMsg->endpoint;
  afMsg.wasBroadcast = pInMsg->wasbroadcast;
  afMsg.LinkQuality = pInMsg->linkquality;
  afMsg.correlation = pInMsg->correlation;
  afMsg.rssi = pInMsg->rssi;
  afMsg.SecurityUse = pInMsg->securityuse;
  afMsg.timestamp = pInMsg->timestamp;
  afMsg.nwkSeqNum = pInMsg->nwkseqnum;
  afMsg.macSrcAddr = pInMsg->macsrcaddr;
  afMsg.cmd.TransSeqNumber = pInMsg->transseqnum;
  afMsg.cmd.DataLength = pInMsg->payload.len;
  afMsg.cmd.Data = pInMsg->payload.data;

/*
  uiPrintfEx(trDEBUG, "\n\nAF Incoming Msg Ind: endpoint: %d, srcaddr:0x%x, "
    "cluster id: 0x%x, transaction seq. no. %d\n", pInMsg->endpoint, 
    pInMsg->srcaddr->shortaddr, pInMsg->clusterid, pInMsg->transseqnum);
*/

  zcl_ProcessMessageMSG( &afMsg );
}

/*
 * @fn          processAfDataConfirmInd
 *
 * @brief       Process AF Data Confirm Message Indication message
 *
 * @param	Message of type AfDataConfirmInd
 *
 * @return      none
 *
 ******************************************************************************/
static void processAfDataConfirmInd( AfDataConfirmInd *pInMsg )
{

  if (pInMsg->status != 0) {
    uiPrintfEx(trDEBUG, "Error******  \n"
        " AF Data Confirm Msg: endpoint: %d, status:%d, "
        "cmdID: %d, transID:%d\n\n", pInMsg->endpoint, pInMsg->status, 
        pInMsg->cmdid, pInMsg->transid);
  }

}


/******************************************************************************
 * @fn      zclOTA_PermitOta
 *
 * @brief   Called to enable/disable OTA operation.
 *
 * @param   permit - 0 to enable OTA, 1 to disable new OTA downloads, 2 to 
 * 	    disable all.
 *
 * @return  none
 */
void zclOTA_PermitOta(OtaEnableModes permit)
{
  FILE * file_ptr = NULL;
  uiPrintfEx(trDEBUG,"zclOTA_PermitOta: Changing OTA permission to 0x%x\n",
    (int)permit);
  zclOTA_Permit = permit;
  file_ptr = fopen(OTASERVER_CTXT_INFO, "w+b");
  if (NULL == file_ptr) {
    uiPrintfEx(trDEBUG,"zclOTA_PermitOta: Unable to save context of Permit status"
	" %d\n", zclOTA_Permit);
  }
  else {
    uiPrintfEx(trDEBUG,"zclOTA_PermitOta: Saving context of Permit status to %d\n",
	zclOTA_Permit);
    fwrite(&zclOTA_Permit, sizeof(zclOTA_Permit), 1, file_ptr);
    fflush(file_ptr);
    fclose(file_ptr);
  }
}



/*********************************************************************
 * @fn      basicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to  default values.
 *
 * @param   none
 *
 * @return  none
 */
static void basicResetCB( void )
{
  // Restart the application
}

/*********************************************************************
 * @fn      identifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void identifyCB( zclIdentify_t *pCmd )
{
  //zclSampleSw_IdentifyTime = pCmd->identifyTime;
  //zclSampleSw_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      identifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - source address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void identifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp )
{
  (void) pRsp;
}

static uint32 gettimeinms()
{
    uint32 msTime = 0;
    struct timeval retTime;
    if (-1 != gettimeofday(&retTime, NULL)) {
        msTime = retTime.tv_sec * 1000 + retTime.tv_usec/1000; 
    }	
    return msTime;	
}


/*******************************************************************************
 * APS Interface messages
 ******************************************************************************/
endPointDesc_t *afFindEndPointDesc( uint8 EndPoint )
{
  if ( EndPoint == zotaEpDesc.endPoint )
  {
    return ( &zotaEpDesc);
  }
  else
  {
   return ( (endPointDesc_t *)NULL );
  }
}

/******************************************************************************
 * @fn      zclOTA_ServerHdlIncoming
 *
 * @brief   Handle incoming server commands.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ServerHdlIncoming( zclIncoming_t *pInMsg )
{
  switch (pInMsg->hdr.commandID)
  {
    case COMMAND_QUERY_NEXT_IMAGE_REQ:
      return zclOTA_ProcessQueryNextImageReq( pInMsg );

    case COMMAND_IMAGE_BLOCK_REQ:
      return zclOTA_ProcessImageBlockReq( pInMsg );

    case COMMAND_IMAGE_PAGE_REQ:
      return zclOTA_ProcessImagePageReq( pInMsg );

    case COMMAND_UPGRADE_END_REQ:
      return zclOTA_ProcessUpgradeEndReq( pInMsg );

    case COMMAND_QUERY_SPECIFIC_FILE_REQ:
      return zclOTA_ProcessQuerySpecificFileReq( pInMsg );

/*    case ZCL_CMD_DEFAULT_RSP:
      return processDefaultResponse( pInMsg);
*/

    default:
      return ZFailure;
  }
}

uint8 processIncomingZCLCommands( zclIncoming_t *pInMsg )
{

  switch(pInMsg->hdr.commandID) {

/*
    case ZCL_CMD_READ_RSP:
      processZclReadRsp(pInMsg);
      break;

    case ZCL_CMD_DEFAULT_RSP:
      processDefaultResponse(pInMsg);
      break;
*/

    default:
      uiPrintfEx(trDEBUG,"processIncomingZCLCommands: Error: Unexpected command "
        "recvd 0x%x\n",pInMsg->hdr.commandID);
      break;
  }

  return ( TRUE );
}


/******************************************************************************
 * @fn      zclOTA_ProcessQueryNextImageReq
 *
 * @brief   Received "Query Next Image Request" from Client
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ProcessQueryNextImageReq( zclIncoming_t *pInMsg )
{
  zclOTA_QueryNextImageReqParams_t  param;
  uint8 *pData;

  /* verify message length */
  if ((pInMsg->pDataLen != PAYLOAD_MAX_LEN_QUERY_NEXT_IMAGE_REQ) &&
      (pInMsg->pDataLen != PAYLOAD_MIN_LEN_QUERY_NEXT_IMAGE_REQ))
  {
    /* no further processing if invalid */
    return ZCL_STATUS_MALFORMED_COMMAND;
  }

  /* parse message parameters */
  pData = pInMsg->pData;
  param.fieldControl = *pData++;
  param.fileId.manufacturer = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.type = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.version = zcl_build_uint32( pData, 4 );
  pData += 4;
  if ((param.fieldControl & 0x01) != 0)
  {
    param.hardwareVersion = BUILD_UINT16(pData[0], pData[1]);
  }

  /* call callback */
  return zclOTA_Srv_QueryNextImageReq(&pInMsg->msg->srcAddr, &param);
}

/******************************************************************************
 * @fn      zclOTA_ProcessImageBlockReq
 *
 * @brief   Process received Image Block Request.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ProcessImageBlockReq( zclIncoming_t *pInMsg )
{
  zclOTA_ImageBlockReqParams_t  param;
  uint8 *pData;

  uiPrintfEx(trDEBUG,"zclOTA_ProcessImageBlockReq: Recvd image block request\n");

  /* verify message length */
  if ( ( pInMsg->pDataLen > PAYLOAD_MAX_LEN_IMAGE_BLOCK_REQ ) &&
       ( pInMsg->pDataLen < PAYLOAD_MIN_LEN_IMAGE_BLOCK_REQ ) )
  {
    /* no further processing if invalid */
    return ZCL_STATUS_MALFORMED_COMMAND;
  }

  /* parse message parameters */
  pData = pInMsg->pData;
  param.fieldControl = *pData++;
  param.fileId.manufacturer = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.type = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.version = zcl_build_uint32( pData, 4 );
  pData += 4;
  param.fileOffset = zcl_build_uint32( pData, 4 );
  pData += 4;
  param.maxDataSize = *pData++;
  if ( ( param.fieldControl & OTA_BLOCK_FC_NODES_IEEE_PRESENT ) != 0 )
  {
    zcl_cpyExtAddr(param.nodeAddr, pData);
    pData += 8;
  }
  if ( ( param.fieldControl & OTA_BLOCK_FC_REQ_DELAY_PRESENT ) != 0 )
  {
    param.blockReqDelay = BUILD_UINT16(pData[0], pData[1]);
  }

  uiPrintfEx(trDEBUG,"zclOTA_ProcessImageBlockReq: calling ImageBlockReq cb\n");
  /* call callback */
  return zclOTA_Srv_ImageBlockReq(&pInMsg->msg->srcAddr, &param);
}

/******************************************************************************
 * @fn      zclOTA_ProcessImagePageReq
 *
 * @brief   Process received Image Page Request.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ProcessImagePageReq( zclIncoming_t *pInMsg )
{
  zclOTA_ImagePageReqParams_t  param;
  uint8 *pData;

  /* verify message length */
  if ((pInMsg->pDataLen != PAYLOAD_MAX_LEN_IMAGE_PAGE_REQ) &&
      (pInMsg->pDataLen != PAYLOAD_MIN_LEN_IMAGE_PAGE_REQ))
  {
    /* no further processing if invalid */
    return ZCL_STATUS_MALFORMED_COMMAND;
  }

  /* parse message parameters */
  pData = pInMsg->pData;
  param.fieldControl = *pData++;
  param.fileId.manufacturer = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.type = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.version = zcl_build_uint32( pData, 4 );
  pData += 4;
  param.fileOffset = zcl_build_uint32( pData, 4 );
  pData += 4;
  param.maxDataSize = *pData++;
  param.pageSize = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.responseSpacing = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  if ((param.fieldControl & 0x01) != 0)
  {
    zcl_cpyExtAddr(param.nodeAddr, pData);
  }

  /* call callback */
  return zclOTA_Srv_ImagePageReq(&pInMsg->msg->srcAddr, &param);
}

/******************************************************************************
 * @fn      zclOTA_ProcessUpgradeEndReq
 *
 * @brief   Process received Upgrade End Request.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ProcessUpgradeEndReq( zclIncoming_t *pInMsg )
{
  zclOTA_UpgradeEndReqParams_t  param;
  uint8 *pData;

  /* verify message length */
  if ((pInMsg->pDataLen != PAYLOAD_MAX_LEN_UPGRADE_END_REQ) &&
      (pInMsg->pDataLen != PAYLOAD_MIN_LEN_UPGRADE_END_REQ))
  {
    /* no further processing if invalid */
    return ZCL_STATUS_MALFORMED_COMMAND;
  }

  /* parse message parameters */
  pData = pInMsg->pData;
  param.status = *pData++;
 
  //if (param.status == ZCL_STATUS_SUCCESS)
  {
    param.fileId.manufacturer = BUILD_UINT16(pData[0], pData[1]);
    pData += 2;
    param.fileId.type = BUILD_UINT16(pData[0], pData[1]);
    pData += 2;
    param.fileId.version = zcl_build_uint32( pData, 4 );
  }

  /* call callback */
  return zclOTA_Srv_UpgradeEndReq(&pInMsg->msg->srcAddr, &param);
}

/******************************************************************************
 * @fn      zclOTA_ProcessQuerySpecificFileReq
 *
 * @brief   Process received Image Page Request.
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static ZStatus_t zclOTA_ProcessQuerySpecificFileReq( zclIncoming_t *pInMsg )
{
  zclOTA_QuerySpecificFileReqParams_t  param;
  uint8 *pData;

  /* verify message length */
  if (pInMsg->pDataLen != PAYLOAD_MAX_LEN_QUERY_SPECIFIC_FILE_REQ)
  {
    /* no further processing if invalid */
    return ZCL_STATUS_MALFORMED_COMMAND;
  }

  /* parse message parameters */
  pData = pInMsg->pData;
  zcl_cpyExtAddr(param.nodeAddr, pData);
  pData += Z_EXTADDR_LEN;
  param.fileId.manufacturer = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.type = BUILD_UINT16(pData[0], pData[1]);
  pData += 2;
  param.fileId.version = zcl_build_uint32( pData, 4 );
  pData += 4;
  param.stackVersion = BUILD_UINT16(pData[0], pData[1]);

  /* call callback */
  return zclOTA_Srv_QuerySpecificFileReq(&pInMsg->msg->srcAddr, &param);
}

/******************************************************************************
 * @fn      zclOTA_Srv_QueryNextImageReq
 *
 * @brief   Handle a "Query Next Image" Request.
 *
 * @param   pSrcAddr - The source of the message
 *          pParam -   message parameters
 *
 * @return  ZStatus_t
 *
 * @note    On a query next image, the server sends back a response that 
 * 	    indicates the availability of an image matching the  
 * 	    manufacturer code, image type, and the highest available file 
 *          version of that image on the server. 
 */
ZStatus_t zclOTA_Srv_QueryNextImageReq(afAddrType_t *pSrcAddr, 
    zclOTA_QueryNextImageReqParams_t *pParam)
{
  uint8 options = 0;
  uint8 status;
  uint32 imageSize;
  zclOTA_QueryImageRspParams_t queryRsp;

  //Respond to a "Query Next" only if Permit set to 0
  if (zclOTA_Permit == OTA_ENABLE_MODES__DOWNLOAD_ENABLE) {

    if (pParam->fieldControl)
    {
      options |= MT_OTA_HW_VER_PRESENT_OPTION;
    }

    // Request the next image for this device from image database 
    status = OtaServer_GetImage(&pParam->fileId, &imageSize, 
            pParam->hardwareVersion, NULL, options, pSrcAddr);

    uiPrintfEx(trDEBUG,"zclOTA_Srv_QueryNextImageReq: Match image for manu 0x%x"
        " type 0x%x file version 0x%x returned with status 0x%x\n", 
        pParam->fileId.manufacturer, pParam->fileId.type, 
        pParam->fileId.version, status);

    if (status == OTASERVER_SUCCESS) {
      // Fill in the response parameters
      memcpy(&queryRsp.fileId, &pParam->fileId, sizeof(zclOTA_FileID_t));
      queryRsp.status = ZSuccess;
      queryRsp.imageSize = imageSize; //Get Filesize using fileId ?!

      uiPrintfEx(trDEBUG,"zclOTA_Srv_QueryNextImageReq: File size 0x%x\n", 
        imageSize);
      // Send a success response to the client
      zclOTA_SendQueryNextImageRsp(pSrcAddr, &queryRsp, ++zclOTA_SeqNo);

      return ZCL_STATUS_CMD_HAS_RSP;
    }
    else queryRsp.status = ZOtaNoImageAvailable;
  }
  else {
   
    //No image available, and not authorized. What should I use in case of
    //abort ?
    //queryRsp.status = ZOtaAbort;
    queryRsp.status = ZOtaNoImageAvailable;

    //Download status sent only on Upgrade End Req
    //sendDownloadStatus(appConnectionHandle, pSrcAddr, OTA_STATUS__ABORT);
  }

  // Fill in the response parameters
  memcpy(&queryRsp.fileId, &pParam->fileId, sizeof(zclOTA_FileID_t));
  queryRsp.imageSize = 0;

  //Send failure response
  zclOTA_SendQueryNextImageRsp(pSrcAddr, &queryRsp, ++zclOTA_SeqNo);

  return ZCL_STATUS_CMD_HAS_RSP;
}

/******************************************************************************
 * @fn      zclOTA_Srv_ImageBlockReq
 *
 * @brief   Handle an Image Block Request.
 *
 * @param   pSrcAddr - The source of the message
 *          pParam - message parameters
 *
 * @return  ZStatus_t
 *
 * @note    On an Image Block Request, the server sends back a block of data 
 * 	    corresponding to the fileId in the ImageBlockReqParams from the  
 *          offset specified.	
 */
ZStatus_t zclOTA_Srv_ImageBlockReq(afAddrType_t *pSrcAddr, 
    zclOTA_ImageBlockReqParams_t *pParam)
{
  uint8 status = ZFailure;

  if (zclOTA_Permit != OTA_ENABLE_MODES__DOWNLOAD_DISABLE) {

    uint8 len = pParam->maxDataSize;

    if (len > OTA_MAX_MTU) {
      len = OTA_MAX_MTU;
    }

    // check if client supports rate limiting feature, and if client rate 
    // needs to be set
    if ( ( ( pParam->fieldControl & OTA_BLOCK_FC_REQ_DELAY_PRESENT ) != 0 ) &&
        ( pParam->blockReqDelay != OTASERVER_MINBLOCKREQUEST_DELAY) ) {
      zclOTA_ImageBlockRspParams_t blockRsp;

      uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Field control enabled. \n");
      // Fill in the response parameters
      blockRsp.status = ZOtaWaitForData;
      memcpy(&blockRsp.rsp.success.fileId, &pParam->fileId, 
            sizeof(zclOTA_FileID_t));
      blockRsp.rsp.wait.currentTime = 0;
      blockRsp.rsp.wait.requestTime = 0;
      blockRsp.rsp.wait.blockReqDelay = OTASERVER_MINBLOCKREQUEST_DELAY;

      // Send a wait response with updated rate limit timing
      uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Sending block response with "
        "rate limiting. \n");
      zclOTA_SendImageBlockRsp(pSrcAddr, &blockRsp, ++zclOTA_SeqNo);
      status = ZCL_STATUS_CMD_HAS_RSP;
    }
    else {

      int readLen = 0;
      uint8 rspBuf[512], *pRsp = rspBuf ;
      
      //Read image into pRsp
      readLen = OtaServer_GetImageBlock(&pParam->fileId, pParam->fileOffset, 
            pRsp, len);
      //imgLen = OtaServer_GetImageSize(&pParam->fileId);
          
      /*
      uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Obtained image with len 0x%x\n"
        ,readLen);
      */
      if (readLen != 0) status = ZSuccess;
      
      if (status == ZSuccess) {

        zclOTA_ImageBlockRspParams_t blockRsp;
        //On Success
        blockRsp.status = ZSuccess;
        memcpy(&blockRsp.rsp.success.fileId, &pParam->fileId, 
        sizeof(zclOTA_FileID_t));
        blockRsp.rsp.success.fileOffset = pParam->fileOffset;
        blockRsp.rsp.success.dataSize = readLen;
        blockRsp.rsp.success.pData = pRsp;
     
        uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Sending block Rsp with "
            "fileOffset 0x%x\n", pParam->fileOffset);

        zclOTA_SendImageBlockRsp(pSrcAddr, &blockRsp, ++zclOTA_SeqNo); 
        status = ZCL_STATUS_CMD_HAS_RSP;
      }
      else {

        //Spec says to send "NoImageAvailable" (6.10.6.5.2)
        status = ZOtaNoImageAvailable;

        uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: ERROR read failed "
        "status is ZOtaNoImageAvailable\n");
#if 0
        blockRsp.status = ZOtaNoImageAvailable; //ZOtaAbort; //ZOtaWaitForData;
        memcpy(&blockRsp.rsp.success.fileId, &pParam->fileId, 
        sizeof(zclOTA_FileID_t));
        blockRsp.rsp.wait.currentTime = 0;
        blockRsp.rsp.wait.requestTime = OTA_SEND_BLOCK_WAIT;
        blockRsp.rsp.wait.blockReqDelay = zclOTA_MinBlockReqDelay;
        // Send the block to the peer
        zclOTA_SendImageBlockRsp(pSrcAddr, &blockRsp, ++zclOTA_SeqNo);
#endif
      }
    }
  }
  else {

    uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: OTA DISABLE\n");

    zclOTA_ImageBlockRspParams_t blockRsp;
    blockRsp.status = ZOtaAbort;
    uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Sending ABORT response to peer\n");
    zclOTA_SendImageBlockRsp(pSrcAddr, &blockRsp, ++zclOTA_SeqNo);
    status = ZCL_STATUS_CMD_HAS_RSP;
  }

  //uiPrintfEx(trDEBUG,"zclOTA_Srv_ImageBlockReq: Returning status 0x%x\n", status); 

  return status;
}

/******************************************************************************
 * @fn      zclOTA_Srv_ImagePageReq
 *
 * @brief   Handle an Image Page Request.  Note: Not currently supported.
 *
 * @param   pSrcAddr - The source of the message
 *          pParam - message parameters
 *
 * @return  ZStatus_t
 *
 * @note    Not supported. 
 */
ZStatus_t zclOTA_Srv_ImagePageReq(afAddrType_t *pSrcAddr, 
    zclOTA_ImagePageReqParams_t *pParam)
{
  // Send not supported resposne
  return ZUnsupClusterCmd;
}

/******************************************************************************
 * @fn      zclOTA_Srv_UpgradeEndReq
 *
 * @brief   Handle an Upgrade End Request.
 *
 * @param   pSrcAddr - The source of the message
 *          pParam - message parameters
 *
 * @return  ZStatus_t
 *
 * @note    Processes an upgrade end request from the client. On SUCCESS server 
 * 	    	reples with the Upgrade End Response indicating when the client 
 * 	    	shall upgrade to the newly retrieved image. For other status values 
 *          it shall send default response command with status of success and it
 *          shall wait for the client to reinitiate the upgrade process.
 */
ZStatus_t zclOTA_Srv_UpgradeEndReq(afAddrType_t *pSrcAddr, 
    zclOTA_UpgradeEndReqParams_t *pParam)
{
  uint8 status = ZSuccess;

  OtaStatus pUpdateStatus = OTA_STATUS__OTA_SUCCESS;

  //Check Permit setting 
  if ((zclOTA_Permit != OTA_ENABLE_MODES__DOWNLOAD_DISABLE)) {

    zclOTA_UpgradeEndRspParams_t rspParms;

    if (pParam->status == ZSuccess) {

      uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Upgrade End Response with "
        "status SUCCESS\n");

      OTA_UpgradeInstance_t * pItem = NULL;

      //Obtain UpgradeInstance pointer
      pItem = OtaServer_GetInstance(&pParam->fileId);

      if (pItem == NULL) {
        uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: No file found\n"); 
        return (status);
      }
      else {

        //rspParms.disableDefaultResp = TRUE;

        //Check execution settings
        switch(pItem->executionType) {

          case OTA_EXECUTE_TYPE__IMMEDIATE:

            uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Execute now\n");

            rspParms.currentTime = 0x0;
            rspParms.upgradeTime = 0x0;
            break;

          case OTA_EXECUTE_TYPE__DELAY:
            //Assuming no ZCL Time cluster implementation
            uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Execute with delay "
                " of 0x%x\n", pItem->executionDelay);
            rspParms.currentTime = 0x0;
            rspParms.upgradeTime = pItem->executionDelay;
            break;

          case OTA_EXECUTE_TYPE__TIME:
            rspParms.currentTime = gettimeinms()/1000; //Round up time to secs
            rspParms.upgradeTime = pItem->executionTime;
                uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Execute at time 0x%x"
            "\n",pItem->executionTime);
            break;

          case OTA_EXECUTE_TYPE__HOLD: 
            rspParms.currentTime = 0xFFFFFFFF; 
            rspParms.upgradeTime = 0xFFFFFFFF;
            uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Hold execution\n");
            break;

          default:
            uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Bad EXECUTE TYPE arg\n");
            return (status);
        }

        //Send out Upgrade End response
        memcpy(&rspParms.fileId, &pParam->fileId, sizeof(zclOTA_FileID_t));
        zclOTA_SendUpgradeEndRsp(pSrcAddr, &rspParms, ++zclOTA_SeqNo);

        //Send out OTA_UPDATE_DL_FINISHED_IND
        pUpdateStatus = OTA_STATUS__OTA_SUCCESS;
      }
    }
    else if (pParam->status == ZOtaAbort) {

      pUpdateStatus = OTA_STATUS__ABORT;
      uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Status is ABORT\n"); 

    }
    else if (pParam->status == ZOtaImageInvalid) {

      //Bad return status from Upgrade end request	
      uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Invalid Image.\n"); 
      pUpdateStatus = OTA_STATUS__INVALID_IMAGE;
    }
    else if (pParam->status == ZOtaRequireMoreImage) {

      uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: Require More Image.\n");
      pUpdateStatus = OTA_STATUS__REQUIRE_MORE_IMAGE;
    }

    status = ZCL_STATUS_CMD_HAS_RSP;
  }
  else {
    uiPrintfEx(trDEBUG,"zclOTA_Srv_UpgradeEndReq: ABORT\n"); 

    pUpdateStatus = OTA_STATUS__ABORT;
  }

  sendDownloadStatus(appConnectionHandle, pSrcAddr, pUpdateStatus);

  return status;
}

/******************************************************************************
 * @fn      zclOTA_Srv_QuerySpecificFileReq
 *
 * @brief   Handles a Query Specific File Request.
 *
 * @param   pSrcAddr - The source of the message
 *          pParam - message parameters
 *
 * @return  ZStatus_t
 * 
 * @note    Requests a specific file from the Server and it responds with
 * 	    correspondign status. 
 */
ZStatus_t zclOTA_Srv_QuerySpecificFileReq(afAddrType_t *pSrcAddr, 
    zclOTA_QuerySpecificFileReqParams_t *pParam)
{
  uint8 status = ZFailure;
  uint32 imageSize = 0;
  zclOTA_QueryImageRspParams_t queryRsp;

  // Request the image from the console
  if (zclOTA_Permit != OTA_ENABLE_MODES__DOWNLOAD_DISABLE) {

    if (OTASERVER_SUCCESS == OtaServer_GetImage(&pParam->fileId, &imageSize, 
        0, pParam->nodeAddr, MT_OTA_QUERY_SPECIFIC_OPTION, pSrcAddr))
    {
      status = ZSuccess;
    }
    else status = ZOtaNoImageAvailable;
  }
  else {
    status = ZOtaNoImageAvailable; //Since no error for Abort supported
  }

  // Fill in the response parameters
  memcpy(&queryRsp.fileId, &pParam->fileId, sizeof(zclOTA_FileID_t));
  queryRsp.status = status;
  queryRsp.imageSize = imageSize;

  // Send a response to the client
  zclOTA_SendQuerySpecificFileRsp(pSrcAddr, &queryRsp, ++zclOTA_SeqNo);

  return ZCL_STATUS_CMD_HAS_RSP;
}

/******************************************************************************
 * @fn      sendDownloadStatus
 *
 * @brief   Sends Download Status notification to Linux Gateway API  
 *
 * @param   connection - connection to the API server 
 *          pSrcAddr - node address
 * 	    status - status of download
 * 	    fileId - fileId of the image being downloaded
 *
 * @return  ZStatus_t
 * 
 * @note    Sends the Download status of the image to the Gateway API 
 */
static void sendDownloadStatus(int connection, afAddrType_t * pSrcAddr,
    OtaStatus status)
{
  OtaUpdateDlFinishedInd pDlStatus = OTA_UPDATE_DL_FINISHED_IND__INIT;
  AddressStruct address = ADDRESS_STRUCT__INIT;
  uint8 len = 0;
  uint8 * pBuf = NULL;

  pDlStatus.cmdid = OTA_MGR_CMD_ID_T__OTA_UPDATE_DL_FINISHED_IND;
  pDlStatus.status = status;

  address.addrmode = ADDRESS_MODE__UNICAST;

  if (pSrcAddr->addrMode == afAddr64Bit) {
    address.has_ieeeaddr = TRUE;
    memcpy(&address.ieeeaddr, pSrcAddr->addr.extAddr, Z_EXTADDR_LEN);
  }
  else if (pSrcAddr->addrMode == afAddr16Bit) {

    //Call Network Manager API to convert 16bit address to ieee address
    if (!(gwPb_SrvrGetIeeeAddress(pSrcAddr->addr.shortAddr, &address.ieeeaddr)))
    {
      uiPrintfEx(trDEBUG,"sendDownloadStatus: Error generating ieee addr from "
            "short address 0x%x\n",pSrcAddr->addr.shortAddr);
    }
    else address.has_ieeeaddr = TRUE;
  }
  else {
    uiPrintfEx(trDEBUG,"sendDownloadStatus: Unrecognized addrMode %d\n", 
        pSrcAddr->addrMode);
    return; //Don't send a message, if you don't have a valid addrMode
  }

  address.has_endpointid = FALSE; 
  //address.endpointid = pSrcAddr->endPoint;

  pDlStatus.address = &address;

  len = ota_update_dl_finished_ind__get_packed_size(&pDlStatus);
  pBuf = malloc ( len );
 
  if (pBuf) {
    ota_update_dl_finished_ind__pack(&pDlStatus, pBuf);

    APIS_SendData(connection, ZSTACK_OTASYS_IDS__RPC_SYS_PB_OTA_MGR, 
            TRUE, OTA_MGR_CMD_ID_T__OTA_UPDATE_DL_FINISHED_IND, len, pBuf);

    free(pBuf);
  }
}

/*******************************************************************************
 *
 * @fn          sendAPICExpectDefaultStatus
 *
 * @brief       Send a request message and expect the normal "default" response (MacDefaultRsp)
 *
 * @param       handle - api client handle 
 * @param       cmdID - messages command ID
 * @param       len - length of pData buffer to send
 * @param       pData - pointer to buffer to send
 *
 * @return      synchronous return status
 *
*******************************************************************************/
ZStatusValues sendAPICExpectDefaultStatus( apicHandle_t handle, int cmdID, int len, uint8 *pData )
{
  uint8 rspCmdId;
  uint8 *pRsp;  
  uint16 rspLen;
  ZstackDefaultRsp *pDefaultRsp;
  ZStatusValues status = ZSTATUS_VALUES__ZDecodeError;

  // send serialized request to API Client synchronously
  pRsp = apicSendSynchData( handle, ZSTACK_SYS_IDS__RPC_SYS_PROTOBUF, 
                            cmdID, len, pData,
                            NULL, &rspCmdId, &rspLen );

  if ( pRsp )
  {
    if ( (cmdID == rspCmdId) && (rspLen > 0) )
    {
      pDefaultRsp = zstack_default_rsp__unpack( NULL, rspLen, pRsp );
      if ( pDefaultRsp )
      {
        status = pDefaultRsp->status;
        zstack_default_rsp__free_unpacked( pDefaultRsp, NULL );
      }
    }
    apicFreeSynchData( pRsp );
  }

  // return the status
  return (status);
}
