#ifndef API_SERVER_H
#define API_SERVER_H

/*********************************************************************
 Filename:       api_server.h
 Revised:        $Date$
 Revision:       $Revision$

 Description:   API Server module

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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 *********************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "api_client.h"
#include "configparser.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Size in bytes of the stack for an internal thread
// created by APIS server
// Application may change the value before making APIS_Init() call
// to customize the stack size.
extern size_t APIS_threadStackSize;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define API_SERVER_DEFAULT_PORT 2536

/*********************************************************************
 * TYPEDEFS
 */

// Type of callback or the reason for the callback.
#define  SERVER_DATA         0    // A data message received on a connection
#define  SERVER_CONNECT      1    // A new connection
#define  SERVER_DISCONNECT   2    // A connection has been removed
// This function pointer typedef is used by the API server to notify
// when a message is received or when the connection is removed.
// "type" indicates the reason for the callback (SERVER_DATA,
// SERVER_CONNECT, or SERVER_DISCONNECT)
typedef void (*pfnAPISMsgCB)( int connection, uint8 subSys, uint8 cmdId,
                              uint16 len, uint8 *pData, uint8 cbType );

typedef struct
{
  int port;                 // Listening Port
  bool serverVerbose; // TRUE to display (console) the server information, FALSE if not
  configTableItem_t *pConfigDesc;  // pointer to configuration parameters
  int numConfigDescs;       // Number of records in pConfigDesc
  int numClients;           // Number of API clients
  pfnAsyncMsgCb pfNPICB; // pointer to function that handles incoming NPI messages and
                         // a remove of a connection.
  pfnAPISMsgCB pfServerCB; // pointer to function that handles incoming server messages
} apisSysParams_t;

/*********************************************************************
 * Application Interface
 */

/*********************************************************************
 * @fn      appArgs
 *
 * @brief   Called to pass cmdline to application
 *
 * @param   pointers to argc and argv
 *
 * @return  if the application pulls arguments out of argv,
 *          it must modify argc and argv appropriately.
 *			normally the start of argv doesn't need to change
 *			because that is the program name (not an argument)
 *			for an example of how to parse the arguments see
 *			appArgs in nwkmgr/nwkmgrsrv.c
 *
 *			if an application doesn't need any special cmdline parms,
 *			it is sufficient to create a stub function that does 
 *			nothing but return 0
 *
 *********************************************************************/
extern int appArgs( int *p_argc, char ***p_argv );

/*********************************************************************
 * @fn      appInit
 *
 * @brief   Called to return the needed configuration information.
 *
 * @param   none
 *
 * @return  pointer to the application' parameter function
 *
 *********************************************************************/
extern apisSysParams_t *appInit( void );

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Main loop of the application, this function will not end
 *          until the app is completely done.
 *
 * @param   handles - array of API client handles.
 *                    The size shall correspond to numClients field value
 *                    of apisSysParams_t data structure returned from
 *                    appInit() function.
 *
 * @return  error code
 *
 *********************************************************************/
extern int appMain( apicHandle_t *handles );

/*********************************************************************
 * API Server Functions
 */

/*********************************************************************
 * @fn      APIS_Init
 *
 * @brief   Start and maintain an API Server
 *
 * @param   port - UDP server to listen on
 * @param   verbose - TRUE to display startup information
 * @param   pfCB - function pointer to function
 *
 * @return  TRUE if started server, FALSE if error
 *
 *********************************************************************/
extern bool APIS_Init( int port, bool verbose, pfnAPISMsgCB pfCB );

/*********************************************************************
 * @fn      APIS_SendData
 *
 * @brief   Send a packet to a connection
 *
 * @param   connection - connection to send message (for synchronous
 *                       response) otherwise -1 for all connections
 * @param   sysID - MT Format system ID (ie. RPC_SYS_PROTOBUF)
 * @param   areq - TRUE if AREQ message, FALSE if SRSP message
 * @param   cmdId - command ID
 * @param   len - length to send
 * @param   pData - pointer to buffer to send
 *
 * @return  status
 *
 *********************************************************************/
extern void APIS_SendData( int connection, uint8 sysID, bool areq, uint8 cmdId,
                           uint16 len, uint8 *pData );

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif // API_SERVER_H
