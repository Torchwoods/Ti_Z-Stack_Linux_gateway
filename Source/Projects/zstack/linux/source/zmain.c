/**************************************************************************************************
 Filename:       zmain.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    Main loop and intialization


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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <poll.h>

#include "hal_types.h"
#include "../srvwrapper/api_client.h"
#include "../srvwrapper/hal_rpc.h"
#include "api_server.h"
#include "zstackpb.h"
#include "trace.h"

#include "OSAL.h"
#include "OSAL_Nv.h"
#include "ZComDef.h"
#include "OnBoard.h"

#include "ZGlobals.h"
#if !defined ( LINUX_ZNP )
#include "ZMAC.h"
#include "nwk_globals.h"
#endif

/**************************************************************************************************
 * Constant
 **************************************************************************************************/


//set the poll time out to not timeout
#define POLL_TIMEOUT -1

#define ZSTACK_SERVER_DEFAULT_PORT API_SERVER_DEFAULT_PORT

#define DEFAULT_OSALNV_FILENAME "ZStackOsalNVFile"

/**************************************************************************************************
 * Typedefs
 **************************************************************************************************/

/**************************************************************************************************
 * Globals
 **************************************************************************************************/
apicHandle_t zmainClientHandle;

/**************************************************************************************************
 * Locals
 **************************************************************************************************/
struct pollfd* pollFds;
int numPollFds;
struct timespec ppollTimeout =
{ .tv_sec = 0, .tv_nsec = 0 };

#ifdef USE_KEY
extern int keyFd;
int keyFdIdx;
#endif

static int polltimeOut = POLL_TIMEOUT;

#define MAX_DEV_STR_LEN 128
char gZNP_DEVICE[MAX_DEV_STR_LEN];

char osalNvFileName[] = DEFAULT_OSALNV_FILENAME;

static bool pollWait = FALSE;
static pid_t pollPid = 0;

/**************************************************************************************************
 * Function Prototypes
 **************************************************************************************************/
static void initPoll( void );

extern void osal_GetTimerFds( int *fds, int maxFds );
extern void handleAsyncMsgs( apicHandle_t handle, uint8 subSys, uint8 cmdID,
                             uint16 len, uint8 *pMsg );
extern void HalKeyPoll( void );
extern void ZMacSetPBMode( bool mode );

extern uint8 zspbPermitJoin;
extern uint8 nwkUseMultiCast;
extern uint16 jammerContinuousEvents;
extern uint8 jammerHighNoiseLevel;
extern uint32 jammerDetectPeriodTime;
extern uint8 maxSupportedEndpoints;

/**************************************************************************************************
 * Globals
 **************************************************************************************************/
apisSysParams_t sysParams;

#if !defined ( LINUX_ZNP )
  // Parameter Descriptors
  configTableItem_t configItems[] =
  {
   { &(sysParams.port), "SERVER_PORT", TYPE_UINT16, 1 },
   { &zgDefaultChannelList, "DEFAULT_CHANLIST", TYPE_UINT32, 1 },
   { &zgConfigPANID, "ZDAPP_CONFIG_PAN_ID", TYPE_UINT16, 1 },
   { &zgRouteExpiryTime, "ROUTE_EXPIRY_TIME", TYPE_UINT8, 1 },
   { &zgApscAckWaitDurationPolled, "APSC_ACK_WAIT_DURATION_POLLED", TYPE_UINT16, 1 },
   { &zgIndirectMsgTimeout, "NWK_INDIRECT_MSG_TIMEOUT", TYPE_UINT8, 1 },
   { &gMAX_RREQ_ENTRIES, "MAX_RREQ_ENTRIES", TYPE_UINT16, 1 },
   { &zgApscMaxFrameRetries, "APSC_MAX_FRAME_RETRIES", TYPE_UINT8, 1 },
   { &zgMaxDataRetries, "NWK_MAX_DATA_RETRIES", TYPE_UINT8, 1 },
   { &zgMaxPollFailureRetries, "MAX_POLL_FAILURE_RETRIES", TYPE_UINT8, 1 },
   { &gMAX_BCAST, "MAX_BCAST", TYPE_UINT16, 1 },
   { &gAPS_MAX_GROUPS, "APS_MAX_GROUPS", TYPE_UINT8, 1 },
   { &gMAX_RTG_ENTRIES, "MAX_RTG_ENTRIES", TYPE_UINT16, 1 },
   { &gNWK_MAX_BINDING_ENTRIES, "NWK_MAX_BINDING_ENTRIES", TYPE_UINT16, 1 },
   { &gMAX_BINDING_CLUSTER_IDS, "MAX_BINDING_CLUSTER_IDS", TYPE_UINT8, 1 },
   { &defaultKey, "DEFAULT_KEY", TYPE_UINT8, 16 },
   { &zgNwkMgrMinTransmissions, "ZDNWKMGR_MIN_TRANSMISSIONS", TYPE_UINT8, 1 },
   { &zgPollRate, "POLL_RATE", TYPE_UINT16, 1 },
   { &zgQueuedPollRate, "QUEUED_POLL_RATE", TYPE_UINT16, 1 },
   { &zgResponsePollRate, "RESPONSE_POLL_RATE", TYPE_UINT16, 1 },
   { &zgRejoinPollRate, "REJOIN_POLL_RATE", TYPE_UINT16, 1 },
   { &gMAX_NEIGHBOR_ENTRIES,"MAX_NEIGHBOR_ENTRIES", TYPE_UINT16, 1 },
   { &gMAX_RTG_SRC_ENTRIES, "MAX_RTG_SRC_ENTRIES", TYPE_UINT16, 1 },
   { &zspbPermitJoin, "PERMIT_JOIN", TYPE_UINT8, 1 },
   { &jammerContinuousEvents, "JAMMER_CONTINUOUS_EVENTS", TYPE_UINT16, 1 },
   { &jammerHighNoiseLevel, "JAMMER_HIGH_NOISE_LEVEL", TYPE_UINT8, 1 },
   { &jammerDetectPeriodTime, "JAMMER_DETECT_PERIOD_TIME", TYPE_UINT32, 1 },
   { &nwkUseMultiCast, "NWK_USE_MULTICAST", TYPE_UINT8, 1 },
   { &maxSupportedEndpoints, "MAX_SUPPORTED_ENDPOINTS", TYPE_UINT8, 1 },
  };
#else
  uint8 deviceType = 0;

  // Parameter Descriptors
  configTableItem_t configItems[] =
  {
   { &zgDefaultChannelList, "DEFAULT_CHANLIST", TYPE_UINT32, 1 },
   { &zgConfigPANID, "ZDAPP_CONFIG_PAN_ID", TYPE_UINT16, 1 },
   { &deviceType, "DEVICE_TYPE", TYPE_UINT8, 1 },
   { &(sysParams.port), "SERVER_PORT", TYPE_UINT16, 1 },
   { &zspbPermitJoin, "PERMIT_JOIN", TYPE_UINT8, 1 },
   { &jammerContinuousEvents, "JAMMER_CONTINUOUS_EVENTS", TYPE_UINT16, 1 },
   { &jammerHighNoiseLevel, "JAMMER_HIGH_NOISE_LEVEL", TYPE_UINT8, 1 },
   { &jammerDetectPeriodTime, "JAMMER_DETECT_PERIOD_TIME", TYPE_UINT32, 1 },
   { &nwkUseMultiCast, "NWK_USE_MULTICAST", TYPE_UINT8, 1 },
   { &maxSupportedEndpoints, "MAX_SUPPORTED_ENDPOINTS", TYPE_UINT8, 1 },
  };
#endif // LINUX_ZNP
apisSysParams_t sysParams =
{
  ZSTACK_SERVER_DEFAULT_PORT,          // Default port
  TRUE,                                // ZStack Server Verbose mode
  (configTableItem_t *)configItems,    // Configuration structure array
  (sizeof ( configItems ) / sizeof (configTableItem_t)),  //
  1,                                   // single API client
  handleAsyncMsgs,                     // function to handle incoming API Client messages
  zspbHandlePbCb                       // handles incoming ZStack Protobuf messages
};

/**************************************************************************************************
 **************************************************************************************************/

// Default signal handler for the process
void signal_handler( int signum )
{
  //uiPrintfEx(trUNMASKABLE,  "\nsignal handler: %d\n", signum );
}

int appArgs( int *p_argc, char ***p_argv )
{
  return 0;
}

apisSysParams_t *appInit( void )
{
  return (&sysParams);
}

int appMain( apicHandle_t *handles )
{
  struct sigaction new_action, old_action;

  // ZMain uses only one API client
  zmainClientHandle = handles[0];

  if (NULL == zmainClientHandle)
  {
    uiPrintfEx(trFATAL, "appMain> Connection to NPI server failed. "
	"Exiting...\n");
    exit(254);
  }

#if !defined ( LINUX_ZNP )
  // Set the ZMac mode
  ZMacSetPBMode( FALSE );
#endif // !LINUX_ZNP
  // Initialize the signal handler, this will wake up the "poll" on file descriptors
  // for OSAL events
  new_action.sa_handler = signal_handler;
  sigemptyset( &new_action.sa_mask );
  new_action.sa_flags = 0;

  sigaction( SIGUSR1, NULL, &old_action );
  if ( old_action.sa_handler != SIG_IGN )
  {
    sigaction( SIGUSR1, &new_action, NULL );
  }

#if !defined ( LINUX_ZNP )
#if defined ( NV_RESTORE )
  uiPrintf( "\nzmain NV Restore Enabled\n" );
#endif

#if ( SECURE == 1 )
  uiPrintf( "\nzmain Security Enabled\n" );
#endif
#endif // !LINUX_ZNP
  // Do the rest of the initialization
  osal_int_disable( INTS_ALL );

#if !defined ( LINUX_ZNP )
  osal_nv_init( osalNvFileName );
#endif // !LINUX_ZNP
  InitBoard( 0 );

#if !defined ( LINUX_ZNP )
  // Initialize basic NV items
  zgInit();

  // Setup the ieee address
  {
    uint8 extAddr[8];
    int x;
    for ( x = 0; x < 8; x++ )
    {
      extAddr[x] = rand();
    }
    if ( osal_nv_item_init( ZCD_NV_EXTADDR, 8, extAddr ) != NV_ITEM_UNINIT )
    {
      osal_nv_read( ZCD_NV_EXTADDR, 0, 8, extAddr );
    }
    ZMacSetReq( ZMacExtAddr, extAddr );
  }

#if defined ZCL_KEY_ESTABLISH
  // Initialize the Certicom certificate information.
  zmain_cert_init();
#endif
#endif // !LINUX_ZNP
  osal_init_system();
  osal_int_enable( INTS_ALL );

  osal_start_system();  // No Return from here

  exit( 0 );
}

/*********************************************************************
 * @fn      SetPollTimeOut()
 * @brief   set the timeout time in ms for the poll.
 * @param   delay - poll timeout in ms
 * @return  None
 */
void SetPollTimeOut( int delay )
{
  polltimeOut = delay;

  // Is the ZStack thread waiting?
  if ( pollWait )
  {
    // Signal the process to wakeup (come out of poll).
    union sigval sigVal;
    sigVal.sival_int = 0;
    sigqueue( pollPid, SIGUSR1, sigVal );
  }
}

/*********************************************************************
 * @fn      initPoll()
 * @brief   initialise the fd's for the poll.
 * @param   delay - poll timeout in ms
 * @return  None
 */
static void initPoll( void )
{
  int i, numOsalFds;
  int* osal_fds;

  numPollFds = 0;

#if defined ( USE_KEY )
  keyFdIdx = numPollFds++;
#endif

  //Add the number of fd's in osal timers
  numOsalFds = osal_timer_num_active();
  numPollFds += numOsalFds;

  if ( pollFds )
  {
    free( pollFds );
  }

  pollFds = malloc( numPollFds * sizeof(struct pollfd) );
  if ( pollFds == 0 )
  {
    uiPrintf( "zmain error allocation pollFds\n" );
  }

  if ( pollFds )
  {
#ifdef USE_KEY
    pollFds[keyFdIdx].fd = keyFd;
    pollFds[keyFdIdx].events = POLLIN;
    //uiPrintfEx(trUNMASKABLE, "keyFdIdx=%d, fd=%d\n", keyFdIdx, pollFds[keyFdIdx].fd);
#endif

    //add osal timer fd's if there are any
    if ( numOsalFds )
    {
      osal_fds = malloc( numOsalFds * sizeof(int) );

      if ( osal_fds )
      {
        osal_GetTimerFds( osal_fds, numOsalFds );
        //uiPrintfEx(trUNMASKABLE, "numOsalFds=%d\n", numOsalFds);

        for ( i = 0; i < numOsalFds; i++ )
        {
          pollFds[i + (numPollFds - numOsalFds)].fd = osal_fds[i];
          pollFds[i + (numPollFds - numOsalFds)].events = POLLIN;
          //uiPrintfEx(trUNMASKABLE, "osal timer idx:%d fd:%d\n", i+(numPollFds - numOsalFds), pollFds[i+(numPollFds - numOsalFds)].fd) ;
        }

        free( osal_fds );
      }
    }
  }
}

/**************************************************************************************************
 * @fn      Hal_ProcessPoll
 *
 * @brief   This routine will be called by OSAL to poll on UART, TIMER...
 *
 * @param   none
 *
 * @return  none
 **************************************************************************************************/
void Hal_ProcessPoll( void )
{
  int ret;
  sigset_t sigmask;
  sigset_t origmask;

  //fd's are added dynamically by the osal timer
  //so we need to init every time
  initPoll();

  pollPid = getpid();

  // Poll, or wait, for a timer, keypress or osal event( signal)
  //uiPrintfEx(trUNMASKABLE,  "poll++[%d:%d]\n", numPollFds, polltimeOut );
  sigemptyset( &sigmask );
  sigprocmask( SIG_SETMASK, &sigmask, &origmask );
  pollWait = TRUE;
  ret = poll( pollFds, numPollFds, polltimeOut );
  pollWait = FALSE;
  sigprocmask( SIG_SETMASK, &origmask, NULL );
  //uiPrintfEx(trUNMASKABLE,  "poll--\n" );

  //reset the poll timout to default
  SetPollTimeOut( POLL_TIMEOUT );

  if ( ret > 0 )
  {
    int i;
    /* An event on one of the fds has occurred. */

#ifdef USE_KEY
    //check key
    if( (pollFds[keyFdIdx].revents) )
    {
      HalKeyPoll();
    }
    else
#endif
    {
      //If no other event then it must have been a timer
      osalTimerUpdate( 0 );
    }

    for ( i = 0; i < numPollFds; i++ )
    {
      if ( pollFds[i].revents )
      {
        //uiPrintfEx(trUNMASKABLE,  "poll on %d, fd:%d\n", i, pollFds[i].fd );
      }
    }

  }
}

#if !defined ( LINUX_ZNP )
// These 2 dummy functions are called in ZDApp.c
void HalLedBlink( uint8 leds, uint8 cnt, uint8 duty, uint16 time )
{
}
uint8 HalLedSet( uint8 led, uint8 mode )
{
  return (0);
}
#endif // !LINUX_ZNP
