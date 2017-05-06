/*********************************************************************
 Filename:       api_server.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    API Server module

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

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <bits/local_lim.h>
#include <errno.h>

#include "hal_rpc.h"
#include "api_server.h"
#include "trace.h"

/*********************************************************************
 * MACROS
 */
#if !defined( MAX )
#define MAX(a,b)    ((a > b) ? a : b)
#endif

#define NPI_LNX_ERROR_MODULE(a)     ((uint8)((a & 0xFF00) >> 8))
#define NPI_LNX_ERROR_THREAD(a)     ((uint8)(a & 0x00FF))

/*********************************************************************
 * CONSTANTS
 */

#if !defined ( APIS_CONNECTION_QUEUE_SIZE )
#define APIS_CONNECTION_QUEUE_SIZE 4
#endif

#define APIS_LNX_SUCCESS 0
#define APIS_LNX_FAILURE 1

#define APIS_ERROR_SOCKET_GET_ADDRESS_INFO                    -1
#define APIS_ERROR_IPC_SOCKET_SET_REUSE_ADDRESS               -2
#define APIS_ERROR_IPC_SOCKET_BIND                            -3
#define APIS_ERROR_IPC_SOCKET_LISTEN                          -4
#define APIS_ERROR_IPC_SOCKET_SELECT_CHECK_ERRNO              -5
#define APIS_ERROR_IPC_SOCKET_ACCEPT                          -6
#define APIS_ERROR_IPC_ADD_TO_ACTIVE_LIST_NO_ROOM             -7
#define APIS_ERROR_IPC_REMOVE_FROM_ACTIVE_LIST_NOT_FOUND      -8
#define APIS_ERROR_IPC_RECV_DATA_DISCONNECT                   -9
#define APIS_ERROR_IPC_RECV_DATA_CHECK_ERRNO                  -10
#define APIS_ERROR_IPC_RECV_DATA_TOO_FEW_BYTES                -11
#define APIS_ERROR_HAL_GPIO_WAIT_SRDY_CLEAR_POLL_TIMEDOUT     -12
#define APIS_ERROR_IPC_RECV_DATA_INCOMPATIBLE_CMD_TYPE        -13
#define APIS_ERROR_IPC_SEND_DATA_ALL                          -14
#define APIS_ERROR_IPC_SEND_DATA_SPECIFIC_CONNECTION_REMOVED  -15
#define APIS_ERROR_IPC_SEND_DATA_SPECIFIC                     -16
#define APIS_ERROR_MALLOC_FAILURE                             -17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool APIS_initialized = FALSE;
size_t APIS_threadStackSize = (PTHREAD_STACK_MIN * 3);   // 16k*3

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static char strBuffer[128] =
{ 0 };

static int apisErrorCode = 0;

static int listeningPort = 0;

struct addrinfo hints;
struct addrinfo *listenServerInfo;

char *toAPISLnxLog = NULL;

fd_set activeConnectionsFDs;
fd_set activeConnectionsFDsSafeCopy;
int fdmax;

typedef struct _connectioninfo_t
{
  struct _connectioninfo_t *next;
  bool garbage;
  pthread_mutex_t mutex;
  int sock;
} ConnectionInfo_t;

ConnectionInfo_t *activeConnectionList = NULL;
size_t activeConnectionCount = 0;
pthread_mutex_t aclMutex = PTHREAD_MUTEX_INITIALIZER;
size_t aclNoDeleteCount = 0;
pthread_cond_t aclDeleteAllowCond = PTHREAD_COND_INITIALIZER;

struct
{
  int list[APIS_CONNECTION_QUEUE_SIZE];
  int size;
} activeConnections;

// Socket handles
static uint32 listenSocketHndl;

static pthread_t listeningThreadId;

static char* logPath = NULL;

static pfnAPISMsgCB apisMsgCB = NULL;

static apic16BitLenMsgHdr_t apisHdrBuf;

/*********************************************************************
 * LOCAL PROTOTYPES
 */

void *apislisteningThreadFunc( void *ptr );

static int apisSendData( uint16 len, uint8 *pData, int connection );
static int createListeningSocket( void );
static void apiServerExit( int exitCode );
static void startupInfo( void );
static void writetoAPISLnxLog( const char* str );
static int addToActiveList( int c );
static int apisConnectionHandle( int connection );

/*********************************************************************
 * Public Functions
 */

/*********************************************************************
 * APIS_Init - Start and maintain an API Server
 *
 * public API is defined in api_server.h.
 *********************************************************************/
bool APIS_Init( int port, bool verbose, pfnAPISMsgCB pfCB )
{
  pthread_attr_t attr;
  listeningPort = port;
  apisMsgCB = pfCB;

  if ( verbose )
  {
    startupInfo();
  }

  // prepare thread creation
  if ( pthread_attr_init( &attr ) )
  {
    perror( "pthread_attr_init" );
    return (FALSE);
  }

  if ( pthread_attr_setstacksize( &attr, APIS_threadStackSize ) )
  {
    perror( "pthread_attr_setstacksize" );
    return (FALSE);
  }

  // Create a listening socket
  if ( createListeningSocket() < 0 )
  {
    return (FALSE);
  }

  if ( verbose )
  {
    uiPrintf( "waiting for first connection on #%d...\n", listenSocketHndl );
  }

  // Create thread for listening
  if ( pthread_create( &listeningThreadId, &attr, apislisteningThreadFunc,
      NULL ) )
  {
    // thread creation failed
    uiPrintf( "Failed to create an API Server Listening thread\n" );
    return (FALSE);
  }

  APIS_initialized = TRUE;

  return (TRUE);
}

/*********************************************************************
 * APIS_SendData - Send a packet to a connection
 *
 * public API is defined in api_server.h.
 *********************************************************************/
void APIS_SendData( int connection, uint8 sysID, bool areq, uint8 cmdId,
                    uint16 len, uint8 *pData )
{
  size_t msglen = len;
  apic16BitLenMsgHdr_t *pMsg;

  if ( len > (uint16)( len + sizeof(apic16BitLenMsgHdr_t) ) )
  {
    uiPrintf( "[ERR] APIS_SendData() called with length exceeding max allowed" );
    return;
  }
  msglen += sizeof(apic16BitLenMsgHdr_t);

  pMsg = malloc( msglen );
  if ( !pMsg )
  {
    // TODO: abort
    uiPrintf( "[ERR] APIS_SendData() failed to allocate memory block" );
    return;
  }

  pMsg->lenL = (uint8) len;
  pMsg->lenH = (uint8)( len >> 8 );
  if ( areq )
  {
    pMsg->subSys = (uint8)( RPC_CMD_AREQ | sysID );
  }
  else
  {
    pMsg->subSys = (uint8)( RPC_CMD_SRSP | sysID );
  }
  pMsg->cmdId = cmdId;

  memcpy( pMsg + 1, pData, len );

  apisSendData( (len + sizeof(apic16BitLenMsgHdr_t)), (uint8 *) pMsg,
      connection );

  free( pMsg );
}

/*********************************************************************
 * Local Functions
 */

/*********************************************************************
 * @fn      reserveActiveConnection
 *
 * @brief   Reserve an active connection so that active connection
 *          list does not change and also the send() to the active
 *          connection is performed in sequence.
 *
 * @param   connection - connection
 *
 * @return  0 when successful. -1 when failed.
 *
 *********************************************************************/
static int reserveActiveConnection( int connection )
{
  ConnectionInfo_t *entry;

  if ( pthread_mutex_lock( &aclMutex ) != 0 )
  {
    perror( "pthread_mutex_lock" );
    exit( 1 );
  }
  entry = activeConnectionList;
  while ( entry )
  {
    if ( entry->sock == connection )
    {
      aclNoDeleteCount++;
      pthread_mutex_unlock( &aclMutex );
      if ( pthread_mutex_lock( &entry->mutex ) != 0 )
      {
        perror( "pthread_mutex_lock" );
        exit( 1 );
      }
      return 0;
    }
    entry = entry->next;
  }
  pthread_mutex_unlock( &aclMutex );
  return -1;
}

/*********************************************************************
 * @fn      unreserveActiveConnection
 *
 * @brief   Free a reserved active connection.
 *
 * @param   connection - connection
 *
 * @return  None
 *
 *********************************************************************/
static void unreserveActiveConnection( int connection )
{
  ConnectionInfo_t *entry;

  if ( pthread_mutex_lock( &aclMutex ) != 0 )
  {
    perror( "pthread_mutex_lock" );
    exit( 1 );
  }
  entry = activeConnectionList;
  while ( entry )
  {
    if ( entry->sock == connection )
    {
      pthread_mutex_unlock( &entry->mutex );
      if ( --aclNoDeleteCount == 0 )
      {
        pthread_cond_signal( &aclDeleteAllowCond );
      }
      break;
    }
    entry = entry->next;
  }
  pthread_mutex_unlock( &aclMutex );
}

/*********************************************************************
 * @fn      dropActiveConnection
 *
 * @brief   Mark an active connection as a garbage
 *
 * @param   connection - connection
 *
 * @return  None
 *
 *********************************************************************/
static void dropActiveConnection( int connection )
{
  ConnectionInfo_t *entry;

  if ( pthread_mutex_lock( &aclMutex ) != 0 )
  {
    perror( "pthread_mutex_lock" );
    exit( 1 );
  }
  entry = activeConnectionList;
  while ( entry )
  {
    if ( entry->sock == connection )
    {
      entry->garbage = TRUE;
      break;
    }
    entry = entry->next;
  }
  pthread_mutex_unlock( &aclMutex );
}

/*********************************************************************
 * @fn      aclGarbageCollect
 *
 * @brief   Garbage collect connections from active connection list
 *
 * @param   None
 *
 * @return  None
 *
 *********************************************************************/
static void aclGarbageCollect( void )
{
  ConnectionInfo_t **entryPtr;
  ConnectionInfo_t *deletedlist = NULL;

  if ( pthread_mutex_lock( &aclMutex ) != 0 )
  {
    perror( "pthread_mutex_lock" );
    exit( 1 );
  }

  for ( ;; )
  {
    if ( !aclNoDeleteCount )
    {
      break;
    }
    pthread_cond_wait( &aclDeleteAllowCond, &aclMutex );
  }

  entryPtr = &activeConnectionList;
  while ( *entryPtr )
  {
    if ( (*entryPtr)->garbage )
    {
      ConnectionInfo_t *deleted = *entryPtr;
      *entryPtr = deleted->next;
      close( deleted->sock );
      FD_CLR( deleted->sock, &activeConnectionsFDs );
      pthread_mutex_destroy( &deleted->mutex );
      deleted->next = deletedlist;
      deletedlist = deleted;
      continue;
    }
    entryPtr = &(*entryPtr)->next;
  }
  pthread_mutex_unlock( &aclMutex );

  // Indicate connection is done
  while ( deletedlist )
  {
    ConnectionInfo_t *deleted = deletedlist;
    deletedlist = deletedlist->next;

    if ( apisMsgCB )
    {
      apisMsgCB( deleted->sock, 0, 0, 0xFFFFu, NULL, SERVER_DISCONNECT );
    }
    free( deleted );
    activeConnectionCount--;
  }
}

/*********************************************************************
 * @fn      apisSendData
 *
 * @brief   Send Packet
 *
 * @param   len - length to send
 * @param   pData - pointer to buffer to send
 * @param   connection - connection to send message (for synchronous
 *                       response) otherwise -1 for all connections
 *
 * @return  status
 *
 *********************************************************************/
static int apisSendData( uint16 len, uint8 *pData, int connection )
{
  int bytesSent = 0;
  int ret = APIS_LNX_SUCCESS;

  // Send to all connections?
  if ( connection < 0 )
  {
    // retain the list
    ConnectionInfo_t *entry;

    if ( pthread_mutex_lock( &aclMutex ) != 0 )
    {
      perror( "pthread_mutex_lock" );
      exit( 1 );
    }

    entry = activeConnectionList;

    // Send data to all connections, except listener
    while ( entry )
    {
      if ( entry->sock != listenSocketHndl )
      {
        // Repeat till all data is sent out
        if ( pthread_mutex_lock( &entry->mutex ) != 0 )
        {
          perror( "pthread_mutex_lock" );
          exit( 1 );
        }

        for ( ;; )
        {
          bytesSent = send( entry->sock, pData, len, MSG_NOSIGNAL );
          if ( bytesSent < 0 )
          {
            if ( errno != ENOTSOCK )
            {
              char *errorStr = (char *) malloc( 30 );
              sprintf( errorStr, "send %d, %d", entry->sock, errno );
              perror( errorStr );
              // Remove from list if detected bad file descriptor
              if ( errno == EBADF )
              {
                uiPrintf( "Removing connection #%d\n", entry->sock );
                entry->garbage = TRUE;
              }
              else
              {
                apisErrorCode = APIS_ERROR_IPC_SEND_DATA_ALL;
                ret = APIS_LNX_FAILURE;
              }
            }
          }
          else if ( bytesSent < len )
          {
            pData += bytesSent;
            len -= bytesSent;
            continue;
          }
          break;
        }
        pthread_mutex_unlock( &entry->mutex );
      }
      entry = entry->next;
    }
    pthread_mutex_unlock( &aclMutex );
  }
  else
  {
    // Protect thread against asynchronous deletion
    if ( reserveActiveConnection( connection ) == 0 )
    {

      // Repeat till all data is sent
      for ( ;; )
      {
        // Send to specific connection only
        bytesSent = send( connection, pData, len, MSG_NOSIGNAL );

        uiPrintfEx(trINFO, "...sent %d bytes to Client\n", bytesSent );

        if ( bytesSent < 0 )
        {
          perror( "send" );
          // Remove from list if detected bad file descriptor
          if ( errno == EBADF )
          {
            uiPrintf( "Removing connection #%d\n", connection );
            dropActiveConnection( connection );
            // Connection closed. Remove from set
            apisErrorCode =
                APIS_ERROR_IPC_SEND_DATA_SPECIFIC_CONNECTION_REMOVED;
            ret = APIS_LNX_FAILURE;
          }
        }
        else if ( bytesSent < len )
        {
          pData += bytesSent;
          len -= bytesSent;
          continue;
        }
        break;
      }
      unreserveActiveConnection( connection );
    }
  }

  return (ret);
}

/*********************************************************************
 * @fn      createListeningSocket
 *
 * @brief   Create an API Server Listening socket
 *
 * @param   ptr -
 *
 * @return  none
 *
 *********************************************************************/
static int createListeningSocket( void )
{
  int yes = 1;

  apisErrorCode = 0;

  listenSocketHndl = socket( listenServerInfo->ai_family,
      listenServerInfo->ai_socktype, listenServerInfo->ai_protocol );

  // avoid "Address already in use" error message
  if ( setsockopt( listenSocketHndl, SOL_SOCKET, SO_REUSEADDR, &yes,
      sizeof(int) ) == -1 )
  {
    perror( "setsockopt" );
    apisErrorCode = APIS_ERROR_IPC_SOCKET_SET_REUSE_ADDRESS;
    return (apisErrorCode);
  }

  // Bind socket
  if ( bind( listenSocketHndl, listenServerInfo->ai_addr,
      listenServerInfo->ai_addrlen ) == -1 )
  {
    perror( "bind" );
    apisErrorCode = APIS_ERROR_IPC_SOCKET_BIND;
    return (apisErrorCode);
  }

  // Listen, allow 4 connections in the queue
  if ( listen( listenSocketHndl, APIS_CONNECTION_QUEUE_SIZE ) == -1 )
  {
    perror( "listen" );
    apisErrorCode = APIS_ERROR_IPC_SOCKET_LISTEN;
    return (apisErrorCode);
  }

  toAPISLnxLog = (char *) malloc( AP_MAX_BUF_LEN );

  // Clear file descriptor sets
  FD_ZERO( &activeConnectionsFDs );
  FD_ZERO( &activeConnectionsFDsSafeCopy );

  // Add the listener to the set
  FD_SET( listenSocketHndl, &activeConnectionsFDs );
  fdmax = listenSocketHndl;

  return (apisErrorCode);
}

/*********************************************************************
 * @fn      apislisteningThreadFunc
 *
 * @brief   API Server listening thread
 *
 * @param   ptr -
 *
 * @return  none
 *
 *********************************************************************/
void *apislisteningThreadFunc( void *ptr )
{
  bool done = FALSE;
  int ret;
  int justConnected;
  int c;
  struct sockaddr_storage their_addr;
  struct timeval *pTimeout = NULL;

  trace_init_thread("LSTN");

  memset( &their_addr, 0, sizeof(their_addr) );
  //
  do
  {
    activeConnectionsFDsSafeCopy = activeConnectionsFDs;

    // First use select to find activity on the sockets
    if ( select( fdmax + 1, &activeConnectionsFDsSafeCopy, NULL, NULL,
        pTimeout ) == -1 )
    {
      if ( errno != EINTR )
      {
        perror( "select" );
        apisErrorCode = APIS_ERROR_IPC_SOCKET_SELECT_CHECK_ERRNO;
        ret = APIS_LNX_FAILURE;
        break;
      }
      continue;
    }

    // Then process this activity
    for ( c = 0; c <= fdmax; c++ )
    {
      if ( FD_ISSET( c, &activeConnectionsFDsSafeCopy ) )
      {
        if ( c == listenSocketHndl )
        {
          int addrLen = 0;

          // Accept a connection from a client.
          addrLen = sizeof(their_addr);
          justConnected = accept( listenSocketHndl,
              (struct sockaddr *) &their_addr, (socklen_t *) &addrLen );

          if ( justConnected == -1 )
          {
            perror( "accept" );
            apisErrorCode = APIS_ERROR_IPC_SOCKET_ACCEPT;
            ret = APIS_LNX_FAILURE;
            break;
          }
          else
          {
            char ipstr[INET6_ADDRSTRLEN];
            char ipstr2[INET6_ADDRSTRLEN];

            ret = addToActiveList( justConnected );
            if ( ret != APIS_LNX_SUCCESS )
            {
              // Adding to the active connection list failed.
              // Close the accepted connection.
              close( justConnected );
            }
            else
            {

              FD_SET( justConnected, &activeConnectionsFDs );
              if ( justConnected > fdmax )
              {
                fdmax = justConnected;
              }

              //                                            debug_
              inet_ntop( AF_INET,
                  &((struct sockaddr_in *) &their_addr)->sin_addr, ipstr,
                  sizeof ipstr );
              inet_ntop( AF_INET6,
                  &((struct sockaddr_in6 *) &their_addr)->sin6_addr, ipstr2,
                  sizeof ipstr2 );
              sprintf( toAPISLnxLog, "Connected to #%d.(%s / %s)",
                  justConnected, ipstr, ipstr2);
              uiPrintf( "%s\n", toAPISLnxLog );
              writetoAPISLnxLog( toAPISLnxLog );

              apisMsgCB( justConnected, 0, 0, 0, NULL, SERVER_CONNECT );
            }
          }
        }
        else
        {
          ret = apisConnectionHandle( c );
          if ( ret == APIS_LNX_SUCCESS )
          {
            // Everything is ok
          }
          else
          {
            uint8 childThread;
            switch ( apisErrorCode )
            {
              case APIS_ERROR_IPC_RECV_DATA_DISCONNECT:
                close( c );
                uiPrintf( "Removing connection #%d\n", c );

                // Connection closed. Remove from set
                FD_CLR( c, &activeConnectionsFDs );

                // We should now set ret to APIS_LNX_SUCCESS, but there is still one fatal error
                // possibility so simply set ret = to return value from removeFromActiveList().
                dropActiveConnection( c );

                sprintf( toAPISLnxLog, "\t\tRemoved connection #%d", c );
                writetoAPISLnxLog( toAPISLnxLog );
                break;

              default:
                if ( apisErrorCode == APIS_LNX_SUCCESS )
                {
                  // Do not report and abort if there is no real error.
                  ret = APIS_LNX_SUCCESS;
                }
                else
                {
                  //              debug_
                  uiPrintf( "[ERR] apisErrorCode 0x%.8X\n", apisErrorCode );

                  // Everything about the error can be found in the message, and in npi_ipc_errno:
                  childThread = apisHdrBuf.cmdId;

                  sprintf( toAPISLnxLog, "Child thread with ID %d"
                      " in module %d reported error",
                      NPI_LNX_ERROR_THREAD( childThread ),
                      NPI_LNX_ERROR_MODULE( childThread ));
                  writetoAPISLnxLog( toAPISLnxLog );
                }
                break;
            }
          }
        }
      }
    }

    // Collect garbages
    // Note that for thread-safety garbage connections are collected
    // only from this single thread
    aclGarbageCollect();

  } while ( !done );

  return (ptr);
}

/*********************************************************************
 * @fn      startupInfo
 *
 * @brief   Print/Display the connection information
 *
 * @param   none
 *
 * @return  none
 *
 *********************************************************************/
static void startupInfo( void )
{
//  struct sockaddr_storage their_addr;
  int status;

  // Check initialization and parameters

  memset( &hints, 0, sizeof(hints) );
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  uiPrintf( "Port: %d\n", listeningPort );
  sprintf( strBuffer, "%d", listeningPort );

  if ( (status = getaddrinfo( NULL, strBuffer, &hints, &listenServerInfo ))
      != 0 )
  {
    uiPrintfEx(trERROR, "getaddrinfo error: %s\n", gai_strerror( status ) );

    listeningPort = API_SERVER_DEFAULT_PORT;
    sprintf( strBuffer, "%d", listeningPort );
    uiPrintf( "Trying default port: %d instead\n", listeningPort );
    if ( (status = getaddrinfo( NULL, strBuffer, &hints, &listenServerInfo ))
        != 0 )
    {
      uiPrintfEx(trERROR, "getaddrinfo error: %s\n", gai_strerror( status ) );
      apisErrorCode = APIS_ERROR_SOCKET_GET_ADDRESS_INFO;
      apiServerExit( APIS_LNX_FAILURE );
    }
  }

  uiPrintf( "Following IP addresses are available:\n\n" );
  {
    struct ifaddrs * ifAddrStruct = NULL;
    struct ifaddrs * ifa = NULL;
    void * tmpAddrPtr = NULL;

    getifaddrs( &ifAddrStruct );

    for ( ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next )
    {
      if ( ifa->ifa_addr->sa_family == AF_INET )
      { // check it is IP4
        char addressBuffer[INET_ADDRSTRLEN];
        // is a valid IP4 Address
        tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
        inet_ntop( AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN );
        uiPrintf( " IPv4: interface: %s\t IP Address %s\n", ifa->ifa_name,
            addressBuffer );
      }
      else if ( ifa->ifa_addr->sa_family == AF_INET6 )
      { // check it is IP6
        char addressBuffer[INET6_ADDRSTRLEN];
        // is a valid IP6 Address
        tmpAddrPtr = &((struct sockaddr_in6 *) ifa->ifa_addr)->sin6_addr;
        inet_ntop( AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN );
        uiPrintf( " IPv6: interface: %s\t IP Address %s\n", ifa->ifa_name,
            addressBuffer );
      }
    }

    if ( ifAddrStruct != NULL )
    {
      freeifaddrs( ifAddrStruct );
    }
  }

#ifdef __APP_UI__
  uiPrintf( "The socket will listen on the following IP addresses:\n\n" );
  {
    struct addrinfo *p;
    char ipstr[INET6_ADDRSTRLEN];

    for ( p = listenServerInfo; p != NULL; p = p->ai_next )
    {
      void *addr;
      char *ipver;

      // get the pointer to the address itself,
      // different fields in IPv4 and IPv6:
      if ( p->ai_family == AF_INET )
      { // IPv4
        struct sockaddr_in *ipv4 = (struct sockaddr_in *) p->ai_addr;
        addr = &(ipv4->sin_addr);
        ipver = "IPv4";
      }
      else
      { // IPv6
        struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *) p->ai_addr;
        addr = &(ipv6->sin6_addr);
        ipver = "IPv6";
      }

      // convert the IP to a string and print it:
      inet_ntop( p->ai_family, addr, ipstr, sizeof ipstr );
      uiPrintf( "  %s: %s\n", ipver, ipstr );
    }
  }
  uiPrintf( "0.0.0.0 means it will listen to all available IP address\n\n" );
#endif //__APP_UI__

}

/*********************************************************************
 * @fn      apiServerExit
 *
 * @brief   Exit the API Server
 *
 * @param   exitCode - reason for exit
 *
 * @return  none
 *
 *********************************************************************/
static void apiServerExit( int exitCode )
{
  // doesn't work yet, TBD
  //exit ( apisErrorCode );
}

/*********************************************************************
 * @fn      writetoAPISLnxLog
 *
 * @brief   Write a log file entry
 *
 * @param   str - String to write
 *
 * @return  none
 *
 *********************************************************************/
static void writetoAPISLnxLog( const char* str )
{
  int APISLnxLogFd, i = 0;
  char *fullStr;
  char *inStr;

  time_t timeNow;
  struct tm * timeNowinfo;

  if ( logPath )
  {
    fullStr = (char *) malloc( 255 );
    inStr = (char *) malloc( 255 );

    time( &timeNow );
    timeNowinfo = localtime( &timeNow );

    sprintf( fullStr, "[%s", asctime( timeNowinfo ) );
    sprintf( inStr, "%s", str );

    // Remove \n characters
    fullStr[strlen( fullStr ) - 2] = 0;
    for ( i = strlen( str ) - 1; i > MAX( strlen( str ), 4 ); i-- )
    {
      if ( inStr[i] == '\n' )
      {
        inStr[i] = 0;
      }
    }
    sprintf( fullStr, "%s] %s", fullStr, inStr );

    // Add global error code
    sprintf( fullStr, "%s. Error: %.8X\n", fullStr, apisErrorCode );

    // Write error message to /dev/SSLnxLog
    APISLnxLogFd = open( logPath, (O_WRONLY | O_APPEND | O_CREAT), S_IRWXU );
    if ( APISLnxLogFd > 0 )
    {
      write( APISLnxLogFd, fullStr, strlen( fullStr ) + 1 );
    }
    else
    {
      uiPrintf( "Could not write \n%s\n to %s. Error: %.8X\n", str, logPath,
          errno );
      perror( "open" );
    }

    close( APISLnxLogFd );
    free( fullStr );
    free( inStr );
  }
}

/*********************************************************************
 * @fn      addToActiveList
 *
 * @brief  Add a connection to the active list
 *
 * @param   c - connection handle
 *
 * @return  APIS_LNX_SUCCESS or APIS_LNX_FAILURE
 *
 *********************************************************************/
static int addToActiveList( int c )
{
  if ( pthread_mutex_lock( &aclMutex ) != 0 )
  {
    perror( "pthread_mutex_lock" );
    exit( 1 );
  }
  if ( activeConnectionCount <= APIS_CONNECTION_QUEUE_SIZE )
  {
    // Entry at position activeConnections.size is always the last available entry
    ConnectionInfo_t *newinfo = malloc( sizeof(ConnectionInfo_t) );
    newinfo->next = activeConnectionList;
    newinfo->garbage = FALSE;
    pthread_mutex_init( &newinfo->mutex, NULL );
    newinfo->sock = c;
    activeConnectionList = newinfo;

    // Increment size
    activeConnectionCount++;

    pthread_mutex_unlock( &aclMutex );

    return APIS_LNX_SUCCESS;
  }
  else
  {
    pthread_mutex_unlock( &aclMutex );

    // There's no more room in the list
    apisErrorCode = APIS_ERROR_IPC_ADD_TO_ACTIVE_LIST_NO_ROOM;

    return (APIS_LNX_FAILURE);
  }
}

/*********************************************************************
 * @fn      apisConnectionHandle
 *
 * @brief   Handle receiving a message
 *
 * @param   c - connection handle to remove
 *
 * @return  APIS_LNX_SUCCESS or APIS_LNX_FAILURE
 *
 *********************************************************************/
static int apisConnectionHandle( int connection )
{
  int n;
  int ret = APIS_LNX_SUCCESS;

// Handle the connection
  uiPrintfEx(trINFO, "Receive message...\n" );

// Receive only NPI header first. Then then number of bytes indicated by length.
  n = recv( connection, &apisHdrBuf, sizeof(apisHdrBuf), MSG_WAITALL );
  if ( n <= 0 )
  {
    if ( n < 0 )
    {
      perror( "recv" );
      if ( errno == ENOTSOCK )
      {
        uiPrintfEx(trDEBUG, "[ERROR] Tried to read #%d as socket\n", connection );
        apisErrorCode = APIS_ERROR_IPC_RECV_DATA_DISCONNECT;
        ret = APIS_LNX_FAILURE;
      }
      else
      {
        apisErrorCode = APIS_ERROR_IPC_RECV_DATA_CHECK_ERRNO;
        ret = APIS_LNX_FAILURE;
      }
    }
    else
    {
      uiPrintfEx(trINFO, "Will disconnect #%d\n", connection );
      apisErrorCode = APIS_ERROR_IPC_RECV_DATA_DISCONNECT;
      ret = APIS_LNX_FAILURE;
    }
  }
  else if ( n == sizeof(apisHdrBuf) )
  {
    uint8 *buf = NULL;
    uint16 len = apisHdrBuf.lenL;
    len |= (uint16) apisHdrBuf.lenH << 8;

    // Now read out the payload of the NPI message, if it exists
    if ( len > 0 )
    {
      buf = malloc( len );

      if ( !buf )
      {
        uiPrintf( "[ERR] APIS receive thread memory allocation failure" );
        apisErrorCode = APIS_ERROR_MALLOC_FAILURE;
        return APIS_LNX_FAILURE;
      }

      n = recv( connection, buf, len, MSG_WAITALL );
      if ( n != len )
      {
        uiPrintf( "[ERR] Could not read out the NPI payload."
            " Requested %d, but read %d!\n", len, n );

        apisErrorCode = APIS_ERROR_IPC_RECV_DATA_TOO_FEW_BYTES;
        ret = APIS_LNX_FAILURE;

        if ( n < 0 )
        {
          perror( "recv" );
          // Disconnect this
          apisErrorCode = APIS_ERROR_IPC_RECV_DATA_DISCONNECT;
          ret = APIS_LNX_FAILURE;
        }
      }
      else
      {
        ret = APIS_LNX_SUCCESS;
      }
    }

    /*
     * Take the message from the client and pass it to be processed
     */
    if ( ret == APIS_LNX_SUCCESS )
    {
      if ( len != 0xFFFF )
      {
        if ( apisMsgCB )
        {
          apisMsgCB( connection, apisHdrBuf.subSys, apisHdrBuf.cmdId, len, buf,
              SERVER_DATA );
        }
      }
      else
      {
        // len == 0xFFFF is used for connection removal
        // hence is not supported.
        uiPrintf( "[WARN] APIS received message"
            " with size 0xFFFFu was discarded silently\n" );
      }
    }

    if ( buf )
    {
      free( buf );
    }
  }

  if ( (ret == APIS_LNX_FAILURE)
      && (apisErrorCode == APIS_ERROR_IPC_RECV_DATA_DISCONNECT) )
  {
    uiPrintfEx(trINFO, "Done with %d\n", connection );
  }
  else
  {
    uiPrintfEx(trINFO, "!Done\n" );
  }

  return ret;
}

/*********************************************************************
 *********************************************************************/
