/**************************************************************************************************
 Filename:       main.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:    Main loop and initialization


 Copyright 2013 -2014 Texas Instruments Incorporated. All rights reserved.

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
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <time.h>

#include "hal_types.h"
#include "api_client.h"
#include "hal_rpc.h"
#include "api_server.h"
#include "configparser.h"
#include "trace.h"

/**************************************************************************************************
 * Constant
 **************************************************************************************************/

#if defined ( NPI_VERSION )
#define DISPLAY_VERSION  TRUE     // Yes, display NPI version information
#else
#define DISPLAY_VERSION  FALSE    // Don't display the NPI Server's version
#endif

#define CALL_STACK_TRACE_DEPTH 10

/**************************************************************************************************
 * Typedefs
 **************************************************************************************************/

/**************************************************************************************************
 * Globals
 **************************************************************************************************/
static apisSysParams_t *pAPIS_SysParams = NULL;

/**************************************************************************************************
 * Locals
 **************************************************************************************************/

/**************************************************************************************************
 * Function Prototypes
 **************************************************************************************************/
void segmentation_fault_handler( int signum, siginfo_t *info, void *context );
void register_segmentation_fault_handler( void );
void unregister_segmentation_fault_handler( void );
int processUint32Flag(const char * flagLong, const char * flagShort, uint32_t * value, int * argc_p, char *argv[]);

/**************************************************************************************************
 **************************************************************************************************/

int main( int argc, char *argv[] )
{
  int exitCode = 0;
  
  processUint32Flag("--verbose", "-v", &default_trace_enable_mask, &argc, argv);

  trace_init_main("MAIN");
  
  register_segmentation_fault_handler();

  // Seed the random number generator with the current time
  srand( time( NULL ) );

  appArgs( &argc, &argv );

  // Parse the command line parameters
  if ( argc == 1 )
  {
    uiPrintf( "\nUsage:\n\n" );uiPrintf( "  %s ipaddr:port config.ini\n\n", argv[0] );
    uiPrintf( "Example:\n\n" );uiPrintf( "  $ %s 192.168.0.1:2531 config.ini\n\n", argv[0] );
    exit( -1 );
  }

  pAPIS_SysParams = appInit();
  if ( pAPIS_SysParams == NULL )
  {
    uiPrintf( "\nApp initialization problem - exiting!!\n\n" );
    exit( -1 );
  }

  if ( argc > (pAPIS_SysParams->numClients + 1) )
  {
    if ( pAPIS_SysParams->pConfigDesc )
    {
      if ( parseConfigFile( (char *) argv[(pAPIS_SysParams->numClients + 1)],
          pAPIS_SysParams->pConfigDesc, pAPIS_SysParams->numConfigDescs ) != 0 )
      {
        uiPrintf( "\nConfig File Error (%s) - exiting!!\n\n", argv[(pAPIS_SysParams->numClients+1)] );
        exit( -1 );
      }
    }
  }

  // Setup the API Client interface
  apicHandle_t handles[pAPIS_SysParams->numClients];
  size_t i;

  for ( i = 0; i < pAPIS_SysParams->numClients; i++ )
  {
    handles[i] = apicInit( argv[1 + i], // the IP address and port to a TCP Server
        DISPLAY_VERSION, pAPIS_SysParams->pfNPICB );
  }

  // Setup the API Server
  APIS_Init( pAPIS_SysParams->port, pAPIS_SysParams->serverVerbose,
      pAPIS_SysParams->pfServerCB );

  // Pass control to the application
  exitCode = appMain( handles );  // No Return from here

  unregister_segmentation_fault_handler();

  exit( exitCode );
}

void segmentation_fault_handler( int signum, siginfo_t *info, void *context )
{
  void *array[CALL_STACK_TRACE_DEPTH];
  size_t size;

  uiPrintfEx(trERROR, "ERROR: signal %d was trigerred:\n", signum );

  uiPrintfEx(trERROR, "  Fault address: %p\n", info->si_addr );

  switch ( info->si_code )
  {
    case SEGV_MAPERR:
      uiPrintfEx(trERROR, "  Fault reason: address not mapped to object\n" );
      break;

    case SEGV_ACCERR:
      uiPrintfEx(trERROR,
          "  Fault reason: invalid permissions for mapped object\n" );
      break;

    default:
      uiPrintfEx(trERROR, "  Fault reason: %d (this value is unexpected).\n",
          info->si_code );
      break;
  }

  // get pointers for the stack entries
  size = backtrace( array, CALL_STACK_TRACE_DEPTH );

  if ( size == 0 )
  {
    uiPrintfEx(trERROR, "Stack trace unavailable\n" );
  }
  else
  {
    uiPrintfEx(trERROR, "Stack trace folows%s:\n",
        (size < CALL_STACK_TRACE_DEPTH) ? "" : " (partial)" );

    // print out the stack frames symbols to stderr
    backtrace_symbols_fd( array, size, STDERR_FILENO );
  }

  /* unregister this handler and let the default action execute */
  uiPrintfEx(trERROR, "Executing original handler...\n" );
  unregister_segmentation_fault_handler();
}

void register_segmentation_fault_handler( void )
{
  struct sigaction action;

  action.sa_flags = SA_SIGINFO;
  action.sa_sigaction = segmentation_fault_handler;
  memset( &action.sa_mask, 0, sizeof(action.sa_mask) );
  action.sa_restorer = NULL;

  if ( sigaction( SIGSEGV, &action, NULL ) < 0 )
  {
    perror( "sigaction" );
  }
}

void unregister_segmentation_fault_handler( void )
{
  struct sigaction action;

  action.sa_flags = 0;
  action.sa_handler = SIG_DFL;
  memset( &action.sa_mask, 0, sizeof(action.sa_mask) );
  action.sa_restorer = NULL;

  sigaction( SIGSEGV, &action, NULL );
}

// look for flagLong or flagShort following by a numeric value. If more than one occurance exist, the rightmost takse effect. Any instance of this flag found is removed from the argument list, together with its value. Return value is the number of times this flag was found.
int processUint32Flag(const char * flagLong, const char * flagShort, uint32_t * value, int * argc_p, char *argv[])
{
  int i;
  int new_i;
  int flagCount = 0;
  char * endptr;

  for (i = 1; i < *argc_p; i++)
  {
  	new_i = i + (2 * flagCount);
	
    argv[i] = argv[new_i];

    if ((strcmp(argv[i] , flagLong) == 0) || (strcmp(argv[i] , flagShort) == 0))
    {
      if ((new_i + 1) >= *argc_p)
      {
        uiPrintf("ERROR: %s was specified without a value\n. Aborting now.", argv[i]);
        exit(-1);
      }

      *value = strtoul(argv[new_i + 1], &endptr, 0);

      if (*endptr != '\0')
      {
        uiPrintf("ERROR: Value following %s has invalid character(s): \"%s\"\n. Aborting now.", argv[i], endptr);
        exit(-1);
      }

      (*argc_p) -= 2;

      flagCount++;

      i--;
    }
  }

  return flagCount;
}
