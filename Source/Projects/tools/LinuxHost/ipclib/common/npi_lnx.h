/**************************************************************************************************
  Filename:       npi_lnx.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file contains the Network Processor Interface (NPI),
                  which abstracts the physical link between the Application
                  Processor (AP) and the Network Processor (NP). The NPI
                  serves as the HAL's client for the SPI and UART drivers, and
                  provides API and callback services for its client.

  Copyright (C) {2012} Texas Instruments Incorporated - http://www.ti.com/


   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

     Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the
     distribution.

     Neither the name of Texas Instruments Incorporated nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#ifndef NPI_H
#define NPI_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
//#include "hal_board.h"
#include "hal_rpc.h"

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/

#define AP_MAX_BUF_LEN 255

#define NPI_PORT			"2533"

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/

// Trace hook function prototype
typedef void (*npi_tracehook_t)(uint8 subsystem, uint8 cmd, uint8 *data, uint8 len);

// ensure correct padding rule for MSVC compiler
#ifdef _MSC_VER
# pragma pack(1)
#endif

#ifndef ATTR_PACKED
# ifdef __GNUC__
#  define ATTR_PACKED __attribute__ ((__packed__))
# else
#  define ATTR_PACKED
# endif
#endif

// NPI API and NPI Callback Message structure
// NOTE: Fields are position dependent. Do not rearrange!
typedef struct ATTR_PACKED
{
	uint8 len;
	uint8 subSys;
	uint8 cmdId;
	uint8 pData[AP_MAX_BUF_LEN];
} npiMsgData_t;

#ifdef _MSC_VER
# pragma pack()
#endif

/**************************************************************************************************
 * GLOBALS
 **************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////
// globals

// trace hook function pointers
// user can update the trace hook function pointer to user's own trace hook
// to monitor all incoming and outgoing NPI packets
extern npi_tracehook_t npi_tracehook_rx; // incoming packet trace hook
extern npi_tracehook_t npi_tracehook_tx; // outgoing packet trace hook

// NPI task event flags
extern volatile uint16 npiEventFlags;

/*********************************************************************
 * FUNCTIONS
 */

// NPI OSAL related functions
extern void NPI_Init( void );
extern uint16 NPI_ProcessEvent( void );

/* NPI server API */
void NPI_ReadVersionReq( uint8 *pValue );
void NPI_ReadParamReq( uint8 paramId, uint8 len, uint8 *pValue );

//
// Network Processor Interface APIs
//

/******************************************************************************
 * @fn         NPI_OpenDevice
 *
 * @brief      This function establishes a serial communication connection with
 *             a network processor device.
 *             As windows machine does not have a single dedicated serial
 *             interface, this function will designate which serial port shall
 *             be used for communication.
 *
 * input parameters
 *
 * @param      portName � name of the serial port
 * @param		 pCfg	- pointer to configuration parameters
 *
 * output parameters
 *
 * None.
 *
 * @return     TRUE if the connection is established successfully.
 *             FALSE, otherwise.
 ******************************************************************************
 */
typedef int (*pNPI_OpenDeviceFn) (const char *portName, void *pCfg);
extern const pNPI_OpenDeviceFn NPI_OpenDeviceFnArr[];

/******************************************************************************
 * @fn         NPI_CloseDevice
 *
 * @brief      This function closes connection with a network processor device
 *
 * input parameters
 *
 * @param      pDevice   - pointer to a device data structure
 *
 * output parameters
 *
 * None.
 *
 * @return     None
 ******************************************************************************
 */
typedef void (*pNPI_CloseDeviceFn) (void);
extern const pNPI_CloseDeviceFn NPI_CloseDeviceFnArr[];

/**************************************************************************************************
 * @fn          NPI_SendAsynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent asynchronously. This routine allocates an AREQ buffer,
 *              copies the client's payload, and sets up the send.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent asynchronously (i.e. AREQ).
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
typedef int (*pNPI_SendAsynchDataFn) ( npiMsgData_t *pMsg );
extern const pNPI_SendAsynchDataFn NPI_SendAsynchDataFnArr[];


/**************************************************************************************************
 * @fn          NPI_SendSynchData
 *
 * @brief       This function is called by the client when it has data ready to
 *              be sent synchronously. This routine allocates a SREQ buffer,
 *              copies the client's payload, sends the data, and waits for the
 *              reply. The input buffer is used for the output data.
 *
 * input parameters
 *
 * @param *pMsg  - Pointer to data to be sent synchronously (i.e. the SREQ).
 *
 * output parameters
 *
 * @param *pMsg  - Pointer to reply data (i.e. the SRSP).
 *
 * @return      None.
 **************************************************************************************************
 */
typedef int (*pNPI_SendSynchDataFn) ( npiMsgData_t *pMsg );
extern const pNPI_SendSynchDataFn NPI_SendSynchDataFnArr[];

/**************************************************************************************************
 * @fn          NPI_AsynchMsgCback
 *
 * @brief       This function is a NPI callback to the client that indicates an
 *              asynchronous message has been received. The client software is
 *              expected to complete this call.
 *
 *              Note: The client must copy this message if it requires it
 *                    beyond the context of this call.
 *
 * input parameters
 *
 * @param       *pMsg - A pointer to an asychronously received message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
extern int NPI_AsynchMsgCback ( npiMsgData_t *pMsg );

/**************************************************************************************************
 * @fn          NPI_ResetSlave
 *
 * @brief       Reset the RNP. NOTE! Only valid for SPI and I2C
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
typedef int (*pNPI_ResetSlaveFn) ( void );
extern const pNPI_ResetSlaveFn NPI_ResetSlaveFnArr[];

/**************************************************************************************************
 * @fn          NPI_SynchSlave
 *
 * @brief       do the HW synchronization between the host and the RNP
 * 				NOTE! Only valid for SPI
 *
 * input parameters
 *
 * @param      none
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
typedef int (*pNPI_SynchSlaveFn) ( void );
extern const pNPI_SynchSlaveFn NPI_SynchSlaveFnArr[];

/**************************************************************************************************
*/

#ifdef __cplusplus
}
#endif

#endif /* NPI_H */
