/**************************************************************************************************
  Filename:       npi_lnx.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file defines linux specific interface to Network Processor Interface
                  module.


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
#ifndef NPI_UART_LNX_H
#define NPI_UART_LNX_H

#ifdef __cplusplus
extern "C" {
#endif


#if !defined PACK_1
#define PACK_1
#endif

// To be compatible with MS and unix native target
// declare pragma for structure packing
#if defined(_MSC_VER) || defined(unix) || (defined(__ICC430__) && (__ICC430__==1))
#pragma pack(1)
#endif

  /////////////////////////////////////////////////////////////////////////////
  // Typedefs

PACK_1 typedef struct ATTR_PACKED
{
	  uint32 speed;
	  uint8 flowcontrol;
} npiUartCfg_t;

  /////////////////////////////////////////////////////////////////////////////
  // globals

  /////////////////////////////////////////////////////////////////////////////
  // Interface function prototypes

  /******************************************************************************
   * @fn         NPI_UART_OpenDevice
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
  extern int NPI_UART_OpenDevice(const char *portName, void *pCfg);

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
  extern void NPI_UART_CloseDevice(void);

  /**************************************************************************************************
   * @fn          NPI_UART_SendAsynchData
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
  extern int NPI_UART_SendAsynchData( npiMsgData_t *pMsg );

  /**************************************************************************************************
   * @fn          NPI_UART_SendSynchData
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
   * @param *pMsg  - Pointer to replay data (i.e. the SRSP).
   *
   * @return      None.
   **************************************************************************************************
   */
  extern int NPI_UART_SendSynchData( npiMsgData_t *pMsg );

#ifdef __cplusplus
}
#endif

#endif // NPI_UART_LNX_H
