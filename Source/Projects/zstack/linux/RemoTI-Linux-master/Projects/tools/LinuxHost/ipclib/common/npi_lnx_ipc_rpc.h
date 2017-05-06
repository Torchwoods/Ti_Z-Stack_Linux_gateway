/**************************************************************************************************
  Filename:       npi_lnx_ipc_rpc.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file contains information required by services that
                  use the Remote Procedure Call (RPC) standard to communicate
                  with (and control) the NPI Server.

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

#ifndef NPI_LNX_IPC_RPC_H
#define NPI_LNX_IPC_RPC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#define NPI_LNX_CMD_ID_CTRL_TIME_PRINT_REQ			0x01
#define NPI_LNX_CMD_ID_CTRL_BIG_DEBUG_PRINT_REQ		0x02
#define NPI_LNX_CMD_ID_VERSION_REQ					0x03
#define NPI_LNX_CMD_ID_GET_PARAM_REQ				0x04
#define NPI_LNX_CMD_ID_RESET_DEVICE					0x05
#define NPI_LNX_CMD_ID_DISCONNECT_DEVICE			0x06
#define NPI_LNX_CMD_ID_CONNECT_DEVICE				0x07

///////////////////////////////////////////////////////////////////////////////////////////////////
// Common
///////////////////////////////////////////////////////////////////////////////////////////////////
//Version 0.7.0
#define NPI_LNX_MAJOR_VERSION		0
#define NPI_LNX_MINOR_VERSION		7
#define NPI_LNX_REVISION			0

#define NPI_LNX_PARAM_NB_CONNECTIONS 		1
#define NPI_LNX_PARAM_DEVICE_USED			2

#define NPI_LNX_WORKAROUND_CDC_BOOTLOADER	1
/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif /* NPI_LNX_IPC_RPC_H */
