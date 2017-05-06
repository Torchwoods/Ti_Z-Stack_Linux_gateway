/**************************************************************************************************
  Filename:       aic.h
  Revised:        $Date: 2011-11-23 12:02:49 -0800 (Wed, 23 Nov 2011) $
  Revision:       $Revision: 108 $

  Description:    This file defines interface to Asymmetric Inter-processor
  	  	  	  	  Communication module.


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
#ifndef AIC_H
#define AIC_H

#include "hal_rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

  /////////////////////////////////////////////////////////////////////////////
  // Constants

// Frame formatting constants

// Start of frame character value
#define AIC_UART_SOF              RPC_UART_SOF

// SOF and FCS
#define AIC_UART_FRAME_OVHD   2
  
// 1st byte is the length of the data field, 2nd/3rd bytes are command field.
#define AIC_FRAME_HDR_SZ  3

// maximum length of data in the general frame format
#define AIC_DATA_MAX       (256 - AIC_FRAME_HDR_SZ - AIC_UART_FRAME_OVHD)

// The 3 MSB's of the 1st command field byte are for command type.
#define AIC_CMD_TYPE_MASK  0xE0

// The 5 LSB's of the 1st command field byte are for the subsystem.
#define AIC_SUBSYSTEM_MASK 0x1F

// position of fields in the general format frame
#define AIC_POS_LEN        0
#define AIC_POS_CMD0       1
#define AIC_POS_CMD1       2
#define AIC_POS_DAT0       3
  
// Subsystem field values
#define AIC_SUBSYSTEM_RCN         RPC_SYS_RCN
#define AIC_SUBSYSTEM_RCN_CLIENT  RPC_SYS_RCN_CLIENT
#define AIC_SUBSYSTEM_ASCII_TRACE RPC_SYS_MAX


#ifdef __cplusplus
}
#endif

#endif // AIC_H
