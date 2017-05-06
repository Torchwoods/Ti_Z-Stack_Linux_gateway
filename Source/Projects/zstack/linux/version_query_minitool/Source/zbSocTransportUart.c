/*
 * zbSocTransportUart.c
 *
 * This module contains the API for the zll SoC Host Interface.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*********************************************************************
 * INCLUDES
 */
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <errno.h>
#include <string.h>

#include "zbSocCmd.h"

/*********************************************************************
 * MACROS
 */
#define socWrite(fd,rpcBuff,rpcLen) \
	do { \
		if (uartDebugPrintsEnabled) \
		{ \
			int x; \
			time_t t = time(NULL); \
			struct tm tm = *localtime(&t); \
			printf("UART OUT %04d-%02d-%02d %02d:%02d:%02d --> %d Bytes: SOF:%02X, Len:%02X, CMD0:%02X, CMD1:%02X, Payload:", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, (rpcLen), (rpcBuff)[0], (rpcBuff)[1] , (rpcBuff)[2], (rpcBuff)[3] ); \
			for (x = 4; x < (rpcLen) - 1; x++) \
			{ \
			  printf("%02X%s", (rpcBuff)[x], x < (rpcLen) - 1 - 1 ? ":" : ","); \
			} \
			printf(" FCS:%02X\n", (rpcBuff)[x]); \
			printf("write=%d\n",write((fd),(rpcBuff),(rpcLen))); \
		} \
		else \
		{ \
			write((fd),(rpcBuff),(rpcLen)); \
		} \
	} while (0)
	
/*********************************************************************
 * CONSTANTS
 */
#define SB_FORCE_BOOT               0x10
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t uartDebugPrintsEnabled;

/*********************************************************************
 * GLOBAL VARIABLES
 */
int serialPortFd = -1;
unsigned int active_baudrate = B115200;

/*********************************************************************
 * FUNCTIONS
 */

void zbSocTransportUpdateBaudrate(unsigned int _active_baudrate)
{
	struct termios tio;

	active_baudrate = _active_baudrate;
	
	/* c-iflags
	B115200 : set board rate to 115200
	CRTSCTS : HW flow control (disabled below)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters*/

	/* c-iflags
	ICRNL   : maps 0xD (CR) to 0x10 (LR), we do not want this.
	IGNPAR  : ignore bits with parity erros, I guess it is 
	better to ignStateore an erronious bit then interprit it incorrectly. */
	
	tio.c_cflag = active_baudrate | CS8 | CLOCAL | CREAD; //CRTSCTS | 

	tio.c_iflag = IGNPAR & ~ICRNL; 
	tio.c_oflag = 0;
	tio.c_lflag = 0;

	tcsetattr(serialPortFd,TCSANOW,&tio);
}

 
bool zbSocTransportOpen( char *_devicePath  )
{
	static char lastUsedDevicePath[255];
	char * devicePath;

	if (_devicePath != NULL)
	{
		if (strlen(_devicePath) > (sizeof(lastUsedDevicePath) - 1))
		{
			printf("%s - device path too long\n",_devicePath);
			return false;
		}
		devicePath = _devicePath;
		strcpy(lastUsedDevicePath, _devicePath);
	}
	else
	{
		devicePath = lastUsedDevicePath;
	}

	/* open the device to be non-blocking (read will return immediatly) */
	serialPortFd = open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK); //oded: actually, O_NONBLOCK means that the call to open() is nonblocking; for non blocking i/o, should specify O_NDELAY
	if (serialPortFd <0) 
	{
		perror(devicePath); 
		printf("%s open failed\n",devicePath);
		return false;
	}

	//make the access exclusive so other instances will return -1 and exit
	ioctl(serialPortFd, TIOCEXCL);

	tcflush(serialPortFd, TCIFLUSH);

	zbSocTransportUpdateBaudrate(active_baudrate);

	return true;
}

void zbSocTransportClose( void )
{
	tcflush(serialPortFd, TCOFLUSH);
	close(serialPortFd);

	return;
}

void zbSocTransportWrite( uint8_t* buf, uint8_t len )
{
	socWrite(serialPortFd, buf, len);

	return;
}

uint8_t zbSocTransportRead( uint8_t* buf, uint8_t len )
{
	return read(serialPortFd, buf, len);
}
