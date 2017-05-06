
/**************************************************************************************************
 * Filename:       bbb_reset.c
 * Description:    Main File for BBB USB Reset Tool
 *
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

/*********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
 
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

//defined if app is hardcoded for bbb
#define BEAGLEBONE_RESET_USB

//register 
#define MUSB_DEVCT 0x47401C60

//register bit position
#define MUSB_DEVCTL_SESSION  0


int main(int argc, char **argv) {
        int fd;
	int bit_position; 
        void *map_base, *virt_addr; 
	unsigned char read_result, writeval;
	off_t target;
        
 #ifndef BEAGLEBONE_RESET_USB       
        int set_reset ; 
	bool read = false ;
#endif 
        
#ifdef BEAGLEBONE_RESET_USB
        
        target = MUSB_DEVCT ;
        bit_position = MUSB_DEVCTL_SESSION;
        
#else   // if not bbbb     
	
	if(argc == 1) {
		fprintf(stderr, "\nHelp:\t%s { address } { set/reset } { bit position }\n"
			"\taddress       : register address\n"
			"\tset/reset     : s= set, r = reset\n"
			"\tbit position  : bit position\n\n",
			argv[0]);
		exit(1);
	}

	target = strtoul(argv[1], 0, 0);

	if(argc ==2 )
	{   
		read = true; 		
	}
	else if (argc == 4)
	{
		read = false ; 
		set_reset = tolower(argv[2][0]);
		bit_position = strtol(argv[3], NULL, 0 );
	}
#endif
        
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) 
		{
			printf("Error !\n"); 
			exit (1);
	    }
    //printf("/dev/mem opened.\n"); 
    fflush(stdout);
    
    /* Map one page */
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    if(map_base == (void *) -1)
	{
			printf("Error !\n"); 
			exit (1);
	}
    //printf("Memory mapped at address %p.\n", map_base); 
    fflush(stdout);
    
    virt_addr = map_base + (target & MAP_MASK);

    read_result = *((unsigned char *) virt_addr);
    

#ifdef BEAGLEBONE_RESET_USB
    //printf("rvalue 0x%X \n",read_result);
   
    //Reset bit 0
    writeval = read_result & (~(1 << bit_position));
    *((unsigned short *) virt_addr) = writeval;
    
    //sleep for 1 sec
    usleep(1000000);
    read_result = *((unsigned char *) virt_addr);
    //printf("rvalue low 0x%X \n",read_result);
    
    //set bit 0
    writeval = read_result | (1 << bit_position);
    *((unsigned char *) virt_addr) = writeval;
    
    read_result = *((unsigned char *) virt_addr);    
    //printf("rvalue high 0x%X \n",read_result);
    
#else       
        
        printf(" initial reg value 0x%X Value: 0x%X\n", target, read_result); 
        printf(" set_reset %c bit_position: 0x%d \n", set_reset, bit_position); 

  
	if (read == false)
	{
		switch(set_reset) 
		{
			case 's':
				 writeval = read_result | (1 << bit_position);
				*((unsigned char *) virt_addr) = writeval;
				read_result = *((unsigned char *) virt_addr);
				break;
			case 'r':			
				 writeval = read_result & (~(1 << bit_position));
				*((unsigned short *) virt_addr) = writeval;
				read_result = *((unsigned short *) virt_addr);
				break;
		}

		printf("write byte 0x%X; read back 0x%X\n", writeval, read_result); 
	}

 #endif  	
	if(munmap(map_base, MAP_SIZE) == -1) 
	{
			printf("Error !\n"); 
			exit (1);
	};
    close(fd);
    
 
    return 0;
}

