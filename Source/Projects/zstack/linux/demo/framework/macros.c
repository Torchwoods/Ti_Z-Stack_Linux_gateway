/*******************************************************************************
 Filename:      macros.c
 Revised:        $Date: 2014-05-19 16:36:17 -0700 (Mon, 19 May 2014) $
 Revision:       $Revision: 38594 $

 Description:	Macro processing 


 Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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
*******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "user_interface.h"
#include "macros.h"

/*******************************************************************************
 * Types
 ******************************************************************************/

typedef struct _macro_element
{
	pkt_buf_t * macro;
	uint32_t endpointiddest;
	uint32_t endpointidsrc;
	struct _macro_element * next;
} macro_element;

/*******************************************************************************
 * Globals
 ******************************************************************************/

macro_element * macro_list = NULL;

bool record_macro = false;
bool assign_macro = false;
pkt_buf_t * pending_macro = NULL;

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

bool macro_save_all(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

void macro_confirmation_handler (pkt_buf_t *pkt, void *cbArg) 
{
	UI_PRINT_LOG("Received macro confirmation");
}

bool macro_assign_new(uint32_t endpointiddest, uint32_t endpointidsrc)
{
	macro_element ** current_macro_entry_ptr = &macro_list;
	macro_element * current_macro_entry = macro_list;
	uint16_t macro_count = 0;

	while ((current_macro_entry != NULL) && ((current_macro_entry->endpointiddest != endpointiddest) || (current_macro_entry->endpointidsrc != endpointidsrc)))
	{
		current_macro_entry_ptr = &current_macro_entry->next;
		current_macro_entry = current_macro_entry->next;
		macro_count++;
	}
	
	if (current_macro_entry == NULL)
	{
		*current_macro_entry_ptr = malloc(sizeof(macro_element));
		current_macro_entry = *current_macro_entry_ptr;
		if (current_macro_entry != NULL)
		{
			current_macro_entry->macro = NULL;
			current_macro_entry->next = NULL;
			UI_PRINT_LOG("Created new macro entry (#%d)", macro_count);
		}
	}
	else
	{
		UI_PRINT_LOG("Reusing existing macro entry (#%d)", macro_count);
	}

	if (current_macro_entry != NULL)
	{
		if (current_macro_entry->macro != NULL)
		{
			free(current_macro_entry->macro);
			UI_PRINT_LOG("Overwriting previously assigned macro");
		}
		
		current_macro_entry->endpointiddest = endpointiddest;
		current_macro_entry->endpointidsrc = endpointidsrc;
		current_macro_entry->macro = pending_macro;
		pending_macro = NULL;
	}
	else
	{
		return false;
	}
	
	macro_save_all();
	
	return true;
}

pkt_buf_t * macro_retrieve(uint32_t endpointiddest, uint32_t endpointidsrc)
{
	macro_element * current_macro_entry = macro_list;

	while ((current_macro_entry != NULL) && ((current_macro_entry->endpointiddest != endpointiddest) || (current_macro_entry->endpointidsrc != endpointidsrc)))
	{
		current_macro_entry = current_macro_entry->next;
	}
	
	if (current_macro_entry == NULL)
	{
		return NULL;
	}

	else
	{
		return current_macro_entry->macro;
	}
}

bool macro_save_all(void)
{
	FILE * macros_file;
	macro_element * current_macro_entry = macro_list;
	uint16_t macro_len;
	uint16_t macro_count = 0;
	bool rc = true;

	macros_file = fopen("macros.dat","wb");

	if (macros_file == NULL)
	{
		UI_PRINT_LOG("ERROR: failed opening macro file for writing");
		return false;
	}

	while ((current_macro_entry != NULL))
	{
		if (current_macro_entry->macro != NULL)
		{
			macro_len = sizeof(pkt_buf_hdr_t) + current_macro_entry->macro->header.len;
			if ((fwrite(&current_macro_entry->endpointiddest, sizeof(current_macro_entry->endpointiddest), 1, macros_file) != 1) ||
				(fwrite(&current_macro_entry->endpointidsrc, sizeof(current_macro_entry->endpointidsrc), 1, macros_file) != 1) ||
				(fwrite(&macro_len, sizeof(macro_len), 1, macros_file) != 1) ||
				(fwrite(current_macro_entry->macro, macro_len, 1, macros_file) != 1))
			{
				UI_PRINT_LOG("ERROR: failed writing to macro file");
				rc = false;
				break;
			}

			macro_count++;
		}
		
		current_macro_entry = current_macro_entry->next;
	}

	fclose(macros_file);

	UI_PRINT_LOG("Saved %d macros to file", macro_count);
	
	return rc;
}

bool macro_clear_all(void)
{
	macro_element * current_macro_entry = macro_list;
	uint16_t macro_count = 0;
	bool rc;

	while ((current_macro_entry != NULL))
	{
		if (current_macro_entry->macro != NULL)
		{
			free(current_macro_entry->macro);

			macro_count++;
		}
		
		macro_list = current_macro_entry->next;
		free(current_macro_entry);
		current_macro_entry = macro_list;
	}

	UI_PRINT_LOG("Discarded %d macros", macro_count);

	rc = macro_save_all();
	
	return rc;
}

bool macro_restore_all(void)
{
	FILE * macros_file;
	macro_element ** current_macro_entry_ptr = &macro_list;
	uint16_t macro_len;
	uint16_t macro_count = 0;
	bool rc = true;
	uint32_t endpointiddest;
	uint32_t endpointidsrc;

	macros_file = fopen("macros.dat","rb");

	if (macros_file == NULL)
	{
		UI_PRINT_LOG("Cannot open macro file for reading");
		return false;
	}


	while (fread(&endpointiddest, sizeof(endpointiddest), 1, macros_file) == 1)
	{
		*current_macro_entry_ptr = malloc(sizeof(macro_element));
		if (*current_macro_entry_ptr == NULL)
		{
			UI_PRINT_LOG("ERROR: failed allocating memory when restoring macro list");
			rc = false;
			break;
		}
		
		(*current_macro_entry_ptr)->next = NULL;
		
		(*current_macro_entry_ptr)->endpointiddest = endpointiddest;
		
		if ((fread(&endpointidsrc, sizeof(endpointidsrc), 1, macros_file) != 1) ||
			(fread(&macro_len, sizeof(macro_len), 1, macros_file) != 1) ||
			(((*current_macro_entry_ptr)->macro = malloc(macro_len)) == NULL) ||
			(fread((*current_macro_entry_ptr)->macro, macro_len, 1, macros_file) != 1))
		{
			UI_PRINT_LOG("ERROR: failed restoring macro list from file while processing macro #%d", macro_count);
			rc = false;
			break;
		}
		
		(*current_macro_entry_ptr)->endpointidsrc = endpointidsrc;

		macro_count++;
		
		current_macro_entry_ptr = &((*current_macro_entry_ptr)->next);
	}

	fclose(macros_file);

	UI_PRINT_LOG("Restored %d macros from file", macro_count);

	return rc;
}
