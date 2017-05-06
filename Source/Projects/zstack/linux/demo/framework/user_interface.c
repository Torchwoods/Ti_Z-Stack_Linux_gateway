/*******************************************************************************
 Filename:      user_interface.c
 Revised:        $Date$
 Revision:       $Revision$

 Description:   User Interface


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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "user_interface.h"
#include "polling.h"
#include "types.h"
#include "socket_interface.h"
#include "timer_utils.h"
#include "data_structures.h"
#include "actions_engine.h"
#include "commissioning_engine.h"
#include "sensor_engine.h"
#include "attribute_engine.h"
#include "system_engine.h"
#include "nwkmgr.pb-c.h"
#include "ota_engine.h"
#include "group_scene_engine.h"
#include "macros.h"
#include "device_list_engine.h"

/******************************************************************************
 * Constants
 *****************************************************************************/
#define HA_GW_VERSION_MAJOR 1
#define HA_GW_VERSION_MINOR 00 

#define CLEAR_SCREEN		"\x1B[2J"

#define HIDE_CURSOR			"\x1B[?25l"
#define SHOW_CURSOR			"\x1B[?25h"

#define SCROLLING_ALL		"\x1B[r"
#define SCROLLING_(FROM,TO)	"\x1B[" #FROM ";" #TO "r"
#define SCROLLING(FROM,TO)	printf("\x1B[%d;%dr", (FROM), (TO))

#define SCROLL_DOWN			"\x1BD"
#define SCROLL_UP			"\x1BM"

#define LOCATE_(Y,X) "\x1B[" #Y ";" #X "H"
#define LOCATE(Y,X) printf("\x1B[%d;%dH",(Y),(X))

#define MAX_SUPPORTED_LINE_LENGTH 300
#define UI_MIN_LINE_LENGTH 150
#define UI_STATUS_LINE_LENGTH 80
#define UI_NETWORK_LINE_LENGTH 80
uint16_t ui_device_line_length = UI_MIN_LINE_LENGTH;

#define SECTION_TITLE_TOP 1
#define SECTION_TITLE_LINES 0

#define SECTION_NETWORK_TOP (SECTION_TITLE_TOP + SECTION_TITLE_LINES + 1)
#define SECTION_NETWORK_LINES 1

#define SECTION_DEVICES_TOP_0 (SECTION_NETWORK_TOP + SECTION_NETWORK_LINES + 1)
#define SECTION_DEVICES_TOP (SECTION_DEVICES_TOP_0 + 1)
uint8_t section_devices_lines = 10;

#define SECTION_GROUPS_TOP (SECTION_DEVICES_TOP + section_devices_lines + 1)
#define SECTION_GROUPS_LINES 1

#define SECTION_SCENES_TOP (SECTION_GROUPS_TOP + SECTION_GROUPS_LINES + 1)
#define SECTION_SCENES_LINES 1

#define SECTION_ACTIONS (SECTION_SCENES_TOP + SECTION_SCENES_LINES + 1)
uint8_t section_actions_lines = 2;
#define MAX_ACTIONS_IN_LINE_1 7

#define SECTION_LOG_TOP (SECTION_ACTIONS + section_actions_lines + 1)
uint8_t section_log_lines = 22;

#define SECTION_STATUS (SECTION_LOG_TOP + section_log_lines + 1)
#define SECTION_STATUS_LINES 1

#define SECTION_HELP_TOP (SECTION_STATUS + SECTION_STATUS_LINES + 1)
#define SECTION_HELP_LINES 2

#define UI_CURRENT_MAX_USED_LINE (SECTION_HELP_TOP + SECTION_HELP_LINES)

#define SENSOR_READ_INTERVAL 5000

#define ESCAPE_SEQUENCE_TIMEOUT 20

#define PROFILE_ID_HOME_AUTOMATION 0x0104
#define PROFILE_ID_ZIGBEE_LIGHT_LINK 0xC05E
#define DEVICE_ID_ON_OFF_SWITCH 0x0000
#define DEVICE_ID_MAINS_POWER_OUTLET 0x0009
#define DEVICE_ID_NON_STANDARD_MAINS_POWER_OUTLET 0x0501
#define DEVICE_ID_COLOR_LIGHT 0x0200
#define DEVICE_ID_EXT_COLOR_LIGHT 0x0210
#define DEVICE_ID_HVAC_TEMPERATURE_SENSOR 0x0302
#define DEVICE_ID_OCCUPANCY_SENSOR 0x0107
#define CLUSTER_ID_SMART_ENERGY_METERING 0x0702
#define CLUSTER_ID_TEMPERATURE_MEASUREMENT 0x0402
#define CLUSTER_ID_OCCUPANCY_SENSING 0x0406
#define CLUSTER_ID_HUMIDITY_MEASUREMENT 0x0405
#define CLUSTER_ID_ONOFF 0x0006

#define GROUP_MAX 10
#define SCENE_MAX 10

/******************************************************************************
 * Enums
 *****************************************************************************/
enum
{
	ACTION_ON_OFF,
	ACTION_LVL,
	ACTION_HUE,
	ACTION_SAT,
	ACTION_PRMT_JOIN,
	ACTION_REMOVE,
	ACTION_BIND_UNBIND,
	ACTION_GROUP_ADD_REMOVE,
	ACTION_SCENE_STORE_REMOVE,
	ACTION_SCENE_RECALL,
	ACTION_QUIT
};

enum
{
	ACTION_OFF,
	ACTION_ON
};

enum
{
	ACTION_BIND,
	ACTION_UNBIND
};

enum
{
	ACTION_GROUP_ADD,
	ACTION_GROUP_REMOVE
};

enum
{
	ACTION_SCENE_STORE,
	ACTION_SCENE_REMOVE
};

enum
{
	ACTION_TYPE_INTEGER,
	ACTION_TYPE_TRIGGER,
	ACTION_TYPE_STRING
};

enum
{
	SELECTION_MODE_DEVICES,
	SELECTION_MODE_GROUPS,
	SELECTION_MODE_SCENES
};

#define SELECTION_MODE_MAX SELECTION_MODE_SCENES

enum
{
	ADDRESSING_MODE_UNICAST,
	ADDRESSING_MODE_GROUPCAST,
	ADDRESSING_MODE_BROADCAST
};

#define ADDRESSING_MODE_MAX ADDRESSING_MODE_BROADCAST


/******************************************************************************
 * Constants
 *****************************************************************************/
#define MAX_SUPPORTED_SENSORS MAX_DEVICE_LIST


/******************************************************************************
 * Functional Macros
 *****************************************************************************/
#define NUM_OF_ACTIONS (sizeof(ACTION_NAMES) / sizeof(ACTION_NAMES[0]))

#define max(A,B) ((A) >= (B) ? (A) : (B))
#define min(A,B) ((A) <= (B) ? (A) : (B))

#define  UI_PRINT_HELP_START(fmt, ...) do { current_help_line = 0; UI_PRINT_HELP((fmt), ##__VA_ARGS__);} while (0)
#define  UI_PRINT_HELP(fmt, ...) do { last_help_line = max(last_help_line, current_help_line); if (last_printed_help_line == last_help_line) {last_printed_help_line = -1; current_help_line_start = 0;} if ((current_help_line == 0) && (last_printed_help_line != -1) && (last_printed_help_line < last_help_line)) {current_help_line_start += section_log_lines - (first_help_section ? 0 : 1);} if ((current_help_line >= current_help_line_start) && (current_help_line < current_help_line_start + section_log_lines - (first_help_section ? 0 : 1))) {_ui_print_log( __func__, false, (fmt), ##__VA_ARGS__); last_printed_help_line = current_help_line; if (current_help_line == 0) {first_help_section = true;}} else {if (current_help_line == 0) {first_help_section = false;}} current_help_line++;} while (0)


/******************************************************************************
 * Variables
 *****************************************************************************/
static tu_timer_t status_line_timeout = TIMER_RESET_VALUE;
static tu_timer_t sensor_read_periodic_timer = TIMER_RESET_VALUE;

bool sensor_reading_acive = false;
bool sensors_found = false;

struct termios old_tio;

FILE * logfile = NULL;

char * ACTION_NAMES[] =
{
	"O&N/O&FF",
	"&LVL",
	"&HUE",
	"&SAT",
	"&PERMIT_JOIN",
	"&REMOVE",
	"BIN&D/&UNBIND",
	"&GROUP_ADD/RE&MOVE",
	"SC&ENE_STORE/REMO&VE",
	"SCENE_RE&CALL",
	"&QUIT"
};

char * ACTION_SUBNAMES_0[] = 
{
	"O&FF",
	"O&N"
};

char * ACTION_SUBNAMES_6[] = 
{
	"BIN&D",
	"&UNBIND"
};

char * ACTION_SUBNAMES_7[] = 
{
	"&GROUP_ADD",
	"RE&MOVE"
};

char * ACTION_SUBNAMES_8[] = 
{
	"SC&ENE_STORE",
	"REMO&VE"
};

char ** ACTION_SUBNAMES[] = 
{
	ACTION_SUBNAMES_0,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	ACTION_SUBNAMES_6,
	ACTION_SUBNAMES_7,
	ACTION_SUBNAMES_8,
	NULL,
	NULL
};

static uint8_t current_action = 0;

uint16_t action_value[NUM_OF_ACTIONS] =
{
	1,
	250,
	250,
	254,
	60,
	0,
	0,
	0,
	0,
	0,
	0
};

uint16_t ACTION_VALUE_MAX[NUM_OF_ACTIONS] =
{
	2,
	256,
	256,
	256,
	256,
	1,
	2,
	2,
	2,
	1,
	1
};


uint16_t ACTION_TYPE[NUM_OF_ACTIONS] =
{
	ACTION_TYPE_STRING,
	ACTION_TYPE_INTEGER,
	ACTION_TYPE_INTEGER,
	ACTION_TYPE_INTEGER,
	ACTION_TYPE_INTEGER,
	ACTION_TYPE_TRIGGER,
	ACTION_TYPE_STRING,
	ACTION_TYPE_STRING,
	ACTION_TYPE_STRING,
	ACTION_TYPE_TRIGGER,
	ACTION_TYPE_TRIGGER
};

bool fast_action_enabled = false;
uint8_t addressing_mode = ADDRESSING_MODE_UNICAST;

uint8_t escape_state = 0;

char device_string[MAX_SUPPORTED_LINE_LENGTH * 2]; //*2 is for formatting symbol (just rough estimation of the required length)

uint8_t list_of_sensors_pending_read[MAX_SUPPORTED_SENSORS];

static uint8_t current_read_sensor;

uint8_t selected_group = 0;
uint8_t selected_scene = 0;

uint8_t selection_mode = SELECTION_MODE_DEVICES;

uint16_t current_help_line_start = 0;
int16_t last_printed_help_line = -1;
uint16_t last_help_line = 0;
bool first_help_section = true;
uint16_t current_help_line;

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void sensor_read_handler(void * arg);
void ui_redraw_actions(void);
void ui_redraw_device_list(void);
void ui_redraw_groups_list(void);
void ui_redraw_scenes_list(void);
void ui_redraw_selection_mode(void);
void ui_draw_screen_layout(void);
void ui_redraw_addressing_mode(void);

void ui_process_device_list(int * selected_device_index, int * prev_valid_device_index, int * next_valid_device_index);
int ui_get_selected_device_index(void);
int ui_get_selected_device_number(void);
void ui_process_bind(binding_mode_t binding_mode, int selected_device_index);

void ui_draw_separation_line(char * color);

/******************************************************************************
 * Functions
 *****************************************************************************/

void ui_refresh_display(void)
{
	ui_redraw_device_list();
	ui_redraw_groups_list();
	ui_redraw_scenes_list();
	ui_redraw_network_info();
}

void escape_sequence_timeout_handler(void * arg)
{
	escape_state = 0;
	UI_PRINT_LOG("escape_state ==> 0");
}

void ui_clear_status_line(void * arg)
{
	ui_print_status(0, "");
}

void ui_handle_cursor_location_report(uint16_t row, uint16_t col)
{
	ui_device_line_length = min(col, MAX_SUPPORTED_LINE_LENGTH);
	section_actions_lines = (ui_device_line_length < 142 ? 2 : 1);

	if (row > UI_CURRENT_MAX_USED_LINE)
	{
		section_log_lines += (row - UI_CURRENT_MAX_USED_LINE);
	}
	else if (row < UI_CURRENT_MAX_USED_LINE)
	{
		if ((UI_CURRENT_MAX_USED_LINE - row) < section_log_lines - 1)
		{
			section_log_lines -= (UI_CURRENT_MAX_USED_LINE - row);
		}
		else
		{
			section_log_lines = 2;
		}
	}

	ui_draw_screen_layout();
}

void console_event_handler(void * arg)
{
	char c;
	char ch[3];
	int i;
	int num_of_chars;
	static tu_timer_t escape_sequence_timeout = {0,0,0,0,0};
	int selected_device_index;
	int prev_valid_device_index;
	int next_valid_device_index;
	zb_addr_t destination_addr;
	bool perform_action = false;
	bool redraw_screen = false;
	bool addressing_ok = false;
	static uint16_t ansi_arg_1;
	static uint16_t ansi_arg_2;

	num_of_chars = read(STDIN_FILENO, ch, 3);

	for (i = 0; i < num_of_chars; i++)
	{
		ui_clear_status_line(NULL);

		c = ch[i];

		if (escape_state > 0)
		{
			tu_kill_timer(&escape_sequence_timeout);
		}

		if (escape_state == 5)
		{
			if (c == 'R')
			{
				ui_handle_cursor_location_report(ansi_arg_1, ansi_arg_2);
				
				escape_state = 99;
			}
			else if ((c >= '0') && (c <= '9'))
			{
				ansi_arg_2 = (ansi_arg_2 * 10) + (c - '0');
			}
			else
			{
				escape_state = 0;
			}
		}
		
		if (escape_state == 4)
		{
			if ((c >= '0') && (c <= '9'))
			{
				ansi_arg_2 = (c - '0');
				escape_state = 5;
			}
			else
			{
				escape_state = 0;
			}
		}

		if (escape_state == 3)
		{
			if (c == ';')
			{
				escape_state = 4;
			}
			else if ((c >= '0') && (c <= '9'))
			{
				ansi_arg_1 = (ansi_arg_1 * 10) + (c - '0');
			}
			else
			{
				escape_state = 0;
			}
		}

		if (escape_state == 2)
		{
			if ( (c == 65) || (c == 66) || (c == 67) || (c == 68))
			{
				escape_state = 99;

				switch (c)
				{
					case 65:
						if (selection_mode == SELECTION_MODE_DEVICES)
						{
							ui_process_device_list(&selected_device_index, &prev_valid_device_index, &next_valid_device_index);
							
							if (prev_valid_device_index != MAX_DEVICE_LIST)
							{
								ds_device_table[selected_device_index].selected = false;
								ds_device_table[prev_valid_device_index].selected = true;
								
								ui_redraw_device_list();
							}
							else
							{
								ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Reached top of device list");
							}
						}
						else
						{
							ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Use [`] to switch to Device Selection");
						}
						break;
					case 66:
						if (selection_mode == SELECTION_MODE_DEVICES)
						{
							ui_process_device_list(&selected_device_index, &prev_valid_device_index, &next_valid_device_index);
							ui_process_device_list(&selected_device_index, &prev_valid_device_index, &next_valid_device_index);
							if (next_valid_device_index != MAX_DEVICE_LIST)
							{
								if (selected_device_index != MAX_DEVICE_LIST)
								{
									ds_device_table[selected_device_index].selected = false;
								}
								ds_device_table[next_valid_device_index].selected = true;
								
								ui_redraw_device_list();
							}
							else
							{
								ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Reached bottom of device list");
							}
						}
						else
						{
							ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Use [`] to switch to Device Selection");
						}
						break;
					case 67:
						if (selection_mode == SELECTION_MODE_DEVICES)
						{
							selected_device_index = ui_get_selected_device_index();
							if ((selected_device_index != MAX_DEVICE_LIST) && (ds_device_table[selected_device_index].selected_endpoint_index < (ds_device_table[selected_device_index].num_endpoints - 1)))
							{
								ds_device_table[selected_device_index].selected_endpoint_index++;
								ui_redraw_device_list();
							}
						}
						else if (selection_mode == SELECTION_MODE_GROUPS)
						{
							if (selected_group < GROUP_MAX - 1)
							{
								selected_group++;

								ui_redraw_groups_list();
							}
						}
						else // if (selection_mode == SELECTION_MODE_SCENES)
						{
							if (selected_scene < SCENE_MAX - 1)
							{
								selected_scene++;

								ui_redraw_scenes_list();
							}
						}
						break;
					case 68:
						if (selection_mode == SELECTION_MODE_DEVICES)
						{
							selected_device_index = ui_get_selected_device_index();
							if ((selected_device_index != MAX_DEVICE_LIST) && (ds_device_table[selected_device_index].selected_endpoint_index > 0))
							{
								ds_device_table[selected_device_index].selected_endpoint_index--;
								ui_redraw_device_list();
							}
						}
						else if (selection_mode == SELECTION_MODE_GROUPS)
						{
							if (selected_group > 0)
							{
								selected_group--;

								ui_redraw_groups_list();
							}
						}
						else // if (selection_mode == SELECTION_MODE_SCENES)
						{
							if (selected_scene > 0)
							{
								selected_scene--;

								ui_redraw_scenes_list();
							}
						}
						break;
					default:
						break;
				}
			}
			else if ((c >= '0') && (c <= '9'))
			{
				ansi_arg_1 = (c - '0');
				escape_state = 3;
			}
			else
			{
				escape_state = 0;
			}
		}

		if (escape_state == 1)
		{
			if (c == 91)
			{
				escape_state = 2;
				tu_set_timer(&escape_sequence_timeout, ESCAPE_SEQUENCE_TIMEOUT, false, escape_sequence_timeout_handler, NULL);
			}
			else
			{
				escape_state = 0;
			}
		}

		if (escape_state == 0)
		{
			switch (c)
			{
				case ' ':
					current_action = (current_action + 1) % NUM_OF_ACTIONS;
					ui_redraw_actions();
					break;
				case 'q':
				case 'Q':
					current_action = ACTION_QUIT;
					perform_action = true;
					break;
				case 'f':
				case 'F':
					current_action = ACTION_ON_OFF;
					action_value[current_action] = ACTION_OFF;
					perform_action = true;
					break;
				case 'w':
				case 'W':
					selected_device_index = ui_get_selected_device_index();
					if (selected_device_index != MAX_DEVICE_LIST)
					{
						UI_PRINT_LOG("calling snsr_get_power()");
						destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
						destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
						snsr_get_power(destination_addr);
					}
					break;
				case 'n':
				case 'N':
					current_action = ACTION_ON_OFF;
					action_value[current_action] = ACTION_ON;
					perform_action = true;
					break;
				case 'a':
				case 'A':
					sensor_reading_acive = !sensor_reading_acive;
					UI_PRINT_LOG("sensor_reading_acive=%d", sensor_reading_acive);
					LOCATE(SECTION_ACTIONS, 12); printf(DARK_WHITE "%s", sensor_reading_acive ? "A" : "=");

					if (sensors_found && sensor_reading_acive)
					{
						if (!sensor_read_periodic_timer.in_use)
						{
							tu_set_timer(&sensor_read_periodic_timer, SENSOR_READ_INTERVAL, true, sensor_read_handler, NULL);
							sensor_read_handler(NULL);
						}
					}
					else
					{
						if (sensor_read_periodic_timer.in_use)
						{
							tu_kill_timer(&sensor_read_periodic_timer);
						}
					}
					break;
				case 'r':
				case 'R':
					current_action = ACTION_REMOVE;
					perform_action = true;
					break;
				case 'h':
				case 'H':
					current_action = ACTION_HUE;
					perform_action = true;
					break;
				case 's':
				case 'S':
					current_action = ACTION_SAT;
					perform_action = true;
					break;
				case '1':
					section_log_lines--;
					redraw_screen = true;
					break;
				case '2':
					section_log_lines++;
					redraw_screen = true;
					break;
				case '3':
					section_devices_lines--;
					section_log_lines++;
					redraw_screen = true;
					break;
				case '4':
					section_devices_lines++;
					section_log_lines--;
					redraw_screen = true;
					break;
				case 'l':
				case 'L':
					current_action = ACTION_LVL;
					perform_action = true;
					break;
				case 'y':
					{
						selected_device_index = ui_get_selected_device_index();
						if (selected_device_index != MAX_DEVICE_LIST)
						{
							destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
							destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
							attr_configure_reporting(destination_addr, gateway_self_addr, 5, CLUSTER_ID_TEMPERATURE_MEASUREMENT, ATTRID_TEMPERATURE, GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_INT16);
							attr_configure_reporting(destination_addr, gateway_self_addr, 5, CLUSTER_ID_HUMIDITY_MEASUREMENT, ATTRID_HUMDITY, GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_UINT16);
						}
					}
					break;
				case 'Y':
					{
						selected_device_index = ui_get_selected_device_index();
						if (selected_device_index != MAX_DEVICE_LIST)
						{
							destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
							destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
							attr_configure_reporting(destination_addr, gateway_self_addr, 0xFFFF, CLUSTER_ID_TEMPERATURE_MEASUREMENT, ATTRID_TEMPERATURE, GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_INT16);
							attr_configure_reporting(destination_addr, gateway_self_addr, 0xFFFF, CLUSTER_ID_HUMIDITY_MEASUREMENT, ATTRID_HUMDITY, GW_ZCL_ATTRIBUTE_DATA_TYPES_T__ZCL_DATATYPE_UINT16);
						}
					}
					break;
				case 'p':
				case 'P':
					current_action = ACTION_PRMT_JOIN;
					perform_action = true;
					break;
				case 10: //[ENTER]
					perform_action = true;
					break;
				case '+':
					action_value[current_action] = (action_value[current_action] + 1) % ACTION_VALUE_MAX[current_action];
					ui_redraw_actions();
					if ((fast_action_enabled)
						&& ((current_action == ACTION_LVL) 
						|| (current_action == ACTION_HUE)
						|| (current_action == ACTION_SAT)))
					{
						perform_action = true;
					}	
					break;
				case '[':
					record_macro = !record_macro;
					ui_redraw_toggles_indications();
					break;
				case ']':
					if ((pending_macro == NULL) && (!assign_macro))
					{
						ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT,"Please record a macro first");
					}
					else
					{
						assign_macro = !assign_macro;
						ui_redraw_toggles_indications();
					}
					break;
				case '=':
					printf(SCROLLING_ALL CURSOR_SAVE "\x1B[999B" "\x1B[999C" CURSOR_GET_POS CURSOR_RESTORE);
					break;
				case '-':
					action_value[current_action] = (action_value[current_action] + ACTION_VALUE_MAX[current_action] - 1) % ACTION_VALUE_MAX[current_action];
					ui_redraw_actions();
					if ((fast_action_enabled)
						&& ((current_action == ACTION_LVL) 
						|| (current_action == ACTION_HUE)
						|| (current_action == ACTION_SAT)))
					{
						perform_action = true;
					}	
					break;
				case '`':
					if (selection_mode < SELECTION_MODE_MAX)
					{
						selection_mode++;
					}
					else
					{
						selection_mode = 0;
					}

					ui_redraw_selection_mode();
					break;
				case 'z':
				case 'Z':
					UI_PRINT_LOG("");
					break;
				case '9':
					selected_device_index = ui_get_selected_device_index();
					if (selected_device_index != MAX_DEVICE_LIST)
					{
						destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
						destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
						attr_get_attribute_list(destination_addr);
					}
					break;
				case 't':
				case 'T':
					selected_device_index = ui_get_selected_device_index();
					if (selected_device_index != MAX_DEVICE_LIST)
					{
						destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
						destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
						snsr_get_temperature(destination_addr);
					}
					break;
				case 'x':
					system_send_reset_request(true);
					break;
				case 'X':
					system_send_reset_request(false);
					break;
				case 'k':
				case 'K':
					system_send_selfshutdown_request();
					break;
				case 'o':
				case 'O':
					ota_enable_state_machine(false, NULL);
					break;
				case 'b':
				case 'B':
					if (addressing_mode < ADDRESSING_MODE_MAX)
					{
						addressing_mode++;
					}
					else
					{
						addressing_mode = 0;
					}

					ui_redraw_addressing_mode();
					break;
				case 'd':
				case 'D':
					current_action = ACTION_BIND_UNBIND;
					action_value[current_action] = ACTION_BIND;
					perform_action = true;
					break;
				case 'g':
				case 'G':
					current_action = ACTION_GROUP_ADD_REMOVE;
					action_value[current_action] = ACTION_GROUP_ADD;
					perform_action = true;
					break;
				case 'm':
				case 'M':
					current_action = ACTION_GROUP_ADD_REMOVE;
					action_value[current_action] = ACTION_GROUP_REMOVE;
					perform_action = true;
					break;
				case 'e':
				case 'E':
					current_action = ACTION_SCENE_STORE_REMOVE;
					action_value[current_action] = ACTION_SCENE_STORE;
					perform_action = true;
					break;
				case 'v':
				case 'V':
					current_action = ACTION_SCENE_STORE_REMOVE;
					action_value[current_action] = ACTION_SCENE_REMOVE;
					perform_action = true;
					break;
				case 'c':
				case 'C':
					current_action = ACTION_SCENE_RECALL;
					perform_action = true;
					break;
				case 'u':
				case 'U':
					current_action = ACTION_BIND_UNBIND;
					action_value[current_action] = ACTION_UNBIND;
					perform_action = true;
				    break;
				case '*':
					fast_action_enabled = !fast_action_enabled;
					UI_PRINT_LOG("fast_action_enabled=%d", fast_action_enabled);
					LOCATE(SECTION_ACTIONS, 10); printf(DARK_WHITE "%s", fast_action_enabled ? "*" : "=");
					break;
				case '?':
					UI_PRINT_HELP_START(DARK_BLUE"Usage help start -------------------------------------------------");
					UI_PRINT_HELP(DARK_BLUE"- Either Devices, Groups or Scenes are selectable at a given time.");
					UI_PRINT_HELP(DARK_BLUE"  "LIGHT_BLUE"[`]"DARK_BLUE" - toggle between the three selection modes (Devices/Groups/Scenes).");
					UI_PRINT_HELP(DARK_BLUE"  The section title of the current selection mode is highlighted with a brighter white, and marked with [].");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[UP/DOWN]"DARK_BLUE" - select a device,");
					UI_PRINT_HELP(DARK_BLUE"  "LIGHT_BLUE"[LEFT/RIGHT]"DARK_BLUE" - select an endpoint/group/scene");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[SPACE]"DARK_BLUE" toggles between the available actions,");
					UI_PRINT_HELP(DARK_BLUE"  "LIGHT_BLUE"[+/-]"DARK_BLUE" changes the value associated with the action,");
					UI_PRINT_HELP(DARK_BLUE"  "LIGHT_BLUE"[ENTER]"DARK_BLUE" performs the selected action.");
					UI_PRINT_HELP(DARK_BLUE"- Each action has a quick access key associated with it (underlined in the action name);");
					UI_PRINT_HELP(DARK_BLUE"  Press a quick access key to select an action and perform it immidiately.");
					UI_PRINT_HELP(DARK_BLUE"- To issue a Bind/Unbind the action needs to be executed twice:");
					UI_PRINT_HELP(DARK_BLUE"  First it will select the binding source device, Second it will select the binding destination and perform the action.");
					UI_PRINT_HELP(DARK_BLUE"  if the selected binding destination is the same as the source, the operation is canceled.");
					UI_PRINT_HELP(DARK_BLUE"- Actions that requires destination address will use unicast, groupcast or broadcast addressing,");
					UI_PRINT_HELP(DARK_BLUE"  depending on the current addressing mode (binding and device removal always use unicast)");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[A/a]"DARK_BLUE" - Automatically read data from all sensors periodically.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[x]"DARK_BLUE" - Perform a soft reset.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[X]"DARK_BLUE" - Perform a hard reset.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[K/k]"DARK_BLUE" - Shutdown the gateway subsystem.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[O/o]"DARK_BLUE" - Initiate OTA operation.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[B/b]"DARK_BLUE" - Toggle between addressing modes (Unicast/Groupcast/Broadcast");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[*]"DARK_BLUE" - Enable/Disable immediate execution. (When enabled, '*' appears on the actions title line.)");
					UI_PRINT_HELP(DARK_BLUE"  When immediate execution is enabled, actions will be triggered as soon as [+/-] is pressed, after the respective value is updated.");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[[]"DARK_BLUE" - Record a single-command macro");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[]]"DARK_BLUE" - Assign a a recorded macro");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[1]"DARK_BLUE" - Display less log lines");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[2]"DARK_BLUE" - Display more log lines");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[3]"DARK_BLUE" - Display less device lines, but more log lines");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[4]"DARK_BLUE" - Display more device lines, but less log lines");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[=]"DARK_BLUE" - Redraw user interface according to actual terminal dimentions");
					UI_PRINT_HELP(DARK_BLUE"- "LIGHT_BLUE"[z]"DARK_BLUE" - Add a blank line to the log");
					UI_PRINT_HELP(DARK_BLUE"Usage help end ---------------------------------------------------");
					break;
				case 27:
					escape_state = 1;
					tu_set_timer(&escape_sequence_timeout, ESCAPE_SEQUENCE_TIMEOUT, false, escape_sequence_timeout_handler, NULL);
					break;
				default:
					break;
			}

			if (perform_action)
			{
				ui_redraw_actions();
				
				if ((current_action == ACTION_ON_OFF) 
					|| (current_action == ACTION_LVL)
					|| (current_action == ACTION_HUE)
					|| (current_action == ACTION_SAT)
					|| (current_action == ACTION_GROUP_ADD_REMOVE)
					|| (current_action == ACTION_SCENE_STORE_REMOVE)
					|| (current_action == ACTION_SCENE_RECALL))
				{
					switch (addressing_mode)
					{
						case ADDRESSING_MODE_BROADCAST:
						destination_addr.ieee_addr = 0;
						destination_addr.groupaddr = 0xFFFFFFFF;
						addressing_ok = true;
							break;
						case ADDRESSING_MODE_UNICAST:
								selected_device_index = ui_get_selected_device_index();
								if (selected_device_index != MAX_DEVICE_LIST)
								{
									destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
								destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;
									addressing_ok = true;
								}
								break;
						case ADDRESSING_MODE_GROUPCAST:
								destination_addr.ieee_addr = 0;
								destination_addr.groupaddr = selected_group;
								addressing_ok = true;
								break;
							default:
								break;
						}

					if (addressing_ok)
					{						
						switch (current_action)
						{
							case ACTION_ON_OFF:
								act_set_onoff(&destination_addr , action_value[current_action]);
								break;
							case ACTION_LVL:
								act_set_level(&destination_addr , 0, action_value[current_action]);
								break;
							case ACTION_HUE:
							case ACTION_SAT:
								act_set_color(&destination_addr, action_value[ACTION_HUE], action_value[ACTION_SAT]);
								break;
							case ACTION_GROUP_ADD_REMOVE:
								if (action_value[current_action] == ACTION_GROUP_ADD)
								{
									gs_add_group(&destination_addr, selected_group, "");
								}
								else
								{
									gs_remove_from_group(&destination_addr, selected_group);
								}
								break;
							case ACTION_SCENE_STORE_REMOVE:
								if (action_value[current_action] == ACTION_GROUP_ADD)
								{
									gs_store_scene(&destination_addr, selected_group, selected_scene);
								}
								else
								{
									gs_remove_scene(&destination_addr, selected_group, selected_scene);
								}
								break;
							case ACTION_SCENE_RECALL:
								gs_recall_scene(&destination_addr, selected_group, selected_scene);
								break;
							default:
								break;
						}
					}
				} else if ((current_action == ACTION_BIND_UNBIND)
					|| (current_action == ACTION_REMOVE))
				{
					selected_device_index = ui_get_selected_device_index();
					if (selected_device_index != MAX_DEVICE_LIST)
					{						
						destination_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
						destination_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;

						switch (current_action)
						{
							case ACTION_REMOVE:
								comm_remove_device_request(&destination_addr);
								break;
							case ACTION_BIND_UNBIND:
								ui_process_bind(action_value[current_action], selected_device_index);
								break;
							default:
								break;
						}
					}
				}
				else
				{
					switch (current_action)
					{
						case ACTION_PRMT_JOIN:
							comm_send_permit_join(action_value[current_action]); //TBD: update the API doc - unlimited should be 0xFF and not 0xFFFF as this is a uint8 value (verify in ZNP doc + spec)
							break;
						case ACTION_QUIT:
							polling_quit = true;
							break;
						default:
							break;
					}
				}
			}

			if (redraw_screen)
			{
				printf(CLEAR_SCREEN HIDE_CURSOR);
				
				ui_draw_screen_layout();
			}
		}
		
		if (escape_state == 99)
		{
			escape_state = 0;
		}
	}
}

void ui_print_status(uint64_t timeout_ms, char * format, ...)
{
	va_list args;
	char tempstr[UI_STATUS_LINE_LENGTH + 1];
	
	va_start(args, format);
	
	LOCATE(SECTION_STATUS + 1, 1);
	printf(DARK_CYAN);
	vsnprintf(tempstr, sizeof(tempstr), format, args);
	printf("%-*s", UI_STATUS_LINE_LENGTH, tempstr);
	printf(RESET_ATTRIBUTES);

	va_end(args);

	if (timeout_ms > 0)
	{
		tu_set_timer(&status_line_timeout, UI_STATUS_NOTIFICATION_TIMEOUT, false, ui_clear_status_line, NULL);
	}
	else
	{
		tu_kill_timer(&status_line_timeout);
	}
}

void _ui_print_log(const char * func_name, bool use_default_color, char * format, ...)
{
	va_list args;
	
	va_start(args, format);
	
	SCROLLING(SECTION_LOG_TOP + 1,SECTION_LOG_TOP + section_log_lines);
	LOCATE(SECTION_LOG_TOP + section_log_lines, 1);
	printf("\n");

	if (use_default_color)
	{
		printf(DARK_GREEN);
	}
	
	//printf("%s: ", func_name);
	vprintf(format, args);
	printf(RESET_ATTRIBUTES);

	if (logfile != NULL)
	{
		//fprintf(logfile, "%s: ", func_name);
		vfprintf(logfile, format, args);
		fprintf(logfile, "\n");
		fflush(logfile);
	}

	va_end(args);
}

bool ui_is_valid_device_record(device_info_t * device)
{
	return (device->valid);//TBD: && (device->device_status != GW_DEVICE_STATUS_T__DEVICE_REMOVED));
}

bool ui_is_device_a_smart_plug(device_info_t * device, uint8_t * sensor_ep)
{
	int ep_index;
	int cluster_index;
	
	if (ui_is_valid_device_record(device))
	{
		for (ep_index = 0; ep_index < device->num_endpoints; ep_index++)
		{
			if (device->ep_list[ep_index].profile_id == PROFILE_ID_HOME_AUTOMATION)
			{
				if ((device->ep_list[ep_index].device_id == DEVICE_ID_MAINS_POWER_OUTLET)
					|| (device->ep_list[ep_index].device_id == DEVICE_ID_NON_STANDARD_MAINS_POWER_OUTLET))
				{
					for (cluster_index = 0; cluster_index < device->ep_list[ep_index].num_ip_clusters; cluster_index++)
					{
						if (device->ep_list[ep_index].ip_cluster_list[cluster_index].cluster_id == CLUSTER_ID_SMART_ENERGY_METERING)
						{
							if (sensor_ep != NULL)
							{
								*sensor_ep = device->ep_list[ep_index].endpoint_id;
							}
							return true;
						}
					}
				}
			}
		}
	}
	
	return (false);
}

bool ui_is_device_a_temperature_sensor(device_info_t * device, uint8_t * sensor_ep)
{
	int ep_index;
	int cluster_index;
	
	if (ui_is_valid_device_record(device))
	{
		for (ep_index = 0; ep_index < device->num_endpoints; ep_index++)
		{
			if (device->ep_list[ep_index].profile_id == PROFILE_ID_HOME_AUTOMATION)
			{
				if (device->ep_list[ep_index].device_id == DEVICE_ID_HVAC_TEMPERATURE_SENSOR)
				{
					for (cluster_index = 0; cluster_index < device->ep_list[ep_index].num_ip_clusters; cluster_index++)
					{
						if (device->ep_list[ep_index].ip_cluster_list[cluster_index].cluster_id == CLUSTER_ID_TEMPERATURE_MEASUREMENT)
						{
							if (sensor_ep != NULL)
							{
								*sensor_ep = device->ep_list[ep_index].endpoint_id;
							}
							return true;
						}
					}
				}
			}
		}
	}
	
	return (false);
}

bool ui_is_device_a_humidity_sensor(device_info_t * device, uint8_t * sensor_ep)
{
	int ep_index;
	int cluster_index;
	
	if (ui_is_valid_device_record(device))
	{
		for (ep_index = 0; ep_index < device->num_endpoints; ep_index++)
		{
			if (device->ep_list[ep_index].profile_id == PROFILE_ID_HOME_AUTOMATION)
			{
				if (device->ep_list[ep_index].device_id == DEVICE_ID_HVAC_TEMPERATURE_SENSOR)
				{
					for (cluster_index = 0; cluster_index < device->ep_list[ep_index].num_ip_clusters; cluster_index++)
					{
						if (device->ep_list[ep_index].ip_cluster_list[cluster_index].cluster_id == CLUSTER_ID_HUMIDITY_MEASUREMENT)
						{
							if (sensor_ep != NULL)
							{
								*sensor_ep = device->ep_list[ep_index].endpoint_id;
							}
							return true;
						}
					}
				}
			}
		}
	}
	
	return (false);
}

bool ui_is_device_an_occupancy_sensor(device_info_t * device, uint8_t * sensor_ep)
{
	int ep_index;
	int cluster_index;
	
	if (ui_is_valid_device_record(device))
	{
		for (ep_index = 0; ep_index < device->num_endpoints; ep_index++)
		{
			if (device->ep_list[ep_index].profile_id == PROFILE_ID_HOME_AUTOMATION)
			{
				if (device->ep_list[ep_index].device_id == DEVICE_ID_OCCUPANCY_SENSOR)
				{
					for (cluster_index = 0; cluster_index < device->ep_list[ep_index].num_ip_clusters; cluster_index++)
					{
						if (device->ep_list[ep_index].ip_cluster_list[cluster_index].cluster_id == CLUSTER_ID_OCCUPANCY_SENSING)
						{
							if (sensor_ep != NULL)
							{
								*sensor_ep = device->ep_list[ep_index].endpoint_id;
							}
							return true;
						}
					}
				}
			}
		}
	}
	
	return (false);
}

bool ui_is_device_a_sensor(device_info_t * device)
{
	return (ui_is_device_a_smart_plug(device, NULL)
		|| ui_is_device_a_temperature_sensor(device, NULL)
		|| ui_is_device_an_occupancy_sensor(device, NULL));
}

void ui_redraw_server_state(void)
{
	LOCATE(SECTION_STATUS, 23);
	printf(DARK_WHITE "{%c%c%c}",
		si_is_server_ready(SI_SERVER_ID_NWK_MGR) ? 'N' : 'n', 
		si_is_server_ready(SI_SERVER_ID_GATEWAY) ? 'G' : 'g', 
		si_is_server_ready(SI_SERVER_ID_OTA) ? 'O' : 'o');
}

void ui_redraw_network_info(void)
{
	LOCATE(SECTION_NETWORK_TOP + 1, 1);
	
	if (ds_network_status.state == ZIGBEE_NETWORK_STATE_READY)
	{
		printf(DARK_MAGENTA "EXTPANID:" LIGHT_MAGENTA "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", (unsigned int)((ds_network_status.ext_pan_id >> 0) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 8) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 16) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 24) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 32) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 40) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 48) & 0xFF), (unsigned int)((ds_network_status.ext_pan_id >> 56) & 0xFF));
		printf(DARK_MAGENTA " PANID:" LIGHT_MAGENTA "0x%04X", ds_network_status.pan_id);
		printf(DARK_MAGENTA " CHANNEL:" LIGHT_MAGENTA "%d", ds_network_status.nwk_channel);
		printf(DARK_MAGENTA " STATE:" LIGHT_MAGENTA);

		if (ds_network_status.permit_remaining_time == 255)
		{
			printf(" OPEN          " );
		}
		else if (ds_network_status.permit_remaining_time > 0)
		{
			printf(" OPEN (%d s.)  ", ds_network_status.permit_remaining_time);
		}
		else
		{
			printf(" ACTIVE,CLOSED");
		}
	}
	else if (ds_network_status.state == ZIGBEE_NETWORK_STATE_INITIALIZING)
	{
		printf(LIGHT_MAGENTA "%-*s", UI_NETWORK_LINE_LENGTH, "Initializing network...");
	}
	else
	{
		printf(LIGHT_MAGENTA "%-*s", UI_NETWORK_LINE_LENGTH, "Waiting to connect to network manager server...");
	}
}

char * ui_make_string_GwAddrStructT(GwAddressStructT * addr)
{
	static char log_string[UI_LOG_LINE_LENGTH];
	
	STRING_START(log_string, "< type=%d", addr->addresstype);
	
	if (addr->has_ieeeaddr)
	{
		STRING_ADD(log_string, " ieeeaddr=");
		STRING_ADD_64BIT_HEX(log_string, addr->ieeeaddr);
	}
	
	if (addr->has_groupaddr)
	{
		STRING_ADD(log_string, " groupaddr=%08d", addr->groupaddr);
	}
	
	if (addr->has_broadcastaddr)
	{
		STRING_ADD(log_string, " broadcastaddr=%08d", addr->broadcastaddr);
	}
	
	if (addr->has_endpointid)
	{
		STRING_ADD(log_string, " endpointid=%08d", addr->endpointid);
	}
	
	STRING_ADD(log_string, " >");

	return log_string;
}

char * ui_make_string_OtaAddrStruct(AddressStruct * addr)
{
	static char log_string[UI_LOG_LINE_LENGTH];
	
	STRING_START(log_string, "< type=%d", addr->addrmode);
	
	if (addr->has_ieeeaddr)
	{
		STRING_ADD(log_string, " ieeeaddr=");
		STRING_ADD_64BIT_HEX(log_string, addr->ieeeaddr);
	}
	
	if (addr->has_groupaddr)
	{
		STRING_ADD(log_string, " groupaddr=%08d", addr->groupaddr);
	}
	
	if (addr->has_broadcaseaddr)
	{
		STRING_ADD(log_string, " broadcastaddr=%08d", addr->broadcaseaddr);
	}
	
	if (addr->has_endpointid)
	{
		STRING_ADD(log_string, " endpointid=%08d", addr->endpointid);
	}
	
	STRING_ADD(log_string, " >");

	return log_string;
}

void * last_sensor_read = NULL;

//For simplicity, this sample app supports only one sensor type per device, to avoid need for a state machine while waiting for the confirmation of each sensor reading before reading the next one (of the same device)
void ui_send_sensor_read(device_info_t * device)
{
	zb_addr_t destination_addr;
	uint8_t sensor_ep;
	uint32_t attr_id[5];

	if ((device == last_sensor_read) && ui_is_device_a_humidity_sensor(device, &sensor_ep))
	{
		destination_addr.ieee_addr = device->ieee_addr;
		destination_addr.endpoint = sensor_ep;
		attr_id[0] = ATTRID_HUMDITY;
		attr_send_read_attribute_request(&destination_addr, CLUSTER_ID_HUMIDITY_MEASUREMENT, 1, attr_id);
	}
	else if (ui_is_device_a_temperature_sensor(device, &sensor_ep))
	{
		destination_addr.ieee_addr = device->ieee_addr;
		destination_addr.endpoint = sensor_ep;
		snsr_get_temperature(destination_addr);
	}
	else if (ui_is_device_an_occupancy_sensor(device, &sensor_ep))
	{
		destination_addr.ieee_addr = device->ieee_addr;
		destination_addr.endpoint = sensor_ep;
		attr_id[0] = ATTRID_OCCUPANCY;
		attr_send_read_attribute_request(&destination_addr, CLUSTER_ID_OCCUPANCY_SENSING, 1, attr_id);
	}
	else if (ui_is_device_a_smart_plug(device, &sensor_ep))
	{
		destination_addr.ieee_addr = device->ieee_addr;
		destination_addr.endpoint = sensor_ep;
		attr_id[0] = ATTRID_SE_METERING_FORMATTING_SUMMATION_FORMATING;
		attr_id[1] = ATTRID_SE_METERING_FORMATTING_UNIT_OF_MEASURE;
		attr_id[2] = ATTRID_SE_METERING_FORMATTING_MULTIPLIER;
		attr_id[3] = ATTRID_SE_METERING_FORMATTING_DIVISOR;
		attr_id[4] = ATTRID_SE_METERING_INSTANTANEOUS_DEMAND;
		attr_send_read_attribute_request(&destination_addr, CLUSTER_ID_SMART_ENERGY_METERING, 5, attr_id);
	}
	
	last_sensor_read = device;
}

void ui_clear_list_of_sensors_pending_read(void)
{
	memset(list_of_sensors_pending_read, MAX_DEVICE_LIST, MAX_SUPPORTED_SENSORS);
	last_sensor_read = NULL;
}

void ui_sensor_read_state_machine(bool timed_out, void * arg)
{
	if (current_read_sensor == MAX_SUPPORTED_SENSORS)
	{
		current_read_sensor = 0;
	}
	else
	{
		if (timed_out)
		{
			UI_PRINT_LOG("Timeout reading sensor");
		}

		current_read_sensor++;
	}

	if (!si_is_server_ready(SI_SERVER_ID_GATEWAY))
	{
		si_unregister_idle_callback();
		UI_PRINT_LOG("sensor read - gateway server unavailable");
	}
	else
	{
		// The list of devices may have changed on the fly, so make sure this index is still pointing to a valid sensor
		while ((current_read_sensor < MAX_SUPPORTED_SENSORS) && (list_of_sensors_pending_read[current_read_sensor] != MAX_DEVICE_LIST) && (!ui_is_device_a_sensor(&ds_device_table[list_of_sensors_pending_read[current_read_sensor]])))
		{		
			UI_PRINT_LOG("Device list has changed, index %d does not point to a sensor now", list_of_sensors_pending_read[current_read_sensor]);
			current_read_sensor++;
		}
		
		if ((current_read_sensor < MAX_SUPPORTED_SENSORS) && (list_of_sensors_pending_read[current_read_sensor] != MAX_DEVICE_LIST))
		{
			ui_send_sensor_read(&ds_device_table[list_of_sensors_pending_read[current_read_sensor]]);
			UI_PRINT_LOG("sensor read - read request sent to 0x%04X", ds_device_table[list_of_sensors_pending_read[current_read_sensor]].nwk_addr);
		}
		else
		{
			si_unregister_idle_callback();
			UI_PRINT_LOG("sensor read - idle");
		}
	}
}

//assuming that by the time this function is called again, the previous call has already completed (all sensors from previous call were already processed).
void sensor_read_handler(void * arg)
{
	int j = 0;
	int i;
	
	if (si_get_idle_callback() == ui_sensor_read_state_machine)
	{
		UI_PRINT_LOG("Previous sensor scan still active");
	}
	else if (si_get_idle_callback() != NULL) //unexpected
	{
		UI_PRINT_LOG("Cannot register sensor read handler at this point");
	}
	else
	{
		ui_clear_list_of_sensors_pending_read(); //just in case
		
		for (i = 0; i < MAX_DEVICE_LIST; i++)
		{
			if (ui_is_valid_device_record(&ds_device_table[i]) && ui_is_device_a_sensor(&ds_device_table[i]))
			{
				if (ui_is_device_a_humidity_sensor(&ds_device_table[i], NULL) && ui_is_device_a_temperature_sensor(&ds_device_table[i], NULL))
				{
					list_of_sensors_pending_read[j++] = i;
				}
				list_of_sensors_pending_read[j++] = i;
			}
		}
		
		current_read_sensor = MAX_SUPPORTED_SENSORS;
		
		si_register_idle_callback(ui_sensor_read_state_machine, NULL);
		if (!si_is_waiting_for_confirmation())
		{
			ui_sensor_read_state_machine(false, NULL);
		}
	}
}

void ui_redraw_groups_list(void)
{
	int i;

	LOCATE(SECTION_GROUPS_TOP + 1, 1);
	printf(CYAN);

	if (selected_group > 0)
	{
		printf(NO_UNDERSCORE);
	}

	for (i = 0; i < GROUP_MAX; i++)
	{
		if (i == selected_group)
		{
			printf(UNDERSCORE BOLD);
			printf("GROUP_%d", i);
			printf(NO_UNDERSCORE NORMAL);
		}
		else
		{
			printf("group_%d", i);
		}

		printf(" ");
	}
}

void ui_redraw_scenes_list(void)
{
	int i;

	LOCATE(SECTION_SCENES_TOP + 1, 1);
	printf(GREEN);

	if (selected_scene > 0)
	{
		printf(NO_UNDERSCORE);
	}

	for (i = 0; i < SCENE_MAX; i++)
	{
		if (i == selected_scene)
		{
			printf(UNDERSCORE BOLD);
			printf("SCENE_%d", i);
			printf(NO_UNDERSCORE NORMAL);
		}
		else
		{
			printf("scene_%d", i);
		}

		printf(" ");
	}
}

void ui_redraw_selection_mode(void)
{
	switch (selection_mode)
	{
		case SELECTION_MODE_DEVICES:
			LOCATE(SECTION_SCENES_TOP, 1); ui_draw_separation_line(DARK_WHITE); printf("=SCENES");
			LOCATE(SECTION_DEVICES_TOP_0, 1); ui_draw_separation_line(DARK_WHITE); printf("="LIGHT_WHITE"[DEVICES]");
			break;
		case SELECTION_MODE_GROUPS:
			LOCATE(SECTION_DEVICES_TOP_0, 1); ui_draw_separation_line(DARK_WHITE); printf("=DEVICES");
			LOCATE(SECTION_GROUPS_TOP, 1); ui_draw_separation_line(DARK_WHITE); printf("="LIGHT_WHITE"[GROUPS]");
			break;
		case SELECTION_MODE_SCENES:
			LOCATE(SECTION_GROUPS_TOP, 1); ui_draw_separation_line(DARK_WHITE); printf("=GROUPS");
			LOCATE(SECTION_SCENES_TOP, 1); ui_draw_separation_line(DARK_WHITE); printf("="LIGHT_WHITE"[SCENES]");
			break;
		default:
			break;
	}
}

void ui_redraw_device_list(void)
{
	int i, j, k;
	uint16_t devices_found = 0;
	int selected_device_number;
	static uint16_t devicelist_offset = 0;
	int ep_index;
	int32_t value;
	uint8_t summation_formatting;
	bool leading_zeros = false;
	uint8_t right_digits = 2;
	uint8_t left_digits = 5;
	uint32_t multiplier = 1;
	uint32_t divisor = 1;
	float power;
	uint8_t unit_of_measure = 0;

	
	selected_device_number = ui_get_selected_device_number();
	if (selected_device_number > 0)
	{
		while (devicelist_offset > (selected_device_number - 1))
		{
			devicelist_offset--;
		}
		
		while ((devicelist_offset + section_devices_lines) < selected_device_number)
		{
			devicelist_offset++;
		}
	}
	else
	{
		devicelist_offset = 0;
	}
	
	for (i = 0; i < section_devices_lines; i++)
	{
		LOCATE(SECTION_DEVICES_TOP + 1 + i, 1);
		printf("%-*s", ui_device_line_length, "");
	}

	for (i = 0; (i < MAX_DEVICE_LIST) && ((devices_found - devicelist_offset) < section_devices_lines); i++)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]))
		{
			devices_found++;
			
			if ((!sensors_found) && ui_is_device_a_sensor(&ds_device_table[i]))
			{
				sensors_found = true;
			}
			
			if (devices_found > devicelist_offset)
			{
				STRING_START(device_string, "%s", "");
				STRING_ADD_64BIT_HEX(device_string, ds_device_table[i].ieee_addr);
				STRING_ADD(device_string, " %04X ", ds_device_table[i].nwk_addr);

				for (ep_index = 0; ep_index < ds_device_table[i].num_endpoints; ep_index++)
				{
					if (ep_index > 0)
					{
						STRING_ADD(device_string, DARK_WHITE " |%s%s" , ds_device_table[i].selected_as_bind_destination ? GREEN : RED, ds_device_table[i].selected ? BOLD : "");
						STRING_ADD(device_string, " ");
					}
					
					if ((ep_index == ds_device_table[i].selected_endpoint_index) && (ds_device_table[i].selected || ds_device_table[i].selected_as_bind_destination))
					{
						STRING_ADD(device_string, "%s", UNDERSCORE);
					}
					
					if (ds_device_table[i].ep_list[ep_index].profile_id == PROFILE_ID_HOME_AUTOMATION)
					{
						switch (ds_device_table[i].ep_list[ep_index].device_id)
						{
							case DEVICE_ID_NON_STANDARD_MAINS_POWER_OUTLET:
							case DEVICE_ID_MAINS_POWER_OUTLET:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " MAINS POWER OUTLET");
								break;
							case DEVICE_ID_HVAC_TEMPERATURE_SENSOR:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " TEMPERATURE SENSOR");
								break;
							case DEVICE_ID_OCCUPANCY_SENSOR:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " OCCUPANCY SENSOR");
								break;
							case DEVICE_ID_ON_OFF_SWITCH:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " ON/OFF SWITCH");
								break;
							default:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " %04X(HA)", ds_device_table[i].ep_list[ep_index].device_id);
								break;
						}
					}
					else if (ds_device_table[i].ep_list[ep_index].profile_id == PROFILE_ID_ZIGBEE_LIGHT_LINK)
					{
						switch (ds_device_table[i].ep_list[ep_index].device_id)
						{
							case DEVICE_ID_COLOR_LIGHT:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " COLOR LIGHT");
								break;
							case DEVICE_ID_EXT_COLOR_LIGHT:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " EXTENDED COLOR LIGHT");
								break;
							default:
								STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
								STRING_ADD(device_string, " %04X(ZLL)", ds_device_table[i].ep_list[ep_index].device_id);
								break;
						}
					}
					else
					{
						STRING_ADD(device_string, "%02X", ds_device_table[i].ep_list[ep_index].endpoint_id);
						STRING_ADD(device_string, " %04X(%04X)", ds_device_table[i].ep_list[ep_index].device_id, ds_device_table[i].ep_list[ep_index].profile_id);
					}

					if ((ep_index == ds_device_table[i].selected_endpoint_index) && (ds_device_table[i].selected || ds_device_table[i].selected_as_bind_destination))
					{
						STRING_ADD(device_string, "%s", NO_UNDERSCORE);
					}

					for (j = 0; j < ds_device_table[i].ep_list[ep_index].num_ip_clusters; j++)
					{
						for (k = 0; k < MAX_ATTRIBUTES; k++) //ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].num_attributes; k++)
						{
							if (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].valid)
							{
								switch(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].cluster_id)
								{
									case ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_MS_TEMPERATURE_MEASURED_VALUE:
												value = LITTLE_ENDIAN_TO_INT16(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val);
												STRING_ADD(device_string, ", tmp=%d.%d", value / 100, value % 100);
												break;
											default:
												break;
										}
										break;
									case ZCL_CLUSTER_ID_SE_SIMPLE_METERING:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_SE_METERING_FORMATTING_UNIT_OF_MEASURE:
												unit_of_measure = ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												break;

											case ATTRID_SE_METERING_FORMATTING_MULTIPLIER:
												multiplier = LITTLE_ENDIAN_TO_UINT24(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val);
												break;

											case ATTRID_SE_METERING_FORMATTING_DIVISOR:
												divisor = LITTLE_ENDIAN_TO_UINT24(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val);
												break;

											case ATTRID_SE_METERING_FORMATTING_SUMMATION_FORMATING:
												summation_formatting  = ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												leading_zeros = ((summation_formatting & 0x80) == 0);
												right_digits = (summation_formatting & 0x07);
												left_digits = ((summation_formatting >> 3) & 0x0F);
												break;

											case ATTRID_SE_METERING_INSTANTANEOUS_DEMAND:
												power = LITTLE_ENDIAN_TO_INT24(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val);
												power *= multiplier;
												power /= divisor;
												STRING_ADD(device_string, leading_zeros ? ", P=%0*.*f%s" : ", P=%*.*f%s", left_digits, right_digits, power, unit_of_measure == 0 ? " kW" : " ");
												break;
											default:
												break;
										}
										break;
									case ZCL_CLUSTER_ID_GEN_ON_OFF:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_ON_OFF:
												value = ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												STRING_ADD(device_string, ", %s", (value == 0) ? " OFF" : " ON");
												break;
											default:
												break;
										}
										break;
									case ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_LEVEL_CURRENT_LEVEL:
												value = (uint8_t)ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												STRING_ADD(device_string, ", L=%d", value);
												break;
											default:
												break;
										}
										break;
									case ZCL_CLUSTER_ID_LIGHTING_COLOR_CONTROL:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_HUE:
												value = (uint8_t)ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												STRING_ADD(device_string, ", H=%d", value);
												break;
											case ATTRID_LIGHTING_COLOR_CONTROL_CURRENT_SATURATION:
												value = (uint8_t)ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												STRING_ADD(device_string, ", S=%d", value);
												break;
											default:
												break;
										}
										break;
									case CLUSTER_ID_OCCUPANCY_SENSING:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_OCCUPANCY:
												value = (uint8_t)ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val[0];
												STRING_ADD(device_string, ", %s", value == 0 ? "UNOCCUPIED" : "OCCUPIED");
												break;
											default:
												break;
										}
										break;
									case CLUSTER_ID_HUMIDITY_MEASUREMENT:
										switch (ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_id)
										{
											case ATTRID_HUMDITY:
												value = LITTLE_ENDIAN_TO_UINT16(ds_device_table[i].ep_list[ep_index].ip_cluster_list[j].attribute_list[k].attr_val);
												STRING_ADD(device_string, ", hmd=%d.%d%%", value / 100, value % 100);
												break;
											default:
												break;
										}
										break;
									default:
										break;
								}
							}
						}
					}
				}
				
				LOCATE(SECTION_DEVICES_TOP + devices_found - devicelist_offset, 1);
				//printf("%s%-*s", (ds_device_table[i].selected ? LIGHT_RED ">" : DARK_RED " "), ui_device_line_length, device_string);

				printf(RESET_ATTRIBUTES "%s%s%-*s", ds_device_table[i].selected_as_bind_destination ? GREEN : RED, (ds_device_table[i].selected ? BOLD ">" : " "), ui_device_line_length, device_string);
			}
		}
	}
	
	for (; (!sensors_found) && (i < MAX_DEVICE_LIST); i++)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]))
		{
			if ((!sensors_found) && ui_is_device_a_sensor(&ds_device_table[i]))
			{
				sensors_found = true;
			}
		}
	}

	if (sensors_found && sensor_reading_acive)
	{
		if (!sensor_read_periodic_timer.in_use)
		{
			tu_set_timer(&sensor_read_periodic_timer, SENSOR_READ_INTERVAL, true, sensor_read_handler, NULL);
			sensor_read_handler(NULL);
		}
	}
	else
	{
		if (sensor_read_periodic_timer.in_use)
		{
			tu_kill_timer(&sensor_read_periodic_timer);
		}
	}
}

int ui_get_selected_device_number(void)
{
	int i;
	uint16_t devices_found = 0;
		
	for (i = 0; i < MAX_DEVICE_LIST; i++)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]))
		{
			devices_found++;
			
			if (ds_device_table[i].selected)
			{
				return devices_found;
			}
		}
	}
	
	return 0;
}

void ui_process_device_list(int * selected_device_index, int * prev_valid_device_index, int * next_valid_device_index)
{
	int i;
	
	if ((selected_device_index == NULL) | (prev_valid_device_index == NULL) || (next_valid_device_index == NULL))
	{
		return;
	}

	*selected_device_index = MAX_DEVICE_LIST;
	*prev_valid_device_index = MAX_DEVICE_LIST;
	*next_valid_device_index = MAX_DEVICE_LIST;
	
	for (i = MAX_DEVICE_LIST - 1; i >= 0 ; i--)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]))
		{
			if (ds_device_table[i].selected)
			{
				*selected_device_index = i;
			}
			else if (*selected_device_index == MAX_DEVICE_LIST)
			{
				*next_valid_device_index = i;
			}
			else
			{
				*prev_valid_device_index = i;
				return;
			}
		}
	}
}

int ui_get_selected_device_index(void)
{
	int i;
	
	for (i = 0; i < MAX_DEVICE_LIST; i++)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]) && ds_device_table[i].selected)
		{
			return i;
		}
	}

	ui_print_status(UI_STATUS_NOTIFICATION_TIMEOUT, "Please select a device first");
	
	return MAX_DEVICE_LIST;
}

void ui_redraw_addressing_mode(void)
{
	LOCATE(SECTION_ACTIONS, 17); printf(DARK_WHITE "%s", (addressing_mode == ADDRESSING_MODE_BROADCAST) ? "(BROADCAST)" : (addressing_mode == ADDRESSING_MODE_GROUPCAST) ? "(GROUPCAST)" : "(UNICAST)==");
}

void ui_redraw_actions(void)
{
	int i, j;

	LOCATE(SECTION_ACTIONS + 1, 1);
	for (i = 0; i < NUM_OF_ACTIONS; i++)
	{
		if ((section_actions_lines > 1) && (i == MAX_ACTIONS_IN_LINE_1))
		{
			printf("\n");
		}
		switch (ACTION_TYPE[i])
		{
			case ACTION_TYPE_INTEGER:
				printf("%s[%.*s"UNDERSCORE"%c"NO_UNDERSCORE"%s:%03d] ", (i == current_action ? LIGHT_YELLOW : DARK_YELLOW), strchr(ACTION_NAMES[i],'&') - ACTION_NAMES[i], ACTION_NAMES[i], strchr(ACTION_NAMES[i],'&')[1], strchr(ACTION_NAMES[i],'&') + 2, action_value[i]);
				break;
			case ACTION_TYPE_TRIGGER:
				printf("%s[%.*s"UNDERSCORE"%c"NO_UNDERSCORE"%s] ", (i == current_action ? LIGHT_YELLOW : DARK_YELLOW), strchr(ACTION_NAMES[i],'&') - ACTION_NAMES[i], ACTION_NAMES[i], strchr(ACTION_NAMES[i],'&')[1], strchr(ACTION_NAMES[i],'&') + 2);
				break;
			case ACTION_TYPE_STRING:
				printf("%s[", (i == current_action ? LIGHT_YELLOW : DARK_YELLOW));
				
				for (j = 0; j < ACTION_VALUE_MAX[i]; j++)
				{
					printf("%s%.*s"UNDERSCORE"%c"NO_UNDERSCORE"%s",
						(j == action_value[i]) ? ((i == current_action) ? LIGHT_WHITE : DARK_WHITE) : ((i == current_action) ? LIGHT_YELLOW : DARK_YELLOW),
						strchr(ACTION_SUBNAMES[i][j],'&') - ACTION_SUBNAMES[i][j],
						ACTION_SUBNAMES[i][j],
						strchr(ACTION_SUBNAMES[i][j],'&')[1],
						strchr(ACTION_SUBNAMES[i][j],'&') + 2);
					if (j < (ACTION_VALUE_MAX[i] - 1))
					{
						printf("%s\\", (i == current_action ? LIGHT_YELLOW : DARK_YELLOW));
					}
				}

				printf("%s] ", (i == current_action ? LIGHT_YELLOW : DARK_YELLOW));
				
				break;
		}
	}
	printf(RESET_ATTRIBUTES);
}

void ui_draw_separation_line(char * color)
{
	printf("%s%.*s\r", color, ui_device_line_length, "================================================================================================================================================================================================================================================================================================================================================================================================================");
}

void ui_draw_subtitle_line(char * color)
{
	printf("%s" UNDERSCORE "%*.s\r", color, ui_device_line_length, "");
}

void ui_draw_title_line(char * color)
{
	printf("%s%.*s\r", color, ui_device_line_length, "****************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************");
}

void ui_redraw_toggles_indications(void)
{
	LOCATE(SECTION_ACTIONS, 10); printf(DARK_WHITE "%s", fast_action_enabled ? "*" : "=");
	LOCATE(SECTION_ACTIONS, 12); printf(DARK_WHITE "%s", sensor_reading_acive ? "A" : "=");
	LOCATE(SECTION_ACTIONS, 13); printf(DARK_WHITE "%s", record_macro ? "R" : "=");
	LOCATE(SECTION_ACTIONS, 14); printf(DARK_WHITE "%s", assign_macro ? "M" : "=");
}

void ui_draw_screen_layout(void)
{
	printf(CLEAR_SCREEN);
	
	LOCATE(SECTION_TITLE_TOP, 1); 		ui_draw_title_line(LIGHT_WHITE);     printf(LIGHT_WHITE	"*** TI GATEWAY DEMO APP v%d.%02d ", HA_GW_VERSION_MAJOR, HA_GW_VERSION_MINOR);
	LOCATE(SECTION_NETWORK_TOP, 1);		ui_draw_separation_line(DARK_WHITE); printf("=NETWORK");
	LOCATE(SECTION_DEVICES_TOP_0, 1); 	ui_draw_separation_line(DARK_WHITE); printf("=DEVICES");
	LOCATE(SECTION_DEVICES_TOP, 1);		ui_draw_subtitle_line(DARK_WHITE);   printf(NO_UNDERSCORE " " UNDERSCORE "IEEE_ADDR              " NO_UNDERSCORE " " UNDERSCORE "ADDR" NO_UNDERSCORE " " UNDERSCORE "EP DETAILS | EP DETAILS | ...");
	LOCATE(SECTION_GROUPS_TOP, 1);		ui_draw_separation_line(DARK_WHITE); printf("=GROUPS");
	LOCATE(SECTION_SCENES_TOP, 1);		ui_draw_separation_line(DARK_WHITE); printf("=SCENES");
	LOCATE(SECTION_ACTIONS, 1);			ui_draw_separation_line(DARK_WHITE); printf("=ACTIONS");
	LOCATE(SECTION_LOG_TOP, 1);			ui_draw_separation_line(DARK_WHITE); printf("=LOG");
	LOCATE(SECTION_STATUS, 1);			ui_draw_separation_line(DARK_WHITE); printf("=APPLICATION-STATUS");
	LOCATE(SECTION_HELP_TOP, 1);		ui_draw_separation_line(DARK_WHITE); printf("=HELP");

	LOCATE(SECTION_HELP_TOP + 1, 1); printf(DARK_BLUE	"Press "LIGHT_BLUE"[?]"DARK_BLUE" to display usage help. ");
	LOCATE(SECTION_HELP_TOP + 2, 1); printf(DARK_BLUE	"If the log window is small, press [?] again to see more help.");

	ui_redraw_toggles_indications();
	ui_redraw_selection_mode();
	ui_redraw_actions();
	ui_redraw_network_info();
	ui_redraw_device_list();
	ui_redraw_groups_list();
	ui_redraw_scenes_list();
	ui_redraw_server_state();
	ui_redraw_addressing_mode();
}

int ui_init(char * log_filename)
{
	struct termios new_tio;
	char temp_char;
	uint16_t pos_row, pos_col;
	bool check_terminal_dimentions = true;

	if (log_filename != NULL)
	{
		logfile = fopen(log_filename, "a");
		
		if (logfile == NULL)
		{
			printf("ERROR could not open the output log file\n");
			
			return -1;
		}
	}

	tcgetattr(STDIN_FILENO,&old_tio);
	new_tio=old_tio;
	new_tio.c_lflag &=(~ICANON & ~ECHO);
	tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

	polling_define_poll_fd(STDIN_FILENO, POLLIN, console_event_handler, NULL);
	
	setvbuf(stdout, NULL, _IONBF, 0); //apparently this one does not need to be undone at deinit.
	ds_devices_total = 0;
	
	printf(CLEAR_SCREEN HIDE_CURSOR);
	
	while (check_terminal_dimentions)
	{
		printf(CURSOR_HOME "\x1B[%dB" "\x1B[%dC" CURSOR_GET_POS CURSOR_HOME, 999, 999);

		//note: if the user presses any key while/before reading the cursor position, the reading may be wrong. this case is not handled due to the low priority of this feature.
		if ((getchar() != 27) || (getchar() != 91))
		{
			check_terminal_dimentions = false; //any failure to read the terminal dimentions will fall-back to using the terminal as-is.
		}
		else
		{
			//note: not checking the sanity of the received digits
			pos_row = getchar() - 48;
			while ((temp_char = getchar()) != ';')
			{
				pos_row = pos_row * 10 + (temp_char - 48);
			}
			
			pos_col = getchar() - 48;
			while ((temp_char = getchar()) != 'R')
			{
				pos_col = pos_col * 10 + (temp_char - 48);
			}

			if ((pos_row < UI_CURRENT_MAX_USED_LINE) || (pos_col < UI_MIN_LINE_LENGTH))
			{
				printf("Terminal dimentions are not optimal - it is currently set to %d rows by %d columns.\n"
					   "Please resize terminal to at least %d rows by %d columns.\n"
					   "Press [ENTER] to recheck, [Q/q] to quit or any other key to use the current terminal dimentions.\n",
					   pos_row, pos_col, UI_CURRENT_MAX_USED_LINE, UI_MIN_LINE_LENGTH);

				temp_char = getchar();
				if ((temp_char == 'Q') || (temp_char == 'q'))
				{
					old_tio.c_lflag |=(ICANON | ECHO);
					tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);

					printf("Execution aborted by user.\n");
					printf("\n");

					printf(SHOW_CURSOR);
					return -1;
				}
				else if (temp_char != 10)
				{
					check_terminal_dimentions = false;
				}
			}
			else
			{
				check_terminal_dimentions = false;
			}
			
			if (check_terminal_dimentions == false)
			{
				ui_handle_cursor_location_report(pos_row, pos_col);
			}
		}
	}
		
	printf(CLEAR_SCREEN);

	ui_draw_screen_layout();

	ui_clear_list_of_sensors_pending_read();
	
	UI_PRINT_LOG("--- App started ---");
	UI_PRINT_LOG(BOLD "TI Home Automation Gateway Sample Application, version %d.%d", HA_GW_VERSION_MAJOR, HA_GW_VERSION_MINOR);

	macro_restore_all();
	
	return 0;
}

void ui_process_bind(binding_mode_t binding_mode, int selected_device_index)
{
	zb_addr_t source_addr;
	zb_addr_t dst_addr;
	int binding_source_device_index = MAX_DEVICE_LIST;
	int i;


	for (i = 0; i < MAX_DEVICE_LIST; i++)
	{
		if (ui_is_valid_device_record(&ds_device_table[i]) && ds_device_table[i].selected_as_bind_destination)
		{
			binding_source_device_index = i;
			break;
		}
	}
	
	if (binding_source_device_index == MAX_DEVICE_LIST)
	{
		UI_PRINT_LOG("Binding source selected");
		ds_device_table[selected_device_index].selected_as_bind_destination = true ;
		ui_redraw_device_list();
	}
	else if (binding_source_device_index == selected_device_index)
	{
		UI_PRINT_LOG("Binding source deselected");
		ds_device_table[selected_device_index].selected_as_bind_destination = false ;
		ui_redraw_device_list();
		return;
	}
	else
	{
		UI_PRINT_LOG("Sending %s command", binding_mode == BINDING_MODE_BIND ? "bind" : "unbind");
		
		source_addr.ieee_addr = ds_device_table[binding_source_device_index].ieee_addr;
		source_addr.endpoint = ds_device_table[binding_source_device_index].ep_list[ds_device_table[binding_source_device_index].selected_endpoint_index].endpoint_id;

		dst_addr.ieee_addr = ds_device_table[selected_device_index].ieee_addr;
		dst_addr.endpoint = ds_device_table[selected_device_index].ep_list[ds_device_table[selected_device_index].selected_endpoint_index].endpoint_id;

		comm_device_binding_entry_request(&source_addr, &dst_addr, CLUSTER_ID_ONOFF, binding_mode);

		ds_device_table[binding_source_device_index].selected_as_bind_destination = false;

		ui_redraw_device_list();

	}


}

void ui_deinit(void)
{
	if (logfile != NULL)
	{
		fclose(logfile);
	}
	
	printf(RESET_ATTRIBUTES SHOW_CURSOR SCROLLING_ALL CLEAR_SCREEN);
	LOCATE(1,1);
	
	old_tio.c_lflag |=(ICANON | ECHO);
	tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
}
