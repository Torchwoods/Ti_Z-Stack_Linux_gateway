#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "trace.h"

#ifndef DEFAULT_TRACE_ENABLE_MASK
#define DEFAULT_TRACE_ENABLE_MASK (trMaskFATAL | trMaskERROR | trMaskWARNING)
#endif

uint32_t default_trace_enable_mask = DEFAULT_TRACE_ENABLE_MASK;

pthread_key_t thread_name_key;
pthread_key_t thread_trace_mask_key;

bool trace_engine_initialized = FALSE;

char const * TRACE_LEVEL_NAME[] = 
{
	"",
	"FATAL",
	"ERROR",
	"WARNING",
	"INFO",
	"DEBUG",
	"CUSTOM",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	"",
	""
};

void thread_basic_destructor(void * name)
{
	if (name != NULL)
	{
		free(name);
	}
}

bool trace_init_main(char * thread_name)
{
	trace_engine_initialized = TRUE;
	
	if (pthread_key_create(&thread_name_key, thread_basic_destructor) != 0)
	{
		return FALSE;
	}

	if (pthread_key_create(&thread_trace_mask_key, thread_basic_destructor) != 0)
	{
		return FALSE;
	}

	return trace_init_thread(thread_name);
}

bool trace_init_thread(char * thread_name)
{
	char * specific_thread_name;
	uint32_t * thread_trace_mask;

	specific_thread_name = malloc(strlen(thread_name) + 1);
	if (specific_thread_name == NULL)
	{
		return FALSE;
	}
	
	thread_trace_mask = malloc(sizeof(uint32_t));
	if (thread_trace_mask == NULL)
	{
		free(specific_thread_name);
		return FALSE;
	}

	strcpy(specific_thread_name, thread_name);
	
	*thread_trace_mask =  default_trace_enable_mask;

	if (pthread_setspecific(thread_name_key, specific_thread_name) != 0)
	{
		free (specific_thread_name);
		free (thread_trace_mask);
		return FALSE;
	}

	if (pthread_setspecific(thread_trace_mask_key, thread_trace_mask) != 0)
	{
		pthread_setspecific(thread_name_key, NULL);
		free (specific_thread_name);
		free (thread_trace_mask);
		return FALSE;
	}

	return TRUE;
}

