#ifndef TRACE_H
#define TRACE_H

#include <pthread.h>
#include <stdint.h>

extern pthread_key_t thread_name_key;
extern pthread_key_t thread_trace_mask_key;

extern char const * TRACE_LEVEL_NAME[];
extern bool trace_engine_initialized;

extern uint32_t default_trace_enable_mask;

#define trUNMASKABLE 0
#define trFATAL      1
#define trERROR      2
#define trWARNING    3
#define trINFO       4
#define trDEBUG      5
#define trCUSTOM     6
#define trMISC       7

#define trMaskUNMASKABLE (1 << trUNMASKABLE)
#define trMaskFATAL      (1 << trFATAL)
#define trMaskERROR      (1 << trERROR)
#define trMaskWARNING    (1 << trWARNING)
#define trMaskINFO       (1 << trINFO)
#define trMaskDEBUG      (1 << trDEBUG)
#define trMaskCUSTOM     (1 << trCUSTOM)
#define trMaskMISC       (1 << trMISC)

#define SPECIFIC_THREAD_NAME() (pthread_getspecific(thread_name_key) != NULL ? (char *)pthread_getspecific(thread_name_key) : "UNNAMED_THREAD")
#define THREAD_TRACE_MASK() (*(uint32_t *)pthread_getspecific(thread_trace_mask_key))

#define to_string(a) _to_string(a)
#define _to_string(a) # a

#ifndef SERVER_NAME
#define SERVER_NAME UNNAMED_PROCESS
#endif

#define uiPrintfEx(trace_group, fmt, ...) \
	do \
	{ \
		if (!trace_engine_initialized) \
		{ \
			printf("WARNING: uiPrintf used before calling trace_init_main(). Traces not available.\n"); \
			fflush(stdout); \
		} \
		else \
		{ \
			if (((1 << trace_group) & (THREAD_TRACE_MASK() | trMaskUNMASKABLE)) != 0) \
			{ \
				printf("[%s/%s] %s: " fmt, to_string(SERVER_NAME), SPECIFIC_THREAD_NAME(), TRACE_LEVEL_NAME[trace_group], ##__VA_ARGS__); \
				fflush(stdout); \
			} \
		} \
	} while (0)


#define uiPrintf(fmt, ...) uiPrintfEx(trMISC, fmt, ##__VA_ARGS__)

bool trace_init_main(char * thread_name);
bool trace_init_thread(char * thread_name);

#endif /* TRACE_H */
