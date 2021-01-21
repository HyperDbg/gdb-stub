#pragma once

#ifndef _GDBSTUB_H_
#define _GDBSTUB_H_

 /* Enable debug statements (printf) */
#ifndef DEBUG
#define DEBUG 0
#endif

/* Include platform specific definitions */
#include "gdbstub_sys.h"

/*****************************************************************************
 * Macros
 ****************************************************************************/

#if DEBUG
#define DEBUG_PRINT(...) fprintf(stderr, __VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

#ifndef EOF
#define EOF (-1)
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

#ifndef ASSERT
#if DEBUG
#define ASSERT(x) { \
	if (!(x)) { \
		fprintf(stderr, "ASSERTION FAILED\n"); \
		fprintf(stderr, "  Assertion: %s\n", #x); \
		fprintf(stderr, "  Location:  %s @ %s:%d\n", __func__, \
		                                             __FILE__, __LINE__); \
		exit(1); \
	} \
}
#else
#define ASSERT(x) \
	do {} while (0)
#endif
#endif

 /*****************************************************************************
  * Prototypes
  ****************************************************************************/

int dbg_main(struct dbg_state* state);

/* System functions, supported by all stubs */
int dbg_sys_getc(void);
int dbg_sys_putchar(int ch);
int dbg_sys_mem_readb(address addr, char* val);
int dbg_sys_mem_writeb(address addr, char val);
int dbg_sys_continue();
int dbg_sys_step();

#endif