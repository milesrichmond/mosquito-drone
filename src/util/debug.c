/**
 ************************************************************************************************
 * @file    debug.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "debug.h"
#include <stdio.h>

#ifdef DEBUG

int (*dbg_log)(const char *, ...) = printf;

#else

int empty_function(const char *, ...) {}
int (*dbg_log)(const char *, ...) = empty_function;
void initialize_monitor_handles(void) {}

#endif
