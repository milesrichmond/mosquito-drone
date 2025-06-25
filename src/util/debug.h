/**
 ************************************************************************************************
 * @file    debug.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#ifndef DEBUG_H
#define DEBUG_H

/**
 *  Depending on the compiler flags, this will either have no function
 *  body (DEBUG undefined), or will point to printf (DEBUG defined).
 */
extern int (*dbg_log)(const char *, ...);

extern int initialise_monitor_handles(void);

#endif
