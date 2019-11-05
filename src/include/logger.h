/*
 * logger.h
 * very simple logging functions
 *
 * needs basename(), the prototype of which is defined in this file
 *
 * Copyright (C) 2019 Kiffie
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <logger_config.h>

#include <terminal.h>

/* log level definitions */
#define LOG_NONE    0
#define LOG_ERROR   1
#define LOG_INFO    2
#define LOG_DEBUG   3
#define LOG_VERBOSE 4

/* The macro LOGGER_ADD_FILENAME controls whether or not a source file name
 * and line number appear in the log output.
 */

char *basename(const char *path);

#ifdef LOGGER_ADD_FILENAME
    #define _log_printf(format, ...)\
	term_printf("[%s:%d] " format, basename(__FILE__), __LINE__, ## __VA_ARGS__)
#else
    #define _log_printf(format, ...)\
	term_printf(LOG_PREFIX ": " format, ## __VA_ARGS__)
#endif

#define log_verbose(format, ...) \
    if( LOGLEVEL >= LOG_VERBOSE ) _log_printf(format, ## __VA_ARGS__)

#define log_debug(format, ...) \
    if( LOGLEVEL >= LOG_DEBUG ) _log_printf(format, ## __VA_ARGS__)
#define log_debug_hexdump(buf, len)  \
    if( LOGLEVEL >= LOG_DEBUG ) term_hexdump8(buf, len)
	
#define log_info(format, ...) \
    if( LOGLEVEL >= LOG_INFO ) _log_printf(format, ## __VA_ARGS__)

#define log_error(format, ...) \
    if( LOGLEVEL >= LOG_ERROR ) _log_printf(format, ## __VA_ARGS__)

#endif
