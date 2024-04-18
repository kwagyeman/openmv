/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2024 oclyke oclyke@oclyke.dev
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Logging macros.
 */
#pragma once

#ifdef OMV_DEBUG_PRINTF
#warning "OMV_DEBUG_PRINTF is deprecated, use OMV_LOG_LEVEL instead."
#ifndef OMV_LOG_LEVEL
#warning "OMV_DEBUG_PRINTF being used to set OMV_LOG_LEVEL to 3."
#define OMV_LOG_LEVEL 3
#endif
#endif // OMV_DEBUG_PRINTF

#define OMV_LOG_LEVEL_NONE 0
#define OMV_LOG_LEVEL_ERROR 1
#define OMV_LOG_LEVEL_WARNING 2
#define OMV_LOG_LEVEL_DEBUG 3
#define OMV_LOG_LEVEL_INFO 4
#define OMV_LOG_LEVEL_TRACE 5
#define OMV_NUM_LOG_LEVEL 6

// Default log level.
#ifndef OMV_LOG_LEVEL
#define OMV_LOG_LEVEL OMV_LOG_LEVEL_ERROR
#endif

// Validate log level.
#if OMV_LOG_LEVEL >= OMV_NUM_LOG_LEVEL
#error "OMV_LOG_LEVEL is set to an invalid value."
#endif

// If any logging is enabled then provide the LOG_PRINTF macro.
#if OMV_LOG_LEVEL == OMV_LOG_LEVEL_NONE
#define LOG_PRINTF(...)
#else
#include <stdio.h>
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#endif

// Trace logging.
#if OMV_LOG_LEVEL >= OMV_LOG_LEVEL_TRACE
#define LOG_TRACE(...) do { \
    LOG_PRINTF("TRACE: "); \
    LOG_PRINTF(__VA_ARGS__); \
} while (0)
#else
#define LOG_TRACE(...)
#endif

// Informational logging.
#if OMV_LOG_LEVEL >= OMV_LOG_LEVEL_INFO
#define LOG_INFO(...) do { \
    LOG_PRINTF("INFO: "); \
    LOG_PRINTF(__VA_ARGS__); \
} while (0)
#else
#define LOG_INFO(...)
#endif

// Debug logging.
#if OMV_LOG_LEVEL >= OMV_LOG_LEVEL_DEBUG
#define LOG_DEBUG(...) do { \
    LOG_PRINTF("DEBUG: "); \
    LOG_PRINTF(__VA_ARGS__); \
} while (0)
#else
#define LOG_DEBUG(...)
#endif

// Warning logging.
#if OMV_LOG_LEVEL >= OMV_LOG_LEVEL_WARNING
#define LOG_WARNING(...) do { \
    LOG_PRINTF("WARNING: "); \
    LOG_PRINTF(__VA_ARGS__); \
} while (0)
#else
#define LOG_WARNING(...)
#endif

// Error logging.
#if OMV_LOG_LEVEL >= OMV_LOG_LEVEL_ERROR
#define LOG_ERROR(...) do { \
    LOG_PRINTF("ERROR: "); \
    LOG_PRINTF(__VA_ARGS__); \
} while (0)
#else
#define LOG_ERROR(...)
#endif
