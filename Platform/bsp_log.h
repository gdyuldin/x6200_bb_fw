#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Logging for every layer. A line is emitted as "[L][tag] text\r\n", where L is
 * the level letter and tag is the LOG_TAG of the calling module. Levels are
 * macros (not enum constants) so the preprocessor can cut the disabled ones:
 * the LOG_* macros become ((void)0) above BSP_LOG_MAX_LEVEL, so neither the
 * calls nor the format strings reach the binary. Only a compile-time threshold
 * exists; there is no runtime level switch.
 */
#define BSP_LOG_ERROR 0u
#define BSP_LOG_WARN  1u
#define BSP_LOG_INFO  2u
#define BSP_LOG_DEBUG 3u
#define BSP_LOG_TRACE 4u

typedef uint8_t BspLogLevel;

/* Highest level compiled in. The build sets it; default keeps every level. */
#ifndef BSP_LOG_MAX_LEVEL
#define BSP_LOG_MAX_LEVEL BSP_LOG_TRACE
#endif

int    Bsp_Log_Init(void);
int    Bsp_Log_Print(BspLogLevel level, const char *tag, const char *fmt, ...);
int    Bsp_Log_Write(const char *data, size_t len);
int    Bsp_Log_WriteStr(const char *str);
size_t Bsp_Log_Dropped(void);

/*
 * Every logging .c defines LOG_TAG as "layer.module" (for example "app.power").
 * The tag is part of the emitted line, so a module logs without repeating its
 * name in the message text.
 */
#if BSP_LOG_MAX_LEVEL >= BSP_LOG_ERROR
#define LOG_ERROR(...) Bsp_Log_Print(BSP_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOG_ERROR(...) ((void)0)
#endif

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_WARN
#define LOG_WARN(...) Bsp_Log_Print(BSP_LOG_WARN, LOG_TAG, __VA_ARGS__)
#else
#define LOG_WARN(...) ((void)0)
#endif

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_INFO
#define LOG_INFO(...) Bsp_Log_Print(BSP_LOG_INFO, LOG_TAG, __VA_ARGS__)
#else
#define LOG_INFO(...) ((void)0)
#endif

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_DEBUG
#define LOG_DEBUG(...) Bsp_Log_Print(BSP_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#else
#define LOG_DEBUG(...) ((void)0)
#endif

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_TRACE
#define LOG_TRACE(...) Bsp_Log_Print(BSP_LOG_TRACE, LOG_TAG, __VA_ARGS__)
#else
#define LOG_TRACE(...) ((void)0)
#endif
