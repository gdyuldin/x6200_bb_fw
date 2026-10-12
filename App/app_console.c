#include "app_console.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_scheduler.h"
#include "bsp_log.h"
#include "bsp_uart.h"

#define LOG_TAG "app.console"

#define APP_CONSOLE_MAX_MODES 8u
#define APP_CONSOLE_POLL_MS   20u
#define APP_CONSOLE_TICK_MS   1000u

static const AppConsoleMode *s_modes[APP_CONSOLE_MAX_MODES];
static uint32_t              s_mode_count;
static const AppConsoleMode *s_active;

static const AppConsoleMode *mode_by_key(char c) {
    for (uint32_t i = 0u; i < s_mode_count; i++) {
        if (s_modes[i]->key == c) {
            return s_modes[i];
        }
    }
    return 0;
}

static void console_help(void) {
    if (s_active != 0 && s_active->help != 0) {
        s_active->help();
        return;
    }

    LOG_INFO("modes:");
    for (uint32_t i = 0u; i < s_mode_count; i++) {
        LOG_INFO("  %c - %s", s_modes[i]->key, s_modes[i]->name);
    }
    LOG_INFO("  ? help, q/esc leave mode");
}

static void console_dispatch(char c) {
    const AppConsoleMode *mode;

    if (c == '\r' || c == '\n') {
        return;
    }
    if (c == '?') {
        console_help();
        return;
    }
    if (c == 'q' || c == 0x1B) {
        if (s_active != 0) {
            App_Console_SetMode(0);
        }
        return;
    }

    mode = mode_by_key(c);
    if (mode != 0) {
        if (mode != s_active) {
            App_Console_SetMode(mode);
        } else {
            App_Console_Report();
        }
        return;
    }

    if (s_active == 0) {
        LOG_WARN("no mode, press ? for help");
        return;
    }
    if (s_active->on_key != 0) {
        s_active->on_key(c);
    }
    App_Console_Report();
}

static void console_poll(void) {
    uint8_t c = 0;

    while (Bsp_Uart_ReadChar(BSP_UART_DEBUG, &c) == 1) {
        console_dispatch((char)c);
    }
}

static void console_tick(void) {
    if (s_active != 0 && s_active->tick != 0) {
        s_active->tick();
    }
}

int App_Console_Init(void) {
    if (Bsp_Uart_StartRx(BSP_UART_DEBUG) != 0) {
        LOG_ERROR("debug rx start failed");
        return -1;
    }
    if (App_Scheduler_Add(console_poll, APP_CONSOLE_POLL_MS) != 0) {
        return -1;
    }
    return App_Scheduler_Add(console_tick, APP_CONSOLE_TICK_MS);
}

int App_Console_RegisterMode(const AppConsoleMode *mode) {
    if (mode == 0 || mode->key == '\0' || s_mode_count >= APP_CONSOLE_MAX_MODES) {
        return -1;
    }
    s_modes[s_mode_count++] = mode;
    return 0;
}

void App_Console_SetMode(const AppConsoleMode *mode) {
    if (mode == s_active) {
        return;
    }
    if (s_active != 0 && s_active->exit != 0) {
        s_active->exit();
    }
    s_active = mode;
    if (s_active != 0 && s_active->enter != 0) {
        s_active->enter();
    }
}

void App_Console_Report(void) {
    if (s_active != 0 && s_active->report != 0) {
        s_active->report();
    }
}
