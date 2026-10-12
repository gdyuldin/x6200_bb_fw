#pragma once

/*
 * Key-oriented command console on the debug UART (BSP_UART_DEBUG), the same
 * port the log is written to. Bytes are dispatched to the active mode; the
 * module owns no commands itself. A mode supplies its own key handler, status
 * report and optional 1 Hz tick, so calibration sets stay independent.
 */

typedef struct AppConsoleMode AppConsoleMode;

struct AppConsoleMode {
    char        key;
    const char *name;
    void      (*enter)(void);
    void      (*exit)(void);
    void      (*on_key)(char c);
    void      (*report)(void);
    void      (*help)(void);
    void      (*tick)(void);
};

int  App_Console_Init(void);
int  App_Console_RegisterMode(const AppConsoleMode *mode);
void App_Console_SetMode(const AppConsoleMode *mode);
void App_Console_Report(void);
