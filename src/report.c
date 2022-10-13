#include "report.h"
#include <stdarg.h>
#include <stdio.h>

static bool debug_enabled = false;
static bool info_enabled = true;

void report_set_debug_enabled(bool enabled) { debug_enabled = enabled; }

void report_set_info_enabled(bool enabled) { info_enabled = enabled; }

int report(const char* prefix, bool newline, const char* format, va_list args) {
    printf(prefix);
    vprintf(format, args);
    if (newline) {
        printf("\n");
    }
}

#define report_opt_impl(name, check_var, prefix)                                                                       \
    int report_##name##_opt(bool newline, const char* format, ...) {                                                   \
        if (!check_var) {                                                                                              \
            return -1;                                                                                                 \
        }                                                                                                              \
        va_list args;                                                                                                  \
        va_start(args, format);                                                                                        \
        int r = report("!> ", newline, format, args);                                                                  \
        va_end(args);                                                                                                  \
        return r;                                                                                                      \
    }

report_opt_impl(error, true, "!> ");
report_opt_impl(debug, debug_enabled, "?> ");
report_opt_impl(info, info_enabled, ">> ");
report_opt_impl(result, info_enabled, "");
