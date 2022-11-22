/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void report_set_debug_enabled(bool enabled);
void report_set_info_enabled(bool enabled);

int report_error_opt(bool newline, const char* format, ...) __attribute__((format(printf, 2, 3)));
int report_debug_opt(bool newline, const char* format, ...) __attribute__((format(printf, 2, 3)));
int report_info_opt(bool newline, const char* format, ...) __attribute__((format(printf, 2, 3)));
int report_result_opt(bool newline, const char* format, ...) __attribute__((format(printf, 2, 3)));

#define report_error(format, ...) report_error_opt(false, format __VA_OPT__(, ) __VA_ARGS__)
#define report_debug(format, ...) report_debug_opt(false, format __VA_OPT__(, ) __VA_ARGS__)
#define report_info(format, ...) report_info_opt(false, format __VA_OPT__(, ) __VA_ARGS__)
#define report_result(format, ...) report_result_opt(false, format __VA_OPT__(, ) __VA_ARGS__)

#define report_error_ln(format, ...) report_error_opt(true, format __VA_OPT__(, ) __VA_ARGS__)
#define report_debug_ln(format, ...) report_debug_opt(true, format __VA_OPT__(, ) __VA_ARGS__)
#define report_info_ln(format, ...) report_info_opt(true, format __VA_OPT__(, ) __VA_ARGS__)
#define report_result_ln(format, ...) report_result_opt(true, format __VA_OPT__(, ) __VA_ARGS__)
