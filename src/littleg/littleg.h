/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Macros and constants */
#define LILG_FIELDC(command, c) (command.fields[(size_t)(c - 'A')])
#define LILG_FIELD(command, letter) (command.fields[(size_t)(#letter[0] - 'A')])

/* Structs */

struct lilg_Decimal {
    int32_t real;
    int32_t frac;
    int32_t exp;
    bool set;
};

struct lilg_Command {
    char first_field;
    struct lilg_Decimal G;
    struct lilg_Decimal M;
    struct lilg_Decimal X;
    struct lilg_Decimal Y;
    struct lilg_Decimal Z;
    struct lilg_Decimal fields[26];
};

enum lilg_ParseResult {
    LILG_VALID = 0,
    LILG_INVALID = 1,
    LILG_INCOMPLETE = 2,
};

/* Public methods */

inline static float lilg_Decimal_to_float(struct lilg_Decimal d) {
    float real = (float)(d.real);
    float frac = (float)(d.frac) * powf(10.0f, (float)(d.exp));
    if (real < 0)
        frac = -frac;
    return real + frac;
}

enum lilg_ParseResult lilg_parse(struct lilg_Command* cmd, char c);

void lilg_Command_print(struct lilg_Command* cmd);
