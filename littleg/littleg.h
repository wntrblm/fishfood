#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Structs */

struct lilg_Decimal {
  int32_t real;
  int32_t frac;
  int32_t exp;
  bool set;
};

struct lilg_Command {
  struct lilg_Decimal G;
  struct lilg_Decimal M;
  struct lilg_Decimal X;
  struct lilg_Decimal Y;
  struct lilg_Decimal Z;
  struct lilg_Decimal fields[26];
};

/* Public methods */

inline static float lilg_Decimal_to_float(struct lilg_Decimal d) {
  float real = (float)(d.real);
  float frac = (float)(d.frac) * powf(10.0f, (float)(d.exp));
  if (real < 0)
    frac = -frac;
  return real + frac;
}

bool lilg_parse(struct lilg_Command* cmd, char c);

void lilg_Command_print(struct lilg_Command* cmd);
