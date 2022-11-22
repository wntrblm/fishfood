/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "littleg/littleg.h"
#include <stdint.h>

void gpio_commands_m42_set_pin(const struct lilg_Command cmd);
void gpio_commands_m43_report_pin(const struct lilg_Command cmd);
