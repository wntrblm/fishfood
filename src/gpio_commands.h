#pragma once

#include "littleg/littleg.h"
#include <stdint.h>

void gpio_commands_m42_set_pin(const struct lilg_Command cmd);
void gpio_commands_m43_report_pin(const struct lilg_Command cmd);
