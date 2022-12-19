/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "config/serial.h"
#include "graviton/phason.h"
#include "littleg/littleg.h"

struct FeederInfo {
    uint8_t addr;
    uint8_t protocol_version;
    uint8_t firmware_year;
    uint8_t firmware_month;
    uint8_t firmware_day;
    uint8_t uid[PHASON_UID_SIZE];
    uint8_t sequence;
};

struct FeedersState {
    struct FeederInfo feeders[128];
};

void feeders_init(struct FeedersState* s);
void feeders_scan(struct FeedersState* s);
struct FeederInfo* feeders_info(struct FeedersState* s, uint8_t addr);
void feeders_reset(struct FeedersState* s, uint8_t addr);
void feeders_feed(struct FeedersState* s, uint8_t addr, int32_t micrometers);
void feeders_query(struct FeedersState* s, uint8_t* uid);
