/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "feeders.h"
#include "drivers/graviton_io.h"
#include "pico/time.h"
#include "report.h"

/*
    Public functions
*/

void feeders_init(struct FeedersState* s) { (void)s; };

void feeders_scan(struct FeedersState* s) {
    report_info_ln("Scanning for feeders");
    for (uint8_t i = 1; i < 128; i++) {
        struct FeederInfo* result = feeders_info(s, i);
        if (result == NULL) {
            report_result_ln("0x%02X: no response", i);
        } else {
            report_result_ln("0x%02X: present", i);
        }
    }
}

struct FeederInfo* feeders_info(struct FeedersState* s, uint8_t addr) {
    struct GravitonIODriver io_driver;
    GravitonIODriver_init(&io_driver, 1, 40);

    struct PhasonRequest req = {
        .command = PHASON_GET_FEEDER_INFO_REQ,
    };
    struct PhasonGetFeederInfoResponse resp;

    if (phason_send_get_feeder_info_request(&io_driver.io, addr, &req, &resp) != GRAVITON_READ_OK) {
        report_error_ln("No feeder found at address %u", addr);
        return NULL;
    }

    struct FeederInfo* feeder = &(s->feeders[addr]);
    feeder->addr = addr;
    feeder->protocol_version = resp.protocol_version;
    feeder->firmware_year = resp.firmware_year;
    feeder->firmware_month = resp.firmware_month;
    feeder->firmware_day = resp.firmware_day;
    memcpy(feeder->uid, resp.uid, PHASON_UID_SIZE);

    report_result(
        "Feeder address:%u protocol_version:%u firmware:%u.%u.%u uid",
        addr,
        feeder->protocol_version,
        feeder->firmware_year,
        feeder->firmware_month,
        feeder->firmware_day);

    for (size_t i = 0; i < PHASON_UID_SIZE; i++) { report_result(":%02X", feeder->uid[i]); }
    report_result_ln("");

    return &(s->feeders[addr]);
}
