/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "feeders.h"
#include "drivers/graviton_io.h"
#include "pico/time.h"
#include "report.h"

/*
    Macros and constants
*/

#define FIRST_BYTE_TIMEOUT_MS 2
#define TOTAL_TIMEOUT_MS 60
#define FEED_CHECK_DEADLINE_MS 10000lu
#define FEED_CHECK_INTERVAL_MS 1000lu

/*
    Public functions
*/

void feeders_init(struct FeedersState* s) { (void)s; };

void feeders_scan(struct FeedersState* s) {
    report_info_ln("Scanning for feeders");
    for (uint8_t i = 1; i < 128; i++) {
        struct FeederInfo* result = feeders_info(s, i);
        if (result != NULL) {
            report_result_ln("@%2u: present", i);
        }
    }
}

struct FeederInfo* feeders_info(struct FeedersState* s, uint8_t addr) {
    struct GravitonIODriver io_driver;
    GravitonIODriver_init(&io_driver, FIRST_BYTE_TIMEOUT_MS, TOTAL_TIMEOUT_MS);

    struct PhasonRequest req = {
        .command = PHASON_FEEDER_INFO_REQ,
    };
    struct PhasonFeederInfoResponse resp;

    if (phason_send_feeder_info_request(&io_driver.io, addr, &req, &resp) != GRAVITON_READ_OK) {
        report_error_ln("No feeder found @%u", addr);
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

void feeders_reset(struct FeedersState* s, uint8_t addr) {
    struct GravitonIODriver io_driver;
    GravitonIODriver_init(&io_driver, FIRST_BYTE_TIMEOUT_MS, TOTAL_TIMEOUT_MS);

    struct PhasonRequest req = {
        .command = PHASON_RESET_FEEDER_REQ,
    };
    struct PhasonResponse resp;

    if (phason_send_reset_feeder_request(&io_driver.io, addr, &req, &resp) != GRAVITON_READ_OK) {
        report_error_ln("No response from feeder @%u", addr);
        return;
    }

    s->feeders[addr].sequence = 0;

    report_result_ln("Reset feeder @%u", addr);
}

void feeders_feed(struct FeedersState* s, uint8_t addr, int32_t micrometers) {
    struct GravitonIODriver io_driver;
    GravitonIODriver_init(&io_driver, FIRST_BYTE_TIMEOUT_MS, TOTAL_TIMEOUT_MS);

    struct FeederInfo* feeder = &(s->feeders[addr]);
    feeder->sequence += 1;

    struct PhasonStartFeedRequest req = {
        .command = PHASON_START_FEED_REQ,
        .micrometers = micrometers,
        .sequence = feeder->sequence,
    };
    struct PhasonStartFeedResponse resp;

    if (phason_send_start_feed_request(&io_driver.io, addr, (const struct PhasonRequest*)&req, &resp) !=
        GRAVITON_READ_OK) {
        report_error_ln("No response from feeder @%u", addr);
        return;
    }

    report_info_ln(
        "Feed started, deadline in %lu ms, checking status in %lu ms...",
        FEED_CHECK_DEADLINE_MS,
        FEED_CHECK_INTERVAL_MS);
    sleep_ms(FEED_CHECK_INTERVAL_MS);

    absolute_time_t deadline = make_timeout_time_ms(FEED_CHECK_DEADLINE_MS);

    while (true) {
        GravitonIODriver_init(&io_driver, FIRST_BYTE_TIMEOUT_MS, TOTAL_TIMEOUT_MS);

        struct PhasonFeedStatusRequest status_req = {
            .command = PHASON_FEED_STATUS_REQ,
            .sequence = feeder->sequence,
        };
        struct PhasonFeedStatusResponse status_resp;

        int32_t read_status = phason_send_feed_status_request(
            &io_driver.io, addr, (const struct PhasonRequest*)&status_req, &status_resp);

        if (read_status != GRAVITON_READ_OK) {
            if (time_reached(deadline)) {
                report_error_ln("Feed deadline exceeded");
                return;
            } else {
                report_info_ln("No response, checking again in %lu ms...", FEED_CHECK_INTERVAL_MS);
                sleep_ms(FEED_CHECK_INTERVAL_MS);
                continue;
            }
        }

        switch (status_resp.status) {
            case PHASON_OK:
                report_result_ln("Feed complete, actual feed distance: %li", status_resp.actual_micrometers);
                return;
            case PHASON_ERROR:
                report_error_ln("Feed failed, unspecified reason");
                return;
            case PHASON_INVALID_REQUEST:
                report_error_ln(
                    "Feed failed, invalid request (bad sequence? have %u, got %u)",
                    feeder->sequence,
                    status_resp.sequence);
                return;
            case PHASON_MOTOR_ERROR:
                report_error_ln("Feed failed, motor error");
                return;
            case PHASON_NOT_READY:
                report_info_ln("Not ready, checking again in %lu ms...", FEED_CHECK_INTERVAL_MS);
                sleep_ms(FEED_CHECK_INTERVAL_MS);
                continue;
                break;
        }
    }
}
