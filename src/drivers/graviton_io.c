/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "graviton_io.h"
#include "report.h"
#include "rs485.h"

/*
    Forward declarations
*/
static int32_t graviton_io_read_impl(struct GravitonIO* io);
static int32_t graviton_io_write_impl(struct GravitonIO* io, uint8_t* data, size_t len);

/*
    Public functions
*/

void GravitonIODriver_init(
    struct GravitonIODriver* io_driver, uint32_t first_byte_timeout_ms, uint32_t total_timeout_ms) {
    io_driver->context.first_byte_timeout_ms = first_byte_timeout_ms;
    io_driver->context.total_timeout_ms = total_timeout_ms;
    io_driver->context.first_byte_deadline = nil_time;
    io_driver->context.total_deadline = nil_time;
    io_driver->context.seen_first_byte = false;

    io_driver->io.read = graviton_io_read_impl;
    io_driver->io.write = graviton_io_write_impl;
    io_driver->io.context = &io_driver->context;
}

/*
    Private functions
*/

int32_t graviton_io_read_impl(struct GravitonIO* io) {
    struct GravitonIODriverContext* ctx = (struct GravitonIODriverContext*)io->context;

    // The deadline is unset first time this is called for a transaction,
    // this allows setting the deadline at the proper absolute time.
    if (is_nil_time(ctx->first_byte_deadline)) {
        ctx->first_byte_deadline = make_timeout_time_ms(ctx->first_byte_timeout_ms);
    }
    if (is_nil_time(ctx->total_deadline)) {
        ctx->total_deadline = make_timeout_time_ms(ctx->total_timeout_ms);
    }

    if ((!ctx->seen_first_byte && time_reached(ctx->first_byte_deadline)) || time_reached(ctx->total_deadline)) {
        report_debug_ln("Graviton bus timed out");
        return GRAVITON_IO_ABORT;
    }

    int32_t result = rs485_read();

    ctx->seen_first_byte |= (result == 0x55);

    return result;
}

int32_t graviton_io_write_impl(struct GravitonIO* io, uint8_t* data, size_t len) {
    (void)io;
    rs485_write(data, len);
    return 1;
}
