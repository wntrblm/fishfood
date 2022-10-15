#include "xgzp6857d.h"
#include "pico/time.h"
#include "report.h"

#define XGZP6857D_ADDR 0x6D

int32_t XGZP6857D_read(i2c_inst_t* i2c, uint timeout_us) {
    // Configure the measurement parameters.
    uint8_t buf[2] = {0x30, 0x0A};
    int32_t result = i2c_write_timeout_us(i2c, XGZP6857D_ADDR, buf, 2, false, timeout_us);

    if (result < 0) {
        report_error_ln("failed to setup XGZP6857D measurement error %li", result);
        return -1;
    }

    // Wait a bit... 20ms according to datasheet. Alternatively, readback
    // the 0x30 register and check for bit 3 to be clear.
    sleep_ms(25);

    // Read each byte needed to form the 24 bit pressure value.
    uint32_t pressure = 0;

    buf[0] = 0x06;
    i2c_write_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    i2c_read_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    pressure = buf[0] << 16;

    buf[0] = 0x07;
    i2c_write_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    i2c_read_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    pressure = pressure | (buf[0] << 8);

    buf[0] = 0x08;
    i2c_write_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    i2c_read_timeout_us(i2c, XGZP6857D_ADDR, buf, 1, false, timeout_us);
    pressure = pressure | buf[0];

    return pressure;
}
