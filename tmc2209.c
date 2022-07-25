#include "tmc2209.h"
#include <stdio.h>

/* Contants and macros */

#define TMC_SYNC_BTYE 0x05
#define TMC_WRITE_ADDR 0x80

/* Private data and methods */

/* Public methods */

void TMC2209_init(
    struct TMC2209* tmc, void* uart, uint8_t uart_address, TMC2209_uart_send_receive_func uart_send_receive) {
    tmc->uart = uart;
    tmc->uart_address = uart_address;
    tmc->uart_send_receive = uart_send_receive;
}

enum TMC2209_read_result TMC2209_read(struct TMC2209* tmc, uint8_t register_addr, uint32_t* out) {
    uint8_t read_datagram[] = {TMC_SYNC_BTYE, tmc->uart_address, register_addr, 0};
    read_datagram[3] = TMC2209_CRC8(read_datagram, 3);

    uint8_t reply_datagram[8] = {};

    tmc->uart_send_receive(tmc, read_datagram, 4, reply_datagram, 8);

    if (reply_datagram[0] != 0x05) {
        return TMC_READ_BAD_SYNC;
    }
    if (reply_datagram[1] != 0xFF) {
        return TMC_READ_BAD_ADDRESS;
    }
    if (reply_datagram[2] != register_addr) {
        return TMC_READ_BAD_REGISTER;
    }
    if (reply_datagram[7] != TMC2209_CRC8(reply_datagram, 7)) {
        return TMC_READ_BAD_CRC;
    }

    (*out) = ((uint32_t)(reply_datagram[3]) << 24) | ((uint32_t)(reply_datagram[4]) << 16) |
             ((uint32_t)(reply_datagram[5]) << 8) | (uint32_t)(reply_datagram[6]);

    return TMC_READ_OK;
}

void TMC2209_write(struct TMC2209* tmc, uint8_t register_addr, uint32_t value) {
    uint8_t datagram[8] = {
        TMC_SYNC_BTYE,
        tmc->uart_address,
        register_addr | TMC_WRITE_ADDR,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value,
        0};

    datagram[7] = TMC2209_CRC8(datagram, 7);

    tmc->uart_send_receive(tmc, datagram, 8, NULL, 0);
}

uint8_t TMC2209_CRC8(uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t byte_n = 0; byte_n < len; byte_n++) {
        uint8_t byte = data[byte_n];
        for (size_t bit_n = 0; bit_n < 8; bit_n++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

void TMC2209_print_GCONF(uint32_t gconf) {
    printf("GCONF.I_scale_analog:       %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG));
    printf("GCONF.internal_Rsense:      %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE));
    printf("GCONF.en_SpreadCycle:       %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE));
    printf("GCONF.shaft:                %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_SHAFT));
    printf("GCONF.index_otpw:           %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_OTPW));
    printf("GCONF.index_step:           %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_STEP));
    printf("GCONF.pdn_disable:          %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE));
    printf("GCONF.mstep_reg_select:     %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT));
    printf("GCONF.multistep_filt:       %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_MULTISTEP_FILT));
    printf("GCONF.test_mode:            %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_TEST_MODE));
}

void TMC2209_print_CHOPCONF(uint32_t chopconf) {
    printf("CHOPCONF.diss2vs:           %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2VS));
    printf("CHOPCONF.diss2g:            %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2G));
    printf("CHOPCONF.dedge:             %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DEDGE));
    printf("CHOPCONF.intpol:            %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_INTPOL));
    printf("CHOPCONF.MRES:              %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_MRES));
    printf("CHOPCONF.vsense:            %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_VSENSE));
    printf("CHOPCONF.TBL:               %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TBL));
    printf("CHOPCONF.HEND:              %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HEND));
    printf("CHOPCONF.HSTRT:             %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HSTRT));
    printf("CHOPCONF.TOFF:              %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TOFF));
}

void TMC2209_print_PWMCONF(uint32_t pwmconf) {
    printf("PWMCONF.PWM_LIM:            %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_LIM));
    printf("PWMCONF.PWM_REG:            %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_REG));
    printf("PWMCONF.freewheel:          %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_FREEWHEEL));
    printf("PWMCONF.pwm_autograd:       %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOGRAD));
    printf("PWMCONF.pwm_autoscale:      %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOSCALE));
    printf("PWMCONF.pwm_freq:           %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_FREQ));
    printf("PWMCONF.PWM_GRAD:           %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_GRAD));
    printf("PWMCONF.PWM_OFS:            %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_OFS));
}

void TMC2209_print_DRVSTATUS(uint32_t drvstatus) {
    printf("DRVSTATUS.otpw:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OTPW));
    printf("DRVSTATUS.ot:               %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OT));
    printf("DRVSTATUS.s2ga:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GA));
    printf("DRVSTATUS.s2gb:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GB));
    printf("DRVSTATUS.s2vsa:            %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSA));
    printf("DRVSTATUS.s2vsb:            %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSB));
    printf("DRVSTATUS.ola:              %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLA));
    printf("DRVSTATUS.olb:              %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLB));
    printf("DRVSTATUS.t120:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T120));
    printf("DRVSTATUS.t143:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T143));
    printf("DRVSTATUS.t150:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T150));
    printf("DRVSTATUS.t157:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T157));
    printf("DRVSTATUS.CS_ACTUAL:        %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_CS_ACTUAL));
    printf("DRVSTATUS.stealth:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STEALTH));
    printf("DRVSTATUS.stst:             %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STST));
}
