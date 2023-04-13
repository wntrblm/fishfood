/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "drivers/tmc2209_helper.h"
#include "config/motion.h"
#include "hardware/gpio.h"
#include "report.h"

/* Macros and constants */

#define _INDIRECT_LOOKUP(k, n) k##_##n
#define INDIRECT_LOOKUP(k, n) _INDIRECT_LOOKUP(k, n)

/* Public methods */

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin) {
    enum TMC2209_read_result result = 0;
    uint32_t slaveconf = 0;
    uint32_t gconf = 0;
    uint32_t chopconf = 0;

    report_debug_ln("configuring TMC2209 @ %u", tmc->uart_address);

    TMC_SET_FIELD(slaveconf, TMC2209_SLAVECONF, 3);
    report_debug_ln("setting reply SENDDELAY to 0x%08lX...", slaveconf);
    TMC2209_write(tmc, TMC2209_SLAVECONF, slaveconf);

    report_debug_ln("reading initial GCONF...");

    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;

    // There's no external VREF on Candela.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG, TMC_EXTERNAL_VREF);
    // Use PDN_UART pin for UART only
    TMC_SET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE, 1);
    // Don't use the MS pins for UART address, not microstepping setting.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT, 1);
    // Don't use stealthChop by default, as it messes up high speed movement
    TMC_SET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE, 1);
    // Use external RDSon sense resistors
    TMC_SET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE, TMC_INTERNAL_RSENSE);
    // Use filtering on the step pin
    TMC_SET_FIELD(gconf, TMC2209_GCONF_MULTISTEP_FILT, 1);

    report_debug_ln("setting GCONF to 0x%08lX...", gconf);
    TMC2209_write(tmc, TMC2209_GCONF, gconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_GCONF(gconf);

    report_debug_ln("reading initial CHOPCHOF...");
    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &chopconf);
    if (result != TMC_READ_OK)
        return false;

    // Set up microstep resolution
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_MRES, INDIRECT_LOOKUP(TMC2209_CHOPCONF_MRES, TMC_MICROSTEPS));
    // Interpolate microsteps to 256
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_INTPOL, TMC_INTERPOLATION);
    // Only advance on one edge of the step pulses (datasheet 1.3.1)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_DEDGE, 0);
    // Set blank time to the datasheet-recommended 1 (24 tCLK) (datasheet 7.1)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_TBL, 1);
    // Set hysteresis parameters (taken from Marlin's 24V defaults)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_TOFF, 4);
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_HSTRT, 0);
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_HEND, 5);
    // Set Vsense
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_VSENSE, TMC_VSENSE);

    report_debug_ln("setting CHOPCONF to 0x%08lX...", chopconf);
    TMC2209_write(tmc, TMC2209_CHOPCONF, chopconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &chopconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_CHOPCONF(chopconf);

    // Disable coolstep altogether.
    uint32_t tcoolthrs = 0xFFFFF;
    report_debug_ln("setting TCOOLTHRS to 0x%04lX...", tcoolthrs);
    TMC2209_write(tmc, TMC2209_TCOOLTHRS, tcoolthrs);

    uint32_t tpowerdown = TMC2209_S_TO_TPOWERDOWN(TMC_HOLD_TIME);
    float tpowerdown_s = TMC2209_TPOWERDOWN_TO_S(tpowerdown);
    report_debug_ln("setting TPOWERDOWN to 0x%02lX (%0.2f s)...", tpowerdown, (double)tpowerdown_s);
    TMC2209_write(tmc, TMC2209_TPOWERDOWN, tpowerdown);

    report_debug_ln("setting SGTHRS to 0x%02X...", 0);
    TMC2209_write(tmc, TMC2209_SGTHRS, 0);

    report_debug_ln("enabling motor drivers...");
    gpio_put(enable_pin, 0);

    report_debug_ln("clearing status flags...");
    TMC2209_write(tmc, TMC2209_DRVSTATUS, 0b111);

    report_info_ln("TMC2209 @ %u configured", tmc->uart_address);

    return true;
}

void TMC2209_print_all(struct TMC2209* tmc) {
    report_debug_ln("TMC2209 @ %u status:", tmc->uart_address);

    enum TMC2209_read_result result;
    uint32_t value;

    result = TMC2209_read(tmc, TMC2209_GCONF, &value);
    if (result != TMC_READ_OK) {
        goto fail;
    }
    TMC2209_print_GCONF(value);

    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &value);
    if (result != TMC_READ_OK) {
        goto fail;
    }
    TMC2209_print_CHOPCONF(value);

    result = TMC2209_read(tmc, TMC2209_PWMCONF, &value);
    if (result != TMC_READ_OK) {
        goto fail;
    }
    TMC2209_print_PWMCONF(value);

    result = TMC2209_read(tmc, TMC2209_DRVSTATUS, &value);
    if (result != TMC_READ_OK) {
        goto fail;
    }
    TMC2209_print_DRVSTATUS(value);

    report_debug_ln("");

    return;

fail:
    report_error_ln("unable to communicate with TMC2209 @ %u", tmc->uart_address);
}

void TMC2209_set_current(struct TMC2209* tmc, float run_a, float hold_a) {
    uint32_t ihold_irun = 0;
    uint32_t irun = TMC2209_RMS_TO_CS(TMC_RSENSE, TMC_VSENSE, run_a);
    float irun_a = TMC2209_CS_TO_RMS(TMC_RSENSE, TMC_VSENSE, irun);
    uint32_t ihold = TMC2209_RMS_TO_CS(TMC_RSENSE, TMC_VSENSE, hold_a);
    float ihold_a = TMC2209_CS_TO_RMS(TMC_RSENSE, TMC_VSENSE, ihold);

    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IRUN, irun);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLD, ihold);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLDDELAY, 10);

    report_debug_ln(
        "setting IRUN to %lu (%0.1fA), IHOLD = %lu (%0.1fA)...", irun, (double)irun_a, ihold, (double)ihold_a);
    TMC2209_write(tmc, TMC2209_IHOLD_IRUN, ihold_irun);
}

void TMC2209_print_GCONF(uint32_t gconf) {
    report_debug_ln("GCONF.I_scale_analog:    %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG));
    report_debug_ln("GCONF.internal_Rsense:   %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE));
    report_debug_ln("GCONF.en_SpreadCycle:    %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE));
    report_debug_ln("GCONF.shaft:             %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_SHAFT));
    report_debug_ln("GCONF.index_otpw:        %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_OTPW));
    report_debug_ln("GCONF.index_step:        %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_STEP));
    report_debug_ln("GCONF.pdn_disable:       %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE));
    report_debug_ln("GCONF.mstep_reg_select:  %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT));
    report_debug_ln("GCONF.multistep_filt:    %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_MULTISTEP_FILT));
    report_debug_ln("GCONF.test_mode:         %lu", TMC_GET_FIELD(gconf, TMC2209_GCONF_TEST_MODE));
}

void TMC2209_print_CHOPCONF(uint32_t chopconf) {
    report_debug_ln("CHOPCONF.diss2vs:        %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2VS));
    report_debug_ln("CHOPCONF.diss2g:         %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2G));
    report_debug_ln("CHOPCONF.dedge:          %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DEDGE));
    report_debug_ln("CHOPCONF.intpol:         %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_INTPOL));
    report_debug_ln("CHOPCONF.MRES:           %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_MRES));
    report_debug_ln("CHOPCONF.vsense:         %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_VSENSE));
    report_debug_ln("CHOPCONF.TBL:            %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TBL));
    report_debug_ln("CHOPCONF.HEND:           %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HEND));
    report_debug_ln("CHOPCONF.HSTRT:          %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HSTRT));
    report_debug_ln("CHOPCONF.TOFF:           %lu", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TOFF));
}

void TMC2209_print_PWMCONF(uint32_t pwmconf) {
    report_debug_ln("PWMCONF.PWM_LIM:         %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_LIM));
    report_debug_ln("PWMCONF.PWM_REG:         %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_REG));
    report_debug_ln("PWMCONF.freewheel:       %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_FREEWHEEL));
    report_debug_ln("PWMCONF.pwm_autograd:    %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOGRAD));
    report_debug_ln("PWMCONF.pwm_autoscale:   %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOSCALE));
    report_debug_ln("PWMCONF.pwm_freq:        %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_FREQ));
    report_debug_ln("PWMCONF.PWM_GRAD:        %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_GRAD));
    report_debug_ln("PWMCONF.PWM_OFS:         %lu", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_OFS));
}

void TMC2209_print_DRVSTATUS(uint32_t drvstatus) {
    report_debug_ln("DRVSTATUS.otpw:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OTPW));
    report_debug_ln("DRVSTATUS.ot:            %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OT));
    report_debug_ln("DRVSTATUS.s2ga:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GA));
    report_debug_ln("DRVSTATUS.s2gb:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GB));
    report_debug_ln("DRVSTATUS.s2vsa:         %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSA));
    report_debug_ln("DRVSTATUS.s2vsb:         %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSB));
    report_debug_ln("DRVSTATUS.ola:           %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLA));
    report_debug_ln("DRVSTATUS.olb:           %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLB));
    report_debug_ln("DRVSTATUS.t120:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T120));
    report_debug_ln("DRVSTATUS.t143:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T143));
    report_debug_ln("DRVSTATUS.t150:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T150));
    report_debug_ln("DRVSTATUS.t157:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T157));
    report_debug_ln("DRVSTATUS.CS_ACTUAL:     %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_CS_ACTUAL));
    report_debug_ln("DRVSTATUS.stealth:       %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STEALTH));
    report_debug_ln("DRVSTATUS.stst:          %lu", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STST));
}
