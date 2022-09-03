#include "drivers/tmc2209_helper.h"
#include "config/motion.h"
#include "hardware/gpio.h"
#include <stdio.h>

/* Macros and constants */

#define _INDIRECT_LOOKUP(k, n) k##_##n
#define INDIRECT_LOOKUP(k, n) _INDIRECT_LOOKUP(k, n)

/* Public methods */

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin) {
    enum TMC2209_read_result result;
    uint32_t slaveconf;
    uint32_t gconf;
    uint32_t chopconf;

    printf("Configuring TMC2209 @ %u\n", tmc->uart_address);

    TMC_SET_FIELD(slaveconf, TMC2209_SLAVECONF, 3);
    printf("- Setting reply SENDDELAY to 0x%08x...\n", slaveconf);
    TMC2209_write(tmc, TMC2209_SLAVECONF, slaveconf);

    printf("- Reading initial GCONF...\n");

    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;

    // There's no external VREF on Candela.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG, TMC_EXTERNAL_VREF);
    // Use PDN_UART pin for UART only
    TMC_SET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE, 1);
    // Don't use the MS pins for UART address, not microstepping setting.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT, 1);
    // Use stealthChop
    TMC_SET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE, 0);
    // Use external RDSon sense resistors
    TMC_SET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE, TMC_INTERNAL_RSENSE);
    // Use filtering on the step pin
    TMC_SET_FIELD(gconf, TMC2209_GCONF_MULTISTEP_FILT, 1);

    printf("- Setting GCONF to 0x%08X ...\n", gconf);
    TMC2209_write(tmc, TMC2209_GCONF, gconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_GCONF(gconf);

    printf("- Reading initial CHOPCHOF...\n");
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

    printf("- Setting CHOPCONF to 0x%08X ...\n", chopconf);
    TMC2209_write(tmc, TMC2209_CHOPCONF, chopconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &chopconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_CHOPCONF(chopconf);

    // Disable coolstep altogether.
    uint32_t tcoolthrs = 0xFFFFF;
    printf("- Setting TCOOLTHRS to 0x%04X ...\n", tcoolthrs);
    TMC2209_write(tmc, TMC2209_TCOOLTHRS, tcoolthrs);

    uint32_t tpowerdown = TMC2209_S_TO_TPOWERDOWN(TMC_HOLD_TIME);
    float tpowerdown_s = TMC2209_TPOWERDOWN_TO_S(tpowerdown);
    printf("- Setting TPOWERDOWN to 0x%02X (%0.2f s)...\n", tpowerdown, tpowerdown_s);
    TMC2209_write(tmc, TMC2209_TPOWERDOWN, tpowerdown);

    printf("- Setting SGTHRS to 0x%02X ...\n", 0);
    TMC2209_write(tmc, TMC2209_SGTHRS, 0);

    printf("- Enabling stepper... \n");
    gpio_put(enable_pin, 0);

    printf("- Clearing status flags... \n");
    TMC2209_write(tmc, TMC2209_DRVSTATUS, 0b111);

    printf("TMC2209 configured!\n\n");
}

void TMC2209_print_all(struct TMC2209* tmc) {
    printf("> TMC2209 @ %u status:\n\n", tmc->uart_address);

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

    printf("\n");

    return;

fail:
    printf("Unable to communicate with TMC2209!\n");
}

bool TMC2209_set_current(struct TMC2209* tmc, float run_a, float hold_a) {
    uint32_t ihold_irun = 0;
    uint32_t irun = TMC2209_RMS_TO_CS(TMC_RSENSE, TMC_VSENSE, run_a);
    float irun_a = TMC2209_CS_TO_RMS(TMC_RSENSE, TMC_VSENSE, irun);
    uint32_t ihold = TMC2209_RMS_TO_CS(TMC_RSENSE, TMC_VSENSE, hold_a);
    float ihold_a = TMC2209_CS_TO_RMS(TMC_RSENSE, TMC_VSENSE, ihold);

    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IRUN, irun);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLD, ihold);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLDDELAY, 10);

    printf("- Setting IRUN to %u (%0.1fA), IHOLD = %u (%0.1fA)...\n", irun, irun_a, ihold, ihold_a);
    TMC2209_write(tmc, TMC2209_IHOLD_IRUN, ihold_irun);
}

void TMC2209_print_GCONF(uint32_t gconf) {
    printf("GCONF.I_scale_analog:    %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG));
    printf("GCONF.internal_Rsense:   %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE));
    printf("GCONF.en_SpreadCycle:    %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE));
    printf("GCONF.shaft:             %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_SHAFT));
    printf("GCONF.index_otpw:        %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_OTPW));
    printf("GCONF.index_step:        %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_INDEX_STEP));
    printf("GCONF.pdn_disable:       %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE));
    printf("GCONF.mstep_reg_select:  %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT));
    printf("GCONF.multistep_filt:    %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_MULTISTEP_FILT));
    printf("GCONF.test_mode:         %u\n", TMC_GET_FIELD(gconf, TMC2209_GCONF_TEST_MODE));
}

void TMC2209_print_CHOPCONF(uint32_t chopconf) {
    printf("CHOPCONF.diss2vs:        %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2VS));
    printf("CHOPCONF.diss2g:         %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DISS2G));
    printf("CHOPCONF.dedge:          %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_DEDGE));
    printf("CHOPCONF.intpol:         %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_INTPOL));
    printf("CHOPCONF.MRES:           %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_MRES));
    printf("CHOPCONF.vsense:         %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_VSENSE));
    printf("CHOPCONF.TBL:            %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TBL));
    printf("CHOPCONF.HEND:           %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HEND));
    printf("CHOPCONF.HSTRT:          %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_HSTRT));
    printf("CHOPCONF.TOFF:           %u\n", TMC_GET_FIELD(chopconf, TMC2209_CHOPCONF_TOFF));
}

void TMC2209_print_PWMCONF(uint32_t pwmconf) {
    printf("PWMCONF.PWM_LIM:         %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_LIM));
    printf("PWMCONF.PWM_REG:         %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_REG));
    printf("PWMCONF.freewheel:       %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_FREEWHEEL));
    printf("PWMCONF.pwm_autograd:    %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOGRAD));
    printf("PWMCONF.pwm_autoscale:   %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_AUTOSCALE));
    printf("PWMCONF.pwm_freq:        %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_FREQ));
    printf("PWMCONF.PWM_GRAD:        %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_GRAD));
    printf("PWMCONF.PWM_OFS:         %u\n", TMC_GET_FIELD(pwmconf, TMC2209_PWMCONF_PWM_OFS));
}

void TMC2209_print_DRVSTATUS(uint32_t drvstatus) {
    printf("DRVSTATUS.otpw:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OTPW));
    printf("DRVSTATUS.ot:            %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OT));
    printf("DRVSTATUS.s2ga:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GA));
    printf("DRVSTATUS.s2gb:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2GB));
    printf("DRVSTATUS.s2vsa:         %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSA));
    printf("DRVSTATUS.s2vsb:         %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_S2VSB));
    printf("DRVSTATUS.ola:           %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLA));
    printf("DRVSTATUS.olb:           %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_OLB));
    printf("DRVSTATUS.t120:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T120));
    printf("DRVSTATUS.t143:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T143));
    printf("DRVSTATUS.t150:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T150));
    printf("DRVSTATUS.t157:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_T157));
    printf("DRVSTATUS.CS_ACTUAL:     %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_CS_ACTUAL));
    printf("DRVSTATUS.stealth:       %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STEALTH));
    printf("DRVSTATUS.stst:          %u\n", TMC_GET_FIELD(drvstatus, TMC2209_DRVSTATUS_STST));
}
