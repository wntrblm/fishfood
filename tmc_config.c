#include "tmc_config.h"
#include "hardware/gpio.h"
#include <stdio.h>

bool tmc_config(struct TMC2209* tmc, uint32_t enable_pin) {
    enum TMC2209_read_result result;
    uint32_t gconf;
    uint32_t chopconf;

    printf("Configuring TMC2209 @ %u\n", tmc->uart_address);

    printf("- Setting GCONF...\n");
    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;

    // There's no external VREF on Candela.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_I_SCALE_ANALOG, 0);
    // Use PDN_UART pin for UART only
    TMC_SET_FIELD(gconf, TMC2209_GCONF_PDN_DISABLE, 1);
    // Don't use the MS pins for UART address, not microstepping setting.
    TMC_SET_FIELD(gconf, TMC2209_GCONF_MSTEP_REG_SELECT, 1);
    // Use stealthChop
    TMC_SET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE, 0);
    // Use external RDSon sense resistors
    TMC_SET_FIELD(gconf, TMC2209_GCONF_INTERNAL_RSENSE, 0);

    TMC2209_write(tmc, TMC2209_GCONF, gconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_GCONF, &gconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_GCONF(gconf);

    printf("- Setting CHOPCONF...\n");
    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &chopconf);
    if (result != TMC_READ_OK)
        return false;

    // Use 32 microsteps
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_MRES, TMC2209_CHOPCONF_MRES_32);
    // Interpolate to 256
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_INTPOL, 1);
    // Advance on both rising and falling edge of step pulses (datasheet 1.3.1)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_DEDGE, 0);
    // Set blank time to the datasheet-recommended 1 (24 tCLK) (datasheet 7.1)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_TBL, 1);
    // Set hysteresis parameters (taken from Marlin's 24V defaults)
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_TOFF, 4);
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_HSTRT, 0);
    TMC_SET_FIELD(chopconf, TMC2209_CHOPCONF_HEND, 5);

    TMC2209_write(tmc, TMC2209_CHOPCONF, chopconf);

    // Readback
    result = TMC2209_read(tmc, TMC2209_CHOPCONF, &chopconf);
    if (result != TMC_READ_OK)
        return false;
    TMC2209_print_CHOPCONF(chopconf);

    printf("- Setting TCOOLTHRS...\n");
    // Disable coolstep altogether.
    TMC2209_write(tmc, TMC2209_TCOOLTHRS, 0xFFFF);

    printf("- Setting IHOLD_IRUN... \n");
    uint32_t ihold_irun = 0;
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IRUN, 31);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLD, 25);
    TMC_SET_FIELD(ihold_irun, TMC2209_IHOLD_IRUN_IHOLDDELAY, 10);
    TMC2209_write(tmc, TMC2209_IHOLD_IRUN, ihold_irun);

    printf("- Setting up stallguard... \n");
    TMC2209_write(tmc, TMC2209_SGTHRS, 50);

    printf("- Enabling stepper... \n");
    gpio_put(enable_pin, 0);

    printf("- Clearing status flags... \n");
    TMC2209_write(tmc, TMC2209_DRVSTATUS, 0b111);

    printf("TMC2209 configured!\n\n");
}
