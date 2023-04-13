#include "stepper.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "report.h"

// TMC2209 Datasheet section 13.1 notes timing requirements:
// - T(DSU) - DIR to STEP setup time = 20 ns
// - T(SH) - STEP minimum high time = 100 ns
// - T(SL) - STEP minimum low time = 100 ns
// We add a little bit just to be on the safe side.
#define TIMING_T_DSU_NS 80
#define TIMING_T_SH_NS 150
#define TIMING_T_SL_NS 150
// Running at 125 MHz, each clock cycle is 8 ns.
#define TIMING_CYCLE_NS 8
#define NS_TO_CYCLES(n) ((uint32_t)(n / TIMING_CYCLE_NS))

// Calculate delays based on timing information above
#define DIR_SETUP_DELAY_CYCLES NS_TO_CYCLES(TIMING_T_DSU_NS)
#define STEP_HIGH_DELAY_CYCLES NS_TO_CYCLES(TIMING_T_SH_NS)
#define STEP_LOW_DELAY_CYCLES NS_TO_CYCLES(TIMING_T_SL_NS)

/*
    Public functions
*/

void Stepper_init(
    struct Stepper* s,
    struct TMC2209* tmc,
    uint8_t pin_enn,
    uint8_t pin_dir,
    uint8_t pin_step,
    uint8_t pin_diag,
    bool reversed,
    float run_current,
    float hold_current) {
    s->tmc = tmc;
    s->pin_enn = pin_enn;
    s->pin_dir = pin_dir;
    s->pin_step = pin_step;
    s->pin_diag = pin_diag;
    s->reversed = reversed;
    s->direction = 1;
    s->run_current = run_current;
    s->hold_current = hold_current;

    s->total_steps = 0;
}

bool Stepper_setup(struct Stepper* s) {
    gpio_init(s->pin_enn);
    gpio_set_dir(s->pin_enn, GPIO_OUT);
    gpio_put(s->pin_enn, true);

    gpio_init(s->pin_dir);
    gpio_set_dir(s->pin_dir, GPIO_OUT);
    gpio_put(s->pin_dir, s->direction > 0 ? !s->reversed : s->reversed);

    gpio_init(s->pin_step);
    gpio_set_dir(s->pin_step, GPIO_OUT);
    gpio_put(s->pin_step, false);

    gpio_init(s->pin_diag);
    gpio_set_dir(s->pin_diag, GPIO_IN);
    gpio_pull_down(s->pin_diag);

    if (!TMC2209_write_config(s->tmc, s->pin_enn)) {
        report_error_ln("error configuring TMC2209 @ %u", s->tmc->uart_address);
        return false;
    }

    Stepper_set_current(s, s->run_current, s->hold_current);

    return true;
}

void Stepper_disable(struct Stepper* s) { gpio_put(s->pin_enn, 1); }
void Stepper_enable(struct Stepper* s) { gpio_put(s->pin_enn, 0); }

void Stepper_set_current(struct Stepper* s, float run_current, float hold_current) {
    TMC2209_set_current(s->tmc, run_current, hold_current);
    s->run_current = run_current;
    s->hold_current = hold_current;
}

void Stepper_enable_stallguard(struct Stepper* s, uint8_t threshold) {
    TMC2209_write(s->tmc, TMC2209_SGTHRS, threshold);
}

void Stepper_disable_stallguard(struct Stepper* s) { TMC2209_write(s->tmc, TMC2209_SGTHRS, 0); }

static inline void set_stealthchop(struct Stepper* s, bool enabled) {
    uint32_t gconf;
    enum TMC2209_read_result result = TMC2209_read(s->tmc, TMC2209_GCONF, &gconf);

    if (result != TMC_READ_OK) {
        report_error_ln("Unable to read GCONF!");
        return;
    }

    TMC_SET_FIELD(gconf, TMC2209_GCONF_EN_SPREADCYCLE, !enabled);

    TMC2209_write(s->tmc, TMC2209_GCONF, gconf);
}

void Stepper_enable_stealthchop(struct Stepper* s) { set_stealthchop(s, true); }
void Stepper_disable_stealthchop(struct Stepper* s) { set_stealthchop(s, false); }

bool Stepper_stalled(struct Stepper* s) {
    // Note: this works well because Fishfood doesn't do stepping using an
    // interrupt, so there's no need to asynchronously handle this. If,
    // in the future, this assumption changes this needs to use the GPIO
    // IRQ.
    return gpio_get(s->pin_diag);
}

void Stepper_update_direction(struct Stepper* s) {
    gpio_put(s->pin_dir, s->direction > 0 ? !s->reversed : s->reversed);
    busy_wait_at_least_cycles(DIR_SETUP_DELAY_CYCLES);
}

void Stepper_step(struct Stepper* s) {
    gpio_put(s->pin_step, true);
    busy_wait_at_least_cycles(STEP_HIGH_DELAY_CYCLES);
    gpio_put(s->pin_step, false);
    busy_wait_at_least_cycles(STEP_LOW_DELAY_CYCLES);

    s->total_steps += s->direction;
}

void Stepper_step_two(struct Stepper* s1, struct Stepper* s2) {
    gpio_put(s1->pin_step, true);
    gpio_put(s2->pin_step, true);

    busy_wait_at_least_cycles(STEP_HIGH_DELAY_CYCLES);
    gpio_put(s1->pin_step, false);
    gpio_put(s2->pin_step, false);
    busy_wait_at_least_cycles(STEP_LOW_DELAY_CYCLES);

    s1->total_steps += s1->direction;
    s2->total_steps += s2->direction;
}
