#include <stdio.h>
#include <string.h>
#include "tmc2209.h"

inline static void _print_array(uint8_t* addr, size_t len) {
    for (size_t i = 0; i < len; i += 16) {
        for (size_t j = 0; j < 16 && i + j < len; j++) { printf("%02X ", addr[i + j]); }
        printf("\n");
    }
}

static uint8_t test_tmc_reply[8];

static void tmc_uart_read_write(struct TMC2209* tmc, uint8_t* send_buf, size_t send_len, uint8_t* receive_buf, size_t receive_len) {
    printf("TMC send: ");
    _print_array(send_buf, send_len);

    if(receive_len > 0) {
        memcpy(receive_buf, test_tmc_reply, receive_len);
    }
}

int main(int argc, char **argv) {
    uint32_t gconf = 0x00000040u;
    TMC2209_print_GCONF(gconf);

    uint32_t chopconf = 0x10000053u;
    TMC2209_print_CHOPCONF(chopconf);

    uint32_t pwmconf = 0xC10D0024u;
    TMC2209_print_PWMCONF(pwmconf);

    struct TMC2209 tmc_0 = {};
    TMC2209_init(&tmc_0, NULL, 0, tmc_uart_read_write);

    TMC2209_write(&tmc_0, 2, 0x00011405);

    test_tmc_reply[0] = 0x05;
    test_tmc_reply[1] = 0xFF;
    test_tmc_reply[2] = 0x02;
    test_tmc_reply[3] = 0x00;
    test_tmc_reply[4] = 0x01;
    test_tmc_reply[5] = 0x14;
    test_tmc_reply[6] = 0x05;
    test_tmc_reply[7] = TMC2209_CRC8(test_tmc_reply, 7);

    uint32_t out;
    enum TMC2209_read_result result = TMC2209_read(&tmc_0, 0x02, &out);

    printf("Read result: %i, value: 0x%08X\n", result, out);

    // uint8_t datagram[] = {0x05, 0x81, 0x02};
    // uint8_t crc = TMC2209_crc8(datagram, 3);
    // printf("CRC: %02X\n", crc);
}

/*
  bool tmc_enable_stallguard(TMC2209Stepper &st) {
    const bool stealthchop_was_enabled = !st.en_spreadCycle();

    st.TCOOLTHRS(0xFFFFF);
    st.en_spreadCycle(false);
    return stealthchop_was_enabled;
  }
  void tmc_disable_stallguard(TMC2209Stepper &st, const bool restore_stealth) {
    st.en_spreadCycle(!restore_stealth);
    st.TCOOLTHRS(0);
  }
*/

/*
#define CHOPPER_DEFAULT_24V  { 4,  2, 1 }
*/

/*
void TMC2209Stepper::push() {
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	TPOWERDOWN(TPOWERDOWN_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
	GCONF(GCONF_register.sr);
	SLAVECONF(SLAVECONF_register.sr);
	VACTUAL(VACTUAL_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
	TCOOLTHRS(TCOOLTHRS_register.sr);
	SGTHRS(SGTHRS_register.sr);
	COOLCONF(COOLCONF_register.sr);
}

void TMC2208Stepper::defaults() {
	GCONF_register.i_scale_analog = 1;
	GCONF_register.internal_rsense = 0; // OTP
	GCONF_register.en_spreadcycle = 0; // OTP
	GCONF_register.multistep_filt = 1; // OTP
	IHOLD_IRUN_register.iholddelay = 1; // OTP
	TPOWERDOWN_register.sr = 20;
	CHOPCONF_register.sr = 0x10000053;
	PWMCONF_register.sr = 0xC10D0024;
}

int16_t homing_threshold() { return TMC2209Stepper::SGTHRS(); }
void homing_threshold(int16_t sgt_val) {
    sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
    TMC2209Stepper::SGTHRS(sgt_val);
    TERN_(HAS_MARLINUI_MENU, this->stored.homing_thrs = sgt_val);
}

  template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
  void tmc_init(
      // STEPPER_ST
      TMCMarlin<TMC2209Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st,
      // ST_CURRENT
      const uint16_t mA,
      // ST_MICROSTEPS
      const uint16_t microsteps,
      // ST_HYBRID_THRESHOLD
      const uint32_t hyb_thrs,
      // stealthchop_by_axis[STEALTH_INDEX]
      const bool stealth,
      // chopper_timing_ST
      const chopper_timing_t &chop_init,
      // ST_INTERPOLATE
      const bool interpolate,
      // ST_HOLD_MULTIPLIER
      float hold_multiplier) {
    TMC2208_n::GCONF_t gconf{0};
    gconf.pdn_disable = true; // Use UART
    gconf.mstep_reg_select = true; // Select microsteps with UART
    gconf.i_scale_analog = false;
    gconf.en_spreadcycle = !stealth;
    st.GCONF(gconf.sr);
    st.stored.stealthChop_enabled = stealth;

    TMC2208_n::CHOPCONF_t chopconf{0};
    chopconf.tbl = 0b01; // blank_time = 24
    // { toff=4,  hend=2+3, hstrt=1-1 }
    chopconf.toff = chop_init.toff;
    chopconf.intpol = interpolate;
    chopconf.hend = chop_init.hend + 3;
    chopconf.hstrt = chop_init.hstrt - 1;
    TERN_(SQUARE_WAVE_STEPPING, chopconf.dedge = true);
    st.CHOPCONF(chopconf.sr);

    st.rms_current(mA, hold_multiplier);
    st.microsteps(microsteps);
    st.iholddelay(10);
    st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

    TMC2208_n::PWMCONF_t pwmconf{0};
    pwmconf.pwm_lim = 12;
    pwmconf.pwm_reg = 8;
    pwmconf.pwm_autograd = true;
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_freq = 0b01;
    pwmconf.pwm_grad = 14;
    pwmconf.pwm_ofs = 36;
    st.PWMCONF(pwmconf.sr);

    TERN(HYBRID_THRESHOLD, st.set_pwm_thrs(hyb_thrs), UNUSED(hyb_thrs));

    st.GSTAT(0b111); // Clear
    delay(200);
  }
*/
