#pragma once

/*  Field get/set helper macros */
#define TMC_GET_FIELD(data, field) (((data) & (field ## _MASK)) >> (field ## _SHIFT))
#define TMC_SET_FIELD(data, field, value) data = (((data) & (~(field ## _MASK))) | (((value) << (field ## _SHIFT)) & (field ## _MASK)))

/*  Datasheet section 5.1: General Registers */

/*  CGONF (RW)
    Global configuration flags */
#define TMC2209_GCONF 0x00u
/*  CGONF.I_scale_analog (reset=1)
    0: Use internal reference derived from 5VOUT
    1: Use voltage supplied to VREF as current reference */
#define TMC2209_GCONF_I_SCALE_ANALOG_MASK          0x01u
#define TMC2209_GCONF_I_SCALE_ANALOG_SHIFT         0u
/*  GCONF.internal_Rsense (reset=OTP)
    0:  Operation with external sense resistors
    1:  Internal sense resistors. Use current supplied into
        VREF as reference for internal sense resistor. VREF
        pin internally is driven to GND in this mode. */
#define TMC2209_GCONF_INTERNAL_RSENSE_MASK         0x02u
#define TMC2209_GCONF_INTERNAL_RSENSE_SHIFT        1u
/*  GCONF.en_SpreadCycle (reset=OTP)
    0:  StealthChop PWM mode enabled (depending on
        velocity thresholds). Initially switch from off to
        on state while in stand still, only.
    1:  SpreadCycle mode enabled.

    A high level on the pin SPREAD inverts this flag to
    switch between both chopper modes. */
#define TMC2209_GCONF_EN_SPREADCYCLE_MASK          0x04u
#define TMC2209_GCONF_EN_SPREADCYCLE_SHIFT         2
/*  GCONF.shaft
    1:  Inverse motor direction. */
#define TMC2209_GCONF_SHAFT_MASK                   0x08u
#define TMC2209_GCONF_SHAFT_SHIFT                  3u
/*  GCONF.index_otpw:
    0:  INDEX shows the first microstep position of
        sequencer.
    1:  INDEX pin outputs overtemperature prewarning
        flag (otpw) instead */
#define TMC2209_GCONF_INDEX_OTPW_MASK              0x10u
#define TMC2209_GCONF_INDEX_OTPW_SHIFT             4u
/*  GCONF.index_step:
    0:  INDEX output as selected by index_otpw.
    1:  INDEX output shows step pulses from internal
        pulse generator (toggle upon each step). */
#define TMC2209_GCONF_INDEX_STEP_MASK              0x20u
#define TMC2209_GCONF_INDEX_STEP_SHIFT             5u
/*  GCONF.pdn_disable:
    0:  PDN_UART controls standstill current reduction
    1:  PDN_UART input function disabled. Set this bit,
        when using the UART interface! */
#define TMC2209_GCONF_PDN_DISABLE_MASK             0x40u
#define TMC2209_GCONF_PDN_DISABLE_SHIFT            6u
/*  GCONF.mstep_reg_select:
    0:  Microstep resolution selected by pins MS1, MS2
    1:  Microstep resolution selected by MRES register */
#define TMC2209_GCONF_MSTEP_REG_SELECT_MASK        0x80u
#define TMC2209_GCONF_MSTEP_REG_SELECT_SHIFT       7u
/*  GCONF.multistep_filt (reset=1)
    0:  No filtering of STEP pulses
    1:  Software pulse generator optimization enabled
        when fullstep frequency > 750Hz (roughly). TSTEP
        shows filtered step time values when active. */
#define TMC2209_GCONF_MULTISTEP_FILT_MASK          0x0100u
#define TMC2209_GCONF_MULTISTEP_FILT_SHIFT         8u
/*  GCONF.test_mode
    0:  Normal operation
    1:  Enable analog test output on pin ENN (pull down
        resistor off), ENN treated as enabled.
        IHOLD[1..0] selects the function of DCO:
        0…2: T120, DAC, VDDH

    Attention: Not for user, set to 0 for normal operation! */
#define TMC2209_GCONF_TEST_MODE_MASK               0x0200u
#define TMC2209_GCONF_TEST_MODE_SHIFT              9u

/*  GSTAT (RW)
    Global status flags, write to clear flags */
#define TMC2209_GSTAT                              0x01u
/*  GSTAT.reset
    1:  Indicates that the IC has been reset since the last
        read access to GSTAT. All registers have been
        cleared to reset values. */
#define TMC2209_GSTAT_RESET_MASK                   0x01u
#define TMC2209_GSTAT_RESET_SHIFT                  0u
/*  GSTAT.drv_err
    1:  Indicates that the driver has been shut down
        due to overtemperature or short circuit detection
        since the last read access. Read DRV_STATUS for
        details. The flag can only be cleared when all
        error conditions are cleared. */
#define TMC2209_GSTAT_DRV_ERR_MASK                 0x02u
#define TMC2209_GSTAT_DRV_ERR_SHIFT                1u
/*  GSTAT.uv_cp
    1:  Indicates an undervoltage on the charge pump.
        The driver is disabled in this case. This flag is not
        latched and thus does not need to be cleared. */
#define TMC2209_GSTAT_UV_CP_MASK                   0x04u
#define TMC2209_GSTAT_UV_CP_SHIFT                  2u

/*  IFCNT (R)
    Interface transmission counter. This register becomes
    incremented with each successful UART interface write
    access. Read out to check the serial transmission for
    lost data. Read accesses do not change the content.
    The counter wraps around from 255 to 0. */
#define TMC2209_IFCNT                        0x02u
#define TMC2209_IFCNT_MASK                   0xFFu
#define TMC2209_IFCNT_SHIFT                  0u

/*  SLAVECONF (W)
    SENDDELAY for read access (time until reply is sent):
    0, 1: 8 bit times (Attention: Don't use in multi-slave)
    2, 3: 3*8 bit times
    4, 5: 5*8 bit times
    6, 7: 7*8 bit times
    8, 9: 9*8 bit times
    10, 11: 11*8 bit times
    12, 13: 13*8 bit times
    14, 15: 15*8 bit times */
#define TMC2209_SLAVECONF                    0x03u
#define TMC2209_SLAVECONF_MASK               0x0F00u
#define TMC2209_SLAVECONF_SHIFT              8u

/*  OTP_PROG (W)
    Write access programs OTP memory (one bit at a time),
    Read access refreshes read data from OTP after a write */
#define TMC2209_OTP_PROG                     0x04u
/*  OTP_PROG.OTPBIT
    Selection of OTP bit to be programmed to the selected
    byte location (n=0..7: programs bit n to a logic 1) */
#define TMC2209_OTP_PROG_OTPBIT_MASK                  0x07u
#define TMC2209_OTP_PROG_OTPBIT_SHIFT                 0u
/*  OTP_PROG.OTPBYTE
    Selection of OTP programming location (0, 1 or 2) */
#define TMC2209_OTP_PROG_OTPBYTE_MASK                 0x30u
#define TMC2209_OTP_PROG_OTPBYTE_SHIFT                4u
/*  OTP_PROG.OTPMAGIC
    Set to 0xBD to enable programming. A programming
    time of minimum 10ms per bit is recommended (check
    by reading OTP_READ). */
#define TMC2209_OTP_PROG_OTPMAGIC_MASK                0xFF00u
#define TMC2209_OTP_PROG_OTPMAGIC_SHIFT               8u

/*  OTP_READ (R)
    Access to OTP memory result and update) */
#define TMC2209_OTP_READ                              0x05u
/*  OTP_READ.OTP0
    byte 0 read data */
#define TMC2209_OTP_READ_OTP0_MASK                    0x01u
#define TMC2209_OTP_READ_OTP0_SHIFT                   0u
/*  OTP_READ.OTP1
    byte 1 read data */
#define TMC2209_OTP_READ_OTP1_MASK                    0x02u
#define TMC2209_OTP_READ_OTP1_SHIFT                   8u
/*  OTP_READ.OTP2
    byte 2 read data */
#define TMC2209_OTP_READ_OTP2_MASK                    0x04u
#define TMC2209_OTP_READ_OTP2_SHIFT                   16u


/*  IOIN (R)
    Reads the state of all input pins available */
#define TMC2209_IOIN                              0x06u
/*  IOIN.ENN */
#define TMC2209_IOIN_ENN_MASK                     0x01u
#define TMC2209_IOIN_ENN_SHIFT                    0u
/*  IOIN.MS1 */
#define TMC2209_IOIN_MS1_MASK                     0x04u
#define TMC2209_IOIN_MS1_SHIFT                    2u
/*  IOIN.MS2 */
#define TMC2209_IOIN_MS2_MASK                     0x08u
#define TMC2209_IOIN_MS2_SHIFT                    3u
/*  IOIN.DIAG */
#define TMC2209_IOIN_DIAG_MASK                    0x10u
#define TMC2209_IOIN_DIAG_SHIFT                   4u
/*  IOIN.PDN_UART */
#define TMC2209_IOIN_PDN_UART_MASK                0x40u
#define TMC2209_IOIN_PDN_UART_SHIFT               6u
/*  IOIN.STEP */
#define TMC2209_IOIN_STEP_MASK                    0x80u
#define TMC2209_IOIN_STEP_SHIFT                   7u
/*  IOIN.SPREAD_EN */
#define TMC2209_IOIN_SPREAD_EN_MASK               0x0100u
#define TMC2209_IOIN_SPREAD_EN_SHIFT              8u
/*  IOIN.DIR */
#define TMC2209_IOIN_DIR_MASK                     0x0200u
#define TMC2209_IOIN_DIR_SHIFT                    9u
/*  IOIN.VERSION
    0x21=first version of the IC
    Identical numbers mean full digital compatibility. */
#define TMC2209_IOIN_VERSION_MASK                 0xFF000000u
#define TMC2209_IOIN_VERSION_SHIFT                24u

/*  FACTORY_CONF (RW) */
#define TMC2209_FACTORY_CONF                 0x07u
/*  FACTORY_CONF.FCLKTRIM (reset=OTP)
    Lowest to highest clock frequency. Check at
    charge pump output. The frequency span is not
    guaranteed, but it is tested, that tuning to 12MHz
    internal clock is possible. The devices come preset to
    12MHz clock frequency by OTP programming. */
#define TMC2209_FACTORY_CONF_FCLKTRIM_MASK                0x1Fu
#define TMC2209_FACTORY_CONF_FCLKTRIM_SHIFT               0u
/*  FACTORY_CONF.OTTRIM (reset=OTP)
    0b00: OT=143°C, OTPW=120°C
    0b01: OT=150°C, OTPW=120°C
    0b10: OT=150°C, OTPW=143°C
    0b11: OT=157°C, OTPW=143°C */
#define TMC2209_FACTORY_CONF_OTTRIM_MASK                  0x30u
#define TMC2209_FACTORY_CONF_OTTRIM_SHIFT                 8u


/*  Datasheet section 5.2: Velocity Dependent Control */


/*  IHOLD_IRUN (W)
    Driver current control */
#define TMC2209_IHOLD_IRUN                   0x10u
/*  IHOLD_IRUN.IHOLD (reset=OTP)
    Standstill current (0=1/32 … 31=32/32)
    In combination with StealthChop mode, setting
    IHOLD=0 allows to choose freewheeling or coil
    short circuit (passive braking) for motor stand still. */
#define TMC2209_IHOLD_IRUN_IHOLD_MASK                   0x1Fu
#define TMC2209_IHOLD_IRUN_IHOLD_SHIFT                  0u
/*  IHOLD_IRUN.IRUN (reset=OTP)
    Motor run current (0=1/32 … 31=32/32)
    Hint: Choose sense resistors in a way that normal
    IRUN is 16 to 31 for best microstep performance. */
#define TMC2209_IHOLD_IRUN_IRUN_MASK                    0x1F00u
#define TMC2209_IHOLD_IRUN_IRUN_SHIFT                   8u
/*  IHOLD_IRUN.IHOLDDELAY (reset=OTP)
    Controls the number of clock cycles for motor
    power down after standstill is detected (stst=1) and
    TPOWERDOWN has expired. The smooth transition
    avoids a motor jerk upon power down.
    0:      instant power down
    1..15:  Delay per current reduction step in multiple
            of 2^18 clocks */
#define TMC2209_IHOLD_IRUN_IHOLDDELAY_MASK              0x0F0000u
#define TMC2209_IHOLD_IRUN_IHOLDDELAY_SHIFT             16u

/*  TPOWERDOWN (W)
    Sets the delay time from stand still (stst) detection to motor
    current power down. Time range is about 0 to 5.6 seconds.

    0…((2^8)-1) * 2^18 tCLK

    Attention: A minimum setting of 2 is required to allow
    automatic tuning of StealthChop PWM_OFFS_AUTO. */
#define TMC2209_TPOWERDOWN                   0x11u
#define TMC2209_TPOWERDOWN_MASK              0xFFu
#define TMC2209_TPOWERDOWN_SHIFT             0u

/*  TSTEP (R)
    Actual measured time between two 1/256 microsteps derived
    from the step input frequency in units of 1/fCLK. Measured
    value is (2^20)-1 in case of overflow or stand still. TSTEP
    always relates to 1/256 step, independent of the actual MRES.

    The TSTEP related threshold uses a hysteresis of 1/16 of the
    compare value to compensate for jitter in the clock or the step
    frequency: (Txxx*15/16)-1 is the lower compare value for each
    TSTEP based comparison.

    This means, that the lower switching velocity equals the
    calculated setting, but the upper switching velocity is higher as
    defined by the hysteresis setting. */
#define TMC2209_TSTEP                        0x12u
#define TMC2209_TSTEP_MASK                   0x0FFFFFu
#define TMC2209_TSTEP_SHIFT                  0u

/*  TPWMTHRS (W)
    Sets the upper velocity for StealthChop voltage PWM mode.
    When TSTEP ≥ TPWMTHRS StealthChop PWM mode is enabled, if configured
    When the velocity exceeds the limit set by TPWMTHRS, the
    driver switches to SpreadCycle.

    0: Disabled */
#define TMC2209_TPWMTHRS                     0x13u
#define TMC2209_TPWMTHRS_MASK                0x0FFFFFu
#define TMC2209_TPWMTHRS_SHIFT               0u

/*  VACTUAL (W)
    VACTUAL allows moving the motor by UART control.
    It gives the motor velocity in +-(2^23)-1 [µsteps / t]

    0: Normal operation. Driver reacts to STEP input.

    /=0: Motor moves with the velocity given by VACTUAL. Step
    pulses can be monitored via INDEX output. The motor
    direction is controlled by the sign of VACTUAL.
*/
#define TMC2209_VACTUAL                      0x22u
#define TMC2209_VACTUAL_MASK                 0xFFFFFFu
#define TMC2209_VACTUAL_SHIFT                0u


/*  Datasheet section 5.3: StallGuard Control */

/*  TCOOLTHRS (W)
    This is the lower threshold velocity for switching on smart
    energy CoolStep and StallGuard to DIAG output. (unsigned)

    Set this parameter to disable CoolStep at low speeds, where it
    cannot work reliably. The stall output signal become enabled
    when exceeding this velocity. It becomes disabled again once
    the velocity falls below this threshold.

    TCOOLTHRS ≥ TSTEP > TPWMTHRS
    - CoolStep is enabled, if configured (only with StealthChop)
    - Stall output signal on pin DIAG is enabled */
#define TMC2209_TCOOLTHRS                       0x14u
#define TMC2209_TCOOLTHRS_MASK                  0xFFFFFu
#define TMC2209_TCOOLTHRS_SHIFT                 0u

/*  SGTHRS (W)
    Detection threshold for stall. The StallGuard value SG_RESULT
    becomes compared to the double of this threshold.
    A stall is signaled with SG_RESULT ≤ SGTHRS*2

    A higher value gives a higher sensitivity. A higher value makes
    StallGuard4 more sensitive and requires less torque to indicate
    a stall. */
#define TMC2209_SGTHRS                          0x40u
#define TMC2209_SGTHRS_MASK                     0xFFu
#define TMC2209_SGTHRS_SHIFT                    0u

/*  SG_RESULT (R)
    StallGuard result.

    SG_RESULT becomes updated with each fullstep, independent of
    TCOOLTHRS and SGTHRS. A higher value signals a lower motor load
    and more torque headroom.

    Intended for StealthChop mode, only. Bits 9 and 0 will always
    show 0. Scaling to 10 bit is for compatibility to StallGuard2. */
#define TMC2209_SG_RESULT                       0x41u
#define TMC2209_SG_RESULT_MASK                  0x3FFu
#define TMC2209_SG_RESULT_SHIFT                 0u

/*  COOLCONF (W)
    CoolStep configuration (datasheet section 5.3.1) */
#define TMC2209_COOLCONF                              0x42u
/*  COOLCONF.seimin
    minimum current for smart current control

    0:  1/2 of current setting (IRUN)
        Attention: use with IRUN≥10
    1:  1/4 of current setting (IRUN)
        Attention: use with IRUN≥20 */
#define TMC2209_COOLCONF_SEMIN_MASK                   0x0000000Fu
#define TMC2209_COOLCONF_SEMIN_SHIFT                  0u
/*  COOLCONF.sedn
    current step down speed

    0b00: For each 32 StallGuard4 values decrease by one
    0b01: For each 8 StallGuard4 values decrease by one
    0b10: For each 2 StallGuard4 values decrease by one
    0b11: For each StallGuard4 value decrease by one */
#define TMC2209_COOLCONF_SEDN_MASK                    0x00006000u
#define TMC2209_COOLCONF_SEDN_SHIFT                   13u
/*  COOLCONF.semax
    StallGuard hysteresis value for smart current control

    If the StallGuard4 result is equal to or above
    (SEMIN+SEMAX+1)*32, the motor current becomes
    decreased to save energy.

    0b0000 … 0b1111: 0 … 15 */
#define TMC2209_COOLCONF_SEMAX_MASK                   0x00000F00u
#define TMC2209_COOLCONF_SEMAX_SHIFT                  8u
/*  COOLCONF.seup
    current up step width

    Current increment steps per measured StallGuard value

    0b00 … 0b11: 1, 2, 4, 8  */
#define TMC2209_COOLCONF_SEUP_MASK                    0x00000060u
#define TMC2209_COOLCONF_SEUP_SHIFT                   5u
/*  COOLCONF.seimin
    minimum StallGuard value for smart current control and
    smart current enable

    If the StallGuard4 result falls below SEMIN*32, the motor
    current becomes increased to reduce motor load angle.

    0b0000: smart current control CoolStep off
    0b0001 … 0b1111: 1 … 15
    */
#define TMC2209_COOLCONF_SEIMIN_MASK                  0x00008000u
#define TMC2209_COOLCONF_SEIMIN_SHIFT                 15u


/*  Datasheet section 5.4: Sequencer Registers */


/*  MSCNT (R)
    Microstep counter. Indicates actual position
    in the microstep table for CUR_A. CUR_B uses
    an offset of 256 into the table. Reading out
    MSCNT allows determination of the motor
    position within the electrical wave. */
#define TMC2209_MSCNT                        0x6Au
#define TMC2209_MSCNT_MASK                   0x03FFu
#define TMC2209_MSCNT_SHIFT                  0u

/*  MSCURACT (R) */
#define TMC2209_MSCURACT                              0x6Bu
/*  MSCURACT.CUR_A (signed):
    Actual microstep current for
    motor phase A (co-sine wave) as
    read from the internal sine wave
    table (not scaled by current
    setting) */
#define TMC2209_MSCURACT_CUR_A_MASK                   0x01FFu
#define TMC2209_MSCURACT_CUR_A_SHIFT                  0u
/*  MSCURACT.CUR_B (signed):
    Actual microstep current for
    motor phase B (sine wave) as
    read from the internal sine wave
    table (not scaled by current
    setting). */
#define TMC2209_MSCURACT_CUR_B_MASK                   0x01FF0000u
#define TMC2209_MSCURACT_CUR_B_SHIFT                  16u


/*  Datasheet section 5.5: Chopper Control Registers */


/*  CHOPCONF (RW)
    Chopper and driver configuration (reset=0x10000053) */
#define TMC2209_CHOPCONF                              0x6Cu
/*  CHOPCONF.TOFF
    Off time and driver enable

    Off time setting controls duration of slow decay phase

    NCLK= 24 + 32*TOFF

    0b0000: Driver disable, all bridges off
    0b0001: 1 – use only with TBL ≥ 2
    0b0010 … %1111: 2 … 15

    (Default: OTP, resp. 3 in StealthChop mode) */
#define TMC2209_CHOPCONF_TOFF_MASK                    0x0Fu
#define TMC2209_CHOPCONF_TOFF_SHIFT                   0u
/*  CHOPCONF.HSTRT
    hysteresis start value added to HEND

    0b000 … 0b111: Add 1, 2, …, 8 to hysteresis low value HEND
    (1/512 of this setting adds to current setting)

    Attention: Effective HEND+HSTRT ≤ 16.
    Hint: Hysteresis decrement is done each 16 clocks */
#define TMC2209_CHOPCONF_HSTRT_MASK                   0x70u
#define TMC2209_CHOPCONF_HSTRT_SHIFT                  4u
/*  CHOPCONF.HEND
    hysteresis low value OFFSET sine wave offset

    0b0000 … 0b1111: Hysteresis is -3, -2, -1, 0, 1, …, 12

    (1/512 of this setting adds to current setting)

    This is the hysteresis value which becomes used for the
    hysteresis chopper.

    (Default: OTP, resp. 0 in StealthChop mode) */
#define TMC2209_CHOPCONF_HEND_MASK                    0x0780u
#define TMC2209_CHOPCONF_HEND_SHIFT                   7u
/*  CHOPCONF.TBL
    blank time select
    0b00 … 0b11: Set comparator blank time to 16, 24, 32 or 40 clocks

    Hint: %00 or %01 is recommended for most applications
    (Default: OTP) */
#define TMC2209_CHOPCONF_TBL_MASK                     0x018000u
#define TMC2209_CHOPCONF_TBL_SHIFT                    15u
/*  CHOPCONF.vsense
    sense resistor voltage based current scaling

    0: Low sensitivity, high sense resistor voltage
    1: High sensitivity, low sense resistor voltage */
#define TMC2209_CHOPCONF_VSENSE_MASK                  0x020000u
#define TMC2209_CHOPCONF_VSENSE_SHIFT                 17u
/*  CHOPCONF.MRES
    micro step resolution

    0b0000: Native 256 microstep setting.

    0b0001 … 0b1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
    Reduced microstep resolution.

    The resolution gives the number of microstep entries per sine quarter wave.
    When choosing a lower microstep resolution, the driver automatically
    uses microstep positions which result in a symmetrical wave.

    Number of microsteps per step pulse = 2^MRES

    (Selection by pins unless disabled by GCONF. mstep_reg_select) */
#define TMC2209_CHOPCONF_MRES_MASK                    0x0F000000u
#define TMC2209_CHOPCONF_MRES_SHIFT                   24u
#define TMC2209_CHOPCONF_MRES_256                     0u
#define TMC2209_CHOPCONF_MRES_128                     1u
#define TMC2209_CHOPCONF_MRES_64                      2u
#define TMC2209_CHOPCONF_MRES_32                      3u
#define TMC2209_CHOPCONF_MRES_16                      4u
#define TMC2209_CHOPCONF_MRES_8                       5u
#define TMC2209_CHOPCONF_MRES_4                       6u
#define TMC2209_CHOPCONF_MRES_2                       7u
#define TMC2209_CHOPCONF_MRES_FULLSTEP                8u
/*  CHOPCONF.intpol
    interpolation to 256 microsteps

    1: The actual microstep resolution (MRES) becomes
    extrapolated to 256 microsteps for smoothest motor
    operation.

    (Default: 1) */
#define TMC2209_CHOPCONF_INTPOL_MASK                  0x10000000u
#define TMC2209_CHOPCONF_INTPOL_SHIFT                 28u
/*  CHOPCONF.dedge
    enable double edge step pulses

    1: Enable step impulse at each step edge to reduce step
    frequency requirement. This mode is not compatible
    with the step filtering function (multistep_filt)
*/
#define TMC2209_CHOPCONF_DEDGE_MASK                   0x20000000u
#define TMC2209_CHOPCONF_DEDGE_SHIFT                  29u
/*  CHOPCONF.diss2g
    short to GND protection disable

    0: Short to GND protection is on
    1: Short to GND protection is disabled */
#define TMC2209_CHOPCONF_DISS2G_MASK                  0x40000000u
#define TMC2209_CHOPCONF_DISS2G_SHIFT                 30u
/*  CHOPCONF.diss2vs
    Low side short protection disable

    0: Short protection low side is on
    1: Short protection low side is disabled */
#define TMC2209_CHOPCONF_DISS2VS_MASK                 0x80000000u
#define TMC2209_CHOPCONF_DISS2VS_SHIFT                31u

/*  DRVSTATUS (R)
    Driver status flags and current level read back */
#define TMC2209_DRVSTATUS                              0x6Fu
/*  DRVSTATUS.otpw
    overtemperature prewarning flag

    1:  The selected overtemperature pre-warning threshold
        is exceeded.

    The overtemperature pre-warning flag is common for
    both bridges. */
#define TMC2209_DRVSTATUS_OTPW_MASK                    0x01u
#define TMC2209_DRVSTATUS_OTPW_SHIFT                   0u
/*  DRVSTATUS.ot
    overtemperature flag

    1: The selected overtemperature limit has been reached.

    Drivers become disabled until otpw is also cleared due
    to cooling down of the IC.

    The overtemperature flag is common for both bridges.*/
#define TMC2209_DRVSTATUS_OT_MASK                      0x02u
#define TMC2209_DRVSTATUS_OT_SHIFT                     1u
/*  DRVSTATUS.s2ga
    short to ground indicator phase A

    1:  Short to GND detected on phase A or B. The driver
        becomes disabled. The flags stay active, until the driver
        is disabled by software (TOFF=0) or by the ENN input.

    Flags are separate for both chopper modes. */
#define TMC2209_DRVSTATUS_S2GA_MASK                    0x04u
#define TMC2209_DRVSTATUS_S2GA_SHIFT                   2u
/*  DRVSTATUS.s2gb
    short to ground indicator phase B

    1:  Short to GND detected on phase A or B. The driver
        becomes disabled. The flags stay active, until the driver
        is disabled by software (TOFF=0) or by the ENN input.

    Flags are separate for both chopper modes. */
#define TMC2209_DRVSTATUS_S2GB_MASK                    0x08u
#define TMC2209_DRVSTATUS_S2GB_SHIFT                   3u
/*  DRVSTATUS.s2vsa
    low side short indicator phase A

    1:  Short on low-side MOSFET detected on phase A or B.
        The driver becomes disabled. The flags stay active, until
        the driver is disabled by software (TOFF=0) or by the
        ENN input.

    Flags are separate for both chopper modes. */
#define TMC2209_DRVSTATUS_S2VSA_MASK                   0x10u
#define TMC2209_DRVSTATUS_S2VSA_SHIFT                  4u
/*  DRVSTATUS.s2vsb
    low side short indicator phase B

    1:  Short on low-side MOSFET detected on phase A or B.
        The driver becomes disabled. The flags stay active, until
        the driver is disabled by software (TOFF=0) or by the
        ENN input.

    Flags are separate for both chopper modes. */
#define TMC2209_DRVSTATUS_S2VSB_MASK                   0x20u
#define TMC2209_DRVSTATUS_S2VSB_SHIFT                  5u
/*  DRVSTATUS.ola
    open load indicator phase A

    1:  Open load detected on phase A or B.

    Hint: This is just an informative flag. The driver takes no
    action upon it. False detection may occur in fast motion
    and standstill. Check during slow motion, only.
*/
#define TMC2209_DRVSTATUS_OLA_MASK                     0x40u
#define TMC2209_DRVSTATUS_OLA_SHIFT                    6u
/*  DRVSTATUS.olb
    open load indicator phase B

    1:  Open load detected on phase A or B.

    Hint: This is just an informative flag. The driver takes no
    action upon it. False detection may occur in fast motion
    and standstill. Check during slow motion, only.
*/
#define TMC2209_DRVSTATUS_OLB_MASK                     0x80u
#define TMC2209_DRVSTATUS_OLB_SHIFT                    7u
/*  DRVSTATUS.t120
    120°C comparator

    1:  Temperature threshold is exceeded */
#define TMC2209_DRVSTATUS_T120_MASK                    0x0100u
#define TMC2209_DRVSTATUS_T120_SHIFT                   8u
/*  DRVSTATUS.t143
    143°C comparator

    1:  Temperature threshold is exceeded */
#define TMC2209_DRVSTATUS_T143_MASK                    0x0200u
#define TMC2209_DRVSTATUS_T143_SHIFT                   9u
/*  DRVSTATUS.t150
    150°C comparator

    1:  Temperature threshold is exceeded */
#define TMC2209_DRVSTATUS_T150_MASK                    0x0400u
#define TMC2209_DRVSTATUS_T150_SHIFT                   10u
/*  DRVSTATUS.t157
    157°C comparator

    1:  Temperature threshold is exceeded */
#define TMC2209_DRVSTATUS_T157_MASK                    0x0800u
#define TMC2209_DRVSTATUS_T157_SHIFT                   11u
/*  DRVSTATUS.CS_ACTUAL
    actual motor current / smart energy current

    Actual current control scaling, for monitoring the
    function of the automatic current scaling. */
#define TMC2209_DRVSTATUS_CS_ACTUAL_MASK               0x1F0000u
#define TMC2209_DRVSTATUS_CS_ACTUAL_SHIFT              16u
/*  DRVSTATUS.stealth
    StealthChop indicator

    1:  Driver operates in StealthChop mode
    0:  Driver operates in SpreadCycle mode */
#define TMC2209_DRVSTATUS_STEALTH_MASK                 0x40000000u
#define TMC2209_DRVSTATUS_STEALTH_SHIFT                30u
/*  DRVSTATUS.stst
    standstill indicator

    This flag indicates motor stand still in each operation
    mode. This occurs 2^20 clocks after the last step pulse. */
#define TMC2209_DRVSTATUS_STST_MASK                    0x80000000u
#define TMC2209_DRVSTATUS_STST_SHIFT                   31u

/*  PWMCONF (RW)
    Voltage PWM Mode StealthChop */
#define TMC2209_PWMCONF                              0x70u
/*  PWMCONF.PWM_OFS
    User defined amplitude (offset)

    User defined PWM amplitude offset (0-255) related to full
    motor current (CS_ACTUAL=31) in stand still.
    (Reset default=36)

    When using automatic scaling (pwm_autoscale=1) the
    value is used for initialization, only. The autoscale
    function starts with PWM_SCALE_AUTO=PWM_OFS and
    finds the required offset to yield the target current
    automatically.

    PWM_OFS = 0 will disable scaling down motor current
    below a motor specific lower measurement threshold.
    This setting should only be used under certain
    conditions, i.e. when the power supply voltage can vary
    up and down by a factor of two or more. It prevents
    the motor going out of regulation, but it also prevents
    power down below the regulation limit.

    PWM_OFS > 0 allows automatic scaling to low PWM duty
    cycles even below the lower regulation threshold. This
    allows low (standstill) current settings based on the
    actual (hold) current scale (register IHOLD_IRUN).
*/
#define TMC2209_PWMCONF_PWM_OFS_MASK                 0xFFu
#define TMC2209_PWMCONF_PWM_OFS_SHIFT                0u
/*  PWMCONF.PWM_GRAD
    User defined amplitude gradient

    Velocity dependent gradient for PWM amplitude:

    PWM_GRAD * 256 / TSTEP

    This value is added to PWM_AMPL to compensate for
    the velocity-dependent motor back-EMF.

    With automatic scaling (pwm_autoscale=1) the value is
    used for first initialization, only. Set PWM_GRAD to the
    application specific value (it can be read out from
    PWM_GRAD_AUTO) to speed up the automatic tuning
    process. An approximate value can be stored to OTP by
    programming OTP_PWM_GRAD. */
#define TMC2209_PWMCONF_PWM_GRAD_MASK                0xFF00u
#define TMC2209_PWMCONF_PWM_GRAD_SHIFT               8u
/*  PWMCONF.PWM_FREQ
    PWM frequency selection

    0b00: fPWM=2/1024 fCLK
    0b01: fPWM=2/683 fCLK
    0b10: fPWM=2/512 fCLK
    0b11: fPWM=2/410 fCLK */
#define TMC2209_PWMCONF_PWM_FREQ_MASK                0x030000u
#define TMC2209_PWMCONF_PWM_FREQ_SHIFT               16u
/*  PWMCONF.pwm_autoscale

    PWM automatic amplitude scaling

    0.  User defined feed forward PWM amplitude. The
        current settings IRUN and IHOLD are not enforced
        by regulation but scale the PWM amplitude, only!
        The resulting PWM amplitude (limited to 0…255) is:

            PWM_OFS * ((CS_ACTUAL+1) / 32) + PWM_GRAD * 256 / TSTEP

    1.   Enable automatic current control (Reset default) */
#define TMC2209_PWMCONF_PWM_AUTOSCALE_MASK           0x040000u
#define TMC2209_PWMCONF_PWM_AUTOSCALE_SHIFT          18u
/*  PWMCONF.pwm_autograd

    PWM automatic gradient adaptation

    0.  Fixed value for PWM_GRAD
        (PWM_GRAD_AUTO = PWM_GRAD)

    1.  Automatic tuning (only with pwm_autoscale=1)
        PWM_GRAD_AUTO is initialized with PWM_GRAD
        and becomes optimized automatically during
        motion.

        Preconditions
        1.  PWM_OFS_AUTO has been automatically
            initialized. This requires standstill at IRUN for
            >130ms in order to a) detect standstill b) wait >
            128 chopper cycles at IRUN and c) regulate
            PWM_OFS_AUTO so that -1 < PWM_SCALE_AUTO < 1
        2.  Motor running and PWM_SCALE_SUM < 255 and
            1.5 * PWM_OFS_AUTO * (IRUN+1)/32
            < PWM_SCALE_SUM < 4 * PWM_OFS_AUTO * (IRUN+1)/32.

        Time required for tuning PWM_GRAD_AUTO
        About 8 fullsteps per change of +/-1. */
#define TMC2209_PWMCONF_PWM_AUTOGRAD_MASK            0x080000u
#define TMC2209_PWMCONF_PWM_AUTOGRAD_SHIFT           19u
/*  PWMCONF.freewheel
    Allows different standstill modes

    Stand still option when motor current setting is zero (I_HOLD=0).

    0b00: Normal operation
    0b01: Freewheeling
    0b10: Coil shorted using LS drivers
    0b11: Coil shorted using HS drivers */
#define TMC2209_PWMCONF_FREEWHEEL_MASK               0x300000u
#define TMC2209_PWMCONF_FREEWHEEL_SHIFT              20u
/*  PWMCONF.PWM_REG
    Regulation loop gradient

    User defined maximum PWM amplitude change per half
    wave when using pwm_autoscale=1. (1…15):

    1: 0.5 increments (slowest regulation)
    2: 1 increment (default with OTP2.1=1)
    3: 1.5 increments
    4: 2 increments
    …
    8: 4 increments (default with OTP2.1=0)
    …
    15: 7.5 increments (fastest regulation) */
#define TMC2209_PWMCONF_PWM_REG_MASK                 0x0F000000u
#define TMC2209_PWMCONF_PWM_REG_SHIFT                24u
/*  PWMCONF.PWM_LIM
    PWM automatic scale amplitude limit when switching on

    Limit for PWM_SCALE_AUTO when switching back from
    SpreadCycle to StealthChop. This value defines the upper
    limit for bits 7 to 4 of the automatic current control
    when switching back. It can be set to reduce the current
    jerk during mode change back to StealthChop.

    It does not limit PWM_GRAD or PWM_GRAD_AUTO offset.

    (Default = 12) */
#define TMC2209_PWMCONF_PWM_LIM_MASK                 0xF0000000u
#define TMC2209_PWMCONF_PWM_LIM_SHIFT                28u

/*  PWM_SCALE (R)
    Results of StealthChop amplitude regulator.

    These values can be used to monitor automatic PWM
    amplitude scaling (255=max. voltage). */
#define TMC2209_PWM_SCALE      0x71u
/*  PWM_SCALE.PWM_SCALE_SUM
    Actual PWM duty cycle. This value is used for scaling the
    values CUR_A and CUR_B read from the sine wave table */
#define TMC2209_PWM_SCALE_SUM_MASK           0xFFu
#define TMC2209_PWM_SCALE_SUM_SHIFT          0u
/*  PWM_SCALE.PWM_SCALE_AUTO
    9 Bit signed offset added to the calculated PWM duty cycle.
    This is the result of the automatic amplitude regulation
    based on current measurement */
#define TMC2209_PWM_SCALE_AUTO_MASK          0x01FF0000u
#define TMC2209_PWM_SCALE_AUTO_SHIFT         16u

/*  PWM_AUTO (R)
    These automatically generated values can be
    read out in order to determine a default /
    power up setting for PWM_GRAD and PWM_OFS. */
#define TMC2209_PWM_AUTO                     0x72u
/*  PWM_AUTO.PWM_OFS_AUTO
    Automatically determined offset value */
#define TMC2209_PWM_AUTO_OFS_MASK            0xFFu
#define TMC2209_PWM_AUTO_OFS_SHIFT           0u
/*  PWM_AUTO.PWM_GRAD_AUTO
    Automatically determined gradient value */
#define TMC2209_PWM_AUTO_GRAD_MASK           0xFF0000u
#define TMC2209_PWM_AUTO_GRAD_SHIFT          16u
