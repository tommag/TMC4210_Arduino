/*
 * TMC4210 Motion control IC by Trinamic.
 *
 * Note that this library doesn't provide a clock to the TMC4210 as this is platform-dependent.
 *
 * Tom Magnier <tom@tmagnier.fr> 07/2018
 */

#ifndef TMC4210_H
#define TMC4210_H

#include "Arduino.h"
#include "SPI.h"
#include <util/Bitfield.h>

class TMC4210
{
public:
  /* Register addresses. Add 1 for read access. */
  enum {
    /* Stepper motor register set */
    X_TARGET             = 0x00,
    X_ACTUAL             = 0x02,
    V_MIN                = 0x04,
    V_MAX                = 0x06,
    V_TARGET             = 0x08,
    V_ACTUAL             = 0x0A,
    A_MAX                = 0x0C,
    A_ACTUAL             = 0x0E,
    PMUL_PDIV            = 0x12,
    RAMPMODE_REFCONF     = 0x14,
    INTERRUPT_MASK_FLAGS = 0x16,
    PULSE_DIV_RAMP_DIV   = 0x18,
    DX_REF_TOLERANCE     = 0x1A,
    X_LATCHED            = 0x1C,
    USTEP_COUNT_4210     = 0x1E,

    /* Global parameter registers */
    IF_CONFIGURATION_4210= 0x68,
    POS_COMP_4210        = 0x6A,
    POS_COMP_INT_4210    = 0x6C,
    POWER_DOWN           = 0x70,
    TYPE_VERSION         = 0x72,
    REFERENCE_SWITCHES   = 0x7C,
    GLOBAL_PARAMETERS    = 0x7E
  };

    /* Register bit fields */
  union PMUL_PDIV_Register {
    uint32_t value;
    BitField< 0, 4> PDIV;
    BitField< 8, 8> PMUL;
  };

  union RAMPMODE_REFCONF_Register {
    uint32_t value;
    BitField< 0, 2> RAMP_MODE;
    BitField< 8, 4> REF_CONF;
    BitField< 8> disable_stop_l;
    BitField< 9> disable_stop_r;
    BitField<10> soft_stop;
    BitField<11> ref_RnL;
    BitField<16> lp;
  };

  union INTERRUPT_MASK_FLAGS_Register {
    uint32_t value;
    BitField< 0, 8> INTERRUPT_FLAGS;
    BitField< 8, 8> INTERRUPT_MASK;
    BitField< 0> int_pos_end;
    BitField< 1> int_ref_wrong;
    BitField< 2> int_ref_miss;
    BitField< 3> int_stop;
    BitField< 4> int_stop_left_low;
    BitField< 5> int_stop_right_low;
    BitField< 6> int_stop_left_high;
    BitField< 7> int_stop_right_high;
    BitField< 8> mask_pos_end;
    BitField< 9> mask_ref_wrong;
    BitField<10> mask_ref_miss;
    BitField<11> mask_stop;
    BitField<12> mask_stop_left_low;
    BitField<13> mask_stop_right_low;
    BitField<14> mask_stop_left_high;
    BitField<15> mask_stop_right_high;
  };

  union PULSE_DIV_RAMP_DIV_Register {
    uint32_t value;
    BitField< 8, 4> RAMP_DIV;
    BitField<12, 4> PULSE_DIV;
  };

  union IF_CONFIGURATION_4210_Register {
    uint32_t value;
    BitField< 0> inv_ref;
    BitField< 1> sdo_int;
    BitField< 2> step_half;
    BitField< 3> inv_stp;
    BitField< 4> inv_dir;
    BitField< 5> en_sd;
    BitField< 8> en_refr;
  };

  union POS_COMP_INT_4210_Register {
    uint32_t value;
    BitField< 0> interrupt;
    BitField< 8> mask;
  };

  union REFERENCE_SWITCHES_Register {
    uint32_t value;
    BitField< 0> r_r;
    BitField< 1> r_l;
    BitField< 3> gp_in;
  };

  union GLOBAL_PARAMETERS_Register {
    uint32_t value;
    BitField< 8, 4> STPDIV_4210;
    BitField<21> mot1r;
  };

  union STATUS_BITS {
    uint8_t value;
    BitField<0> TARGET_REACHED;
    BitField<1> R_L;
    BitField<3> GP_IN;
    BitField<5> R_R;
    BitField<7> INT;
  };

  enum RampMode {
    RAMP_MODE = 0x00,
    SOFT_MODE = 0x01,
    VELOCITY_MODE = 0x02,
    HOLD_MODE = 0x03
  };

  TMC4210();

  /* Init TMC4210.
   * clockFreq : TMC4210 CLK frequency (Hz)
   * csPin : SPI Chip Select pin to use
   * maxSpeed : desired max speed in microsteps / s
   * maxAccel : desired max accel in microsteps / s^2
   * These last 2 parameters are used to determine the clock dividers values to
   * get the best possible resolution.
   * You can't set higher speeds / accel afterwards.
   */
  void begin(long clockFreq, int csPin, long maxSpeed, long maxAccel);

  bool isTargetReached();

  //TODO interrupt configuration ; event handling

  /* Set step/dir outputs polarity
   * if stepInverted is true, LOW indicates an active step
   * if dirInverted is true, HIGH indicates negative direction
   */
  void setOutputsPolarity(bool stepInverted, bool dirInverted);

  /* Set step duration in microseconds
   */
  void setStepDurationUs(int stepDuration);

  /* Enable right reference switch */
  void enableRightReferenceSwitch();

  /* Enable / disable stop function of reference switches */
  void setStopSwitchesEnabled(bool left, bool right);

  /* Ramp generator mode :
   * RAMP_MODE : positioning mode, with a trapezoidal profile set by setAcceleration(), setMaxSpeed() and setTargetPosition()
   * SOFT_MODE : same as positioning mode with an exponential deceleration profile
   * VELOCITY_MODE : constant speed mode, with linear acceleration ramps set by setAcceleration() and setTargetSpeed()
   * HOLD_MODE : no ramp generation, direct velocity control by the microcontroller set by setHoldModeSpeed()
   */
  void setRampMode(RampMode mode);

  /* Return the current internal position (in microsteps) */
  long getCurrentPosition();

  /* Set the current internal position (in microsteps) */
  void setCurrentPosition(long position);

  /* Return the current speed (in microsteps / second) */
  int getCurrentSpeed();

  /* Return the current acceleration (in microsteps / second^2) */
  int getCurrentAcceleration();

  /* Set the max speed VMAX (microsteps/second)
   * Useful only in RAMP and SOFT modes */
  void setMaxSpeed(int speed);

  /* Set the target speed VTARGET (microsteps/second)
   * Useful only in VELOCITY mode */
  void setTargetSpeed(int speed);

  /* Set the hold mode speed VACTUAL (microsteps/second)
   * Useful only in HOLD mode */
  void setHoldModeSpeed(int speed);

  /* Set the ramp acceleration (in microsteps / second^2)*/
  void setAcceleration(int maxAccel);

  /* Get the target position in microsteps */
  long getTargetPosition();

  /* Set the target position
   * /!\ Set all other motion profile parameters before
   */
  void setTargetPosition(long position);

  /* Stop the current motion according to the set ramp mode and motion parameters. The max speed is set to 0 but the target position stays unchanged. */
  void stop();

private:
  const static int _defaultStepLength = 5; //us

  long _clockFreq; //TMC4210 clock frequency (Hz)
  int _csPin; //Chip Select pin number

  /* TMC4210 speed & acceleration pre-dividers */
  int _pulseDiv;
  int _rampDiv;

  STATUS_BITS _spiStatus; //Contents of the status bits updated on each SPI transaction

  SPISettings _spiSettings;

  void writeRegister(const byte address, const long data);
  long readRegister(const byte address);
  long spiTransfer(const byte address, const long data);

  /* Calculate and set pulseDiv & rampDiv given the desired maxSpeed (usteps/s) and
   * maxAccel (usteps/s^2)
   */
  void setPulseRampDiv(long maxSpeed, long maxAccel);

  /* Calculate and set PMUL & PDIV deceleration proportionality factor */
  void setPmulPdiv(long accel);

  /* Conversion between real world units and internal units */
  int speedHzToInternal(int speedHz);
  int speedInternalToHz(int speedInternal);
  int accelHzToInternal(int accelHz);
  int accelInternalToHz(int accelInternal);
};

#endif //TMC4210_H
