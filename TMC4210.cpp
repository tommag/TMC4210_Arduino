/*
 * TMC4210 Motion control IC by Trinamic.
 *
 * Tom Magnier <tom@tmagnier.fr> 07/2018
 */

#include "Arduino.h"
#include "TMC4210.h"

// #define SERIAL_DEBUG


TMC4210::TMC4210()
{

}

void TMC4210::begin(long clockFreq, int csPin, long maxSpeed, long maxAccel)
{
  _clockFreq = clockFreq;
  _csPin = csPin;
  _spiStatus = {0};

  SPI.begin(); //Init SPI hardware
  _spiSettings = SPISettings(clockFreq/16, MSBFIRST, SPI_MODE3);

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  setStepDurationUs(_defaultStepLength);

  setPulseRampDiv(maxSpeed, maxAccel);

  //Enable en_sd bit and init IF_CONFIGURATION_4210
  IF_CONFIGURATION_4210_Register reg = {0};
  reg.en_sd = true;
  writeRegister(IF_CONFIGURATION_4210, reg.value);

  writeRegister(V_MIN, 1);

  //Init ramp mode and set XTarget to XActual to avoid an undesired movement
  setTargetPosition(getCurrentPosition());
  setRampMode(RAMP_MODE);

  //Init VMAX, AMAX
  setMaxSpeed(maxSpeed);
  setAcceleration(maxAccel);
}

bool TMC4210::isTargetReached()
{
  readRegister(TYPE_VERSION); //Dummy SPI transfer
  return bool(_spiStatus.TARGET_REACHED);
}

void TMC4210::setOutputsPolarity(bool stepInverted, bool dirInverted)
{
  IF_CONFIGURATION_4210_Register reg = {0};
  reg.value = readRegister(IF_CONFIGURATION_4210);
  reg.inv_stp = stepInverted;
  reg.inv_dir = dirInverted;

  writeRegister(IF_CONFIGURATION_4210, reg.value);
}

void TMC4210::setStepDurationUs(int stepDuration)
{
  long registerValue = (stepDuration * _clockFreq / 16000000L) - 1;

  GLOBAL_PARAMETERS_Register reg = {0};
  reg.value = readRegister(GLOBAL_PARAMETERS);
  reg.STPDIV_4210 = registerValue;

  writeRegister(GLOBAL_PARAMETERS, reg.value);
}

void TMC4210::setRampMode(TMC4210::RampMode mode)
{
  RAMPMODE_REFCONF_Register reg = {0};
  reg.value = readRegister(RAMPMODE_REFCONF);
  reg.RAMP_MODE = mode;
  writeRegister(RAMPMODE_REFCONF, reg.value);
}

long TMC4210::getCurrentPosition()
{
  long returnValue = readRegister(X_ACTUAL);
  //Handle negative values
  if (bitRead(returnValue, 23))
    returnValue |= 0xFF000000;
  return returnValue;
}

void TMC4210::setCurrentPosition(long position)
{
  writeRegister(X_ACTUAL, position);
}

int TMC4210::getCurrentSpeed()
{
  return speedInternalToHz(readRegister(V_ACTUAL));
}

int TMC4210::getCurrentAcceleration()
{
  return accelInternalToHz(readRegister(A_ACTUAL));
}

void TMC4210::setMaxSpeed(int speed)
{
  writeRegister(V_MAX, speedHzToInternal(abs(speed)));
}

void TMC4210::setTargetSpeed(int speed)
{
  //Update VMAX as well
  setMaxSpeed(speed);
  writeRegister(V_TARGET, speedHzToInternal(speed));
}

void TMC4210::setHoldModeSpeed(int speed)
{
  //Update VMAX as well
  setMaxSpeed(speed);
  writeRegister(V_ACTUAL, speedHzToInternal(speed));
}

void TMC4210::setAcceleration(int maxAccel)
{
  writeRegister(A_MAX, accelHzToInternal(abs(maxAccel)));
  setPmulPdiv(accelHzToInternal(abs(maxAccel)));
}

long TMC4210::getTargetPosition()
{
  long returnValue = readRegister(X_TARGET);
  //Handle negative values
  if (bitRead(returnValue, 23))
    returnValue |= 0xFF000000;
  return returnValue;

}

void TMC4210::setTargetPosition(long position)
{
  writeRegister(X_TARGET, position);
}

void TMC4210::stop()
{
  setMaxSpeed(0);
}

void TMC4210::writeRegister(const byte address, const long data)
{
  #ifdef SERIAL_DEBUG
  Serial.print("Writing reg 0x");
  Serial.print(address, HEX);
  Serial.print(":");
  Serial.println(data);
  #endif
  spiTransfer(address, data);
}

long TMC4210::readRegister(const byte address)
{
  long returnValue = spiTransfer(address + 1, 0);
  #ifdef SERIAL_DEBUG
  Serial.print("Reading reg 0x");
  Serial.print(address, HEX);
  Serial.print(":");
  Serial.println(returnValue);
  #endif
  return returnValue;
}

long TMC4210::spiTransfer(const byte address, const long data)
{
  long returnBuffer = 0;

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);

  _spiStatus.value = SPI.transfer(address);
  //Send data MSB first. 3 bytes only.
  for (int i = 2; i >= 0; i--)
    returnBuffer |= (SPI.transfer((data >> (i*8)) & 0xFF) << (i*8));

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  return returnBuffer;
}

void TMC4210::setPulseRampDiv(long maxSpeed, long maxAccel)
{
  /* See Trinamic AN016 */
  _pulseDiv = floor(log( (float)(((long long)_clockFreq * 2047LL) / ((long long)abs(maxSpeed) * 2048LL * 32LL)) ) / log(2.0)); //TODO ceil ?
  _rampDiv = floor(log( (float)(((long long)_clockFreq * (long long)_clockFreq * 2047LL) / ((long long)abs(maxAccel) * (1LL << _pulseDiv+29)))) / log(2.0)); //TODO ceil() ?

  #ifdef SERIAL_DEBUG
  Serial.print("PULSE_DIV: ");
  Serial.print(_pulseDiv);
  Serial.print(", RAMP_DIV: ");
  Serial.println(_rampDiv);
  #endif

  PULSE_DIV_RAMP_DIV_Register reg = {0};
  reg.PULSE_DIV = _pulseDiv;
  reg.RAMP_DIV = _rampDiv;
  writeRegister(PULSE_DIV_RAMP_DIV, reg.value);
}

/* Taken from TMC4210 datasheet */
void TMC4210::setPmulPdiv(long accel)
{
  int pdiv, pmul, pm, pd ;

  pm=-1; pd=-1; // -1 indicates : no valid pair found
  double p = accel / ( 128.0 * pow(2, _rampDiv-_pulseDiv) );
  double p_reduced = p * 0.99;

  #ifdef SERIAL_DEBUG
  Serial.print("P: ");
  Serial.println(p);
  #endif

  for (pdiv=0; pdiv<=13; pdiv++)
  {
    pmul = (int)(p_reduced * 8.0 * pow(2, pdiv)) - 128;
    if ( (0 <= pmul) && (pmul <= 127) )
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }

  #ifdef SERIAL_DEBUG
  Serial.print("PMUL: ");
  Serial.print(pm);
  Serial.print(", PDIV: ");
  Serial.println(pd);

  double p_best = ((double)(pm)) / ((double)pow(2,pd+3));
  Serial.print("P actual: ");
  Serial.println(p_best);
  #endif

  PMUL_PDIV_Register reg = {0};
  reg.PMUL = pm;
  reg.PDIV = pd;
  writeRegister(PMUL_PDIV, reg.value);
}

int TMC4210::speedHzToInternal(int speedHz)
{
  return constrain(((long long)speedHz * (1LL << _pulseDiv) * 2048LL * 32LL) / (long long)_clockFreq, -2048, 2047);
}

int TMC4210::speedInternalToHz(int speedInternal)
{
  // Handle signed values
  if (speedInternal >= 2048)
    speedInternal = speedInternal - 4096;
  return (long long)_clockFreq * (long long)speedInternal / ((1LL << _pulseDiv) * 2048LL * 32LL);
}

int TMC4210::accelHzToInternal(int accelHz)
{
  return constrain(((long long)accelHz * (1LL << (_pulseDiv + _rampDiv + 29))) / ((long long)_clockFreq * (long long)_clockFreq), 0, 2047);
}

int TMC4210::accelInternalToHz(int accelInternal)
{
  // Handle signed values
  if (accelInternal >= 2048)
    accelInternal = accelInternal - 4096;

  return (long long)_clockFreq * (long long)_clockFreq * (long long)accelInternal / (1LL << (_pulseDiv + _rampDiv + 29));
}
