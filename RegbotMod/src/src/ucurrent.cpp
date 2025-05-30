/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
 *   jcan@dtu.dk            
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include <stdio.h>
#include "ucontrol.h"
#include "urobot.h"
#include "ucommand.h"
#include "ueeconfig.h"
#include "uusb.h"
#include "usubss.h"

#include "ulinesensor.h"

#include "umotor.h"
#include "ucurrent.h"
#include "uad.h"

UCurrent current;



void UCurrent::setup()
{
  // info messages
  addPublistItem("mca", "Get motor current 'mca m1 m2' [Amps]");
  addPublistItem("mco", "Get motor current offset (AD: 0..4096) 'mco o1 o2 ad1 ad2'");
  //   addPublistItem("va", "Get motor current");
  usb.addSubscriptionService(this);
  //
  /// current measurement scale
  /// ADC returns 0->4095, but filtered to range 8192 in uad.cpp: ad.adInterrupt(...)
  const float lpFilteredMaxADC = pow(2, ad.useADCresolution) * 2;
  #if defined(REGBOT_HW41) || defined(REGBOT_63_35)
  // op-amp sensor, Rsense=0.1, op-amp gain=1
  if (robot.robotHWversion >= 9)
  { /// blue PCB - 5A HAL sensor ACS714
    /// 185mV/A, voltage divider 7.5k/15k, 3.3Vref, 12bit, scaled by factor 300
    /// calculated value and measurements differ by a factor 2.5 without filter cap on R31 and R33
    const float calibration_factor = 1/1;
    scale = - 3.3 * 22.5e3 / 15e3 / 0.185 / lpFilteredMaxADC / 300 * calibration_factor;
  }
  else
    // purple PCB - current measurement has design error (don't use!).
    scale = - 3.3 / 0.1 / lpFilteredMaxADC / 300;
  #else
  // HAL sensor
  // sensor: 2.5V (5V/2) is zero and 185mV/A
  // offset to zero about 0.7V and still 185mV/A
  // A/D max=1.2V 12bit
  // measured value is up-scaled with factor 300 to
  // improve accuracy with low-pass filter
  scale = 1.2 / lpFilteredMaxADC / 0.185 / 300.0 ; // 185 mV/A
  #endif
  versioncpp = "$Id: ucurrent.cpp 1702 2024-11-28 15:00:22Z jcan $";
}


void UCurrent::tick()
{ // 
  logIntervalChanged();
  if (currentOffsetting and not motor.motorPreEnabled)
  { // stop calibration
    // usb.send("# Current offset switching to run mode\n");
  }
  else if (motor.motorPreEnabled and not currentOffsetting)
  { // wait a bit after stop before starting calibration
    postpondCalibration = 300; // ticks
    // usb.send("# Current offset in progress in a little while\n");
  }
  currentOffsetting = motor.motorPreEnabled;
  if (postpondCalibration > 0)
    postpondCalibration--;
  //
  if (motor.motorPreEnabled and postpondCalibration == 0)
  { // low pass input values (using long integer) - about 100ms time constant (if currentCntMax==1)
    if (motor.motorPreEnabledRestart)
    { // just started - first measurement
      motor.motorPreEnabledRestart = false;
      motorCurrentMLowPass[0] = ad.motorCurrentRawAD[0] * 300;
      motorCurrentMLowPass[1] = ad.motorCurrentRawAD[1] * 300;
      // usb.send("# motor.motorPreEnabledRestart=true\n");
    }
    else
    { // running average until motor is enabled (over 300 samples)
      motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 299)/300 + ad.motorCurrentRawAD[0];
      motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 299)/300 + ad.motorCurrentRawAD[1];
    }
    // save as direct usable offset value
    // also makes the value 0 when calculating offset
    motorCurrentMOffset[0] = motorCurrentMLowPass[0];
    motorCurrentMOffset[1] = motorCurrentMLowPass[1];
    // snprintf(s, MSL, "#current %d %d raw\n", motorCurrentM[0], motorCurrentM[1]);
    // usb.send(s);
  }
  else
  { // measurement in progress
    // average current as function of log interval (but keep value 300 times larger than raw data)
    motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 
                               (300 - lowPassFactor))/300 +
                               ad.motorCurrentRawAD[0] * lowPassFactor;
    motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 
                               (300 - lowPassFactor))/300 +
                               ad.motorCurrentRawAD[1] * lowPassFactor;
  }
  motorCurrentA[0] = getMotorCurrentM(0, motorCurrentMLowPass[0]);
  motorCurrentA[1] = getMotorCurrentM(1, motorCurrentMLowPass[1]);
  
  // service subscriptions
//   subscribeTick();
}


void UCurrent::sendHelp()
{
  usb.send("# Motor current -------\r\n");
}

bool UCurrent::decode(const char* buf)
{ // no current commands
  return false;
}

void UCurrent::sendData(int item)
{
  if (item == 0)
    sendMotorCurrent();
  else if (item == 1)
    sendMotorCurrentOffset();
}

void UCurrent::sendMotorCurrent()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"mca %g %g\r\n", motorCurrentA[0], motorCurrentA[1]);
  usb.send(reply);
}

void UCurrent::sendMotorCurrentOffset()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"mco %ld %ld  %d %d\r\n", 
           motorCurrentMOffset[0]/300, motorCurrentMOffset[1]/300, 
           ad.motorCurrentRawAD[0], ad.motorCurrentRawAD[1]);
  usb.send(reply);
}

// void UCurrent::sendStatusCurrentVolt()
// {
//   const int MRL = 250;
//   char reply[MRL];
//   snprintf(reply, MRL, "va 0 0 0 0 0 0\n" // %d %.3f  %d %ld %.3f  %d %ld %.3f\r\n",
// //            batVoltRawAD, robot.batteryVoltage, 
// //            motorCurrentRawAD[0], motorCurrentMOffset[0], getMotorCurrentM(0, motorCurrentM[0]),
// //            motorCurrentRawAD[1], motorCurrentMOffset[1], getMotorCurrentM(1, motorCurrentM[1])
//   );
//   usb.send(reply);
// }

/**
 * get motor current for motor 0 or 1 in amps.
 * NB! no test for valid index.
 * \returns current in amps */
float UCurrent::getMotorCurrentM(int m, int32_t value)
{ 
  if (m == 0)
    return float(value - motorCurrentMOffset[0]) * scale;
  else
    // right motor runs backwards, so current is negative,
    // change sign so that forward shows a positive current
    return - float(value - motorCurrentMOffset[1]) * scale;
}

void UCurrent::logIntervalChanged()
{ // average value is always a factor 300 more than AD value
  // Low pass filter for motor current
  // based on actual logging sample interval (in ms)
  if (logger.logInterval <= 2)
    lowPassFactor = 300/1; // use new value only
  else if (logger.logInterval > 300)
    lowPassFactor = 300/150; // time constant about 150ms
  else
    // use half the sample interval
    lowPassFactor = 2*300/logger.logInterval;
}


void UCurrent::eePromLoad()
{
  // deviceID = eeConfig.readWord();
}

void UCurrent::eePromSave()
{
  // eeConfig.pushWord(deviceID);
}


