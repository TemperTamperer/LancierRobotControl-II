/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
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

#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "ulog.h"
#include "umotor.h"
#include "umotortest.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
#include "ucontrol.h"
#include "usubss.h"
#include "ucurrent.h"
#include "uencoder.h"
#include "uad.h"

UMotorTest motortest;



void UMotorTest::setup()
{ //
  if (setupCnt == 0)
  { // only once
    addPublistItem("motest", "Get estimated parameters 'motest mot[0,1] cv=0/ccv=1 Km R L B S I");
    addPublistItem("motpar", "Get test parameters 'motpars lowVolt highVolt state-time'");
    usb.addSubscriptionService(this);
  }
  setupCnt++;
  versioncpp = "$Id: umotortest.cpp 1699 2024-11-26 16:41:47Z jcan $";
}


void UMotorTest::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Motor test -------\r\n");
  snprintf(reply, MRL, "# -- \tmottest C\tStart motor test C=0:CV, C=1:CCV, C=-1:stop (NB! Allow wheels to rotate freely)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotset Vlow Vhigh\tMotortest voltage Vlow 3-6 Volt, Vhigh > VLow, samples per state (ms).\r\n");
  usb.send(reply);
}


bool UMotorTest::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "motset ", 7) == 0)
  {
    const char * p1 = &buf[7];
    voltageLow = strtof(p1, (char**)&p1);
    voltageHigh = strtof(p1, (char**)&p1);
    if (voltageLow < 0.5)
      voltageLow = 3;
    if (voltageHigh < 0.6)
      voltageHigh = 6;

    int state_ms = strtol(p1, (char**)&p1, 10);
    int max = LOG_BUFFER_MAX / sizeof(UMotorTestMeasurementData) - 1;
    if (state_ms * 7 > max)
      state_ms = max/7 -1;
    stateLength = state_ms;
  }
  else if (strncmp(buf, "mottest", 7) == 0)
  { // get parameter value
    // default is 0 (CV test)
    if (voltageLow < 0.5)
      voltageLow = 3;
    if (voltageHigh < 0.6)
      voltageHigh = 6;
    const char * p1 = &buf[7];
    switch (strtol(p1, nullptr, 10))
    {
      case 0:
//         motorTestType = MT_CV;
        voltageCCV = 1;
        mLogIndex = 0;
        motorTestRunning = true;
        break;
      case 1:
//         motorTestType = MT_CCV;
        voltageCCV = -1;
        mLogIndex = 0;
        motorTestRunning = true;
        break;
      default:
//         motorTestType = MT_STOP;
        motorTestEnd = true;
        break;
    }
    usb.send("# ----- starting motor test (in 1 second) ------\n");
  }
  else
    used = false;
  return used;
}

void UMotorTest::sendData(int item)
{
  if (item == 0)
    sendMotorParameters();
  else if (item == 1)
    sendMotortestSetup();
}

void UMotorTest::sendMotorParameters()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "motest %d %f %g %g %g %g %g %g %d\r\n",
           0, voltageCCV, mKonstant[0], mResistance[0], mInductance[0], mFricDyn[0], mFricStat[0], mInertia[0], testValidLeft);
  usb.send(s);
  snprintf(s, MSL, "motest %d %f %g %g %g %g %g %g %d\r\n",
           1, voltageCCV, mKonstant[1], mResistance[1], mInductance[1], mFricDyn[1], mFricStat[1], mInertia[1], testValidRight);
  usb.send(s);
}

void UMotorTest::sendMotortestSetup()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "motpar %f %f %d\r\n",
           voltageLow * voltageCCV, voltageHigh * voltageCCV, stateLength);
  usb.send(s);
}

void UMotorTest::tick()
{ //
  if (mLogIndex == 0 and motorTestRunning)
  { // motortest uses the logger buffer
    logger.stopLogging();
    logger.logRowCnt = 0;
    mLog = (UMotorTestMeasurementData *)logger.logBuffer;
    mLogIndexMax = LOG_BUFFER_MAX / sizeof(UMotorTestMeasurementData) - 1;
    motorTestEnd = false;
    tickCnt = 0;
    testState = MS_OFFSET;
    motor.motorVoltage[0] = 0;
    motor.motorVoltage[1] = 0;
    motor.motorSetEnable(1,1);
    encoderStart[0] = encoder.encoder[0];
    encoderStart[1] = encoder.encoder[1];
  }
  if (motorTestRunning)
  {
    if (testState == MS_OFFSET)
    { // current offset should not be needed
      // skipped for now
      testState = MS_OFF;
    }
    if (mLogIndex < mLogIndexMax and mLog != nullptr)
    {
      mLog[mLogIndex].mTime10us = hb10us;
      for (int i = 0; i < 2; i++)
      {
        mLog[mLogIndex].mCurrent[i] = current.getMotorCurrentM(i);
        mLog[mLogIndex].mVoltage[i] = motor.motorVoltage[i];
        mLog[mLogIndex].mEncoder[i] = encoder.encoder[i];
      }
      mLogIndex++;
    }
    else
    {
      motorTestEnd = true;
      usb.send("# motor test log is full (or no buffer)\n");
    }
    // state model
    if (tickCnt > stateLength)
    { // test state finished
      tickCnt = 0;
      switch (testState)
      { // switch to next state and note log index
        case MS_OFF:
          // start rolling
          testState = MS_LLOW;
          leftTestIndex[0] = mLogIndex;
          leftTestIndex[1] = mLogIndex + steadyStateTime;
          motor.motorVoltage[0] = voltageLow * voltageCCV;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LLOW:
          // switch to high voltage
          testState = MS_LHIGH;
          leftTestIndex[2] = mLogIndex - 1; // end of low
          leftTestIndex[3] = mLogIndex; // start of high
          leftTestIndex[4] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = voltageHigh * voltageCCV;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LHIGH:
          // end of low
          testState = MS_LEND;
          leftTestIndex[5] = mLogIndex - 1; // end of low
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LEND:
          // start right motor
          testState = MS_RLOW;
          rightTestIndex[0] = mLogIndex; // start of low
          rightTestIndex[1] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = -voltageLow * voltageCCV;
          break;
        case MS_RLOW:
          // switch to high voltage
          testState = MS_RHIGH;
          rightTestIndex[2] = mLogIndex -1 ; // end of low
          rightTestIndex[3] = mLogIndex; // start of high
          rightTestIndex[4] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = -voltageHigh * voltageCCV;
          break;
        case MS_RHIGH:
          // switch to roll off
          testState = MS_ROLL_OFF;
          rightTestIndex[5] = mLogIndex - 1; // end of high
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = 0;
          break;
        default:
          motorTestEnd = true;
          testState = MS_END;
          break;
      }
    }
    // terminate
    if (motorTestEnd)
    { // stop
      motorTestRunning = false;
      // stop motors
      motor.motorVoltage[0] = 0;
      motor.motorVoltage[1] = 0;
      motor.motorSetEnable(0,0);
      // calculate
      estimateMotorParams();
    }
  }
  else if (motorTestGetLog)
  {
//     usb.send("# motor test log get\n");
    if (motorTestGetIndex >= mLogIndex)
    {
      motorTestGetLog = false;
      usb.send("logend\n");
    }
    else
    {
      sendTestLog();
      motorTestGetIndex++;
    }
  }
  tickCnt++;
}

///////////////////////////////////////////////////////

void UMotorTest::eePromSave()
{
  // save status
  uint16_t flags = 0;
  flags |= testValidLeft << 0;
  flags |= testValidRight << 1;
  flags |= encoderReversed << 4;
  eeConfig.pushWord(flags);
  // save in kHz
  stateLength = eeConfig.readWord();
  eeConfig.pushFloat(voltageLow);
  eeConfig.pushFloat(voltageHigh);
  for (int i = 0; i < 2; i++)
  {
    eeConfig.pushFloat(mResistance[i]);
    eeConfig.pushFloat(mInductance[i]);
    eeConfig.pushFloat(mKonstant[i]);
    eeConfig.pushFloat(mFricDyn[i]);
    eeConfig.pushFloat(mFricStat[i]);
    eeConfig.pushFloat(mInertia[i]);
  }
}

void UMotorTest::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    uint16_t flags = eeConfig.readWord();
    testValidLeft = (flags & 0x01) > 0;
    testValidRight = (flags & 0x02) > 0;
    encoderReversed = (flags & 0x10) > 0;
    // stored value is in kHz
    stateLength = eeConfig.readWord();
    voltageLow = eeConfig.readFloat();
    voltageHigh = eeConfig.readFloat();
    if (stateLength < 300 or stateLength > 2000)
      stateLength = 1000;
    if (voltageLow < 0.5 or isnanf(voltageLow))
      voltageLow = 3;
    if (voltageHigh < 0.6 or isnanf(voltageHigh))
      voltageHigh = 6;
    for (int i = 0; i < 2; i++)
    {
      mResistance[i] = eeConfig.readFloat();
      mInductance[i] = eeConfig.readFloat();
      mKonstant[i] = eeConfig.readFloat();
      mFricDyn[i] = eeConfig.readFloat();
      mFricStat[i] = eeConfig.readFloat();
      mInertia[i] = eeConfig.readFloat();
    }
    setup();
  }
  else
  { // skip robot specific items
    // one word + 3+12 floats
    int skip = 2 + 3*4 + 2*6*4;
    eeConfig.skipAddr(skip);
  }
}

bool UMotorTest::estimateKandR(int motor, int idx[], float & estK, float & estR)
{
  int lowN = idx[2] - idx[1];
  int highN = idx[5] - idx[4];
  float dtLow = float(mLog[idx[2]].mTime10us - mLog[idx[1]].mTime10us)/100000.0;
  float dtHigh = float(mLog[idx[5]].mTime10us - mLog[idx[4]].mTime10us)/100000.0;
  // debug
//   const int MSL = 200;
//   char s[MSL];
//   snprintf(s, MSL, "# calc K and R init: lowIdx=%d, %d, %d; highIdx=%d, %d, %d\n", idx[0], idx[1], idx[2], idx[3], idx[4], idx[5]);
//   usb.send(s);
//   snprintf(s, MSL, "# calc K and R init: Nlow=%d, Nhigh=%d, dtLow=%.3f, dtHigh=%.3f\n", lowN, highN, dtLow, dtHigh);
//   usb.send(s);
  // debug end
  // test sanity for motor voltage 0
  //
  float sumCurLow = 0;
  float sumCurHigh = 0;
  UMotorTestMeasurementData * pd = &mLog[idx[1]];
  for (int i = 0; i < lowN; i++)
    sumCurLow += pd++->mCurrent[motor];
  meanCurLow[motor] = sumCurLow / lowN;
  pd = &mLog[idx[4]];
  for (int i = 0; i < highN; i++)
    sumCurHigh += pd++->mCurrent[motor];
  meanCurHigh[motor] = sumCurHigh / highN;
  // debug
//   snprintf(s, MSL, "# calc K and R vel: sumCurLow=%.3f, sumCurHigh=%.3f, curLow=%.3fA, curHigh=%.3f A\n",
//            sumCurLow, sumCurHigh, meanCurLow[motor], meanCurHigh[motor]);
//   usb.send(s);
  // debug end
  // velocity in rad/s
  uint32_t encStart = encoderStart[motor];
  float app = M_PI * 2.0f / float(encoder.pulsPerRev); // angle per encoder pulse
  if (motor == 0)
    app *= -1;
  float enc1 = (int32_t(mLog[idx[1]].mEncoder[motor] - encStart)) * app;
  float enc2 = (int32_t(mLog[idx[2]].mEncoder[motor] - encStart)) * app;
  float enc4 = (int32_t(mLog[idx[4]].mEncoder[motor] - encStart)) * app;
  float enc5 = (int32_t(mLog[idx[5]].mEncoder[motor] - encStart)) * app;
  float dradLow  = enc2 - enc1;
  float dradHigh = enc5 - enc4;
  velLow[motor] = dradLow / dtLow;
  velHigh[motor] = dradHigh / dtHigh;
  // debug
//   snprintf(s, MSL, "# calc K and R vel: dRadLow=%.1f, dRadHigh=%.1f, velLow=%.3f r/s, velHigh=%.3f r/s\n",
//            dradLow, dradHigh, velLow[motor], velHigh[motor]);
//   usb.send(s);
  // debug end
  float voltLow = mLog[idx[1]].mVoltage[motor];
  float voltHigh = mLog[idx[4]].mVoltage[motor];
  if (voltHigh * velHigh[motor] < 0.0 and motor == 0)
  { // these should have same sign
    // otherwise encoder is reversed (encoder A and B swapped)
    encoderReversed = not encoderReversed;
    usb.send("# encoder reversed is fixed, run test again\n");
    return false;
  }
  /**
   % % calculation                              *
   lcurRel = lHighCur/llowCur
   lhvdif = lHighvel - llowvel * lcurRel
   lK = (lHighVolt - lLowVolt * lcurRel)/lhvdif
   rcurRel = lHighCur/llowCur
   rhvdif = rHighvel - rlowvel * rcurRel
   rK = (rHighVolt - rLowVolt * rcurRel)/rhvdif   * */
  float curRel = meanCurHigh[motor] / meanCurLow[motor];
  float velRelDif = velHigh[motor] - velLow[motor] * curRel;
  estK = (voltHigh - voltLow * curRel)/velRelDif;
  // debug
//   snprintf(s, MSL, "# calc K and R vel: vL=%f, vH=%f, curRel=%.3f, velRelDif=%.3f, estK=%.4f\n",
//            voltLow, voltHigh, curRel, velRelDif, estK);
//   usb.send(s);
  // debug end
  estR = (voltHigh - velHigh[motor] * estK) / meanCurHigh[motor];
  // debug
//   snprintf(s, MSL, "# calc K and R vel: R=%.3f, OK=%d\n", estR, estR > 0 and estK > 0);
//   usb.send(s);
  // debug end
  return estR > 0 and estK > 0;
}


bool UMotorTest::estimateRandL(int motor, int idx[3], float& estK, float& estR)
{ // Use the initial acceleration with high current
  // to estimate L and re-estimate R
  //
  // hmmm ... need to think about this
  //
  return true;
}


bool UMotorTest::estimateBandS(int motor, int idx[], float & estB, float & estS)
{ // dunamic friction
  estB = (meanCurHigh[motor] - meanCurLow[motor]) * mKonstant[motor] / (velHigh[motor] - velLow[motor]);
  // static friction (Nm)
  estS = fabsf(meanCurHigh[motor] * mKonstant[motor] - velHigh[motor] * estB);
  //
//   // debug
//   const int MSL = 200;
//   char s[MSL];
//   snprintf(s, MSL, "# calc B and S init: cur*K=%g Nm, dyn friction = %g Nm\n", meanCurHigh[motor] * mKonstant[motor], velHigh[motor] * estB);
//   usb.send(s);
//   snprintf(s, MSL, "# calc B and S init: dynamic friction=%g Nm/(rad/sec), Static friction limit = %g Nm\n", estB, estS);
//   usb.send(s);
//   // debug end
  return estB > 0 and estS > 0;
}

bool UMotorTest::estimateJ(int motor, int idx[], float & estJ)
{
  int nLow = idx[1] - idx[0];

  float a1, a2, a3, da; // rad
  float dt; // sec
  uint32_t t1, t2, t3; // 10us
  float w; // rad/s
  float j = 0; // kg m^2
  float cur; // Amps
  float tw; // torque for acceleration
  float k = mKonstant[motor];
  float bf = mFricDyn[motor];
  float sf = mFricStat[motor];
  uint32_t encStart = encoderStart[motor];
  float app = M_PI * 2.0f / float(encoder.pulsPerRev); // angle per encoder pulse
  if (motor == 0)
    app *= -1;
  // set start sample pointer
  UMotorTestMeasurementData * pd = &mLog[idx[0] - 1];
  a1 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
  t1 = pd->mTime10us - logStartTime;
  pd++;
  a2 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
  t2 = pd->mTime10us - logStartTime;
  // debug
//   float t0 = float(t2)/100000.0; // start time (sec)
//   const int MSL = 200;
//   char s[MSL];
//   snprintf(s, MSL, "# estimate J: t0=%f, t1=%f, t2=%f (sec)\n", t0, t1/100000.0f, t2/100000.0f);
//   usb.send(s);
//   snprintf(s, MSL, "# estimate J: t1=%lu, t2=%lu, t3=%lu (10us)\n", pd[-1].mTime10us, pd[0].mTime10us, pd[1].mTime10us);
//   usb.send(s);
  // debug end
  // integrate energy into inertia
  for (int i = 0; i < nLow; i++)
  { // current in this sample
    cur = pd->mCurrent[motor];
    // use next and previous sample to calculate
    // velocity for this sample.
    pd++;
    a3 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
    da = a3 - a1;
    t3 = pd->mTime10us - logStartTime;
    dt = float(t3 - t1) / 100000.0;
    w = da/dt;
    // integrate this sample:
    // torque after static friction
    if (cur > 0.0)
    {
      tw = cur * k - sf;
      if (tw > 0)
        // more than static friction
        j += tw - bf * w;
    }
    else
    {
      tw = cur * k + sf;
      if (tw < 0)
        // more than static friction
        j += tw - bf * w;
    }
    // debug
//     if (i % 30 == 0)
//     {
//       snprintf(s, MSL, "# estimate J: i=%d, da=%.3f, dt=%.3f, w=%f, cur=%.3f, tw=%g, j=%g\n", i, da, dt, w, cur, tw, j);
//       usb.send(s);
//     }
    // debug end
    // iterate
    t1 = t2;
    t2 = t3;
    a1 = a2;
    a2 = a3;
  }
  estJ = j / velLow[motor];
  bool isOK = estJ > 0;
  // debug
//   snprintf(s, MSL, "# estimate J: j=%g, estJ=%g, velLow=%g\n", j, estJ, velLow[motor]);
//   usb.send(s);
  // debug end
  return isOK;
}


void UMotorTest::estimateMotorParams()
{
  if (mLog == nullptr or mLogIndex == 0)
    return;
  const int MSL = 300;
  char s[MSL];
  // reset values
  for (int i = 0; i < 2; i++)
  { // set dummy values, if test fails
    mKonstant[i] = 1.0;
    mResistance[i] = 0.1;
    mInductance[i] = 0.001;
    mFricDyn[i] = 1.0;
    mFricStat[i] = 1.0;
    mInertia[i] = 1.0;
  }
  // save start time for easy debug
  logStartTime = mLog[0].mTime10us;
  // steady state
  bool isOK = estimateKandR(0, leftTestIndex, mKonstant[0], mResistance[0]);
  if (isOK)
    isOK = estimateRandL(0, leftTestIndex, mKonstant[0], mResistance[0]);
  if (isOK)
    isOK = estimateBandS(0, leftTestIndex, mFricDyn[0], mFricStat[0]);
  if (isOK)
    isOK = estimateJ(0, leftTestIndex, mInertia[0]);
  if (not isOK)
    usb.send("# estimate failed for motor 0\n");
  // save result
  testValidLeft = isOK;
  //
  if (true)
    isOK = estimateKandR(1, rightTestIndex, mKonstant[1], mResistance[1]);
  if (isOK)
    isOK = estimateRandL(1, rightTestIndex, mKonstant[1], mResistance[1]);
  if (isOK)
    isOK = estimateBandS(1, rightTestIndex, mFricDyn[1], mFricStat[1]);
  if (isOK)
    isOK = estimateJ(1, rightTestIndex, mInertia[1]);
  if (not isOK)
    usb.send("# estimate failed for motor 1\n");
  // save result
  testValidRight = isOK;
  //
  snprintf(s, MSL, "# UMotorTest:: encoder reversed=%d, Low voltage=%g, High voltage=%g\n",
           encoderReversed, voltageLow * voltageCCV, voltageHigh * voltageCCV);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: R=%g Ohm, K=%g V/(rad/s) or Nm/A - left motor)\n",
           mResistance[0], mKonstant[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: R=%g Ohm, K=%g V/(rad/s) or Nm/A - right)\n",
           mResistance[1], mKonstant[1]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: friction: static=%g Nm, dynamic=%g Nm/(rad/sec) - left\n",
           mFricStat[0], mFricDyn[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: friction: static=%g Nm, dynamic=%g Nm/(rad/sec) - right\n",
           mFricStat[1], mFricDyn[1]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: inertia=%g kg m^2 - left\n",
           mInertia[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: inertia=%g kg m^2 - right\n",
           mInertia[1]);
  usb.send(s);
}

void UMotorTest::getMotorTestLog()
{
  usb.send("% Motor test log\n");
  usb.send("% 1 time stamp (sec)\n");
  usb.send("% 2 Motor test type (1 = CV, -1 = CCV)\n");
  usb.send("% 3 sample number\n");
  usb.send("% 4 Left motor voltage\n");
  usb.send("% 5 Left motor current\n");
  usb.send("% 6 Left motor position (rad)\n");
  usb.send("% 7 Right motor voltage\n");
  usb.send("% 8 Right motor current\n");
  usb.send("% 9 Right motor position (rad)\n");
  if (mLogIndex == 0 or mLog == nullptr)
  {
    usb.send("%\n% Log empty\n");
  }
  else
  {
    motorTestGetIndex = 0;
    motorTestGetLog = true;
    logStartTime = mLog[0].mTime10us;
  }
}

void UMotorTest::sendTestLog()
{
  UMotorTestMeasurementData * ml = &mLog[motorTestGetIndex];
  const int MSL = 300;
  char s[MSL];
  snprintf(s, MSL, "%.4f %d %.0f %.2f %.3f %.2f  %.2f %.3f %.2f\n",
           float(ml->mTime10us - logStartTime) / 100000.0, motorTestGetIndex, voltageCCV,
           ml->mVoltage[0], ml->mCurrent[0], (-int32_t(ml->mEncoder[0] - encoderStart[0])) / float(encoder.pulsPerRev) * M_PI * 2.0f ,
           ml->mVoltage[1], ml->mCurrent[1], (int32_t(ml->mEncoder[1] - encoderStart[1])) / float(encoder.pulsPerRev) * M_PI * 2.0f
           );
  usb.send(s);
}
