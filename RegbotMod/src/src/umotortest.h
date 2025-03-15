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

#ifndef UMOTORTEST_H
#define UMOTORTEST_H

#include <stdint.h>
#include "main.h"
#include "usubss.h"

//////////////////////////////////////////////////////////////////

class UMotorTestMeasurementData
{
public:
  uint32_t mTime10us;
  float mVoltage[2];
  float mCurrent[2];
  uint32_t mEncoder[2];
//   uint32_t adCurrent[2];
//   uint32_t ad1[2];
};


///////////////////////////////////////////////////////////////////


class UMotorTest : public USubss
{
public:
  /**
  * setup */
  void setup();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf) override;
  /**
   * update every sample */
  void tick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * send current motor values (voltage and reference settings)
   * */
  void sendMotorParameters();
  void sendMotortestSetup();

  bool motorTestRunning = false;

  void getMotorTestLog();

  int mLogIndex = 0;

  //
  // is encoder A and B
  // positive voltage should go forward,
  // i.e. for positive voltage on both motors (using motv)
  // left motor should count down and right motor up
  //
  // moved from Encoder
  bool encoderReversed = false;

  const char * versionh = "$Id: umotortest.h 1699 2024-11-26 16:41:47Z jcan $";
  const char * versioncpp = nullptr;

protected:
  
  void sendData(int item) override;
  
  void estimateMotorParams();

  bool estimateKandR(int motor, int idx[3], float & estK, float & estR);

  bool estimateRandL(int motor, int idx[3], float & estK, float & estR);

  bool estimateBandS(int motor, int idx[], float & estB, float & estS);

  bool estimateJ(int motor, int idx[], float & estJ);

  void sendTestLog();
  
private:
  //
  int tickCnt = 0;
  int setupCnt = 0;
  uint32_t logStartTime = 0;
  uint32_t encoderStart[2] = {0};
//   bool invertedEncoder[2] = false;

//   typedef enum {MT_CV, MT_CCV, MT_STOP} TestType;
//   TestType motorTestType = MT_STOP;
  bool motorTestEnd = false;
  bool testValidLeft = false;
  bool testValidRight = false;

  UMotorTestMeasurementData * mLog = nullptr;
  int mLogIndexMax = 0;
  bool motorTestGetLog = false;
  int motorTestGetIndex = 0;

  typedef enum {MS_OFFSET, MS_OFF, MS_LLOW, MS_LHIGH, MS_LEND, MS_RLOW, MS_RHIGH, MS_ROLL_OFF, MS_END} TestState;
  TestState testState = MS_OFF;
  int stateLength = 1000;    // state period in ms at sample time 1ms
  int steadyStateTime = 200; // time to stabalize in ms at sample time 1ms
  float voltageLow = 3;
  float voltageHigh = 9;
  float voltageCCV = 1.0; // -1=CCV, 1=CV
  /** leftTestIndex indexing
   * 0= start low, 1=start of steady state, 2 = end of low
   * 3= start high, 4=start of steady state, 5 = end of high
   */
  int leftTestIndex[6] ={0};
  int rightTestIndex[6] ={0};
  /**
   * partial results for left and right motor */
  float meanCurLow[2] = {0};
  float meanCurHigh[2] = {0};
  float velLow[2] = {0};
  float velHigh[2] = {0};
  /** Parameters index
   * 0 = left CV,  1 = right CV
   * 2 = left CCV, 3 = right CCV
  / */
  float mResistance[2] = {0};
  float mInductance[2] = {0};
  float mKonstant[2] = {0};
  float mFricDyn[2] = {0};
  float mFricStat[2] = {0};
  float mInertia[2] = {0};
};

extern UMotorTest motortest;

#endif // UMOTOR_H
