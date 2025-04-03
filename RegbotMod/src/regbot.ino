/***************************************************************************
*   Copyright (C) 2019-2024 by DTU                             *
*   jcan@dtu.dk                                                *
*
*   Main function for small regulation control object (regbot)
*   build for Teensy 4.1,
*   intended for 31300/1 Linear control 1 (now 34721/34722)
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


#include <malloc.h>
#include <IntervalTimer.h>

#include "src/main.h"
#include "src/ulog.h"
#include "src/umission.h"
#include "src/ulinesensor.h"
#include "src/ueeconfig.h"
#include "src/wifi8266.h"
#include "src/uservo.h"

#include "src/uusb.h"
#include "src/urobot.h"
#include "src/uencoder.h"
#include "src/umotor.h"
#include "src/umotortest.h"
#include "src/uad.h"
#include "src/ucurrent.h"
#include "src/uirdist.h"
#include "src/uimu2.h"
#include "src/udisplay.h"
#include "src/uusbhost.h"

#include "src/RadioCommunication.h"

#include <RFM69.h>              // https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>          // https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>     

#define NODEID        2    // The ID of this node (must be different for every node on network)
#define NETWORKID     100  // The network ID
#define FREQUENCY      RF69_433MHZ
#define SERIAL_BAUD   57600
#define RF69_SPI_CS   6
#define RF69_IRQ_PIN  33
String targetID = ":90";

float distance;
float heading;

String message;
String segment;

int dIndex;
int hIndex;

String distanceStr;
String headingStr;

void processIncomingMessage();

RFM69 radio(RF69_SPI_CS, RF69_IRQ_PIN, false);

// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period - typically 1ms)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
volatile uint32_t tsec = 0; /// time that will not overrun
volatile uint32_t tusec = 0; /// time that will stay within [0...999999]
float missionTime = 0; // time in seconds
// flag for start of new control period
volatile bool startNewCycle = false;
int pushHBlast = 0;
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///
const char * versioncpp = "$Id: regbot.ino 1704 2024-12-22 16:46:18Z jcan $";

// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  ad.setup();
  robot.setup();
  robot.setStatusLed(HIGH);
  usb.setup();
  command.setup();
  encoder.setup();
  ls.setup();
  irdist.setup();
  userMission.setup();
  control.setup();
  imu2.setup();
  usbhost.setup();
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr,10 ); // heartbeat timer, value in usec
  // data logger init
  logger.setup();
  logger.setLogFlagDefault();
  logger.initLogStructure ( 100000 / robot.CONTROL_PERIOD_10us );
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.setup();
  // configuration changes setup
  current.setup();
  servo.setup();  // set PWM for available servo pins
  motor.setup();  // set motor pins
  motortest.setup();
  display.setup();
  // start heartbeat to USB
  robot.decode("sub hbt 400\n");
  robot.setStatusLed(LOW);

  //Sets radio values
  Serial.begin(SERIAL_BAUD);

  // Initialize the radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  Serial.println("Setup complete");
}

int debugSaved = 0;
/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop ( void )
{
  control.resetControl();
  bool cycleStarted = false;
  robot.setStatusLed(LOW);
  // - listen for incoming commands
  //   and at regular intervals (1ms)
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  while ( true ) 
  { // main loop
    if (radio.receiveDone()) {
      processIncomingMessage();
      // Serial.println("message recieved");
    }else{
      // Serial.println("Couldnt find message");
    }

    usb.tick(); // service commands from USB
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle ) // start of new control cycle
    { // error detect
      startNewCycle = false;
      cycleStarted = true;
      // AD converter should start as soon as possible, to also get a reading at half time
      // values are not assumed to change faster than this
      ad.tick();
      // robot.timing is to get some statistics on which part uses the CPU time
      robot.timing(1);
      // estimate velocity and pose
      encoder.tick();
      // calculate motor current
      current.tick();
      // net new acc/gyro measurements
      imu2.tick();
      // record read sensor time
      robot.timing(2);
      // calculate sensor-related values
      // process line sensor readings and
      // estimate line edge posiitons
      //ls.tick();
      //// distance sensor (sharp sensor)
      irdist.tick();
      //// advance mission
      userMission.tick();
      // do control
      control.tick();
      // Implement on actuators
      servo.tick();
      motor.tick();
      // optional, summarize for motor parameter estimate
      motortest.tick();
      // monitor robot state
      robot.tick();
      // record read sensor time + control time
      robot.timing(3);
      // non-critical functions
      // save selected log data to RAM buffer
      logger.tick();
      // update display
      display.tick();
      // service USB-host plug
      usbhost.tick();
    }
    // loop end time
    if (cycleStarted)
    { // mostly timing (total sample timing)
      robot.timing(4);
      robot.saveCycleTime();
      cycleStarted = false;
    }
  }
}

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{ // as basis for all timing
  hb10us++;
  tusec += 10;
  if (tusec > 1000000)
  {
    tsec++;
    tusec = 0;
  }
  if ( hb10us % robot.CONTROL_PERIOD_10us == 0 ) // main control period start
  { // time to start new processing sample
    userMission.missionTime += 1e-5 * robot.CONTROL_PERIOD_10us;
    hbTimerCnt++;
    startNewCycle = true;
    robot.timing(0);
  }
  if ( int(hb10us % robot.CONTROL_PERIOD_10us) == robot.CONTROL_PERIOD_10us/2 ) // start half-time ad conversion
  { // Time to read a LEDs off value (and turn LEDs on for next sample)
    ad.tickHalfTime();
  }
}


void processIncomingMessage() {
  if (radio.DATALEN > 0 && radio.DATA != nullptr) {
    // Create a String from the incoming data
    message = String((char*)radio.DATA);

    // Find the target ID in the message
    int startIndex = message.indexOf(targetID);
    if (startIndex != -1) {
      // Find the position of the next space after the target ID
      int endIndex = message.indexOf(' ', startIndex);
      if (endIndex == -1) {
        endIndex = message.length(); // If no space is found, set to end of message
      }
      segment = message.substring(startIndex, endIndex);

      // Serial.println(segment);

      // Parse the distance and heading values
      dIndex = segment.indexOf('D');
      hIndex = segment.indexOf('H');

      if (dIndex != -1 && hIndex != -1) {
        distanceStr = segment.substring(dIndex + 1, hIndex);
        headingStr = segment.substring(hIndex + 1);

        distance = distanceStr.toFloat();
        heading = headingStr.toFloat();
        
        // Output the extracted values
        Serial.print("Robot ID: ");
        Serial.println(targetID.substring(1)); // Remove the colon for display
        Serial.print("Distance: ");
        Serial.println(distance);
        Serial.print("Heading: ");
        Serial.println(heading);
      } else {
        Serial.println("Error: Invalid segment format.");
      }
    } else {
      Serial.println("Target ID not found in message.");
    }
  }
}