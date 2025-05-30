/***************************************************************************
 *   Copyright (C) 2019-2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
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

#ifndef UCOMMAND_H
#define UCOMMAND_H

#include <string.h>
#include <stdlib.h>
#include <HardwareSerial.h>
// to avoid conflict in ADC library that define abs and round as macro
// but in conflict with other definitions.
#ifdef round
#undef round
#endif
#ifdef abs
#undef abs
#endif
#include <mutex>
#include "usubss.h"


class UCommand : public USubss
{
public:
  void setup(); 
  /**
  * Get revision number from SVN annotation */
  uint16_t getRevisionNumber();
  /**
  * Parse commands from the USB connection and implement those commands.
  * \param buf is string to send
  * The function is served by the main loop, when time allows. */
  void parse_and_execute_command(char *buf);
  /**
   * at every sample time */
  void tick();
  /**
   * Send command help */
  void sendHelp() override;
  
  bool decode(const char * buf) override;
  
  const char * versionh = "$Id: ucommand.h 1698 2024-11-26 16:39:58Z jcan $";
  const char * versioncpp = nullptr;

protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;
 
private:
  /**
   * Send version string to client */
  void sendStatusVersion();
  /**
   * send to USB cvhannel 
   * \param str is string to send
   * \param n is number of bytes to send
   * \param blocking if false, then send if space only, else don't return until send
   */
  const char * getCompileDate();
  
  static const int MCDL = 100;
  char compileDate[MCDL];
};

extern UCommand command;

#endif
