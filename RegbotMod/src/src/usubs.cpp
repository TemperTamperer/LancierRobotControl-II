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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "usubs.h"
#include "uusb.h"
#include "urobot.h"

/**
 * Workaround for linker error
 * https://forum.arduino.cc/t/arduino-due-warning-std-__throw_length_error-char-const/308515
 * */
namespace std {
  void __throw_length_error(char const*) {
  }
}

USubs::USubs(const char * key, const char * help)
{
  msgKey = key;
  helpText = help;
  keySize = strlen(msgKey);
}

bool USubs::decode(const char * keyLine, bool newSubscription)
{
  bool used = false;
  if (newSubscription)
  {
    if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == ' ')
    {
  //     usb.send("# USubs:: set subscription\n");
      int n = strtol(&keyLine[keySize],nullptr, 10);
      // convert from ms to tics
      subN = int(n/(robot.SAMPLETIME * 1000));
      if (n > 0 and subN == 0)
        // requested faster than sample time, so get sample time.
        subN = 1;
      used = true;
    }
  }
  else if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == 'i')
  { // one-time request for data
    used = true;
    dataRequest = true;
  }
  return used;
}


bool USubs::tick()
{
  bool isTime = dataRequest;
  if (dataRequest)
    dataRequest = false;
  else
  { // no single request, so check for subscriptions
    if (subN > 0 and (int32_t(hb10us - sendTime)) >= subN * 100)
    {
      isTime = true;
      sendTime = hb10us;
    }
//     else if (subN > 0)
//     { // debug
//       serviceStatus();
//     }
  }
  return isTime;
}


void USubs::serviceStatus()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "# USubs::serviceStatus: dataReq=%d, sendTime=%ld, subN=%d, dt=%ld\n",
           dataRequest, (sendTime/100), subN, (int32_t(hb10us - sendTime)/100));
  usb.send(s);
}


void USubs::sendHelpLine()
{
  const int MSL = 600;
  char s[MSL];
  snprintf(s, MSL, "# -- \t%si and 'sub %s N' \t%s\r\n", msgKey, msgKey, helpText);
//   snprintf(s, MSL, "#   %si and 'sub %s N', %s\r\n", msgKey, msgKey, helpText);
  usb.send(s);
}

void USubs::sendPublishList(int & listNum)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "pub %d %s %s\r\n", listNum++, msgKey, helpText);
  usb.send(s);
}
