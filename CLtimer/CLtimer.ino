/* 
CL Timer OpenRC4CL 25 October 2025

MIT license

Copyright 2025 Jaap van de Loosdrecht

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files (the “Software”), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// CLtimer com7 white usb

#include "OpenRC4CL_util.h"

const int maxFlight = 7*60;      // seconds
const int warnEndFlight = 6;     // seconds before max
const int nrWarns = 2;           // numver of warnings

const int pinLed = LED_BUILTIN;  // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinMaxTime = A1;  
const int pinMaxThrottle = A2;
const int pinPushButton = D3;

class CLtimer {  
public:
  static const int NR_COUNTS = 10;
  CLtimer() { throttle.failsafe(); }
  void update() {
    int thr = throttle.failSaveValue();
    if (!start) {
      int mt = maxTime.Read();
      if (abs(mt-maxSecs) > 5) { throttle.resetTimer(maxSecs = mt); }
      int thrM = maxThrottle.Read();
      if (abs(thrM-thrValue) > 5) thrValue = thrM;
      if (button.read()) { timer.start(maxSecs); start = true; led.set(StatusOk); header = "[Timer]"; }
    } else {
      thr = throttle.writeTx(thrValue);
    }
    if (timer.elapsed()) led.set(StatusEndFlight);
    led.update();
    if (++count >= NR_COUNTS) {  
      Serial.printf("%s t:%d, thr:%d, thrV:%d, sec:%d, \n", header, (start) ? maxSecs-timer.seconds() : 0, thr, thrValue, maxSecs);
      count = 0;
    }
  }
private:
  PushButton button{pinPushButton};
  int maxSecs = maxFlight;
  Potmeter maxTime{pinMaxTime, 10, maxSecs};
  RcTimer timer{0};
  Throttle throttle{pinThrottle, &timer, 0, maxSecs, warnEndFlight, nrWarns};
  Potmeter maxThrottle{pinMaxThrottle, TxMinPulse, TxMaxPulse};
  Led led{pinLed, StatusWaitStart, true};
  bool start = false;
  int thrValue = -1, count = -1;
  char *header = "[Params]";
};

CLtimer timer;

void setup() {
  Serial.begin(115200);
  Serial.printf("Start CLtimer, OpenRC4CL %s\n", OpenRC4CL_VERSION);
}

void loop() {
  timer.update();
  delay(100);
}
