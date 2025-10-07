/* 
Utils for OpenRC4CL 4 October 2025

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

#ifndef OpenRC4CL_util
#define OpenRC4CL_util

#include <ESP32_NOW.h>
#include <MacAddress.h>
#include <WiFi.h>
#include <ESP32Servo.h>

const char *OpenRC4CL_VERSION = "V0.03";

struct TxData { int checkSum; int id; int throttle; int chan1; }; 
inline int CheckSum(struct TxData &d) { return d.id ^ d.throttle ^ d.chan1; } 

struct Telemetry { int checkSum; int id; int vBatLow; int vBat; int rsi; int max_time; };
inline int CheckSum(struct Telemetry &t) { return t.id ^ t.vBatLow ^ t.vBat ^ t.rsi ^ t.max_time; }

const int TxMinPulse = 1000;  // us
const int TxMaxPulse = 2000;
const int TxMidPulse = TxMinPulse + (TxMaxPulse - TxMinPulse) / 2;
const int TxThrottleHoldPulse = TxMinPulse - 100;

class RcTimer { 
public:
  static const int TicksPerSec = 1000;
  RcTimer(int secs) { start(_secs = secs); }
  void start(int secs = 0) { 
    begin = millis();
    if (secs > 0) _secs = secs; 
    end = begin + _secs * TicksPerSec;
  }
  bool elapsed() { return millis() > end; }
  int time() { return (millis() - begin); }
  int time2secs(int time) { return time / TicksPerSec; }
  int seconds() { return time2secs(time()); }
  void setEnd(int time) { end = time; }  
  void log() { Serial.printf("[T] t:%d, s:%d, b:%d, e:%d\n", seconds(), _secs, begin, end); } 
private:
  int _secs;
  unsigned long begin, end;
};

class RcServo {  // Note in standard RC connector the middle connector is always Vcc
public:
  RcServo(int pin, int min=TxMinPulse, int max=TxMaxPulse, int mid=0, int fail=-1, bool reverse=false) {  // -1 = last position
    set(min, max, mid, fail, reverse);
    servo.attach(pin, min, max);
	failsafe();
  }
  ~RcServo() { servo.detach(); }
  void set(int min=TxMinPulse, int max=TxMaxPulse, int mid=0, int fail=-1, bool reverse=false) { 
    _min = min; _max = max; _mid = mid; _fail = fail;
	if (reverse) {_min = max; _max = min;}
  }
  void writeTx(int txValue) { write(map(txValue + _mid, TxMinPulse, TxMaxPulse, _min, _max)); }
  void write(int usValue) { servo.writeMicroseconds(usValue); }
  void failsafe() { if (_fail >= 0) writeTx(_fail); }
  int read() { return servo.readMicroseconds(); }
  int failSaveValue() { return _fail; }
protected:
  Servo servo;
  int _min, _max, _mid, _fail;
};

class Throttle : public RcServo { 
public:
  Throttle(int pin, RcTimer *rctimer=0, int minThrottle=0, int maxFlightSecs=0, 
           int warnEndFlightSecs=5, int nrWarns=2, bool reverse=false) : 
           RcServo(pin, TxMinPulse, TxMaxPulse, 0, TxMinPulse-100, reverse) {
	timer = rctimer;
    minThr = minThrottle;
    timerSet = minThrottle == 0;
    maxFlight = maxFlightSecs;         
	warnEndFlight = warnEndFlightSecs; 
	nrWarn = nrWarns;
	setWarn();
  }
  int writeTx(int txValue) {
    int v = txValue;
    if ((!timerSet) && (minThr > 0) && (v > minThr)) {
      timer->start(maxFlight);
      timerSet = true;
    }
    if (timerSet && (maxFlight > 0)) {    
      if (timer->elapsed()) { RcServo::failsafe(); return failSaveValue(); }
      int t = timer->time();
      if ((t >= warnTime) && (t < endWarnTime) && (t % RcTimer::TicksPerSec < RcTimer::TicksPerSec/2)) 
		if (_min < _max)
          v = _min + (v-_min) / 2;  // normal
		else
          v = _max - (_max-v) / 2;  // reverse
    }
    RcServo::writeTx(v);
	return v;
  }
  void WarningNow() { 
	timerSet = true;
	warnTime = millis();
	endWarnTime = warnTime + nrWarn * RcTimer::TicksPerSec;
	timer->setEnd(warnTime + warnEndFlight * RcTimer::TicksPerSec);
  } 
  void resetTimer(int maxFlightSecs) {
	maxFlight = maxFlightSecs;
    timer->start(maxFlight);
    timerSet = true;
	setWarn();
  }
private:
  RcTimer *timer;
  bool timerSet;
  int minThr, maxFlight, warnEndFlight, nrWarn, endWarn;      
  unsigned long warnTime, endWarnTime;  
  void setWarn() {
    this->warnTime = (maxFlight - warnEndFlight) * RcTimer::TicksPerSec;
	this->endWarnTime = this->warnTime  + nrWarn * RcTimer::TicksPerSec;
  }
};

class PushButton {  // one end to GND, other end to digital input, no external pullup Rs
public:             // note: the two pins on each side, close to each other, are connected 
  PushButton(int pin) { _pin = pin; pinMode(_pin, INPUT_PULLUP); }
  bool read() { return !digitalRead(_pin); }   // LOW (pushed) or HIGH (not pushed)
private:
  int _pin;
};

class Switch {  // 3 (or 2) pos switch, middle to GND, left and right to digital inputs, no external pullup Rs
public:         // NOTE: fallback is middle (3 pos) or right (2 pos) if switch becomes disconnected! 
  enum Pos {middle, left, right};
  Switch(int pinLeft, int pinRight=-1) {
	leftP = new PushButton(pinLeft);
    if (pinRight >= 0) rightP = new PushButton(pinRight); else rightP = 0;
  }
  Pos read() { 
    int res = (int)(leftP->read());
    if (rightP) res += 2 * (int)(rightP->read());
    return (Pos)res; 
  }
  int readTx() {
	static const int tab3[] = {TxMidPulse, TxMinPulse, TxMaxPulse}; 
	static const int tab2[] = {TxMinPulse, TxMaxPulse};
    Pos p = this->read();	
	if (rightP) return tab3[p]; else return tab2[p];
  }
private:
  PushButton *leftP, *rightP;
};

class Potmeter {  // use 10K ohm and 3.3V (NOT 5V)
public:
  Potmeter(int pin, int min=TxMinPulse, int max=TxMaxPulse) { _pin = pin; _min = min; _max = max; }
  int Read() { return map(ReadV(), 0, vref, _min, _max); }  // ESP32-C6 analogReadMilliVolt is factory calibrated, analogRead returns raw ADC value.
  int ReadV() { return analogReadMilliVolts(_pin); }
private:
  const int vref = 3350; 
  int _pin, _min, _max;
};

class RcPeer : public ESP_NOW_Peer {  
public:
  bool readyToSend = true;
  RcPeer(MacAddress mac_peer, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_peer, channel, iface, lmk) {}
  ~RcPeer() {}
  bool add_self() { return add(); }     // Note add is protected function
  bool send_data(const uint8_t *data, size_t len) { 
    if (!readyToSend) return false;
    readyToSend = !send(data, len) > 0; 
    return !readyToSend;
  }
  void onSent(bool success) { readyToSend = true; }
};

#endif
