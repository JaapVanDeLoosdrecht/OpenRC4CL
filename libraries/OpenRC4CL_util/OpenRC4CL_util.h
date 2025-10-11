/* 
Utils for OpenRC4CL 11 October 2025

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

const char *OpenRC4CL_VERSION = "0.0.6";

struct TxData { int checkSum; int id; int throttle; int chan1; }; 
inline int CheckSum(struct TxData &d) { return d.id ^ d.throttle ^ d.chan1; } 

struct Telemetry { int checkSum; int id; int vBatLow; int vBat; int rsi; int time_left; int totalLost; };
inline int CheckSum(struct Telemetry &t) { return t.id ^ t.vBatLow ^ t.vBat ^ t.rsi ^ t.time_left ^t.totalLost; }

const int TxMinPulse = 1000;  // us
const int TxMaxPulse = 2000;
const int TxMidPulse = TxMinPulse + (TxMaxPulse - TxMinPulse) / 2;
const int ThrHoldDelta = 100;  
const int TxThrottleHoldPulse = TxMinPulse - ThrHoldDelta;

int avgAnalogMilliVolts(int pin, int nrSamples) {
  int sum = 0; for(int i = 0; i < nrSamples; i++) sum += analogReadMilliVolts(pin);
  return sum / nrSamples;
}

class BlinkLed {  // 0 hz is always on
public:
  BlinkLed(int pin, float hz) { pinMode(pin, OUTPUT); _pin = pin; setHz(hz); }
  void setHz(float hz) { ms = (hz != 0) ? (int)(1000 / hz) : 0; } 
  void update() { digitalWrite(_pin, ((ms == 0) || (millis() % ms <= ms / 2)) ? LOW : HIGH); }
private:
  int _pin, ms;
};

class PushButton {  // one end to GND, other end to digital input, no external pullup Rs
public:             // note: the two pins on each side, close to each other, are connected 
  PushButton(int pin) { _pin = pin; pinMode(_pin, INPUT_PULLUP); }
  bool read() { return !digitalRead(_pin); }   // LOW (pushed) or HIGH (not pushed)
private:
  int _pin;
};

class Switch {  // 3 (or 2) pos switch, middle to GND, left and right to digital inputs, no external pullup Rs
public:         // NOTE: fallback is middle for both 3 pos and 2 pos if switch becomes disconnected! 
  enum Pos {middle, left, right};  // NOTE: 2 pos has {middle, left} 
  Switch(int pinLeft, int pinRight=-1) {
	leftP = new PushButton(pinLeft);
    if (pinRight >= 0) { rightP = new PushButton(pinRight); tab = tab3; }
  }
  Pos read() { int lt = leftP->read(); return (Pos)(rightP ? lt + 2 * rightP->read() : lt); }
  int readTx() { return tab[this->read()]; }
private:
  static constexpr int tab3[3] = {TxMidPulse, TxMinPulse, TxMaxPulse}; 
  static constexpr int tab2[2] = {TxMinPulse, TxMaxPulse};
  const int *tab = tab2;
  PushButton *leftP, *rightP = 0;
};

class Potmeter {  // use 10K ohm and 3.3V (NOT 5V), ESP32-C6 analogReadMilliVolt is factory calibrated, analogRead returns raw ADC value.
public:
  Potmeter(int pin, int min=TxMinPulse, int max=TxMaxPulse, int nrSamples=16) { _pin = pin; _min = min; _max = max; _nrS = nrSamples; }
  int Read() { return map(ReadV(), 0, VRef, _min, _max); }
  int ReadV() { return avgAnalogMilliVolts(_pin, _nrS); }
private:
  const int VRef = 3350;  // note 50 mV above reference
  int _pin, _min, _max, _nrS;
};

class RcTimer { 
public:
  static const int TicksPerSec = 1000;
  RcTimer(int secs = 0) { start(_secs = secs); }
  void start(int secs = 0) { 
    if (secs > 0) _secs = secs; 
    end = (begin = millis()) + _secs * TicksPerSec;
  }
  bool elapsed() { return millis() > end; }
  int time() { return (millis() - begin); }
  int time2secs(int time) { return time / TicksPerSec; }
  int seconds() { return time2secs(time()); }
  int secondsLeft () {return _secs - seconds(); }
  void setEnd(int secs) { _secs = secs; end = begin + _secs * TicksPerSec; }  
  void log() { Serial.printf("[T] m:%d, t:%d, s:%d, b:%d, e:%d\n", millis(), seconds(), _secs, begin, end); } 
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
    _min = min; _max = max; _mid = mid; _fail = fail; _reverse = reverse, _last = (_fail >= 0) ? _fail : TxMidPulse;
	if (_reverse) {_min = max; _max = min;}
  }
  void writeTx(int txValue) { write(map(txValue + _mid, TxMinPulse, TxMaxPulse, _min, _max)); _last = txValue;}
  void write(int usValue) { servo.writeMicroseconds(usValue); }
  void failsafe() { int v = (_fail >= 0) ? _fail : _last; writeTx(v); }
  int read() { return servo.readMicroseconds(); }
  int failSaveValue() { return _fail; }
  bool reverse() { return _reverse; }
protected:
  Servo servo;
  bool _reverse;
  int _min, _max, _mid, _fail, _last; 
};

class Throttle : public RcServo { 
public:
  Throttle(int pin, RcTimer *rctimer=0, int minThrottle=0, int maxFlightSecs=0, 
           int warnEndFlightSecs=5, int nrWarns=2, bool reverse=false) : 
           RcServo(pin, TxMinPulse, TxMaxPulse, 0, reverse ? (TxMaxPulse+ThrHoldDelta) : (TxMinPulse-ThrHoldDelta), reverse) {
	timer = rctimer; lastThr = failSaveValue(); minThr = minThrottle; timerSet = minThrottle == 0; nrWarn = nrWarns;
    maxFlight = maxFlightSecs; warnEndFlight = warnEndFlightSecs; 
	setWarnTime();
  }
  int writeTx(int txValue) {
    int v = lastThr = txValue;
	if ((v == TxThrottleHoldPulse) || (timer->elapsed())) {
	  v = failSaveValue();
	} else {	
      if ((!timerSet) && (minThr > 0) && (v > minThr)) { timer->start(maxFlight); timerSet = true; }
      if (timerSet && (maxFlight > 0)) {    
        int t = timer->time();
        if ((t >= warnTime) && (t < endWarnTime) && (t % RcTimer::TicksPerSec < RcTimer::TicksPerSec/2)) 
          v = reverse() ? (_max - (_max-v) / 2) : (_min + (v-_min) / 2); 
	  }
	}
    RcServo::writeTx(v);
	return v;
  }
  void warnAndStop() { // initiate end warning and stop
    if (timer) {
      if (!stop) { 
        timer->setEnd(maxFlight = timer->seconds() + warnEndFlight);
 	    setWarnTime(); timerSet = true; stop = true; 
	  }
	  writeTx(lastThr);
	} else {
      if (!stop) stop = true;
	  if (stop) failsafe();
	}
  } 
private:
  RcTimer *timer;
  bool timerSet = false, stop = false;
  int lastThr, minThr, maxFlight, warnEndFlight, nrWarn, endWarn;      
  unsigned long warnTime, endWarnTime;  
  void setWarnTime() {  // set start and end warning time
    warnTime = (maxFlight - warnEndFlight) * RcTimer::TicksPerSec;
	endWarnTime = warnTime  + nrWarn * RcTimer::TicksPerSec;
  }
};

class RcPeer : public ESP_NOW_Peer {  
public:
  bool readyToSend = true;
  RcPeer(MacAddress mac_peer, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_peer, channel, iface, lmk) {}
  ~RcPeer() {}
  bool add_self() { return add(); }     // Note add is protected function
  bool send_data(const uint8_t *data, size_t len) { 
    if (!readyToSend) return false;
	return !(readyToSend = !send(data, len) > 0);
  }
  void onSent(bool success) { readyToSend = true; }
};

#endif
