/* 
OpenRC4CL V0.01 29Sep25

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

#ifndef OpenRC4CL_util_h
#define OpenRC4CL_util_h

#include "ESP32_NOW.h"
#include "MacAddress.h"
#include "WiFi.h"
#include <ESP32Servo.h>

struct TxData { int checkSum; int id; int throttle; int chan1; }; 
inline int CheckSum(struct TxData &d) { return d.id ^ d.throttle ^ d.chan1; } 

struct Telemetry { int checkSum; int id; int vBatLow; int vBat; int rsi; int max_time; };
inline int CheckSum(struct Telemetry &t) { return t.id ^ t.vBatLow ^ t.vBat ^ t.rsi ^ t.max_time; }

const int TxMinPulse = 1000;  // us
const int TxMaxPulse = 2000;
const int TxMidPulse = TxMinPulse + (TxMaxPulse - TxMinPulse) / 2;

class RcTimer {
public:
  static const int TicksPerSec = 1000;
  RcTimer(int secs) { 
    this->secs = secs; 
    this->start(secs); 
  }
  void start(int secs = 0) { 
    this->begin = millis();
    if (secs > 0) this->secs = secs; 
    this->end = this->begin + this->secs * TicksPerSec;
  }
  bool elapsed() { return millis() > this->end; }
  int time() { return (millis() - this->begin); }
  int time2secs(int time) { return time / TicksPerSec; }
  int seconds() { return this->time2secs(this->time()); } 
  void log() { Serial.printf("[T] t:%d, s:%d, b:%d, e:%d\n", this->seconds(), this->secs, this->begin, this->end); } 
  int secs;
  unsigned long begin, end;
};

class RcServo {  // Note in standard RC connector the middle connector is always Vcc
public:
  RcServo(int pin, int min=TxMinPulse, int max=TxMaxPulse, int mid=0, int fail=-1, bool reverse=false) {  // -1 = last position
    this->set(min, max, mid, fail, reverse);
    this->servo.attach(pin, min, max);
	this->failsafe();
  }
  ~RcServo() { this->servo.detach(); }
  void set(int min=TxMinPulse, int max=TxMaxPulse, int mid=0, int fail=-1, bool reverse=false) { 
    this->min = min; this->max = max; this->mid = mid; this->fail = fail;
	if (reverse) {this->min = max; this->max = min;}
  }
  void writeTx(int txValue) { this->write(map(txValue + this->mid, TxMinPulse, TxMaxPulse, this->min, this->max)); }
  void write(int usValue) { this->servo.writeMicroseconds(usValue); }
  void failsafe() { if (this->fail >= 0) this->writeTx(this->fail); }
  int read() { return this->servo.readMicroseconds(); }
private:
  Servo servo;
  int min, max, mid, fail;
};

class Throttle : public RcServo {
public:
  Throttle(int pin, RcTimer &timer, int setTimer=0, int maxFlight=0, int warnEndFlight=5, int nrWarns=2, bool reverse=false) : 
           RcServo(pin, TxMinPulse, TxMaxPulse, 0, TxMinPulse-100, reverse) {
	this->timer = &timer;
    this->setTimer = setTimer;
    this->timerSet = setTimer == 0;
    this->maxFlight = maxFlight;          // secs 
	this->warnEndFlight = warnEndFlight;  // secs
	this->nrWarns = nrWarns;
    this->warnTime = (maxFlight - warnEndFlight) * RcTimer::TicksPerSec;
    this->endWarnTime = this->warnTime  + nrWarns * RcTimer::TicksPerSec;
  }
  void writeTx(int txValue) {
    int v = txValue;
    if ((!this->timerSet) && (this->setTimer > 0) && (v > this->setTimer)) {
      timer->start(this->maxFlight);
      this->timerSet = true;
    }
    if (this->timerSet && (this->maxFlight > 0)) {    
      if (timer->elapsed()) { RcServo::failsafe(); return; }
      int t = timer->time();
      if ((t >= this->warnTime) && (t < this->endWarnTime) && (t % RcTimer::TicksPerSec < RcTimer::TicksPerSec/2)) 
        v /= 2;
    }
    RcServo::writeTx(v);
  }
  void WarningNow() { 
	  this->timerSet = true;
	  this->warnTime = millis();
	  this->endWarnTime = this->warnTime + this->nrWarns * RcTimer::TicksPerSec;
	  this->timer->end = this->warnTime + this->warnEndFlight * RcTimer::TicksPerSec;
  } 
private:
  RcTimer *timer;
  bool timerSet;
  int setTimer, maxFlight, warnEndFlight, nrWarns, endWarn;      
  unsigned long warnTime, endWarnTime;  
};

// NOTE: fallback is middle if switch becomes disconnected!
class Switch {  // 3 (or 2) pos switch, middle to GND, left and right to digital outputs, no external pullup Rs
public:
  enum Pos3 {middle, left, right};
  Switch(int pinLeft, int pinRight=-1) {
    this->pinLeft = pinLeft; this->pinRight = pinRight;
    pinMode(pinLeft, INPUT_PULLUP); 
    if (pinRight >= 0) pinMode(pinRight, INPUT_PULLUP);
  }
  Pos3 read() { 
    int res = (int)!digitalRead(this->pinLeft);
    if (pinRight >= 0) res += 2 * (int)!digitalRead(this->pinRight);
    return (Pos3)res; 
  }
  int readTx() {
	static const int tab[] = {TxMidPulse, TxMinPulse, TxMaxPulse}; 
	return tab[this->read()];
  }
private:
  int pinLeft, pinRight;
};

class Potmeter {  // use 10Kohm and 3.3V (NOT 5V)
public:
  Potmeter(int pin, int min=TxMinPulse, int max=TxMaxPulse) { this->pin = pin; this->min = min; this->max = max; }
  int Read() { return map(this->ReadV(), 0, this->vref, this->min, this->max); }  // ESP32-C6 analogReadMilliVolt is factory calibrated, analogRead returns raw ADC value.
  int ReadV() { return analogReadMilliVolts(this->pin); }
private:
  const int vref = 3350; 
  int pin, min, max;
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
