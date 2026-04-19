/* 
Utils for OpenRC4CL 19 April 2026

MIT license

Copyright 2026 Jaap van de Loosdrecht

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

// NOTE: this is only tested on XIAO ESP32-C6

#ifndef OpenRC4CL_util
#define OpenRC4CL_util

const char *OpenRC4CL_VERSION = "1.0.10"; 

#include <ESP32_NOW.h>
#include <MacAddress.h>
#include <WiFi.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include "ESP32C6_back_side_pins.h"
#include <BLESerial.h>

// pin layout for PCB
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinVBatt = A2;                      
const int pinThrottleHold = D6;
const int pinBeep = D7;
const int pinCh1 = A1;                      // Rx chan1 servo
const int pinLeftCh1 = D4;                  // Tx chan1 3-position switch
const int pinRightCh1 = D5;
const int pinCh2 = A4;
const int pinCh3 = A5; 
const int pinCh4 = A6; 
// const int pinExternLed = D10;            // todo

BLESerial<> SerialBLE;  // make SerialBLE global accesable like Serial, use Serial Bluetooth Terminal App (Google play)
extern BLESerial<> SerialBLE;  

struct Status {	
  enum StatusId                                {  Ok,   WaitThrHold,   WaitTxRx,   VBattLow,   TxBattLow,   EndFlight,   Failsafe,   WaitStart,   Error };
  static inline const char* const  strTab[9] = { "Ok", "WaitThrHold", "WaitTxRx", "VBattLow", "TxBattLow", "EndFlight", "Failsafe", "WaitStart", "Error" };
  static constexpr const int pulseTab[9] =     {  0,    1000,          0,          2000,       2000,        2000,        500,        1000,        200 };
  static const char* val2str(int val) { return strTab[val]; }  
  Status(StatusId s=Error) { value = s; }
  const int pulse() { return pulseTab[value]; }  // status blink pulse in us 
  const char* str() { return val2str(value); }
  StatusId value;
};

struct TxData { int checkSum, id, throttle, chan1, chan2, chan3, chan4; }; 
inline int CheckSum(struct TxData &d) { return d.id ^ d.throttle ^ d.chan1 ^ d.chan2 ^ d.chan3 ^ d.chan4; } 

struct Telemetry { int checkSum, id, status, vBat, vBatLow, rsi, time_left, stop, totalLost, errors; };
inline int CheckSum(struct Telemetry &t) { return t.id ^ t.status ^ t.vBat ^ t.vBatLow ^ t.rsi ^ t.time_left ^ t.stop ^ t.totalLost ^ t.errors; }

const int TxMinPulse = 1000;  // Tx pulses in us    todo class TxPulse::Min, etc
const int TxMaxPulse = 2000;
const int TxMidPulse = TxMinPulse + (TxMaxPulse - TxMinPulse) / 2;
const int ThrHoldDelta = 100;  
const int TxThrottleHoldPulse = TxMinPulse - ThrHoldDelta;

String buildDate() {
  char month[4];
  int day, year;
  sscanf(__DATE__, "%s %d %d", month, &day, &year);
  return String(day) + String(month) + String(year-2000);
}

int avgAnalogMilliVolts(int pin, int nrSamples) {
  if (pin == PIN_NOT_USED) return 0; 
  int sum = 0; for(int i = 0; i < nrSamples; i++) sum += analogReadMilliVolts(pin);
  return sum / nrSamples;
}

class VoltageDiv {
public:
  VoltageDiv(int pin, int r1, int r2) {  _pin = pin; _r1 = r1; _r2 = r2; _div = ((r1+r2)/r1); }
  int read() { return avgAnalogMilliVolts(_pin, 16) * _div; }  // V in mV over r1
  int max() { return 3300 * _div; }
private:
  int _pin, _r1, _r2, _div;
};

class Blink {  // base class Led/Beep, ms pulse width, 0 ms is always on, -1 ms is off
public:
  Blink(int pin, int ms, bool inv, int rep) {   // Note LED_BUILTIN is reversed on C6
    _pin = pin; 
	if (inv) { off = HIGH; on = LOW; } else { off = LOW; on = HIGH; }  
	set(ms, rep); 
	if (_pin != PIN_NOT_USED) { pinMode(_pin, OUTPUT); } 
  }
  void set(int ms, int rep=0) { _ms = ms; _rep = rep; count = 0; active = false; } 
  void update() {
	if (_pin == PIN_NOT_USED) return;
    if ((_rep == 0) || (count <= _rep)) {
      if (_ms != 0) {
        bool s = (millis() % _ms < _ms / 2);
        if ((s != active) && s) count++;
        active = ((count <= _rep) || (_rep == 0)) ? s : false;
      } else {
        active = true;
      }
    } else {
      active = false;
    }
	// if (_pin == D8) Serial.printf("[Blink] pin:%d, ms:%d, active:%d, rep:%d, count:%d, on:%d, off:%d\n", _pin, _ms, active, _rep, count, on, off);
    digitalWrite(_pin, active ? on : off);  
  }
private:
  int _pin, _ms, _rep, count;
  bool active, on, off;
};

class Led : public Blink {  // status blink led in ms pulse width, 0 ms is always on, -1 ms is off
public:
  Led(int pin, int ms, int inv=false, int rep=0): Blink(pin, ms, inv, rep) {}
};

class Beep : public Blink {  // status beeper in ms pulse width, 0 ms is always on, -1 ms is off
public:
  Beep(int pin, int ms, int inv=false, int rep=1): Blink(pin, ms, inv, rep) {}
};

class PushButton {  // one end to GND, other end to digital input, no external pullup Rs
public:             // note: the two pins on each side, close to each other, are connected 
  PushButton(int pin) { _pin = pin; if (_pin != PIN_NOT_USED) pinMode(_pin, INPUT_PULLUP); }
  bool read() { return (_pin != PIN_NOT_USED) ? !digitalRead(_pin) : HIGH; }   // LOW (pushed) or HIGH (not pushed)
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
  Pos readPos() { int lt = leftP->read(); return (Pos)(rightP ? lt + 2 * rightP->read() : lt); }
  int read() { return tab[this->readPos()]; }
private:
  static constexpr int tab3[3] = {TxMidPulse, TxMinPulse, TxMaxPulse}; 
  static constexpr int tab2[2] = {TxMinPulse, TxMaxPulse};
  const int *tab = tab2;
  PushButton *leftP, *rightP = 0;
};

class DipSwitch {
public:
  static const int MAX_DIPS = 4;
  DipSwitch(int nr, int pins[]) { 
    _nr = nr;
    for (int s = 0; s < _nr; s++) { _switch[s] = new Switch(pins[s]); };
  }
  int readOne(int s) { return (int)_switch[s]->readPos(); }
  int read() {  // aggregated binairy result of all switches
    int res = 0;
    for (int s = 0; s < _nr; s++) { res += (s+1) * readOne(s); }
    return res;
  }
private:
  int _nr;
  Switch* _switch[MAX_DIPS];
};

class Potmeter {  // use 10K ohm and 3.3V (NOT 5V), ESP32-C6 analogReadMilliVolt is factory calibrated, analogRead returns raw ADC value.
public:
  Potmeter(int pin, int min=TxMinPulse, int max=TxMaxPulse, int nrSamples=16) { _pin = pin; _min = min; _max = max; _nrS = nrSamples; }
  int read() { return map(readV(), 0, VRef, _min, _max); }
  int readV() { return avgAnalogMilliVolts(_pin, _nrS); }
  void setMinMax(int min, int max=TxMaxPulse) { _min = min; _max = max; }
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
  _pin = pin;
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
  void write(int usValue) { if (_pin != PIN_NOT_USED) servo.writeMicroseconds(usValue); }
  void failsafe() { int v = (_fail >= 0) ? _fail : _last; writeTx(v); }
  int read() { return (_pin != PIN_NOT_USED) ? servo.readMicroseconds() : _min; }
  int failSaveValue() { return _fail; }
  bool reverse() { return _reverse; }
protected:
  Servo servo;
  bool _reverse;
  int _pin, _min, _max, _mid, _fail, _last; 
};

class Throttle : public RcServo { // always [TxMinPulse..TxMaxPulse]
public:
  Throttle(int pin, RcTimer *rctimer, int minThrottle, int maxFlightSecs, bool escWarning,
           int nrWarns=2, int stopEngine=5, bool reverse=false) : 
           RcServo(pin, TxMinPulse, TxMaxPulse, 0, reverse ? (TxMaxPulse+ThrHoldDelta) : (TxMinPulse-ThrHoldDelta), reverse) {
	timer = rctimer; lastThr = failSaveValue(); minThr = minThrottle; timerSet = minThrottle == 0; nrWarn = nrWarns;
    maxFlight = maxFlightSecs; escWarn = escWarning; stopEng = stopEngine; 
	setWarnTime();
  }
  int writeTx(int txValue) {
    int v = lastThr = txValue;
	if ((v == TxThrottleHoldPulse) || (escWarn && timer->elapsed())) {
	  v = failSaveValue();
	} else {	
      if ((!timerSet) && (minThr > 0) && (v > minThr)) { timer->start(maxFlight); timerSet = true; }
      if (escWarn && timerSet && (maxFlight > 0)) {    
        int t = timer->time();
		stop = (t >= warnTime) || stop;
        if (stop && (t < endWarnTime) && (t % RcTimer::TicksPerSec < RcTimer::TicksPerSec/2)) {
          v = reverse() ? (_max - (_max-v) / 2) : (_min + (v-_min) / 2); 
		}
	  }
	}
    RcServo::writeTx(v);
	return v;
  }
  void resetTimer(int maxFlightSecs) { 
    if (timer) {
      timer->start(maxFlight=maxFlightSecs); 
	  setWarnTime(); 
	}
  }
  void warnAndStop() { // initiate end warning and stop, used by failsafe and batt low
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
  bool isStop() { return stop; }  // True: warn stopping is started
private:
  RcTimer *timer;
  bool escWarn, timerSet = false, stop = false;
  int lastThr, minThr, maxFlight, warnEndFlight, nrWarn, stopEng;      
  unsigned long warnTime, endWarnTime;  
  void setWarnTime() {  // set start and end warning time
    warnEndFlight = min(maxFlight, nrWarn * Status::pulseTab[Status::EndFlight]/1000 + stopEng);  // secs before end of flight
    warnTime = (maxFlight - warnEndFlight) * RcTimer::TicksPerSec;                                // wall clock
	endWarnTime = warnTime  + nrWarn * RcTimer::TicksPerSec;                                      // wall clock
	stop = false;
  }
};

class Logger {  // log msgs to both Serial and SerialBLE
public:
  Logger(int max_buf = 120) { _max_buf = max_buf; _buf = new char[_max_buf]; }
  void printf(const char* format, ...) {
    va_list args; va_start(args, format);
    vsnprintf(_buf, _max_buf, format, args);
    va_end(args);
    Serial.print(_buf); SerialBLE.print(_buf); SerialBLE.flush();
  }
private:
  char *_buf = 0; int _max_buf;
};

struct NVS_PARAM { char* name; PreferenceType p_type; int* int_p; int int_0; int min; int max; String* str_p; String str_0; };
#define NVS_INT(p, min, max) {#p, PT_I32, &p, p, min, max, 0, ""}  // see Preference.h source code
#define NVS_STR(p) {#p, PT_STR, 0, 0, 0, 0, &p, p.c_str()}
class NVS {  // Non Volatile Storage
public:
  NVS(const int max_ps, Logger *logger) { 
    max_params = max_ps;
    tab = new NVS_PARAM[max_params]; 
    nr_params = 0;
    log = logger;
  }
  void add(NVS_PARAM np) {  // must be by value
	if (nr_params >= max_params-1) { log->printf("Error: NVS table full\n"); return; }
	NVS_PARAM* p = &tab[nr_params];
	tab[nr_params++] = np;
    pref.begin(nmspace, true);  // readonly, initialse from eprom or use default from table
    if (isStr(p->name)) { 
	  *(p->str_p) = pref.getString(p->name, p->str_0);
	} else { // PT_I32
	  *(p->int_p) = pref.getInt(p->name, p->int_0);
    }
    pref.end();  
  }
  int nrParams(void) { return nr_params; }
  NVS_PARAM* nvsTab(void) { return tab; }
  bool isParam(char* name) { return getParam(name); }
  bool isInt(char* name) { return getParam(name)->p_type == PT_I32; };
  bool isStr(char* name) { return getParam(name)->p_type == PT_STR; };
  int readInt(char* name) { 
    NVS_PARAM* p = getParam(name);
    return (p) ? *(p->int_p) : 0; 
  }
  void writeInt(char* name, int value) { 
    NVS_PARAM* p = getParam(name);
    if (p == 0) return;  // todo log error
	if ((value < p->min) || (value > p->max)) {
	  log->printf("Error: value must be in range %d..%d\n", p->min, p->max);
      return;
	}
    pref.begin(nmspace, false);  // read/write
    pref.putInt(p->name, value);
    *(p->int_p) = value;
    pref.end();
  }
  String readStr(char* name) {
    NVS_PARAM* p = getParam(name);
    return (p) ? *(p->str_p) : ""; 
  }
  void writeStr(char* name, String value) { 
    NVS_PARAM* p = getParam(name);
    if (p == 0) return;
    pref.begin(nmspace, false);  // read/write
    pref.putString(p->name, value);
    *(p-> str_p) = value;
    pref.end();
  }
private:
  NVS_PARAM* getParam(char *name) { 
    for (int i = 0; i < nr_params; i++) if (strcmp_P(name, tab[i].name) == 0) return &tab[i];
    log->printf("Unknown NVS param name:%s\n", name);
    return 0;
  }
  char* nmspace = "OpenRC4CL";
  Preferences pref;
  NVS_PARAM* tab = 0; 
  int max_params;
  int nr_params;
  Logger *log = 0;
};
static void nvs_erase() {  // erase the NVS partition
  nvs_flash_erase();  
  nvs_flash_init();
  Serial.print(F("NVS erased!\n"));
}

class CMD { // CoMmanD interpreter
  public:
    CMD(NVS* _nvs, Logger *logger) { 
      nvs = _nvs;
      log = logger;
	  tab = nvs->nvsTab(); 
      nr_params = nvs->nrParams();
      Serial.printf("%s=%s\n", passwdCmd, nvs->readStr(passwdCmd)); // Note log passwd to console only!
    }
    void exec(char *cmdline) {
      char *cmd = strsep(&cmdline, " ");
      if (not chk_passwd) passwd(cmd);
	  else if (!strcmp(cmd, "lock")) chk_passwd = false;
      else if (!strcmp(cmd, "set")) set(cmdline);
      else if (!strcmp(cmd, "get")) get(cmdline);
      else if (!strcmp(cmd, "list")) list();
      else if (!strcmp(cmd, "help")) log->printf("set param value\nget param\nhelp\nversion\ndefault\nreboot\nlock\n");
      else if (!strcmp(cmd, "version")) log->printf("%s\n", OpenRC4CL_VERSION);
      else if (!strcmp(cmd, "default")) { nvs_erase(); ESP.restart(); }
      else if (!strcmp(cmd, "reboot")) ESP.restart();
      else log->printf("Unknown command:%s\n", cmd);
    }
    void update() {
      int data;
      bool read = false;
      if (SerialBLE.available()) { data = SerialBLE.read(); read = true; } 
      else if (Serial.available()) { data = Serial.read(); read = true; }
      if (read) {
        if (data == '\r') {          // interpret
          buffer[length] = '\0';     // properly terminate the string
          if (length) exec(buffer);  
          length = 0;                // reset for next command
        } else if ((data != '\n') && (length < BUF_LENGTH - 1)) {  // discard \n
          buffer[length++] = data;   // buffer the incoming byte
        }
      }
    }
  protected:
    bool is_digits(const char *s) {
	  while(isspace((unsigned char)*s)) s++;
      if (!s || !(*s)) return false;
      while (*s) {
        if (!isdigit((unsigned char)*s)) return false;
        s++;
      }
      return true;
	}
	void passwd(char *cmd) {
      chk_passwd = nvs->readStr(passwdCmd) == String(cmd); 
      (chk_passwd) ? list() : log->printf("invalid password!\n");
    }
	void set(char *cmdline) {
      char* param = strsep(&cmdline, " ");
	  if (nvs->isParam(param)) { 
		if (nvs->isStr(param)) {
          Serial.printf("set %s %s\n", param, cmdline);
          nvs->writeStr(param, cmdline);
          log->printf("%s=%s\n", param, cmdline);
		} else {
		  if (!param || !is_digits(cmdline)) { log->printf("invalid command\n"); return; }
          int val = atoi(cmdline);
          nvs->writeInt(param, val);
          log->printf("%s=%d\n", param, val);
		}
	  }
	}
	void get(char *cmdline) {
	  if (!cmdline) { log->printf("invalid command\n"); return; }
	  if (nvs->isStr(cmdline)) {
        log->printf("%s=%s\n", cmdline, nvs->readStr(cmdline).c_str());
	  } else {
        log->printf("%s=%d\n", cmdline, nvs->readInt(cmdline));
	  }
	}
    void list() {
      const int maxp = 6;
	  NVS_PARAM* p = nvs->nvsTab(); 
      for (int i = 0; i < nvs->nrParams(); i++) {
        if (strcmp(p->name, passwdCmd)) {
		  if (nvs->isStr(p->name)) { 
		    log->printf("%s=%s ", p->name, p->str_p->c_str());
		  } else {
		    log->printf("%s=%d ", p->name, *(p->int_p));
		  }
		}
        if (((i > 0) && ((i % maxp) == 0)) || (i == nr_params-1)) log->printf("\n");
		p++;
      }
	}
  private:
    static constexpr char* passwdCmd = "passwd";
    static const int BUF_LENGTH = 32;
    char buffer[BUF_LENGTH];
    int length = 0;            // length of line received so far
    bool chk_passwd = false;
    NVS_PARAM* tab = 0;
    int nr_params = 0;
    NVS* nvs = 0;
    Logger *log = 0;
};

class RcPeer : public ESP_NOW_Peer {  // ESP peer wrapper for Tx and Rx
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