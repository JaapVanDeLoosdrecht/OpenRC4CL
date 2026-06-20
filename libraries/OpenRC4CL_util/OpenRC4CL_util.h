/* 
Utils for OpenRC4CL 19 June 2026

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

const char *OpenRC4CL_VERSION = "1.0.17"; 

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

BLESerial<> SerialBLE;         // make SerialBLE global accesable like Serial, use Serial Bluetooth Terminal App (Google play)
extern BLESerial<> SerialBLE;  // windows 11 https://github.com/healthywalk/BLE-Serial-Terminal, use CR+LF

struct Status {	
  enum StatusId                                {  Ok,   WaitThrHold,   WaitTxRx,   RxBattLow,   TxBattLow,   EndFlight,   Failsafe,   WaitStart,   Error };
  static inline const char* const  strTab[9] = { "Ok", "WaitThrHold", "WaitTxRx", "RxBattLow", "TxBattLow", "EndFlight", "Failsafe", "WaitStart", "Error" };
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

const char* NVSNameSpace = "OpenRC4CL";

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
  enum Pos {Left, Middle, Right};
  Switch(int pinLeft, int pinRight = PIN_NOT_USED) {
	if (pinLeft != PIN_NOT_USED) leftP = new PushButton(pinLeft);
    if (pinRight != PIN_NOT_USED) rightP = new PushButton(pinRight); 
  }
  Pos readPos() { 
	if (leftP && leftP->read()) return Left;
	if (rightP && rightP->read()) return Right; 
	return Middle;
  }
  int read() { return tab[this->readPos()]; }
private:
  static constexpr int tab[3] = {TxMinPulse, TxMidPulse, TxMaxPulse}; 
  PushButton *leftP, *rightP = 0;
};

class DipSwitch {  
public:
  static const int MAX_DIPS = 4;
  DipSwitch(int nr, int pins[]) { 
    _nr = nr;
    for (int s = 0; s < _nr; s++) { _switch[s] = new Switch(pins[s]); };
  }
  int readOne(int s) { return (int)(_switch[s]->readPos() == Switch::Left); }
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
	_last = read();
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

String GetStringNVS(const char* nmspace, const char* key) { 
  Preferences pref;
  pref.begin(nmspace, true);  // read mode
  String str = (pref.isKey(key)) ? pref.getString(key) : String("");
  pref.end();
  return str;  
}

struct NVS_PARAM { char* name; bool edit; PreferenceType p_type; int* int_p; int int_0; int min; int max; String* str_p; String str_0; };
#define NVS_INT(p, e, min, max) {#p, e, PT_I32, &p, p, min, max, 0, ""}  // see Preference.h source code
#define NVS_STR(p, e) {#p, e, PT_STR, 0, 0, 0, 0, &p, p.c_str()}
class NVS {  // Non Volatile Storage
public:
  NVS(const char* nspace, const int max_ps, Logger *logger) { 
    nmspace = nspace;
    max_params = max_ps;
    tab = new NVS_PARAM[max_params]; 
    nr_params = 0;
    log = logger;
  }
  void add(NVS_PARAM np) {  // must be by value
	if (nr_params >= max_params-1) { log->printf("Error: NVS table full\n"); return; }
	NVS_PARAM* p = &tab[nr_params];
	tab[nr_params++] = np;
	Preferences pref; 
    pref.begin(nmspace, false);  // read/write mode
    if (pref.isKey(p->name) && p->edit) {
      if (isStr(p->name)) {      // initialse from eprom
	    *(p->str_p) = pref.getString(p->name);
	  } else { // PT_I32
	    *(p->int_p) = pref.getInt(p->name);
      }
	} else {                     // initialse with default from table
      if (p->p_type == PT_STR)   
	    writeStr(p->name, p->str_0);
	  else // PT_I32
	    writeInt(p->name, p->int_0);
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
	Preferences pref; 
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
	Preferences pref; 
    pref.begin(nmspace, false);  // read/write
    pref.putString(p->name, value);
    *(p->str_p) = value;
    pref.end();
  }
private:
  NVS_PARAM* getParam(char *name) { 
    for (int i = 0; i < nr_params; i++) if (strcmp_P(name, tab[i].name) == 0) return &tab[i];
    log->printf("Unknown NVS param name:%s\n", name);
    return 0;
  }
  const char* nmspace = 0; //"OpenRC4CL";
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

struct BindElm { char* devName; char* mac; };
class BindTab {
public:
  static const int maxStrLen = 19;
  struct Elm { char devName[(maxStrLen+1)]; char mac[(maxStrLen+1)]; };  // +1 for \0
  BindTab(const char* nvs, const int maxBinds, const int nrBs, BindElm peerTab[]) {
	nmspace = nvs;
    max = maxBinds;
    tab = new Elm[max];
	memset(tab, 0, max * sizeof(Elm));
	if (!read()) {
	  Serial.printf("Use compile time peerTab");
	  nr = 0;
      for (int i = 0; i < nrBs; i++) {
		BindElm* e = &peerTab[i];
	    if (!addElm(e->devName, e->mac)) Serial.printf("Can not add bind %s %s", e->devName, e->mac);
	  }
      write();
	}
  }
  bool addElm(char* devName, char* mac) {
	if ((nr > max) || (strlen(devName) > maxStrLen) || (strlen(mac) > maxStrLen)) return false;
	Elm* e = devNameElm(devName);
	if (!e) { e = &tab[nr]; nr++; }  // new devName
    strcpy(e->devName, devName);
    strcpy(e->mac, mac);	
	return true;
  }
  bool add(char* devName, char* mac) {
	if (!addElm(devName, mac)) return false;
    write();
    return true;	
  }	
  bool remove(char* devName) {
  	int idx = devNameElmIdx(devName);
	if (idx == -1) return false;
	for (int i = idx; i < nr-1; i++) tab[idx] = tab[idx+1];
    nr--;
	write();
	return true;
  }
  char* mac(const char* devName) { 
	Elm* e = devNameElm(devName);
	return e ? e->mac : 0;
  }
  void write() {  // to NVS todo test
    Preferences pref;
    pref.begin(nmspace, false);  // read/write mode
	pref.putInt(nrKey, nr);
    pref.putBytes(tabKey, tab, nr*sizeof(Elm));
    pref.end();
  }
  bool read() {  // from NVS todo test
    Preferences pref;
    pref.begin(nmspace, true);  // read mode
	bool inNVS = pref.isKey(tabKey);
	if (!inNVS) return false;
    nr = pref.getInt(nrKey);
	Serial.printf("Read bindtab with #bind=%d\n", nr);
	pref.getBytes(tabKey, tab, nr*sizeof(Elm)); 
    pref.end();
	return true;
  }
  int devNameElmIdx(const char* devName) {  // return Elm idx with devName
    for (int i = 0; i < nr; i++) if (!strcmp(tab[i].devName, devName)) return i;
    return -1;
  }
  Elm* devNameElm(const char* devName) {  // return Elm with devName
	int idx = devNameElmIdx(devName);
    return (idx != -1) ? &tab[idx] : 0;
  }
  int nr = 0, max = 0;
  Elm* tab = 0;
  const char* tabKey = "#bindtab";
  const char* nrKey = "#nrbinds";
  const char* nmspace = 0;
};

class CMD { // CoMmanD interpreter
  public:
    CMD(NVS* _nvs, Logger *logger, BindTab* bTab) { 
      nvs = _nvs;
      log = logger;
	  tab = nvs->nvsTab();
      bindTab = bTab;	  
      nr_params = nvs->nrParams();
      Serial.printf("%s=%s\n", passwdCmd, nvs->readStr(passwdCmd)); // Note log passwd to console only!
    }
    void exec(char *cmdline) {
	  Serial.printf("cmd %s\n", cmdline);
      char *cmd = strsep(&cmdline, " ");
      if (not chk_passwd) passwd(cmd);
	  else if (!strcmp(cmd, "lock")) chk_passwd = false;
      else if (!strcmp(cmd, "set")) set(cmdline);
      else if (!strcmp(cmd, "get")) get(cmdline);
      else if (!strcmp(cmd, "list")) list();
      else if (!strcmp(cmd, "bindtab")) bindtab();
      else if (!strcmp(cmd, "bind")) bind(cmdline);
      else if (!strcmp(cmd, "addbind")) addbind(cmdline);
      else if (!strcmp(cmd, "removebind")) removebind(cmdline);
      else if (!strcmp(cmd, "mac")) log->printf("mac=%s\n", WiFi.macAddress().c_str());
      else if (!strcmp(cmd, "help")) help();
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
    void bindtab() {
	  for (int i = 0; i < bindTab->nr; i++) {
		BindTab::Elm* b = &(bindTab->tab[i]);
		log->printf("%s=%s\n", b->devName, b->mac);
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
	void help() {  // not logger has limited buffer lenght
      log->printf("set param value\n"
	              "get param\n"
		          "default  <set all params to default>\n");
	  log->printf("reboot\n"
		          "version\n"
		          "lock     <lock cmds>\n"
		          "mac\n");
	  log->printf("bindtab  <show all binds>\n"
		          "bind devName\n"
		          "addbind devName mac\n"
		          "removebind devName\n");
	}
	void set(char *cmdline) {
      char* param = strsep(&cmdline, " ");
	  if (nvs->isParam(param)) { 
		if (nvs->isStr(param)) {
          // Serial.printf("set %s %s\n", param, cmdline);
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
	void bind(char *cmdline) {
	  if (!cmdline) { log->printf("invalid command\n"); return; }
	  if (char* mac = bindTab->mac(cmdline)) {
	    nvs->writeStr("macPeer", mac);
	    nvs->writeStr("devPeer", cmdline);
	    ESP.restart();
	  } else {
        log->printf("Unkown devName %s\n", cmdline);
	  }
	}
	void addbind(char *cmdline) {
	  if (!cmdline) { log->printf("invalid command\n"); return; }
	  char *devName = strsep(&cmdline, " ");
	  if (!cmdline) { log->printf("invalid command\n"); return; }
	  Serial.printf("addbind %s %s\n", devName, cmdline);
	  if (!bindTab->add(devName, cmdline)) log->printf("error bindtab full\n");
	  bindtab();
	}
	void removebind(char *cmdline) {
	  if (!cmdline) { log->printf("invalid command\n"); return; }
	  if (!bindTab->remove(cmdline)) log->printf("Unkown devName %s\n", cmdline);
	  bindtab();
	}
    static constexpr char* passwdCmd = "passwd";
    static const int BUF_LENGTH = 64;
    char buffer[BUF_LENGTH];
    int length = 0;            // length of line received so far
    bool chk_passwd = false;
    NVS_PARAM* tab = 0;
    int nr_params = 0;
    NVS* nvs = 0;
    Logger *log = 0;
	BindTab *bindTab = 0;
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