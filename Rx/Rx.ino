/* 
Rx OpenRC4CL 13 June 2026

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

// NOTE: this is only tested on XIAO ESP32-C6, use partition schema NO OTA (2MB APP/2MB SPIFFS)
// RX com7 (proto=9) black 2nd usb

#include <MacAddress.h>
#include <WiFi.h>
#include <limits.h>
#include "OpenRC4CL_util.h"
char* macTx = "00:00:00:00:00:00";          // modify with your mac address of Tx or use serial CMD: set macPeer 00:00:00:00:00:00

// system
const unsigned long FAILSAFE_TIME = 500;    // ms
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 8S
const int lipoDivR2 = 100000;               // R2 lipo voltage divider
// user settings (NVS params)
String devName = "Rx-OpenRC4CL";
String macPeer(macTx); 
String passwd = "123";
String date = buildDate();
int wifiChan = 6;                           // [1..13]
int maxTime = 5*60;                         // seconds
int vBattLow = 3500;                        // mV
int escWarn = 0;                            // [0,1] if 1 issue Esc warnings end stop engine at end of flight 
int nrWarns = 3;                            // number of Esc warnings before stopping engine
int stopEngine = 5;                         // nr of secs to stop engine after last Esc warning
int minThrottle = TxMidPulse;               // min throttle value to start timer, 0 = no restart timer 
int nrPackets = 100;                        // nr packets (50 Hz) to receive for telemetry and logging
int logging = 1;                            // 1: log status, 0: no log
int vBatt = 0;                              // 0 = no vBatt connected, 1 = vBatt connected, to avoid floating analog inputs warnings
int ch1Left = 1000;                         // pulses in us for 3 pos switch channel 1 
int ch1Middle = 1500;
int ch1Right = 2000;

NVS* buildNVS(Logger *log) {
  const int max_nvs_params = 32; 
  // nvs_erase();
  NVS* nvs = new NVS(max_nvs_params, log); 
  nvs->add(NVS_STR(devName, true));
  nvs->add(NVS_STR(macPeer, true));
  nvs->add(NVS_STR(passwd, true));
  nvs->add(NVS_STR(date, false));
  nvs->add(NVS_INT(wifiChan, true, 1, 13));
  nvs->add(NVS_INT(maxTime, true, 10, 10*60));
  nvs->add(NVS_INT(vBattLow, true, 3000, 8*4350));
  nvs->add(NVS_INT(escWarn, true, 0, 1));
  nvs->add(NVS_INT(nrWarns, true, 0, 10));
  nvs->add(NVS_INT(stopEngine, true, 0, 10));
  nvs->add(NVS_INT(minThrottle, true, TxMinPulse, TxMaxPulse));
  nvs->add(NVS_INT(nrPackets, true, 50, 1000));
  nvs->add(NVS_INT(logging, false, 0, 1));
  nvs->add(NVS_INT(vBatt, true, 0, 1));
  nvs->add(NVS_INT(ch1Left, true, TxMinPulse, TxMaxPulse));
  nvs->add(NVS_INT(ch1Middle, true, TxMinPulse, TxMaxPulse));
  nvs->add(NVS_INT(ch1Right, true, TxMinPulse, TxMaxPulse));
  return nvs;
}

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk, Logger *log) : RcPeer(mac_tx, channel, iface, lmk) { 
    failsafe(); 
    logger = log;
    logStatus();
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) { 
    struct TxData rc = *(struct TxData *)data;  // copy received data
    unsigned long now = millis();
    command(rc, now);
    telemetry(rc, now);
    status.value = (timer.elapsed() ? Status::EndFlight : (packetsLost > 0) ? Status::Error : (waitThrHold) ? Status::WaitThrHold: Status::Ok);
  }
  void update() {
    if (connected) {                       // check failsafe
      unsigned long last = timeLastTx;     // copy of last before geting now, this function can be interupted
      unsigned long now = millis();
      if ((now - last) > FAILSAFE_TIME) {  // note check can be interrupted by callback, abs doesn't work on unsigned
        failsafe();
        waitThrHold = true;                // reset from FailSafe with TH
        if (status.value != Status::Failsafe) {
         status.value = Status::Failsafe;
         logger->printf("FAILSAFE!!!!\n"); // log once
        }
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
    led.set(status.pulse()); led.update(); 
  }
protected:
  void logStatus() { 
    if (logging)   
      logger->printf("Rx:%s vbatt:%d t:%d maxt:%d lost:%d errors:%d\n", 
                      status.str(), rxBatt.read(), timer.secondsLeft(), maxTime, totalLost, errors);
  }
  void command(struct TxData &rc, unsigned long now) {
    if (CheckSum(rc) == rc.checkSum) { 
      if (!connected) { connected = true; timer.start(); status.value = Status::Ok; }  // timer can be reset using first minimal throttle command
      timeLastTx = now;
      if (rc.throttle == TxThrottleHoldPulse) waitThrHold = false;  // wait for (first) TH at start Rx
      thrLast = throttle.writeTx(waitThrHold ? throttle.failSaveValue() : rc.throttle);
      rc.chan1 = UpdateSwitchCh1(rc.chan1);
      chan1.writeTx(rc.chan1); chan2.writeTx(rc.chan2); chan3.writeTx(rc.chan3); chan4.writeTx(rc.chan4);
      if (rc.id < 100) {  // new Tx 
        timer.start(); status.value = Status::Ok;
        packetsLost = 0; lastId = -1; count = -1; 
      }    
      if (lastId > 0) packetsLost += rc.id - lastId - 1;
      lastId = rc.id;
    } else {
      packetsLost++; totalLost++;
    }
  }
  void telemetry(struct TxData &rc, unsigned long now) {
    if ((count == -1) && (rc.id % nrPackets != 0)) return;  // new Rx or Tx, sync id to multiple of nrPackets
    if (++count >= nrPackets) {  
      int rsi = max(nrPackets-packetsLost,0);
      struct Telemetry tel = {0, rc.id, status.value, rxBatt.read(), vBattLow, rsi, timer.secondsLeft(), totalLost, errors};
      tel.checkSum = CheckSum(tel);
      if (!send_data((const uint8_t *)&tel, sizeof(tel))) errors++;
      int avg_time = (int)(now - timeLast1st) / nrPackets;
      if (logging)  
        logger->printf("Rx:%s vbatt:%d thr:%d ch1:%d ch2:%d ch3:%d ch4:%d ms:%d rsi:%d t:%d maxt:%d lost:%d errors:%d\n", 
                        status.str(), rxBatt.read(), thrLast, rc.chan1, rc.chan2, rc.chan3, rc.chan4, avg_time, rsi, 
                        timer.secondsLeft(), maxTime, totalLost, errors);
      count = packetsLost = 0;
      timeLast1st = now;
    }
  }
  void failsafe() {
    throttle.failsafe(); chan1.failsafe(); chan2.failsafe(); chan3.failsafe(); chan4.failsafe(); 
  }
protected:
  int UpdateSwitchCh1(int value) {
    switch (value) {
      case TxMinPulse: return ch1Left;
      case TxMidPulse: return ch1Middle;
      case TxMaxPulse: return ch1Right;
      default: return value;
    }
  }
private:
  RcTimer timer{maxTime};
  Throttle throttle{pinThrottle, &timer, minThrottle, maxTime, escWarn, nrWarns, stopEngine};
  RcServo chan1{pinCh1, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan2{pinCh2, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan3{pinCh3, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan4{pinCh4, TxMinPulse, TxMaxPulse, 0, -1};
  VoltageDiv rxBatt{(vBatt) ? pinVBatt : PIN_NOT_USED, lipoDivR1, lipoDivR2};
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true}; 
  bool connected = false, waitThrHold = true;
  unsigned long timeLastTx = 0, timeLast1st = 0;  // time last 1st packadge;
  int thrLast = -1, lastId = -1, count = -1, packetsLost = 0, totalLost = 0, errors = 0;
  Logger *logger = 0;
};

Rx *rx = 0;                   // initialisation must be in setup 
CMD* cmd = 0; 

void setup() {
  Serial.begin(115200);
  SerialBLE.begin(devName);
  Logger *logger = new Logger;    // Serial and BLE logger
  NVS* nvs = buildNVS(logger);
  cmd = new CMD(nvs, logger);
  WiFi.mode(WIFI_STA); WiFi.setChannel(wifiChan);
  logger->printf("OpenRC4CL %s Rx %s channel=%d MAC-Rx=%s MAC-Tx=%s\n", 
                  OpenRC4CL_VERSION, devName.c_str(), wifiChan, WiFi.macAddress().c_str(),macPeer.c_str());
  rx = new Rx(MacAddress(macPeer), wifiChan, WIFI_IF_STA, nullptr, logger);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx->add_self())) {
    logger->printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
}

void loop() {
  rx->update();
  cmd->update(); 
}
