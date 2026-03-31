// #include <ESP32C6_back_side_pins.h>
// #include <OpenRC4CL_util.h>
// #include <secret.h>

// #include <ESP32C6_back_side_pins.h>
// #include <OpenRC4CL_util.h>
// #include <secret.h>

// #include <ESP32C6_back_side_pins.h>
// #include <OpenRC4CL_util.h>
// #include <secret.h>

// #include <ESP32C6_back_side_pins.h>
// #include <OpenRC4CL_util.h>
// #include <secret.h>

/* 
Rx OpenRC4CL 26 March 2026

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
// RX com7 black 2nd usb

#include <MacAddress.h>
#include <WiFi.h>
#include <limits.h>
#include "OpenRC4CL_util.h"
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out
#ifndef SECRET
char* macTx = "00:00:00:00:00:00";          // modify with your mac address of Tx or use serial CMDs
#endif

// pin layout
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinCh1 = PIN_NOT_USED;            // A1; 
const int pinVBatt = A2;                    
const int pinCh2 = PIN_NOT_USED;            // A4; 
const int pinCh3 = PIN_NOT_USED;            // A5; 
const int pinCh4 = PIN_NOT_USED;            // A6; 
// const int pinExternLed = D10;            // todo
// system
const unsigned long FAILSAFE_TIME = 500;    // ms
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 8S
const int lipoDivR2 = 100000;               // R2 lipo voltage divider
// user settings (NVS params)
String devName = "Rx-OpenRC4CL";
String macPeer(macTx); 
String passwd = "123";
int wifiChan = 6;                           // [1..13]
int maxTime = 10*60;                    
int vBattLow = 3500;
int maxFlight = 10*60;                      // seconds, if maxTime is not in used.
int escWarning = 0;                         // [0,1] if 1 issue Esc warning of end of flight
int nrWarns = 2;                            // number of warnings before stopping engine
int stopEngine = 1;                         // nr of secs to stop engine after last warning
int minThrottle = TxMidPulse;               // min throttle value to start timer, 0 = no restart timer 
int nrPackets = 100;                        // nr packets (50 Hz) to receive for telemetry and logging

NVS* buildNVS(Logger *logger) {
  const int max_nvs_params = 16; 
  NVS* nvs = new NVS(max_nvs_params, logger); 
  nvs->add(NVS_STR(devName));
  nvs->add(NVS_STR(macPeer));
  nvs->add(NVS_STR(passwd));
  nvs->add(NVS_INT(wifiChan, 1, 13));
  nvs->add(NVS_INT(maxTime, 10, maxFlight));
  nvs->add(NVS_INT(vBattLow, 3000, 8*4350));
  nvs->add(NVS_INT(maxFlight, 10, 10*60));
  nvs->add(NVS_INT(escWarning, 0, 1));
  nvs->add(NVS_INT(nrWarns, 0, 10));
  nvs->add(NVS_INT(minThrottle, TxMinPulse, TxMaxPulse));
  nvs->add(NVS_INT(nrPackets, 50, 1000));
  return nvs;
}

// global vars
Logger *logger = 0; 

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : RcPeer(mac_tx, channel, iface, lmk) { 
    throttle.failsafe(); chan1.failsafe(); 
  }
  void command(struct TxData &rc, unsigned long now) {
    if (CheckSum(rc) == rc.checkSum) { 
      if (!connected) { connected = true; timer.start(); status.value = Status::Ok; }  // timer can be reset using first minimal throttle command
      timeLastTx = now;
      if (rc.throttle == TxThrottleHoldPulse) waitThrHold = false;  // wait for (first) TH at start Rx
      thrLast = throttle.writeTx(waitThrHold ? throttle.failSaveValue() : rc.throttle);
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
      int rsi =  max(nrPackets-packetsLost,0);
      struct Telemetry tel = {0, rc.id, status.value, vBatt.read(), vBattLow, rsi, timer.secondsLeft(), totalLost, errors};
      tel.checkSum = CheckSum(tel);
      if (!send_data((const uint8_t *)&tel, sizeof(tel))) errors++;
      int avg_time = (int)(now - timeLast1st) / nrPackets;
      logger->printf("Rx:%s thr:%d ch1:%d ch2:%d ch3:%d ch4:%d ms:%d rsi:%d t:%d maxt:%d lost:%d errors:%d\n", 
                      status.str(), thrLast, rc.chan1, rc.chan2, rc.chan3, rc.chan4, avg_time, rsi, 
                      timer.secondsLeft(), maxSecs, totalLost, errors);
      count = packetsLost = 0;
      timeLast1st = now;
    }
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) { 
    struct TxData rc = *(struct TxData *)data;  // copy received data
    unsigned long now = millis();
    command(rc, now);
    telemetry(rc, now);
    status.value = (timer.elapsed() ? Status::EndFlight : (packetsLost > 0) ? Status::Error : (waitThrHold) ? Status::WaitThrHold: Status::Ok);
  }
  void statusUpdate() { led.set(status.pulse()); led.update(); }
  void checkFailsafe() { 
    if (connected) {
      unsigned long last = timeLastTx;     // copy of last before geting now, this function can be interupted
      unsigned long now = millis();
      if ((now - last) > FAILSAFE_TIME) {  // note check can be interrupted by callback, abs doesn't work on unsigned
        throttle.failsafe();
        chan1.failsafe(); chan2.failsafe(); chan3.failsafe();
        status.value = Status::Failsafe;
        waitThrHold = true;                // reset from FailSafe with Th
        logger->printf("FAILSAFE!!!!\n");
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
  }
private:
  RcTimer timer{maxFlight};
  Throttle throttle{pinThrottle, &timer, minThrottle, maxTime, escWarning, nrWarns, stopEngine};
  RcServo chan1{pinCh1, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan2{pinCh2, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan3{pinCh3, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan4{pinCh4, TxMinPulse, TxMaxPulse, 0, -1};
  VoltageDiv vBatt{pinVBatt, lipoDivR1, lipoDivR2};
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true}; 
  bool connected = false, waitThrHold = true;
  unsigned long timeLastTx = 0, timeLast1st = 0;  // time last 1st packadge;
  int maxSecs = -1, thrLast = -1, lastId = -1, count = -1, packetsLost = 0, totalLost = 0, errors = 0;
};

Rx *rx = 0; // initialisation must in setup 
NVS *nvs = 0;
CMD* cmd = 0;

void setup() {
  Serial.begin(115200);
  SerialBLE.begin(devName);
  logger = new Logger;
  // nvs_erase();
  nvs = buildNVS(logger); 
  cmd = new CMD(nvs, logger);
  WiFi.mode(WIFI_STA); WiFi.setChannel(wifiChan);
  rx = new Rx(MacAddress(macPeer), wifiChan, WIFI_IF_STA, nullptr);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx->add_self())) {
    logger->printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  logger->printf("OpenRC4CL %s Rx %s channel:%d MAC Address:%s\n", 
                  OpenRC4CL_VERSION, devName.c_str(), wifiChan, WiFi.macAddress().c_str());
  logger->printf("Waiting to connect to Tx %s\n", macPeer.c_str()); 
}

void loop() {
  rx->checkFailsafe();
  rx->statusUpdate();
  cmd->update(); 
}
