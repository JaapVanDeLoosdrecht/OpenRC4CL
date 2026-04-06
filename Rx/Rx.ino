/* 
Rx OpenRC4CL 3 April 2026

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

// #define PROTOBOARD

#include <MacAddress.h>
#include <WiFi.h>
#include <limits.h>
#include "OpenRC4CL_util.h"
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out
#ifndef SECRET
char* macTx = "00:00:00:00:00:00";          // modify with your mac address of Tx or use serial CMDs
#endif

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

NVS* buildNVS(Logger *log) {
  const int max_nvs_params = 16; 
  // nvs_erase();
  NVS* nvs = new NVS(max_nvs_params, log); 
  nvs->add(NVS_STR(devName));
  nvs->add(NVS_STR(macPeer));
  nvs->add(NVS_STR(passwd));
  nvs->add(NVS_STR(date));
  nvs->add(NVS_INT(wifiChan, 1, 13));
  nvs->add(NVS_INT(maxTime, 10, 10*60));
  nvs->add(NVS_INT(vBattLow, 3000, 8*4350));
  nvs->add(NVS_INT(escWarn, 0, 1));
  nvs->add(NVS_INT(nrWarns, 0, 10));
  nvs->add(NVS_INT(stopEngine, 0, 10));
  nvs->add(NVS_INT(minThrottle, TxMinPulse, TxMaxPulse));
  nvs->add(NVS_INT(nrPackets, 50, 1000));
  return nvs;
}

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk, Logger *log) : RcPeer(mac_tx, channel, iface, lmk) { 
    throttle.failsafe(); chan1.failsafe(); 
    logger = log;
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
        throttle.failsafe();
        chan1.failsafe(); chan2.failsafe(); chan3.failsafe();
        status.value = Status::Failsafe;
        waitThrHold = true;                // reset from FailSafe with TH
        logger->printf("FAILSAFE!!!!\n");
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
    led.set(status.pulse()); led.update(); 
  }
protected:
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
      int rsi = max(nrPackets-packetsLost,0);

      struct Telemetry tel = {0, rc.id, status.value, vBatt.read(), vBattLow, rsi, timer.secondsLeft(), totalLost, errors};
      tel.checkSum = CheckSum(tel);
      if (!send_data((const uint8_t *)&tel, sizeof(tel))) errors++;
      int avg_time = (int)(now - timeLast1st) / nrPackets;
      logger->printf("Rx:%s thr:%d ch1:%d ch2:%d ch3:%d ch4:%d ms:%d rsi:%d t:%d maxt:%d lost:%d errors:%d\n", 
                      status.str(), thrLast, rc.chan1, rc.chan2, rc.chan3, rc.chan4, avg_time, rsi, 
                      timer.secondsLeft(), maxTime, totalLost, errors);
      count = packetsLost = 0;
      timeLast1st = now;
    }
  }
private:
  RcTimer timer{maxTime};
  Throttle throttle{pinThrottle, &timer, minThrottle, maxTime, escWarn, nrWarns, stopEngine};
  RcServo chan1{pinCh1, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan2{pinCh2, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan3{pinCh3, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan4{pinCh4, TxMinPulse, TxMaxPulse, 0, -1};
  VoltageDiv vBatt{pinVBatt, lipoDivR1, lipoDivR2};
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
  rx = new Rx(MacAddress(macPeer), wifiChan, WIFI_IF_STA, nullptr, logger);
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
  rx->update();
  cmd->update(); 
}
