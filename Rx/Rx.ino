/* 
Rx OpenRC4CL 7 December 2025

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

// NOTE: this is only tested on XIAO ESP32-C6, use partition schema NO OTA (2MB APP/2MB SPIFFS)
// RX com7 white usb

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_chan, mac address and BLE_device
#ifndef SECRET
#define WIFI_CHANNEL 6
const MacAddress macTx(({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with your mac address of Tx
char *BLE_device_Rx = "Rx-OpenRC4CL";                           // modify with your name
#endif

// user settings
const int maxFlight = 8*60;                 // seconds, if maxTime is not in used. Note in general
const int nrWarns = 2;                      // number if throttle warnings before stopping engine
const int stopEngine = 1;                   // nr of secs to stop engine after last warning
const bool escWarning = false;              // if true issue Esc warning of end of flight
const int minThrottle = TxMidPulse;         // min trhottle value to start timer, 0 = no restart timer 
// hardware
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinChan1 = PIN_NOT_USED;          // A1;                    
const int pinChan2 = PIN_NOT_USED;          // A2;                    
const int pinChan3 = PIN_NOT_USED;          // A4;  
const int pinMaxTime = PIN_NOT_USED;        // A5;  
const int pinVBattLow = A1;                 // A5;  
const int pinVBatt = A2;                    // A6;  
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 8S
const int lipoDivR2 = 100000;               // R2 lipo voltage divider
// system
const int NR_PACKETS = 50;                  // nr packets for telemetry and logging
const unsigned long FAILSAFE_TIME = 500;    // ms
Logger *logger = 0; 

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : RcPeer(mac_tx, channel, iface, lmk) { 
    throttle.failsafe(); chan1.failsafe(); 
  }
  void command(struct TxData &rc, unsigned long now) {
    if (!connected) { connected = true; timer.start(); status.value = Status::Ok; }  // timer can be reset using first minimal throttle command
    if (CheckSum(rc) == rc.checkSum) { 
      timeLastTx = now;
      if (rc.throttle == TxThrottleHoldPulse) {  
        if (pinMaxTime != PIN_NOT_USED) {    
          int mt = maxTime.read();
          if (abs(mt-maxSecs) > 5) throttle.resetTimer(maxSecs = mt);  // only modify max time if TH 
        }
        waitThrHold = false;  // wait for (first) TH at start Rx
      }
      thrLast = throttle.writeTx(waitThrHold ? throttle.failSaveValue() : rc.throttle);
      chan1.writeTx(rc.chan1); chan2.writeTx(rc.chan2); chan3.writeTx(rc.chan3);
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
    if ((count == -1) && (rc.id % NR_PACKETS != 0)) return;  // new Rx or Tx, sync id to multiple of NR_PACKETS
    if (++count >= NR_PACKETS) {  
      int rsi =  max(NR_PACKETS-packetsLost,0);
      struct Telemetry tel = {0, rc.id, lowVBatt.read(), vBatt.read(), rsi, timer.secondsLeft(), totalLost, errors};
      tel.checkSum = CheckSum(tel);
      if (!send_data((const uint8_t *)&tel, sizeof(tel))) errors++;
      int avg_time = (int)(now - timeLast1st) / NR_PACKETS;
      logger->printf("Rx:%s] thr:%d ch1:%d ch2:%d ch3:%d ms:%d rsi:%d t:%d maxt:%d lost:%d errors:%d id:%d\n", 
                      status.str(), thrLast, rc.chan1,  rc.chan2,  rc.chan3, avg_time, rsi, timer.secondsLeft(), 
                      maxSecs, totalLost, errors, rc.id);
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
        logger->printf("FAILSAFE!!!!\n");
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
  }
private:
  RcTimer timer{maxFlight};
  Throttle throttle{pinThrottle, &timer, minThrottle, maxFlight, escWarning, nrWarns, stopEngine};
  Potmeter maxTime{pinMaxTime, 10, maxFlight}; 
  RcServo chan1{pinChan1, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan2{pinChan2, TxMinPulse, TxMaxPulse, 0, -1};
  RcServo chan3{pinChan3, TxMinPulse, TxMaxPulse, 0, -1};
  VoltageDiv vBatt{pinVBatt, lipoDivR1, lipoDivR2};
  Potmeter lowVBatt{pinVBattLow, 0, vBatt.max()}; 
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true};  // todo
  bool connected = false, waitThrHold = true;
  unsigned long timeLastTx = 0, timeLast1st = 0;  // time last 1st packadge;
  int maxSecs = -1, thrLast = -1, lastId = -1, count = -1, packetsLost = 0, totalLost = 0, errors = 0;
};

Rx *rx = 0; // initialisation must in setup 

void setup() {
  Serial.begin(115200);
  logger = new Logger(BLE_device_Rx, 120);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  rx = new Rx(macTx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx->add_self())) {
    logger->printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  logger->printf("OpenRC4CL %s, Rx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                  OpenRC4CL_VERSION, WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
}

void loop() {
  rx->checkFailsafe();
  rx->statusUpdate();
}
