/* 
Rx OpenRC4CL 26 October 2025

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

// NOTE: this is only tested on XIAO ESP32-C6
// RX com7 white usb

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"
#include "mac_chan.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_chan and mac address
#ifndef MAC_CHAN
#define WIFI_CHANNEL 6
const MacAddress macTx(({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with mac address of Tx
#endif

// user settings
const int maxFlight = 9;                    // seconds
const int warnEndFlight = 3;                // seconds before max
const int nrWarns = 2;                      // number if throttle warnings befor stopping engine
const int minThrottle = TxMidPulse;         // min trhottle value to start timer, 0 = no restart timer 
const int maxV1cell = 4200;                 // max voltage 1 lipo cell, LiHV = 4350
const int lipoDivR1 = 10000;                // R1 lipo voltage divider
const int lipoDivR2 = 100000;               // R2 lipo voltage divider
// hardware
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinChan1 = A1;                    
const int pinMaxThrottle = A2;
// system
const int NR_PACKETS = 50;                  // nr packets for telemetry and logging
const unsigned long FAILSAFE_TIME = 500;    // ms

class LipoV {
public:
  LipoV(int pin, int r1 = lipoDivR1, int r2 = lipoDivR2, int maxV = maxV1cell) {  
    _pin = pin; _r1 = r1; _r2 = r2; _div = ((r1+r2)/r1); _maxV = maxV; 
    int mV = readV();
    for (_nrCells = MaxCells; _nrCells == 1; _nrCells--) {
      if (mV > (_maxV * c / _div) * PercentFull / 100) break;
    }
  }
  int read() { return readV() / _nrCells; }  // avgV cell in mV
  int readV() { return refV(avgAnalogMilliVolts(_pin, 16) * _div); }  // V all cells in mV
  int nrCells() { return _nrCells; }
private:
  const int MaxCells = 8, PrecentFull = 90; 
  int _pin, _r1, _r2, _div, _maxV, _nrCells;
};

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : RcPeer(mac_tx, channel, iface, lmk) { 
    throttle.failsafe(); chan1.failsafe(); 
  }
  void command(struct TxData &rc, unsigned long now) {
    if (!connected) { connected = true; timer.start(); led.set(StatusOk); }  // timer can be reset using first minimal throttle command
    if (CheckSum(rc) == rc.checkSum) { 
      timeLastTx = now;
      int mThr = maxThrottle.read();
      if (firstThrHold && rc.throttle > TxThrottleHoldPulse) { firstThrHold = false; }
      if (abs(thrMax-mThr) > 5) thrMax = mThr;
      thrLast = throttle.writeTx(min(rc.throttle, thrMax));
      chan1.writeTx(rc.chan1);
      if (rc.id < 100) {  // new Tx 
        timer.start(); led.set(StatusOk); 
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
      struct Telemetry tel = {0, rc.id, 0, 123, rsi, timer.secondsLeft(), throttle.isStop(), totalLost};
      tel.checkSum = CheckSum(tel);
      send_data((const uint8_t *)&tel, sizeof(tel));
      int avg_time = (int)(now - timeLast1st) / NR_PACKETS;
      Serial.printf("[Rx] id:%d, thr:%d, rcthr:%d, ch1:%d, ch2:%d, ch3:%d, ms:%d, rsi:%d, t:%d, stop:%d, lost:%d, mthr:%d\n", 
                    rc.id, thrLast, rc.throttle, rc.chan1,  rc.chan2,  rc.chan3, avg_time, rsi, timer.secondsLeft(), throttle.isStop(), totalLost, thrMax);
      count = packetsLost = 0;
      timeLast1st = now;
    }
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) { 
    struct TxData rc = *(struct TxData *)data;  // copy received data
    unsigned long now = millis();
    command(rc, now);
    telemetry(rc, now);
    led.set((timer.elapsed()) ? StatusEndFlight : (packetsLost > 0) ? StatusError : (firstThrHold) ? StatusWaitThrHold: StatusOk);
  }
  void statusUpdate() { led.update(); }
  void checkFailsafe() { 
    if (connected) {
      unsigned long last = timeLastTx;     // copy of last before geting now, this function can be interupted
      unsigned long now = millis();
      if ((now - last) > FAILSAFE_TIME) {  // note check can be interrupted by callback, abs doesn't work on unsigned
        if (timer.elapsed()) throttle.failsafe(); else throttle.warnAndStop();
        chan1.failsafe();
        led.set(StatusFailsafe);
        Serial.printf("FAILSAFE!!!!\n");
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
  }
  void checkVBatt() {    // WIP
    // static bool warnOnce = false;
    // if (!warnOnce && (timer.seconds() > 3)) {
    //   throttle.warnAndStop();
    //   warnOnce = true;
    //   Serial.printf("Bat low!!!!\n");
    // }
  }
private:
  RcTimer timer{maxFlight};
  RcServo chan1{pinChan1, TxMinPulse, TxMaxPulse, 0, -1};
  Throttle throttle{pinThrottle, &timer, minThrottle, maxFlight, warnEndFlight, nrWarns};
  Potmeter maxThrottle{pinMaxThrottle, TxMinPulse, TxMaxPulse};
  Led led{pinLed, StatusWaitTxRx};
  bool connected = false, firstThrHold = true;
  unsigned long timeLastTx = 0, timeLast1st = 0;  // time last 1st packadge;
  int thrLast = -1, thrMax = -1, lastId = -1, count = -1, packetsLost = 0, totalLost = 0;
};

Rx *rx = 0; // initialisation must in setup 

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  rx = new Rx(macTx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx->add_self())) {
    Serial.printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  Serial.printf("OpenRC4CL %s, Rx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                 OpenRC4CL_VERSION, WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
}

void loop() {
  rx->checkFailsafe();
  rx->checkVBatt();
  rx->statusUpdate();
}
