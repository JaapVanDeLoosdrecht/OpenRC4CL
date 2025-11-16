/* 
Rx OpenRC4CL 16 November 2025

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

// NOTE: this is only tested on XIAO ESP32-C6, use partion schema NO ATO (2MB APP/2MB SPIFFS)
// RX com7 white usb

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_chan, mac address and BLE_device
#ifndef SECRET
#define WIFI_CHANNEL 6
const MacAddress macTx(({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with mac address of Tx
char *BLE_device_Rx = "Rx-OpenRC4CL";                           // modify with your name
#endif

// user settings
const int maxFlight = 9;                    // seconds
const int warnEndFlight = 3;                // seconds before max
const int nrWarns = 2;                      // number if throttle warnings befor stopping engine
const int minThrottle = TxMidPulse;         // min trhottle value to start timer, 0 = no restart timer 
// hardware
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinChan1 = A1;                    
const int pinMaxThrottle = A2;
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 8S
const int lipoDivR2 = 100000;               // R2 lipo voltage divider
// system
const int NR_PACKETS = 50;                  // nr packets for telemetry and logging
const unsigned long FAILSAFE_TIME = 500;    // ms
Logger *logger = 0; 

class VoltageDiv {
public:
  VoltageDiv(int pin, int r1, int r2) {  _pin = pin; _r1 = r1; _r2 = r2; _div = ((r1+r2)/r1); }
  int read() { return avgAnalogMilliVolts(_pin, 16) * _div; }  // V in mV
private:
  int _pin, _r1, _r2, _div;
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
      logger->printf("[Rx] id:%d, thr:%d, rcthr:%d, ch1:%d, ch2:%d, ch3:%d, ms:%d, rsi:%d, t:%d, stop:%d, lost:%d, mthr:%d\n", 
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
        logger->printf("FAILSAFE!!!!\n");
        timeLastTx = now;                  // another test in FAILSAFE_TIME
      }
    }
  }
  void checkVBatt() {    // WIP
    // static bool warnOnce = false;
    // if (!warnOnce && (timer.seconds() > 3)) {
    //   throttle.warnAndStop();
    //   warnOnce = true;
    //   logger->printf("Bat low!!!!\n");
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
  rx->checkVBatt();
  rx->statusUpdate();
}
