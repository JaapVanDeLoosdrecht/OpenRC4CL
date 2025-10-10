/* 
Rx OpenRC4CL 10 October 2025

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

// RX com7 white usb

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"
#include "mac_chan.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_can and mac address
#ifndef MAC_CHAN
#define WIFI_CHANNEL 6
const MacAddress macTx(({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with mac address of Tx
#endif

const int maxFlight = 15;    // seconds
const int warnEndFlight = 3;   // seconds before max
const int nrWarns = 2;
const int minThrottle = TxMidPulse;  // min trhottle value to start timer, 0 = no restart timer 

const int pinThrottle = A0;
const int pinChan1 = A1;                    // todo enable again. find another anologe pin or use digital pin
const int pinMaxThrottle = A2;
const int NR_PACKETS = 50;                  // nr packets for telemetry and logging
const unsigned long FAILSAFE_TIME = 500;    // ms

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : 
    RcPeer(mac_tx, channel, iface, lmk) { throttle.failsafe(); chan1.failsafe();  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) { 
    int thr = -1;
    struct TxData rc = *(struct TxData *)data;  // copy received data
    unsigned long now = millis();
    if (!connected) { connected = true; timer.start(); }  // timer can be reset using first minimal throttle command
    if (CheckSum(rc) == rc.checkSum) { 
      timeLastTx = now;
      int mThr = maxThrottle.Read();
      if (abs(thrMax-mThr) > 5) thrMax = mThr;
      thr = throttle.writeTx(min(rc.throttle, thrMax));
      chan1.writeTx(rc.chan1);
      if (rc.id < 100) { packetsLost = 0; lastId = -1; count = -1; }  // new Tx   
      if (lastId > 0) packetsLost += rc.id - lastId - 1;
      lastId = rc.id;
    } else {
      packetsLost++; totalLost++;
    }
    if ((count == -1) && (rc.id % NR_PACKETS != 0)) return;  // new Rx or Tx, sync id to multiple of NR_PACKETS
    if (++count >= NR_PACKETS) {  
      int rsi =  max(NR_PACKETS-packetsLost,0);
      struct Telemetry tel = {0, rc.id, 0, 123, rsi, timer.secondsLeft(), totalLost};
      tel.checkSum = CheckSum(tel);
      send_data((const uint8_t *)&tel, sizeof(tel));
      int avg_time = (int)(now - timeLast1st) / NR_PACKETS;
      Serial.printf("[Rx] id:%d, thr:%d, rcthr:%d, ch1:%d, ms:%d, rsi:%d, t:%d, lost:%d\n", 
                    rc.id, thr, rc.throttle, rc.chan1, avg_time, rsi, timer.secondsLeft(), totalLost);
      count = packetsLost = 0;
      timeLast1st = now;
    }
  }
  void checkFailsafe() { 
    unsigned long last = timeLastTx;     // copy of last before geting now
    unsigned long now = millis();
    if ((now - last) > FAILSAFE_TIME) {  // note check can be interrupted by callback, abs doesn't work on unsigned
      throttle.warnAndStop(); 
      chan1.failsafe();
      Serial.printf("FAILSAFE!!!!\n");
      timeLastTx = now;                  // another test in FAILSAFE_TIME
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
  bool connected = false;
  unsigned long timeLastTx = 0, timeLast1st = 0;  // time last 1st packadge;
  int thrMax = -1, lastId = -1, count = -1, packetsLost = 0, totalLost = 0;
};

Rx rx(macTx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx.add_self())) {
    Serial.printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  Serial.printf("OpenRC4CL %s, Rx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                 OpenRC4CL_VERSION, WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
}

void loop() {
  rx.checkFailsafe();
  rx.checkVBatt();
}
