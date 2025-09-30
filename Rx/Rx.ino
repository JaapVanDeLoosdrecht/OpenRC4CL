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

/*
Todo
- buzzer on Tx
- first tx throttle = 0 before start throttle > 0  OR kill push button on Rx
- Add optional pin for thr cut switch
- using battery as powersupply
- lipo monitoring and low bat, 
  make settup with V divider using R for max 6S (or use 1 lipo only)
  use Schottky diode for protection?? and compensate diode loss in calculation
- log own mac address at startup
- soft mac address
- paper display on TX
- use IMU as crash sensor to kill engine
- use the BluetoothSerial.h for logging with Bluetooth Terminal App (Google play)
*/

// RX com7 white usb
// Tx mac 98:A3:16:61:1D:5C Rx mac 98:A3:16:61:12:D4

#include "MacAddress.h"
#include "WiFi.h"
#include "OpenRC4CL_util.h"

#define LOG
#define WIFI_CHANNEL 6
const MacAddress macTx({0x98, 0xA3, 0x16, 0x61, 0x1D, 0x5C});

const int maxFlight = 9;       // seconds
const int warnEndFlight = 4;   // seconds before max
const int nrWarns = 2;
const int startTimerThrottle = TxMidPulse;  // 0 = no restart timer with initial throttle

const int pinThrottle = D0;
const int pinChan1 = D1;
const int NR_PACKETS = 50;                  // nr packets for telemetry and logging
const unsigned long FAILSAFE_TIME = 500;    // ms

RcTimer timer(maxFlight);
RcServo chan1(pinChan1, TxMinPulse, TxMaxPulse, 0, -1);
Throttle throttle(pinThrottle, timer, startTimerThrottle, maxFlight, warnEndFlight, nrWarns);
unsigned long timeLastTx = 0;

class Rx : public RcPeer {  
public:
  Rx(MacAddress mac_tx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : RcPeer(mac_tx, channel, iface, lmk) {}
  void onReceive(const uint8_t *data, size_t len, bool broadcast) { 
    static int lastId = -1;
    static int count = -1;
    static int packetsLost = 0;
    static unsigned long timeLast = 0;           // time last 1st packadge
    struct TxData rc = *(struct TxData *)data;   // copy received data
    unsigned long now = millis();
    if (count == -1) timer.start();              // timer can be reset using first minimal throttle command
    if (CheckSum(rc) == rc.checkSum) { 
      timeLastTx = now;
      throttle.writeTx(rc.throttle);
      chan1.writeTx(rc.chan1);
      if (rc.id < 100) { packetsLost = 0; lastId = -1; count = -1; }  // new Tx   
      if (lastId > 0) packetsLost += rc.id - lastId - 1;
      lastId = rc.id;
    } else {
      packetsLost++;
    }
    if ((count == -1) && (rc.id % NR_PACKETS != 0)) return;  // new Rx or Tx, sync id to multiple of NR_PACKETS
    if (++count == NR_PACKETS) {  
      struct Telemetry tel = {0, rc.id, 0, 123, max(NR_PACKETS-packetsLost,0), timer.elapsed()};
      tel.checkSum = CheckSum(tel);
      this->send_data((const uint8_t *)&tel, sizeof(tel));
      #ifdef LOG
        int avg_time = (int)(now - timeLast) / NR_PACKETS;
        Serial.printf("[Rx] id:%d, thr:%d, ch1:%d, ms:%d, lost:%d, t:%d\n", 
                      rc.id, rc.throttle, rc.chan1, avg_time, packetsLost, timer.seconds());
        count = packetsLost = 0;
        timeLast = now;
      #endif
    }
  }
};

void failsafe() { throttle.failsafe(); chan1.failsafe(); }

void setup() {
  static Rx rx(macTx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);
  failsafe();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!rx.add_self())) {
    Serial.printf("Failed to initialize Rx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  Serial.printf("Rx channel: %d, MAC Address: %s, ESP-NOW version: %d\n", WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
}

void checkVBatt() {    // WIP
  // static bool warnOnce = false;
  // if (!warnOnce && (timer.seconds() > 3)) {
  //   throttle.WarningNow();
  //   warnOnce = true;
  //   Serial.printf("FAILSAFE!!!!\n");
  // }
}

void checkFailsafe() { 
  unsigned long last = timeLastTx;  // copy of last before geting now
  unsigned long now = millis();
  if ((now - last) > FAILSAFE_TIME) {      // note check can be interrupted by callback, abs doesn't work on unsigned
    failsafe();
    Serial.printf("FAILSAFE!!!!\n");
    timeLastTx = now;               // another test in FAILSAFE_TIME
  }
}

void loop() {
  checkFailsafe();
  checkVBatt();
}
