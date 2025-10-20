/* 
TX OpenRC4CL 20 October 2025

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

// Tx com5 black usb 

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"  
#include "mac_chan.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_can and mac address
#ifndef MAC_CHAN
#define WIFI_CHANNEL 6
const MacAddress macRx({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with mac address of Rx
#endif

const int pinLed = LED_BUILTIN;
const int pinThrottle = A0;
const int pinThrottleHold = D3;
const int pinLeftCh1 = D4;
const int pinRightCh1 = D5;

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk): RcPeer(mac_rx, channel, iface, lmk) {}
  void wait4ThrHold() {
    Serial.printf("[Tx] wait for throttle hold\n");
    led.setPulse(Led::WaitThrHold);
    while (hold.read() != Thr_Hold) { delay(100); led.update(); }
    Serial.printf("[Tx] throttle hold activated\n");
    led.setPulse(Led::Error);
  }
  void sendTx() {
    int thr = throttle.Read();
    if (hold.read() == Thr_Hold) thr = TxThrottleHoldPulse;
    struct TxData rc{0, ++id, thr, chan1.readTx()}; 
    rc.checkSum = CheckSum(rc);
    if ((!this->send_data((uint8_t *)&rc, sizeof(rc))) && connected) {
      led.setPulse(Led::Error);
      Serial.printf("[Tx] FAILED TO SEND id: %d\n", id);
    }
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    struct Telemetry tel = *(struct Telemetry *)data;
    connected = true;
    if (CheckSum(tel) != tel.checkSum) { 
      led.setPulse(Led::Error);
      Serial.printf("[Tx tele] CHECKSUM ERROR id: %d\n", tel.id);
      return;
    }
    led.setPulse((tel.time_left > 0) ? Led::Ok : Led::EndFlight);
    Serial.printf("[Tx tele] id:%d, vLow:%d, v:%d, rsi:%d, time:%d, lost:%d\n", 
                   tel.id, tel.vBatLow, tel.vBat, tel.rsi, tel.time_left, tel.totalLost);
  }
  void ledUpdate() { led.update(); }
private:
  static const Switch::Pos Thr_Hold = Switch::middle;
  Potmeter throttle{pinThrottle};
  Switch hold{pinThrottleHold};
  Switch chan1{pinLeftCh1, pinRightCh1};
  Led led{pinLed, Led::WaitTxRx};
  bool connected = false;
  int id = 0;
};

Tx tx(macRx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(100);
  if ((!ESP_NOW.begin()) || (!tx.add_self())) {
    Serial.printf("Failed to initialize Tx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  Serial.printf("OpenRC4CL %s, Tx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                 OpenRC4CL_VERSION, WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
  tx.wait4ThrHold();
}

void loop() {
  tx.sendTx();
  tx.ledUpdate();
  delay(20);
}
