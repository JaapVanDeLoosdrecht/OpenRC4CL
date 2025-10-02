/* 
OpenRC4CL V0.02 2 October 2025

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

// todo Switch as killswitch and potmeter (instel pot) in arm box for normal handle

// Tx com5 black usb 

#include "MacAddress.h"
#include "WiFi.h"
#include "OpenRC4CL_util.h"

// #define DEBUG
#define LOG
#define WIFI_CHANNEL 5
const MacAddress macRx({0x??, 0x??, 0x??, 0x??, 0x??, 0x??});  // modify with mac address of Rx

void debugTx(struct TxData &rc) {
  static int count = 0;
  static unsigned long time_start = millis();
  const int NR_PACKETS = 50;
  if (++count == NR_PACKETS) {
    Serial.printf("[Tx] id:%d, thr:%d, ch1:%d, avg_ms:%d\n", 
                  rc.id, rc.throttle, rc.chan1, (millis() - time_start) / NR_PACKETS);
    time_start = millis();
    count = 0;
  }
}

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk): RcPeer(mac_rx, channel, iface, lmk) {}
  void sendTx() {
    const int pinThrottle = D0;
    const int pinLeft = D2;
    const int pinRight = D3;
    static Potmeter throttle(pinThrottle);
    static Switch chan1(pinLeft, pinRight);
    static int id = 0;
    struct TxData rc;
    rc.id = ++id;
    rc.throttle = throttle.Read();
    rc.chan1 = chan1.readTx(); 
    rc.checkSum = CheckSum(rc);
    bool s = this->send_data((uint8_t *)&rc, sizeof(rc));
    #ifdef DEBUG
    if (s) debugTx(rc); else Serial.printf("[Tx] FAILED TO SEND id: %d\n", id);
    #endif
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    struct Telemetry tel = *(struct Telemetry *)data;
    if (CheckSum(tel) != tel.checkSum) { 
      Serial.printf("[Tx tele] CHECKSUM ERROR id: %d\n", tel.id);
      return;
    }
    #ifdef LOG
      Serial.printf("[Tx tele] id:%d, vLow:%d, v:%d, rsi:%d, mtime:%d\n", tel.id, tel.vBatLow, tel.vBat, tel.rsi, tel.max_time);
    #endif
  }
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
  Serial.printf("Tx channel: %d, MAC Address: %s, ESP-NOW version: %d\n", WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
}

void loop() {
  tx.sendTx();
  delay(20);
}
