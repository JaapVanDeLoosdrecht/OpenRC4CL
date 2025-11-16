/* 
TX OpenRC4CL 16 November 2025

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
// Tx com5 black usb 

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"  
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_chan, mac address and BLE_device
#ifndef SECRET
#define WIFI_CHANNEL 6
const MacAddress macRx({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with mac address of Rx
char *BLE_device_Tx = "Tx-OpenRC4CL";                          // modify with your name
#endif

const int pinLed = LED_BUILTIN;  // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinThrottleHold = D3;
const int pinLeftCh1 = D4;
const int pinRightCh1 = D5;
const int pinLeftCh2 = D6;
const int pinRightCh2 = D7;
const int pinCh3 = A5;    // todo, is tested with A1
const int pinBeep = D8;   

Logger *logger = 0; 

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk): RcPeer(mac_rx, channel, iface, lmk) {}
  void wait4ThrHold() {
    logger->printf("[Tx] wait for throttle hold\n");
    led.set(StatusWaitThrHold); beep.set(StatusWaitThrHold);  
    while (hold.readPos() != Thr_Hold) { delay(100); statusUpdate(); }
    logger->printf("[Tx] throttle hold activated\n");
    led.set(StatusWaitTxRx); beep.set(StatusOk);
  }
  void sendTx() {
    int thr = throttle.read();
    if (hold.readPos() == Thr_Hold) thr = TxThrottleHoldPulse;
    struct TxData rc{0, ++id, thr, chan1.read(), chan2.read(), chan3.read()}; 
    rc.checkSum = CheckSum(rc);
    if (this->send_data((uint8_t *)&rc, sizeof(rc)) || !connected) {   
      lastId = id;
    } else {
      if (abs(id - lastId) > 5) {
        led.set(StatusError); if (!beep_end_flight) beep.set(StatusError);
        logger->printf("[Tx] FAILED TO SEND id: %d\n", id);
      }
    }
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    struct Telemetry tel = *(struct Telemetry *)data;
    connected = true;
    if (CheckSum(tel) != tel.checkSum) { 
      led.set(StatusError); if (!beep_end_flight) beep.set(StatusError);
      logger->printf("[Tx tele] CHECKSUM ERROR id: %d\n", tel.id);
      return;
    }
    if (!tel.stop) {
      led.set(StatusOk); beep.set(StatusOk); 
      beep_end_flight = false;  // possible restart Rx:  
    } else {
      led.set(StatusEndFlight);
      if (!beep_end_flight) {beep.set(StatusEndFlight, 2); beep_end_flight = true; }
    }
    logger->printf("[Tx tele] id:%d, vLow:%d, v:%d, rsi:%d, time:%d, stop:%d, lost:%d\n", 
                    tel.id, tel.vBatLow, tel.vBat, tel.rsi, tel.time_left, tel.stop, tel.totalLost);
  }
  void statusUpdate() { led.update(); beep.update(); }
private:
  static const Switch::Pos Thr_Hold = Switch::middle;
  Potmeter throttle{pinThrottle};
  Switch hold{pinThrottleHold};
  Switch chan1{pinLeftCh1, pinRightCh1};
  Switch chan2{pinLeftCh2};
  Potmeter chan3{pinCh3};
  Led led{pinLed, StatusWaitTxRx, true};  
  Beep beep{pinBeep, StatusWaitTxRx};
  bool connected = false, beep_end_flight = false;
  int id = 0, lastId = 0;
};

Tx *tx = 0;  // initialisation must in setup due to changing pinModes to OUTPUT

void setup() {
  Serial.begin(115200);
  logger = new Logger(BLE_device_Tx);
  WiFi.mode(WIFI_STA); WiFi.setChannel(WIFI_CHANNEL);
  while (!WiFi.STA.started()) delay(100);
  tx = new Tx(macRx, WIFI_CHANNEL, WIFI_IF_STA, nullptr);
  if ((!ESP_NOW.begin()) || (!tx->add_self())) {
    logger->printf("Failed to initialize Tx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  logger->printf("OpenRC4CL %s, Tx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                  OpenRC4CL_VERSION, WIFI_CHANNEL, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
  tx->wait4ThrHold();
}

void loop() {
  tx->sendTx();
  tx->statusUpdate();
  delay(15);  // some headroom to have at least 50Hz
}
