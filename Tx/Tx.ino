/* 
TX OpenRC4CL 12 December 2025

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
// Tx com5 black usb 

#include <MacAddress.h>
#include <WiFi.h>
#include "OpenRC4CL_util.h"  
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out, specify wifi_chan, mac address and BLE_device
#ifndef SECRET
#define WIFI_CHANNEL 6
const MacAddress macRx({0x00, 0x00, 0x00, 0x00, 0x00, 0x00});  // modify with your mac address of Rx
char *BLE_device_Tx = "Tx-OpenRC4CL";                          // modify with your name
#endif

// user settings
const int nrWarns = 2;                      // number if throttle warnings before stopping engine
const int stopEngine = 1;                   // seconds to stop engine after last warning  TODO or do this manuanlly with TH??
// hardware
const int pinLed = LED_BUILTIN;             // Note LED_BUILTIN is reversed on C6
const int pinThrottle = A0;
const int pinThrottleHold = D3;
const int pinLeftCh1 = D4;
const int pinRightCh1 = D5;
const int pinLeftCh2 = D6;
const int pinRightCh2 = D7;
const int pinCh3 = PIN_NOT_USED;            // A2; 
const int pinDeadBand = A2;                 // A4; 
const int pinBeep = D8;   
const int pinWifi0 = D9;
const int pinWifi1 = D10;
const int pinVBatt = A1;                      
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 5V usb, div=2
const int lipoDivR2 = 10000;                // R2 lipo voltage divider
// system
const int txBattLow = 3500;                 // min voltage for Tx lipo
const int deadBandMax = ThrHoldDelta-10;    // deadband delta for throttle, no overleap with TH
const int deadBandMin = -deadBandMax;              

Logger *logger = 0; 

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk): RcPeer(mac_rx, channel, iface, lmk) {}
  void wait4ThrHold() {
    logger->printf("[Tx] wait for throttle hold\n");
    status.value = Status::WaitThrHold;
    while (hold.readPos() != Thr_Hold) { delay(100); statusUpdate(); }
    logger->printf("[Tx] waiting for Rx\n");
    status.value = Status::WaitTxRx;
  }
  int readThrottle() {
    throttle.setMinMax(TxMinPulse + deadBand.read());
    return (hold.readPos() == Thr_Hold) ? TxThrottleHoldPulse : throttle.read();
  }
  void sendTx() {
    struct TxData rc{0, ++id, readThrottle(), chan1.read(), chan2.read(), chan3.read()}; 
    rc.checkSum = CheckSum(rc);
    if (send_data((uint8_t *)&rc, sizeof(rc)) || !connected) {   
      lastId = id;
    } else {
      if (abs(id - lastId) > 1) { status.value = Status::Error; errors++; }
    }
  }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    struct Telemetry tel = *(struct Telemetry *)data;
    connected = true;
    if (CheckSum(tel) != tel.checkSum) { 
      status.value = Status::Error; errors++;
      logger->printf("[Tx:%s bat:%d thr:%d err:%d]\n", status.str(), txBatt.read(), readThrottle(), errors);
      return;
    }
    bool txBattLow = (txBatt.read() < txBattLow);
    bool vBattLow = (tel.vBatLow > 0) && (tel.vBat < tel.vBatLow);
    bool eot = tel.time_left < warnEndFlight;
    if (!(endFlight = txBattLow || vBattLow || eot)) {
      status.value = Status::Ok;                                                                                                                                                                                              ;
      beepEndFlight = false; // needed if Rx is reset after end of flight
    } else {
      status.value = (txBattLow) ? Status::TxBattLow : (vBattLow) ? Status::VBattLow : Status::EndFlight;
    }
    logger->printf("[Tx:%s bat:%d thr:%d dead:%d err:%d] [tele vBat:%d vLow:%d rsi:%d time:%d lost:%d err:%d id:%d]\n", 
                   status.str(), txBatt.read(), readThrottle(), deadBand.read(), errors, tel.vBat, tel.vBatLow, tel.rsi, 
                   tel.time_left, tel.totalLost, tel.errors, tel.id);
  }
  void statusUpdate() { 
    led.set(status.pulse()); led.update(); 
    if (endFlight) {
      if (!beepEndFlight) { beep.set(status.pulse(), nrWarns); beepEndFlight = true; }  
    } else {
      beep.set(status.pulse());
    }
    beep.update(); 
  }
private:
  static const Switch::Pos Thr_Hold = Switch::middle;
  Potmeter throttle{pinThrottle};
  Switch hold{pinThrottleHold};
  Switch chan1{pinLeftCh1, pinRightCh1};
  Switch chan2{pinLeftCh2};
  Potmeter chan3{pinCh3};
  Potmeter deadBand{pinDeadBand, deadBandMin, deadBandMax};
  VoltageDiv txBatt{pinVBatt, lipoDivR1, lipoDivR2};
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true};  
  Beep beep{pinBeep, status.pulse()};
  bool connected = false, endFlight = false, beepEndFlight = false;
  int id = 0, lastId = 0, errors = 0;  // #errors = #send + #checksum
  int warnEndFlight = nrWarns * Status::pulseTab[Status::EndFlight]/1000 + stopEngine;
};

Tx *tx = 0;  // initialisation must in setup due to changing pinModes to OUTPUT

void setup() {
  Serial.begin(115200);
  logger = new Logger(BLE_device_Tx, 120);
  int pins_dip[] = {pinWifi0, pinWifi1};
  DipSwitch dip(2, pins_dip);
  int wifiChan = (pinWifi0 != PIN_NOT_USED) ? wifiChans[dip.read()] : WIFI_CHANNEL;
  WiFi.mode(WIFI_STA); WiFi.setChannel(wifiChan);
  while (!WiFi.STA.started()) delay(100);
  tx = new Tx(macRx, wifiChan, WIFI_IF_STA, nullptr);
  if ((!ESP_NOW.begin()) || (!tx->add_self())) {
    logger->printf("Failed to initialize Tx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  logger->printf("OpenRC4CL %s, Tx channel:%d, MAC Address:%s, ESP-NOW version:%d\n", 
                  OpenRC4CL_VERSION, wifiChan, WiFi.macAddress().c_str(), ESP_NOW.getVersion());
  tx->wait4ThrHold();
}

void loop() {
  tx->sendTx();
  tx->statusUpdate();
  delay(15);  // some headroom to have at least 50Hz
}
