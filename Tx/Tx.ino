/* 
TX OpenRC4CL 19 April 2026

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
// Tx com5 (proto=8) white 1st usb 

// #define PROTOBOARD

#include <MacAddress.h>
#include <WiFi.h>
#include <limits.h>
#include "OpenRC4CL_util.h"  
#include "secret.h"  # NOTE this file is NOT in repro and this line should be commented out
#ifndef SECRET
char* macRx = "00:00:00:00:00:00";          // modify with your mac address of Rx or use serial CMDs
#endif

// system
const int deadBandMax = ThrHoldDelta-10;    // deadband delta for throttle, no overleap with TH
const int deadBandMin = -deadBandMax;    
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 5V usb, div=2
const int lipoDivR2 = 10000;                // R2 lipo voltage divider
// user settings (NVS params)
String devName = "Tx-OpenRC4CL";
String macPeer(macRx); 
String passwd = "123";
String date = buildDate();
int wifiChan = 6;                           // [1..13]
int deadBand = 0;                           // for TH low
int txBattLow = 3500;                       // min voltage for Tx lipo
int nrWarns = 3;                            // number beeps if TxBattLow or timer is expired number 

NVS* buildNVS(Logger *logger) {
  const int max_nvs_params = 16; 
  // nvs_erase();
  NVS* nvs = new NVS(max_nvs_params, logger); 
  nvs->add(NVS_STR(devName));
  nvs->add(NVS_STR(macPeer));
  nvs->add(NVS_STR(passwd));
  nvs->add(NVS_STR(date));
  nvs->add(NVS_INT(wifiChan, 1, 13));
  nvs->add(NVS_INT(deadBand, deadBandMin, deadBandMax));  // deadband delta for throttle
  nvs->add(NVS_INT(txBattLow, 3000, 4350));
  nvs->add(NVS_INT(nrWarns, 1, 10));
  return nvs;
}

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk, Logger *log):
    RcPeer(mac_rx, channel, iface, lmk) { logger = log; }
  void wait4ThrHold() {
    logger->printf("[Tx] wait for throttle hold\n");
    status.value = Status::WaitThrHold;
    while (hold.readPos() != Thr_Hold) { delay(100); statusUpdate(); }
    logger->printf("[Tx] waiting for Rx\n");
    status.value = Status::WaitTxRx;
  }
  void update() { sendTx(); statusUpdate(); }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    struct Telemetry tel = *(struct Telemetry *)data;
    if (CheckSum(tel) != tel.checkSum) { 
      status.value = Status::Error; errors++;
      logger->printf("[Tx:%s bat:%d thr:%d err:%d]\n", status.str(), txBatt.read(), readThrottle(), errors);
      return;
    }
    if (tel.status == Status::WaitThrHold) {
      status.value = Status::WaitThrHold; 
    } else {
      bool vBattLow = (tel.vBat > vbatThr) && (tel.vBat < tel.vBatLow);
      if (!(endFlight = vBattLow || (tel.time_left < 0))) {
        status.value = Status::Ok;                                                                                                                                                                                              ;
        beepEndFlight = false; // needed if Rx is reset after end of flight
      } else {
        status.value = (vBattLow) ? Status::VBattLow : Status::EndFlight;
      }
    }
    logger->printf("[Tx:%s bat:%d thr:%d err:%d] [Rx:%s vBat:%d vLow:%d rsi:%d time:%d lost:%d err:%d]\n", 
                   status.str(), txBatt.read(), readThrottle(), errors, 
                   Status::val2str(tel.status), tel.vBat, tel.vBatLow, tel.rsi, tel.time_left, tel.totalLost, tel.errors);
  }
protected:
  int readThrottle() {
    throttle.setMinMax(TxMinPulse + deadBand);  // deadBand can be changed by CMD
    return (hold.readPos() == Thr_Hold) ? TxThrottleHoldPulse : throttle.read();
  }
  void sendTx() {
    struct TxData rc{0, ++id, readThrottle(), chan1.read(), chan2.read(), chan3.read(), chan4.read()}; 
    rc.checkSum = CheckSum(rc);
    if (send_data((uint8_t *)&rc, sizeof(rc)) || !connected) {   
      lastId = id;
    } else {
      status.value = Status::Error; errors++;
    }
    statusUpdate();
  }
  void statusUpdate() { 
    if (!endFlight) {
      int txV = txBatt.read();                             // Check txV here because Rx could be off and so no telemetry
      if (endFlight = (txV > vbatThr) && (txV < txBattLow)) {  // tBatt ~ 0 if usb powered without lipo
        status.value = Status::TxBattLow;
      }
    }
    led.set(status.pulse()); led.update(); 
    if (endFlight) {
      if (!beepEndFlight) { 
        beep.set(status.pulse(), nrWarns); 
        beepEndFlight = true; 
      }  
    } else {
      int p = status.pulse();
      beep.set((p == Status::Ok) ? p = -1 : p);            // no beep if Ok
    }
    beep.update(); 
  }
private:
  static const Switch::Pos Thr_Hold = Switch::middle;
  static const int vbatThr = 500;                          // if vbat not connected on Rx -> vbat < vbatThr
  Potmeter throttle{pinThrottle};
  Switch chan1{pinLeftCh1, pinRightCh1};
  Potmeter chan2{pinCh2};
  Potmeter chan3{pinCh3};
  Potmeter chan4{pinCh4};
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true};  
  #ifdef PROTOBOARD                                          // TODO
    Switch hold{D3};
    Beep beep{D8, status.pulse()}; 
    VoltageDiv txBatt{A1, lipoDivR1, lipoDivR2}; 
  #else
    Switch hold{pinThrottleHold};
    Beep beep{pinBeep, status.pulse()};
    VoltageDiv txBatt{pinVBatt, lipoDivR1, lipoDivR2};
  #endif
  bool connected = false, endFlight = false, beepEndFlight = false;
  int id = 0, lastId = 0, errors = 0;      // #errors = #send + #checksum
  Logger *logger = 0;
};

Tx *tx = 0;    // initialisation must be in setup due to changing pinModes to OUTPUT
CMD* cmd = 0; 

void setup() {
  Serial.begin(115200);
  SerialBLE.begin(devName); 
  Logger *logger = new Logger;    // Serial and BLE logger
  NVS* nvs = buildNVS(logger);
  cmd = new CMD(nvs, logger);
  WiFi.mode(WIFI_STA); WiFi.setChannel(wifiChan);
  while (!WiFi.STA.started()) delay(100);
  tx = new Tx(MacAddress(macPeer), wifiChan, WIFI_IF_STA, nullptr, logger);
  if ((!ESP_NOW.begin()) || (!tx->add_self())) {
    logger->printf("Failed to initialize Tx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
  logger->printf("OpenRC4CL %s Tx %s channel:%d MAC Address:%s\n", 
                  OpenRC4CL_VERSION, devName.c_str(), wifiChan, WiFi.macAddress().c_str());
  logger->printf("Waiting to connect to Rx %s\n", macPeer.c_str()); 
  tx->wait4ThrHold();
}

const unsigned long msPol = 20;
unsigned long next = millis() + msPol;
void loop() {
  if (millis()-next > msPol) {  // function delay() will block onReceive
    tx->update();
    cmd->update();
    next += msPol;
  }
}
