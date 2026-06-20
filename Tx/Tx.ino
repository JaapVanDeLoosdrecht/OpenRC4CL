/* 
TX OpenRC4CL 20 June 2026

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

#include <MacAddress.h>
#include <WiFi.h>
#include <limits.h>
#include "OpenRC4CL_util.h"  
#include "secret.h" // NOTE: file secret.h contains mac adrs used by developper and is NOT part of distro, comment out this line!!!
#ifndef OpenRC4CL_SECRET
BindElm rxTab[] = { {"Rx-OpenRC4CL", "00:00:00:00:00:00"},   // modify with your devName and mac address of Rx or use serial CMDs
//                  {"Rx-name2", "00:00:00:00:00:00"},       // you can add more Rxs
                  };
#endif

// system
const int deadBandMax = ThrHoldDelta-10;    // deadband delta for throttle, no overleap with TH
const int deadBandMin = -deadBandMax;    
const int lipoDivR1 = 10000;                // R1 lipo voltage divider, max 5V usb, div=2
const int lipoDivR2 = 10000;                // R2 lipo voltage divider
const int maxBinds = 10;                    // max nr of Rxs
// user settings (NVS params)
String devName = "Tx-OpenRC4CL";            // modify with your Tx devName and or use serial CMD
String devPeer = rxTab[0].devName; 
String macPeer(rxTab[0].mac); 
String passwd = "123";
String date = buildDate();
int wifiChan = 6;                           // [1..13]
int deadBand = 0;                           // deadband delta for TH low
int txBattLow = 3500;                       // min voltage for Tx lipo
int nrWarns = 10;                           // number beeps if TxBattLow or timer is expired number 
int logging = 1;                            // 1: log status, 0: no log
int nrChans = 2;                            // nr of channels used [2..5], to avoid floating analog inputs warnings
int ch1Switch = 1;                          // pins used for ch1 switch: 0 = pinLeftCh1, 1 = pinRightCh1, 2 = both pins
int vBatt = 1;                              // 0 = no vBatt connected, 1 = vBatt connected, to avoid floating analog inputs warnings

NVS* buildNVS(Logger *logger) {
  const int max_nvs_params = 32; 
  NVS* nvs = new NVS(NVSNameSpace, max_nvs_params, logger); 
  nvs->add(NVS_STR(devName, true));
  nvs->add(NVS_STR(devPeer, true));
  nvs->add(NVS_STR(macPeer, true));
  nvs->add(NVS_STR(passwd, true));
  nvs->add(NVS_STR(date, false));
  nvs->add(NVS_INT(wifiChan, true, 1, 13));
  nvs->add(NVS_INT(deadBand, true, deadBandMin, deadBandMax));
  nvs->add(NVS_INT(txBattLow, true, 3000, 4350));
  nvs->add(NVS_INT(nrWarns, true, 1, 20));
  nvs->add(NVS_INT(logging, false, 0, 1));
  nvs->add(NVS_INT(nrChans, true, 2, 5));
  nvs->add(NVS_INT(ch1Switch, true, 0, 2));
  nvs->add(NVS_INT(vBatt, true, 0, 1));
  return nvs;
}

class Tx : public RcPeer {
public:
  Tx(MacAddress mac_rx, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk, Logger *log):
    RcPeer(mac_rx, channel, iface, lmk) { logger = log; }
  void wait4ThrHold() {
    status.value = Status::WaitThrHold;
    logStatus(); 
    while (hold.readPos() != Thr_Hold) { delay(100); statusUpdate(); }
    status.value = Status::WaitTxRx;
    logStatus();
  }
  void update() { sendTx(); statusUpdate(); }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    connected = true;
    lastTelm = millis();
    struct Telemetry tel = *(struct Telemetry *)data;
    if (CheckSum(tel) != tel.checkSum) { 
      status.value = Status::Error; errors++;
      logStatus();
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
        status.value = (vBattLow) ? Status::RxBattLow : Status::EndFlight;
      }
    }
    if (logging) 
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
      errors++;
    }  
    unsigned long now = millis();  // check still connected
    if ((now - lastTelm) > 5000) {  // note check can be interrupted by callback, abs doesn't work on unsigned
      connected = false; errors = 0;
      if (status.value != Status::WaitTxRx) {
        status.value = Status::WaitTxRx;
        logStatus();  // only log once
      }
    }
    statusUpdate();
  }
  void logStatus() { if (logging) logger->printf("[Tx:%s bat:%d thr:%d err:%d]\n", status.str(), txBatt.read(), readThrottle(), errors); }
  void statusUpdate() {  // update status for led and beep
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
  static const Switch::Pos Thr_Hold = Switch::Middle;
  static const int vbatThr = 500;                          // if vbat not connected on Rx -> vbat < vbatThr
  Potmeter throttle{pinThrottle};
  Switch chan1{((ch1Switch = 0) || (ch1Switch = 2)) ? pinLeftCh1 : PIN_NOT_USED, 
               ((ch1Switch = 1) || (ch1Switch = 2)) ? pinRightCh1 : PIN_NOT_USED};
  Potmeter chan2{(nrChans > 2) ? pinCh2 : PIN_NOT_USED}; 
  Potmeter chan3{(nrChans > 3) ? pinCh3 : PIN_NOT_USED}; 
  Potmeter chan4{(nrChans > 4) ? pinCh4 : PIN_NOT_USED};
  Status status{Status::WaitTxRx};
  Led led{pinLed, status.pulse(), true};  
  Switch hold{pinThrottleHold};
  Beep beep{pinBeep, status.pulse()};
  VoltageDiv txBatt{(vBatt) ? pinVBatt : PIN_NOT_USED, lipoDivR1, lipoDivR2};
  bool connected = false, endFlight = false, beepEndFlight = false;
  int id = 0, lastId = 0, errors = 0;      // #errors = #send + #checksum
  unsigned long lastTelm = 0;
  Logger *logger = 0;
};

Tx *tx = 0;    // initialisation must be in setup due to changing pinModes to OUTPUT
CMD* cmd = 0; 
BindTab* bindTab = 0;

void setup() {
  // nvs_erase();
  String dn = GetStringNVS(NVSNameSpace, "devName");
  if (dn != String("")) devName = dn;
  Serial.begin(115200);
  SerialBLE.begin(devName); 
  bindTab = new BindTab(NVSNameSpace, maxBinds, sizeof(rxTab)/sizeof(BindElm), rxTab);
  Logger *logger = new Logger;    // Serial and BLE logger
  NVS* nvs = buildNVS(logger);
  cmd = new CMD(nvs, logger, bindTab);
  WiFi.mode(WIFI_STA); WiFi.setChannel(wifiChan);
  while (!WiFi.STA.started()) delay(100);
  logger->printf("OpenRC4CL %s Tx=%s channel=%d Rx=%s\nMAC-Tx=%s MAC-Rx=%s\n", 
                  OpenRC4CL_VERSION, devName.c_str(), wifiChan, devPeer.c_str(), WiFi.macAddress().c_str(), macPeer.c_str());
  cmd->list(); cmd->bindtab();
  tx = new Tx(MacAddress(macPeer), wifiChan, WIFI_IF_STA, nullptr, logger);
  if ((!ESP_NOW.begin()) || (!tx->add_self())) {
    logger->printf("Failed to initialize Tx, rebooting in 2 seconds...\n");
    delay(2000); ESP.restart();
  }
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
