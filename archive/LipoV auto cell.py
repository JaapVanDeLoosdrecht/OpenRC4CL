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

