# OpenRC4CL

Control Line (CL), also called U-Control, is a way of controlling model airplanes, see https://en.wikipedia.org/wiki/Control_line.

Low-cost DIY project for expanding CL handles with Radio Control (RC) functions like throttle control, servos, timers, telemetry and failsafe.
Uses the ESP-NOW protocol transmitted via the 2.4 GHz Wi-Fi band.

The software is released as opensource under the MIT licence.

Note this project is Work In Progress and is develloped for ESP32 C6, does not need external antenna.

Feedback is welcome and I am open for ideas and/or colaboration.

## Functional overview OpenRC4CL.

Expanding CL handles with RC functions like throttle control, servos, timers, telemetry and failsafe.
Usage cases:

- Combat/trainer CL handle with throttle hold switch and small potmeter for setting throttle value between flights.
- Carrier/scale CL handle with continuous throttle 'trigger' control, throttle hold switch and 
  3 (two 3 states and one continuous) servo functions like for dropping hook, etc.
- etc.

Default servo pulse width [1000..2000] us @ 50 Hz, in software configurable.

Tx channels:

- 0: throttle potmeter
- 1: 3 position switch
- 2: 3 position switch
- 3: potmeter

Rx servo channels:

- 0: throttle
- 1: servo 1
- 2: servo 2
- 3: servo 3

Special functions:

- check pairing Tx and Rx based on MAC address
- checksum on transmitted data
- failsafe if > 0.5 seconds connection lost OR TxBatt low:
  initiate throttle warning (like CLTimer) followed by stop engine and hold servos in last position
- telemetry with VBatt, VBatt low and RSI 
- throttle hold must be activated before Tx starts to transmit
- logging status Tx/Rx by bluetooth to smartphone
- exponential throttle curve (in software configurable) [planned]
- select wifi channel using 2 position dipswitch [planned]
- softstart throttle option (in software configurable) [planned]

Tx ESP32C6 inputs:

- throttle (analog input potmeter)
- chan1 (2 digital inputs for 3 position switch)
- chan2 (2 digital inputs for 3 position switch)
- chan3 (analog input potmeter)
- throttle hold (digital input 2 position switch)
- Tx lipo voltage monitor (1 cell lipo) (analog input)
- max flight time (analog input potmeter)
- select wifi chan (2 digital inputs for 2 position dipswitch)
- deadband throttle correction (analog input potmeter)

Tx ESP32C6 outputs:

- status led (digital output)
- beeper (digital output)

Rx ESP32C6 inputs:

- low VBatt warning voltage setting (input potmeter) [3000..8*4350] mV, max 8S LiHV
- 8S lipo voltage monitor (input)

Rx ESP32C6 outputs:

- throttle (analoge servo output)
- servo 1  (analoge servo output)
- servo 2  (analoge servo output)
- servo 3  (analoge servo output)
- status led (digital output)
- beeper (digital output)

Status codes for led and beeper (blink pulse in ms):

- Ok = 0 (off)
- WaitThrHold = 1000
- WaitConnection = 2000
- VBattLow and TxBattLow = 2000
- EndFlight = 4000
- Failsafe = 500
- Error = 200

## CLTimer 

CLTimer is 'spinoff' project from components used for Tx and Rx projects in the OpenRC4CL repro.

Usage case: typical classic control line timer without remote RC functionality.

Inputs:

- parameters (analog input with potmeters):
	- delay start [0..60] secs
	- throttle level [0..100]%
	- runtime [0..8*60] secs
	- min VBatt [3000..8*4350] mV, max 8S LiHV
- VBatt (max 8S) (analog input)
- start/stop button (digital input)

Outputs:

- servo signal for ESC (analog output)
- status led (digital output)
- beeper (digital output)

Note if VBatt is not connected, only timer function avialable.

Phases (N and W are parameters configurable in software):  [WIP]

- connect battery and boot
- read parameters, log every 0.5 second parameters to serial monitor and BT 
- press button for 2 seconds to intiate 'ready_to_start', confirm with beep
- during ready_to_start
  - beep every 10 secs, last 10 sec every sec
  - abort ready_to_start if start button is pressed for 2nd time
    go to phase read parameters
- start engine and throttle timer, log every 0.5 second status to serial monitor and BT
- abort running if start button is (short) pressed for 2nd time
  go to phase read parameters
- W seconds before throttle timer expires OR if VBatt below min VBatt:
  issue for N (N < W) seconds throttle warnings (0.5 s half, 0.5 s normal throttle level) 
  (last W-N seconds full throttle level)
- stop engine 

Logging status by bluetooth or usb to smartphone or laptop.

