// tmp project for experimenting with new modules

#include <MacAddress.h>
#include <WiFi.h>
MacAddress mac({0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}); 
char* mac_str = "98:A3:16:61:1D:5C";
void str2macAdr (char *str, MacAddress &mac) { // hex str to mac address
  sscanf(str,"%2x:%2x:%2x:%2x:%2x:%2x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); WiFi.setChannel(6);
  printf("MAC Address:%s\n", WiFi.macAddress().c_str());
}
void loop() {
}


// #include <nvs_flash.h>
// #include <Preferences.h>
// #include "OpenRC4CL_util.h"
// // #include <BLESerial.h>  // use Serial Bluetooth Terminal App (Google play)  #### ADD this comment to Rx, Tx and Timer code

// // NOTE Logger has been changed!!!! <-------------- !!!
// // class Logger {  // log msgs to both Serial and SerialBLE
// // public:
// //   // Logger(BLESerial<>* ble, int max_buf = 80) { SerialBLE = ble; _max_buf = max_buf; _buf = new char[_max_buf]; }
// //   Logger(int max_buf = 120) { _max_buf = max_buf; _buf = new char[_max_buf]; }
// //   void printf(const char* format, ...) {
// //     va_list args; va_start(args, format);
// //     vsnprintf(_buf, _max_buf, format, args);
// //     va_end(args);
// //     Serial.print(_buf); SerialBLE.print(_buf); SerialBLE.flush();
// //   }
// // private:
// //   char *_buf = 0; int _max_buf;
// // };

// // Non_Volatile_Storage
// struct NVS_elm { char* name; int& param; }; 
// #define NVS_ELM(p) {PSTR(#p), p}
// class NVS {  
// public:
//   NVS(const int nr_ps, NVS_elm* nvs_tab, Logger *logger) { 
//     tab = nvs_tab; 
//     nr_params = nr_ps;
//     log = logger;
//     pref.begin(nmspace, true);  // readonly, initialse from eprom or use default from table
//     for (int i = 0; i < nr_params; i++) tab[i].param = pref.getInt(tab[i].name, tab[i].param);
//     pref.end();
//   }
//   int read(char* name) {
//     NVS_elm* elm = getElm(name);
//     if (elm != 0) return elm->param; else 0; 
//   }
//   void write(char* name, int value) { 
//     NVS_elm* elm = getElm(name);
//     if (elm == 0) return;
//     pref.begin(nmspace, false);  // read/write
//     pref.putInt(elm->name, value);
//     elm->param = value;
//     pref.end();
//   }
// private:
//   NVS_elm* getElm(char * name) { 
//     for (int i = 0; i < nr_params; i++) if (! strcmp_P(name, tab[i].name)) return &tab[i];
//     log->printf("Unknown NVS param name:%d", name);
//     return 0;
//   }
//   char* nmspace = PSTR("OpenRC4CL");
//   Preferences pref;
//   NVS_elm* tab;
//   int nr_params;
//   Logger *log;
// };
// static void nvs_erase() {  // erase the NVS partition
//   nvs_flash_erase();  
//   nvs_flash_init();
//   Serial.print(F("NVS erased!\n"));
// }

// // - todo only edit if before disable Throttle Hold
// class CMD { // command interpreter
//   public:
//     CMD(const int nr_ps, NVS_elm* nvs_tab, Logger *logger) { 
//       tab = nvs_tab; 
//       nr_params = nr_ps;
//       log = logger;
//       nvs = new NVS(nr_params, nvs_tab, log); 
//       Serial.printf("passwd=%d\n", nvs->read(PSTR("passwd"))); // Note log passwd to console only!
//       log->printf("password\n");
//     }
//     void exec(char *cmdline) {
//       char *cmd = strsep(&cmdline, " ");
//       if (not chk_passwd) {
//         chk_passwd = nvs->read(PSTR("passwd")) == atoi(cmd); 
//         if (not chk_passwd) log->printf("password\n");
//       } else if (strcmp_P(cmd, PSTR("set")) == 0) {
//         char* param = strsep(&cmdline, " ");
//         int val = atoi(cmdline);
//         nvs->write(param, val);
//       } else if (strcmp_P(cmd, PSTR("get")) == 0) {
//         log->printf("%s=%d\n", cmdline, nvs->read(cmdline));
//       } else if (strcmp_P(cmd, PSTR("list")) == 0) {
//         const int maxp = 10;
//         for (int i = 0; i < nr_params; i++) {
//           if (strcmp_P(tab[i].name, PSTR("passwd"))) log->printf("%s=%d ", tab[i].name, tab[i].param);
//           if (((i > 0) && ((i % maxp) == 0)) || (i == nr_params-1)) log->printf("\n");
//         }
//       } else if (strcmp_P(cmd, PSTR("help")) == 0) {
//         log->printf("set param value\nget param\nhelp\nversion\n");
//       } else if (strcmp_P(cmd, PSTR("version")) == 0) {
//         log->printf("%s\n", "0.0.1"); // #############  todo OpenRC4CL_VERSION);
//       } else {
//         log->printf("Unknown command:%s\n", cmd);
//       }
//     }
//     void update() {
//       int data;
//       bool read = false;
//       if (SerialBLE.available()) { data = SerialBLE.read(); read = true; } 
//       else if (Serial.available()) { data = Serial.read(); read = true; }
//       if (read) {
//         if (data == '\r') {
//             buffer[length] = '\0';     // properly terminate the string
//             if (length) exec(buffer);  // give to interpreter
//             length = 0;                // reset for next command
//         } else if ((data != '\n') && (length < BUF_LENGTH - 1)) {  // discard \n
//             buffer[length++] = data;   // buffer the incoming byte
//         }
//       }
//     }
//   private:
//     static const int BUF_LENGTH = 32;
//     char buffer[BUF_LENGTH];
//     int length = 0;  // length of line received so far
//     bool chk_passwd = false;
//     NVS_elm* tab;
//     int nr_params;
//     NVS* nvs;
//     Logger *log;
// };
// int p1 = 11;
// int p2 = 22;
// int passwd = 123;
// const int nr_params = 3;
// NVS_elm nvs_tab[nr_params] = { NVS_ELM(passwd), NVS_ELM(p1), NVS_ELM(p2) };
// Logger *logger;
// CMD* cmd;
// void setup() {
//     Serial.begin(115200);
//     SerialBLE.begin("JL_TEST"); 
//     logger = new Logger;
//     cmd = new CMD(nr_params, nvs_tab, logger);
// }
// void loop() {
//   cmd->update();
// }

// // - todo only edit if before disable Throttle Hold
// // disable list passwd in nvs echo passwd to Serial only at startup
// class CMD { // command interpreter
//   public:
//     CMD(NVS* nv) { nvs = nv; Serial.print(F("password\n")); }   // must have params: const int nr_ps, NVS_elm* nvs_tab, and store nvs_tab ptr and create nvs class var
//     void exec(char *cmdline) {
//       char *cmd = strsep(&cmdline, " ");
//       if (not chk_passwd) {
//         chk_passwd = nvs->getElm(PSTR("passwd")).param == atoi(cmd); 
//         if (not chk_passwd) Serial.print(F("password\n"));
//       } else if (strcmp_P(cmd, PSTR("set")) == 0) {
//         char* param = strsep(&cmdline, " ");
//         int val = atoi(cmdline);
//         nvs->write(param, val);
//       } else if (strcmp_P(cmd, PSTR("get")) == 0) {
//         nvs->read(cmdline);
//       } else if (strcmp_P(cmd, PSTR("list")) == 0) {
//         nvs->list();
//       } else if (strcmp_P(cmd, PSTR("help")) == 0) {
//         Serial.print(F("set param value\nget param\nhelp\nversion\n"));
//       } else if (strcmp_P(cmd, PSTR("version")) == 0) {
//         Serial.printf("%s\n", OpenRC4CL_VERSION);
//       } else {
//         Serial.print(F("Error: Unknown command: "));
//         Serial.println(cmd);
//       }
//     }
//     void update() {
//       if (Serial.available()) {
//         int data = Serial.read();
//         if (data == '\r') {
//             buffer[length] = '\0';     // properly terminate the string
//             if (length) exec(buffer);  // give to interpreter
//             length = 0;                // reset for next command
//         } else if ((data != '\n') && (length < BUF_LENGTH - 1)) {  // discard \n
//             buffer[length++] = data;   // buffer the incoming byte
//         }
//       }
//     }
//   private:
//     static const int BUF_LENGTH = 32;
//     char buffer[BUF_LENGTH];
//     int length = 0;  // length of line received so far
//     bool chk_passwd = false;
//     NVS* nvs;
// };
// int p1 = 11;
// int p2 = 22;
// int passwd = 123;
// const int nr_params = 3;
// NVS_elm nvs_tab[nr_params] = { NVS_ELM(passwd), NVS_ELM(p1), NVS_ELM(p2) };
// Logger *logger;
// NVS* nvs;
// CMD* cmd;
// void setup() {
//     Serial.begin(115200);
//     logger = new Logger("JL_TEST", 120);
//     nvs = new NVS(nr_params, nvs_tab, logger);
//     nvs->list();
//     cmd = new CMD(nvs);
// }
// void loop() {
//   cmd->update();
// }

// See https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html
// #include <Arduino.h>
// #include <Preferences.h>
// Preferences preferences;
// void test_non_volatile_mem() {  // note is also non volatile under new code flash
//   Serial.begin(115200);
//   Serial.println();
//   preferences.begin("OpenRC4CL", false);
//   //preferences.clear();
//   //preferences.remove("counter");
//   unsigned int counter = preferences.getUInt("counter", 0);  // Get the counter value, if the key does not exist, return a default value of 0
//   counter++;
//   Serial.printf("Current counter value: %u\n", counter);
//   preferences.putUInt("counter", counter);
//   preferences.end();   // update eeprom
//   Serial.println("Restarting in 10 seconds...");
//   delay(10000);
//   ESP.restart();
// }
// void setup() { test_non_volatile_mem();}
// void loop() 

// To completely erase and reformat the NVS memory used by Preferences, create and run a sketch
// You should download a new sketch to your board immediately after running the above 
// or else it will reformat the NVS partition every time it is powered up or restarted!
// #include <nvs_flash.h>
// void setup() {
//     nvs_flash_erase();      // erase the NVS partition and...
//     nvs_flash_init();       // initialize the NVS partition.
//     while (true);
// }
// void loop() {}

// from IDE examples | BLESerial
// #include <BLESerial.h>
// String device_name = "ESP32-BLE-Slave";
// BLESerial SerialBLE;
// void setup() {
//     Serial.begin(9600);
//     SerialBLE.begin(device_name);
// }
// void loop() {
//     if (Serial.available()) {
//         SerialBLE.write(Serial.read());
//         SerialBLE.flush();
//     }
//     if (SerialBLE.available()) {
//         Serial.write(SerialBLE.read());
//     }
// }

// command interpreter
// https://gist.github.com/edgar-bonet/607b387260388be77e96
// https://arduino.stackexchange.com/questions/40004/constructing-my-own-serial-command-syntax-to-control-arduino
// cmds:
// - set name value
// - get name 
// - help
// - todo list (all params), version
// - todo add password before using and only edit if Throttle Hold
// const int BUF_LENGTH = 32;
// static void exec(char *cmdline) {
//     char *cmd = strsep(&cmdline, " ");
//     if (strcmp_P(cmd, PSTR("set")) == 0) {
//         char* param = strsep(&cmdline, " ");
//         int val = atoi(cmdline);
//         Serial.printf("set p=%s v=%d\n", param, val);
//     } else if (strcmp_P(cmd, PSTR("get")) == 0) {
//         Serial.printf("get p=%s\n", cmdline);
//     } else if (strcmp_P(cmd, PSTR("help")) == 0) {
//         Serial.print(F("set param value\nget param\nhelp\n"));
//     } else {
//         Serial.print(F("Error: Unknown command: "));
//         Serial.println(cmd);
//     }
// }
// void setup() {
//     Serial.begin(9600);
// }
// void loop() {
//   while (Serial.available()) {
//       static char buffer[BUF_LENGTH];
//       static int length = 0;  // length of line received so far
//       int data = Serial.read();
//       if (data == '\r') {
//           buffer[length] = '\0';     // properly terminate the string
//           if (length) exec(buffer);  // give to interpreter
//           length = 0;                // reset for next command
//       } else if ((data != '\n') && (length < BUF_LENGTH - 1)) {  // discard \n
//           buffer[length++] = data;   // buffer the incoming byte
//       }
//   }
// }

// #include <nvs_flash.h>
// #include <Preferences.h>
// #include "OpenRC4CL_util.h"
// // Non_Volatile_Storage
// struct NVS_elm { char* name; int& param; }; 
// #define NVS_ELM(p) {PSTR(#p), p}
// class NVS {  
// public:
//   NVS(const int nr_ps, NVS_elm* nvs_tab, Logger* logger) { 
//     tab = nvs_tab; 
//     nr_params = nr_ps;
//     log = logger;
//     pref.begin(nmspace, true);  // readonly, initialse from eprom or use default from table
//     for (int i = 0; i < nr_params; i++) tab[i].param = pref.getInt(tab[i].name, tab[i].param);
//     pref.end();
//   }
//   int read(char* name) { return getElm(name).param; }
//   void write(char* name, int value) { 
//     NVS_elm& elm = getElm(name);
//     pref.begin(nmspace, false);  // read/write
//     pref.putInt(elm.name, value);
//     elm.param = value;
//     pref.end();
//   }
//   void list() {
//     const int maxp = 10;
//     for (int i = 0; i < nr_params; i++) {
//       log->printf("%s=%d ", tab[i].name, tab[i].param);
//        if (((i > 0) and ((i % maxp) == 0)) or (i == nr_params-1)) log->printf("\n");
//     }
//   }
// private:
//   NVS_elm& getElm(char * name) {
//     for (int i = 0; i < nr_params; i++) if (not strcmp_P(name, tab[i].name)) return tab[i];
//     log->printf("Unknown NVS param name:%d", name);  // also for BLE
//   }
//   Logger* log;
//   char* nmspace = PSTR("OpenRC4CL");
//   Preferences pref;
//   NVS_elm* tab;
//   int nr_params;
// };
// static void nvs_erase() {  // erase the NVS partition
//   nvs_flash_erase();  
//   nvs_flash_init();
//   Serial.print(F("NVS erased!\n"));
// }
// const int nr_params = 2;
// int p1 = 11;
// int p2 = 22;
// NVS_elm nvs_tab[nr_params] = { {"p1",p1}, {"p2",p2} };
// Logger *logger;
// NVS* nvs;
// void setup() {
//     Serial.begin(115200);
//     Serial.printf("nvs test\n");
//     // nvs_erase();
//     logger = new Logger("JL_TEST", 120);
//     nvs = new NVS(nr_params, nvs_tab, logger);
//     Serial.printf("p1=%d\n", nvs->read("p1"));
//     Serial.printf("p2=%d\n", nvs->read("p2"));
//     nvs->list();
//     nvs->write("p1", 111110);
//     nvs->write("p2", 222220);
//     nvs->list();
//     Serial.printf("p1=%d\n", p1);
//     Serial.printf("p2=%d\n", p2);
// }
// void loop() {}

