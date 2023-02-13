// Lin Control Truma Combi 4/6 - Protocol: 4.1
// Gas Mode only - Mixed/Electric Mode not coded but you can find the data

/* LIN PACKET:
   It consist of:
    ___________ __________ _______ ____________ _________
   |           |          |       |            |         |
   |Synch Break|Synch Byte|ID byte| Data Bytes |Checksum |
   |___________|__________|_______|____________|_________|

   Every byte have start bit and stop bit and it is send LSB first.
   Synch Break - 13 bits of dominant state ("0"), followed by 1 bit recesive state ("1")
   Synch Byte - Byte for Bound rate syncronization, always 0x55
   ID Byte - consist of parity, length and address; parity is determined by LIN standard and depends from address and message length
   Data Bytes - user defined; depend on devices on LIN bus
   Checksum - inverted 256 checksum; data bytes are sumed up and then inverted
*/
// https://github.com/mestrode/Lin-Interface-Library/blob/main/src/Lin_Interface.cpp

//#define mitWIFI

#include <Arduino.h>
#include <Streaming.h>
#include <myMacros.h>
#include <myUtils.h>

#define DEBUG__EIN  //"Schalter" zum aktivieren von DEBUG-Ausgaben
#include <espSoftwareSerial.h>   //INFO: im cpp falsche include  ändern auf #include "espSoftwareSerial.h"
#include <myDebug.h>
#include <truma.h>

#ifdef mitWIFI
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#endif

// LIN serial Interface
const uint16_t serSpeed      = 9600;  // speed LIN of IBS-Sensor (do not change)
const uint16_t breakDuration = 13;    // number of bits break signal
const uint16_t linRX         = 2;     // rot    13;   // RX Pin LIN serial
const uint16_t linTX         = 3;     // orange 12;   // TX Pin LIN serial
const uint16_t linSLP        = 4;     // gelb

SoftwareSerial linSerial(linTX, linRX);  // RX, TX
//#define linSerial Serial

#ifdef mitWIFI
// WIFI Setup
const char* ssid      = WLAN_SSID;
const char* password  = WLAN_PASS;
const char* ssid2     = "";
const char* password2 = "";

// MQTT Setup
const char* mqtt_server = MQTT_HOST;
const char* userMqqt    = MQTT_USER;
const char* pwdMqqt     = MQTT_PASS;

WiFiClient espClient;
PubSubClient client(espClient);
#endif

// Configuration
boolean outputSerial = true;   // true if json output to serial, false if only display
boolean debug        = true;   // Debug output UART
boolean SleepMode    = false;  // Set deep sleep (e.g. 60sec)

// Global Variables
uint8_t LinMessage[9]       = {0};
uint8_t LinMessageA[12]     = {0};  // INFO: war 200   LIN answer
boolean linSerialOn         = 0;
unsigned long previous_time = 0;
unsigned long delaytime     = 20000;  // 20 seconds delay
float tRoom;                          // Room temperature [degC]
float tWater;                         // Water temperature [degC]
const char* diagMessage;              // Diagnose Message (warning or error e.g. W401H, W255H, E517H)

// char msg[50]; // msg of mqtt suscriptions (see callback function)

// Control variables
int16_t temp      = 0;      // Temperature [degC]
boolean heat      = false;  // Heating on/off (on=1,off=0)
boolean boil      = false;  // Boiler on/off (on=1,off=0)
uint16_t boilMode = 0;      // Boiler mode (off=0,eco=1,hot=2,boost=3)
uint16_t fan      = 0;      // Ventilator (off=0,Stufe1=1 - Stufe10=10,Eco=11,High=13)
boolean rst       = false;  // Error reset (if=1)

// Temperature code [degC] 5-30 (0-4)=0xAA=off
//                                off                5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21     22   23    24    25    26    27    28    29    30
uint8_t tempHex[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xDC, 0xE6, 0xF0, 0xFA, 0x04, 0x0E, 0x18, 0x22, 0x2C, 0x36, 0x40, 0x4A, 0x54, 0x5E, 0x68, 0x72, 0x7C, 0x86, 0x90, 0x9A, 0xA4, 0xAE, 0xB8, 0xC2, 0xCC, 0xD6};

//------------------------------------------
void setup();
void loop();
bool readFrame(uint8_t mID);
void serialBreak();
void sendMessage(uint8_t mID, int nByte);
uint8_t LINChecksum(uint8_t* message, uint8_t length, uint16_t sum);
uint8_t getChecksum(uint8_t* message, uint8_t dataLen, uint8_t ProtectedID);
uint8_t addIDParity(uint8_t linID);
void trum_message(int16_t temp, boolean heat, boolean boil, uint16_t boilMode, uint16_t fan);
void trum_diagn_request(int ixx);
void trum_reset();
void setup_wifi();
void connect_mqtt();
void callback(char* topic, uint8_t* payload, unsigned int length);
void serialCommand();
//-----------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(BAUD);
  while (!Serial && (millis() < 3000))
    ;
  Serial << "\n\n" << ProjektName << " - " << VERSION << "  (" << BUILDDATE << "  " __TIME__ << ")" << endl;

  linSerial.begin(serSpeed);  // Original
  linSerialOn = 1;

#ifdef mitWIFI
  // WIFI und MQTT Server
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  connect_mqtt();  // connect MQTT

  // Receive values from mqtt broker
  client.subscribe("truma_heating");  // Heating on/off [ON/OFF]
  client.subscribe("truma_boiler");   // Boiler on/off [0=off/1=eco/2=hot]
  client.subscribe("truma_temp");     // Heating temp [degC]
  client.subscribe("truma_fan");      // Fan level {-]
  client.subscribe("truma_reset");    // Reset of fatal error [boolean]
  // client.subscribe("truma_boilMode");   // Boiler mode [off/eco/hot/boost]
#endif
  // LIN-Bus Wakeup
  serialBreak();
  delay(1200);  // Delay original 1,6s to wakeup heater
}

//-----------------------------------------------------------------------------------------------------------------
// Loop-Funktion
void loop() {
#ifdef mitWIFI
  // Reconnect WiFi if connection is lost
  unsigned long current_time = millis();  // number of milliseconds since the upload
  if ((WiFi.status() != WL_CONNECTED) && ((current_time - previous_time) >= delaytime)) {
    WiFi.disconnect();
    setup_wifi();
    previous_time = current_time;
  }

  // Get data from IO Broker serveral requests to make sure data is received
  for (int i = 0; i < 20; i++) {
    client.loop();  // Hole Daten vom IO Broker und halte Verbindung aufrecht
    delay(100);
  }
#endif

  serialCommand();

  if (rst == true) {
    trum_reset();
    rst = false;
  }

  for (int ixx = 0; ixx < 3; ixx++) {  // Loop for diagnose frames

    // Output to serial
    if (debug) {
      Serial.print("Block ");
      Serial.print(ixx + 1);
      Serial.println(" ------------------------------------------------");
      // Serial.print("Time: ");
      // Serial.print(millis());
      // Serial.println(" ms");
    }

    // Set truma mode (e.g. mode,temp,...)
    trum_message(temp, heat, boil, boilMode, fan);

    // Read data
    // Read ID=21
    if (readFrame(0x21)) {                                                                // Status, ID=21, PID=61, Room temp, water temp,...
      tRoom  = float(LinMessageA[0] + ((LinMessageA[1] & 0x0F) << 8)) / 10 - 273;         // Room temperature [degC]
      tWater = float((LinMessageA[2] << 4) + ((LinMessageA[1] & 0xF0) >> 4)) / 10 - 273;  // Water temperature [degC]
    }
    // Read ID=22
    if (readFrame(0x22)) {  // Status, ID=22, PID=E2, Mode(on,off), heating,...
      Serial_printf("Spannung = %0.2f\n", float(LinMessageA[0] / 10.0));
    };
    // Start Diagnosis
    trum_diagn_request(ixx);
  }

  // Output to serial
  if (outputSerial) {
    Serial.print("Raumtemperatur=");
    Serial.print(tRoom, 1);
    Serial.print("  Wassertemperatur=");
    Serial.println(tWater, 1);
    if (strlen(diagMessage) > 1) {
      Serial.print("*** Error message: ");
      Serial.println(diagMessage);
    }
    if (SleepMode) {
      Serial.println("Deep sleep.");
    }
  }

#ifdef mitWIFI
  // Publish to IO Broker
  connect_mqtt();                                                    // reconnect MQTT (not publishing without it
  client.publish("Truma_TempRoom", String(tRoom).c_str());           // Room temp [degC]
  client.publish("Truma_TempWater", String(tWater).c_str());         // Water temp [degC] (not valid yet)
  client.publish("truma_diagMessage", String(diagMessage).c_str());  // Diagnose Message (warning or error e.g. W401H, W255H, E517H)

  client.loop();  // Hole Daten vom IO Broker und halte Verbindung aufrecht

  delay(500);

  if (SleepMode) {
    ESP.deepSleep(60e6);  // put the ESP8266 in deep sleep (µs)
  }
#endif
}  // end of function loop

//-----------------------------------------------------------------------------------------------------------------
// Read answer from Lin bus
bool readFrame(uint8_t mID) {
  memset(LinMessageA, 0, sizeof(LinMessageA));  // Clear LinMessageA (set values to zero)
  int ix        = 0;
  uint8_t linID = (mID & 0x3F) | addIDParity(mID);

  delay(40);               // TODO:  ms oder µs?   Wait 40µs
  serialBreak();           // Funktion die den Sync Break erzeugt
  linSerial.write(0x55);   // Sync (entspricht: 01010101)
  linSerial.write(linID);  // ID
  linSerial.flush();
  delay(400);
  // delay(800);
  if (linSerial.available()) {  // read serial
    // linSerial.read(); // Erstes Byte ignorieren da Identifier Frame (z.B A8 für Frame 0x28) von TX zurückgeschrieben wird
    while (linSerial.available()) {
      LinMessageA[ix] = linSerial.read();
      ix++;
      if (ix > 9) {
        break;
      }
    }

    if (debug) {
      Serial.print("ID: ");
      Serial.print(mID, HEX);
      Serial.print(" --> ");
      Serial.print(linID, HEX);
      Serial.print(": ");
      for (int i = 0; i < 8; i++) {  // war 9
        Serial_printf("%02x:", LinMessageA[i]);
      }
      uint8_t cksum = LinMessageA[8];
      Serial.print(" - ");
      Serial_printf("%02x:", cksum);

      // uint8_t cksum2 = LINChecksum2(LinMessageA,8,true);
      // uint8_t cksum2 = LINChecksum(LinMessageA, 8, linID);
      uint8_t cksum2 = getChecksum(LinMessageA, 8, linID);
      if (cksum != cksum2) {
        Serial.print(" -error  ");
        DEBUG__PRINTF("chksum: %02x  %02x   ", cksum, cksum2);
      }
      Serial.println("  readFrame Ende");
    }
    return true;
  }
  DEBUG__PRINTF("Keine Anwort auf ID=%02x\n", mID);
  return false;
}

// Generate Break signal LOW on LIN bus  (13bit low, 1bit high)
void serialBreak() {
  if (linSerialOn == 1) linSerial.end();
  pinMode(linRX, OUTPUT);
  digitalWrite(linRX, LOW);                               // send break
  delayMicroseconds(1000000 / serSpeed * breakDuration);  // duration break time pro bit in milli seconds * number of bit for break
  digitalWrite(linRX, HIGH);
  delayMicroseconds(1000000 / serSpeed);  // wait 1 bit
  linSerial.begin(serSpeed);
  linSerialOn = 1;
}

// https://github.com/MarkusLange/Teensy_3.x_4.x_and_LC_LIN_Master/blob/master/src/lin_bus.cpp
// sum = 0 LIN 1.X CRC, sum = PID LIN 2.X CRC Enhanced
uint8_t LINChecksum(uint8_t* message, int length, uint16_t sum) {
  for (int i = 0; i < length; i++) {
    sum += message[i];
    if (sum >= 256) sum -= 255;
  }
  return (~sum);
}

/// @brief Checksum calculation for LIN Frame
/// @details
/// EnhancedChecksum considers ProtectedID
///     LIN 2.0 only for FrameID between 0x00..0x3B
///     LIN 2.0 uses for 0x3C and above ClassicChecksum for legacy (auto detected)
/// ClassicChecksum
///     LIN 1.x in general (use 'ProtectedID' = 0x00 to ensure that)
/// see LIN Specification 2.2A (2021-12-31) for details
///     https://microchipdeveloper.com/local--files/lin:specification/LIN-Spec_2.2_Rev_A.PDF
///     2.8.3 Example of Checksum Calculation
/// @param ProtectedID initial Byte, set to 0x00, when calc Checksum for classic LIN Frame
/// @param dataLen length of Frame (only Data Bytes)
/// @returns calculated checksum
uint8_t getChecksum(uint8_t* message, uint8_t dataLen, uint8_t ProtectedID) {
  uint16_t sum = ProtectedID;
  // test FrameID bits for classicChecksum
  if ((sum & 0x3F) >= 0x3C) {
    // LIN 1.x: legacy
    // LIN 2.0: don't include PID for ChkSum calculation on configuration and reserved frames
    sum = 0x00;
  }
  // sum up all bytes (including carryover to the high uint8_t)
  // ID allready considered
  while (dataLen-- > 0) sum += message[dataLen];
  // add high byte (carry over) to the low byte
  while (sum >> 8) sum = (sum & 0xFF) + (sum >> 8);
  // inverting result
  return (~sum);
}

uint8_t addIDParity(uint8_t linID) {
  uint8_t p0 = bitRead(linID, 0) ^ bitRead(linID, 1) ^ bitRead(linID, 2) ^ bitRead(linID, 4);
  uint8_t p1 = ~(bitRead(linID, 1) ^ bitRead(linID, 3) ^ bitRead(linID, 4) ^ bitRead(linID, 5));
  return ((p0 | (p1 << 1)) << 6);
}

//-----------------------------------------------------------------------------------------------------------------
void trum_message(int16_t temp, boolean heat, boolean boil, uint16_t boilMode, uint16_t fan) {
  DEBUG__PRINTF("truma_message temp=%i  heat=%i  boil=%i boilMode=%i  fan=%i\n", temp, heat, boil, boilMode, fan);
  // uint8_t i = 0;
  uint8_t h = 0x0A;  // Heating on/off
  uint8_t b = 0xA0;  // Boiler on/off

  const uint8_t PID = 0x20;

  LinMessage[0] = PID;  // Frame identifier (20-PID)

  // LinMessage [2]
  // Truma Mode (XA-Heating off, XB-Heating on,AX-Boiler-off,2X-Boiler on, Example: Heating on+Boiler on = 2B) - Wichtig: "X" steht als Platzhalter
  // Heating
  //                            off                   5     6     7     8     9     10    11    12    13    14    15    16    17    18    19    20    21     22   23    24    25    26    27    28    29    30
  // uint8_t tempHex[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xDC, 0xE6, 0xF0, 0xFA, 0x04, 0x0E, 0x18, 0x22, 0x2C, 0x36, 0x40, 0x4A, 0x54, 0x5E, 0x68, 0x72, 0x7C, 0x86, 0x90, 0x9A, 0xA4, 0xAE, 0xB8, 0xC2, 0xCC, 0xD6};

  if (heat) {
    h             = 0x0B;           // Heating on
    LinMessage[1] = tempHex[temp];  // Selected Temp (AA-Off,DC-5,E6-6,F0-7,... degC | Fahrenheit*10)
  } else {
    h             = 0x0A;  // Heating off
    LinMessage[1] = 0xAA;  // Temperature off (0xAA)
  }

  // Set boiler
  if (boil) {
    b = 0x20;  // Boiler on
  } else {
    b             = 0xA0;  // Boiler off
    LinMessage[3] = 0xAA;  // Boiler mode off
  }

  LinMessage[2] = b + h;  // Heater and Boiler mode (combined Byte)

  if (boil) {  // if boiler on
    // Boiler mode (AA-off,C3-eco, D0- Hot+Boost)
    switch (boilMode) {
      case 1:
        LinMessage[3] = 0xC3;  // Boiler eco
        break;
      case 2:
        LinMessage[3] = 0xD0;  // Boiler hot
        break;
      case 3:                  // Boiler boost
        LinMessage[1] = 0xAA;  // Temp off (notwendig für Boost)
        LinMessage[2] = 0x2A;  // Heating off + Boiler on (notwendig für Boost)
        LinMessage[3] = 0xD0;  // Boiler boost
        break;
    }
  }

  LinMessage[4] = 0xFA;  // Energiemix (FA-Gas+Mix,00-Elektro)
  LinMessage[5] = 0x00;  // Mode (00-Gas,09-Mix/Elektro1,12-Mix/Elektro2)

  // Ventilator
  if (heat & (fan < 11)) {  // Avoid fan to be in level 1-10 or off when heating on
    DEBUG__PRINTLN("Lüfter auf ECO");
    LinMessage[6] = uint8_t(0xB1);  // Lüfter auf ECO wenn geheizt wird
  } else if (!heat & boil) {
    DEBUG__PRINTLN("Lüfter ausschalten");
    LinMessage[6] = uint8_t(0x01);  // Lüfter ausschalten wenn nur Boiler läuft und nicht geheizt wird
  } else {
    // DEBUG__PRINTF("Lüfter:%d\n", fan);
    LinMessage[6] = uint8_t(16 * fan) + uint8_t(0x01);  // Model (X1 - Gas  X2 - Elektro  X3 - Mix  BX - Lüfter Eco DX - Lüfter High  1X - vent 1 2X - Vent 2 3X - Vent 3 4X - Vent 4 5X - Vent 5 6X - Vent 6 7X - Vent 7 8X - Vent 8 9X - Vent 9 AX - Vent 10)
  }

  LinMessage[7] = 0xE0;  // Unknown (E0)
  LinMessage[8] = 0x0F;  // Unknown (0F)

  // uint8_t linTest1[] = {0x20,0xAA,0x2A,0xD1,0xFA,0x00,0x01,0xE0,0x0F};

  // memcpy(LinMessage,linTest1, 9);

  // uint8_t cksum2 = LINChecksum(9, true);
  uint8_t cksum = LINChecksum(LinMessage + 1, 8, PID);
  // TEST: DEBUG__PRINTF("chksum:%x  %x\n", cksum, cksum2);

  serialBreak();
  linSerial.write(0x55);                            // Sync
  linSerial.write(LinMessage, sizeof(LinMessage));  // Message (array from 1..8) // Original
  linSerial.write(cksum);
  linSerial.flush();

  if (debug) {
    Serial.print("ID: ");
    Serial.print(0x20, HEX);
    Serial.print(" --> ");
    Serial.print(0x20, HEX);
    Serial.print(": ");
    for (int ixx = 1; ixx < 9; ixx++) {
      Serial_printf("%02x:", LinMessage[ixx]);
    }
    Serial_printf(" - %02x", cksum);
    Serial.println("  truma_message");
  }

}  // end of function
//-----------------------------------------------------------------------------------------------------------------
void trum_diagn_request(int ixx) {
  // DEBUG__PRINTLN("trum_diagn_request");
  switch (ixx) {
    case 0:
      LinMessage[0] = 0x3C;  // Frame identifier (20-PID)
      LinMessage[1] = 0x01;
      LinMessage[2] = 0x06;
      LinMessage[3] = 0xB8;
      LinMessage[4] = 0x40;
      LinMessage[5] = 0x03;
      if ((heat | boil | fan) > 0) {  // If any function is selected, byte 6 changes too "0x01"
        LinMessage[6] = 0x01;
      } else {
        LinMessage[6] = 0x00;
      }
      LinMessage[7] = 0x00;
      LinMessage[8] = 0xFF;
      break;
    case 1:
      LinMessage[0] = 0x3C;  // Frame identifier (20-PID)
      LinMessage[1] = 0x7F;
      LinMessage[2] = 0x06;
      LinMessage[3] = 0xB2;
      LinMessage[4] = 0x00;
      LinMessage[5] = 0x17;
      LinMessage[6] = 0x46;
      LinMessage[7] = 0x40;
      LinMessage[8] = 0x03;
      break;
    case 2:
      LinMessage[0] = 0x3C;  // Frame identifier
      LinMessage[1] = 0x7F;
      LinMessage[2] = 0x06;
      LinMessage[3] = 0xB2;
      LinMessage[4] = 0x23;  // Difference
      LinMessage[5] = 0x17;
      LinMessage[6] = 0x46;
      LinMessage[7] = 0x40;
      LinMessage[8] = 0x03;
      break;
  }

  // uint8_t cksum = LINChecksum(9, false);
  uint8_t cksum = LINChecksum(LinMessage + 1, 8, 0);

  serialBreak();
  linSerial.write(0x55);                            // Sync
  linSerial.write(LinMessage, sizeof(LinMessage));  // Message (array from 1..8) // Original
  linSerial.write(cksum);
  linSerial.flush();

  if (debug) {
    Serial.print("ID: ");
    Serial.print(0x3C, HEX);
    Serial.print(" --> ");
    Serial.print(0x3C, HEX);
    Serial.print(": ");
    for (int i = 1; i < 9; i++) {
      Serial_printf("%02x:", LinMessage[i]);
    }
    Serial_printf(" - %02x", cksum);
    Serial.println("  diagn_request");
  }

  // Evaluate answer
  if (readFrame(0x3D)) {  // Read diagnostic answer
    // Error messages for IO Broker (MQTT)
    const char* ErrMessage[] = {
        "E517H - with reset", "E517H - with reset", "E517H", "E517H - reset unknown", "-", "W255H", "W255H",
    };  // Error messages

    // LIN error messages
    uint8_t err[][9] = {
        {0x7d, 0x01, 0x06, 0xF2, 0x02, 0x05, 0x11, 0xCA, 0x0A},  // E517H - Gas empty
        {0x7d, 0x01, 0x06, 0xF2, 0x02, 0x05, 0x11, 0x2D, 0x0C},  // E517H - Gas empty
        {0x7d, 0x01, 0x06, 0xF2, 0x02, 0x05, 0x11, 0xE5, 0x0C},  // E517H - Gas empty
        {0x7d, 0x01, 0x06, 0xF2, 0x02, 0x05, 0x11, 0x1A, 0x0C},  // E517H - Gas empty
        {0x7d, 0x01, 0x06, 0xF2, 0x02, 0x00, 0x00, 0x00, 0x00},  // Normal - normal operating mode
        {0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // W255H - no connection between devices (no Lin answer);
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // W255H - no connection between devices (no Lin answer)

    };

    const int nElements = sizeof(err) / sizeof(err[0]);  // Anzahl
    if (ixx == 2) {                                      // Only look at answer diagnose request of case 2
      diagMessage = "";
      for (int i = 0; i < nElements; i++) {
        if (memcmp(LinMessageA, err[i], sizeof(err[i])) == 0) {
          diagMessage = ErrMessage[i];
          break;
        }
      }
    }
  }

}  // end of function

//-----------------------------------------------------------------------------------------------------------------
void trum_init()  // Not functional and maybe not necessary
{
  DEBUG__PRINTLN("Truma-Init");
  int ixy = 0;

  for (int ixx = 0; ixx < 7; ixx++) {
    if (debug) {
      Serial.print("Block Init ");
      Serial.print(ixx + 1);
      Serial.println(" ------------------------------------------------");
    }

    LinMessage[0] = 0x3C;  // Frame identifier
    LinMessage[1] = 0x7F;
    LinMessage[2] = 0x06;
    LinMessage[3] = 0xB2;
    LinMessage[4] = 0x00;
    LinMessage[5] = 0x17;
    LinMessage[6] = 0x46;
    LinMessage[7] = 0x01;  // Ändert sich
    LinMessage[8] = 0x0C;

    switch (ixx) {
      case 0:
        LinMessage[7] = 0x01;
        break;
      case 1:
        LinMessage[7] = 0x00;
        break;
      case 2:
        LinMessage[7] = 0x04;
        break;
      case 3:
        LinMessage[7] = 0x05;
        break;
      case 4:
        LinMessage[7] = 0x06;
        break;
      case 5:
        LinMessage[7] = 0x07;
        break;
      case 6:
        LinMessage[0] = 0x3C;  // Frame identifier
        LinMessage[1] = 0xFF;
        LinMessage[2] = 0xFF;
        LinMessage[3] = 0xFF;
        LinMessage[4] = 0xFF;
        LinMessage[5] = 0xFF;
        LinMessage[6] = 0xFF;
        LinMessage[7] = 0xFF;
        LinMessage[8] = 0xFF;
        break;
    }

    // uint8_t cksum = LINChecksum(9, false);
    uint8_t cksum = LINChecksum(LinMessage + 1, 8, 0);

    serialBreak();
    linSerial.write(0x55);                            // Sync
    linSerial.write(LinMessage, sizeof(LinMessage));  // Message (array from 1..8) // Original
    linSerial.write(cksum);
    linSerial.flush();

    if (debug) {
      Serial.print("ID: ");
      Serial.print(0x3C, HEX);
      Serial.print(" --> ");
      Serial.print(0x3C, HEX);
      Serial.print(": ");
      for (int ixx = 1; ixx < 9; ixx++) {
        Serial_printf("%02x:", LinMessage[ixx]);
      }
      Serial_printf(" - %02x", cksum);
      Serial.println("  Init Ende");
    }

    if (ixx < 6) {
      readFrame(0x3D);  // Read diagnostic answer
      //      readFrame(0x21); // Read frame 21
      //      readFrame(0x22); // Read frame 22

      //  Start Diagnosis
      trum_diagn_request(ixy);
      ixy++;
      if (ixy > 2) {
        ixy = 0;
      }
    }
  }
  delay(1500);  // Same delay as in spreadsheet

}  // end of function

void trum_reset() {
  DEBUG__PRINTLN("Truma-Reset");
  LinMessage[0] = 0x3C;  // Frame identifier
  LinMessage[1] = 0xFF;
  LinMessage[2] = 0xFF;
  LinMessage[3] = 0xFF;
  LinMessage[4] = 0xFF;
  LinMessage[5] = 0xFF;
  LinMessage[6] = 0xFF;
  LinMessage[7] = 0xFF;
  LinMessage[8] = 0xFF;

  // uint8_t cksum = LINChecksum(9, false);
  uint8_t cksum = LINChecksum(LinMessage + 1, 8, 0);

  serialBreak();
  linSerial.write(0x55);                            // Sync
  linSerial.write(LinMessage, sizeof(LinMessage));  // Message (array from 1..8) // Original
  linSerial.write(cksum);
  linSerial.flush();

  if (debug) {
    Serial.print("ID: ");
    Serial.print(0x3C, HEX);
    Serial.print(" --> ");
    Serial.print(0x3C, HEX);
    Serial.print(": ");
    for (int ixx = 1; ixx < 9; ixx++) {
      Serial_printf("%02x:", LinMessage[ixx]);
      Serial.print(":");
    }
    Serial_printf(" - %02x", cksum);
    Serial.println("  Reset Ende");
  }

}  // end of function

//-----------------------------------------------------------------------------------------------------------------
//
void setup_wifi() {
#ifdef mitWIFI
  Serial.begin(BAUD);
  delay(500);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to wifi network: ");

  WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);

  // scan....
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == ssid) {
      WiFi.begin(ssid, password);  // trying to SSID (OscarPC)
      Serial.println(ssid);
      break;
    }
    if (WiFi.SSID(i) == ssid2) {
      WiFi.begin(ssid2, password2);  // trying to connect SSID2 (Gehoert_mir!!!)
      Serial.println(ssid2);
      break;
    }
  }

  for (int i = 0; i <= 20; i++) {
    if (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.flush();
  delay(50);
  Serial.end();
#endif
}  // end of function
//-----------------------------------------------------------------------------------------------------------------
//
void connect_mqtt() {
#ifdef mitWIFI
  // Loop until we're reconnected
  Serial.begin(BAUD);
  delay(50);
  Serial.print("Attempting MQTT connection...");
  String clientId = "TrumaHeater";  // Create a random client ID
  // Attempt to connect
  if (client.connect(clientId.c_str(), userMqqt, pwdMqqt)) {  // put your clientId/userName/passWord here
    Serial.println("connected");
    delay(50);
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    pinMode(2, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
    for (int i = 0; i <= 5; i++) {
      digitalWrite(2, LOW);  // Turn the LED on (Note that LOW is the voltage level
      delay(500);
      digitalWrite(2, HIGH);  // Turn the LED off
      delay(500);
    }
  }
  //}
  Serial.end();
#endif
}  // end of function
//-----------------------------------------------------------------------------------------------------------------
//
void callback(char* topic, uint8_t* payload, unsigned int length) {
#ifdef mitWIFI
  char msg[length + 1];
  for (uint16_t i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }

  if (debug) {
    Serial.begin(BAUD);
    delay(50);
    Serial.print("Received message [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.print(msg);
    Serial.println();
    Serial.flush();
    Serial.end();
  }

  // Set heating
  if (strcmp(topic, "truma_heating") == 0) {
    if (strcmp(msg, "ON") == 0) {
      heat = true;  // Heizung aktivieren
      //            Serial.println("Heating activated");
    } else {
      heat = false;  // Heizung deaktivieren
      //      Serial.println("Heating deactivated");
    }
  }

  ////  // Set boiler
  //  if (strcmp(topic, "truma_boiler") == 0) {
  //    if (strcmp(msg, "ON") == 0) {
  //      boil = true; // Boiler aktivieren
  //      boilMode = 1; // Set boiler to "hot"
  //      //      Serial.println("Boiler activated");
  //    }
  //    else {
  //      boil = false; // Boiler deaktivieren
  //      boilMode = 0; // Set boiler to "off"
  //      //      Serial.println("Boiler deactivated");
  //    }
  //  }

  // Set boiler mode
  if (strcmp(topic, "truma_boiler") == 0) {
    msg[length + 1] = '\0';  // Make payload a string by NULL terminating it.
    boilMode        = atoi((char*)msg);

    if (boilMode == 0) {
      boil = false;  // Deactivate boiler if mode=0 (off)
    } else {
      boil = true;  // Activate boiler if mode>0 (not off)
    }
    //    Serial.begin(9600);
    //    delay(50);
    //    Serial.println("Set boiler mode");
    //    Serial.println(boilMode);
    //    Serial.println(boil);
    //    Serial.flush();
    //    delay(50);
    //    Serial.end();
  }

  // Set tempearature
  if (strcmp(topic, "truma_temp") == 0) {
    msg[length + 1] = '\0';  // Make payload a string by NULL terminating it.
    temp            = atoi((char*)msg);
    //    Serial.println("Set Truma temp");
    //    Serial.println(temp);
  }

  // Set ventilation
  if (strcmp(topic, "truma_fan") == 0) {
    msg[length + 1] = '\0';  // Make payload a string by NULL terminating it.
    fan             = atoi((char*)msg);
    //    Serial.println("Set fan level");
    //    Serial.println(fan);
  }

  // Error reset
  if (strcmp(topic, "truma_reset") == 0) {
    if (strcmp(msg, "ON") == 0) {
      rst = true;  // Delete error
      //            Serial.println("Heating activated");
    } else {
      rst = false;  //
      //      Serial.println("Heating deactivated");
    }
  }
#endif
}  // end of function

//-----------------------------------------------------------------------------------------------------------------
//

void serialCommand() {
  String msg;
  int tmp;
  if (Serial.available()) {
    msg = Serial.readString();
    DEBUG__PRINTF("msg:%s", msg.c_str());
    switch (msg[0]) {
      case 't':  // Temperatur Heizung
        tmp = msg.substring(1).toInt();
        DEBUG__PRINTF("t:%i\n", tmp);
        if ((tmp >= 5) && (tmp <= 30)) {
          heat = true;  // Heizung einschalten
          temp = tmp;
        } else {
          heat = false;  // Heizung ausschalten
          temp = 0;
        }
        break;
      case 'b':  // Boiler    (off=0,eco=1,hot=2,boost=3)
        tmp = msg.substring(1).toInt();
        DEBUG__PRINTF("b:%i\n", tmp);
        if ((tmp <= 0) || (tmp >= 3)) {
          boilMode = 0;
          boil     = false;  // Boiler ausschalten if mode = 0 oder unzulässig
        } else {
          boilMode = tmp;
          boil     = true;  // Boiler einschalten if mode = 1|2|3
        }
        break;
      case 'v':  // Ventilator (off=0,Stufe1=1 - Stufe10=10,Eco=11,High=13)
        tmp = msg.substring(1).toInt();
        DEBUG__PRINTF("v:%i\n", tmp);
        if ((tmp >= 0) && (tmp <= 13)) {
          fan = tmp;  // Ventilator einschalten
        } else {
          fan = 0;  // Ventilator ausschalten
        }
        break;
      case 'r':  // Reset
        DEBUG__PRINTF("r: \n");
        rst = true;  // Fehler quitieren
        break;

      default:
        break;
    }
    delay(1000);
  }
}
