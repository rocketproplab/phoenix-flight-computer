// ─────────────────────────────────────────────────────────────────────────────
// Libraries
// ─────────────────────────────────────────────────────────────────────────────

// SPI Libs
#include <SPI.h>
// Ethernet Lib
#include "./src/w5500/w5500.h"
// Constants
#include "./src/constants.h"
#include "./src/role_config.h"
// SD Card Lib
// #include <SD.h>
// File myFile;

// Sensors and actuators
#if PHOENIX_ROLE_HAS_VALVES
#include "src/valve/valve.h"
#endif
#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
#include "src/pt/pt.h"
#include "src/lc/lc.h"
#include "src/tc/tc.h"
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Pin Definitions
// ─────────────────────────────────────────────────────────────────────────────

//Ethernet Chip Select
#define EthernetCS 10
#define SDCS 40

// ─────────────────────────────────────────────────────────────────────────────
// Ethernet (MAC‑RAW)
// ─────────────────────────────────────────────────────────────────────────────

#if PHOENIX_ROLE_HAS_VALVES
uint8_t buffer[64];
uint8_t valveState = 0;
#endif

#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
// 
struct TelemetryFrame {
  // –––– Ethernet header (14 B) ––––
  uint8_t dstMac[6];
  uint8_t srcMac[6];
  uint16_t ethType; // always 0x8889

  // –––– Payload ––––
  uint8_t payload[200]; // 200 bytes usually enough.. optimize later
};
TelemetryFrame f;
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Struct Declaration
// ─────────────────────────────────────────────────────────────────────────────

//Ethernet
Wiznet5500 w5500;

//Valves
#if PHOENIX_ROLE_HAS_VALVES
Valve valves[] = {
#if defined(PHOENIX_ROLE_FLOW_VALVES)
  {"LNG_PRES", LNG_PRES_MASK, LNG_PRES_PIN, false, 0},
  {"LOX_PRES", LOX_PRES_MASK, LOX_PRES_PIN, false, 0},
  {"LNG_FLOW", LNG_FLOW_MASK, LNG_FLOW_PIN, false, 0},
  {"LOX_FLOW", LOX_FLOW_MASK, LOX_FLOW_PIN, false, 0}
#elif defined(PHOENIX_ROLE_RELIEF_VALVES)
  {"LNG_VENT", LNG_VENT_MASK, LNG_VENT_PIN, false, 0},
  {"LOX_VENT", LOX_VENT_MASK, LOX_VENT_PIN, false, 0}
#endif
};

static const size_t NUM_VALVES = sizeof(valves) / sizeof(valves[0]);
#endif

#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
unsigned long lastSend = 0;

// pressure transducer declaration
PT pts[] = {
  // name,            pin,  range,   voltageMin, voltageMax, analogRange, voltageRange, initialized
  { "GN2",            PT_GN2_PIN,            5000.0f, 0.5f,       4.5f,       1023.0f,     5.0f,         false },
  { "LOX-UPSTREAM",   PT_LOX_UPSTREAM_PIN,   1000.0f, 1.0f,       5.0f,       1023.0f,     5.0f,         false },
  { "LNG-UPSTREAM",   PT_LNG_UPSTREAM_PIN,   1000.0f, 1.0f,       5.0f,       1023.0f,     5.0f,         false },
  { "LOX-DOWNSTREAM", PT_LOX_DOWNSTREAM_PIN, 1000.0f, 1.0f,       5.0f,       1023.0f,     5.0f,         false },
  { "LNG-DOWNSTREAM", PT_LNG_DOWNSTREAM_PIN, 1500.0f, 0.5f,       4.5f,       1023.0f,     5.0f,         false },
  { "LOX-DOME",       PT_LOX_DOME_PIN,       1500.0f, 0.5f,       4.5f,       1023.0f,     5.0f,         false },
  { "LNG-DOME",       PT_LNG_DOME_PIN,       1500.0f, 0.5f,       4.5f,       1023.0f,     5.0f,         false },
};
static const size_t NUM_PTS = sizeof(pts) / sizeof(pts[0]);

// load cell declaration
LC loadCells[] = {
  {"STAR", LC_STAR_CALIBRATION_FACTOR, LC_STAR_DOUT_PIN, LC_STAR_CLK_PIN, false},
  {"CIRCLE", LC_CIRCLE_CALIBRATION_FACTOR, LC_CIRCLE_DOUT_PIN, LC_CIRCLE_CLK_PIN, false},
  {"TRIANGLE", LC_TRIANGLE_CALIBRATION_FACTOR, LC_TRIANGLE_DOUT_PIN, LC_TRIANGLE_CLK_PIN, false},
};
static const size_t NUM_LCS = sizeof(loadCells) / sizeof(loadCells[0]);

// thermocouple declaration
TC thermocouples[] = {
  {'k', TC_I2C_ADDRESS_1, false},
  {'t', TC_I2C_ADDRESS_2, false},
  {'t', TC_I2C_ADDRESS_3, false},
  {'k', TC_I2C_ADDRESS_4, false},
  {'k', TC_I2C_ADDRESS_5, false},
};
static const size_t NUM_TCS = sizeof(thermocouples) / sizeof(thermocouples[0]);
#endif

void setup() {
  Serial.begin(115200);

  // Ethernet setup
#if defined(PHOENIX_ROLE_SENSOR_TELEMETRY)
  w5500.begin(MAC_SENSOR_GIGA);
#elif defined(PHOENIX_ROLE_FLOW_VALVES)
  w5500.begin(MAC_FLOW_VALVE);
#elif defined(PHOENIX_ROLE_RELIEF_VALVES)
  w5500.begin(MAC_RELIEF_VALVE);
#endif
  Serial.println("Ethernet Setup Complete");

#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
  // pt setup
  for (size_t i = 0; i < NUM_PTS; i++) {
    initPT(&pts[i]);
  }

  //load cell setup
  for (size_t i = 0; i < NUM_LCS; i++) {
    Serial.print("Load ");
    Serial.print(loadCells[i].name);
    if (!initLC(&loadCells[i])) Serial.println(" failed");
    else Serial.println(" Done");
  }

  //thermocouple setup
  Serial.println("MCP9600 HW test");
  for (size_t i = 0; i < NUM_TCS; i++) {
    if (!initTC(&thermocouples[i])) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" not found. Check wiring!");
    }
  }

#endif

#if PHOENIX_ROLE_HAS_VALVES
  for (size_t i = 0; i < NUM_VALVES; i++) {
    initValve(&valves[i]);
  }
#endif
  Serial.println("Done Initialized");
}

#if PHOENIX_ROLE_HAS_VALVES
void receiveValveState() {
  uint16_t len;
  while ((len = w5500.readFrame(buffer, sizeof(buffer))) > 0){
    if (len < 15)
      continue;
    if (buffer[12] != 0x63 || buffer[13] != 0xe4)
      continue;
    valveState = buffer[14];
    // Serial.println(valveState);
    for (size_t i = 0; i < NUM_VALVES; i++) {
      valveSetOpen(&valves[i], (valveState & valves[i].mask) != 0);
    }
  }
}
#endif

#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
void sendSensorData() {
  float pressureReadings[NUM_PTS] = {};
  float loadReadings[NUM_LCS] = {};
  float temperatureReadings[NUM_TCS] = {};

  for (size_t i = 0; i < NUM_PTS; i++) {
    readPT(&pts[i], &pressureReadings[i]);
  }

  for (size_t i = 0; i < NUM_LCS; i++) {
    readLC(&loadCells[i], &loadReadings[i]);
  }

  for (size_t i = 0; i < NUM_TCS; i++) {
    readTC(&thermocouples[i], &temperatureReadings[i]);
  }

  // LIVE ETHERNET TRANSMISSION
  // Put all data into an output string to be sent over
  String dataOut;

  for (size_t i = 0; i < NUM_PTS; i++) {
    if (dataOut.length() > 0) dataOut += ",";
    dataOut += String(pressureReadings[i]);
  }

  for (size_t i = 0; i < NUM_LCS; i++) {
    if (dataOut.length() > 0) dataOut += ",";
    dataOut += String(loadReadings[i]);
  }

  for (size_t i = 0; i < NUM_TCS; i++) {
    if (dataOut.length() > 0) dataOut += ",";
    dataOut += String(temperatureReadings[i]);
  }
                   
  Serial.println(dataOut);

  // Send data

  const char *data = dataOut.c_str();
  int len = strlen(data);
  memset(&f, 0, sizeof(f)); // set to 0
  memcpy(f.dstMac, MAC_GROUND_STATION, 6); //dst
  memcpy(f.srcMac, MAC_SENSOR_GIGA, 6); //src
  ((byte *)&f)[12] = 0x88; ((byte *)&f)[13] = 0x89; // sensor frame
  // Serial.print("Data len: ");
  // Serial.println(len);
  memcpy(f.payload, data, len);
  if (w5500.sendFrame((byte *)&f, sizeof(f)) <= 0) {
    Serial.println("Ethernet send Failed");
  }
  
  // myFile = SD.open("StaticFire.txt", FILE_WRITE);
  // if (myFile) {
  //   Serial.print("Writing to file...");
  //   myFile.println(dataOut);
  //   // close the file:
  //   myFile.close();
  //   Serial.println("done.");
  // } else {
  //   // if the file didn't open, print an error:
  //   Serial.println("error opening file");
  // }
}
#endif

void loop() {
#if PHOENIX_ROLE_HAS_VALVES
  receiveValveState();
  for (size_t i = 0; i < NUM_VALVES; i++) {
    valveApplyState(&valves[i]);
  }
#endif

#if PHOENIX_ROLE_HAS_SENSOR_TELEMETRY
  if (millis() - lastSend >= TELEMETRY_DELAY) {
    lastSend = millis();
    sendSensorData();
  }
#endif
}
