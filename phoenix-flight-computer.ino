// ─────────────────────────────────────────────────────────────────────────────
// Libraries
// ─────────────────────────────────────────────────────────────────────────────

// SPI Libs
#include <SPI.h>
// Ethernet Lib
#include "./src/w5500/w5500.h"
// Constants
#include "./src/constants.h"
// SD Card Lib
// #include <SD.h>
// File myFile;
// Load Cell Lib
// #include "HX711.h"
// Thermocouple Libs
// #include <Wire.h>
// #include <DallasTemperature.h>
// I2C Libs for Thermocouples
// #include <Adafruit_I2CDevice.h>
// #include <Adafruit_I2CRegister.h>
// #include "Adafruit_MCP9600.h"
// #include <OneWire.h>

// ─────────────────────────────────────────────────────────────────────────────
// Pin Definitions
// ─────────────────────────────────────────────────────────────────────────────

//Ethernet Chip Select
#define EthernetCS 10
#define SDCS 40

//Load Cell Pins
// #define DOUT1  30
// #define CLK1  28
// #define DOUT2  34
// #define CLK2  32
// #define DOUT3  38
// #define CLK3  36
//ThermoCouple OneWire
// #define ONE_WIRE_BUS 22
// #define I2C_ADDRESS (0x67)

#include "src/valve/valve.h"
#include "src/pt/pt.h"
// ─────────────────────────────────────────────────────────────────────────────
// Ethernet (MAC‑RAW)
// ─────────────────────────────────────────────────────────────────────────────

uint8_t buffer[500];

// first byte 0x02 = locally‑administered, unicast
const uint8_t MAC_GROUND_STATION[6] = {0x02, 0x47, 0x53,
                                       0x00, 0x00, 0x01}; // GS:  ground station
const uint8_t MAC_RELIEF_VALVE[6] = {0x02, 0x52, 0x56,
                                     0x00, 0x00, 0x02}; // RV:  relief valve
const uint8_t MAC_FLOW_VALVE[6] = {0x02, 0x46, 0x4C,
                                   0x00, 0x00, 0x03}; // FL:  flow valve
const uint8_t MAC_SENSOR_GIGA[6] = {0x02, 0x53, 0x49,
                                    0x00, 0x00, 0x04}; // SI:  sensor interface

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

// ─────────────────────────────────────────────────────────────────────────────
// Instance Declaration
// ─────────────────────────────────────────────────────────────────────────────
//Ethernet
Wiznet5500 w5500;
unsigned long lastSend = 0;
//Load Cells
// HX711 loadCell1;
// HX711 loadCell2;
// HX711 loadCell3;
//Thermocouples
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature thermoCouples(&oneWire);
// DeviceAddress addr;
//Adafruit_MCP9600 mcp;

//Other Consts ***************************************************
// float calibration_factor1 = -7050; //-7050 worked for my 440lb max scale setup
// float calibration_factor2 = -7050; //-7050 worked for my 440lb max scale setup
// float calibration_factor3 = -7050; //-7050 worked for my 440lb max scale setup

// Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;

void setup() {
  Serial.begin(115200);

  // Ethernet setup
  w5500.begin(MAC_SENSOR_GIGA);
  Serial.println("Ethernet Setup Complete");

  //load cell setup
  // loadCell1.begin(DOUT1, CLK1);
  // loadCell1.set_scale();
  // loadCell1.tare(); //Reset the scale to 0
  // Serial.println("Load 1 Done");
  
  // loadCell2.begin(DOUT2, CLK2);
  // loadCell2.set_scale();
  // loadCell2.tare(); //Reset the scale to 0
  // Serial.println("Load 2 Done");

  // loadCell3.begin(DOUT3, CLK3);
  // loadCell3.set_scale();
  // loadCell3.tare(); //Reset the scale to 0
  // Serial.println("Load 3 Done");

  //thermocouple setup
  
  // thermoCouples.begin();
/*
  if (! mcp.begin(I2C_ADDRESS)) {
    Serial.println("Sensor not found. Check wiring!");
    while (1);
  }
  

  //mcp.setAmbientResolution(ambientRes);
  */

  setupValves();
  setupPTs();
  Serial.println("Done Initialized");
}

void receiveValveState() {
  uint16_t len;
  while ((len = w5500.readFrame(buffer, sizeof(buffer))) > 0){
    if (buffer[12] != 0x63 || buffer[13] != 0xe4)
      return;
    uint8_t new_state = buffer[14];
    valveSetState(new_state);
  }
}

void sendSensorData() {
  // LOAD CELL CALIBRATION
  // loadCell1.set_scale(calibration_factor1); // Adjust to this calibration factor
  // loadCell2.set_scale(calibration_factor2); // Adjust to this calibration factor
  // loadCell3.set_scale(calibration_factor3); // Adjust to this calibration factor

  // LOAD CELL DATA
  // double loadOutput1 = loadCell1.get_units();
  // double loadOutput2 = loadCell2.get_units();
  // double loadOutput3 = loadCell3.get_units();
  double loadOutput1 = 0;
  double loadOutput2 = 0;
  double loadOutput3 = 0;

  // THERMOCOUPLE DATA

  // thermoCouples.requestTemperatures();
  // double thermoCouple1 = thermoCouples.getTempCByIndex(0);
  // double thermoCouple2 = thermoCouples.getTempCByIndex(1);
  double thermoCouple1 = 0;
  double thermoCouple2 = 0;
  // double thermoCouple3=thermoCouples.getTempCByIndex(2);

  // LIVE ETHERNET TRANSMISSION
  // Put all data into an output string to be sent over
  // String
  // dataOut=String(PT1)+","+String(PT2)+","+String(PT3)+","+String(PT4)+","+String(PT5);
  // String
  // dataOut=String(loadOutput1)+","+String(loadOutput2)+","+String(loadOutput3)+","+String(thermoCouple1)+","+String(thermoCouple2);

  
  String dataOut;

  // construct PT data
  for (uint8_t ptID = 0; ptID < countPTs(); ++ptID) {
    double val = readPT(ptID);
    if (ptID > 0) dataOut += ",";
    dataOut += String(val);
  } 
  
  dataOut = dataOut + "," + String(loadOutput1) + "," + String(loadOutput2) + "," + String(loadOutput3) + ","
    + String(thermoCouple1) + "," + String(thermoCouple2);
                   
  Serial.println(dataOut);

  // Send data

  char *data = dataOut.c_str();
  int len = strlen(data);
  memset(&f, 0, sizeof(f)); // set to 0
  memcpy(f.dstMac, MAC_GROUND_STATION, 6);
  // ((byte *)&f)[0] = 0xFF;
  // ((byte *)&f)[1] = 0xFF;
  // ((byte *)&f)[2] = 0xFF;
  // ((byte *)&f)[3] = 0xFF;
  // ((byte *)&f)[4] = 0xFF;
  // ((byte *)&f)[5] = 0xFF;
  memcpy(f.srcMac, MAC_SENSOR_GIGA, 6);
  ((byte *)&f)[12] = 0x88;
  ((byte *)&f)[13] = 0x89;
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

void loop() {
  receiveValveState();
  valveUpdateStates();
  valveApplyVoltages();

  if (millis() - lastSend >= TELEMETRY_DELAY) {
    lastSend = millis();
    sendSensorData();
  }
}