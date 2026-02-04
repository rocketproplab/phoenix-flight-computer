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
#define DOUT_STAR 16
#define CLK_STAR  17
#define DOUT_CIRCLE 14
#define CLK_CIRCLE  15
#define DOUT_TRIANGLE 18
#define CLK_TRIANGLE  19

//ThermoCouple
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS1 (0x67) //no shorts, not grounded
#define I2C_ADDRESS2 (0x66) //Top short only
#define I2C_ADDRESS3 (0x65) //Bottom short only
#define I2C_ADDRESS4 (0x64) //Both Shorted
#define I2C_ADDRESS5 (0x60) //GND Adress

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;
Adafruit_MCP9600 mcp3;
Adafruit_MCP9600 mcp4;
Adafruit_MCP9600 mcp5;

/* Set and print ambient resolution */
Ambient_Resolution ambientRes1 = RES_ZERO_POINT_0625;
Ambient_Resolution ambientRes2 = RES_ZERO_POINT_0625;
Ambient_Resolution ambientRes3 = RES_ZERO_POINT_0625;
Ambient_Resolution ambientRes4 = RES_ZERO_POINT_0625;
Ambient_Resolution ambientRes5 = RES_ZERO_POINT_0625;

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
#include "HX711.h"
HX711 loadCell_star;
HX711 loadCell_circle;
HX711 loadCell_triangle;
//Thermocouples
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature thermoCouples(&oneWire);
// DeviceAddress addr;
//Adafruit_MCP9600 mcp;

//Other Consts ***************************************************
float calibration_factor_star = -5600; //-7050 worked for my 440lb max scale setup
float calibration_factor_circle = -5900; //-7050 worked for my 440lb max scale setup
float calibration_factor_triangle = -6000; //-7050 worked for my 440lb max scale setup

// Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;

void setup() {
  Serial.begin(115200);

  // Ethernet setup
  w5500.begin(MAC_SENSOR_GIGA);
  Serial.println("Ethernet Setup Complete");

  //load cell setup
  loadCell_star.begin(DOUT_STAR, CLK_STAR);
  loadCell_star.set_scale();
  loadCell_star.tare(); //Reset the scale to 0
  Serial.println("Load Star Done");
  
  loadCell_circle.begin(DOUT_CIRCLE, CLK_CIRCLE);
  loadCell_circle.set_scale();
  loadCell_circle.tare(); //Reset the scale to 0
  Serial.println("Load Circle Done");

  loadCell_triangle.begin(DOUT_TRIANGLE, CLK_TRIANGLE);
  loadCell_triangle.set_scale();
  loadCell_triangle.tare(); //Reset the scale to 0
  Serial.println("Load Triangle Done");

    Serial.println("MCP9600 HW test");

   /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp1.begin(I2C_ADDRESS1)) {
    Serial.println("Sensor 1 not found. Check wiring!");
  }
  
  if (! mcp2.begin(I2C_ADDRESS2)) {
    Serial.println("Sensor 2 not found. Check wiring!");
  }
  
  if (! mcp3.begin(I2C_ADDRESS3)) {
    Serial.println("Sensor 3 not found. Check wiring!");
  }
  
  if (! mcp4.begin(I2C_ADDRESS4)) {
    Serial.println("Sensor 4 not found. Check wiring!");
  }

  if (! mcp5.begin(I2C_ADDRESS5)) {
    Serial.println("Sensor 5 not found. Check wiring!");
  }

  /* Set and print ambient resolution */
  mcp1.setAmbientResolution(ambientRes1);
  mcp2.setAmbientResolution(ambientRes2);
  mcp3.setAmbientResolution(ambientRes3);
  mcp4.setAmbientResolution(ambientRes4);
  mcp5.setAmbientResolution(ambientRes5);

  mcp1.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp3.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp4.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp5.setADCresolution(MCP9600_ADCRESOLUTION_18);

  mcp1.setThermocoupleType(MCP9600_TYPE_K); //K
  mcp2.setThermocoupleType(MCP9600_TYPE_T);
  mcp3.setThermocoupleType(MCP9600_TYPE_T);
  mcp4.setThermocoupleType(MCP9600_TYPE_K); //K
  mcp5.setThermocoupleType(MCP9600_TYPE_K); //K
  

  mcp1.setFilterCoefficient(3);
  mcp2.setFilterCoefficient(3);
  mcp3.setFilterCoefficient(3);
  mcp4.setFilterCoefficient(3);
  mcp5.setFilterCoefficient(3);

  mcp1.enable(true);
  mcp2.enable(true);
  mcp3.enable(true);
  mcp4.enable(true);
  mcp5.enable(true);

  setupValves();
  setupPTs();
  Serial.println("Done Initialized");
}

void receiveValveState() {
  uint16_t len;
  while ((len = w5500.readFrame(buffer, sizeof(buffer))) > 0){
    if (buffer[12] != 0x63 || buffer[13] != 0xe4)
      continue;
    uint8_t new_state = buffer[14];
    // Serial.println(new_state);
    valveSetState(new_state);
  }
}

void sendSensorData() {
  // LOAD CELL CALIBRATION
  // loadCell_star.set_scale(calibration_factor1); // Adjust to this calibration factor
  // loadCell_circle.set_scale(calibration_factor2); // Adjust to this calibration factor
  // loadCell_triangle.set_scale(calibration_factor3); // Adjust to this calibration factor

  // LOAD CELL DATA
  double loadOutput1 = loadCell_star.get_units();
  double loadOutput2 = loadCell_circle.get_units();
  double loadOutput3 = loadCell_triangle.get_units();
  // double loadOutput1 = 0;
  // double loadOutput2 = 0;
  // double loadOutput3 = 0;

  // THERMOCOUPLE DATA

  // thermoCouples.requestTemperatures();
  // double thermoCouple1 = thermoCouples.getTempCByIndex(0);
  // double thermoCouple2 = thermoCouples.getTempCByIndex(1);
  double thermoCouple1 = mcp1.readThermocouple();
  double thermoCouple2 = mcp2.readThermocouple();
  double thermoCouple3 = mcp3.readThermocouple();
  double thermoCouple4 = mcp4.readThermocouple();
  double thermoCouple5 = mcp5.readThermocouple();
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
  
  dataOut = dataOut + "," 
    + String(loadOutput1) + "," 
    + String(loadOutput2) + "," 
    + String(loadOutput3) + ","
    + String(thermoCouple1) + "," 
    + String(thermoCouple2) + ","
    + String(thermoCouple3) + ","
    + String(thermoCouple4) + ","
    + String(thermoCouple5);
                   
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