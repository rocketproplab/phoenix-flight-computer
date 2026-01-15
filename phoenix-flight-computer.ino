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
// Instance Declaration
// ─────────────────────────────────────────────────────────────────────────────
//Ethernet
Wiznet5500 w5500;
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

// ─────────────────────────────────────────────────────────────────────────────
// Pin Definitions
// ─────────────────────────────────────────────────────────────────────────────

//Ethernet Chip Select
#define EthernetCS 53
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

// String formatting helpers

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry formatting helpers (heap-free, bounded, AVR-safe)
// ─────────────────────────────────────────────────────────────────────────────
static size_t appendStr(char *out, size_t outSz, size_t pos, const char *s) {
  if (outSz == 0) return 0;
  while (*s && (pos + 1) < outSz) out[pos++] = *s++;
  out[pos] = '\0';
  return pos;
}

static size_t appendCsvFloat(char *out, size_t outSz, size_t pos, float v, uint8_t prec, bool leadingComma) {
  if (outSz == 0) return 0;
  if (leadingComma && (pos + 1) < outSz) out[pos++] = ',';

  char num[24];
  dtostrf(v, 0, prec, num);   // AVR-safe float->string (no printf float support needed)
  char *p = num;
  while (*p == ' ') p++;      // trim any left-padding
  return appendStr(out, outSz, pos, p);
}
// ─────────────────────────────────────────────────────────────────────────────
// Ethernet (MAC‑RAW) Constants
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t PHX_ETYPE_HI = 0x88;
static const uint8_t PHX_ETYPE_LO = 0x89;

uint8_t buffer[1514];
// first byte 0x02 = locally‑administered, unicast
const uint8_t MAC_GROUND_STATION[6] = {0x02, 0x47, 0x53,
                                       0x00, 0x00, 0x01}; // GS:  ground station
const uint8_t MAC_RELIEF_VALVE[6] = {0x02, 0x52, 0x56,
                                     0x00, 0x00, 0x02}; // RV:  relief valve
const uint8_t MAC_FLOW_VALVE[6] = {0x02, 0x46, 0x4C,
                                   0x00, 0x00, 0x03}; // FL:  flow valve
const uint8_t MAC_SENSOR_GIGA[6] = {0x02, 0x53, 0x49,
                                    0x00, 0x00, 0x04}; // SI:  sensor interface

// message types
enum : uint8_t {
  MSG_DISCOVER  = 0x01,  // GS -> broadcast (unpaired discovery)
  MSG_AWAKE     = 0x02,  // FC -> GS (ack discover / ready)
  MSG_COMMAND   = 0x03,  // GS -> FC (rocketState)
  MSG_TELEMETRY = 0x04   // FC -> GS (telemetry response)
};

// packed ethernet header
struct __attribute__((packed)) PhxHdr {
  uint8_t dstMac[6];
  uint8_t srcMac[6];
  uint8_t ethType[2]; // {0x88,0x89}
  uint8_t msgType;
  uint8_t seq;
};

static const size_t PHX_HDR_LEN = sizeof(PhxHdr);
static const size_t PHX_MAX_PAYLOAD = 300;      // keep consistent with your RAM budget
static uint8_t txBuf[PHX_HDR_LEN + PHX_MAX_PAYLOAD];

// Link state
enum FcLinkState : uint8_t {
  FC_UNPAIRED = 0,
  FC_AWAKE = 1
};

static FcLinkState fcLink = FC_UNPAIRED;
static unsigned long fcLastRxMs = 0;
static unsigned long fcLastAwakeTxMs = 0;

static const unsigned long FC_LINK_TIMEOUT_MS = 3000;        // declare link lost if no commands/discovers
static const unsigned long FC_AWAKE_THROTTLE_MS = 150;       // rate-limit AWAKE replies

// ─────────────────────────────────────────────────────────────────────────────
// Ethernet Transmission Helpers
// ─────────────────────────────────────────────────────────────────────────────

static uint16_t phxPadLen(uint16_t n) {
  return (n<60) ? 60 : n; // minumum Ethernet frame size
}

static int16_t phxSend (uint8_t msgType, uint8_t seq, const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen > PHX_MAX_PAYLOAD) {
    Serial.print(F("[seq=")); Serial.print(seq);
    Serial.print(F("] len ")); Serial.print(payloadLen);
    Serial.println(F(" over limit!"));
    payloadLen = PHX_MAX_PAYLOAD;
  }

  memcpy(txBuf + 0, MAC_GROUND_STATION, 6);
  memcpy(txBuf + 6, MAC_SENSOR_GIGA,    6);
  txBuf[12] = PHX_ETYPE_HI;
  txBuf[13] = PHX_ETYPE_LO;
  txBuf[14] = msgType;
  txBuf[15] = seq;

  if (payloadLen && payload) {
    memcpy(txBuf + PHX_HDR_LEN, payload, payloadLen);
  }

  uint16_t frameLen = phxPadLen((uint16_t)(PHX_HDR_LEN + payloadLen));

  // single send instance:
  int16_t rc = w5500.sendFrame(txBuf, frameLen);

  return rc;
}

static void fcSendAwake(uint8_t seq)
{
  phxSend(MSG_AWAKE, seq, nullptr, 0);
}

static void fcBuildTelemetryCsv(char *out, size_t outSz)
{
  // LOAD CELL DATA
  float loadOutput1 = 0;
  float loadOutput2 = 0;
  float loadOutput3 = 0;

  // THERMOCOUPLE DATA
  float thermoCouple1 = 0;
  float thermoCouple2 = 0;

  size_t pos = 0;
  if (outSz) out[0] = '\0';

  // PT data
  for (uint8_t ptID = 0; ptID < countPTs(); ++ptID) {
    pos = appendCsvFloat(out, outSz, pos, (float)readPT(ptID), 2, (ptID > 0));
    if (pos + 1 >= outSz) break;
  }

  // Append remaining loads and thermos
  pos = appendCsvFloat(out, outSz, pos, loadOutput1, 2, true);
  pos = appendCsvFloat(out, outSz, pos, loadOutput2, 2, true);
  pos = appendCsvFloat(out, outSz, pos, loadOutput3, 2, true);
  pos = appendCsvFloat(out, outSz, pos, thermoCouple1, 2, true);
  pos = appendCsvFloat(out, outSz, pos, thermoCouple2, 2, true);
}


static void fcSendTelemetry(uint8_t seq)
{
  char csv[PHX_MAX_PAYLOAD];
  fcBuildTelemetryCsv(csv, sizeof(csv));

  // Best-effort serial logging without blocking the main loop
  const size_t csvLen = strnlen(csv, sizeof(csv) - 1);
  if (Serial.availableForWrite() >= (csvLen + 20)) {
    Serial.print(F("[TELEM seq="));
    Serial.print(seq);
    Serial.print(F("] "));
    Serial.write((const uint8_t*)csv, csvLen);
    Serial.println();
  }

  // Include '\0' to make GS printing safe/easy.
  const uint16_t payloadLen = (uint16_t)(strnlen(csv, sizeof(csv) - 1) + 1);
  (void)phxSend(MSG_TELEMETRY, seq, (const uint8_t *)csv, payloadLen);
}

static void fcLinkLost()
{
  if (fcLink == FC_AWAKE && Serial.availableForWrite() >= 48) {
    Serial.println(F("[LINK] lost -> UNPAIRED"));
  }
  fcLink = FC_UNPAIRED;
}

static void fcProcessRx()
{
  uint16_t len;
  while ((len = w5500.readFrame(buffer, sizeof(buffer))) > 0) {

    if (len < PHX_HDR_LEN) {
      continue;
    }

    // Filter by Ethertype
    if (buffer[12] != PHX_ETYPE_HI || buffer[13] != PHX_ETYPE_LO) {
      continue;
    }

    const uint8_t msgType = buffer[14];
    const uint8_t seq     = buffer[15];
    const uint8_t *srcMac = buffer + 6;

    const unsigned long now = millis();
    fcLastRxMs = now;

    if (msgType == MSG_DISCOVER) {
      // set to AWAKE
      fcLink = FC_AWAKE;

      if (now - fcLastAwakeTxMs >= FC_AWAKE_THROTTLE_MS) {
        fcSendAwake(seq);
        fcLastAwakeTxMs = now;
      }
      continue;
    }

    if (msgType == MSG_COMMAND) {
      // 1 byte rocketState payload expected
      if (len < (PHX_HDR_LEN + 1)) continue;

      fcLink = FC_AWAKE;

      const uint8_t newState = buffer[PHX_HDR_LEN];
      valveSetState(newState);
      fcSendTelemetry(seq);
      continue;
    }
  }
}

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

  // LOAD CELL CALIBRATION
  // loadCell1.set_scale(calibration_factor1); // Adjust to this calibration factor
  // loadCell2.set_scale(calibration_factor2); // Adjust to this calibration factor
  // loadCell3.set_scale(calibration_factor3); // Adjust to this calibration factor

  // LOAD CELL DATA
  // double loadOutput1 = loadCell1.get_units();
  // double loadOutput2 = loadCell2.get_units();
  // double loadOutput3 = loadCell3.get_units();
  double loadOutput1 = 0;

  //thermocouple setup
  
  // thermoCouples.begin();
  // if (!mcp.begin(I2C_ADDRESS)) {
  //   Serial.println("Sensor not found. Check wiring!");
  //   while (1);
  // }
  // mcp.setAmbientResolution(ambientRes);

  // THERMOCOUPLE DATA
  // thermoCouples.requestTemperatures();
  // double thermoCouple1 = thermoCouples.getTempCByIndex(0);
  // double thermoCouple2 = thermoCouples.getTempCByIndex(1);
  double thermoCouple1 = 0;
  double thermoCouple2 = 0;

  setupValves();
  setupPTs();
  Serial.println("Done Initialized");
}

void loop() {
  // drain incoming frames
  fcProcessRx();

  // if timeout, return to unpaired
  const unsigned long now = millis();
  if (fcLink == FC_AWAKE && (now - fcLastRxMs) > FC_LINK_TIMEOUT_MS) {
    fcLinkLost();
  }

  // update valve states regardless of timing.
  valveUpdateStates();
  valveApplyVoltages();
}