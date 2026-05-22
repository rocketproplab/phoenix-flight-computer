#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// MAC addresses
static const uint8_t MAC_GROUND_STATION[6] = {0x02, 0x47, 0x53,
                                              0x00, 0x00, 0x01}; // GS: ground station
static const uint8_t MAC_RELIEF_VALVE[6] = {0x02, 0x52, 0x56,
                                            0x00, 0x00, 0x02}; // RV: relief valves
static const uint8_t MAC_FLOW_VALVE[6] = {0x02, 0x46, 0x4C,
                                          0x00, 0x00, 0x03}; // FL: flow valves
static const uint8_t MAC_SENSOR_GIGA[6] = {0x02, 0x53, 0x49,
                                           0x00, 0x00, 0x04}; // SI: sensor interface

static const uint8_t LNG_PRES_MASK = 0b1000000; // LNG Pressurant / Upstream
static const uint8_t LOX_PRES_MASK = 0b0100000; // LOX Pressurant / Upstream
// static const uint8_t GN2_VENT_MASK = 0b0010000; // GN2 Vent, currently removed
static const uint8_t LNG_FLOW_MASK = 0b0001000; // LNG Flow / Downstream
static const uint8_t LNG_VENT_MASK = 0b0000100; // LNG Vent
static const uint8_t LOX_FLOW_MASK = 0b0000010; // Lox Flow / Downstream
static const uint8_t LOX_VENT_MASK = 0b0000001; // LOX Vent

// PWM pins for valves
static const uint8_t LNG_PRES_PIN = 8;
static const uint8_t LOX_PRES_PIN = 6;
static const uint8_t LNG_FLOW_PIN = 10;
static const uint8_t LOX_FLOW_PIN = 9;
static const uint8_t LNG_VENT_PIN = 3;
static const uint8_t LOX_VENT_PIN = 5;

// Load cell pins and calibration factors
static const uint8_t LC_STAR_DOUT_PIN = 16;
static const uint8_t LC_STAR_CLK_PIN = 17;
static const uint8_t LC_CIRCLE_DOUT_PIN = 14;
static const uint8_t LC_CIRCLE_CLK_PIN = 15;
static const uint8_t LC_TRIANGLE_DOUT_PIN = 18;
static const uint8_t LC_TRIANGLE_CLK_PIN = 19;

static const float LC_STAR_CALIBRATION_FACTOR = -5600.0f;
static const float LC_CIRCLE_CALIBRATION_FACTOR = -5900.0f;
static const float LC_TRIANGLE_CALIBRATION_FACTOR = -6000.0f;

// Thermocouple I2C addresses
static const uint8_t TC_I2C_ADDRESS_1 = 0x67; // no shorts, not grounded
static const uint8_t TC_I2C_ADDRESS_2 = 0x66; // top short only
static const uint8_t TC_I2C_ADDRESS_3 = 0x65; // bottom short only
static const uint8_t TC_I2C_ADDRESS_4 = 0x64; // both shorted
static const uint8_t TC_I2C_ADDRESS_5 = 0x60; // GND address

// Pressure transducer pins
static const uint8_t PT_GN2_PIN = A0;
static const uint8_t PT_LOX_UPSTREAM_PIN = A6;
static const uint8_t PT_LNG_UPSTREAM_PIN = A4;
static const uint8_t PT_LOX_DOWNSTREAM_PIN = A5;
static const uint8_t PT_LNG_DOWNSTREAM_PIN = A3;
static const uint8_t PT_LOX_DOME_PIN = A2;
static const uint8_t PT_LNG_DOME_PIN = A1;

static const unsigned long OPEN_MILLIS = 500; // full power duration (ms)

// Timing values
static const int TELEMETRY_DELAY = 200;

#endif
