#ifndef CONSTANTS_H
#define CONSTANTS_H

static const uint8_t LNG_PRES_MASK = 0b1000000; // LNG Pressurant / Upstream Valve
static const uint8_t LOX_PRES_MASK = 0b0100000; // LOX Pressurant / Upstream Valve
static const uint8_t LNG_FLOW_MASK = 0b0001000; // LNG Flow / Downstream Valve
static const uint8_t LOX_FLOW_MASK = 0b0000010; // Lox Flow / Downstream Valve

// PWM pins for valves
static const uint8_t LNG_PRES_PIN = 8;
static const uint8_t LOX_PRES_PIN = 6;
static const uint8_t LNG_FLOW_PIN = 10;
static const uint8_t LOX_FLOW_PIN = 9;

// Vent State Masks (Assumes an independent vent state byte)
static const uint8_t GN2_VENT_MASK = 0b00100000; 
static const uint8_t LOX_VENT_MASK = 0b00000100;
static const uint8_t LNG_VENT_MASK = 0b00000001;


// Vent PWM Pins (Example pins)
static const uint8_t GN2_VENT_PIN = 3; 
static const uint8_t LOX_VENT_PIN = 4;
static const uint8_t LNG_VENT_PIN = 5;

static const unsigned long OPEN_MILLIS = 500; // full power duration (ms)

// Timing values
static const int TELEMETRY_DELAY = 200;

#endif