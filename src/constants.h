#ifndef CONSTANTS_H
#define CONSTANTS_H

static const uint8_t LNG_PRES_MASK = 0b1000000; // LNG Pressurant / Upstream
static const uint8_t LOX_PRES_MASK = 0b0100000; // LOX Pressurant / Upstream
static const uint8_t LNG_FLOW_MASK = 0b0001000; // LNG Flow / Downstream
static const uint8_t LOX_FLOW_MASK = 0b0000010; // Lox Flow / Downstream

// PWM pins for valves
static const uint8_t LNG_PRES_PIN = 8;
static const uint8_t LOX_PRES_PIN = 6;
static const uint8_t LNG_FLOW_PIN = 10;
static const uint8_t LOX_FLOW_PIN = 9;

static const unsigned long OPEN_MILLIS = 500; // full power duration (ms)

#endif