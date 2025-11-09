#ifndef VALVE_H
#define VALVE_H

#include <Arduino.h>

struct Valve {
  const char    *name;
  uint8_t       mask;
  uint8_t       pin;
  bool          state;
  unsigned long lastOpened; 
};

// Initialize valve state
void setupValves();

// Set flight computer stored valve state
void valveSetState(uint8_t newState);

// Get flight computer stored valve state
uint8_t valveGetState();

// Update valve timing according to stored valve state
void valveUpdateStates();

// Update and set valve voltages
void valveApplyVoltages();

#endif