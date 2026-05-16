#ifndef VENT_H
#define VENT_H

#include <Arduino.h>

struct Vent {
  const char    *name;
  uint8_t       mask;
  uint8_t       pin;
  bool          state;
  unsigned long lastOpened; 
};

// Initialize vent states and pins
void setupVents();

// Update the stored vent state (e.g., from network command)
void ventSetState(uint8_t newState);

// Get the currently stored vent state
uint8_t ventGetState();

// Update state tracking and opening timers
void ventUpdateStates();

// Apply the actual PWM values to the pins
void ventApplyVoltages();

int countVents();

#endif
