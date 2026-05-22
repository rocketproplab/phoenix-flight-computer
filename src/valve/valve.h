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

bool initValve(Valve* valve);

void valveOpen(Valve* valve);

void valveClose(Valve* valve);

void valveSetOpen(Valve* valve, bool open);

void valveApplyState(Valve* valve);

#endif
