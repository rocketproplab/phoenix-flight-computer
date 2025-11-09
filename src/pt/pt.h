#ifndef PT_H
#define PT_H

#include <Arduino.h>

struct PT {
  const char  *name; // "LOX/LNG/GN2 FLOW/VENT/PRES"
  uint8_t     pin;   // "A0 ..."
  double      range; // "pressure range, 0 ~ <range> psi"
  double      voltageMin; // "V at 0 psi"
  double      voltageMax; // "V at <range> psi"
  double      analogRange; // "ADC range (0~1023 for Arduino, so 1023)"
  double      voltageRange; // "ADC full voltage (0~5V for Arduino, so 5)"
};

// Initialize PT state
void setupPTs();

// Return total number of PTs
uint8_t countPTs();

// Get read-only PT descriptor for debugging
const PT* getPTInfo(uint8_t id);

// Read PSI from PT
double readPT(uint8_t id);

#endif