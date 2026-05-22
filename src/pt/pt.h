#ifndef PT_H
#define PT_H

#include <Arduino.h>

typedef struct {
  const char  *name; // "LOX/LNG/GN2 FLOW/VENT/PRES"
  uint8_t     pin;   // "A0 ..."
  float       range; // "max pressure in psi, 0 ~ <range> psi"
  float       voltageMin; // "V at 0 psi"
  float       voltageMax; // "V at <range> psi"
  float       analogRange; // "ADC range (0~1023 for Arduino, so 1023)"
  float       voltageRange; // "ADC full voltage (0~5V for Arduino, so 5)"
  bool        initialized;
} PT;

// Initialize PT state
bool initPT(PT* pt);

// Read PSI from PT
bool readPT(PT* pt, float* psi);

#endif
