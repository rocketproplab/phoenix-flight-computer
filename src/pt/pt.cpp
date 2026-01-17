#include "pt.h"

static PT pts[] = {
  // name, pin, range, voltageMin, voltageMax, analogRange, voltageRange
  { "GN2", A0, 5000.0, 0.5, 4.5, 1023.0, 5.0 },
  { "LOX-UPSTREAM", A6, 1000.0, 1.0, 5.0, 1023.0, 5.0 },
  { "LNG-UPSTREAM", A4, 1000.0, 1.0, 5.0, 1023.0, 5.0 },
  { "LOX-DOWNSTREAM", A5, 1000.0, 1.0, 5.0, 1023.0, 5.0 },
  { "LNG-DOWNSTREAM", A3, 1000.0, 1.0, 5.0, 1023.0, 5.0 },
  { "LOX-DOME", A2, 1500.0, 0.5, 4.5, 1023.0, 5.0 },
  { "LNG-DOME", A1, 1500.0, 0.5, 4.5, 1023.0, 5.0 },
};

static const size_t NUM_PTS = sizeof(pts) / sizeof(pts[0]);

// quick sanity check
static inline bool valid(uint8_t id) {
  return id < NUM_PTS;
}

void setupPTs() {
  for (uint8_t i = 0; i < NUM_PTS; i++) {
    pinMode(pts[i].pin, INPUT);
  }
}

uint8_t countPTs() {
  return NUM_PTS;
}

const PT* getPTInfo(uint8_t id) {
  // DANGER!!! really should never trigger this...
  if (!valid(id)) {
    Serial.println("ERROR: referencing unexisting PT, returning 0 as default.");
    return &pts[0];
  }
  return &pts[id];
}

double readPT(uint8_t id) {
  if (!valid(id)) return 0.0;
  const PT& p = pts[id];

  double v = (analogRead(p.pin) / p.analogRange) * p.voltageRange;
  double denom = (p.voltageMax - p.voltageMin);
  if (denom <= 0.0) return 0.0; // why...

  double pres = (v - p.voltageMin)/denom * p.range;

  if (pres <= 0.0) return 0.0; 
  if (pres > p.range) return p.range; // clamp, just in case
  return pres;
}