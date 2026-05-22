#include "pt.h"

bool initPT(PT* pt) {
  if (pt == nullptr) return false;
  if (pt->initialized) return false;
  pinMode(pt->pin, INPUT);
  pt->initialized = true;
  return true;
}

bool readPT(PT* pt, float* psi) {
  if (pt == nullptr || psi == nullptr) return false;
  if (!pt->initialized) return false;

  float denom = pt->voltageMax - pt->voltageMin;
  if (denom <= 0.0f) return false;

  float voltage = (analogRead(pt->pin) / pt->analogRange) * pt->voltageRange;
  float pressure = (voltage - pt->voltageMin) / denom * pt->range;

  if (pressure <= 0.0f) pressure = 0.0f;
  if (pressure > pt->range) pressure = pt->range;

  *psi = pressure;
  return true;
}
