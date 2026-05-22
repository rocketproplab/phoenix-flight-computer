#ifndef LC_H
#define LC_H

#include "HX711.h"

typedef struct {
    const char* name;
    float calibration_factor;
    char dout;
    char clk;
    bool initialized;
    HX711 hx711;
} LC;

bool initLC(LC* lc);

bool readLC(LC* lc, float* lc_units);
#endif