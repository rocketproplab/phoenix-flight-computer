#ifndef TC_H
#define TC_H

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

typedef struct {
    char type;
    char I2C_ADDRESS;
    bool initialized;
    Adafruit_MCP9600 mcp;
} TC;

bool initTC(TC* tc);

// status return + output parameter
bool readTC(TC* tc, float* celsius);


#endif
