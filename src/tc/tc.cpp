#include "tc.h"

bool initTC(TC* tc) {
    if (tc == nullptr) return false;
    // prevent double initialization
    if (tc->initialized) return false;

    // filter thermocouple types
    if (!(tc->type == 'k' || tc->type == 'K' || tc->type == 't' || tc->type == 'T')) return false;
    // initialize driver state
    if (!tc->mcp.begin(tc->I2C_ADDRESS)) return false;
    tc->mcp.setAmbientResolution(RES_ZERO_POINT_0625);
    tc->mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
    if (tc->type == 'k' || tc->type == 'K')
        tc->mcp.setThermocoupleType(MCP9600_TYPE_K);
    else
        tc->mcp.setThermocoupleType(MCP9600_TYPE_T);
    tc->mcp.setFilterCoefficient(3);
    tc->mcp.enable(true);

    // initialize wrapper state
    tc->initialized = true;

    return true;
}

bool readTC(TC* tc, float* celsius) {
    if (tc == nullptr || celsius == nullptr) return false;
    if (!tc->initialized) return false;
    *celsius = tc->mcp.readThermocouple();
    return true;
}
