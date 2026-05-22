#ifndef ROLE_CONFIG_H
#define ROLE_CONFIG_H

// uncomment to select role:
#define PHOENIX_ROLE_SENSOR_TELEMETRY
// #define PHOENIX_ROLE_FLOW_VALVES
// #define PHOENIX_ROLE_RELIEF_VALVES

// no default role, enforce check

#if (defined(PHOENIX_ROLE_SENSOR_TELEMETRY) + \
     defined(PHOENIX_ROLE_FLOW_VALVES) + \
     defined(PHOENIX_ROLE_RELIEF_VALVES)) != 1
#error "Define exactly one Phoenix flight computer role."
#endif

#if defined(PHOENIX_ROLE_SENSOR_TELEMETRY)
#define PHOENIX_ROLE_HAS_SENSOR_TELEMETRY 1
#else
#define PHOENIX_ROLE_HAS_SENSOR_TELEMETRY 0
#endif

#if defined(PHOENIX_ROLE_FLOW_VALVES) || defined(PHOENIX_ROLE_RELIEF_VALVES)
#define PHOENIX_ROLE_HAS_VALVES 1
#else
#define PHOENIX_ROLE_HAS_VALVES 0
#endif

#endif
