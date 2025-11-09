#include "valve.h"
#include "../constants.h"

static Valve valves[] = {
  {"LNG_PRES", LNG_PRES_MASK, LNG_PRES_PIN, false, 0},
  {"LOX_PRES", LOX_PRES_MASK, LOX_PRES_PIN, false, 0},
  {"LNG_FLOW", LNG_FLOW_MASK, LNG_FLOW_PIN, false, 0},
  {"LOX_FLOW", LOX_FLOW_MASK, LOX_FLOW_PIN, false, 0}
};

static const size_t NUM_VALVES = sizeof(valves) / sizeof(valves[0]);

static uint8_t g_valveState = 0;

void setupValves() {
    for (size_t i = 0; i < NUM_VALVES; i++) {
        pinMode(valves[i].pin, OUTPUT);
        analogWrite(valves[i].pin, 0);
        valves[i].state = false;
        valves[i].lastOpened = 0;
    }
}

void valveSetState(uint8_t newState) {
    if (newState == g_valveState) return;
    g_valveState = newState;
}

uint8_t valveGetState() {
    return g_valveState;
}

void valveUpdateStates() {
    // Compare each bit with existing mask
    for (size_t i = 0; i<NUM_VALVES; i++) {
        bool open = (g_valveState & valves[i].mask) != 0;
        if (open && !valves[i].state) {
            // Open at max power first
            valves[i].state = true;
            valves[i].lastOpened = millis();
        }
        else if (!open && valves[i].state) {
            valves[i].state = false;
        }
    }
}

void valveApplyVoltages() {
    // Full PWM for OPEN_MILLIS duration, then hold at 30% to prevent valve burn
    for (size_t i = 0; i<NUM_VALVES; i++) {
        if (valves[i].state) {
            unsigned long elapsed = millis() - valves[i].lastOpened;
            uint8_t pwm = (elapsed > OPEN_MILLIS) ? 77 : 255;
            analogWrite(valves[i].pin, pwm);
        }
        else {
            analogWrite(valves[i].pin, 0);
        }
    }
}