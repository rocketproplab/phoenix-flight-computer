#include "vent.h"
#include "../constants.h"

// Define our three vents
static Vent vents[] = {
    //Name, Mask, Pin, State, lastOpened
  {"GN2_VENT", GN2_VENT_MASK, GN2_VENT_PIN, false, 0},
  {"LOX_VENT", LOX_VENT_MASK, LOX_VENT_PIN, false, 0},
  {"LNG_VENT", LNG_VENT_MASK, LNG_VENT_PIN, false, 0}
};

static const size_t NUM_VENTS = sizeof(vents) / sizeof(vents[0]);

// Flight computer's internal state for vents
static uint8_t g_ventState = 0;

void setupVents() {
    for (size_t i = 0; i < NUM_VENTS; i++) {
        pinMode(vents[i].pin, OUTPUT);
        analogWrite(vents[i].pin, 0);
        vents[i].state = false;
        vents[i].lastOpened = 0;
    }
}

void ventSetState(uint8_t newState) {
    if (newState == g_ventState) return;
    g_ventState = newState;
}

uint8_t ventGetState() {
    return g_ventState;
}

void ventUpdateStates() {
    for (size_t i = 0; i < NUM_VENTS; i++) {
        bool open = (g_ventState & vents[i].mask) != 0;
        if (open && !vents[i].state) {
            // Mark as opened right now
            vents[i].state = true;
            vents[i].lastOpened = millis();
        }
        else if (!open && vents[i].state) {
            // Close the vent
            vents[i].state = false;
        }
    }
}

void ventApplyVoltages() {
    // Full PWM for OPEN_MILLIS duration to actuate, then hold at ~30% (77/255) to prevent vent burn
    for (size_t i = 0; i < NUM_VENTS; i++) {
        if (vents[i].state) {
            unsigned long elapsed = millis() - vents[i].lastOpened;
            uint8_t pwm = (elapsed > OPEN_MILLIS) ? 77 : 255;
            analogWrite(vents[i].pin, pwm);
        }
        else {
            analogWrite(vents[i].pin, 0);
        }
    }
}

int countVents() {
    return NUM_VENTS;
}

