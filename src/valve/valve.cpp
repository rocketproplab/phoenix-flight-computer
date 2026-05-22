#include "valve.h"
#include "../constants.h"

bool initValve(Valve* valve) {
    if (valve == nullptr) return false;
    pinMode(valve->pin, OUTPUT);
    analogWrite(valve->pin, 0);
    valve->state = false;
    valve->lastOpened = 0;
    return true;
}

void valveOpen(Valve* valve) {
    if (valve == nullptr) return;
    if (valve->state) return;
    valve->state = true;
    valve->lastOpened = millis();
    analogWrite(valve->pin, 255);
}

void valveClose(Valve* valve) {
    if (valve == nullptr) return;
    if (!valve->state) return;
    valve->state = false;
    analogWrite(valve->pin, 0);
}

void valveSetOpen(Valve* valve, bool open) {
    if (open) valveOpen(valve);
    else valveClose(valve);
}

void valveApplyState(Valve* valve) {
    if (valve == nullptr) return;
    if (valve->state) {
        unsigned long elapsed = millis() - valve->lastOpened;
        uint8_t pwm = (elapsed > OPEN_MILLIS) ? 77 : 255;
        analogWrite(valve->pin, pwm);
    }
    else {
        analogWrite(valve->pin, 0);
    }
}
