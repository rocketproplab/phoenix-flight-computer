#include "lc.h"

bool initLC(LC* lc){
    if (lc == nullptr) return false;
    if (lc->initialized) return false;
    lc->hx711.begin(lc->dout, lc->clk);
    lc->hx711.set_scale(lc->calibration_factor);
    lc->hx711.tare();
    lc->initialized = true;
    return true;
}

bool readLC(LC* lc, float* lc_units) {
    if (lc == nullptr || lc_units == nullptr) return false;
    if (!lc->initialized) return false;

    *lc_units = lc->hx711.get_units();
    return true;
}
