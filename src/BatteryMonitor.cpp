#include "BatteryMonitor.h"

bool BatteryMonitor::checkBattery(float voltage) {
    int voltageInt = roundf(voltage * 100.0f);
    if (triggered) {
        if (voltageInt >= resetThreshold) {
            triggered = false;
        }
    } else {
        if (voltageInt <= triggerThreshold) {
            triggered = true;
            return true;
        }
    }
    return false;
}

void BatteryMonitor::setState(bool oldState) {
    triggered = oldState;
}
