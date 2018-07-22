#ifndef BATTERY_MONITOR_h
#define BATTERY_MONITOR_h

#include <math.h> 

struct BatteryMonitorState {
    public:
        BatteryMonitorState(bool v) : triggered(v) {};
        const bool triggered;
};

class BatteryMonitor {
    public:
        /** 
         * @param trigger -> voltage in volts * 100 for trigger threshold
         * @param reset -> voltage in volts * 100 for reset threshold (must be higher than trigger)
         */ 
        BatteryMonitor(int trigger, int reset)
            : triggerThreshold(trigger), resetThreshold(reset), triggered(false) {};
        /**
         * Constructor for saved state.
         */ 
        BatteryMonitor(int trigger, int reset, const BatteryMonitorState& oldState) 
            : triggerThreshold(trigger), resetThreshold(reset), triggered(oldState.triggered) {};
        /** 
         * Checks the battery and lets the caller know, if low battery notification should be raised 
         * @param voltage - measured voltage
         * @return - true when notificatin should be raised
         */
        bool checkBattery(float voltage);
        BatteryMonitorState getState();
    private:
        const int triggerThreshold;
        const int resetThreshold;
        bool triggered;

};

#endif
