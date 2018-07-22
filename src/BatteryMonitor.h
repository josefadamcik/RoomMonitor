#ifndef BATTERY_MONITOR_h
#define BATTERY_MONITOR_h

#include <math.h> 
#include "Arduino.h"

struct BatteryMonitorState {
    public:
        const bool triggered;
        /** 
         * We will set this to special know value before we store it into persistent memory, before deep sleep.
         * So after wakep we can distinguish random junk in memory from our stored value.
         */
        uint32_t magic; 
        BatteryMonitorState(bool v) : triggered(v) {};
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
        void setState(const BatteryMonitorState& oldState);
    private:
        const int triggerThreshold;
        const int resetThreshold;
        bool triggered;

};

#endif
