#ifndef BATTERY_MONITOR_h
#define BATTERY_MONITOR_h

#include <math.h> 
#include "Arduino.h"
#include "RoomMonitorState.h"


class BatteryMonitor {
    public:
        /** 
         * @param trigger -> voltage in volts * 100 for trigger threshold
         * @param reset -> voltage in volts * 100 for reset threshold (must be higher than trigger)
         */
     BatteryMonitor(int trigger, int reset)
         : triggered(false), triggerThreshold(trigger), resetThreshold(reset) {};
     /**
      * Constructor for saved state.
      */
     BatteryMonitor(int trigger, int reset, bool oldState)
         : triggered(oldState), 
           triggerThreshold(trigger),
           resetThreshold(reset)
           {};
     /**
      * Checks the battery and lets the caller know, if low battery notification
      * should be raised
      * @param voltage - measured voltage
      * @return - true when notificatin should be raised
      */
     bool checkBattery(float voltage);
     void setState(bool triggered);
     bool triggered;
    private:
        const int triggerThreshold;
        const int resetThreshold;
  

};

#endif
