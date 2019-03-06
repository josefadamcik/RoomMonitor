#include "Arduino.h"

#ifndef _ROOM_MONITOR_STATE_
#define _ROOM_MONITOR_STATE_

struct RoomMonitorState {
   public:
    bool valid = false;  //1 byte
    bool triggered = false;  // 1 byte
    bool validWifi = false; //1 byte
    uint8_t channel;       // 1 byte,   4 in total
    uint8_t bssid[6];      // 6 bytes, 10 in total
    /**
     * We will set this to special know value before we store it into
     * persistent memory, before deep sleep. So after wakep we can
     * distinguish random junk in memory from our stored value.
     */
    uint32_t magic;  // 4bytes -> 14 in total
    uint16_t padding;  //  s2 byte,  16 in total
};

#endif