#pragma once
#include <stdint.h>

// Normalized data only: this header deliberately has no Arduino/FreeRTOS imports.
// IR order is IR1..IR7; line order is front left/right; DIP order is A..E.
enum DipIndex { DIP_A, DIP_B, DIP_C, DIP_D, DIP_E };
struct SensorSnapshot {
    int rawLine[2] = {-1, -1};
    bool line[2] = {};
    bool ir[7] = {};
    bool dip[5] = {};
    bool startActive = false;
    // START is observed at readSensorSnapshot(), independently of acquisition.
    uint32_t startObservedAtMs = 0;
    uint32_t sampledAtMs = 0;
    bool valid = false;
};
