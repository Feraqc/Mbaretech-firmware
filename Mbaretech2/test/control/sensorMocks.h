#pragma once
#include <cassert>
using portMUX_TYPE = int;
constexpr int portMUX_INITIALIZER_UNLOCKED = 0;
using TaskHandle_t = void*;
constexpr int pdPASS = 1;
inline int lockDepth = 0, reads[2] = {}, filters[2] = {}, raw[2] = {100, 200};
inline int gpioReads = 0;
inline bool gpio[64] = {};
inline uint32_t nowMs = 10;
inline bool createSuccess = true;
inline volatile bool startSignal = false;
struct EndCycle {};
inline void portENTER_CRITICAL(int*) { ++lockDepth; }
inline void portEXIT_CRITICAL(int*) { --lockDepth; }
inline int digitalRead(int pin) { assert(lockDepth == 0); ++gpioReads; return gpio[pin]; }
inline int readLineSensorFront(int channel) {
    assert(lockDepth == 0);
    const int side = channel == ADC1_CHANNEL_2 ? 0 : 1;
    ++reads[side]; return raw[side];
}
inline bool checkLineSensora(int value) { ++filters[0]; return value <= THRESHOLD; }
inline bool checkLineSensorb(int value) { ++filters[1]; return value <= THRESHOLD; }
inline uint32_t millis() { return nowMs; }
inline TickType_t xTaskGetTickCount() { return nowMs; }
inline void vTaskDelayUntil(TickType_t*, TickType_t period) { assert(period>0 && lockDepth==0); throw EndCycle{}; }
inline int xTaskCreate(void(*)(void*), const char*, int, void*, int priority, TaskHandle_t* handle) {
    assert(priority == 2);
    if (!createSuccess) return 0;
    *handle = reinterpret_cast<void*>(1); return pdPASS;
}
