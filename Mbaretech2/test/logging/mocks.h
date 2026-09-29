#pragma once
#include <string>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <deque>
#include <vector>
#include <cassert>
#include <iostream>
class String {
    std::string value;
public:
    String() = default;
    String(const char* s): value(s) {}
    String(const std::string& s): value(s) {}
    String(int n): value(std::to_string(n)) {}
    String(unsigned int n): value(std::to_string(n)) {}
    String(long n): value(std::to_string(n)) {}
    const char* c_str() const { return value.c_str(); }
    size_t length() const { return value.size(); }
    char operator[](size_t i) const { return value[i]; }
    long toInt() const { return std::stol(value); }
    bool operator==(const char* s) const { return value == s; }
    String& operator+=(const String& s) { value += s.value; return *this; }
    friend String operator+(const String& a, const String& b) { return a.value + b.value; }
};
using adc1_channel_t = int;
constexpr int LINE_FRONT_LEFT = 2, LINE_FRONT_RIGHT = 7;
using TickType_t = uint32_t;
using TaskHandle_t = void*;
constexpr int pdTRUE = 1, pdPASS = 1;
inline void resetElapsedTime() {}
inline uint32_t fakeTime = 0;
inline uint32_t millis() { return fakeTime; }
inline uint32_t pdMS_TO_TICKS(uint32_t n) { return n; }
inline void vTaskDelay(uint32_t) {}
inline int taskCreateCount = 0;
inline int xTaskCreate(void(*)(void*), const char*, int, void*, int, void*) { ++taskCreateCount; return pdPASS; }
struct Queue { size_t limit, size; std::deque<std::vector<char>> items; };
using QueueHandle_t = Queue*;
inline QueueHandle_t xQueueCreate(int count, size_t size) { return new Queue{size_t(count), size, {}}; }
inline int xQueueSend(Queue* q, const void* value, int wait) {
    assert(wait == 0);
    if (q->items.size() == q->limit) return 0;
    const char* bytes = static_cast<const char*>(value);
    q->items.emplace_back(bytes, bytes + q->size);
    return pdTRUE;
}
inline int xQueueReceive(Queue* q, void* value, int wait) {
    assert(wait == 0);
    if (q->items.empty()) return 0;
    memcpy(value, q->items.front().data(), q->size);
    q->items.pop_front();
    return pdTRUE;
}
inline void xQueueReset(Queue* q) { q->items.clear(); }
class IMU {
public:
    int currentAngle = 0;
    void begin() {}
    void getData() {}
    int getInitError() const { return -3; }
    bool isReady() const { return false; }
    bool hasYaw() const { return false; }
};

