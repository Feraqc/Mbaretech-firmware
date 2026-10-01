#pragma once
#include <cstdint>
#include <algorithm>
// Record every hardware operation, including setup, to verify the master gate.
inline unsigned ioWrites = 0;
inline int pins[64] = {};
constexpr int OUTPUT = 1;
inline void pinMode(int, int) { ++ioWrites; }
inline void digitalWrite(int pin, int value) { ++ioWrites; pins[pin] = value; }
inline long map(long x, long a, long b, long c, long d) {
    return (x - a) * (d - c) / (b - a) + c;
}
template<class T> T constrain(T x, T low, T high) {
    return std::min(std::max(x, low), high);
}
