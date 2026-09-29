#pragma once
#include <stdint.h>

// Records signed output while compiling the real Drive adapter unchanged.
class Motor {
public:
    int command = 0;
    unsigned movementCalls = 0;
    void begin() { command = 0; }
    void forward(uint32_t speed) { command = static_cast<int>(speed); ++movementCalls; }
    void backward(uint32_t speed) { command = -static_cast<int>(speed); ++movementCalls; }
    void brake() { command = 0; }
};
