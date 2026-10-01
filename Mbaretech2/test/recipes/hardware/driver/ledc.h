#pragma once
#include <Arduino.h>
using ledc_channel_t = int;
constexpr int LEDC_LOW_SPEED_MODE = 0, LEDC_TIMER_10_BIT = 10;
constexpr int LEDC_TIMER_0 = 0, LEDC_AUTO_CLK = 0, LEDC_INTR_DISABLE = 0;
constexpr int LEDC_CHANNEL_0 = 0, LEDC_CHANNEL_1 = 1;
struct ledc_timer_config_t {
    int speed_mode, duty_resolution, timer_num, freq_hz, clk_cfg;
};
struct ledc_channel_config_t {
    int gpio_num, speed_mode, channel, intr_type, timer_sel;
    unsigned duty;
};
inline unsigned appliedDuty[2] = {};
inline void ledc_timer_config(const ledc_timer_config_t*) { ++ioWrites; }
inline void ledc_channel_config(const ledc_channel_config_t*) { ++ioWrites; }
inline void ledc_set_duty(int, int channel, unsigned duty) {
    ++ioWrites; appliedDuty[channel] = duty;
}
inline void ledc_update_duty(int, int) { ++ioWrites; }
