"""Compile real motor/line sensor code with hardware substitutes (MSVC)."""
from pathlib import Path
import argparse
import shutil
import subprocess
parser = argparse.ArgumentParser()
parser.add_argument('--motors-disabled', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
build = root / '.pio/hardware-host-tests'
(build / 'driver').mkdir(parents=True, exist_ok=True)
shutil.copy(root / 'include/firmwareConfig.h', build / 'firmwareConfig.h')
shutil.copy(root / 'include/motor.h', build / 'motor.h')
shutil.copy(root / 'src/sensors/lineSensor.cpp', build / 'lineSensor.cpp')
(build / 'Arduino.h').write_text('''#pragma once
#include <cstdint>
#include <algorithm>
constexpr int OUTPUT = 1;
inline int pins[64]{};
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { pins[pin] = value; }
inline long map(long v,long a,long b,long c,long d) {return (v-a)*(d-c)/(b-a)+c;}
#define constrain(v,a,b) std::min(std::max(int(v),int(a)),int(b))
''')
(build / 'driver/ledc.h').write_text('''#pragma once
using ledc_channel_t = int;
constexpr int LEDC_CHANNEL_0=0, LEDC_CHANNEL_1=1, LEDC_LOW_SPEED_MODE=0,
LEDC_TIMER_10_BIT=10, LEDC_TIMER_0=0, LEDC_AUTO_CLK=0, LEDC_INTR_DISABLE=0;
struct ledc_timer_config_t {int speed_mode,duty_resolution,timer_num,freq_hz,clk_cfg;};
struct ledc_channel_config_t {int gpio_num,speed_mode,channel,intr_type,timer_sel; unsigned duty;};
inline unsigned appliedDuty[2]{};
inline void ledc_timer_config(ledc_timer_config_t*) {}
inline void ledc_channel_config(ledc_channel_config_t*) {}
inline void ledc_set_duty(int,int ch,unsigned duty) {appliedDuty[ch]=duty;}
inline void ledc_update_duty(int,int) {}
''')
(build / 'globals.h').write_text('''#pragma once
#include <cstdint>
using adc1_channel_t=int; using adc2_channel_t=int;
constexpr int ADC_WIDTH=12, ADC1_CHANNEL_2=2, ADC1_CHANNEL_7=7,
ADC2_CHANNEL_8=8, ADC2_CHANNEL_9=9, ADC_ATTEN_DB_12=12, ESP_OK=0, THRESHOLD=145;
inline int raw=123, published=-1;
inline void adc1_config_width(int) {}
inline void adc1_config_channel_atten(int,int) {}
inline void adc2_config_channel_atten(int,int) {}
inline int adc1_get_raw(int) {return raw;}
inline int adc2_get_raw(int,int,int* value) {*value=raw;return ESP_OK;}
''')
(build / 'driver/adc.h').write_text('#pragma once\n')
(build / 'dataLogging.h').write_text('inline void loggingLineSample(int,int value) {published=value;}\n')
(build / 'test.cpp').write_text('''#include <cassert>
#include <climits>
#include "motor.h"
#include "lineSensor.cpp"
int main() {
 Motor motor(1,2,3,0); motor.begin();
 assert(motor.currentSpeed==0 && motor.ledc_channel.channel==0);
 motor.forward(50); assert(appliedDuty[0]==511 && pins[3]==1);
 motor.setSpeed(0); assert(appliedDuty[0]==0);
 motor.setSpeed(UINT_MAX); assert(appliedDuty[0]==990);
 motor.backward(100); assert(pins[2]==1 && pins[3]==0);
 motor.brake(); assert(appliedDuty[0]==0 && motor.currentSpeed==0 && motor.ledc_channel.duty==0 && pins[2]==0 && pins[3]==0);
 for(int i=0;i<6;++i) assert(!checkLineSensora(145));
 for(int i=0;i<1024;++i) assert(checkLineSensora(145));
 assert(!checkLineSensorb(145));
 assert(!checkLineSensora(146));
 for(int i=0;i<6;++i) assert(!checkLineSensora(0));
 assert(checkLineSensora(0));
 assert(readLineSensorFront(2)==123 && published==123);
}
''')
if args.motors_disabled:
    (build / 'test.cpp').write_text("""#include <cassert>
#include "motor.h"
int main() {
    Motor motor(1,2,3,0);
    pins[2]=7; pins[3]=7; appliedDuty[0]=123;
    motor.begin(); motor.forward(100); motor.backward(100); motor.setSpeed(100); motor.brake();
    assert(pins[2]==7 && pins[3]==7 && appliedDuty[0]==123 && motor.currentSpeed==0);
}
""")
vswhere=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe')
vs=subprocess.check_output([str(vswhere),'-latest','-products','*','-property','installationPath'],text=True).strip()
vcvars=Path(vs)/'VC/Auxiliary/Build/vcvars64.bat'
motor_flag = 0 if args.motors_disabled else 1
(build/'run.cmd').write_text(f'@echo off\ncall "{vcvars}" >nul\ncl /nologo /std:c++20 /EHsc /I. /DENABLE_LOGGING=1 /DENABLE_SERIAL=1 /DENABLE_MOTORS={motor_flag} /DENABLE_LINE_SENSORS=1 test.cpp /Fe:hardware-tests.exe && hardware-tests.exe\n')
subprocess.run(['cmd.exe','/d','/c','run.cmd'],cwd=build,check=True)
print('PASS: motor IO disabled' if args.motors_disabled else 'PASS: motor zero/clamp/brake and line persistence/reset/cache')
