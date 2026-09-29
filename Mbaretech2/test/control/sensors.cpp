#include "sensorTasks.cpp"
#include <iostream>
int main() {
    assert(!readSensorSnapshot().valid);
#if ENABLE_LINE_SENSORS || ENABLE_IR_SENSORS || ENABLE_DIP_SWITCHES || ENABLE_RECIPE_FSM
    createSuccess=false; assert(!startSensorTask());
#else
    assert(startSensorTask() && sensorTaskHandle == nullptr);
#endif
    createSuccess=true; assert(startSensorTask()); assert(startSensorTask());
    gpio[IR2]=true; gpio[IR3]=false; gpio[IR4]=false;
    gpio[IR5]=true; gpio[IR6]=false;
    gpio[DIPA]=true; gpio[DIPB]=false; gpio[DIPC]=true; gpio[DIPD]=true; gpio[DIPE]=false;
#ifdef MBARETECH_2
    gpio[IR1]=false; gpio[IR7]=true;
#endif
    try { sensorReadTask(nullptr); } catch (EndCycle&) {}
    auto s=readSensorSnapshot();
    assert(s.valid && s.sampledAtMs==10);
    // START must reflect a new interrupt edge without another ADC/GPIO scan.
    assert(!s.startActive);
    const int previousGpioReads = gpioReads;
    startSignal = true;
    nowMs = 11;
    auto started = readSensorSnapshot();
    assert(started.startActive && started.sampledAtMs == s.sampledAtMs);
    assert(started.startObservedAtMs == 11 && s.startObservedAtMs == 10);
    assert(!s.startActive); // Existing consumer copies do not change.
    startSignal = false;
    assert(!readSensorSnapshot().startActive && gpioReads == previousGpioReads);
#if ENABLE_LINE_SENSORS
    assert(s.rawLine[0]==100 && s.rawLine[1]==200 && s.line[0] && !s.line[1]);
    assert(reads[0]==1 && reads[1]==1 && filters[0]==1 && filters[1]==1);
#else
    assert(s.rawLine[0]==-1 && s.rawLine[1]==-1 && !s.line[0] && !s.line[1]);
    assert(reads[0]==0 && reads[1]==0 && filters[0]==0 && filters[1]==0);
#endif
#if ENABLE_DIP_SWITCHES
    assert(s.dip[DIP_A] && !s.dip[DIP_B] && s.dip[DIP_C] && s.dip[DIP_D] && !s.dip[DIP_E]);
#else
    for (bool dip : s.dip) assert(!dip);
#endif
#if ENABLE_IR_SENSORS
    assert(s.ir[TOP_MID]);
#ifdef MBARETECH_2
    assert(!s.ir[SHORT_LEFT] && s.ir[TOP_LEFT] && !s.ir[TOP_RIGHT] && s.ir[SHORT_RIGHT]);
    assert(s.ir[SIDE_LEFT] && !s.ir[SIDE_RIGHT]);
#else
    assert(s.ir[SHORT_LEFT] && !s.ir[TOP_LEFT] && s.ir[TOP_RIGHT] && !s.ir[SHORT_RIGHT]);
    assert(!s.ir[SIDE_LEFT] && !s.ir[SIDE_RIGHT]);
#endif
#else
    for (bool ir : s.ir) assert(!ir);
#endif
#ifdef MBARETECH_2
    constexpr int irCount = 7;
#else
    constexpr int irCount = 5;
#endif
    assert(gpioReads == (ENABLE_IR_SENSORS ? irCount : 0) + (ENABLE_DIP_SWITCHES ? 5 : 0));
    raw[0]=-1; nowMs=11;
    try { sensorReadTask(nullptr); } catch (EndCycle&) {}
    assert(readSensorSnapshot().valid == !bool(ENABLE_LINE_SENSORS));
    assert(s.rawLine[0]==(ENABLE_LINE_SENSORS ? 100 : -1)); // Independent consumer copy.
    assert(lockDepth==0);
    std::cout << "PASS: publication, START latch, single acquisition/filter, polarity, DIP order, invalid ADC, creation failure\n";
}
