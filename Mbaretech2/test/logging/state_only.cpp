#include "states.cpp"
#include <cassert>
#include <cstring>
#include <iostream>
int main() {
    static_assert(IDLE == 0 && FORWARD == 1 && GIRO_U_R_LONG == 28, "Preserve legacy state IDs");
#define CHECK_STATE(name) assert(std::strcmp(stateName(name), #name) == 0);
    FIRMWARE_STATE_LIST(CHECK_STATE)
#undef CHECK_STATE
    assert(std::strcmp(stateName(static_cast<State>(999)), "DESCONOCIDO") == 0);
    changeState(TURN_LEFT_90);
    assert(currentState == TURN_LEFT_90);
    changeState(TURN_LEFT_90);
    changeState(IDLE);
    assert(currentState == IDLE);
    std::cout << "PASS: shared state IDs/names and transitions without logging\n";
}
