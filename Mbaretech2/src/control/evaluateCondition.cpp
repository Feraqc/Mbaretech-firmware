#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/evaluateCondition.h"
namespace fsm {
bool evaluateCondition(fsm_defs::ConditionId id, const SensorSnapshot& sensors) {
    using fsm_defs::ConditionId;
    switch (id) {
    case ConditionId::NONE: return false;
    case ConditionId::START_ACTIVE: return sensors.startActive;
    case ConditionId::IR1_DETECTED: return sensors.ir[0];
    case ConditionId::IR2_DETECTED: return sensors.ir[1];
    case ConditionId::IR3_DETECTED: return sensors.ir[2];
    case ConditionId::IR4_DETECTED: return sensors.ir[3];
    case ConditionId::IR5_DETECTED: return sensors.ir[4];
    case ConditionId::IR6_DETECTED: return sensors.ir[5];
    case ConditionId::IR7_DETECTED: return sensors.ir[6];
    case ConditionId::LINE_LEFT_DETECTED: return sensors.line[0];
    case ConditionId::LINE_RIGHT_DETECTED: return sensors.line[1];
    case ConditionId::COUNT: break;
    }
    return false; // Startup validation rejects unknown IDs.
}
}
#endif
