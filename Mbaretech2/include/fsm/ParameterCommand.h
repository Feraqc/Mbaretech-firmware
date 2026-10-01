#pragma once
#include "fsm/RuntimeParameters.h"
#include <stddef.h>

namespace fsm {
enum class ParameterOperation : uint8_t { SchemaRequest, ValuesRequest, Set, Reset, StartSet };
struct ParameterCommand {
    ParameterOperation operation = ParameterOperation::SchemaRequest;
    uint32_t transaction = 0;
    uint32_t revision = 0;
    char machine[64] = {};
    bool startActive = false;
    ParameterChange changes[MAX_PARAMETER_CHANGES] = {};
    unsigned count = 0;
};
// Sólo acepta el subconjunto JSON del protocolo de comandos; no usa heap.
bool decodeParameterCommand(const char* text, ParameterCommand& output, const char*& error);
} // namespace fsm
