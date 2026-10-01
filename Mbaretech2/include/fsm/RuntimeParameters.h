#pragma once
#include "fsm/FSMRecipeTypes.h"
#include <stdint.h>

namespace fsm {
// El registro tiene capacidad fija: ninguna operación del control asigna memoria.
static constexpr unsigned MAX_RUNTIME_PARAMETERS = 64;
static constexpr unsigned MAX_PARAMETER_CHANGES = 12;
enum class ApplyPolicy : uint8_t { Immediate, NextStateEntry, NextStepEntry, NextRestart, StoppedOnly, MixedEntries };

struct ParameterMetadata {
    char id[72];
    char name[40];
    char group[24];
    const char* unit;
    int32_t defaultValue;
    int32_t minimum;
    int32_t maximum;
    int32_t step;
    ApplyPolicy policy;
    bool writable;
    ParameterId recipeId;
};
struct ParameterSnapshot {
    uint32_t revision = 0;
    int32_t values[MAX_RUNTIME_PARAMETERS] = {};
};
struct ParameterChange { char id[72]; int32_t value; };
struct ParameterResult {
    bool accepted = false;
    const char* error = nullptr;
    uint32_t revision = 0;
    ApplyPolicy effective = ApplyPolicy::NextStateEntry;
    int32_t previous[MAX_PARAMETER_CHANGES] = {};
};

class RuntimeParameters {
public:
    // Lock callbacks guard only a bounded copy/swap. ESP uses a portMUX;
    // host tests can omit them in their single-threaded harness.
    void setLock(void (*enter)(void*), void (*leave)(void*), void* context);
    bool initialize(const MachineRecipe& recipe);
    unsigned count() const { return count_; }
    const ParameterMetadata& metadata(unsigned index) const { return metadata_[index]; }
    int find(const char* id) const;
    void snapshot(ParameterSnapshot& output) const;
    void beginMachine();
    void entrySnapshot(ParameterSnapshot& output) const;
    ParameterResult set(uint32_t expectedRevision, const ParameterChange* changes,
                        unsigned count, bool running);
    ParameterResult reset(uint32_t expectedRevision, bool running);
    MotorCommand motor(const MotorCommand& compiled, const ParameterSnapshot& snapshot) const;
    uint32_t timer(const TriggerRecipe& compiled, const ParameterSnapshot& snapshot) const;
private:
    ParameterMetadata metadata_[MAX_RUNTIME_PARAMETERS] = {};
    int32_t values_[MAX_RUNTIME_PARAMETERS] = {};
    int32_t startValues_[MAX_RUNTIME_PARAMETERS] = {};
    unsigned count_ = 0;
    uint32_t revision_ = 0;
    void (*lock_)(void*) = nullptr;
    void (*unlock_)(void*) = nullptr;
    void* lockContext_ = nullptr;
    bool add(const ParameterDefinition& definition);
};
const char* applyPolicyName(ApplyPolicy policy);
} // namespace fsm
