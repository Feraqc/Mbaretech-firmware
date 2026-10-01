#include "fsm/RuntimeParameters.h"
#include <stdio.h>
#include <string.h>

namespace fsm {
const char* applyPolicyName(ApplyPolicy policy) {
    switch (policy) {
    case ApplyPolicy::Immediate: return "immediate";
    case ApplyPolicy::NextStateEntry: return "next_state_entry";
    case ApplyPolicy::NextStepEntry: return "next_step_entry";
    case ApplyPolicy::NextRestart: return "next_fsm_restart";
    case ApplyPolicy::StoppedOnly: return "stopped_only";
    case ApplyPolicy::MixedEntries: return "next_entries";
    }
    return "unknown";
}
void RuntimeParameters::setLock(void (*enter)(void*), void (*leave)(void*), void* context) {
    lock_ = enter; unlock_ = leave; lockContext_ = context;
}
bool RuntimeParameters::add(const ParameterDefinition& definition) {
    const char* id = definition.key;
    if (!id || !definition.name || !*id || definition.id == NO_PARAMETER ||
        definition.type != ParameterType::Integer ||
        (definition.unit != ParameterUnit::Percent && definition.unit != ParameterUnit::Milliseconds) ||
        static_cast<unsigned>(definition.policy) > static_cast<unsigned>(ParameterPolicy::StoppedOnly) ||
        (definition.access != ParameterAccess::Writable && definition.access != ParameterAccess::ReadOnly) ||
        count_ == MAX_RUNTIME_PARAMETERS || strlen(id) >= sizeof(metadata_[0].id) ||
        find(id) >= 0 || definition.step <= 0 ||
        definition.minimum > definition.maximum ||
        definition.defaultValue < definition.minimum ||
        definition.defaultValue > definition.maximum ||
        (int64_t(definition.defaultValue) - definition.minimum) % definition.step != 0) return false;
    for (unsigned i = 0; i < count_; ++i)
        if (metadata_[i].recipeId == definition.id) return false;
    ParameterMetadata& entry = metadata_[count_];
    snprintf(entry.id, sizeof(entry.id), "%s", id);
    snprintf(entry.name, sizeof(entry.name), "%s", definition.name);
    snprintf(entry.group, sizeof(entry.group), "%s", "RECIPE");
    entry.unit = definition.unit == ParameterUnit::Percent ? "%" : "ms";
    entry.defaultValue = definition.defaultValue;
    entry.minimum = definition.minimum; entry.maximum = definition.maximum;
    entry.step = definition.step;
    switch (definition.policy) {
    case ParameterPolicy::Immediate: entry.policy = ApplyPolicy::Immediate; break;
    case ParameterPolicy::NextStateEntry: entry.policy = ApplyPolicy::NextStateEntry; break;
    case ParameterPolicy::NextStepEntry: entry.policy = ApplyPolicy::NextStepEntry; break;
    case ParameterPolicy::NextMachineStart: entry.policy = ApplyPolicy::NextRestart; break;
    case ParameterPolicy::StoppedOnly: entry.policy = ApplyPolicy::StoppedOnly; break;
    }
    entry.writable = definition.access == ParameterAccess::Writable;
    entry.recipeId = definition.id;
    values_[count_] = startValues_[count_] = definition.defaultValue;
    ++count_;
    return true;
}
bool RuntimeParameters::initialize(const MachineRecipe& recipe) {
    count_ = 0; revision_ = 0;
    if (recipe.parameterCount && !recipe.parameters) return false;
    for (unsigned i = 0; i < recipe.parameterCount; ++i)
        if (!add(recipe.parameters[i])) return false;
    return true;
}
int RuntimeParameters::find(const char* id) const {
    if (!id) return -1;
    for (unsigned i = 0; i < count_; ++i)
        if (strcmp(metadata_[i].id, id) == 0) return static_cast<int>(i);
    return -1;
}
void RuntimeParameters::snapshot(ParameterSnapshot& output) const {
    if (lock_) lock_(lockContext_);
    output.revision = revision_;
    memcpy(output.values, values_, count_ * sizeof(values_[0]));
    if (unlock_) unlock_(lockContext_);
}
void RuntimeParameters::beginMachine() {
    if (lock_) lock_(lockContext_);
    for (unsigned i = 0; i < count_; ++i)
        if (metadata_[i].policy == ApplyPolicy::NextRestart)
            startValues_[i] = values_[i];
    if (unlock_) unlock_(lockContext_);
}
void RuntimeParameters::entrySnapshot(ParameterSnapshot& output) const {
    snapshot(output);
    // Los cambios NEXT_MACHINE_START quedan invisibles para el control hasta begin().
    for (unsigned i = 0; i < count_; ++i)
        if (metadata_[i].policy == ApplyPolicy::NextRestart)
            output.values[i] = startValues_[i];
}
ParameterResult RuntimeParameters::set(uint32_t expectedRevision,
                                       const ParameterChange* changes,
                                       unsigned count, bool running) {
    ParameterResult result{};
    if (!changes || !count || count > MAX_PARAMETER_CHANGES) {
        result.error = "invalid change count"; return result;
    }
    int indices[MAX_PARAMETER_CHANGES];
    for (unsigned i = 0; i < count; ++i) {
        indices[i] = find(changes[i].id);
        if (indices[i] < 0) { result.error = "unknown parameter"; return result; }
        const ParameterMetadata& entry = metadata_[indices[i]];
        if (!entry.writable || (entry.policy == ApplyPolicy::StoppedOnly && running)) {
            result.error = "parameter is not writable now"; return result;
        }
        if (changes[i].value < entry.minimum || changes[i].value > entry.maximum ||
            (int64_t(changes[i].value) - entry.minimum) % entry.step != 0) {
            result.error = "value outside allowed range"; return result;
        }
        for (unsigned j = 0; j < i; ++j)
            if (indices[j] == indices[i]) { result.error = "duplicate parameter"; return result; }
        if (i == 0) result.effective = entry.policy;
        else if (result.effective != entry.policy) result.effective = ApplyPolicy::MixedEntries;
    }
    // La validación se completa antes del bloqueo; se publica el conjunto entero.
    if (lock_) lock_(lockContext_);
    if (revision_ != expectedRevision) {
        result.revision = revision_; result.error = "revision conflict";
    } else {
        for (unsigned i = 0; i < count; ++i) result.previous[i] = values_[indices[i]];
        for (unsigned i = 0; i < count; ++i) values_[indices[i]] = changes[i].value;
        result.revision = ++revision_; result.accepted = true;
    }
    if (unlock_) unlock_(lockContext_);
    return result;
}
ParameterResult RuntimeParameters::reset(uint32_t expectedRevision, bool running) {
    ParameterResult result{};
    result.effective = ApplyPolicy::MixedEntries;
    for (unsigned i = 0; i < count_; ++i)
        if (metadata_[i].policy == ApplyPolicy::StoppedOnly && running) {
            result.error = "parameter is not writable now"; return result;
        }
    if (lock_) lock_(lockContext_);
    if (revision_ != expectedRevision) {
        result.revision = revision_; result.error = "revision conflict";
    } else {
        for (unsigned i = 0; i < count_; ++i) values_[i] = metadata_[i].defaultValue;
        result.revision = ++revision_; result.accepted = true;
    }
    if (unlock_) unlock_(lockContext_);
    return result;
}
MotorCommand RuntimeParameters::motor(const MotorCommand& compiled,
                                      const ParameterSnapshot& snapshot) const {
    MotorCommand result = compiled;
    for (unsigned i = 0; i < count_; ++i) {
        if (compiled.leftParameter == metadata_[i].recipeId)
            result.left_pct = static_cast<int8_t>(snapshot.values[i]);
        if (compiled.rightParameter == metadata_[i].recipeId)
            result.right_pct = static_cast<int8_t>(snapshot.values[i]);
    }
    return result;
}
uint32_t RuntimeParameters::timer(const TriggerRecipe& compiled,
                                  const ParameterSnapshot& snapshot) const {
    for (unsigned i = 0; i < count_; ++i)
        if (compiled.timerParameter == metadata_[i].recipeId)
            return static_cast<uint32_t>(snapshot.values[i]);
    return compiled.timerMs;
}
} // namespace fsm
