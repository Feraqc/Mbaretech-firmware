#include "telemetry/TelemetryProtocol.h"
#include <stdio.h>
#include <string.h>

const char* telemetryTypeName(TelemetryType type) {
    switch (type) {
    case TelemetryType::Hello: return "hello";
    case TelemetryType::Heartbeat: return "heartbeat";
    case TelemetryType::StartChanged: return "start_changed";
    case TelemetryType::SensorSnapshot: return "sensors";
    case TelemetryType::IrChanged: return "ir_changed";
    case TelemetryType::LineChanged: return "line_changed";
    case TelemetryType::MotorCommand: return "motor";
    case TelemetryType::FsmState: return "fsm_status";
    case TelemetryType::FsmTransition: return "fsm_transition";
    case TelemetryType::FsmStepTransition: return "fsm_step";
    case TelemetryType::TaskTiming: return "task_timing";
    case TelemetryType::TelemetryStats: return "stats";
    case TelemetryType::ImuSample: return "imu";
    case TelemetryType::Error: return "error";
    case TelemetryType::Log: return "log";
    case TelemetryType::ParamSchema: return "param_schema";
    case TelemetryType::ParamValues: return "param_values";
    case TelemetryType::ParamAck: return "param_ack";
    case TelemetryType::ParamChanged: return "param_changed";
    case TelemetryType::StartAck: return "start_ack";
    }
    return "unknown";
}

// Escape into caller-owned storage; no allocation is permitted in the service.
static bool quoted(const char* input, char* output, size_t capacity) {
    size_t used = 0;
    if (capacity < 3) return false;
    output[used++] = '"';
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(input); *p; ++p) {
        const char* escape = *p == '"' ? "\\\"" : *p == '\\' ? "\\\\" :
                             *p == '\n' ? "\\n" : *p == '\r' ? "\\r" : nullptr;
        if (escape) {
            if (used + 2 >= capacity) return false;
            output[used++] = escape[0]; output[used++] = escape[1];
        } else if (*p < 0x20) {
            if (used + 6 >= capacity) return false;
            const int n = snprintf(output + used, capacity - used, "\\u%04x", *p);
            if (n != 6) return false;
            used += 6;
        } else {
            if (used + 1 >= capacity) return false;
            output[used++] = static_cast<char>(*p);
        }
    }
    output[used++] = '"'; output[used] = '\0';
    return true;
}

bool formatTelemetry(const TelemetryEvent& event, char* output, size_t capacity) {
    if (!output || !capacity) return false;
    char machine[72], state[72], next[72], condition[200];
    if (!quoted(event.machine, machine, sizeof(machine)) ||
        !quoted(event.state, state, sizeof(state)) ||
        !quoted(event.next, next, sizeof(next)) ||
        !quoted(event.condition, condition, sizeof(condition))) return false;
    const SensorSnapshot& s = event.sensors;
    char payload[400];
    int n = -1;
    switch (event.type) {
    case TelemetryType::Hello:
        n = snprintf(payload, sizeof(payload), "\"protocol\":1,\"schema\":2,\"machine\":%s,\"paramSchema\":%u,\"paramRevision\":%lu,\"bootId\":%lu",
            machine,(unsigned)event.paramCount,(unsigned long)event.paramRevision,
            (unsigned long)event.extra); break;
    case TelemetryType::Heartbeat:
        n = snprintf(payload, sizeof(payload), "\"uptimeMs\":%lu,\"wifiReady\":%s,\"wifiRssi\":%ld",
            (unsigned long)event.value,event.extra ? "true" : "false",(long)event.left); break;
    case TelemetryType::StartChanged:
        n = snprintf(payload, sizeof(payload), "\"start\":%s", event.value ? "true" : "false"); break;
    case TelemetryType::SensorSnapshot:
        n = snprintf(payload, sizeof(payload), "\"start\":%s,\"valid\":%s,\"sampledAtMs\":%lu,\"lineThreshold\":%lu,\"ir\":[%d,%d,%d,%d,%d,%d,%d],\"line\":[%d,%d],\"lineRaw\":[%d,%d]",
            s.startActive ? "true" : "false", s.valid ? "true" : "false", (unsigned long)s.sampledAtMs, (unsigned long)event.value,
            s.ir[0],s.ir[1],s.ir[2],s.ir[3],s.ir[4],s.ir[5],s.ir[6],
            s.line[0],s.line[1],s.rawLine[0],s.rawLine[1]); break;
    case TelemetryType::IrChanged:
    case TelemetryType::LineChanged:
        n = snprintf(payload, sizeof(payload), "\"index\":%lu,\"detected\":%s,\"raw\":%ld",
            (unsigned long)event.value, event.extra ? "true" : "false", (long)event.left); break;
    case TelemetryType::MotorCommand:
        n = snprintf(payload, sizeof(payload), "\"left\":%ld,\"right\":%ld,\"source\":%s,\"paramRevision\":%lu",
            (long)event.left,(long)event.right,state,(unsigned long)event.paramRevision); break;
    case TelemetryType::FsmState:
        n = snprintf(payload, sizeof(payload), "\"machine\":%s,\"state\":%s,\"step\":%ld,\"elapsedMs\":%lu,\"stepElapsedMs\":%lu,\"running\":%s,\"reason\":%s,\"paramRevision\":%lu",
            machine,state,(long)event.step,(unsigned long)event.elapsedMs,(unsigned long)event.extra,event.value ? "true" : "false",condition,(unsigned long)event.paramRevision); break;
    case TelemetryType::FsmTransition:
        n = snprintf(payload, sizeof(payload), "\"machine\":%s,\"from\":%s,\"to\":%s,\"condition\":%s,\"timerMs\":%lu,\"elapsedMs\":%lu,\"paramRevision\":%lu",
            machine,state,next,condition,(unsigned long)event.timerMs,(unsigned long)event.elapsedMs,(unsigned long)event.paramRevision); break;
    case TelemetryType::FsmStepTransition:
        n = snprintf(payload, sizeof(payload), "\"machine\":%s,\"state\":%s,\"step\":%ld,\"nextStep\":%ld,\"condition\":%s,\"timerMs\":%lu,\"elapsedMs\":%lu,\"paramRevision\":%lu",
            machine,state,(long)event.step,(long)event.nextStep,condition,(unsigned long)event.timerMs,(unsigned long)event.elapsedMs,(unsigned long)event.paramRevision); break;
    case TelemetryType::TaskTiming:
        n = snprintf(payload, sizeof(payload), "\"task\":%s,\"executionUs\":%lu,\"gapUs\":%lu",
            state,(unsigned long)event.value,(unsigned long)event.extra); break;
    case TelemetryType::TelemetryStats:
        n = snprintf(payload, sizeof(payload), "\"dropped\":%lu,\"serialDropped\":%lu,\"bleDropped\":%lu,\"wifiDropped\":%lu",
            (unsigned long)event.value,(unsigned long)event.extra,(unsigned long)event.left,(unsigned long)event.right); break;
    case TelemetryType::ImuSample:
        n = snprintf(payload, sizeof(payload), "\"yaw\":%ld,\"valid\":%s",
            (long)event.left,event.extra ? "true" : "false"); break;
    case TelemetryType::Error:
    case TelemetryType::Log:
        n = snprintf(payload, sizeof(payload), "\"message\":%s", condition); break;
    case TelemetryType::ParamSchema: {
        const auto* p = event.parameter;
        char parameterName[96] = {};
        if (p && !quoted(p->name, parameterName, sizeof(parameterName))) return false;
        n = p ? snprintf(payload, sizeof(payload),
            "\"paramSchema\":1,\"revision\":%lu,\"index\":%u,\"count\":%u,\"parameterId\":%u,\"id\":\"%s\",\"name\":%s,\"group\":\"%s\",\"unit\":\"%s\",\"valueType\":\"int32\",\"default\":%ld,\"min\":%ld,\"max\":%ld,\"step\":%ld,\"applyPolicy\":\"%s\",\"writable\":%s",
            (unsigned long)event.paramRevision,event.paramIndex,event.paramCount,
            (unsigned)p->recipeId,p->id,parameterName,p->group,p->unit,
            (long)p->defaultValue,(long)p->minimum,(long)p->maximum,(long)p->step,
            fsm::applyPolicyName(p->policy),p->writable ? "true" : "false") :
            snprintf(payload, sizeof(payload), "\"paramSchema\":1,\"revision\":%lu,\"index\":0,\"count\":0",
                (unsigned long)event.paramRevision);
        break;
    }
    case TelemetryType::ParamValues:
        n = event.parameter ? snprintf(payload, sizeof(payload),
            "\"revision\":%lu,\"index\":%u,\"count\":%u,\"id\":\"%s\",\"value\":%ld",
            (unsigned long)event.paramRevision,event.paramIndex,event.paramCount,event.parameter->id,(long)event.paramValue) :
            snprintf(payload, sizeof(payload), "\"revision\":%lu,\"index\":0,\"count\":0",
                (unsigned long)event.paramRevision);
        break;
    case TelemetryType::ParamAck:
        n = snprintf(payload, sizeof(payload),
            "\"transaction\":%lu,\"revision\":%lu,\"status\":\"%s\",\"effective\":%s,\"error\":%s",
            (unsigned long)event.paramTransaction,(unsigned long)event.paramRevision,
            event.paramAccepted ? "accepted" : "rejected",next,condition);
        break;
    case TelemetryType::ParamChanged:
        n = snprintf(payload, sizeof(payload),
            "\"transaction\":%lu,\"revision\":%lu,\"id\":\"%s\",\"old\":%ld,\"value\":%ld",
            (unsigned long)event.paramTransaction,(unsigned long)event.paramRevision,
            event.parameter ? event.parameter->id : "",(long)event.paramPrevious,(long)event.paramValue);
        break;
    case TelemetryType::StartAck:
        n = snprintf(payload, sizeof(payload),
            "\"transaction\":%lu,\"status\":\"accepted\",\"active\":%s,\"source\":\"remote\"",
            (unsigned long)event.paramTransaction,event.value ? "true" : "false");
        break;
    }
    if (n < 0 || size_t(n) >= sizeof(payload)) return false;
    const int written = snprintf(output, capacity,
        "{\"type\":\"%s\",\"seq\":%lu,\"t\":%lu,\"us\":%llu,%s}",
        telemetryTypeName(event.type), (unsigned long)event.sequence,
        (unsigned long)(event.atUs / 1000), (unsigned long long)event.atUs, payload);
    return written > 0 && size_t(written) < capacity;
}
