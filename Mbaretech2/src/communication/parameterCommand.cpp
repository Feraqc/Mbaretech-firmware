#include "fsm/ParameterCommand.h"
#include <ctype.h>
#include <limits.h>
#include <string.h>

namespace fsm {
namespace {
struct Reader {
    const char* p;
    void space() { while (*p && isspace(static_cast<unsigned char>(*p))) ++p; }
    bool take(char ch) { space(); if (*p != ch) return false; ++p; return true; }
    bool string(char* out, size_t capacity) {
        if (!take('"')) return false;
        size_t n = 0;
        while (*p && *p != '"') {
            // IDs y claves no necesitan escapes; rechazarlos evita ambigüedad.
            if (*p == '\\' || static_cast<unsigned char>(*p) < 0x20 || n + 1 >= capacity) return false;
            out[n++] = *p++;
        }
        if (*p++ != '"') return false;
        out[n] = '\0';
        return true;
    }
    bool number(int32_t& value) {
        space(); bool negative = false;
        if (*p == '-') { negative = true; ++p; }
        if (!isdigit(static_cast<unsigned char>(*p))) return false;
        int64_t n = 0;
        do { n = n * 10 + (*p++ - '0'); if (n > INT32_MAX + int64_t(1)) return false; }
        while (isdigit(static_cast<unsigned char>(*p)));
        if (*p == '.' || *p == 'e' || *p == 'E') return false;
        n = negative ? -n : n;
        if (n < INT32_MIN || n > INT32_MAX) return false;
        value = static_cast<int32_t>(n); return true;
    }
    bool boolean(bool& value) {
        space();
        if (strncmp(p, "true", 4) == 0) { p += 4; value = true; return true; }
        if (strncmp(p, "false", 5) == 0) { p += 5; value = false; return true; }
        return false;
    }
};
bool change(Reader& r, ParameterChange& out) {
    if (!r.take('{')) return false;
    bool id = false, value = false;
    for (;;) {
        char key[24]; if (!r.string(key, sizeof(key)) || !r.take(':')) return false;
        if (strcmp(key, "id") == 0 && !id) {
            if (!r.string(out.id, sizeof(out.id))) return false;
            for (const char* p = out.id; *p; ++p)
                if (!isalnum(static_cast<unsigned char>(*p)) && *p != '_' && *p != '.') return false;
            id = out.id[0] != '\0';
        } else if (strcmp(key, "value") == 0 && !value) {
            if (!r.number(out.value)) return false;
            value = true;
        } else return false;
        if (r.take('}')) return id && value;
        if (!r.take(',')) return false;
    }
}
bool changes(Reader& r, ParameterCommand& out) {
    if (!r.take('[') || r.take(']')) return false;
    for (;;) {
        if (out.count == MAX_PARAMETER_CHANGES || !change(r, out.changes[out.count])) return false;
        ++out.count;
        if (r.take(']')) return true;
        if (!r.take(',')) return false;
    }
}
} // namespace

bool decodeParameterCommand(const char* text, ParameterCommand& output, const char*& error) {
    error = "invalid parameter command";
    if (!text || strlen(text) > 1023) return false;
    Reader r{text}; ParameterCommand parsed{};
    if (!r.take('{')) return false;
    char type[32] = {};
    bool hasType = false, hasTransaction = false, hasRevision = false, hasChanges = false;
    bool hasStart = false, hasMachine = false;
    for (;;) {
        char key[24]; if (!r.string(key, sizeof(key)) || !r.take(':')) return false;
        if (strcmp(key, "type") == 0 && !hasType) {
            if (!r.string(type, sizeof(type))) return false;
            hasType = true;
        } else if (strcmp(key, "transaction") == 0 && !hasTransaction) {
            int32_t n; if (!r.number(n) || n < 0) return false;
            parsed.transaction = static_cast<uint32_t>(n); hasTransaction = true;
        } else if (strcmp(key, "revision") == 0 && !hasRevision) {
            int32_t n; if (!r.number(n) || n < 0) return false;
            parsed.revision = static_cast<uint32_t>(n); hasRevision = true;
        } else if (strcmp(key, "baseRevision") == 0 && !hasRevision) {
            int32_t n; if (!r.number(n) || n < 0) return false;
            parsed.revision = static_cast<uint32_t>(n); hasRevision = true;
        } else if (strcmp(key, "machine") == 0 && !hasMachine) {
            if (!r.string(parsed.machine, sizeof(parsed.machine)) || !parsed.machine[0]) return false;
            hasMachine = true;
        } else if (strcmp(key, "changes") == 0 && !hasChanges) {
            if (!changes(r, parsed)) return false;
            hasChanges = true;
        } else if (strcmp(key, "active") == 0 && !hasStart) {
            if (!r.boolean(parsed.startActive)) return false;
            hasStart = true;
        } else return false;
        if (r.take('}')) break;
        if (!r.take(',')) return false;
    }
    r.space(); if (*r.p || !hasType) return false;
    if (strcmp(type, "param_schema_request") == 0 && !hasTransaction && !hasRevision && !hasChanges && !hasStart)
        parsed.operation = ParameterOperation::SchemaRequest;
    else if (strcmp(type, "param_values_request") == 0 && !hasTransaction && !hasRevision && !hasChanges && !hasStart)
        parsed.operation = ParameterOperation::ValuesRequest;
    else if (strcmp(type, "param_set") == 0 && hasTransaction && hasRevision && hasChanges && hasMachine && !hasStart)
        parsed.operation = ParameterOperation::Set;
    else if (strcmp(type, "param_reset") == 0 && hasTransaction && hasRevision && !hasChanges && hasMachine && !hasStart)
        parsed.operation = ParameterOperation::Reset;
    else if (strcmp(type, "start_set") == 0 && hasTransaction && hasStart && !hasRevision && !hasChanges)
        parsed.operation = ParameterOperation::StartSet;
    else return false;
    output = parsed; error = nullptr; return true;
}
} // namespace fsm
