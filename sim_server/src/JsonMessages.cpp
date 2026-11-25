#include "sim/server/JsonMessages.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace sim::server {
namespace {

// Escapes quotes and backslashes; control characters become spaces.
std::string escapeJson(const std::string& in) {
    std::string out;
    for (const char c : in) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            out += ' ';
        } else {
            out += c;
        }
    }
    return out;
}

}  // namespace

namespace {

std::string number(double v) {
    if (!std::isfinite(v)) {
        return "0";  // JSON has no inf/nan
    }
    char buf[40];
    const int n = std::snprintf(buf, sizeof buf, "%.6g", v);
    return n > 0 ? std::string(buf, static_cast<std::size_t>(n)) : std::string("0");
}

// Advances `i` past a JSON string whose opening quote is at text[i]. Returns false if unterminated.
bool skipString(std::string_view text, std::size_t& i) {
    ++i;  // opening quote
    while (i < text.size()) {
        const char c = text[i];
        if (c == '\\') {
            i += 2;
        } else if (c == '"') {
            ++i;
            return true;
        } else {
            ++i;
        }
    }
    return false;
}

void skipSpace(std::string_view text, std::size_t& i) {
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i])) != 0) {
        ++i;
    }
}

// Skips one JSON value (string, object, array or bare literal). Iterative, so
// deeply nested input cannot overflow the stack.
bool skipValue(std::string_view text, std::size_t& i) {
    if (i >= text.size()) {
        return false;
    }
    if (text[i] == '"') {
        return skipString(text, i);
    }
    if (text[i] == '{' || text[i] == '[') {
        std::size_t depth = 0;
        while (i < text.size()) {
            const char c = text[i];
            if (c == '"') {
                if (!skipString(text, i)) {
                    return false;
                }
                continue;
            }
            if (c == '{' || c == '[') {
                ++depth;
            } else if (c == '}' || c == ']') {
                --depth;
                if (depth == 0) {
                    ++i;
                    return true;
                }
            }
            ++i;
        }
        return false;
    }
    while (i < text.size() && text[i] != ',' && text[i] != '}' && text[i] != ']') {
        ++i;
    }
    return true;
}

}  // namespace

std::string makeHello(const HelloInfo& info) {
    std::string out = "{\"type\":\"hello\",\"protocol\":3,\"scenario\":\"hop\",\"runId\":";
    out += std::to_string(info.runId);
    out += ",\"baseRateHz\":1000,\"snapshotRateHz\":100,\"timeScale\":";
    out += number(info.timeScale);
    out += ",\"vehicle\":{\"name\":\"";
    out += escapeJson(info.vehicleName);
    out += "\",\"length\":";
    out += number(info.vehicleLength);
    out += ",\"radius\":";
    out += number(info.vehicleRadius);
    out += "},\"tower\":{";
    const TowerInfo& t = info.tower;
    out += "\"position\":[" + number(t.position[0]) + "," + number(t.position[1]) + "]";
    out += ",\"approachDir\":[" + number(t.approachDir[0]) + "," + number(t.approachDir[1]) + "]";
    const auto field = [&out](const char* key, double v) {
        out += ",\"";
        out += key;
        out += "\":";
        out += number(v);
    };
    field("mastHalfWidth", t.mastHalfWidth);
    field("mastHeight", t.mastHeight);
    field("hingeHalfSpacing", t.hingeHalfSpacing);
    field("hingeSetback", t.hingeSetback);
    field("armLength", t.armLength);
    field("armHalfWidth", t.armHalfWidth);
    field("armThickness", t.armThickness);
    field("railHeight", t.railHeight);
    field("armOpenAngle", t.armOpenAngle);
    field("armClosedAngle", t.armClosedAngle);
    field("pinHeight", t.pinHeight);
    field("pinOffset", t.pinOffset);
    out += "}}";
    return out;
}

std::string makeRunStarted(uint32_t runId) {
    return "{\"type\":\"run_started\",\"runId\":" + std::to_string(runId) + "}";
}

std::string makeTelemetryHello(uint32_t runId, double timeScale) {
    std::string out = "{\"type\":\"hello\",\"stream\":\"telemetry\",\"protocol\":1,\"runId\":";
    out += std::to_string(runId);
    out += ",\"timeScale\":";
    out += number(timeScale);
    out += ",\"apids\":{\"fsw_status\":256,\"mission_config\":257,\"navigation\":272,"
           "\"guidance_control\":288,\"actuator_feedback\":304,\"gnss\":320,"
           "\"engine_status\":336,\"catch_status\":352,\"catch_config\":353}}";
    return out;
}

// Walks the keys of the top-level object only; "type" inside nested values or
// string contents never matches.
ClientMsg parseClientMessage(std::string_view text) {
    std::size_t i = 0;
    skipSpace(text, i);
    if (i >= text.size() || text[i] != '{') {
        return ClientMsg::Unknown;
    }
    ++i;
    while (true) {
        skipSpace(text, i);
        if (i >= text.size() || text[i] != '"') {
            return ClientMsg::Unknown;  // end of object, or malformed
        }
        const std::size_t keyStart = i + 1;
        if (!skipString(text, i)) {
            return ClientMsg::Unknown;
        }
        const std::string_view key = text.substr(keyStart, i - 1 - keyStart);
        skipSpace(text, i);
        if (i >= text.size() || text[i] != ':') {
            return ClientMsg::Unknown;
        }
        ++i;
        skipSpace(text, i);
        if (key == "type") {
            if (i >= text.size() || text[i] != '"') {
                return ClientMsg::Unknown;
            }
            const std::size_t valueStart = i + 1;
            if (!skipString(text, i)) {
                return ClientMsg::Unknown;
            }
            return text.substr(valueStart, i - 1 - valueStart) == "restart" ? ClientMsg::Restart : ClientMsg::Unknown;
        }
        if (!skipValue(text, i)) {
            return ClientMsg::Unknown;
        }
        skipSpace(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
            continue;
        }
        return ClientMsg::Unknown;  // '}' reached without a type key, or malformed
    }
}

}  // namespace sim::server
