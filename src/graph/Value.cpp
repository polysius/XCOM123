#include "graph/Value.h"

#include <cmath>
#include <cstdio>

namespace graph {
namespace {

const char* const kNames[kPinTypeCount] = {"exec", "bool", "int", "float", "vec3", "string"};

std::string FormatFloat(float value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%g", static_cast<double>(value));
    return buffer;
}

// A float as a whole number, without the undefined behaviour of converting one that is out of range.
int ToInt(float value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    if (value >= 2147483520.0f) {
        return 2147483647;
    }
    if (value <= -2147483648.0f) {
        return -2147483647 - 1;
    }
    return static_cast<int>(value);
}

}  // namespace

const char* PinTypeName(PinType type) {
    return kNames[static_cast<int>(type)];
}

bool ParsePinType(const std::string& name, PinType& type) {
    for (int i = 0; i < kPinTypeCount; ++i) {
        if (name == kNames[i]) {
            type = static_cast<PinType>(i);
            return true;
        }
    }
    return false;
}

PinType TypeOf(const Value& value) {
    switch (value.index()) {
    case 0: return PinType::Bool;
    case 1: return PinType::Int;
    case 2: return PinType::Float;
    case 3: return PinType::Vec3;
    default: return PinType::String;
    }
}

Value DefaultValue(PinType type) {
    switch (type) {
    case PinType::Int: return 0;
    case PinType::Float: return 0.0f;
    case PinType::Vec3: return glm::vec3(0.0f);
    case PinType::String: return std::string();
    default: return false;
    }
}

bool CanConvert(PinType from, PinType to) {
    if (from == to) {
        return true;
    }
    if (from == PinType::Exec || to == PinType::Exec) {
        return false;
    }
    if (to == PinType::String) {
        return true;
    }
    if (from == PinType::Vec3 || to == PinType::Vec3) {
        return false;
    }
    // Among bool, int and float: a bool goes up to a number, and a number goes either way except to a bool.
    return to != PinType::Bool && from != PinType::String;
}

Value Convert(const Value& value, PinType to) {
    const PinType from = TypeOf(value);
    if (from == to) {
        return value;
    }
    if (!CanConvert(from, to)) {
        return DefaultValue(to);
    }
    if (to == PinType::String) {
        return ToText(value);
    }
    if (to == PinType::Int) {
        return from == PinType::Bool ? (std::get<bool>(value) ? 1 : 0) : ToInt(std::get<float>(value));
    }
    // To a float, from a bool or an int.
    return from == PinType::Bool ? (std::get<bool>(value) ? 1.0f : 0.0f) : static_cast<float>(std::get<int>(value));
}

std::string ToText(const Value& value) {
    switch (TypeOf(value)) {
    case PinType::Bool: return std::get<bool>(value) ? "true" : "false";
    case PinType::Int: return std::to_string(std::get<int>(value));
    case PinType::Float: return FormatFloat(std::get<float>(value));
    case PinType::Vec3: {
        const glm::vec3 v = std::get<glm::vec3>(value);
        return "(" + FormatFloat(v.x) + ", " + FormatFloat(v.y) + ", " + FormatFloat(v.z) + ")";
    }
    default: return std::get<std::string>(value);
    }
}

}  // namespace graph
