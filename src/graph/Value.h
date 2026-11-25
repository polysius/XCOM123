#pragma once

#include <cstdint>
#include <string>
#include <variant>

#include <glm/glm.hpp>

// The kinds of value that run along the wires of a graph, and the kinds of pin that carry them.

namespace graph {

// A pin carries execution (the white wires: what runs next) or one of the kinds of data.
enum class PinType : uint8_t {
    Exec,
    Bool,
    Int,
    Float,
    Vec3,
    String,
};

constexpr int kPinTypeCount = 6;

const char* PinTypeName(PinType type);
// Finds a pin type by its name, as saved in a graph file. Returns false for any other text.
bool ParsePinType(const std::string& name, PinType& type);

// A value of any data type. Exec pins carry none.
using Value = std::variant<bool, int, float, glm::vec3, std::string>;

// The type of a value, which is never Exec.
PinType TypeOf(const Value& value);

// The value a pin of this type holds until it is given another: false, 0, 0.0, the zero vector, the empty text. Exec has
// no value, and gets false.
Value DefaultValue(PinType type);

// Whether a wire may carry a value from a pin of one type to a pin of another, with a conversion on the way. Exec only
// joins Exec. A bool becomes a number, a whole number becomes a decimal one and a decimal one a whole number (the part
// after the point is dropped), and anything at all becomes text. A vector only becomes a vector or text.
bool CanConvert(PinType from, PinType to);

// The value in the type asked for, by the rules above. A conversion that is not allowed gives the default of the type.
Value Convert(const Value& value, PinType to);

// A value as text for a person to read, such as in a log: "3", "0.5", "true", "(1, 2, 3)".
std::string ToText(const Value& value);

}  // namespace graph
