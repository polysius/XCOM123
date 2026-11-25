#include "graph/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace graph {
namespace {

// How deep arrays and objects may nest. A document made by this program is a few levels deep; the limit keeps text from
// somewhere else from running the parser out of stack.
constexpr int kMaxDepth = 64;
// Json::Set looks through the keys it has, so a very large object would take very long to read.
constexpr int kMaxKeysInObject = 5000;

void WriteString(std::string& out, const std::string& text) {
    out += '"';
    for (const char c : text) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char escape[8];
                std::snprintf(escape, sizeof(escape), "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                out += escape;
            } else {
                out += c;
            }
        }
    }
    out += '"';
}

void WriteNumber(std::string& out, double value) {
    if (!std::isfinite(value)) {
        out += "null";  // JSON has no way to write infinity or not-a-number
        return;
    }
    char buffer[40];
    if (value == std::floor(value) && std::abs(value) < 1e15) {
        std::snprintf(buffer, sizeof(buffer), "%.0f", value);
    } else {
        // The fewest digits that read back as the very same number, so that writing what was read gives the same text.
        std::snprintf(buffer, sizeof(buffer), "%.15g", value);
        if (std::strtod(buffer, nullptr) != value) {
            std::snprintf(buffer, sizeof(buffer), "%.17g", value);
        }
    }
    out += buffer;
}

class Parser {
public:
    Parser(const std::string& text, std::string& error) : text(text), error(error) {}

    bool ParseDocument(Json& result) {
        SkipSpace();
        if (!ParseValue(result, 0)) {
            return false;
        }
        SkipSpace();
        if (position != text.size()) {
            return Fail("unexpected text after the end of the document");
        }
        return true;
    }

private:
    bool Fail(const std::string& message) {
        int line = 1;
        int column = 1;
        for (size_t i = 0; i < position && i < text.size(); ++i) {
            if (text[i] == '\n') {
                ++line;
                column = 1;
            } else {
                ++column;
            }
        }
        error = "line " + std::to_string(line) + ", column " + std::to_string(column) + ": " + message;
        return false;
    }

    void SkipSpace() {
        while (position < text.size() && (text[position] == ' ' || text[position] == '\n' || text[position] == '\r' || text[position] == '\t')) {
            ++position;
        }
    }

    bool Consume(const char* word) {
        size_t length = 0;
        while (word[length] != '\0') {
            ++length;
        }
        if (text.compare(position, length, word) == 0) {
            position += length;
            return true;
        }
        return false;
    }

    bool ParseValue(Json& result, int depth) {
        if (depth > kMaxDepth) {
            return Fail("nested too deeply");
        }
        if (position >= text.size()) {
            return Fail("the document ends too soon");
        }
        const char c = text[position];
        if (c == '{') {
            return ParseObject(result, depth);
        }
        if (c == '[') {
            return ParseArray(result, depth);
        }
        if (c == '"') {
            std::string value;
            if (!ParseString(value)) {
                return false;
            }
            result = Json::FromString(std::move(value));
            return true;
        }
        if (Consume("true")) {
            result = Json::FromBool(true);
            return true;
        }
        if (Consume("false")) {
            result = Json::FromBool(false);
            return true;
        }
        if (Consume("null")) {
            result = Json();
            return true;
        }
        return ParseNumber(result);
    }

    bool ParseNumber(Json& result) {
        const char* start = text.c_str() + position;
        char* end = nullptr;
        const double value = std::strtod(start, &end);
        if (end == start || !std::isfinite(value)) {
            return Fail("a value was expected");
        }
        position += static_cast<size_t>(end - start);
        result = Json::FromNumber(value);
        return true;
    }

    bool ParseString(std::string& value) {
        ++position;  // the opening quote
        while (position < text.size()) {
            const char c = text[position++];
            if (c == '"') {
                return true;
            }
            if (c != '\\') {
                value += c;
                continue;
            }
            if (position >= text.size()) {
                break;
            }
            const char escape = text[position++];
            switch (escape) {
            case '"': value += '"'; break;
            case '\\': value += '\\'; break;
            case '/': value += '/'; break;
            case 'n': value += '\n'; break;
            case 'r': value += '\r'; break;
            case 't': value += '\t'; break;
            case 'b': value += '\b'; break;
            case 'f': value += '\f'; break;
            case 'u': {
                if (position + 4 > text.size()) {
                    return Fail("a \\u escape needs four digits");
                }
                const std::string digits = text.substr(position, 4);
                if (digits.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
                    return Fail("an escape of the form backslash u needs four hexadecimal digits");
                }
                const unsigned code = static_cast<unsigned>(std::strtoul(digits.c_str(), nullptr, 16));
                position += 4;
                // Only the characters of the basic Latin range are kept as they are; anything else becomes a question
                // mark. Graphs hold names that a person typed, and those are not worth a full Unicode decoder.
                value += code < 0x80 ? static_cast<char>(code) : '?';
                break;
            }
            default:
                return Fail("an unknown escape in a string");
            }
        }
        return Fail("a string is not closed");
    }

    bool ParseArray(Json& result, int depth) {
        ++position;  // the bracket
        result = Json::MakeArray();
        SkipSpace();
        if (position < text.size() && text[position] == ']') {
            ++position;
            return true;
        }
        while (true) {
            SkipSpace();
            Json item;
            if (!ParseValue(item, depth + 1)) {
                return false;
            }
            result.Push(std::move(item));
            SkipSpace();
            if (position < text.size() && text[position] == ',') {
                ++position;
                continue;
            }
            if (position < text.size() && text[position] == ']') {
                ++position;
                return true;
            }
            return Fail("a comma or the end of the array was expected");
        }
    }

    bool ParseObject(Json& result, int depth) {
        ++position;  // the brace
        result = Json::MakeObject();
        int keyCount = 0;
        SkipSpace();
        if (position < text.size() && text[position] == '}') {
            ++position;
            return true;
        }
        while (true) {
            SkipSpace();
            if (position >= text.size() || text[position] != '"') {
                return Fail("a key in quotes was expected");
            }
            std::string key;
            if (!ParseString(key)) {
                return false;
            }
            SkipSpace();
            if (position >= text.size() || text[position] != ':') {
                return Fail("a colon was expected after a key");
            }
            ++position;
            SkipSpace();
            Json value;
            if (!ParseValue(value, depth + 1)) {
                return false;
            }
            if (++keyCount > kMaxKeysInObject) {
                return Fail("an object has more than " + std::to_string(kMaxKeysInObject) + " keys");
            }
            result.Set(key, std::move(value));
            SkipSpace();
            if (position < text.size() && text[position] == ',') {
                ++position;
                continue;
            }
            if (position < text.size() && text[position] == '}') {
                ++position;
                return true;
            }
            return Fail("a comma or the end of the object was expected");
        }
    }

    const std::string& text;
    std::string& error;
    size_t position = 0;
};

}  // namespace

Json Json::FromBool(bool value) {
    Json json;
    json.type = Type::Bool;
    json.boolean = value;
    return json;
}

Json Json::FromNumber(double value) {
    Json json;
    json.type = Type::Number;
    json.number = value;
    return json;
}

Json Json::FromString(std::string value) {
    Json json;
    json.type = Type::String;
    json.text = std::move(value);
    return json;
}

Json Json::MakeArray() {
    Json json;
    json.type = Type::Array;
    return json;
}

Json Json::MakeObject() {
    Json json;
    json.type = Type::Object;
    return json;
}

void Json::Set(const std::string& key, Json value) {
    for (auto& member : members) {
        if (member.first == key) {
            member.second = std::move(value);
            return;
        }
    }
    members.emplace_back(key, std::move(value));
}

const Json* Json::Find(const std::string& key) const {
    for (const auto& member : members) {
        if (member.first == key) {
            return &member.second;
        }
    }
    return nullptr;
}

std::string Json::Write(int indent) const {
    std::string out;
    WriteTo(out, indent, 0);
    return out;
}

void Json::WriteTo(std::string& out, int indent, int depth) const {
    const auto newline = [&](int level) {
        if (indent > 0) {
            out += '\n';
            out.append(static_cast<size_t>(level * indent), ' ');
        }
    };
    switch (type) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += boolean ? "true" : "false"; break;
    case Type::Number: WriteNumber(out, number); break;
    case Type::String: WriteString(out, text); break;
    case Type::Array:
        out += '[';
        for (size_t i = 0; i < items.size(); ++i) {
            out += i == 0 ? "" : ",";
            newline(depth + 1);
            items[i].WriteTo(out, indent, depth + 1);
        }
        if (!items.empty()) {
            newline(depth);
        }
        out += ']';
        break;
    case Type::Object:
        out += '{';
        for (size_t i = 0; i < members.size(); ++i) {
            out += i == 0 ? "" : ",";
            newline(depth + 1);
            WriteString(out, members[i].first);
            out += indent > 0 ? ": " : ":";
            members[i].second.WriteTo(out, indent, depth + 1);
        }
        if (!members.empty()) {
            newline(depth);
        }
        out += '}';
        break;
    }
}

bool Json::Parse(const std::string& text, Json& result, std::string& error) {
    Parser parser(text, error);
    return parser.ParseDocument(result);
}

}  // namespace graph
