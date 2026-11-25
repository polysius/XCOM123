#pragma once

#include <string>
#include <utility>
#include <vector>

// A small JSON document: just enough to save and load a graph. Objects keep the order their keys were added in, so
// writing the same document twice gives the same text, and a saved graph stays easy to compare in a diff.

namespace graph {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Json() = default;
    static Json FromBool(bool value);
    static Json FromNumber(double value);
    static Json FromString(std::string value);
    static Json MakeArray();
    static Json MakeObject();

    Type GetType() const { return type; }
    bool AsBool() const { return boolean; }
    double AsNumber() const { return number; }
    const std::string& AsString() const { return text; }

    // Arrays.
    void Push(Json value) { items.push_back(std::move(value)); }
    const std::vector<Json>& Items() const { return items; }

    // Objects. Set replaces the value of a key that is there already; Find returns null for a key that is not.
    void Set(const std::string& key, Json value);
    const Json* Find(const std::string& key) const;
    const std::vector<std::pair<std::string, Json>>& Members() const { return members; }

    // The text of the document, indented by `indent` spaces per level (0 for a single line).
    std::string Write(int indent = 2) const;

    // Reads a document. Returns false and says where and why in `error` when the text is not valid JSON.
    static bool Parse(const std::string& text, Json& result, std::string& error);

private:
    void WriteTo(std::string& out, int indent, int depth) const;

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<Json> items;
    std::vector<std::pair<std::string, Json>> members;
};

}  // namespace graph
