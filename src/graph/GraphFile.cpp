#include "graph/GraphFile.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <tuple>

#include "graph/Json.h"

namespace graph {
namespace {

constexpr int kFileVersion = 1;
// Ids this large are refused so that the next id, one more, never wraps round to 0 and repeats an id of the graph.
constexpr double kLargestNodeId = 4000000000.0;

// A number for the file. JSON cannot hold infinity or not-a-number, so those are saved as 0, which is also what is read back.
Json NumberOf(double value) {
    return Json::FromNumber(std::isfinite(value) ? value : 0.0);
}

Json ValueToJson(const Value& value) {
    Json json = Json::MakeObject();
    json.Set("t", Json::FromString(PinTypeName(TypeOf(value))));
    switch (TypeOf(value)) {
    case PinType::Bool: json.Set("v", Json::FromBool(std::get<bool>(value))); break;
    case PinType::Int: json.Set("v", NumberOf(std::get<int>(value))); break;
    case PinType::Float: json.Set("v", NumberOf(std::get<float>(value))); break;
    case PinType::Vec3: {
        const glm::vec3 v = std::get<glm::vec3>(value);
        Json items = Json::MakeArray();
        items.Push(NumberOf(v.x));
        items.Push(NumberOf(v.y));
        items.Push(NumberOf(v.z));
        json.Set("v", std::move(items));
        break;
    }
    default: json.Set("v", Json::FromString(std::get<std::string>(value))); break;
    }
    return json;
}

// A float from a number of the file, without being thrown by one that is far out of range.
float FloatOf(const Json& json) {
    if (json.GetType() != Json::Type::Number) {
        return 0.0f;
    }
    return static_cast<float>(std::clamp(json.AsNumber(), -double{FLT_MAX}, double{FLT_MAX}));
}

int IntOf(const Json& json) {
    if (json.GetType() != Json::Type::Number) {
        return 0;
    }
    return static_cast<int>(std::clamp(json.AsNumber(), -2147483648.0, 2147483647.0));
}

bool ValueFromJson(const Json& json, Value& value) {
    const Json* typeName = json.Find("t");
    const Json* content = json.Find("v");
    PinType type;
    if (typeName == nullptr || content == nullptr || typeName->GetType() != Json::Type::String || !ParsePinType(typeName->AsString(), type) || type == PinType::Exec) {
        return false;
    }
    switch (type) {
    case PinType::Bool: value = content->GetType() == Json::Type::Bool && content->AsBool(); break;
    case PinType::Int: value = IntOf(*content); break;
    case PinType::Float: value = FloatOf(*content); break;
    case PinType::Vec3:
        if (content->GetType() != Json::Type::Array || content->Items().size() != 3) {
            return false;
        }
        value = glm::vec3(FloatOf(content->Items()[0]), FloatOf(content->Items()[1]), FloatOf(content->Items()[2]));
        break;
    default: value = content->GetType() == Json::Type::String ? content->AsString() : std::string(); break;
    }
    return true;
}

Json PinToJson(PinRef pin) {
    Json items = Json::MakeArray();
    items.Push(NumberOf(pin.node));
    items.Push(NumberOf(pin.pin));
    return items;
}

bool PinFromJson(const Json* json, PinRef& pin) {
    if (json == nullptr || json->GetType() != Json::Type::Array || json->Items().size() != 2 || json->Items()[0].GetType() != Json::Type::Number ||
        json->Items()[1].GetType() != Json::Type::Number) {
        return false;
    }
    const double node = json->Items()[0].AsNumber();
    if (node < 0.0 || node > 4294967295.0) {
        return false;
    }
    pin.node = static_cast<uint32_t>(node);
    pin.pin = IntOf(json->Items()[1]);
    return true;
}

}  // namespace

std::string SaveGraph(const Graph& graph) {
    Json root = Json::MakeObject();
    root.Set("version", NumberOf(kFileVersion));
    root.Set("nextId", NumberOf(graph.NextId()));

    Json variables = Json::MakeArray();
    for (const Variable& variable : graph.Variables()) {
        Json entry = Json::MakeObject();
        entry.Set("name", Json::FromString(variable.name));
        entry.Set("type", Json::FromString(PinTypeName(variable.type)));
        entry.Set("value", ValueToJson(Convert(variable.initial, variable.type)));
        variables.Push(std::move(entry));
    }
    root.Set("variables", std::move(variables));

    // Nodes and wires in the order of their numbers, so that the text does not depend on the order they were made in.
    std::vector<const Node*> nodes;
    for (const Node& node : graph.Nodes()) {
        nodes.push_back(&node);
    }
    std::sort(nodes.begin(), nodes.end(), [](const Node* a, const Node* b) { return a->id < b->id; });
    Json nodeList = Json::MakeArray();
    for (const Node* node : nodes) {
        Json entry = Json::MakeObject();
        entry.Set("id", NumberOf(node->id));
        entry.Set("type", Json::FromString(node->type));
        entry.Set("x", NumberOf(node->position.x));
        entry.Set("y", NumberOf(node->position.y));
        if (!node->inputValues.empty()) {
            Json inputs = Json::MakeObject();
            for (const auto& [index, value] : node->inputValues) {
                inputs.Set(std::to_string(index), ValueToJson(value));
            }
            entry.Set("inputs", std::move(inputs));
        }
        if (!node->properties.empty()) {
            Json properties = Json::MakeObject();
            for (const auto& [name, value] : node->properties) {
                properties.Set(name, ValueToJson(value));
            }
            entry.Set("properties", std::move(properties));
        }
        nodeList.Push(std::move(entry));
    }
    root.Set("nodes", std::move(nodeList));

    std::vector<Connection> wires = graph.Connections();
    std::sort(wires.begin(), wires.end(), [](const Connection& a, const Connection& b) {
        return std::tie(a.to.node, a.to.pin, a.from.node, a.from.pin) < std::tie(b.to.node, b.to.pin, b.from.node, b.from.pin);
    });
    Json wireList = Json::MakeArray();
    for (const Connection& wire : wires) {
        Json entry = Json::MakeObject();
        entry.Set("from", PinToJson(wire.from));
        entry.Set("to", PinToJson(wire.to));
        wireList.Push(std::move(entry));
    }
    root.Set("connections", std::move(wireList));
    return root.Write(2);
}

LoadResult LoadGraph(const std::string& text, Graph& graph) {
    LoadResult result;
    Json root;
    std::string parseError;
    if (!Json::Parse(text, root, parseError)) {
        result.error = "This is not a graph file: " + parseError;
        return result;
    }
    const Json* version = root.Find("version");
    if (root.GetType() != Json::Type::Object || version == nullptr || version->GetType() != Json::Type::Number) {
        result.error = "This is not a graph file: it has no version.";
        return result;
    }
    if (version->AsNumber() != kFileVersion) {
        result.error = "This graph was saved by another version of the program (version " + Json::FromNumber(version->AsNumber()).Write(0) + ").";
        return result;
    }

    Graph loaded;
    if (const Json* variables = root.Find("variables"); variables != nullptr && variables->GetType() == Json::Type::Array) {
        for (const Json& entry : variables->Items()) {
            const Json* name = entry.Find("name");
            const Json* type = entry.Find("type");
            const Json* value = entry.Find("value");
            Variable variable;
            if (name == nullptr || type == nullptr || name->GetType() != Json::Type::String || type->GetType() != Json::Type::String ||
                !ParsePinType(type->AsString(), variable.type) || variable.type == PinType::Exec) {
                result.warnings.push_back("A variable that could not be read was left out.");
                continue;
            }
            variable.name = name->AsString();
            variable.initial = DefaultValue(variable.type);
            if (value != nullptr) {
                Value read;
                if (ValueFromJson(*value, read)) {
                    variable.initial = Convert(read, variable.type);
                }
            }
            loaded.Variables().push_back(std::move(variable));
        }
    }

    if (const Json* nodes = root.Find("nodes"); nodes != nullptr && nodes->GetType() == Json::Type::Array) {
        for (const Json& entry : nodes->Items()) {
            const Json* id = entry.Find("id");
            const Json* type = entry.Find("type");
            if (id == nullptr || type == nullptr || id->GetType() != Json::Type::Number || type->GetType() != Json::Type::String || id->AsNumber() < 1.0 ||
                id->AsNumber() > kLargestNodeId) {
                result.warnings.push_back("A node that could not be read was left out.");
                continue;
            }
            Node node;
            node.id = static_cast<uint32_t>(id->AsNumber());
            if (loaded.FindNode(node.id) != nullptr) {
                result.warnings.push_back("Node " + std::to_string(node.id) + " is in the file twice; the second one was left out.");
                continue;
            }
            node.type = type->AsString();
            if (const Json* x = entry.Find("x")) {
                node.position.x = FloatOf(*x);
            }
            if (const Json* y = entry.Find("y")) {
                node.position.y = FloatOf(*y);
            }
            if (const Json* inputs = entry.Find("inputs"); inputs != nullptr && inputs->GetType() == Json::Type::Object) {
                for (const auto& [key, value] : inputs->Members()) {
                    Value read;
                    const int index = std::atoi(key.c_str());
                    if (ValueFromJson(value, read) && index >= 0) {
                        node.inputValues[index] = std::move(read);
                    }
                }
            }
            if (const Json* properties = entry.Find("properties"); properties != nullptr && properties->GetType() == Json::Type::Object) {
                for (const auto& [key, value] : properties->Members()) {
                    Value read;
                    if (ValueFromJson(value, read)) {
                        node.properties[key] = std::move(read);
                    }
                }
            }
            loaded.AddLoadedNode(std::move(node));
        }
    }

    if (const Json* wires = root.Find("connections"); wires != nullptr && wires->GetType() == Json::Type::Array) {
        for (const Json& entry : wires->Items()) {
            Connection connection;
            if (!PinFromJson(entry.Find("from"), connection.from) || !PinFromJson(entry.Find("to"), connection.to)) {
                result.warnings.push_back("A wire that could not be read was left out.");
                continue;
            }
            if (loaded.FindNode(connection.from.node) == nullptr || loaded.FindNode(connection.to.node) == nullptr) {
                result.warnings.push_back("A wire to a node that is not in the file was left out.");
                continue;
            }
            loaded.AddLoadedConnection(connection);
        }
    }
    if (const Json* next = root.Find("nextId"); next != nullptr && next->GetType() == Json::Type::Number && next->AsNumber() >= 1.0 &&
                                                next->AsNumber() <= kLargestNodeId + 1.0) {
        loaded.SetNextId(std::max(loaded.NextId(), static_cast<uint32_t>(next->AsNumber())));
    }
    graph = std::move(loaded);
    result.ok = true;
    return result;
}

}  // namespace graph
