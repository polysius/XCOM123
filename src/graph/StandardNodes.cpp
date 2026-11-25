#include <algorithm>
#include <cmath>

#include "graph/Interpreter.h"
#include "graph/NodeRegistry.h"

namespace graph {
namespace {

PinDescription Pin(const std::string& name, PinType type) {
    return {name, type, DefaultValue(type), {}};
}

PinDescription Pin(const std::string& name, PinType type, Value value) {
    return {name, type, std::move(value), {}};
}

PinDescription Exec(const std::string& name) {
    return {name, PinType::Exec, false, {}};
}

// A node that only computes: pins in, pins out, and the function that turns one into the other.
void Pure(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& category, const std::string& tooltip,
          std::vector<PinDescription> inputs, std::vector<PinDescription> outputs, NodeFunction run) {
    NodeDescription description;
    description.id = id;
    description.title = title;
    description.category = category;
    description.tooltip = tooltip;
    description.kind = NodeKind::Pure;
    description.inputs = std::move(inputs);
    description.outputs = std::move(outputs);
    description.run = std::move(run);
    registry.Register(std::move(description));
}

// A node that runs in turn. It starts with an Exec input called "In" and goes on from its first Exec output unless it says
// otherwise.
void Action(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& category, const std::string& tooltip,
            std::vector<PinDescription> inputs, std::vector<PinDescription> outputs, NodeFunction run, bool continuesAutomatically = true) {
    NodeDescription description;
    description.id = id;
    description.title = title;
    description.category = category;
    description.tooltip = tooltip;
    description.kind = NodeKind::Action;
    description.inputs.push_back(Exec("In"));
    for (PinDescription& pin : inputs) {
        description.inputs.push_back(std::move(pin));
    }
    description.outputs = std::move(outputs);
    description.run = std::move(run);
    description.continuesAutomatically = continuesAutomatically;
    registry.Register(std::move(description));
}

void Event(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& tooltip, std::vector<PinDescription> outputs,
           NodeFunction run) {
    NodeDescription description;
    description.id = id;
    description.title = title;
    description.category = "Events";
    description.tooltip = tooltip;
    description.kind = NodeKind::Event;
    description.outputs.push_back(Exec("Then"));
    for (PinDescription& pin : outputs) {
        description.outputs.push_back(std::move(pin));
    }
    description.run = std::move(run);
    registry.Register(std::move(description));
}

// Two floats to a float, for the arithmetic nodes.
void Arithmetic(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& tooltip, float (*operation)(float, float)) {
    Pure(registry, id, title, "Math", tooltip, {Pin("A", PinType::Float), Pin("B", PinType::Float)}, {Pin("Result", PinType::Float)},
         [operation](NodeContext& context) { context.Set(0, operation(context.Get<float>(0), context.Get<float>(1))); });
}

void IntArithmetic(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& tooltip, int (*operation)(int, int)) {
    Pure(registry, id, title, "Math", tooltip, {Pin("A", PinType::Int), Pin("B", PinType::Int)}, {Pin("Result", PinType::Int)},
         [operation](NodeContext& context) { context.Set(0, operation(context.Get<int>(0), context.Get<int>(1))); });
}

void Comparison(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& tooltip, bool (*operation)(float, float)) {
    Pure(registry, id, title, "Compare", tooltip, {Pin("A", PinType::Float), Pin("B", PinType::Float)}, {Pin("Result", PinType::Bool)},
         [operation](NodeContext& context) { context.Set(0, operation(context.Get<float>(0), context.Get<float>(1))); });
}

void Unary(NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& tooltip, float (*operation)(float)) {
    Pure(registry, id, title, "Math", tooltip, {Pin("Value", PinType::Float)}, {Pin("Result", PinType::Float)},
         [operation](NodeContext& context) { context.Set(0, operation(context.Get<float>(0))); });
}

void RegisterEventsAndFlow(NodeRegistry& registry) {
    Event(registry, "event.start", "Start", "Runs once, when the graph starts.", {}, nullptr);
    Event(registry, "event.tick", "Every frame", "Runs every frame, with the time since the last one.", {Pin("Delta Seconds", PinType::Float)},
          [](NodeContext& context) { context.Set(1, context.DeltaSeconds()); });

    Action(registry, "flow.branch", "Branch", "Flow", "Goes on from True or from False, depending on the condition.", {Pin("Condition", PinType::Bool)},
           {Exec("True"), Exec("False")}, [](NodeContext& context) { context.Execute(context.Get<bool>(1) ? 0 : 1); }, false);

    Action(registry, "flow.sequence", "Sequence", "Flow", "Runs what follows each output, one after the other, from the first.", {},
           {Exec("Then 0"), Exec("Then 1"), Exec("Then 2")},
           [](NodeContext& context) {
               for (int output = 0; output < 3; ++output) {
                   context.Execute(output);
               }
           },
           false);

    Action(registry, "flow.for", "For loop", "Flow", "Runs the loop body once for every whole number from First to Last, then goes on from Completed.",
           {Pin("First", PinType::Int, 0), Pin("Last", PinType::Int, 9)}, {Exec("Loop Body"), Pin("Index", PinType::Int), Exec("Completed")},
           [](NodeContext& context) {
               const int first = context.Get<int>(1);
               const int last = context.Get<int>(2);
               for (int index = first; index <= last && !context.Stopped(); ++index) {
                   context.Set(1, index);
                   context.Execute(0);
                   if (index == last) {
                       break;  // a loop that ends at the largest int must not step past it
                   }
               }
               context.Execute(2);
           },
           false);

    Pure(registry, "flow.select", "Select", "Flow", "Gives A when the condition is true, and B when it is false.",
         {Pin("Condition", PinType::Bool), Pin("A", PinType::Float), Pin("B", PinType::Float)}, {Pin("Result", PinType::Float)},
         [](NodeContext& context) { context.Set(0, context.Get<bool>(0) ? context.Get<float>(1) : context.Get<float>(2)); });
}

void RegisterMath(NodeRegistry& registry) {
    Arithmetic(registry, "math.add", "Add", "A plus B.", [](float a, float b) { return a + b; });
    Arithmetic(registry, "math.subtract", "Subtract", "A minus B.", [](float a, float b) { return a - b; });
    Arithmetic(registry, "math.multiply", "Multiply", "A times B.", [](float a, float b) { return a * b; });
    Arithmetic(registry, "math.divide", "Divide", "A divided by B; 0 when B is 0.", [](float a, float b) { return b == 0.0f ? 0.0f : a / b; });
    Arithmetic(registry, "math.min", "Smaller", "The smaller of A and B.", [](float a, float b) { return std::min(a, b); });
    Arithmetic(registry, "math.max", "Larger", "The larger of A and B.", [](float a, float b) { return std::max(a, b); });
    Arithmetic(registry, "math.power", "Power", "A raised to the power B.", [](float a, float b) {
        const float result = std::pow(a, b);
        return std::isfinite(result) ? result : 0.0f;
    });

    IntArithmetic(registry, "int.add", "Add (whole numbers)", "A plus B.", [](int a, int b) { return static_cast<int>(static_cast<unsigned>(a) + static_cast<unsigned>(b)); });
    IntArithmetic(registry, "int.subtract", "Subtract (whole numbers)", "A minus B.", [](int a, int b) { return static_cast<int>(static_cast<unsigned>(a) - static_cast<unsigned>(b)); });
    IntArithmetic(registry, "int.multiply", "Multiply (whole numbers)", "A times B.", [](int a, int b) { return static_cast<int>(static_cast<unsigned>(a) * static_cast<unsigned>(b)); });
    IntArithmetic(registry, "int.divide", "Divide (whole numbers)", "A divided by B, dropping the rest; 0 when B is 0.", [](int a, int b) { return b == 0 || (a == -2147483647 - 1 && b == -1) ? 0 : a / b; });
    IntArithmetic(registry, "int.modulo", "Remainder", "What is left of A after taking away as many B as fit; 0 when B is 0.", [](int a, int b) { return b == 0 || b == -1 ? 0 : a % b; });

    Unary(registry, "math.abs", "Absolute value", "The value without its sign.", [](float v) { return std::abs(v); });
    Unary(registry, "math.floor", "Round down", "The largest whole number not above the value.", [](float v) { return std::floor(v); });
    Unary(registry, "math.round", "Round", "The nearest whole number.", [](float v) { return std::round(v); });
    Unary(registry, "math.sqrt", "Square root", "The square root; 0 for a negative value.", [](float v) { return v > 0.0f ? std::sqrt(v) : 0.0f; });
    Unary(registry, "math.sin", "Sine", "The sine of an angle in radians.", [](float v) { return std::sin(v); });
    Unary(registry, "math.cos", "Cosine", "The cosine of an angle in radians.", [](float v) { return std::cos(v); });
    Unary(registry, "math.radians", "Degrees to radians", "An angle in degrees as radians.", [](float v) { return v * 0.01745329252f; });

    Pure(registry, "math.clamp", "Clamp", "Math", "The value, kept between Min and Max.", {Pin("Value", PinType::Float), Pin("Min", PinType::Float, 0.0f), Pin("Max", PinType::Float, 1.0f)},
         {Pin("Result", PinType::Float)}, [](NodeContext& context) {
             const float low = std::min(context.Get<float>(1), context.Get<float>(2));
             const float high = std::max(context.Get<float>(1), context.Get<float>(2));
             context.Set(0, std::clamp(context.Get<float>(0), low, high));
         });
    Pure(registry, "math.lerp", "Blend", "Math", "From A to B as Alpha goes from 0 to 1.", {Pin("A", PinType::Float), Pin("B", PinType::Float, 1.0f), Pin("Alpha", PinType::Float, 0.5f)},
         {Pin("Result", PinType::Float)}, [](NodeContext& context) {
             const float a = context.Get<float>(0);
             context.Set(0, a + (context.Get<float>(1) - a) * context.Get<float>(2));
         });
    Pure(registry, "math.random", "Random number", "Math", "A number from Min up to Max. It is the same on every run of the graph.", {Pin("Min", PinType::Float, 0.0f), Pin("Max", PinType::Float, 1.0f)},
         {Pin("Result", PinType::Float)}, [](NodeContext& context) {
             const float low = context.Get<float>(0);
             context.Set(0, low + (context.Get<float>(1) - low) * context.Random());
         });
}

void RegisterCompareAndLogic(NodeRegistry& registry) {
    Comparison(registry, "compare.less", "Less than", "A is below B.", [](float a, float b) { return a < b; });
    Comparison(registry, "compare.less_equal", "Less or equal", "A is not above B.", [](float a, float b) { return a <= b; });
    Comparison(registry, "compare.greater", "Greater than", "A is above B.", [](float a, float b) { return a > b; });
    Comparison(registry, "compare.greater_equal", "Greater or equal", "A is not below B.", [](float a, float b) { return a >= b; });
    Comparison(registry, "compare.equal", "Equal", "A is the same number as B.", [](float a, float b) { return a == b; });
    Pure(registry, "compare.text_equal", "Same text", "Compare", "A and B are the same text.", {Pin("A", PinType::String), Pin("B", PinType::String)}, {Pin("Result", PinType::Bool)},
         [](NodeContext& context) { context.Set(0, context.Get<std::string>(0) == context.Get<std::string>(1)); });

    Pure(registry, "logic.and", "And", "Logic", "True when both are true.", {Pin("A", PinType::Bool), Pin("B", PinType::Bool)}, {Pin("Result", PinType::Bool)},
         [](NodeContext& context) { context.Set(0, context.Get<bool>(0) && context.Get<bool>(1)); });
    Pure(registry, "logic.or", "Or", "Logic", "True when at least one is true.", {Pin("A", PinType::Bool), Pin("B", PinType::Bool)}, {Pin("Result", PinType::Bool)},
         [](NodeContext& context) { context.Set(0, context.Get<bool>(0) || context.Get<bool>(1)); });
    Pure(registry, "logic.not", "Not", "Logic", "True when the input is false.", {Pin("Value", PinType::Bool)}, {Pin("Result", PinType::Bool)},
         [](NodeContext& context) { context.Set(0, !context.Get<bool>(0)); });
}

void RegisterVectorsAndText(NodeRegistry& registry) {
    Pure(registry, "vec3.make", "Make vector", "Vector", "A vector from three numbers.", {Pin("X", PinType::Float), Pin("Y", PinType::Float), Pin("Z", PinType::Float)},
         {Pin("Vector", PinType::Vec3)}, [](NodeContext& context) { context.Set(0, glm::vec3(context.Get<float>(0), context.Get<float>(1), context.Get<float>(2))); });
    Pure(registry, "vec3.break", "Break vector", "Vector", "The three numbers of a vector.", {Pin("Vector", PinType::Vec3)},
         {Pin("X", PinType::Float), Pin("Y", PinType::Float), Pin("Z", PinType::Float)}, [](NodeContext& context) {
             const glm::vec3 v = context.Get<glm::vec3>(0);
             context.Set(0, v.x);
             context.Set(1, v.y);
             context.Set(2, v.z);
         });
    Pure(registry, "vec3.add", "Add vectors", "Vector", "A plus B.", {Pin("A", PinType::Vec3), Pin("B", PinType::Vec3)}, {Pin("Result", PinType::Vec3)},
         [](NodeContext& context) { context.Set(0, context.Get<glm::vec3>(0) + context.Get<glm::vec3>(1)); });
    Pure(registry, "vec3.subtract", "Subtract vectors", "Vector", "A minus B.", {Pin("A", PinType::Vec3), Pin("B", PinType::Vec3)}, {Pin("Result", PinType::Vec3)},
         [](NodeContext& context) { context.Set(0, context.Get<glm::vec3>(0) - context.Get<glm::vec3>(1)); });
    Pure(registry, "vec3.scale", "Scale vector", "Vector", "A vector times a number.", {Pin("Vector", PinType::Vec3), Pin("Scale", PinType::Float, 1.0f)}, {Pin("Result", PinType::Vec3)},
         [](NodeContext& context) { context.Set(0, context.Get<glm::vec3>(0) * context.Get<float>(1)); });
    Pure(registry, "vec3.length", "Vector length", "Vector", "The length of a vector.", {Pin("Vector", PinType::Vec3)}, {Pin("Length", PinType::Float)},
         [](NodeContext& context) { context.Set(0, glm::length(context.Get<glm::vec3>(0))); });
    Pure(registry, "vec3.normalize", "Normalize vector", "Vector", "The vector made one unit long; the zero vector stays zero.", {Pin("Vector", PinType::Vec3)}, {Pin("Result", PinType::Vec3)},
         [](NodeContext& context) {
             const glm::vec3 v = context.Get<glm::vec3>(0);
             const float length = glm::length(v);
             context.Set(0, length > 1e-12f ? v / length : glm::vec3(0.0f));
         });
    Pure(registry, "vec3.dot", "Dot product", "Vector", "How far two vectors point the same way.", {Pin("A", PinType::Vec3), Pin("B", PinType::Vec3)}, {Pin("Result", PinType::Float)},
         [](NodeContext& context) { context.Set(0, glm::dot(context.Get<glm::vec3>(0), context.Get<glm::vec3>(1))); });
    Pure(registry, "vec3.cross", "Cross product", "Vector", "The vector at right angles to A and B.", {Pin("A", PinType::Vec3), Pin("B", PinType::Vec3)}, {Pin("Result", PinType::Vec3)},
         [](NodeContext& context) { context.Set(0, glm::cross(context.Get<glm::vec3>(0), context.Get<glm::vec3>(1))); });
    Pure(registry, "vec3.lerp", "Blend vectors", "Vector", "From A to B as Alpha goes from 0 to 1.", {Pin("A", PinType::Vec3), Pin("B", PinType::Vec3), Pin("Alpha", PinType::Float, 0.5f)},
         {Pin("Result", PinType::Vec3)}, [](NodeContext& context) { context.Set(0, glm::mix(context.Get<glm::vec3>(0), context.Get<glm::vec3>(1), context.Get<float>(2))); });

    Pure(registry, "text.join", "Join text", "Text", "A followed by B. Numbers and vectors become text.", {Pin("A", PinType::String), Pin("B", PinType::String)}, {Pin("Result", PinType::String)},
         [](NodeContext& context) { context.Set(0, context.Get<std::string>(0) + context.Get<std::string>(1)); });
}

void RegisterVariablesAndDebug(NodeRegistry& registry) {
    NodeDescription get;
    get.id = "variable.get";
    get.title = "Get variable";
    get.category = "Variables";
    get.tooltip = "The value of a variable.";
    get.kind = NodeKind::Pure;
    get.outputs = {Pin("Value", PinType::Float)};
    get.usesVariable = true;
    get.run = [](NodeContext& context) { context.Set(0, context.GetVariable(context.VariableName())); };
    registry.Register(std::move(get));

    NodeDescription set;
    set.id = "variable.set";
    set.title = "Set variable";
    set.category = "Variables";
    set.tooltip = "Gives a variable a new value.";
    set.kind = NodeKind::Action;
    set.inputs = {Exec("In"), Pin("Value", PinType::Float)};
    set.outputs = {Exec("Then"), Pin("Value", PinType::Float)};
    set.usesVariable = true;
    set.run = [](NodeContext& context) {
        context.SetVariable(context.VariableName(), context.Input(1));
        context.Set(1, context.GetVariable(context.VariableName()));
    };
    registry.Register(std::move(set));

    Action(registry, "debug.print", "Print", "Debug", "Adds a line to the log. Numbers and vectors become text.", {Pin("Text", PinType::String, std::string("Hello"))}, {Exec("Then")},
           [](NodeContext& context) { context.Log(context.Get<std::string>(1)); });
}

}  // namespace

void RegisterStandardNodes(NodeRegistry& registry) {
    RegisterEventsAndFlow(registry);
    RegisterMath(registry);
    RegisterCompareAndLogic(registry);
    RegisterVectorsAndText(registry);
    RegisterVariablesAndDebug(registry);
}

}  // namespace graph
