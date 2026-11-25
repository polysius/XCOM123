#include "graph_editor/AutoLayout.h"

#include <algorithm>
#include <map>
#include <vector>

#include "graph_editor/NodeLayout.h"

namespace graph_editor {
namespace {

constexpr float kColumnGap = 70.0f;
constexpr float kRowGap = 30.0f;

// The column of every node: one more than the farthest column among the nodes that feed it. Execution wires and data wires
// can together form a cycle (an action's result feeds a pure node that feeds an earlier action), so the columns are settled by
// a number of passes that cannot exceed the number of nodes, and not by recursion.
std::map<uint32_t, int> ColumnsOf(const graph::Graph& graph) {
    std::map<uint32_t, int> columns;
    for (const graph::Node& node : graph.Nodes()) {
        columns[node.id] = 0;
    }
    const int passes = static_cast<int>(graph.Nodes().size());
    for (int pass = 0; pass < passes; ++pass) {
        bool changed = false;
        for (const graph::Connection& connection : graph.Connections()) {
            const auto from = columns.find(connection.from.node);
            const auto to = columns.find(connection.to.node);
            if (from == columns.end() || to == columns.end()) {
                continue;
            }
            const int wanted = std::min(from->second + 1, passes);
            if (to->second < wanted) {
                to->second = wanted;
                changed = true;
            }
        }
        if (!changed) {
            break;
        }
    }
    return columns;
}

}  // namespace

void AutoLayout(graph::Graph& graph, const graph::NodeRegistry& registry) {
    if (graph.Nodes().empty()) {
        return;
    }
    glm::vec2 topLeft(graph.Nodes().front().position);
    for (const graph::Node& node : graph.Nodes()) {
        topLeft = glm::min(topLeft, node.position);
    }

    const std::map<uint32_t, int> columns = ColumnsOf(graph);
    const int columnCount = 1 + std::max_element(columns.begin(), columns.end(), [](const auto& a, const auto& b) { return a.second < b.second; })->second;

    std::vector<std::vector<uint32_t>> nodesInColumn(static_cast<size_t>(columnCount));
    std::vector<float> columnWidth(static_cast<size_t>(columnCount), 0.0f);
    for (const graph::Node& node : graph.Nodes()) {
        const size_t column = static_cast<size_t>(columns.at(node.id));
        nodesInColumn[column].push_back(node.id);
        columnWidth[column] = std::max(columnWidth[column], LayoutNode(graph, registry, node).bounds.Size().x);
    }

    // Each node is placed in turn from the left. Within a column the nodes are ordered by the average height of the nodes that
    // feed them, which are already placed, and by their number for those with none.
    std::map<uint32_t, float> centreY;
    float columnX = topLeft.x;
    for (size_t column = 0; column < nodesInColumn.size(); ++column) {
        std::vector<uint32_t>& ids = nodesInColumn[column];
        auto wantedY = [&](uint32_t id) {
            float sum = 0.0f;
            int count = 0;
            for (const graph::Connection& connection : graph.Connections()) {
                const auto fed = centreY.find(connection.from.node);
                if (connection.to.node == id && fed != centreY.end()) {
                    sum += fed->second;
                    ++count;
                }
            }
            return count > 0 ? sum / static_cast<float>(count) : topLeft.y;
        };
        std::stable_sort(ids.begin(), ids.end(), [&](uint32_t a, uint32_t b) { return wantedY(a) < wantedY(b); });

        float y = topLeft.y;
        for (uint32_t id : ids) {
            graph::Node* node = graph.FindNode(id);
            node->position = {columnX, y};
            const float height = LayoutNode(graph, registry, *node).bounds.Size().y;
            centreY[id] = y + height * 0.5f;
            y += height + kRowGap;
        }
        columnX += columnWidth[column] + kColumnGap;
    }
}

}  // namespace graph_editor
