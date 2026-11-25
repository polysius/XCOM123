#include "graph/NodeRegistry.h"

namespace graph {

void NodeRegistry::Register(NodeDescription description) {
    const std::string id = description.id;
    kinds[id] = std::move(description);
}

const NodeDescription* NodeRegistry::Find(const std::string& id) const {
    const auto found = kinds.find(id);
    return found != kinds.end() ? &found->second : nullptr;
}

std::vector<const NodeDescription*> NodeRegistry::All() const {
    std::vector<const NodeDescription*> result;
    result.reserve(kinds.size());
    for (const auto& entry : kinds) {
        result.push_back(&entry.second);
    }
    return result;
}

}  // namespace graph
