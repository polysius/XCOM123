#include "graph_editor/Palette.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace graph_editor {
namespace {

std::string Lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::vector<std::string> Words(const std::string& text) {
    std::vector<std::string> words;
    std::istringstream stream(Lower(text));
    std::string word;
    while (stream >> word) {
        words.push_back(word);
    }
    return words;
}

// How well a word fits a piece of text: 0 when the text starts with it, 1 when a word of the text does, 2 when it is somewhere
// inside, and -1 when it is not there.
int FitOfWord(const std::string& text, const std::string& word) {
    const std::string lower = Lower(text);
    const size_t at = lower.find(word);
    if (at == std::string::npos) {
        return -1;
    }
    if (at == 0) {
        return 0;
    }
    return lower[at - 1] == ' ' ? 1 : 2;
}

// Lower is better. Titles count most, then categories, ids and pins.
constexpr int kNoMatch = 1000;
constexpr int kTitleWeight = 0;
constexpr int kCategoryWeight = 10;
constexpr int kIdWeight = 20;
constexpr int kPinWeight = 30;

int ScoreOfWord(const PaletteEntry& entry, const graph::NodePins& pins, const std::string& word) {
    int best = kNoMatch;
    auto consider = [&](const std::string& text, int weight) {
        const int fit = FitOfWord(text, word);
        if (fit >= 0) {
            best = std::min(best, weight + fit);
        }
    };
    consider(entry.title, kTitleWeight);
    consider(entry.category, kCategoryWeight);
    consider(entry.typeId, kIdWeight);
    for (const std::vector<graph::PinDescription>* list : {&pins.inputs, &pins.outputs}) {
        for (const graph::PinDescription& pin : *list) {
            consider(pin.name, kPinWeight);
            consider(graph::PinTypeName(pin.type), kPinWeight);
        }
    }
    return best;
}

// The first pin of the node that a wire of the wish can end at, or -1.
int FirstPinThatFits(const graph::NodePins& pins, const WireWish& wish) {
    const std::vector<graph::PinDescription>& candidates = wish.fromOutput ? pins.inputs : pins.outputs;
    for (size_t index = 0; index < candidates.size(); ++index) {
        const graph::PinType from = wish.fromOutput ? wish.type : candidates[index].type;
        const graph::PinType to = wish.fromOutput ? candidates[index].type : wish.type;
        const bool bothExec = from == graph::PinType::Exec && to == graph::PinType::Exec;
        const bool bothData = from != graph::PinType::Exec && to != graph::PinType::Exec && graph::CanConvert(from, to);
        if (bothExec || bothData) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

// Every node that can be added: a kind of the registry, or for the kinds that work on a variable, one for each variable.
std::vector<PaletteEntry> AllEntries(const graph::Graph& graph, const graph::NodeRegistry& registry) {
    std::vector<PaletteEntry> entries;
    for (const graph::NodeDescription* description : registry.All()) {
        if (!description->usesVariable) {
            entries.push_back({description->id, description->title, description->category, description->tooltip, {}, -1});
            continue;
        }
        for (const graph::Variable& variable : graph.Variables()) {
            entries.push_back({description->id, description->title + " " + variable.name, description->category, description->tooltip, variable.name, -1});
        }
    }
    return entries;
}

graph::NodePins PinsOfEntry(const graph::Graph& graph, const graph::NodeRegistry& registry, const PaletteEntry& entry) {
    graph::Node probe;
    probe.type = entry.typeId;
    if (!entry.variable.empty()) {
        probe.properties["variable"] = entry.variable;
    }
    return graph::PinsOf(graph, registry, probe);
}

}  // namespace

std::vector<PaletteEntry> SearchPalette(const graph::Graph& graph, const graph::NodeRegistry& registry, const std::string& query,
                                        const std::optional<WireWish>& wish) {
    const std::vector<std::string> words = Words(query);
    struct Scored {
        PaletteEntry entry;
        int score;
    };
    std::vector<Scored> found;
    for (PaletteEntry& entry : AllEntries(graph, registry)) {
        const graph::NodePins pins = PinsOfEntry(graph, registry, entry);
        if (wish) {
            entry.connectPin = FirstPinThatFits(pins, *wish);
            if (entry.connectPin < 0) {
                continue;
            }
        }
        int score = 0;
        for (const std::string& word : words) {
            score += ScoreOfWord(entry, pins, word);
        }
        if (score >= kNoMatch) {
            continue;  // some word fits nothing of this node
        }
        found.push_back({std::move(entry), score});
    }
    std::stable_sort(found.begin(), found.end(), [](const Scored& a, const Scored& b) {
        if (a.score != b.score) {
            return a.score < b.score;
        }
        if (a.entry.category != b.entry.category) {
            return a.entry.category < b.entry.category;
        }
        return a.entry.title < b.entry.title;
    });
    std::vector<PaletteEntry> entries;
    for (Scored& item : found) {
        entries.push_back(std::move(item.entry));
    }
    return entries;
}

}  // namespace graph_editor
