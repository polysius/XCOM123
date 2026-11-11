#pragma once

#include <vector>

#include "world/crowd/CrowdTypes.h"

namespace crowd {

// Every person in the crowd, as parallel arrays indexed by person, plus the units that
// group them for updating. Members of a unit are stored next to each other.
struct CrowdState {
    std::vector<AgentMotion> motion;
    std::vector<StreetRoute> routes;  // only meaningful for street walkers and group followers
    std::vector<AgentBehaviour> behaviour;
    std::vector<AgentLook> looks;
    std::vector<SimTier> tiers;
    // Simulation time, in seconds, that has passed since the person was last updated. Far
    // people are updated only every few steps; until then they are drawn extrapolated.
    std::vector<float> lagSeconds;
    std::vector<CrowdUnit> units;

    size_t AgentCount() const { return motion.size(); }

    // Appends one default person and returns its index.
    uint32_t AddAgent() {
        motion.emplace_back();
        routes.emplace_back();
        behaviour.emplace_back();
        looks.emplace_back();
        tiers.push_back(SimTier::Far);
        lagSeconds.push_back(0.0f);
        return static_cast<uint32_t>(motion.size() - 1);
    }

    void Clear() {
        motion.clear();
        routes.clear();
        behaviour.clear();
        looks.clear();
        tiers.clear();
        lagSeconds.clear();
        units.clear();
    }
};

}  // namespace crowd
