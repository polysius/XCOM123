#pragma once

#include "world/crowd/CrowdSettings.h"

namespace crowd {

class Walkways;
struct CrowdState;

// Fills `state` with settings.agentCount people spread over the walkways: walkers on the
// pavements (alone or in small groups) and, in the squares, strollers and conversation
// circles. The circles' space is reserved in `walkways` so strollers walk around them.
void SpawnCrowd(const CrowdSettings& settings, Walkways& walkways, CrowdState& state);

}  // namespace crowd
