#pragma once

#include <cstdint>

// How many people the crowd has and how they spend their time.
struct CrowdSettings {
    uint32_t agentCount = 10000;
    uint32_t seed = 7;
    // Fraction of people who spend their time in squares instead of walking the streets.
    // Squares only take as many as fit comfortably; the rest walk the streets.
    float plazaShare = 0.3f;
    // Fraction of street walkers who walk in small groups of two to four.
    float groupShare = 0.3f;
    // Fraction of the people in squares who stand together in conversation circles.
    float chatShare = 0.5f;
};
