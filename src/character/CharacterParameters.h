#pragma once

#include <cstdint>

// What a person looks like, in numbers a person would give: the few that decide the shape of the whole body. The
// generator turns them into a skeleton and a skinned mesh (see CharacterGenerator.h). Everything else, such as the exact
// size of a thigh or how the weight sits on the body, follows from them and from the seed.

namespace character {

struct CharacterParameters {
    float masculinity = 0.5f;     // 0 = a female body, 1 = a male one; values between blend the two
    float ageYears = 30.0f;       // adults only, 18 to 80
    float statureMeters = 1.70f;  // standing height, 1.45 to 2.05
    float massKg = 70.0f;         // body mass; limited to a body mass index of 15 to 45 for the stature
    float muscularity = 0.4f;     // 0 = slight, 1 = a trained athlete
    uint32_t seed = 1;            // varies the small proportions that the numbers above leave open
};

// The same parameters with every value inside its range. A value that is not a number is replaced by the default.
CharacterParameters Sanitised(const CharacterParameters& parameters);

// A person drawn from the population: sex, stature, body mass index, age and muscularity follow the usual spread of
// adults (stature by sex after the ANSUR II survey of adults), and the same seed always gives the same person.
CharacterParameters RandomParameters(uint32_t seed);

// Body mass index, kg per square metre of stature.
float BodyMassIndex(const CharacterParameters& parameters);

}  // namespace character
