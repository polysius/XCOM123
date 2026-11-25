#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"

// What a person looks like beyond the shape of the body: the colour of skin, eyes and hair, and how much the skin shows
// of age and sun. They come from the parameters (age, sex) and from a salted hash of the seed, so that a person is the
// same every time and the numbers of CharacterParameters need not change.

namespace character {

enum class HairStyle : uint8_t {
    Bald,
    Short,   // cropped, a couple of centimetres
    Medium,  // to the ears and the collar
    Long,    // to the shoulder blades
    Tied,    // long hair gathered in a ponytail
};

enum class FacialHair : uint8_t {
    None,
    Stubble,
    Beard,
};

struct PersonTraits {
    // Skin. Colours are linear reflectances.
    float melanin = 0.3f;         // 0 = the palest skin, 1 = the darkest
    glm::vec3 skinColour{0.5f};
    float ruddiness = 0.5f;       // how red the cheeks, nose and ears are
    float freckles = 0.0f;        // 0 to 1: how many
    float moles = 0.0f;           // 0 to 1: how many
    float ageSpots = 0.0f;        // 0 to 1: the brown patches on the face and the backs of the hands
    float wrinkles = 0.0f;        // 0 to 1: how deep the lines of the face, neck and hands are
    float veins = 0.0f;           // 0 to 1: how much the veins of the hands and temples show

    // Eyes and hair.
    glm::vec3 irisColour{0.2f, 0.1f, 0.05f};
    glm::vec3 hairColour{0.05f};     // before greying
    float greyFraction = 0.0f;       // the share of the hairs that have turned grey
    float hairDensity = 1.0f;        // 1 = a full head of hair
    float hairlineRecession = 0.0f;  // 0 to 1: how far the hair has withdrawn from the temples
};

PersonTraits DeriveTraits(const CharacterParameters& parameters);

// A choice of hair style and facial hair that follows the population: the same seed always makes the same choice.
HairStyle ChooseHairStyle(const CharacterParameters& parameters);
FacialHair ChooseFacialHair(const CharacterParameters& parameters);

}  // namespace character
