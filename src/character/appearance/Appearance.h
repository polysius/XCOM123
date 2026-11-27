#pragma once

#include "character/CharacterGenerator.h"
#include "character/appearance/Clothing.h"
#include "character/appearance/Eyes.h"
#include "character/appearance/Hair.h"
#include "character/appearance/Skin.h"
#include "character/appearance/Traits.h"

class JobSystem;

// How a character looks, as one result next to the CharacterMesh it was made for: skin with its textures, eyes with lids and
// lashes, hair, eyebrows and facial hair, and clothes. The CharacterMesh is not changed. Every part is skinned to the same
// skeleton, in the rest pose of the mesh; the eyes and hair follow the head bone, the clothes follow the bones of the body they
// were cut from.
//
// All of it is deterministic: the same parameters, seed and settings give the same appearance, on any number of threads.

namespace character {

struct AppearanceSettings {
    HairStyle hairStyle = HairStyle::Short;
    FacialHair facialHair = FacialHair::None;
    OutfitSettings outfit;
    int skinTileSize = 512;  // see SkinSettings
    int eyeTextureSize = 512;
    float hairDetail = 1.0f;  // the share of the usual number of hair cards (see HairSettings)
};

// The same appearance made cheaply, for a figure that is seen from far away: 128 texel tiles, 128 texel eyes and 40 percent of the hair
// cards. The shapes, the clothes and the hair style are the same.
AppearanceSettings LowDetail(const AppearanceSettings& settings);

// Settings that suit the person: a hair style and facial hair as the population has them for this age and sex, and an outfit.
// The same parameters always give the same settings.
AppearanceSettings ChooseAppearance(const CharacterParameters& parameters);

struct CharacterAppearance {
    PersonTraits traits;
    AppearanceSettings settings;
    SkinResult skin;
    EyeSet eyes;
    HairSet hair;
    Outfit outfit;
};

// `jobs` may be null, which runs on the calling thread; the result is the same either way. A character made without a head piece
// (MeshSettings::faceAndHands false) gets no eyes and no hair.
CharacterAppearance GenerateAppearance(const CharacterMesh& character, const AppearanceSettings& settings, JobSystem* jobs = nullptr);

}  // namespace character
