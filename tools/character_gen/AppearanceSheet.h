#pragma once

#include <filesystem>
#include <string>

#include "character/CharacterGenerator.h"
#include "character/appearance/Appearance.h"

// Pictures of a dressed character for character_gen: full-body views and close-ups of the face, drawn by a small software
// renderer that uses what a game renderer would: the baked textures with their normal maps, a key and a fill light, wrapped light
// for skin, a glossy highlight where the roughness map is low (the eyes, the lips), and ribbons for hair, drawn on both sides.
// It also writes the textures themselves as PNG files. No window and no GPU are needed.

namespace appearance_sheet {

// `<name>_dressed.png`: the whole body from the front, the side and three quarters, with its clothes.
// `<name>_face.png`: the head close up from the front, three quarters and the side.
bool WriteViews(const std::filesystem::path& folder, const std::string& name, const character::CharacterMesh& mesh, const character::CharacterAppearance& appearance);

// `<name>_look.png`: one view of any part of the figure, to look at a detail. It is `viewHeight` metres tall, centred on the point
// (x, y, z) of the rest pose, from `turnDegrees` round the figure (0 is the front).
bool WriteLook(const std::filesystem::path& folder, const std::string& name, const character::CharacterAppearance& appearance, glm::vec3 centre, float viewHeight,
               float turnDegrees);

// The skin, the eye and the garment textures as PNG files, for example `<name>_skin_albedo.png`.
bool WriteTextures(const std::filesystem::path& folder, const std::string& name, const character::CharacterAppearance& appearance);

}  // namespace appearance_sheet
