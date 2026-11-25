#include "character/appearance/Appearance.h"
#include "core/JobSystem.h"

int main() {
    character::CharacterParameters p;
    const character::CharacterMesh mesh = character::GenerateCharacter(p);
    const character::CharacterAppearance a = character::GenerateAppearance(mesh, character::ChooseAppearance(p));
    return a.skin.mesh.positions.empty() ? 1 : 0;
}
