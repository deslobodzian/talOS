#pragma once

// The 2026 game (REBUILT) on the rebuilt_2026 field model, as GameRules
// (REQ-DEV-40): scoring through each HUB's scoring site, the HUBs' active
// shifts and grace, the HUBs returning FUEL through their exits, preloads at
// the robot's `preload_N` sites, and the match's periods on the schedule
// clock. README.md "Rules" states every rule and number.

#include <memory>

#include "talOS/simulation/mujoco/game.h"

namespace talos::simulation::mujoco {

// Rules for one plant. Attach throws std::invalid_argument naming a FUEL
// range numeric, a site or a FUEL body the field model does not carry.
std::unique_ptr<GameRules> MakeRebuilt2026Rules(const GameOptions& options);

}  // namespace talos::simulation::mujoco
