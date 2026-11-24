#pragma once

// A field's game: the WorldRules that referee it (REQ-DEV-40), what they
// report, and the random stream their draws come from. A game is a package
// beside its field model (fields/<game>/rules.h); the gateway's field table
// names its factory.

#include <array>
#include <cstdint>
#include <memory>
#include <string_view>

#include "sim/sim_generated.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {

struct GameOptions {
  uint64_t seed{1};  // [simulation.mujoco] seed
};

// What the gateway stamps into every Driver Station packet until the next
// tick.
struct FmsReport {
  float match_time_s{0.0f};
  // driver_station::kMaxGameMessageLength
  std::array<char, 32> game_specific_message{};
};

class GameRules : public WorldRules {
 public:
  // The match after the last Step's rules: every field but sample_time_ns,
  // which the caller owns and Report leaves as it was. shots_count is the
  // scripted shooter's, which no game owns: Report writes 0 there and the
  // gateway fills it.
  virtual void Report(::sim::MatchState& match) const = 0;
  virtual FmsReport Fms() const = 0;
  // Whether the model's body `body` (a free body: a game piece) is in play
  // after the last Step's rules: false for one the game holds out of play
  // (scored and parked off the field, or a reserve never placed), which
  // Studio does not draw (REQ-DEV-43). A body the game does not referee is
  // in play. Any body id of the attached model; no allocation.
  virtual bool InPlay(int /*body*/) const { return true; }
};

struct FieldEntry {
  std::string_view name;       // [simulation.mujoco] field
  std::string_view rlocation;  // the field model's runfile
  // null: a field with no game
  std::unique_ptr<GameRules> (*make_rules)(const GameOptions&);
};

// A game's random stream: splitmix64 (Steele, Lea and Flood 2014), seeded by
// GameOptions::seed. Its outputs are fixed by the algorithm, not by a
// standard library, so a seed draws the same on every host; a game draws in
// an order it documents, and changing that order changes every run.
class SplitMix64 {
 public:
  explicit SplitMix64(uint64_t seed) : state_(seed) {}
  uint64_t Next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state_;
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31U);
  }
  // Uniform in [0, 1): the top 53 bits of Next() times 2^-53.
  double Uniform() {
    constexpr double kScale = 1.0 / static_cast<double>(1ULL << 53U);
    return static_cast<double>(Next() >> 11U) * kScale;
  }

 private:
  uint64_t state_;
};

}  // namespace talos::simulation::mujoco
