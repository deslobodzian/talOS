#pragma once

#include <span>

#include "config.h"
#include "messages.h"

namespace talos::hardware {
// Platform-specific code owns device objects. Configure is startup-only and
// may block; Read/Apply must use cached or nonblocking device APIs.
class Backend {
 public:
  virtual ~Backend() = default;
  virtual bool Configure(const Config&) = 0;
  virtual bool Read(State&) = 0;
  virtual bool Apply(std::span<const MotorRequest>) = 0;
  virtual bool ApplyOutputs(std::span<const DigitalOutputRequest> /*digital_outputs*/,
                            std::span<const PwmRequest> /*pwm_outputs*/) {
    return true;
  }
  virtual void Neutral() = 0;
};

class Gateway {
 public:
  Gateway(Config config, Backend& backend, uint64_t boot_id);
  Gateway(Backend& backend, uint64_t boot_id);
  bool Reconfigure(const Config& config);
  void Tick(uint64_t now_us, bool driver_station_enabled);
  bool Accept(const Command& command, uint64_t sequence, uint64_t now_us);
  State snapshot() const;
  const Config& config() const { return config_; }
  bool configured() const { return configured_; }

 private:
  void Invalidate();
  Config config_;
  Backend& backend_;
  State state_{};
  bool configured_{false}, enabled_{false}, active_{false}, fault_{false};
  bool have_sequence_{false};
  bool healthy_{false};
  uint64_t accepted_at_{}, observed_at_{};
};
}  // namespace talos::hardware
