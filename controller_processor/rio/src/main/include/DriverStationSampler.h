#pragma once

#include <cstdint>

#include "driver_station/driver_station.h"

namespace talos::rio {

// The RoboRIO loop's Driver Station generations and packet cadence
// (Robot.cpp), apart from the socket and the encoding so its tests can drive
// it tick by tick.
//
// Every tick observes the flags the gateway is enabled from, socket open or
// not, so a mode generation (driver_station::ModeTracker) starts on the tick
// that first sees a new mode, and a companion link that drops and returns in
// the same mode keeps its generation. A packet goes out on a tick whose
// socket is open when the mode differs from the last packet's, or
// driver_station::kSamplePeriodUs after the last packet: the Driver Station
// produces new data at 50 Hz, so a packet every hardware tick would only
// repeat it.
class DriverStationSampler {
 public:
  // Every tick, before anything else reads the mode: the flags of the tick's
  // Driver Station view.
  void Observe(uint32_t flags, uint64_t now_us) {
    mode_.Observe(flags, now_us);
  }

  // After the tick's own sends: true when a packet goes out this tick, with
  // `packet` set to `view` sampled at `now_us`, stamped with the mode
  // generation and carrying `boot_id` (the gateway's, as its State). False,
  // and nothing recorded, when the socket is closed or no packet is due.
  bool Sample(const driver_station::DriverStationData& view, uint64_t now_us,
              uint64_t boot_id, bool socket_open,
              driver_station::DriverStationData& packet) {
    const uint32_t mode = driver_station::ModeKey(view.fms.flags);
    const bool sample_due = !has_sent_ || (now_us >= last_sample_us_ &&
                                           now_us - last_sample_us_ >=
                                               driver_station::kSamplePeriodUs);
    if (!socket_open || (mode == last_mode_ && !sample_due)) return false;
    prev_has_sent_ = has_sent_;
    prev_sample_us_ = last_sample_us_;
    prev_mode_ = last_mode_;
    has_sent_ = true;
    last_sample_us_ = now_us;
    last_mode_ = mode;
    // The tick's view, so the packet is one refresh's and agrees with the
    // observed mode and the gateway's enable.
    packet = view;
    packet.sample_time_us = now_us;
    mode_.Stamp(packet);
    // The same boot id as this program's State: a consumer orders packets
    // within one boot and starts over on a new one.
    packet.boot_id = boot_id;
    return true;
  }

  // Undoes the cadence/mode bookkeeping of the immediately preceding Sample()
  // when encoding or sending that packet failed, so the next tick with an open
  // socket retries immediately (preserving the ModeTracker generation).
  void RevertSample() {
    has_sent_ = prev_has_sent_;
    last_sample_us_ = prev_sample_us_;
    last_mode_ = prev_mode_;
  }

 private:
  driver_station::ModeTracker mode_;
  bool has_sent_{false};
  uint64_t last_sample_us_{0};
  // The mode (ModeKey) of the last packet sent.
  uint32_t last_mode_{0};
  bool prev_has_sent_{false};
  uint64_t prev_sample_us_{0};
  uint32_t prev_mode_{0};
};

}  // namespace talos::rio
