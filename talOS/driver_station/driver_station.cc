#include "driver_station.h"

#include <bit>
#include <cmath>
#include <cstring>
#include <span>

namespace talos::driver_station {

static_assert(sizeof(float) == 4);

namespace {
struct Writer {
  std::span<uint8_t> data;
  std::size_t pos{};
  bool ok{true};

  void Int(uint64_t value, int size) {
    for (int i = 0; i < size; ++i) {
      if (pos == data.size()) {
        ok = false;
        return;
      }
      data[pos++] = static_cast<uint8_t>(value);
      value >>= 8;
    }
  }

  void Float(float x) {
    if (!std::isfinite(x)) x = 0.0f;
    Int(std::bit_cast<uint32_t>(x), 4);
  }

  void Str(const char* str, std::size_t max_len) {
    // One short of max_len: the reader reserves the last byte for the
    // terminator and rejects any longer length outright.
    std::size_t len = 0;
    while (len + 1 < max_len && str[len] != '\0') ++len;
    Int(len, 1);
    for (std::size_t i = 0; i < len; ++i) {
      if (pos == data.size()) {
        ok = false;
        return;
      }
      data[pos++] = static_cast<uint8_t>(str[i]);
    }
  }
};

struct Reader {
  std::span<const uint8_t> data;
  std::size_t pos{};
  bool ok{true};

  uint64_t Int(int size) {
    uint64_t value{};
    for (int i = 0; i < size; ++i) {
      if (pos == data.size()) {
        ok = false;
        return 0;
      }
      value |= uint64_t{data[pos++]} << (i * 8);
    }
    return value;
  }

  float Float() {
    const float x = std::bit_cast<float>(static_cast<uint32_t>(Int(4)));
    ok &= std::isfinite(x);
    return x;
  }

  void Str(char* out, std::size_t max_len) {
    std::memset(out, 0, max_len);
    uint8_t len = static_cast<uint8_t>(Int(1));
    if (len >= max_len) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < len; ++i) {
      if (pos == data.size()) {
        ok = false;
        return;
      }
      out[i] = static_cast<char>(data[pos++]);
    }
    out[len] = '\0';
  }

  bool Done() const { return ok && pos == data.size(); }
};
}  // namespace

std::size_t Encode(const DriverStationData& ds, std::span<uint8_t> data) {
  Writer w{data};
  w.Int(ds.sample_time_us, 8);

  // FMS
  w.Int(ds.fms.flags, 4);
  w.Int(static_cast<uint8_t>(ds.fms.alliance), 1);
  w.Int(ds.fms.station, 1);
  w.Int(static_cast<uint8_t>(ds.fms.match_type), 1);
  w.Int(ds.fms.match_number, 2);
  w.Int(ds.fms.replay_number, 1);
  w.Float(ds.fms.match_time_s);
  w.Str(ds.fms.game_specific_message, kMaxGameMessageLength);
  w.Str(ds.fms.event_name, kMaxEventNameLength);

  // Joysticks
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    const auto& stick = ds.joysticks[i];
    w.Int(stick.connected ? 1 : 0, 1);
    if (!stick.connected) continue;

    uint8_t axis_count = std::min<uint8_t>(stick.axis_count, kMaxAxes);
    uint8_t button_count = std::min<uint8_t>(stick.button_count, 32);
    uint8_t pov_count = std::min<uint8_t>(stick.pov_count, kMaxPovs);

    w.Int(axis_count, 1);
    w.Int(button_count, 1);
    w.Int(pov_count, 1);
    w.Int(stick.buttons, 4);

    for (std::size_t a = 0; a < axis_count; ++a) {
      w.Float(stick.axes[a]);
    }
    for (std::size_t p = 0; p < pov_count; ++p) {
      w.Int(static_cast<uint16_t>(stick.povs[p]), 2);
    }
    w.Str(stick.name, kMaxNameLength);
  }

  return w.ok ? w.pos : 0;
}

bool Decode(std::span<const uint8_t> data, DriverStationData& out) {
  DriverStationData ds{};
  Reader r{data};

  ds.sample_time_us = r.Int(8);

  // FMS
  ds.fms.flags = static_cast<uint32_t>(r.Int(4));
  ds.fms.alliance = static_cast<Alliance>(r.Int(1));
  ds.fms.station = static_cast<uint8_t>(r.Int(1));
  ds.fms.match_type = static_cast<MatchType>(r.Int(1));
  ds.fms.match_number = static_cast<uint16_t>(r.Int(2));
  ds.fms.replay_number = static_cast<uint8_t>(r.Int(1));
  ds.fms.match_time_s = r.Float();
  r.Str(ds.fms.game_specific_message, kMaxGameMessageLength);
  r.Str(ds.fms.event_name, kMaxEventNameLength);

  // Joysticks
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    auto& stick = ds.joysticks[i];
    stick.connected = (r.Int(1) == 1);
    if (!stick.connected) continue;

    stick.axis_count = static_cast<uint8_t>(r.Int(1));
    stick.button_count = static_cast<uint8_t>(r.Int(1));
    stick.pov_count = static_cast<uint8_t>(r.Int(1));

    if (stick.axis_count > kMaxAxes || stick.button_count > 32 ||
        stick.pov_count > kMaxPovs) {
      return false;
    }

    stick.buttons = static_cast<uint32_t>(r.Int(4));

    for (std::size_t a = 0; a < stick.axis_count; ++a) {
      stick.axes[a] = r.Float();
    }
    for (std::size_t p = 0; p < stick.pov_count; ++p) {
      stick.povs[p] = static_cast<int16_t>(r.Int(2));
    }
    r.Str(stick.name, kMaxNameLength);
  }

  if (!r.Done()) return false;
  out = ds;
  return true;
}

}  // namespace talos::driver_station
