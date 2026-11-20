#pragma once

#include <algorithm>
#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talos::hardware {

inline constexpr std::size_t kMaxMotors = 16;
inline constexpr std::size_t kMaxSensors = 12;
inline constexpr std::size_t kMaxDigitalInputs = 8;
inline constexpr std::size_t kMaxDigitalOutputs = 4;
inline constexpr std::size_t kMaxAnalogInputs = 4;
inline constexpr std::size_t kMaxEncoders = 4;
inline constexpr std::size_t kMaxPwmOutputs = 4;
inline constexpr std::size_t kMaxConfigNameLength = 63;

// Inline NUL-terminated string bounded to `Capacity` bytes so hardware::Config
// copies, deserialization, and reconfiguration on the RoboRIO loop never
// allocate. Assigning more than `Capacity` bytes sets `overflowed()`, which
// Validate() rejects.
template <std::size_t Capacity>
class FixedString {
 public:
  constexpr FixedString() noexcept = default;
  FixedString(const char* s) noexcept { assign(s); }
  FixedString(std::string_view s) noexcept { assign(s); }
  FixedString(const std::string& s) noexcept {
    assign(std::string_view{s.data(), s.size()});
  }

  FixedString& operator=(const char* s) noexcept {
    assign(s);
    return *this;
  }
  FixedString& operator=(std::string_view s) noexcept {
    assign(s);
    return *this;
  }
  FixedString& operator=(const std::string& s) noexcept {
    assign(std::string_view{s.data(), s.size()});
    return *this;
  }
  FixedString& operator+=(std::string_view s) noexcept {
    append(s);
    return *this;
  }
  void append(std::string_view s) noexcept {
    if (s.size() > Capacity - size_) overflow_ = true;
    const std::size_t add = std::min(s.size(), Capacity - size_);
    if (add > 0) std::memcpy(data_.data() + size_, s.data(), add);
    size_ += add;
    data_[size_] = '\0';
  }

  void assign(const char* s) noexcept {
    assign(s == nullptr ? std::string_view{} : std::string_view{s});
  }
  void assign(const char* s, std::size_t len) noexcept {
    assign(s == nullptr ? std::string_view{} : std::string_view{s, len});
  }
  void assign(std::string_view s) noexcept {
    overflow_ = s.size() > Capacity;
    size_ = std::min(s.size(), Capacity);
    if (size_ > 0) std::memcpy(data_.data(), s.data(), size_);
    data_[size_] = '\0';
  }
  void assign(std::size_t count, char ch) noexcept {
    overflow_ = count > Capacity;
    size_ = std::min(count, Capacity);
    if (size_ > 0)
      std::memset(data_.data(), static_cast<unsigned char>(ch), size_);
    data_[size_] = '\0';
  }
  void clear() noexcept {
    size_ = 0;
    overflow_ = false;
    data_[0] = '\0';
  }

  constexpr std::string_view view() const noexcept {
    return {data_.data(), size_};
  }
  constexpr operator std::string_view() const noexcept { return view(); }
  std::string str() const { return std::string(data_.data(), size_); }
  constexpr const char* c_str() const noexcept { return data_.data(); }
  constexpr const char* data() const noexcept { return data_.data(); }
  constexpr std::size_t size() const noexcept { return size_; }
  constexpr std::size_t length() const noexcept { return size_; }
  constexpr bool empty() const noexcept { return size_ == 0 && !overflow_; }
  constexpr bool overflowed() const noexcept { return overflow_; }
  constexpr char operator[](std::size_t i) const noexcept { return data_[i]; }
  constexpr const char* begin() const noexcept { return data_.data(); }
  constexpr const char* end() const noexcept { return data_.data() + size_; }

  bool ends_with(std::string_view suffix) const noexcept {
    return view().ends_with(suffix);
  }
  std::size_t find(std::string_view s, std::size_t pos = 0) const noexcept {
    return view().find(s, pos);
  }
  std::string substr(std::size_t pos = 0,
                     std::size_t count = std::string_view::npos) const {
    return std::string(view().substr(pos, count));
  }

  bool operator==(const FixedString& other) const noexcept {
    return overflow_ == other.overflow_ && view() == other.view();
  }
  friend bool operator==(const FixedString& a, std::string_view b) noexcept {
    return !a.overflow_ && a.view() == b;
  }
  friend bool operator==(const FixedString& a, const char* b) noexcept {
    return !a.overflow_ && b != nullptr && a.view() == b;
  }
  friend bool operator==(const FixedString& a, const std::string& b) noexcept {
    return !a.overflow_ && a.view() == b;
  }
  friend auto operator<=>(const FixedString& a, const FixedString& b) noexcept {
    return a.view() <=> b.view();
  }
  friend std::string operator+(const std::string& a, const FixedString& b) {
    std::string out;
    out.reserve(a.size() + b.size_);
    out.append(a);
    out.append(b.data(), b.size_);
    return out;
  }
  friend std::string operator+(const FixedString& a, const std::string& b) {
    std::string out;
    out.reserve(a.size_ + b.size());
    out.append(a.data(), a.size_);
    out.append(b);
    return out;
  }
  friend std::string operator+(const char* a, const FixedString& b) {
    const std::string_view av = a == nullptr ? std::string_view{} : a;
    std::string out;
    out.reserve(av.size() + b.size_);
    out.append(av);
    out.append(b.data(), b.size_);
    return out;
  }
  friend std::string operator+(const FixedString& a, const char* b) {
    const std::string_view bv = b == nullptr ? std::string_view{} : b;
    std::string out;
    out.reserve(a.size_ + bv.size());
    out.append(a.data(), a.size_);
    out.append(bv);
    return out;
  }
  friend std::string operator+(std::string_view a, const FixedString& b) {
    std::string out;
    out.reserve(a.size() + b.size_);
    out.append(a);
    out.append(b.data(), b.size_);
    return out;
  }
  friend std::string operator+(const FixedString& a, std::string_view b) {
    std::string out;
    out.reserve(a.size_ + b.size());
    out.append(a.data(), a.size_);
    out.append(b);
    return out;
  }
  friend std::ostream& operator<<(std::ostream& os, const FixedString& s) {
    return os << s.view();
  }

 private:
  std::array<char, Capacity + 1> data_{};
  std::size_t size_{0};
  bool overflow_{false};
};

using ConfigString = FixedString<kMaxConfigNameLength>;

// Inline bounded vector with std::vector-compatible interface so Config and
// Gateway hold device lists without heap allocations on the RoboRIO loop.
template <typename T, std::size_t Capacity>
class FixedVector {
 public:
  static constexpr std::size_t kCapacity = Capacity;
  using value_type = T;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using reference = T&;
  using const_reference = const T&;
  using iterator = T*;
  using const_iterator = const T*;

  constexpr FixedVector() noexcept = default;
  FixedVector(std::initializer_list<T> init) {
    if (init.size() > Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    for (const auto& item : init) data_[size_++] = item;
  }
  FixedVector& operator=(std::initializer_list<T> init) {
    if (init.size() > Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    std::size_t i = 0;
    for (const auto& item : init) data_[i++] = item;
    for (; i < size_; ++i) data_[i] = T{};
    size_ = init.size();
    return *this;
  }
  FixedVector(const FixedVector& other) : size_{other.size_} {
    for (std::size_t i = 0; i < size_; ++i) data_[i] = other.data_[i];
  }
  FixedVector& operator=(const FixedVector& other) {
    if (this == &other) return *this;
    for (std::size_t i = 0; i < other.size_; ++i) data_[i] = other.data_[i];
    for (std::size_t i = other.size_; i < size_; ++i) data_[i] = T{};
    size_ = other.size_;
    return *this;
  }
  FixedVector(FixedVector&& other) noexcept(
      std::is_nothrow_move_assignable_v<T>)
      : size_{other.size_} {
    for (std::size_t i = 0; i < size_; ++i)
      data_[i] = std::move(other.data_[i]);
  }
  FixedVector& operator=(FixedVector&& other) noexcept(
      std::is_nothrow_move_assignable_v<T>) {
    if (this == &other) return *this;
    for (std::size_t i = 0; i < other.size_; ++i) {
      data_[i] = std::move(other.data_[i]);
    }
    for (std::size_t i = other.size_; i < size_; ++i) data_[i] = T{};
    size_ = other.size_;
    return *this;
  }

  constexpr std::size_t size() const noexcept { return size_; }
  constexpr bool empty() const noexcept { return size_ == 0; }
  constexpr std::size_t capacity() const noexcept { return Capacity; }
  constexpr std::size_t max_size() const noexcept { return Capacity; }
  constexpr T* data() noexcept { return data_.data(); }
  constexpr const T* data() const noexcept { return data_.data(); }
  constexpr iterator begin() noexcept { return data_.data(); }
  constexpr iterator end() noexcept { return data_.data() + size_; }
  constexpr const_iterator begin() const noexcept { return data_.data(); }
  constexpr const_iterator end() const noexcept { return data_.data() + size_; }
  constexpr const_iterator cbegin() const noexcept { return data_.data(); }
  constexpr const_iterator cend() const noexcept {
    return data_.data() + size_;
  }
  constexpr T& front() noexcept { return data_[0]; }
  constexpr const T& front() const noexcept { return data_[0]; }
  constexpr T& back() noexcept { return data_[size_ - 1]; }
  constexpr const T& back() const noexcept { return data_[size_ - 1]; }
  constexpr T& operator[](std::size_t i) noexcept { return data_[i]; }
  constexpr const T& operator[](std::size_t i) const noexcept {
    return data_[i];
  }
  T& at(std::size_t i) {
    if (i >= size_) throw std::out_of_range("FixedVector::at");
    return data_[i];
  }
  const T& at(std::size_t i) const {
    if (i >= size_) throw std::out_of_range("FixedVector::at");
    return data_[i];
  }

  void clear() noexcept {
    for (std::size_t i = 0; i < size_; ++i) data_[i] = T{};
    size_ = 0;
  }
  void reserve(std::size_t n) {
    if (n > Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
  }
  void resize(std::size_t n) {
    if (n > Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    for (std::size_t i = size_; i < n; ++i) data_[i] = T{};
    for (std::size_t i = n; i < size_; ++i) data_[i] = T{};
    size_ = n;
  }
  void push_back(const T& value) {
    if (size_ >= Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    data_[size_++] = value;
  }
  void push_back(T&& value) {
    if (size_ >= Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    data_[size_++] = std::move(value);
  }
  template <typename... Args>
  T& emplace_back(Args&&... args) {
    if (size_ >= Capacity) {
      throw std::invalid_argument("invalid hardware device count");
    }
    data_[size_] = T{std::forward<Args>(args)...};
    return data_[size_++];
  }
  void pop_back() noexcept {
    if (size_ == 0) return;
    data_[--size_] = T{};
  }
  void erase_value(const T& value) noexcept {
    std::size_t out = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      if (!(data_[i] == value)) {
        if (out != i) data_[out] = std::move(data_[i]);
        ++out;
      }
    }
    for (std::size_t i = out; i < size_; ++i) data_[i] = T{};
    size_ = out;
  }
  template <typename Pred>
  void erase_if(Pred pred) {
    std::size_t out = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      if (!pred(data_[i])) {
        if (out != i) data_[out] = std::move(data_[i]);
        ++out;
      }
    }
    for (std::size_t i = out; i < size_; ++i) data_[i] = T{};
    size_ = out;
  }

  bool operator==(const FixedVector& other) const {
    return size_ == other.size_ && std::equal(begin(), end(), other.begin());
  }

 private:
  std::array<T, Capacity> data_{};
  std::size_t size_{0};
};
enum class SensorKind : uint8_t { kCANcoder = 1, kPigeon2 = 2 };
enum class Feedback : uint8_t { kRotor = 0, kRemoteCANcoder = 1 };
enum class GravityType : uint8_t { kElevatorStatic = 0, kArmCosine = 1 };
// A switch wired to the TalonFX's own forward or reverse limit input. While it
// is asserted (a normally open switch closed, a normally closed one open) the
// TalonFX itself holds output toward that direction neutral, in every control
// mode and whatever the position frame; output away from it still drives.
// kNormallyOpen, enabled, is Phoenix's default for both inputs.
enum class LimitSwitch : uint8_t {
  kNormallyOpen = 0,
  kNormallyClosed = 1,
  kDisabled = 2
};

struct Gains {
  double p{}, i{}, d{}, s{}, v{}, a{}, g{};
  GravityType gravity_type{GravityType::kElevatorStatic};
  bool operator==(const Gains&) const = default;
};

// A motor setting only the main processor reads. It is not on the config
// wire, so it compares equal whatever its value: Decode(Encode(c)) == c still
// holds, and the gateway's same-config shortcut (Reconfigure) is unaffected.
struct LocalMotorSetting {
  bool value{false};
  bool operator==(const LocalMotorSetting&) const { return true; }
};

// The motor a TalonFX drives. Only the physics simulator reads it, for its
// winding model (sim_physics.h MotorModelFor).
enum class MotorType : uint8_t { kKrakenX60 = 0, kKrakenX44, kFalcon500 };

// A motor type the RoboRIO never reads: like LocalMotorSetting, not on the
// config wire and equal whatever its value.
struct LocalMotorType {
  MotorType value{MotorType::kKrakenX60};
  bool operator==(const LocalMotorType&) const { return true; }
};

struct MotorConfig {
  uint16_t id{};  // Logical ID, independent of CAN address and array index.
  ConfigString name;
  int can_id{};
  ConfigString bus{"rio"};
  bool inverted{false};
  bool brake{true};
  double supply_limit_a{40};
  double stator_limit_a{80};
  double max_voltage{12};
  Feedback feedback{Feedback::kRotor};
  uint16_t feedback_sensor_id{};
  double rotor_to_sensor_ratio{1};
  double sensor_to_mechanism_ratio{1};
  bool continuous_wrap{false};
  bool soft_limits{false};
  double reverse_limit_rot{-1}, forward_limit_rot{1};
  LimitSwitch forward_limit_switch{LimitSwitch::kNormallyOpen};
  LimitSwitch reverse_limit_switch{LimitSwitch::kNormallyOpen};
  double max_velocity_rps{100};
  double cruise_velocity_rps{20}, acceleration_rps2{40}, jerk_rps3{0};
  std::array<Gains, 3> slots{};
  uint16_t master_id{0};  // Zero denotes an independent motor.
  bool oppose_master{false};
  // [motors.*] foc: FOC commutation (Phoenix EnableFOC) on every duty-cycle,
  // voltage, velocity, position and motion-magic request the RoboRIO sends
  // this motor. Needs a Phoenix Pro licence, which the RoboRIO checks at
  // configure by reading the device's IsProLicensed signal (whether a
  // CANivore's licence shows there is unconfirmed). On the config wire and in
  // ConfigurationId; refused on a follower, which copies its master's output.
  bool foc{false};
  // [motors.*] max_torque_current_a: the largest stator current a
  // torque-current demand may ask for, amperes; 0 (the default) refuses the
  // mode. Needs foc, at most stator_limit_a, refused on a follower and on a
  // motor with one (the mode drives a single motor). On the config wire and
  // in ConfigurationId.
  double max_torque_current_a{0};
  // Main processor only (talOS/devices), not sent to the controller and not
  // part of ConfigurationId: [motors.*] position_frame = "absolute" (true)
  // or "rezero" (false, the default). A rezero-frame motor's position and
  // motion-magic demands go out neutral until the owning node's
  // Motor::rezero holds, and again once a device reset or gateway boot drops
  // it. An absolute-frame motor (remote CANcoder feedback only) commands
  // positions in the sensor's frame, which survives a reset, from its first
  // state, and keeps its rezero across resets.
  LocalMotorSetting absolute_position_frame{};
  // [motors.*] motor_type = "kraken_x60" (the default), "kraken_x44" or
  // "falcon_500": the physics simulator's stall current and free speed. Not
  // sent to the controller and not part of ConfigurationId; the physics
  // backend inside sim_gateway takes it from sim_gateway's own parse of the
  // robot config, since a pushed config carries none
  // (sim::PhysicsSimBackend::SetLocalMotorTypes).
  LocalMotorType motor_type{};
  bool operator==(const MotorConfig&) const = default;
};

struct SensorConfig {
  uint16_t id{};
  ConfigString name;
  SensorKind kind{SensorKind::kCANcoder};
  int can_id{};
  ConfigString bus{"rio"};
  bool inverted{false};
  // CANcoder magnet offset, or Pigeon mount yaw added before inversion.
  double offset_rot{0};  // rotations, [-0.5, 0.5)
  bool operator==(const SensorConfig&) const = default;
};

// A simulated home switch on a digital input: the physics simulator reads it
// closed while `motor`'s reported position (mechanism rotations, hardware
// frame) is at or below `closed_at_rot` (at or above when `closed_above`),
// and reads `closed_reads` while closed. Like LocalMotorType, not on the
// config wire and equal whatever its value.
struct LocalHomeSwitch {
  std::string motor;  // Empty: no simulated switch.
  double closed_at_rot{};
  bool closed_above{false};
  bool closed_reads{false};
  bool operator==(const LocalHomeSwitch&) const { return true; }
};

struct DigitalInputConfig {
  uint16_t id{};
  ConfigString name;
  int dio{};
  // [digital_inputs.*] sim_motor, sim_closed_at_rot, sim_closed_above and
  // sim_closed_reads. Not sent to the controller and not part of
  // ConfigurationId; sim_gateway --physics hands them to the physics backend
  // from its own parse of the robot config
  // (sim::PhysicsSimBackend::SetHomeSwitches).
  LocalHomeSwitch sim_home_switch{};
  bool operator==(const DigitalInputConfig&) const = default;
};

struct DigitalOutputConfig {
  uint16_t id{};
  ConfigString name;
  int dio{};
  bool operator==(const DigitalOutputConfig&) const = default;
};

struct AnalogInputConfig {
  uint16_t id{};
  ConfigString name;
  int channel{};
  bool operator==(const AnalogInputConfig&) const = default;
};

enum class EncoderKind : uint8_t { kQuadrature = 1, kDutyCycle = 2 };

struct EncoderConfig {
  uint16_t id{};
  ConfigString name;
  EncoderKind kind{EncoderKind::kQuadrature};
  int dio_a{};
  int dio_b{-1};
  double counts_per_rev{2048};
  double offset_rot{0};
  bool operator==(const EncoderConfig&) const = default;
};

struct PwmConfig {
  uint16_t id{};
  ConfigString name;
  int channel{};
  bool operator==(const PwmConfig&) const = default;
};

struct Devices {
  std::string subsystem;
  std::vector<uint16_t> motors;
  std::vector<uint16_t> sensors;
  std::vector<uint16_t> digital_inputs;
  std::vector<uint16_t> digital_outputs;
  std::vector<uint16_t> analog_inputs;
  std::vector<uint16_t> encoders;
  std::vector<uint16_t> pwm_outputs;
  // Whether this subsystem drives an output (motor, digital output or PWM
  // output): the bridge merges and times out its requests, the configuration
  // bounds its period_us, and the launcher treats it as an actuator owner. A
  // new output kind joins here first.
  bool owns_actuators() const {
    return !motors.empty() || !digital_outputs.empty() || !pwm_outputs.empty();
  }
  bool operator==(const Devices&) const = default;
};

struct Config {
  // Capacity exceeds kMaxMotors so callers that inspect unvalidated configs
  // (e.g. CAN bus load estimation tests) can hold > kMaxMotors entries before
  // Validate() / DeserializeConfig() enforces the kMaxMotors wire limit.
  FixedVector<MotorConfig, kMaxMotors * 2> motors;
  FixedVector<SensorConfig, kMaxSensors> sensors;
  FixedVector<DigitalInputConfig, kMaxDigitalInputs> digital_inputs;
  FixedVector<DigitalOutputConfig, kMaxDigitalOutputs> digital_outputs;
  FixedVector<AnalogInputConfig, kMaxAnalogInputs> analog_inputs;
  FixedVector<EncoderConfig, kMaxEncoders> encoders;
  FixedVector<PwmConfig, kMaxPwmOutputs> pwm_outputs;
  uint32_t period_us{
      5000};  // 200 Hz setpoints/telemetry, PID stays on TalonFX.
  uint32_t command_timeout_us{100000};
  double status_hz{200};
  bool commissioned{false};
  bool operator==(const Config&) const = default;
};

// Throws before any device is opened. Unknown references and duplicate names,
// logical IDs, or CAN addresses on a bus are rejected, and so is a config
// naming more than kMaxCanBuses - 1 CAN buses besides "rio" or two that
// differ only by case (CanBuses). Allocates nothing on a valid config.
void Validate(const Config& config);
// Non-throwing, allocation-free form of Validate() for the RoboRIO loop.
bool IsValid(const Config& config) noexcept;
uint64_t ConfigurationId(const Config& config);
// Non-throwing, allocation-free form of ConfigurationId(): returns 0 when
// IsValid(config) is false.
uint64_t TryConfigurationId(const Config& config) noexcept;

// The CAN buses whose health State carries (HealthSample::can_bus) and
// /hw/status reports, one slot each.
// Slot 0 is "rio" always; then each other distinct bus a motor or sensor
// names, in byte order. config_id covers bus names, so slot i is the same
// bus on both ends; changing this order needs a protocol bump.
inline constexpr std::size_t kMaxCanBuses = 4;
struct CanBusList {
  std::array<std::string_view, kMaxCanBuses> names{};
  std::size_t count{};
};
// The names view `config`'s own strings ("rio" excepted), so they live as
// long as `config` does not change. Throws std::invalid_argument naming the
// buses when there are more than kMaxCanBuses - 1 besides "rio", or two names
// (with "rio") that differ only by ASCII case, naming a device on each.
CanBusList CanBuses(const Config& config);
}  // namespace talos::hardware
