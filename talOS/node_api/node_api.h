#pragma once

// talOS node_api: the C door into talos::Node, the devices layer and the test
// harness, for nodes written in other languages (the Python `talos` package,
// talOS/python). Each entry point wraps one C++ operation 1:1; no transport,
// gating, describe or codec logic lives on the far side.
//
//   libtalos_node.so          node + devices + talos_run
//   libtalos_node_testing.so  all of the above, plus the Sim / SimHardware
//                             section (testonly)
//
// Rules:
//   - talos_abi_version() changes whenever the exported surface does; a
//     caller refuses a library whose version differs from its own.
//   - No exception crosses this boundary. Failures return TALOS_ERR (or NULL,
//     or a negative index) with the message in talos_last_error().
//   - A callback returns 0 on success. Any other value makes the C++ side
//     throw, which unwinds to the entry point that ran the callback (a
//     talos_sim_* call, or talos_run, which then exits non-zero). The caller
//     keeps its own copy of what went wrong.
//   - Durations and times are integer nanoseconds.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TALOS_NODE_ABI_VERSION 9u

enum {
  TALOS_OK = 0,
  TALOS_ERR = -1,
};

typedef struct TalosNode TalosNode;  // a talos::Node
typedef struct TalosSim TalosSim;    // a talos::testing::Sim
typedef struct TalosSimHardware TalosSimHardware;

typedef int32_t (*TalosCallback)(void* user);
typedef int32_t (*TalosMessageCallback)(const void* data, uint32_t size,
                                        void* user);
// Builds the user's object on `node` (subscribing, publishing, taking
// handles, calling talos_node_set_update).
typedef int32_t (*TalosFactory)(TalosNode* node, void* user);

uint32_t talos_abi_version(void);
// Last error on this thread; valid until the next call. Never NULL.
const char* talos_last_error(void);

// --- running a node ------------------------------------------------------

// talos::Run for a node built by `factory`: flags (--sim --describe --log
// --session-id --config --replay --duration-s), describe, replay, logging.
// Returns the process exit code.
int32_t talos_run(int32_t argc, char** argv, const char* node_name,
                  const char* node_target, TalosFactory factory, void* user);

// --- node ----------------------------------------------------------------

const char* talos_node_name(const TalosNode* node);
int32_t talos_node_simulation(const TalosNode* node);
int64_t talos_node_now_ns(const TalosNode* node);
uint64_t talos_node_tick(const TalosNode* node);
// The update() index state machines stamp a transition with.
uint64_t talos_node_next_update(const TalosNode* node);
int64_t talos_node_period_ns(const TalosNode* node);
const char* talos_node_config_path(const TalosNode* node);

// Parameter kinds. ANY takes the key's own type.
enum {
  TALOS_PARAM_ANY = 0,
  TALOS_PARAM_BOOL = 1,
  TALOS_PARAM_INT = 2,
  TALOS_PARAM_DOUBLE = 3,
  TALOS_PARAM_STRING = 4,
};
typedef struct TalosParam {
  int32_t kind;  // what was found
  int32_t boolean;
  int64_t integer;
  double number;
  const char* string;  // valid until the next call on this thread
} TalosParam;
// A key of this node's subsystem table as `kind`. Returns 1 when found, 0
// when absent and not `required`, TALOS_ERR when absent and required or of
// the wrong type (the error names the node and key).
int32_t talos_node_param(const TalosNode* node, const char* key, int32_t kind,
                         int32_t required, TalosParam* out);

// Registration: call while the factory runs. Indexes are >= 0; -1 is an
// error.
int64_t talos_node_subscribe(TalosNode* node, const char* topic,
                             uint32_t message_bytes, int32_t optional,
                             TalosMessageCallback on_message, void* user);
int64_t talos_node_publish(TalosNode* node, const char* topic,
                           uint32_t message_bytes);
int32_t talos_node_every(TalosNode* node, int64_t period_ns, TalosCallback fn,
                         void* user);
// The update() the framework calls every period.
int32_t talos_node_set_update(TalosNode* node, TalosCallback fn, void* user);

// Subscription `index`: the latest message (zeros until the first), ...
const void* talos_subscription_value(const TalosNode* node, int64_t index);
int32_t talos_subscription_has_value(const TalosNode* node, int64_t index);
int64_t talos_subscription_age_ns(const TalosNode* node, int64_t index);
// max_age_ns < 0: [robot] command_timeout_us.
int32_t talos_subscription_fresh(const TalosNode* node, int64_t index,
                                 int64_t max_age_ns);
int32_t talos_subscription_updated(const TalosNode* node, int64_t index);
// Returns 1 sent, 0 dropped by the transport, TALOS_ERR.
int32_t talos_publisher_send(TalosNode* node, int64_t index, const void* data,
                             uint32_t size);

// --- devices -------------------------------------------------------------

enum {  // talos::detail::DeviceKind
  TALOS_DEVICE_MOTOR = 0,
  TALOS_DEVICE_CANCODER = 1,
  TALOS_DEVICE_GYRO = 2,
  TALOS_DEVICE_DIGITAL_INPUT = 3,
  TALOS_DEVICE_DIGITAL_OUTPUT = 4,
  TALOS_DEVICE_ANALOG_INPUT = 5,
  TALOS_DEVICE_ENCODER = 6,
  TALOS_DEVICE_PWM = 7,
};
enum {  // talos::hardware::Mode
  TALOS_MODE_NEUTRAL = 0,
  TALOS_MODE_DUTY_CYCLE = 1,
  TALOS_MODE_VOLTAGE = 2,
  TALOS_MODE_VELOCITY = 3,
  TALOS_MODE_POSITION = 4,
  TALOS_MODE_MOTION_MAGIC = 5,
};

// A device of this node's subsystem, by its config key ([motors.roller] is
// "roller"). Returns its index, or -1 naming the device in the error.
int64_t talos_device(TalosNode* node, int32_t kind, const char* name);
int32_t talos_robot_enabled(TalosNode* node);

// One motor command for the next flush (Motor::set_* / stop).
int32_t talos_motor_set(TalosNode* node, int64_t index, int32_t mode,
                        double demand, double ff_volts, int32_t slot);
int32_t talos_digital_output_set(TalosNode* node, int64_t index, int32_t value);
int32_t talos_pwm_set(TalosNode* node, int64_t index, double output);

// The latched feedback of device `index`, zeros (valid == 0) until an
// accepted hardware state carries it. Fields a kind lacks stay 0.
typedef struct TalosSample {
  int32_t valid;
  int32_t value;    // digital input level
  double position;  // rotations (gyro: yaw)
  double velocity;  // rotations/s (gyro: yaw rate)
  double voltage;   // applied volts (motor), input volts (analog input)
  double current;   // stator amps (motor)
} TalosSample;
int32_t talos_motor_sample(TalosNode* node, int64_t index, TalosSample* out);
int32_t talos_sensor_sample(TalosNode* node, int64_t index,
                            TalosSample* out);  // CANcoder, gyro
int32_t talos_digital_input_sample(TalosNode* node, int64_t index,
                                   TalosSample* out);
int32_t talos_analog_input_sample(TalosNode* node, int64_t index,
                                  TalosSample* out);
int32_t talos_encoder_sample(TalosNode* node, int64_t index, TalosSample* out);

// --- Driver Station --------------------------------------------------------

#define TALOS_DS_MAX_JOYSTICKS 6
#define TALOS_DS_MAX_AXES 12
#define TALOS_DS_MAX_POVS 4
#define TALOS_DS_NAME_LENGTH 32

enum {  // talos::driver_station::DsFlags
  TALOS_DS_ATTACHED = 1 << 0,
  TALOS_DS_FMS_ATTACHED = 1 << 1,
  TALOS_DS_ENABLED = 1 << 2,
  TALOS_DS_AUTONOMOUS = 1 << 3,
  TALOS_DS_TELEOP = 1 << 4,
  TALOS_DS_TEST = 1 << 5,
  TALOS_DS_ESTOP = 1 << 6,
};

// One joystick port, as talos::Joystick reads it: axes past axis_count are
// 0, povs past pov_count are -1, and bit n-1 of `buttons` is button n.
typedef struct TalosJoystick {
  int32_t connected;
  int32_t axis_count;
  int32_t button_count;
  int32_t pov_count;
  uint32_t buttons;
  float axes[TALOS_DS_MAX_AXES];
  int32_t povs[TALOS_DS_MAX_POVS];
  char name[TALOS_DS_NAME_LENGTH];  // NUL-terminated unless full
} TalosJoystick;

// What a talos::DriverStation handle reports.
typedef struct TalosDriverStation {
  int32_t connected;
  int32_t enabled;
  int32_t autonomous;
  int32_t teleop;
  int32_t test;
  int32_t estop;
  int32_t fms_attached;
  int32_t alliance;  // talos::driver_station::Alliance
  int32_t station;
  double match_time_s;
  TalosJoystick joysticks[TALOS_DS_MAX_JOYSTICKS];
} TalosDriverStation;

// Takes a DriverStation handle on `node`: call while the factory runs.
// `on_packet`, when not NULL, runs as each packet arrives.
int32_t talos_driver_station(TalosNode* node, TalosCallback on_packet,
                             void* user);
// Fills `out` from the node's DriverStation, taken by talos_driver_station.
int32_t talos_driver_station_read(TalosNode* node, TalosDriverStation* out);

// --- test harness (libtalos_node_testing.so only) ------------------------

TalosSim* talos_sim_create(const char* config_path);
TalosSim* talos_sim_from_toml(const char* robot_toml);
void talos_sim_destroy(TalosSim* sim);
// Builds a node named `node_name` with `factory`. NULL on error.
TalosNode* talos_sim_add(TalosSim* sim, const char* node_name,
                         TalosFactory factory, void* user);
TalosNode* talos_sim_node(TalosSim* sim, const char* node_name);
int32_t talos_sim_step(TalosSim* sim, int64_t duration_ns);
int64_t talos_sim_now_ns(const TalosSim* sim);
int32_t talos_sim_publish_raw(TalosSim* sim, const char* topic,
                              const void* data, uint32_t size);
// Copies the newest message on `topic` into `out`: 1 copied, 0 none yet.
int32_t talos_sim_latest_raw(const TalosSim* sim, const char* topic, void* out,
                             uint32_t size);

TalosSimHardware* talos_sim_hardware_create(TalosSim* sim);
void talos_sim_hardware_destroy(TalosSimHardware* hw);
int32_t talos_sim_hardware_enable(TalosSimHardware* hw, int32_t on);
int32_t talos_sim_hardware_ideal_motors(TalosSimHardware* hw, int32_t on);
// Feedback for (node, device), by the names the node uses.
int32_t talos_sim_hardware_set_motor(TalosSimHardware* hw, const char* node,
                                     const char* device, double position,
                                     double velocity);
int32_t talos_sim_hardware_set_digital_input(TalosSimHardware* hw,
                                             const char* node,
                                             const char* device, int32_t value);
int32_t talos_sim_hardware_set_analog_input(TalosSimHardware* hw,
                                            const char* node,
                                            const char* device, double volts);
int32_t talos_sim_hardware_set_encoder(TalosSimHardware* hw, const char* node,
                                       const char* device, double position,
                                       double velocity);
int32_t talos_sim_hardware_set_cancoder(TalosSimHardware* hw, const char* node,
                                        const char* device, double position,
                                        double velocity);
int32_t talos_sim_hardware_set_gyro(TalosSimHardware* hw, const char* node,
                                    const char* device, double yaw,
                                    double yaw_rate);
// Marks every device of `node` named `device` (dis)connected
// (SimHardware::set_connected).
int32_t talos_sim_hardware_set_connected(TalosSimHardware* hw, const char* node,
                                         const char* device, int32_t connected);
// The Driver Station packet to forward from now on
// (SimHardware::set_driver_station); the rest of the packet is zero.
typedef struct TalosDriverStationPacket {
  uint32_t flags;    // TALOS_DS_*
  int32_t alliance;  // talos::driver_station::Alliance
  int32_t station;
  float match_time_s;
  TalosJoystick joysticks[TALOS_DS_MAX_JOYSTICKS];
} TalosDriverStationPacket;
int32_t talos_sim_hardware_set_driver_station(
    TalosSimHardware* hw, const TalosDriverStationPacket* packet);
// The last request flushed by `node` for `device`.
typedef struct TalosMotorRequest {
  int32_t mode;  // TALOS_MODE_*
  int32_t slot;
  double demand;
  double ff_volts;
} TalosMotorRequest;
int32_t talos_sim_hardware_motor_request(const TalosSimHardware* hw,
                                         const char* node, const char* device,
                                         TalosMotorRequest* out);
// Returns 0/1, or TALOS_ERR.
int32_t talos_sim_hardware_digital_output(const TalosSimHardware* hw,
                                          const char* node, const char* device);
int32_t talos_sim_hardware_pwm(const TalosSimHardware* hw, const char* node,
                               const char* device, double* out);

#ifdef __cplusplus
}  // extern "C"
#endif
