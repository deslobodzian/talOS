"""ctypes binding to libtalos_node (talOS/node_api/node_api.h).

Framework-internal. Every function here is one C entry point; failures
raise, with the C++ message, or with the Python exception a callback raised.
"""

import ctypes
import os
import threading

_EXPECTED_ABI_VERSION = 26
_LIBRARIES = ("libtalos_node_testing.so", "libtalos_node.so")

c_int32, c_int64, c_uint32, c_uint64 = (
    ctypes.c_int32, ctypes.c_int64, ctypes.c_uint32, ctypes.c_uint64)
c_double, c_float, c_char_p, c_void_p = (
    ctypes.c_double, ctypes.c_float, ctypes.c_char_p, ctypes.c_void_p)

CALLBACK = ctypes.CFUNCTYPE(c_int32, c_void_p)
MESSAGE_CALLBACK = ctypes.CFUNCTYPE(c_int32, c_void_p, c_uint32, c_void_p)
FACTORY = ctypes.CFUNCTYPE(c_int32, c_void_p, c_void_p)

# The C ABI's enum and #define constants, without the TALOS_ prefix. The one
# Python copy: api_parity_test compares each with node_api.h, and node_api.cc
# static_asserts the header against the C++ enums.
OK = 0
ERR = -1
PARAM_ANY = 0
PARAM_BOOL = 1
PARAM_INT = 2
PARAM_DOUBLE = 3
PARAM_STRING = 4
PARAM_STRING_LIST = 5
PARAM_DOUBLE_LIST = 6
PARAM_LIST = 7
DEVICE_MOTOR = 0
DEVICE_CANCODER = 1
DEVICE_GYRO = 2
DEVICE_DIGITAL_INPUT = 3
DEVICE_DIGITAL_OUTPUT = 4
DEVICE_ANALOG_INPUT = 5
DEVICE_ENCODER = 6
DEVICE_PWM = 7
MODE_NEUTRAL = 0
MODE_DUTY_CYCLE = 1
MODE_VOLTAGE = 2
MODE_VELOCITY = 3
MODE_POSITION = 4
MODE_MOTION_MAGIC = 5
DS_MAX_JOYSTICKS = 6
DS_MAX_AXES = 12
DS_MAX_POVS = 4
DS_NAME_LENGTH = 32
DS_ATTACHED = 1 << 0
DS_FMS_ATTACHED = 1 << 1
DS_ENABLED = 1 << 2
DS_AUTONOMOUS = 1 << 3
DS_TELEOP = 1 << 4
DS_TEST = 1 << 5
DS_ESTOP = 1 << 6


class TalosError(RuntimeError):
    """A talOS call failed; the message is the C++ one."""


class Param(ctypes.Structure):
    _fields_ = [("kind", c_int32), ("boolean", c_int32), ("integer", c_int64),
                ("number", c_double), ("string", c_char_p),
                ("count", c_uint64), ("numbers", ctypes.POINTER(c_double)),
                ("strings", ctypes.POINTER(c_char_p))]


class Sample(ctypes.Structure):
    _fields_ = [("valid", c_int32), ("value", c_int32), ("position", c_double),
                ("velocity", c_double), ("voltage", c_double),
                ("current", c_double), ("temperature_c", c_double),
                ("supply_current_a", c_double), ("forward_limit", c_int32),
                ("reverse_limit", c_int32)]


class MotorRequest(ctypes.Structure):
    _fields_ = [("mode", c_int32), ("slot", c_int32), ("demand", c_double),
                ("ff_volts", c_double)]


class Joystick(ctypes.Structure):
    _fields_ = [("connected", c_int32), ("axis_count", c_int32),
                ("button_count", c_int32), ("pov_count", c_int32),
                ("buttons", c_uint32), ("axes", c_float * DS_MAX_AXES),
                ("povs", c_int32 * DS_MAX_POVS),
                ("name", ctypes.c_char * DS_NAME_LENGTH)]


class DriverStation(ctypes.Structure):
    _fields_ = [("connected", c_int32), ("enabled", c_int32),
                ("autonomous", c_int32), ("teleop", c_int32),
                ("test", c_int32), ("estop", c_int32),
                ("fms_attached", c_int32), ("alliance", c_int32),
                ("station", c_int32), ("match_time_s", c_double),
                ("joysticks", Joystick * DS_MAX_JOYSTICKS)]


class DriverStationPacket(ctypes.Structure):
    _fields_ = [("flags", c_uint32), ("alliance", c_int32),
                ("station", c_int32), ("match_time_s", c_float),
                ("joysticks", Joystick * DS_MAX_JOYSTICKS)]


class Schema(ctypes.Structure):
    """TalosSchema: a message class's TALOS_SCHEMA_NAME/HASH/BFBS."""
    _fields_ = [("name", c_char_p), ("hash", c_uint64), ("bfbs", c_char_p),
                ("bfbs_size", c_uint32)]


def schema_of(mtype):
    """The TalosSchema of a talos._messages.MessageType."""
    bfbs = mtype.cls.TALOS_SCHEMA_BFBS
    return Schema(mtype.schema_name.encode(), mtype.schema_hash, bfbs,
                  len(bfbs))

_P = ctypes.POINTER
_SIGNATURES = {
    "talos_abi_version": (c_uint32, []),
    "talos_last_error": (c_char_p, []),
    "talos_run": (c_int32, [c_int32, _P(c_char_p), c_char_p, c_char_p,
                            FACTORY, c_void_p]),
    "talos_node_name": (c_char_p, [c_void_p]),
    "talos_node_simulation": (c_int32, [c_void_p]),
    "talos_node_describing": (c_int32, [c_void_p]),
    "talos_node_now_ns": (c_int64, [c_void_p]),
    "talos_node_tick": (c_uint64, [c_void_p]),
    "talos_node_next_update": (c_uint64, [c_void_p]),
    "talos_node_period_ns": (c_int64, [c_void_p]),
    "talos_node_config_path": (c_char_p, [c_void_p]),
    "talos_node_session_dir": (c_char_p, [c_void_p]),
    "talos_node_param": (c_int32, [c_void_p, c_char_p, c_int32, c_int32,
                                   _P(Param)]),
    "talos_node_param_of": (c_int32, [c_void_p, c_char_p, c_char_p, c_int32,
                                      c_int32, _P(Param)]),
    "talos_node_subscribe": (c_int64, [c_void_p, c_char_p, c_uint32,
                                       _P(Schema), c_int32, MESSAGE_CALLBACK,
                                       c_void_p]),
    "talos_node_publish": (c_int64, [c_void_p, c_char_p, c_uint32,
                                     _P(Schema)]),
    "talos_node_publish_bytes": (c_int64, [c_void_p, c_char_p, c_uint32,
                                           c_uint32, c_int32, c_int32]),
    "talos_node_every": (c_int32, [c_void_p, c_int64, CALLBACK, c_void_p]),
    "talos_node_set_update": (c_int32, [c_void_p, CALLBACK, c_void_p]),
    "talos_subscription_value": (c_void_p, [c_void_p, c_int64]),
    "talos_subscription_has_value": (c_int32, [c_void_p, c_int64]),
    "talos_subscription_age_ns": (c_int64, [c_void_p, c_int64]),
    "talos_subscription_fresh": (c_int32, [c_void_p, c_int64, c_int64]),
    "talos_subscription_updated": (c_int32, [c_void_p, c_int64]),
    "talos_publisher_send": (c_int32, [c_void_p, c_int64, c_char_p, c_uint32]),
    "talos_byte_publisher_send": (c_int32, [c_void_p, c_int64, c_char_p,
                                            c_uint32]),
    "talos_state_machine_create": (c_int64, [c_void_p, c_int64]),
    "talos_state_machine_state": (c_int64, [c_void_p, c_int64]),
    "talos_state_machine_previous": (c_int64, [c_void_p, c_int64]),
    "talos_state_machine_go": (c_int32, [c_void_p, c_int64, c_int64]),
    "talos_state_machine_time_in_state_ns": (c_int64, [c_void_p, c_int64]),
    "talos_state_machine_just_entered": (c_int32, [c_void_p, c_int64]),
    "talos_device": (c_int64, [c_void_p, c_int32, c_char_p]),
    "talos_robot_enabled": (c_int32, [c_void_p]),
    "talos_hardware_sample_time": (c_int32, [c_void_p, _P(c_int64)]),
    "talos_hardware_fresh": (c_int32, [c_void_p, c_int64]),
    "talos_hardware_age": (c_int32, [c_void_p, _P(c_int64)]),
    "talos_motor_set": (c_int32, [c_void_p, c_int64, c_int32, c_double,
                                  c_double, c_int32]),
    "talos_motor_clamped_count": (c_int32, [c_void_p, c_int64, _P(c_uint64)]),
    "talos_motor_rezero": (c_int32, [c_void_p, c_int64, c_double]),
    "talos_gyro_rezero": (c_int32, [c_void_p, c_int64, c_double]),
    "talos_motor_zeroed": (c_int32, [c_void_p, c_int64]),
    "talos_gyro_zeroed": (c_int32, [c_void_p, c_int64]),
    "talos_digital_output_set": (c_int32, [c_void_p, c_int64, c_int32]),
    "talos_pwm_set": (c_int32, [c_void_p, c_int64, c_double]),
    "talos_motor_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_sensor_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_motor_time": (c_int32, [c_void_p, c_int64, c_int32, _P(c_int64)]),
    "talos_sensor_time": (c_int32, [c_void_p, c_int64, c_int32, _P(c_int64)]),
    "talos_digital_input_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_analog_input_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_encoder_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_driver_station": (c_int32, [c_void_p, CALLBACK, c_void_p]),
    "talos_driver_station_read": (c_int32, [c_void_p, _P(DriverStation)]),
    "talos_target_open": (c_int64, [c_void_p, c_uint32, c_uint32,
                                    _P(Schema)]),
    "talos_target_get": (c_int32, [c_void_p, c_int64, c_void_p,
                                   _P(c_int32)]),
    "talos_target_source_name": (c_char_p, [c_void_p, c_int64, c_int32]),
    "talos_target_lane_open": (c_int64, [c_void_p, c_char_p, c_char_p,
                                         c_uint32, c_uint32, _P(Schema),
                                         c_int64]),
    "talos_target_lane_send": (c_int32, [c_void_p, c_int64, c_char_p,
                                         c_uint32]),
    "talos_target_lane_release": (c_int32, [c_void_p, c_int64]),
    "talos_operator_arming_open": (c_int64, [c_void_p]),
    "talos_operator_arming_evaluate": (c_int32, [c_void_p, c_int64, c_int32]),
    "talos_operator_arming_armed": (c_int32, [c_void_p, c_int64]),
    "talos_operator_arming_disarm": (c_int32, [c_void_p, c_int64]),
}
_TESTING_SIGNATURES = {
    "talos_sim_create": (c_void_p, [c_char_p]),
    "talos_sim_from_toml": (c_void_p, [c_char_p]),
    "talos_sim_destroy": (None, [c_void_p]),
    "talos_sim_set_session_dir": (c_int32, [c_void_p, c_char_p]),
    "talos_sim_add": (c_void_p, [c_void_p, c_char_p, FACTORY, c_void_p]),
    "talos_sim_node": (c_void_p, [c_void_p, c_char_p]),
    "talos_sim_step": (c_int32, [c_void_p, c_int64]),
    "talos_sim_pause": (c_int32, [c_void_p, c_char_p, c_int64]),
    "talos_sim_resume": (c_int32, [c_void_p, c_char_p]),
    "talos_sim_now_ns": (c_int64, [c_void_p]),
    "talos_sim_publish_raw": (c_int32, [c_void_p, c_char_p, c_char_p,
                                        c_uint32]),
    "talos_sim_latest_raw": (c_int32, [c_void_p, c_char_p, c_void_p,
                                       c_uint32]),
    "talos_sim_latest_bytes": (c_int32, [c_void_p, c_char_p, c_void_p,
                                         c_uint32, _P(c_uint32)]),
    "talos_sim_driver_station_generation": (c_uint64, [c_void_p]),
    "talos_sim_publish_target": (c_int32, [c_void_p, c_char_p, c_char_p,
                                           c_char_p, c_uint32, c_uint32,
                                           _P(c_uint64)]),
    "talos_sim_latest_target": (c_int32, [c_void_p, c_char_p, c_char_p,
                                          c_void_p, c_uint32, c_uint32,
                                          _P(c_int32)]),
    "talos_sim_hardware_create": (c_void_p, [c_void_p]),
    "talos_sim_hardware_destroy": (None, [c_void_p]),
    "talos_sim_hardware_enable": (c_int32, [c_void_p, c_int32]),
    "talos_sim_hardware_ideal_motors": (c_int32, [c_void_p, c_int32]),
    "talos_sim_hardware_set_motor": (c_int32, [c_void_p, c_char_p, c_char_p,
                                               c_double, c_double,
                                               c_uint32, c_uint32,
                                               c_double, c_double]),
    "talos_sim_hardware_set_limit_switches": (c_int32, [c_void_p, c_char_p,
                                                        c_char_p, c_int32,
                                                        c_int32]),
    "talos_sim_hardware_set_digital_input": (c_int32, [c_void_p, c_char_p,
                                                       c_char_p, c_int32]),
    "talos_sim_hardware_set_analog_input": (c_int32, [c_void_p, c_char_p,
                                                      c_char_p, c_double]),
    "talos_sim_hardware_set_encoder": (c_int32, [c_void_p, c_char_p, c_char_p,
                                                 c_double, c_double]),
    "talos_sim_hardware_set_cancoder": (c_int32, [c_void_p, c_char_p,
                                                  c_char_p, c_double,
                                                  c_double, c_uint32, c_uint32]),
    "talos_sim_hardware_set_gyro": (c_int32, [c_void_p, c_char_p, c_char_p,
                                              c_double, c_double,
                                              c_uint32, c_uint32]),
    "talos_sim_hardware_set_gateway_clock": (c_int32, [c_void_p, c_int64,
                                                       c_int64]),
    "talos_sim_hardware_set_connected": (c_int32, [c_void_p, c_char_p,
                                                   c_char_p, c_int32]),
    "talos_sim_hardware_reset_device": (c_int32, [c_void_p, c_char_p,
                                                  c_char_p]),
    "talos_sim_hardware_lose_states": (c_int32, [c_void_p, c_int32]),
    "talos_sim_hardware_reboot": (c_int32, [c_void_p]),
    "talos_sim_hardware_set_driver_station": (
        c_int32, [c_void_p, _P(DriverStationPacket)]),
    "talos_sim_hardware_motor_request": (c_int32, [c_void_p, c_char_p,
                                                   c_char_p,
                                                   _P(MotorRequest)]),
    "talos_sim_hardware_digital_output": (c_int32, [c_void_p, c_char_p,
                                                    c_char_p]),
    "talos_sim_hardware_pwm": (c_int32, [c_void_p, c_char_p, c_char_p,
                                         _P(c_double)]),
    "talos_sim_hardware_applied_motor": (c_int32, [c_void_p, c_char_p,
                                                   c_char_p,
                                                   _P(MotorRequest)]),
    "talos_sim_hardware_applied_digital_output": (
        c_int32, [c_void_p, c_char_p, c_char_p]),
    "talos_sim_hardware_applied_pwm": (c_int32, [c_void_p, c_char_p, c_char_p,
                                                 _P(c_double)]),
}


def _candidates():
    """TALOS_NODE_LIB, else the libraries beside this package in runfiles
    (<repo>/talOS/python/talos/ and <repo>/talOS/node_api/), the testing one
    first: a test depends on it, a deployed node never ships it."""
    override = os.environ.get("TALOS_NODE_LIB")
    if override:
        return [override]
    node_api = os.path.join(os.path.dirname(__file__), "..", "..", "node_api")
    return [os.path.join(node_api, library) for library in _LIBRARIES]


def _load():
    for path in _candidates():
        if not os.path.isfile(path):
            continue
        library = ctypes.CDLL(path)
        version = library.talos_abi_version()
        if version != _EXPECTED_ABI_VERSION:
            raise TalosError(f"{path} has ABI {version}, talos wants "
                             f"{_EXPECTED_ABI_VERSION}")
        signatures = dict(_SIGNATURES)
        if hasattr(library, "talos_sim_create"):
            signatures.update(_TESTING_SIGNATURES)
        for name, (restype, argtypes) in signatures.items():
            function = getattr(library, name)
            function.restype, function.argtypes = restype, argtypes
        return library
    raise TalosError(f"none of {_candidates()} exists (set TALOS_NODE_LIB, "
                     "or depend on //talOS/python:talos)")


lib = _load()

# The first exception a Python callback raised during the current C call, per
# thread like talos_last_error(): a callback runs on the thread whose C call
# ran it, and a failure there is raised by that call, never by another
# thread's.
_local = threading.local()


def callback(fn):
    """fn as a C callback body: an exception is kept and reported as 1."""
    def body(*args):
        try:
            fn(*args)
            return 0
        except BaseException as error:  # noqa: B902 - re-raised by fail()
            if pending() is None:
                _local.pending = error
            return 1
    return body


def pending():
    """The exception a callback raised during this thread's C call, or None."""
    return getattr(_local, "pending", None)


def fail():
    """Raise what made this thread's last C call fail."""
    error, _local.pending = pending(), None
    if error is not None:
        raise error
    raise TalosError(lib.talos_last_error().decode())


def check(result, failed):
    if failed(result):
        fail()
    return result


def ok(status):
    """For int32 status returns: TALOS_ERR (-1) raises."""
    return check(status, lambda r: r < 0)


def index(result):
    return check(result, lambda r: r < 0)


def value(result, failure):
    """For returns whose failure value is also a valid result (0 from
    talos_state_machine_state): the call failed only when it also left a
    talos_last_error(), which a call that succeeds clears."""
    return check(result, lambda r: r == failure and lib.talos_last_error())


def handle(pointer):
    return check(pointer, lambda p: not p)


def encode(text):
    return text.encode() if isinstance(text, str) else text
