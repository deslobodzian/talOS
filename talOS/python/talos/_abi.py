"""ctypes binding to libtalos_node (talOS/node_api/node_api.h).

Framework-internal. Every function here is one C entry point; failures
raise, with the C++ message, or with the Python exception a callback raised.
"""

import ctypes
import os

ABI_VERSION = 9
_LIBRARIES = ("libtalos_node_testing.so", "libtalos_node.so")

c_int32, c_int64, c_uint32, c_uint64 = (
    ctypes.c_int32, ctypes.c_int64, ctypes.c_uint32, ctypes.c_uint64)
c_double, c_float, c_char_p, c_void_p = (
    ctypes.c_double, ctypes.c_float, ctypes.c_char_p, ctypes.c_void_p)

CALLBACK = ctypes.CFUNCTYPE(c_int32, c_void_p)
MESSAGE_CALLBACK = ctypes.CFUNCTYPE(c_int32, c_void_p, c_uint32, c_void_p)
FACTORY = ctypes.CFUNCTYPE(c_int32, c_void_p, c_void_p)

PARAM_ANY, PARAM_BOOL, PARAM_INT, PARAM_DOUBLE, PARAM_STRING = range(5)
(DEVICE_MOTOR, DEVICE_CANCODER, DEVICE_GYRO, DEVICE_DIGITAL_INPUT,
 DEVICE_DIGITAL_OUTPUT, DEVICE_ANALOG_INPUT, DEVICE_ENCODER,
 DEVICE_PWM) = range(8)
DS_MAX_JOYSTICKS, DS_MAX_AXES, DS_MAX_POVS, DS_NAME_LENGTH = 6, 12, 4, 32
(DS_ATTACHED, DS_FMS_ATTACHED, DS_ENABLED, DS_AUTONOMOUS, DS_TELEOP, DS_TEST,
 DS_ESTOP) = (1 << bit for bit in range(7))


class TalosError(RuntimeError):
    """A talOS call failed; the message is the C++ one."""


class Param(ctypes.Structure):
    _fields_ = [("kind", c_int32), ("boolean", c_int32), ("integer", c_int64),
                ("number", c_double), ("string", c_char_p)]


class Sample(ctypes.Structure):
    _fields_ = [("valid", c_int32), ("value", c_int32), ("position", c_double),
                ("velocity", c_double), ("voltage", c_double),
                ("current", c_double)]


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


_P = ctypes.POINTER
_SIGNATURES = {
    "talos_abi_version": (c_uint32, []),
    "talos_last_error": (c_char_p, []),
    "talos_run": (c_int32, [c_int32, _P(c_char_p), c_char_p, c_char_p,
                            FACTORY, c_void_p]),
    "talos_node_name": (c_char_p, [c_void_p]),
    "talos_node_simulation": (c_int32, [c_void_p]),
    "talos_node_now_ns": (c_int64, [c_void_p]),
    "talos_node_tick": (c_uint64, [c_void_p]),
    "talos_node_next_update": (c_uint64, [c_void_p]),
    "talos_node_period_ns": (c_int64, [c_void_p]),
    "talos_node_config_path": (c_char_p, [c_void_p]),
    "talos_node_param": (c_int32, [c_void_p, c_char_p, c_int32, c_int32,
                                   _P(Param)]),
    "talos_node_subscribe": (c_int64, [c_void_p, c_char_p, c_uint32, c_int32,
                                       MESSAGE_CALLBACK, c_void_p]),
    "talos_node_publish": (c_int64, [c_void_p, c_char_p, c_uint32]),
    "talos_node_every": (c_int32, [c_void_p, c_int64, CALLBACK, c_void_p]),
    "talos_node_set_update": (c_int32, [c_void_p, CALLBACK, c_void_p]),
    "talos_subscription_value": (c_void_p, [c_void_p, c_int64]),
    "talos_subscription_has_value": (c_int32, [c_void_p, c_int64]),
    "talos_subscription_age_ns": (c_int64, [c_void_p, c_int64]),
    "talos_subscription_fresh": (c_int32, [c_void_p, c_int64, c_int64]),
    "talos_subscription_updated": (c_int32, [c_void_p, c_int64]),
    "talos_publisher_send": (c_int32, [c_void_p, c_int64, c_char_p, c_uint32]),
    "talos_device": (c_int64, [c_void_p, c_int32, c_char_p]),
    "talos_robot_enabled": (c_int32, [c_void_p]),
    "talos_motor_set": (c_int32, [c_void_p, c_int64, c_int32, c_double,
                                  c_double, c_int32]),
    "talos_digital_output_set": (c_int32, [c_void_p, c_int64, c_int32]),
    "talos_pwm_set": (c_int32, [c_void_p, c_int64, c_double]),
    "talos_motor_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_sensor_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_digital_input_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_analog_input_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_encoder_sample": (c_int32, [c_void_p, c_int64, _P(Sample)]),
    "talos_driver_station": (c_int32, [c_void_p, CALLBACK, c_void_p]),
    "talos_driver_station_read": (c_int32, [c_void_p, _P(DriverStation)]),
}
_TESTING_SIGNATURES = {
    "talos_sim_create": (c_void_p, [c_char_p]),
    "talos_sim_from_toml": (c_void_p, [c_char_p]),
    "talos_sim_destroy": (None, [c_void_p]),
    "talos_sim_add": (c_void_p, [c_void_p, c_char_p, FACTORY, c_void_p]),
    "talos_sim_node": (c_void_p, [c_void_p, c_char_p]),
    "talos_sim_step": (c_int32, [c_void_p, c_int64]),
    "talos_sim_now_ns": (c_int64, [c_void_p]),
    "talos_sim_publish_raw": (c_int32, [c_void_p, c_char_p, c_char_p,
                                        c_uint32]),
    "talos_sim_latest_raw": (c_int32, [c_void_p, c_char_p, c_void_p,
                                       c_uint32]),
    "talos_sim_hardware_create": (c_void_p, [c_void_p]),
    "talos_sim_hardware_destroy": (None, [c_void_p]),
    "talos_sim_hardware_enable": (c_int32, [c_void_p, c_int32]),
    "talos_sim_hardware_ideal_motors": (c_int32, [c_void_p, c_int32]),
    "talos_sim_hardware_set_motor": (c_int32, [c_void_p, c_char_p, c_char_p,
                                               c_double, c_double]),
    "talos_sim_hardware_set_digital_input": (c_int32, [c_void_p, c_char_p,
                                                       c_char_p, c_int32]),
    "talos_sim_hardware_set_analog_input": (c_int32, [c_void_p, c_char_p,
                                                      c_char_p, c_double]),
    "talos_sim_hardware_set_encoder": (c_int32, [c_void_p, c_char_p, c_char_p,
                                                 c_double, c_double]),
    "talos_sim_hardware_set_cancoder": (c_int32, [c_void_p, c_char_p,
                                                  c_char_p, c_double,
                                                  c_double]),
    "talos_sim_hardware_set_gyro": (c_int32, [c_void_p, c_char_p, c_char_p,
                                              c_double, c_double]),
    "talos_sim_hardware_set_connected": (c_int32, [c_void_p, c_char_p,
                                                   c_char_p, c_int32]),
    "talos_sim_hardware_set_driver_station": (
        c_int32, [c_void_p, _P(DriverStationPacket)]),
    "talos_sim_hardware_motor_request": (c_int32, [c_void_p, c_char_p,
                                                   c_char_p,
                                                   _P(MotorRequest)]),
    "talos_sim_hardware_digital_output": (c_int32, [c_void_p, c_char_p,
                                                    c_char_p]),
    "talos_sim_hardware_pwm": (c_int32, [c_void_p, c_char_p, c_char_p,
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
        if version != ABI_VERSION:
            raise TalosError(f"{path} has ABI {version}, talos wants "
                             f"{ABI_VERSION}")
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

# The first exception a Python callback raised during the current C call.
_pending = None


def callback(fn):
    """fn as a C callback body: an exception is kept and reported as 1."""
    def body(*args):
        global _pending
        try:
            fn(*args)
            return 0
        except BaseException as error:  # noqa: B902 - re-raised by fail()
            if _pending is None:
                _pending = error
            return 1
    return body


def fail():
    """Raise what made the last C call fail."""
    global _pending
    error, _pending = _pending, None
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


def handle(pointer):
    return check(pointer, lambda p: not p)


def encode(text):
    return text.encode() if isinstance(text, str) else text
