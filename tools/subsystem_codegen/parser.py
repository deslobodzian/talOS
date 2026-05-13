import tomllib
from pathlib import Path
from model import Subsystem, Motor, MotorControllerType
from collections import defaultdict

VALID_SUBSYSTEM_KEYS = set([
    "subsystem",
    "motors",
    "sensors",
])

VALID_MOTOR_KEYS = set([
    "type",
    "canbus",
    "id",
    "inverted",
])


def valid_keys(key_subset: set, validation_keys: set):
    subset_diff = key_subset.difference(validation_keys)
    if not key_subset.issubset(validation_keys):
        print(f"Toml keys not in valid keys: {subset_diff}")
        return False
    return True


def find_duplicate_motors(motors: list[Motor]):
    grouped = defaultdict(list)
    for motor in motors:
        grouped[(motor.canbus, motor.can_id)].append(motor)

    return {
        key: group
        for key, group in grouped.items()
        if len(group) > 1
    }


def extract_motor_data(subsystem_data) -> list[Motor]:
    motors = []
    for motor in subsystem_data["motors"]:
        motor_keys_set = set(subsystem_data["motors"][motor].keys())
        if not valid_keys(motor_keys_set, VALID_MOTOR_KEYS):
            return None

        motor_data = subsystem_data["motors"][motor]
        motors.append(
            Motor(
                type=MotorControllerType(motor_data["type"]),
                name=motor,
                can_id=motor_data["id"],
                canbus=motor_data["canbus"],
                inverted=motor_data["inverted"],
            )
        )
    duplicate_motors = find_duplicate_motors(motors)
    for (canbus, id), group in duplicate_motors.items():
        print(f"Duplicate CAN device: bus={canbus}, id={id}")
        for motor in group:
            print(f" - {motor.name}, ({motor.type})")
    return motors


def parse_subsystem(toml_path: Path):
    with open(toml_path, "rb") as f:
        subsystem_data = tomllib.load(f)

    if not valid_keys(set(subsystem_data.keys()), VALID_SUBSYSTEM_KEYS):
        return None

    motors = extract_motor_data(subsystem_data)
    if not motors:
        return None

    subsystem = subsystem_data["subsystem"]

    subsystem = Subsystem(
        name=subsystem["name"],
        subsystem_id=subsystem["id"],
        motors=motors,
        sensors=None
    )
    return subsystem
