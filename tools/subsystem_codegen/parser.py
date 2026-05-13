import tomllib
from pathlib import Path
from model import Subsystem, Motor, MotorControllerType
from collections import defaultdict

VALID_SUBSYSTEM_KEYS = set([
    "subsystem",
    "motors",
    "sensors",
])


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
    return motors


def verify_toml_subsystem(subsystem_data: dict):
    print(subsystem_data.keys())
    key_subset = set(subsystem_data.keys())

    subset_diff = key_subset.difference(VALID_SUBSYSTEM_KEYS)
    if not key_subset.issubset(VALID_SUBSYSTEM_KEYS):
        print(f"Subsystem toml keys not in valid keys: {subset_diff}")
        return False

    motors = extract_motor_data(subsystem_data)
    duplicate_motors = find_duplicate_motors(motors)
    for (canbus, id), group in duplicate_motors.items():
        print(f"Duplicate CAN device: bus={canbus}, id={id}")
        for motor in group:
            print(f" - {motor.name}, ({motor.type})")

    return True


def parse_subsystem(toml_path: Path):
    with open(toml_path, "rb") as f:
        subsystem_data = tomllib.load(f)

    if not verify_toml_subsystem(subsystem_data):
        return None

    subsystem = subsystem_data["subsystem"]

    subsystem = Subsystem(
        name=subsystem["name"],
        subsystem_id=subsystem["id"]
    )
    print(subsystem)

    return subsystem
