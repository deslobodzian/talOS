import tomllib
from pathlib import Path
from model import Subsystem, Motor, MotorControllerType


def verify_toml_subsystem(subsystem_data: dict):
    # Verify keys [subsystem, motors, sensors]
    valid_keys = set(["subsystem", "motors", "sensors"])
    print(subsystem_data.keys())
    key_subset = set(subsystem_data.keys())

    if not key_subset.issubset(valid_keys):
        print(f"Subsystem toml keys not in valid keys: {valid_keys - key_subset}")


def parse_subsystem(toml_path: Path):
    with open(toml_path, "rb") as f:
        subsystem_data = tomllib.load(f)

    verify_toml_subsystem(subsystem_data)

    subsystem = subsystem_data["subsystem"]

    motors = []
    for motor in subsystem_data["motors"]:
        motor_data = subsystem_data["motors"][motor]
        print(motor)
        motors.append(
            Motor(
                type=MotorControllerType(motor_data["type"]),
                name=motor,
                can_id=motor_data["id"],
                inverted=motor_data["inverted"],
            )
        )

    print(motors)
    subsystem = Subsystem(
        name=subsystem["name"],
        subsystem_id=subsystem["id"]
    )
    print(subsystem)

    return subsystem
