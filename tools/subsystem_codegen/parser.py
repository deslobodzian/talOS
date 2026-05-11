import tomllib
from pathlib import Path
from model import Subsystem, Motor, MotorControllerType


def parse_subsystem(toml_path: Path):
    with open(toml_path, "rb") as f:
        subsystem_data = tomllib.load(f)

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
