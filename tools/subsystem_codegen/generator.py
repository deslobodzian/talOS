from model import Subsystem
from pathlib import Path


def append_inline(type, name, value):
    if isinstance(value, str):
        value = f'"{value}"'
    return f"inline constexpr {type} {name} = {value};\n"


def add_includes(includes: list[str]):
    include_str = ""
    print(includes)
    for include in includes:
        include_str += f"#include {include}\n"
    return include_str


def create_constants_file(subsystem: Subsystem, path: Path):
    file_name = f"subsystem_{subsystem.name}_constants.h"
    constants_file = Path(path) / file_name
    print(constants_file)

    with open(constants_file, "w") as file:
        output = ""
        includes = ["string_view", "iostream"]
        output += add_includes(includes)

        motors = ""
        for motor in subsystem.motors:
            motors += append_inline(
                "std::string_view",
                "kCanBus",
                motor.canbus,
            )
            motors += append_inline(
                "int",
                "kCanID",
                motor.can_id
            )

        output += motors
        print(output)
        file.write(output)
