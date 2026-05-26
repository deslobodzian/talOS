from model import Subsystem
from pathlib import Path


def append_inline(type, name, value):
    if isinstance(value, str):
        value = f'"{value}"'
    return f"inline constexpr {type} {name} = {value};"


def add_includes(current_output, includes: list[str]):
    for include in includes:
        current_output.append(f"#include <{include}>")
    return current_output


def add_namespace(output: list[str], subsystem_name: str):
    namespace_str = f"namespace constants::{subsystem_name} {{"
    for idx, line in enumerate(output):
        if "#" not in line:
            output.insert(idx, namespace_str)
            break
    output.append(f"}} /* constants::{subsystem_name} */")
    return output


# All this does is combine output # With a "\n"
def finalize_output(output: list[str]) -> str:
    return "\n".join(output)


def create_constants_file(subsystem: Subsystem, path: Path):
    file_name = f"subsystem_{subsystem.name}_constants.h"
    constants_file = Path(path) / file_name
    print(constants_file)

    output = []
    includes = ["string_view", "iostream"]
    output = add_includes(output, includes)

    for motor in subsystem.motors:
        output.append(f"\n// Motor: {motor.name}")
        output.append(append_inline(
            "std::string_view",
            "kCanBus",
            motor.canbus,
        ))
        output.append(append_inline(
            "int",
            "kCanID",
            motor.can_id
        ))
        output.append(append_inline(
            "boolean",
            "kInverted",
            motor.inverted
        ))
        output.append(append_inline(
            "int",
            "kSlot",
            motor.slot
        ))
        output.append(append_inline(
            "double",
            "kSupplyLimit",
            motor.config.supply_current_limit
        ))
        output.append(append_inline(
            "double",
            "kStatorLimit",
            motor.config.stator_current_limit
        ))

    add_namespace(output, subsystem.name)
    output = finalize_output(output)
    print(output)

    with open(constants_file, "w") as file:
        file.write(output)
