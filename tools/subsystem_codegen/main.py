from model import Subsystem
from parser import parse_subsystem
from generator import create_constants_file


def main():
    subsystem = parse_subsystem("tools/subsystem_codegen/test.toml")
    print(subsystem)
    create_constants_file(subsystem, "")
    print("python test")


if __name__ == "__main__":
    main()
