from model import Subsystem
from parser import parse_subsystem


def main():
    subsystem = parse_subsystem("tools/subsystem_codegen/test.toml")
    print(subsystem)
    print("python test")


if __name__ == "__main__":
    main()
