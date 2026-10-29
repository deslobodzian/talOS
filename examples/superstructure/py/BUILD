load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_py_node")

talos_py_node(
    name = "intake",
    class_name = "Intake",
    main = "intake.py",
    node_name = "intake",
    deps = ["//examples/superstructure/messages:messages_py"],
)

talos_py_node(
    name = "elevator",
    class_name = "Elevator",
    main = "elevator.py",
    node_name = "elevator",
    deps = ["//examples/superstructure/messages:messages_py"],
)

talos_py_node(
    name = "superstructure",
    class_name = "Superstructure",
    main = "superstructure.py",
    node_name = "superstructure",
    deps = ["//examples/superstructure/messages:messages_py"],
)

py_test(
    name = "superstructure_test",
    size = "small",
    srcs = ["superstructure_test.py"],
    data = [
        "elevator.py",
        "intake.py",
        "superstructure.py",
        "//examples/superstructure/config:robot_py.toml",
        "//examples/superstructure/config:subsystems",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath //examples/superstructure/config:robot_py.toml)"},
    legacy_create_init = False,
    deps = [
        "//examples/superstructure/messages:messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
