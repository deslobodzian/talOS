load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_py_node")

talos_py_node(
    name = "intake",
    node_name = "intake",
    main = "intake.py",
    class_name = "Intake",
    deps = ["//examples/superstructure/messages:messages_py"],
)

talos_py_node(
    name = "elevator",
    node_name = "elevator",
    main = "elevator.py",
    class_name = "Elevator",
    deps = ["//examples/superstructure/messages:messages_py"],
)

talos_py_node(
    name = "superstructure",
    node_name = "superstructure",
    main = "superstructure.py",
    class_name = "Superstructure",
    deps = ["//examples/superstructure/messages:messages_py"],
)

py_test(
    name = "superstructure_test",
    size = "small",
    srcs = ["superstructure_test.py"],
    legacy_create_init = False,
    data = [
        "elevator.py",
        "intake.py",
        "superstructure.py",
        "//examples/superstructure/config:robot_py.toml",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath //examples/superstructure/config:robot_py.toml)"},
    deps = [
        "//examples/superstructure/messages:messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
