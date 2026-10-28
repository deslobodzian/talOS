load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# IntakeTarget / IntakeState (namespace intake) and the intake's topics. The
# arbiter, operator interface and telemetry consume them.
talos_messages(
    name = "messages",
    srcs = ["intake_message.fbs"],
    hdrs = ["topics.h"],
)

talos_py_node(
    name = "node",
    node_name = "intake",
    main = "intake.py",
    class_name = "Intake",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    legacy_create_init = False,
    env = {"INTAKE_BINARY": "$(rlocationpath :node)"},
    data = [
        "intake.py",
        "testdata/robot.toml",
        ":node",
    ],
    deps = [
        ":messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
