load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# IntakeTarget / IntakeState (namespace intake) and the intake's topics. The
# operator interface and telemetry consume them. The intake reads its target
# through talos.Target: teleop and autonomous publish on their own lanes,
# /intake/target/<source>, and the [robot.targets] policy picks one.
talos_messages(
    name = "messages",
    srcs = ["intake_message.fbs"],
    topics = {
        "state": "/intake/state",
    },
)

talos_py_node(
    name = "node",
    class_name = "Intake",
    main = "intake.py",
    node_name = "intake",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "intake.py",
        "testdata/robot.toml",
        ":node",
    ],
    env = {"INTAKE_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
    deps = [
        ":messages_py",
        "//talOS/bridge:messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
