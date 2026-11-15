load("@rules_python//python:defs.bzl", "py_test")
load("@talos//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# GateTarget / GateState (namespace gate) and the gate's topics.
# Producers write their own lanes, /gate/target/<source>, through
# talos::TargetLane / talos.TargetLane; the gate reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["gate_message.fbs"],
    include_prefix = "gate",
    topics = {
        "state": "/gate/state",
    },
)

talos_py_node(
    name = "node",
    class_name = "Gate",
    main = "gate.py",
    node_name = "gate",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "gate.py",
        "testdata/robot.toml",
        ":node",
    ],
    env = {"NODE_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":messages_py",
        "@rules_python//python/runfiles",
        "@talos//talOS/python:testing",
    ],
)
