load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# PositionPyTarget / PositionPyState (namespace position_py) and the position_py's topics.
# Producers write their own lanes, /position_py/target/<source>, through
# talos::TargetLane / talos.TargetLane; the position_py reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["position_py_message.fbs"],
    include_prefix = "position_py",
    topics = {
        "state": "/position_py/state",
    },
)

talos_py_node(
    name = "node",
    class_name = "PositionPy",
    main = "position_py.py",
    node_name = "position_py",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "position_py.py",
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
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)

# Golden copy of //tools:new_subsystem output; only this target is not generated.
filegroup(
    name = "golden",
    srcs = glob(["**"]),
    visibility = ["//tools:__pkg__"],
)
