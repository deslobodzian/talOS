load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# ScaffoldPyTarget / ScaffoldPyState (namespace scaffold_py) and the scaffold_py's topics.
# Producers write their own lanes, /scaffold_py/target/<source>, through
# talos::TargetLane / talos.TargetLane; the scaffold_py reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["scaffold_py_message.fbs"],
    include_prefix = "scaffold_py",
    topics = {
        "state": "/scaffold_py/state",
    },
)

talos_py_node(
    name = "node",
    class_name = "ScaffoldPy",
    main = "scaffold_py.py",
    node_name = "scaffold_py",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "scaffold_py.py",
        "testdata/robot.toml",
        ":node",
    ],
    env = {"NODE_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
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
