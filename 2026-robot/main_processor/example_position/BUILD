load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "position_example",
    srcs = ["position_example.py"],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = ["//2026-robot/main_processor/intake:intake_py"],
)

py_test(
    name = "position_example_test",
    size = "small",
    srcs = ["position_example_test.py"],
    legacy_create_init = False,
    deps = [
        ":position_example",
        "//2026-robot/main_processor/intake:intake_py",
    ],
)

exports_files(["subsystem.toml"])
