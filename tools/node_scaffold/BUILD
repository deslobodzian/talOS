load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "node_scaffold_lib",
    srcs = [
        "scaffold.py",
        "templates.py",
    ],
    imports = ["."],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "node_scaffold",
    srcs = [
        "main.py",
    ],
    main = "main.py",
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = [":node_scaffold_lib"],
)

py_test(
    name = "packet_template_test",
    size = "small",
    srcs = ["packet_template_test.py"],
    imports = ["."],
    deps = [
        ":node_scaffold_lib",
        "//2026-robot/main_processor/intake:messages_py",
    ],
)
