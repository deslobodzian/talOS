load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "sample_py_node",
    srcs = ["sample_node.py"],
    deps = [
        "@talos//talOS/ipc/python:rtms_py",
    ],
)

py_test(
    name = "sample_py_node_test",
    size = "small",
    main = "sample_node_test.py",
    srcs = ["sample_node_test.py"],
    deps = [
        ":sample_py_node",
        "@rules_python//python/runfiles",
    ],
)
