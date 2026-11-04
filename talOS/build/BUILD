load("@rules_python//python:defs.bzl", "py_test")

exports_files([
    "contract.bzl",
    "bundle_builder.py",
])

py_test(
    name = "bundle_builder_test",
    size = "small",
    srcs = ["bundle_builder_test.py"],
    data = ["bundle_builder.py"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
