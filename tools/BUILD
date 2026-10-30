load("@rules_python//python:defs.bzl", "py_binary")

# bazel run //tools:new_subsystem -- <name> --lang cpp|py [--robot R] [--dry-run]
# Writes a node package under <robot>/main_processor/<name>/ and wires it into
# that robot's config and bundle. Stdlib only, so it runs on a fresh checkout.
py_binary(
    name = "new_subsystem",
    srcs = ["new_subsystem.py"],
)
