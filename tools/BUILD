load("@rules_python//python:defs.bzl", "py_binary", "py_test")

# bazel run //tools:new_subsystem -- <name> --lang cpp|py [--robot R] [--standalone DIR] [--dry-run]
# Writes a node package under <robot>/main_processor/<name>/ and wires it into
# that robot's config and bundle, or scaffolds a standalone Bzlmod repo.
py_binary(
    name = "new_subsystem",
    srcs = ["new_subsystem.py"],
)

py_test(
    name = "new_subsystem_test",
    size = "small",
    srcs = [
        "new_subsystem.py",
        "new_subsystem_test.py",
    ],
    legacy_create_init = False,
    main = "new_subsystem_test.py",
)
