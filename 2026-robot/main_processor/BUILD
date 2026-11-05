load("@rules_python//python:defs.bzl", "py_test")
load("//2026-robot/main_processor/bundle_testdata:robot_manifest.bzl", RELOCATION_CONFIG_FILES = "CONFIG_FILES", RELOCATION_NODES = "NODES")
load("//2026-robot/main_processor/configuration:robot_field_manifest.bzl", "CONFIG_FILES", "NODES")
load("//talOS/build:contract.bzl", "talos_bundle")

package(default_visibility = ["//visibility:public"])

# NODES and CONFIG_FILES are generated from robot_field.toml:
#   bazel run //tools:sync_manifest -- 2026-robot/main_processor/configuration/robot_field.toml
talos_bundle(
    name = "robot_bundle",
    config = "//2026-robot/main_processor/configuration:robot_field.toml",
    config_files = CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + NODES,
)

# bundle_test only: a config reaching its files through `include`,
# parent-directory `defaults`, a custom `path` and an entry `node`.
talos_bundle(
    name = "relocation_bundle",
    testonly = True,
    config = "//2026-robot/main_processor/bundle_testdata:robot.toml",
    config_files = RELOCATION_CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + RELOCATION_NODES,
)

py_test(
    name = "bundle_test",
    size = "small",
    srcs = ["bundle_test.py"],
    data = [
        ":relocation_bundle",
        ":robot_bundle",
        "//tools:deploy.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
