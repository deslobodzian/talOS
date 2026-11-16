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
    # Studio for `run.sh --start-studio-bridge` (docs/DEPLOYMENT.md).
    studio_bridge = "//studio/bridge:studio_bridge",
    studio_webroot = "//studio:web_dist",
)
