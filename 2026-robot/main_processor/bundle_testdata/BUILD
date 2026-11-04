load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES")

package(default_visibility = ["//2026-robot/main_processor:__pkg__"])

exports_files([
    "robot.toml",
    "mechanisms/shooter.toml",
])

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)
