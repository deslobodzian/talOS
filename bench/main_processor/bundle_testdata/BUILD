load("//talOS/build:contract.bzl", "talos_py_node")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES")

package(default_visibility = ["//bench/main_processor:__pkg__"])

exports_files([
    "robot.toml",
    "mechanisms/motor.toml",
    "mechanisms/native_wheel.toml",
])

# bundle_test only: a Python node importing native wheels (numpy, and the
# gtsam and robotpy-wpimath that talOS pins for robot repositories and no
# other talOS target loads), so relocation_bundle carries wheels' extension
# modules and the shared libraries they bundle. testonly: no robot bundle
# can ship it, and check_node_matrix skips it as a fixture.
talos_py_node(
    name = "native_wheel",
    testonly = True,
    class_name = "NativeWheel",
    main = "native_wheel.py",
    node_name = "native_wheel",
    deps = [
        "@pypi//gtsam",
        "@pypi//numpy",
        "@pypi//robotpy_wpimath",
    ],
)

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)
