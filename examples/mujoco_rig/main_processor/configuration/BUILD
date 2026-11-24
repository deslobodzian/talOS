load("//talOS/build:contract.bzl", "talos_graph_test")
load("//talOS/build:robot_model.bzl", "robot_model_sync_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")
load(":robot_sim_manifest.bzl", SIM_CONFIG_FILES = "CONFIG_FILES", SIM_NODES = "NODES")

package(default_visibility = ["//visibility:public"])

# robot.toml is the robot; robot_manifest.bzl (NODES, CONFIG_FILES) is
# generated from it. Regenerate it after editing [[subsystems]]:
#   bazel run //tools:sync_manifest -- examples/mujoco_rig/main_processor/configuration/robot.toml
# robot_sim.toml, the simulation-only config, includes it and has its own
# robot_sim_manifest.bzl.
exports_files([
    "mujoco/robot.xml",
    "robot.toml",
    "robot_sim.toml",
] + glob(["subsystems/*.toml"]))

# Fails, with the regenerate command, when mujoco/robot.xml (the MuJoCo model
# robot_sim.toml names) is not what mujoco/robot_params.toml generates.
robot_model_sync_test(
    name = "robot_model_sync_test",
    model = "mujoco/robot.xml",
    parameters = "mujoco/robot_params.toml",
)

# Fails, with the regenerate command, when robot_manifest.bzl is stale.
manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)

# Runs every node with --describe and lints the graph, as
# `launcher --describe-only` does.
talos_graph_test(
    name = "graph_test",
    config = "robot.toml",
    data = CONFIG_FILES,
    nodes = NODES,
    # A Python node cannot load the sanitizer runtimes through ctypes.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

manifest_sync_test(
    name = "sim_manifest_sync_test",
    config = "robot_sim.toml",
    config_files = SIM_CONFIG_FILES,
    manifest = "robot_sim_manifest.bzl",
)

# The same lint on the simulation config, with its bindings.
talos_graph_test(
    name = "sim_graph_test",
    config = "robot_sim.toml",
    data = SIM_CONFIG_FILES,
    nodes = SIM_NODES,
    # A Python node cannot load the sanitizer runtimes through ctypes.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)
