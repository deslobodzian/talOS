load("//bench/main_processor/configuration:robot_bench_manifest.bzl", "CONFIG_FILES", "NODES")
load("//talOS/build:contract.bzl", "talos_bundle")

package(default_visibility = ["//visibility:public"])

talos_bundle(
    name = "bench_bundle",
    config = "//bench/main_processor/configuration:robot_bench.toml",
    config_files = CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + NODES,
)
