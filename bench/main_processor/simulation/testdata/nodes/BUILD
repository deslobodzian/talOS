load("//talOS/build:contract.bzl", "talos_cc_node", "talos_py_node")

# Test-only autonomous target producer for the bench soak graph
# (../robot_soak.toml). Lives in its own package so bench/main_processor/simulation
# remains a harness package (allowed to depend on bench node binaries under
# tools/check_layers.py) while this fixture uses the real talos_cc_node macro.
talos_cc_node(
    name = "auto_producer",
    testonly = True,
    srcs = ["auto_producer.cc"],
    node_name = "auto_producer",
    visibility = ["//bench/main_processor/simulation:__pkg__"],
    deps = [
        "//bench/main_processor/motor:messages",
        "//bench/main_processor/swerve_module:messages",
        "//talOS/devices",
    ],
)

# Test-only 20 ms Python actuator owner for the bench soak graph
# (../robot_soak.toml), preserving Python actuator SIGKILL/SIGSTOP/restart,
# request-gap, and mode-handoff coverage alongside both 5 ms C++ bench owners.
talos_py_node(
    name = "python_motor",
    testonly = True,
    class_name = "SoakPythonMotor",
    main = "python_motor.py",
    node_name = "python_motor",
    visibility = ["//bench/main_processor/simulation:__pkg__"],
    deps = [
        "//bench/main_processor/motor:messages_py",
    ],
)
