load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load(
    ":contract.bzl",
    "TALOS_TOPIC_ABBREVIATIONS",
    "TALOS_TOPIC_DIRECTION_SUFFIXES",
    "TALOS_TOPIC_ROLES",
    "talos_sim",
    "talos_sim_test",
    "talos_topic_problem",
)

exports_files([
    "contract.bzl",
    # The runner talos_graph_test and talos_sim run.
    "launcher_runner.py",
    # The check talos_sim_test runs.
    "sim_check.py",
])

# Free wildcard UDP port pair selection for test and tool launches.
py_library(
    name = "udp_ports",
    srcs = ["udp_ports.py"],
    visibility = ["//visibility:public"],
)

# unit (REQ-DEV-25, REQ-DEV-30): UDP port helpers pick two distinct free
# 0.0.0.0 ports below the OS ephemeral range and reject ports held on
# non-loopback addresses.
py_test(
    name = "udp_ports_test",
    size = "small",
    srcs = [
        "sim_check.py",
        "udp_ports_test.py",
    ],
    legacy_create_init = False,
    deps = [
        ":udp_ports",
        "@rules_python//python/runfiles",
    ],
)

# Unlinks the test namespace's live registry after talos_sim_test's session
# (sim_check.py).
cc_binary(
    name = "registry_unlink",
    srcs = ["registry_unlink.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/introspection:registry",
        "//talOS/memory:shared_memory",
    ],
)

# integration (REQ-DEV-30): a launch still running at its bound leaves no
# process of its session, in its process group or another.
py_test(
    name = "sim_check_hang_test",
    size = "small",
    srcs = [
        "sim_check.py",
        "sim_check_hang_test.py",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# The session sim_check_no_state_test judges: telemetry beside the bridge
# and a simulator that sends its State to the wrong port.
talos_sim(
    name = "no_state_sim",
    testonly = True,
    config = "//talOS/build/testdata/sim_check:robot.toml",
    data = ["//talOS/build/testdata/sim_check:sim_gateway"],
    nodes = ["//talOS/telemetry:node"],
)

# e2e (REQ-DEV-30): sim_check fails a session whose bridge never received a
# State, naming the controller link, where every member ran once and
# exited 0.
py_test(
    name = "sim_check_no_state_test",
    size = "medium",
    timeout = "short",
    srcs = [
        "sim_check.py",
        "sim_check_no_state_test.py",
    ],
    data = [
        ":no_state_sim",
        ":registry_unlink",
        "//talOS/launcher:log_dump",
        "//talOS/launcher:session_report",
    ],
    env = {
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
        "REGISTRY_UNLINK": "$(rlocationpath :registry_unlink)",
        "SIM": "$(rlocationpath :no_state_sim)",
    },
    legacy_create_init = False,
    # Spawns an instrumented launcher and members (each spawn is two fork()s
    # plus exec); under macOS TSan one fork costs ~120 ms and an empty main
    # ~194 ms of kernel CPU (vs 0.5/2 ms plain), so on a 3-vCPU hosted runner
    # session start-up overruns the 14 s ready bound (TODO.md CI-13). The
    # Linux sanitizer legs still run this test.
    tags = [
        "e2e",
        "no_macos_sanitizer",
    ],
    deps = ["@rules_python//python/runfiles"],
)

# The session sim_check_studio_pass_test judges: telemetry, the hardware
# bridge, sim_gateway, and the testdata studio_bridge stub.
talos_sim(
    name = "studio_sim",
    testonly = True,
    config = "//talOS/build/testdata/sim_check:studio_robot.toml",
    data = [
        "//talOS/build/testdata/sim_check:studio_bridge",
        "//talOS/simulation:sim_gateway",
    ],
    nodes = ["//talOS/telemetry:node"],
)

# e2e (REQ-DEV-30, REQ-OPS-30, REQ-OPS-34): talos_sim_test(studio = True)
# passes when studio_bridge is present, binds an ephemeral HTTP port, settles,
# and exits cleanly on SIGINT.
talos_sim_test(
    name = "sim_check_studio_pass_test",
    sim = ":studio_sim",
    studio = True,
    # Same macOS TSan fork/exec overhead as sim_check_no_state_test (CI-13).
    tags = ["no_macos_sanitizer"],
)

# Same config without studio_bridge in runfiles: the launcher drops the
# unanswered bridge from the roster and manifest (DropUnansweredStudioBridge),
# which sim_check_studio_test proves sim_check(studio=True) catches.
talos_sim(
    name = "missing_studio_sim",
    testonly = True,
    config = "//talOS/build/testdata/sim_check:studio_robot.toml",
    data = ["//talOS/simulation:sim_gateway"],
    nodes = ["//talOS/telemetry:node"],
)

# e2e (REQ-DEV-30, REQ-OPS-30, REQ-OPS-34): sim_check(studio=True) fails
# cleanly without hanging when studio_bridge is missing from the session
# manifest, and judge(studio=True) rejects missing/bad http_port, restarts,
# recorder_failed, and nonzero exit/signal.
py_test(
    name = "sim_check_studio_test",
    size = "medium",
    timeout = "short",
    srcs = [
        "sim_check.py",
        "sim_check_studio_test.py",
    ],
    data = [
        ":missing_studio_sim",
        ":registry_unlink",
        "//talOS/launcher:log_dump",
        "//talOS/launcher:session_report",
    ],
    env = {
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
        "MISSING_STUDIO_SIM": "$(rlocationpath :missing_studio_sim)",
        "REGISTRY_UNLINK": "$(rlocationpath :registry_unlink)",
    },
    legacy_create_init = False,
    # Same macOS TSan fork/exec overhead as sim_check_no_state_test (CI-13).
    tags = [
        "e2e",
        "no_macos_sanitizer",
    ],
    deps = ["@rules_python//python/runfiles"],
)

# talos_bundle's assembly step, run in the exec configuration's hermetic
# Python.
py_binary(
    name = "bundle_builder",
    srcs = ["bundle_builder.py"],
)

py_test(
    name = "bundle_builder_test",
    size = "small",
    srcs = ["bundle_builder_test.py"],
    data = [
        "bundle_builder.py",
        "//tools:deploy.sh",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# talos_messages' schema step: the field-name lint and the C++/Python
# descriptors, from each schema's .bfbs.
cc_binary(
    name = "descriptor_gen",
    srcs = ["descriptor_gen.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/schema:schema_hash",
        "//talOS/schema:unit_suffix",
        "@flatbuffers//:runtime_cc",
    ],
)

# Lint fixtures, compiled as talos_messages compiles schemas: a real copy of
# an include root, with declaration files recorded relative to it.
genrule(
    name = "lint_fixtures",
    srcs = [
        "testdata/lint_test/bad.fbs",
        "testdata/lint_test/good.fbs",
        "testdata/lint_test/included.fbs",
    ],
    outs = [
        "lint_fixtures/bad.bfbs",
        "lint_fixtures/good.bfbs",
        "lint_fixtures/included.bfbs",
    ],
    cmd = "tmp=$$(mktemp -d) && trap 'rm -rf $$tmp' EXIT && " +
          "mkdir $$tmp/lint_test && cp $(SRCS) $$tmp/lint_test && " +
          "$(execpath @flatbuffers//:flatc) --binary --schema " +
          "--bfbs-filenames $$tmp -I $$tmp -o $(RULEDIR)/lint_fixtures " +
          "$$tmp/lint_test/bad.fbs $$tmp/lint_test/good.fbs $$tmp/lint_test/included.fbs",
    tools = ["@flatbuffers//:flatc"],
    visibility = ["//visibility:private"],
)

py_test(
    name = "descriptor_gen_lint_test",
    size = "small",
    srcs = ["descriptor_gen_lint_test.py"],
    data = [
        "lint_fixtures/bad.bfbs",
        "lint_fixtures/good.bfbs",
        "lint_fixtures/included.bfbs",
        ":descriptor_gen",
    ],
    env = {
        "TALOS_DESCRIPTOR_GEN": "$(rlocationpath :descriptor_gen)",
        "TALOS_LINT_BAD": "$(rlocationpath lint_fixtures/bad.bfbs)",
        "TALOS_LINT_GOOD": "$(rlocationpath lint_fixtures/good.bfbs)",
        "TALOS_LINT_INCLUDED": "$(rlocationpath lint_fixtures/included.bfbs)",
    },
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# talos_messages checks topic names in Starlark at load time; the grammar's
# authority is naming::CheckTopicName. Each line of the verdicts file is
# `<kind> <entry> <ok|bad> <topic>`: a `name` probe for each name below, and
# one probe per role, abbreviation (`<wrong>><right>`) and direction suffix
# in contract.bzl's tables, so the test also sees an entry either side adds
# or drops.
_GRAMMAR_NAMES = [
    "/hw/state",
    "/hw/state/driver_station",
    "/drivetrain/target/teleop",
    "/talos/telemetry",
    "/a/state/b/c",
    "/odometry2/state",
    "/drivetrain/state/" + "x" * 13,
    "/" + "a" * 24 + "/state",
    "",
    "drivetrain/state",
    "/drivetrain/state/",
    "/drivetrain//state",
    "/odometry",
    "/a/state/b/c/d",
    "/drivetrain/state/" + "x" * 14,
    "/Drivetrain/state",
    "/drive-train/state",
    "/drive.train/state",
    "/2026/state",
    "/drivetrain__x/state",
    "/drivetrain_/state",
    "/drivetrain/pose",
]

_GRAMMAR_PROBES = [("name", "-", topic) for topic in _GRAMMAR_NAMES] + [
    (
        "role",
        role,
        "/hw/" + role,
    )
    for role in TALOS_TOPIC_ROLES
] + [
    (
        "abbreviation",
        wrong + ">" + right,
        "/hw/state/" + wrong,
    )
    for wrong, right in TALOS_TOPIC_ABBREVIATIONS.items()
] + [
    (
        "suffix",
        suffix,
        "/hw/state/x" + suffix,
    )
    for suffix in TALOS_TOPIC_DIRECTION_SUFFIXES
]

genrule(
    name = "topic_grammar_verdicts",
    outs = ["topic_grammar_verdicts.txt"],
    cmd = "cat << 'EOF' > $@\n" + "".join([
        "%s %s %s %s\n" % (
            kind,
            entry,
            "bad" if talos_topic_problem(topic) else "ok",
            topic,
        )
        for kind, entry, topic in _GRAMMAR_PROBES
    ]) + "EOF\n",
    visibility = ["//visibility:private"],
)

cc_test(
    name = "topic_grammar_test",
    size = "small",
    srcs = ["topic_grammar_test.cc"],
    data = [":topic_grammar_verdicts.txt"],
    env = {"TALOS_TOPIC_VERDICTS": "$(rootpath :topic_grammar_verdicts.txt)"},
    deps = [
        "//talOS/introspection:names",
        "@googletest//:gtest_main",
    ],
)
