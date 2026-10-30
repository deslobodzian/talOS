load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_bundle")

package(default_visibility = ["//visibility:public"])

talos_bundle(
    name = "robot_bundle",
    config = "//2026-robot/main_processor/configuration:robot.toml",
    nodes = [
        "//talOS/bridge:hardware_node",
        "//2026-robot/main_processor/arbiter:node",
        "//2026-robot/main_processor/drivetrain:node",
        "//2026-robot/main_processor/intake:node",
        "//2026-robot/main_processor/odometry:node",
        "//2026-robot/main_processor/operator_interface:node",
        "//2026-robot/main_processor/shooter:node",
        "//2026-robot/main_processor/telemetry:node",
    ],
    subsystems = [
        "//2026-robot/main_processor/configuration:subsystems/arbiter.toml",
        "//2026-robot/main_processor/configuration:subsystems/drivetrain.toml",
        "//2026-robot/main_processor/configuration:subsystems/intake.toml",
        "//2026-robot/main_processor/configuration:subsystems/odometry.toml",
        "//2026-robot/main_processor/configuration:subsystems/operator_interface.toml",
        "//2026-robot/main_processor/configuration:subsystems/shooter.toml",
        "//2026-robot/main_processor/configuration:subsystems/telemetry.toml",
    ],
)

genrule(
    name = "bundle_test_gen",
    outs = ["bundle_test.py"],
    cmd = """cat << 'EOF' > $@
import os
import shutil
import subprocess
import tempfile
import unittest
from python.runfiles import runfiles


class BundleTest(unittest.TestCase):
    def test_relocated_bundle_describe(self):
        r = runfiles.Create()
        bundle_src = r.Rlocation("_main/2026-robot/main_processor/robot_bundle")
        self.assertTrue(bundle_src and os.path.isdir(bundle_src), f"missing bundle: {bundle_src}")
        with tempfile.TemporaryDirectory() as tmp:
            copied_bundle = os.path.join(tmp, "robot_bundle")
            log_dir = os.path.join(tmp, "logs")
            shutil.copytree(os.path.realpath(bundle_src), copied_bundle, symlinks=True)
            env = dict(os.environ)
            env["TALOS_LOG_DIR"] = log_dir
            proc = subprocess.run(
                ["./run.sh", "--describe-only"],
                cwd=copied_bundle,
                env=env,
                capture_output=True,
                text=True,
                timeout=120,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout + "\\n" + proc.stderr)
            self.assertNotIn("error:", proc.stdout.lower())
            self.assertNotIn("not described", proc.stdout)
            self.assertTrue(os.path.isfile(os.path.join(log_dir, "graph.json")))


if __name__ == "__main__":
    unittest.main()
EOF
""",
    visibility = ["//visibility:private"],
)

py_test(
    name = "bundle_test",
    size = "small",
    srcs = ["bundle_test.py"],
    data = [":robot_bundle"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
