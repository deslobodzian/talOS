"""tools/deploy.sh install selects only a bundle its own launcher accepts.

The bench robot's deployable bundle installs and is selected, showing
nothing of the launcher's but its warnings; a copy whose graph the launcher
accepts with a warning installs and shows it; a copy with a misspelled
parameter is refused with the launcher's own finding, and `current` stays
on the bundle installed before."""

import os
import shutil
import subprocess
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

from bench.main_processor.deploy_test_support import deploy_script


class InstallCheckTest(unittest.TestCase):
    # REQ-OPS-50, REQ-OPS-01, REQ-CONTRACT-09: install selects only a bundle its own
    # launcher accepts, so the boot service never restarts a launcher that
    # refuses every launch.
    def test_install_refuses_a_bundle_the_launcher_refuses(self):
        src = runfiles.Create().Rlocation(
            "_main/bench/main_processor/bench_bundle")
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            prefix = os.path.join(tmp, "prefix")
            current = os.path.join(prefix, "current")

            def install(bundle, to=prefix):
                return subprocess.run(
                    ["bash", deploy_script(), "install", bundle,
                     "--prefix", to],
                    capture_output=True, text=True, timeout=120)

            def shown(proc):
                return [line for line in proc.stdout.splitlines()
                        if line.startswith("  | ")]

            proc = install(src)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            good = os.readlink(current)
            # Accepted: nothing of the launcher's is shown but a warning.
            for line in shown(proc):
                self.assertTrue(line.startswith("  |   warning: "), proc.stdout)
            with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                config = tomllib.load(f)["config_path"]
            # A graph the launcher accepts with a warning (a policy lane
            # nobody writes yet) installs, and the warning is shown.
            # The sandbox links each input file; copy what they name.
            warned = os.path.join(tmp, "warned")
            shutil.copytree(src, warned)
            robot = os.path.join(warned, os.path.dirname(config), "robot.toml")
            with open(robot, encoding="utf-8") as f:
                text = f.read()
            self.assertIn('teleop = ["teleop"]\n', text)
            os.chmod(robot, 0o644)
            with open(robot, "w", encoding="utf-8") as f:
                f.write(text.replace('teleop = ["teleop"]\n',
                                     'teleop = ["teleop", "align"]\n'))
            proc = install(warned, os.path.join(tmp, "warned_prefix"))
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            lines = shown(proc)
            self.assertTrue(any("/target/align" in line for line in lines),
                            proc.stdout)
            for line in lines:
                self.assertTrue(line.startswith("  |   warning: "), proc.stdout)
            # The smallest graph error a person makes: a misspelled
            # parameter, a key no node reads.
            bad = os.path.join(tmp, "bad")
            shutil.copytree(src, bad)
            motor = os.path.join(bad, os.path.dirname(config), "subsystems",
                                 "motor.toml")
            with open(motor, encoding="utf-8") as f:
                text = f.read()
            self.assertIn("max_bench_voltage_v = 2.0\n", text)
            os.chmod(motor, 0o644)
            with open(motor, "w", encoding="utf-8") as f:
                f.write(text.replace("max_bench_voltage_v = 2.0\n",
                                     "max_bench_voltage_v = 2.0\n"
                                     "max_bench_voltag_v = 2.0\n"))
            proc = install(bad)
            out = proc.stdout + proc.stderr
            self.assertNotEqual(proc.returncode, 0, out)
            # The launcher's own finding and refusal, and what stays.
            self.assertIn("max_bench_voltag_v", out)
            self.assertIn("refusing to start", out)
            # The key is the reason: every member of the copy answered.
            self.assertNotIn("did not answer --describe", out)
            self.assertNotIn("(not described)", out)
            self.assertIn(f"{bad} not installed", out)
            self.assertIn(f"; current stays {good}\n", out)
            self.assertEqual(os.readlink(current), good, out)
            self.assertFalse(os.path.lexists(os.path.join(prefix, "previous")),
                             out)
            self.assertEqual(os.listdir(os.path.join(prefix, "bundles")),
                             [os.path.basename(good)], out)


if __name__ == "__main__":
    unittest.main()
