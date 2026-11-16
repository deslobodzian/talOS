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

    # REQ-OPS-48: an installed bundle runs from its installed files alone.
    # A bundle whose files link into a build tree (Bazel's sandbox stages a
    # test's runfiles so; a bundle copied with its links) must not install
    # as links that dangle once that tree is cleaned; a link that stays
    # inside the bundle stays a link. A link that cannot be copied as files
    # refuses the install by its name before anything is written, and the
    # bundle installed before stays: one that names nothing, directly or
    # inside a directory a leaving link names; a loop inside such a
    # directory and a link to the bundle's own ancestor, which the copy
    # would follow until ELOOP or a full disk; and one naming a FIFO, socket
    # or device, which the copy would block on or read forever.
    def test_install_copies_links_that_leave_the_bundle_as_files(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            src = os.path.join(tmp, "bundle")
            build_tree = os.path.join(tmp, "build_tree")
            os.makedirs(os.path.join(src, "config"))
            os.makedirs(build_tree)
            with open(os.path.join(src, "run.sh"), "w", encoding="utf-8") as f:
                f.write("#!/bin/sh\nexit 0\n")
            os.chmod(os.path.join(src, "run.sh"), 0o755)
            with open(os.path.join(src, "config", "robot.toml"), "w",
                      encoding="utf-8") as f:
                f.write("[robot]\n")
            with open(os.path.join(build_tree, "node"), "w",
                      encoding="utf-8") as f:
                f.write("built\n")
            os.symlink(os.path.join(build_tree, "node"),
                       os.path.join(src, "node"))
            os.symlink("config/robot.toml", os.path.join(src, "robot.toml"))
            prefix = os.path.join(tmp, "prefix")
            current = os.path.join(prefix, "current")

            def install():
                # No stub here is a member: nothing to warm.
                return subprocess.run(
                    ["bash", deploy_script(), "install", src, "--prefix",
                     prefix, "--no-warm-up"],
                    capture_output=True, text=True, timeout=60)

            proc = install()
            out = proc.stdout + proc.stderr
            self.assertEqual(proc.returncode, 0, out)
            installed = os.readlink(current)
            node = os.path.join(installed, "node")
            self.assertFalse(os.path.islink(node), out)
            with open(node, encoding="utf-8") as f:
                self.assertEqual(f.read(), "built\n")
            self.assertEqual(
                os.readlink(os.path.join(installed, "robot.toml")),
                "config/robot.toml", out)
            libs = os.path.join(build_tree, "libs")
            os.makedirs(libs)
            os.symlink(os.path.join(build_tree, "cleaned"),
                       os.path.join(libs, "gone"))
            loop = os.path.join(build_tree, "loop")
            os.makedirs(loop)
            os.symlink(".", os.path.join(loop, "self"))
            os.mkfifo(os.path.join(build_tree, "pipe"))
            # Each row's link, added alone; the name install must give.
            # The ancestor row is last: before the fix its copy is the
            # largest.
            rows = (
                ("missing_library", os.path.join(build_tree, "cleaned"),
                 "missing_library"),
                ("libs", libs, "libs/gone"),
                ("loop", loop, "loop/self"),
                ("pipe", os.path.join(build_tree, "pipe"), "pipe"),
                ("up", tmp, "up"),
            )
            for link, target, named in rows:
                os.symlink(target, os.path.join(src, link))
                proc = install()
                os.unlink(os.path.join(src, link))
                out = f"{link}: " + proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                # Named relative to the bundle: the walk before the copy,
                # not a failure of the copy itself.
                self.assertIn(f"error: {named} -> ", out)
                self.assertEqual(os.readlink(current), installed, out)
                self.assertEqual(os.listdir(os.path.join(prefix, "bundles")),
                                 [os.path.basename(installed)], out)


if __name__ == "__main__":
    unittest.main()
