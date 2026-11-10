"""Tests bundle_builder.py's config staging (relative layout and identity),
its lib/ staging of C++ nodes' shared libraries and its run.sh, and
tools/deploy.sh's bundle selection, pruning and session commands."""

import json
import os
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles


def put(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


class BundleBuilderTest(unittest.TestCase):
    def build(self, tmp, files, nodes=(), check=True):
        """Bundles `files` ({short_path: text}); the first is the config.
        `nodes`: {name: {library basename: bytes}} C++ nodes and the shared
        libraries each links."""
        src = os.path.join(tmp, "src")
        entries = []
        for short_path, text in files.items():
            put(os.path.join(src, short_path), text)
            entries.append({"src_path": os.path.join(src, short_path),
                            "short_path": short_path})
        launcher = os.path.join(src, "launcher")
        put(launcher, "")
        node_specs = []
        for name, libs in dict(nodes).items():
            binary = os.path.join(src, "bin", name)
            put(binary, "")
            shared = []
            for lib, text in libs.items():
                path = os.path.join(src, "solib", name, lib)
                put(path, text)
                shared.append({"src_path": path, "name": lib})
            node_specs.append({"name": name, "target": f"//{name}:node",
                               "language": "cpp", "src_path": binary,
                               "runfiles": [], "shared_libs": shared})
        spec = os.path.join(tmp, "spec.json")
        with open(spec, "w", encoding="utf-8") as f:
            json.dump({"bundle_name": "b", "launcher_path": launcher,
                       "config": entries[0], "config_files": entries[1:],
                       "nodes": node_specs}, f)
        out = os.path.join(tmp, "out")
        builder = runfiles.Create().Rlocation("_main/talOS/build/bundle_builder.py")
        proc = subprocess.run([sys.executable, builder, "--spec", spec,
                               "--out", out], capture_output=True, text=True)
        if not check:
            return proc
        self.assertEqual(proc.returncode, 0, proc.stderr)
        with open(os.path.join(out, "bundle_manifest.toml"), encoding="utf-8") as f:
            manifest = f.read()
        return out, manifest

    def echo_launcher(self, out):
        """Replaces the bundle's launcher with one that prints its argv."""
        put(os.path.join(out, "launcher"), "#!/usr/bin/env python3\n"
            "import json, sys\nprint(json.dumps(sys.argv[1:]))\n")

    def test_files_keep_their_paths_relative_to_the_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, manifest = self.build(tmp, {
                "bot/config/robot.toml": "[robot]\n",
                "bot/config/subsystems/arm.toml": "arm\n",
                "bot/config/mechanisms/wrist.toml": "wrist\n",
                "bot/shared/arm_defaults.toml": "defaults\n",
            })
            for rel in ("config/config/robot.toml",
                        "config/config/subsystems/arm.toml",
                        "config/config/mechanisms/wrist.toml",
                        "config/shared/arm_defaults.toml"):
                self.assertTrue(os.path.isfile(os.path.join(out, rel)), rel)
            self.assertIn('config_path = "config/config/robot.toml"', manifest)
            with open(os.path.join(out, "run.sh"), encoding="utf-8") as f:
                self.assertIn('--config "config/config/robot.toml"', f.read())

    def test_config_in_its_own_directory_stays_at_the_top(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, manifest = self.build(tmp, {
                "bot/config/robot.toml": "[robot]\n",
                "bot/config/subsystems/arm.toml": "arm\n",
            })
            self.assertTrue(os.path.isfile(os.path.join(out, "config/robot.toml")))
            self.assertTrue(os.path.isfile(
                os.path.join(out, "config/subsystems/arm.toml")))
            self.assertIn('config_path = "config/robot.toml"', manifest)

    def test_identity_is_relative_paths_and_contents(self):
        def sha(files):
            with tempfile.TemporaryDirectory() as tmp:
                _, manifest = self.build(tmp, files)
            return next(line for line in manifest.splitlines()
                        if line.startswith("config_sha256"))
        base = {"c/robot.toml": "[robot]\n", "c/subsystems/a.toml": "x\n"}
        # Relocating the whole tree keeps the identity.
        self.assertEqual(sha(base), sha({
            "elsewhere/deep/robot.toml": "[robot]\n",
            "elsewhere/deep/subsystems/a.toml": "x\n"}))
        for changed in (
                {"c/robot.toml": "[robot]\n", "c/other/a.toml": "x\n"},
                # Only the root config renamed.
                {"c/practice.toml": "[robot]\n", "c/subsystems/a.toml": "x\n"},
                # Only the root config moved relative to its dependency.
                {"c/robots/robot.toml": "[robot]\n",
                 "c/subsystems/a.toml": "x\n"},
                {"c/robot.toml": "[robot]\n", "c/subsystems/a.toml": "y\n"},
                # Bytes moved across a file boundary.
                {"c/robot.toml": "[robot]\nx\n", "c/subsystems/a.toml": ""}):
            with self.subTest(changed):
                self.assertNotEqual(sha(base), sha(changed))

    def test_cpp_nodes_shared_libraries_land_once_in_lib(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, _ = self.build(tmp, {"c/robot.toml": "[robot]\n"}, nodes={
                "drivetrain": {"libwpimath.dylib": "math", "libwpiutil.dylib": "util"},
                "estimator": {"libwpimath.dylib": "math"},
                "shooter": {},
            })
            self.assertEqual(sorted(os.listdir(os.path.join(out, "lib"))),
                             ["libwpimath.dylib", "libwpiutil.dylib"])
            with open(os.path.join(out, "lib", "libwpimath.dylib"),
                      encoding="utf-8") as f:
                self.assertEqual(f.read(), "math")

    def test_two_different_libraries_under_one_name_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = self.build(tmp, {"c/robot.toml": "[robot]\n"}, nodes={
                "a": {"libwpimath.so": "2026.2.1"},
                "b": {"libwpimath.so": "2027.0.0"},
            }, check=False)
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("libwpimath.so", proc.stderr)
            self.assertIn("'a'", proc.stderr)

    def test_run_script_passes_output_root_without_a_session_suffix(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, _ = self.build(tmp, {"c/robot.toml": "[robot]\n"})
            self.echo_launcher(out)
            env = dict(os.environ, TALOS_LOG_DIR=os.path.join(tmp, "log root"))
            for explicit in ([], ["--output-dir", os.path.join(tmp, "override")],
                             ["--output-dir=" + os.path.join(tmp, "override")]):
                with self.subTest(explicit=explicit):
                    proc = subprocess.run([os.path.join(out, "run.sh"), *explicit],
                                          env=env, capture_output=True, text=True)
                    self.assertEqual(proc.returncode, 0, proc.stderr)
                    args = json.loads(proc.stdout)
                    if explicit:
                        self.assertEqual(args[-len(explicit):], explicit)
                        self.assertNotIn(env["TALOS_LOG_DIR"], args)
                    else:
                        self.assertEqual(args[args.index("--output-dir") + 1],
                                         env["TALOS_LOG_DIR"])

    def test_run_script_through_a_link_passes_the_physical_bundle(self):
        # The boot service starts <prefix>/current/run.sh. Node binaries come
        # from --bundle, so a path through the link would restart nodes from
        # whatever bundle `current` selects later.
        with tempfile.TemporaryDirectory() as tmp:
            out, _ = self.build(tmp, {"c/robot.toml": "[robot]\n"})
            self.echo_launcher(out)
            current = os.path.join(tmp, "current")
            os.symlink(out, current)
            proc = subprocess.run([os.path.join(current, "run.sh")],
                                  capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            args = json.loads(proc.stdout)
            self.assertEqual(args[args.index("--bundle") + 1],
                             os.path.realpath(out))


class DeployTest(unittest.TestCase):
    def deploy(self, prefix, *args, cwd=None):
        script = os.path.abspath(
            runfiles.Create().Rlocation("_main/tools/deploy.sh"))
        return subprocess.run(["bash", script, *args, "--prefix", prefix],
                              cwd=cwd, capture_output=True, text=True)

    def session(self, prefix, name, timestamp):
        path = os.path.join(prefix, "logs", name)
        put(os.path.join(path, "manifest.json"), json.dumps({
            "session_id": name, "config_path": "config/robot.toml",
            "config_sha256": "abcdef", "nodes": []}))
        put(os.path.join(path, "arm.0.tlog"), "first attempt")
        put(os.path.join(path, "arm.1.tlog"), "restarted attempt")
        os.utime(os.path.join(path, "manifest.json"), ns=(timestamp, timestamp))
        return path

    def bundle(self, prefix, stamp):
        """An installed bundle, read-only as install leaves it."""
        path = os.path.join(prefix, "bundles", stamp)
        put(os.path.join(path, "bin", "node"), "")
        put(os.path.join(path, "run.sh"), "")
        for parent, dirs, files in os.walk(path):
            for name in dirs + files:
                os.chmod(os.path.join(parent, name), 0o555)
        os.chmod(path, 0o555)
        return path

    def test_logs_lists_sessions_and_selects_newest_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            # Names deliberately sort opposite to manifest modification time.
            older = self.session(tmp, "z_old", 1_000_000_000)
            newer = self.session(tmp, "a_new", 2_000_000_000)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn(older, proc.stdout)
            self.assertIn(newer, proc.stdout)
            self.assertIn(f"newest session manifest: {newer}/manifest.json",
                          proc.stdout)
            self.assertIn("session a_new config config/robot.toml", proc.stdout)

    def test_logs_does_not_require_an_installed_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("no session directories", proc.stdout)

    def test_fetch_copies_complete_sessions_and_refuses_merging(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = os.path.join(tmp, "prefix")
            sessions = [self.session(prefix, name, index + 1)
                        for index, name in enumerate(("one", "two"))]
            dest = os.path.join(tmp, "collected logs")
            proc = self.deploy(prefix, "fetch", dest)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            for session in sessions:
                copied = os.path.join(dest, os.path.basename(session))
                self.assertEqual(sorted(os.listdir(copied)),
                                 ["arm.0.tlog", "arm.1.tlog", "manifest.json"])
                for name in os.listdir(copied):
                    with open(os.path.join(session, name), "rb") as source:
                        with open(os.path.join(copied, name), "rb") as target:
                            self.assertEqual(source.read(), target.read())
            proc = self.deploy(prefix, "fetch", dest)
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("already exists", proc.stderr)
            proc = self.deploy(prefix, "fetch", os.path.join(prefix, "logs", "copy"))
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("outside the log root", proc.stderr)

    def test_install_with_a_relative_prefix_selects_the_copy(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            build = os.path.join(tmp, "build")
            put(os.path.join(build, "run.sh"), "#!/bin/sh\n")
            os.chmod(os.path.join(build, "run.sh"), 0o755)
            proc = self.deploy("robot", "install", build, cwd=tmp)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertTrue(os.path.isfile(
                os.path.join(tmp, "robot", "current", "run.sh")))

    def test_rollback_refuses_a_previous_bundle_that_is_gone(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            current = self.bundle(tmp, "20260102_000000_1")
            os.symlink(current, os.path.join(tmp, "current"))
            os.symlink(os.path.join(tmp, "bundles", "20260101_000000_1"),
                       os.path.join(tmp, "previous"))
            proc = self.deploy(tmp, "rollback")
            self.assertNotEqual(proc.returncode, 0)
            self.assertEqual(os.readlink(os.path.join(tmp, "current")), current)

    def test_prune_keeps_the_newest_and_the_selected_bundles(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            stamps = [f"2026010{day}_000000_1" for day in range(1, 7)]
            for stamp in stamps:
                self.bundle(tmp, stamp)
            # Both selections older than the newest two, linked relatively:
            # a selection is kept however its target is spelled.
            os.symlink(os.path.join("bundles", stamps[0]),
                       os.path.join(tmp, "current"))
            os.symlink(os.path.join("bundles", stamps[2]),
                       os.path.join(tmp, "previous"))
            proc = self.deploy(tmp, "prune", "--keep", "2")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(sorted(os.listdir(os.path.join(tmp, "bundles"))),
                             [stamps[0], stamps[2], stamps[4], stamps[5]])

if __name__ == "__main__":
    unittest.main()
