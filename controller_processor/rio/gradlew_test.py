"""Tests //controller_processor/rio:gradlew staging, stamping and forwarding.

REQ-DEV-30, REQ-OPS-25, REQ-CONTRACT-30: the Bazel-exported gradlew runner
stages controller_processor/rio and its talOS sources into a cache directory
without mutating the checkout, stamps -PtalosRevision/-PtalosDirty from the
talOS checkout, forwards the caller's .wpilib/wpilib_preferences.json and CLI
arguments, and preserves mtimes for unchanged files.
"""

import json
import os
import stat
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def _make_fake_jdk(root):
    """Creates a minimal fake JAVA_HOME whose bin/java records PWD and gradle
    arguments (everything after GradleWrapperMain) as JSON."""
    jdk = os.path.join(root, "fake_jdk")
    bin_dir = os.path.join(jdk, "bin")
    os.makedirs(bin_dir, exist_ok=True)
    java = os.path.join(bin_dir, "java")
    with open(java, "w", encoding="utf-8") as f:
        f.write(
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "args = sys.argv[1:]\n"
            "marker = 'org.gradle.wrapper.GradleWrapperMain'\n"
            "idx = args.index(marker) + 1 if marker in args else 0\n"
            "payload = {'cwd': os.getcwd(), 'gradle_args': args[idx:]}\n"
            "with open(os.environ['FAKE_JAVA_OUT'], 'w', encoding='utf-8') as out:\n"
            "    json.dump(payload, out)\n"
        )
    os.chmod(java, 0o755)
    return jdk


class GradlewRunnerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = runfiles.Create().Rlocation(os.environ["RIO_GRADLEW"])
        assert cls.runner and os.path.isfile(cls.runner), cls.runner

    def _run_gradlew(self, tmp, args, workspace_dir=None, cache_dir=None):
        jdk = _make_fake_jdk(tmp)
        out_json = os.path.join(tmp, "java_out.json")
        cache = cache_dir or os.path.join(tmp, "cache")
        env = dict(os.environ)
        env["JAVA_HOME"] = jdk
        env["FAKE_JAVA_OUT"] = out_json
        env["TALOS_RIO_CACHE_DIR"] = cache
        if workspace_dir is not None:
            env["BUILD_WORKSPACE_DIRECTORY"] = workspace_dir
        else:
            env.pop("BUILD_WORKSPACE_DIRECTORY", None)
        proc = subprocess.run(
            [self.runner, *args],
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        with open(out_json, encoding="utf-8") as f:
            return json.load(f), cache

    # REQ-DEV-30, REQ-CONTRACT-30
    def test_stages_project_and_talos_sources_and_forwards_args(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, cache = self._run_gradlew(
                tmp, ["buildRio", "-Pcompanion=10.0.0.2", "--offline"]
            )
            rio_dir = os.path.realpath(
                os.path.join(cache, "controller_processor", "rio")
            )
            self.assertEqual(os.path.realpath(result["cwd"]), rio_dir)

            # All six gateway C++ translation units and required headers staged.
            for rel in (
                "controller_processor/rio/build.gradle",
                "controller_processor/rio/gradlew",
                "controller_processor/rio/src/main/cpp/Robot.cpp",
                "talOS/protocol/frame.cc",
                "talOS/protocol/udp.cc",
                "talOS/hardware/config.cc",
                "talOS/hardware/messages.cc",
                "talOS/hardware/gateway.cc",
                "talOS/driver_station/driver_station.cc",
                "talOS/driver_station/schedule.h",
                "talOS/process/durable_sync.h",
            ):
                staged = os.path.join(cache, rel)
                self.assertTrue(os.path.isfile(staged), f"missing {rel}")
                self.assertTrue(
                    os.stat(staged).st_mode & stat.S_IWUSR,
                    f"staged {rel} should be user-writable",
                )

            # Staged preferences carry no built-in team number.
            with open(
                os.path.join(rio_dir, ".wpilib", "wpilib_preferences.json"),
                encoding="utf-8",
            ) as f:
                prefs = json.load(f)
            self.assertNotIn("teamNumber", prefs)

            # CLI args are forwarded and revision/dirty stamped from checkout.
            gargs = result["gradle_args"]
            self.assertIn("buildRio", gargs)
            self.assertIn("-Pcompanion=10.0.0.2", gargs)
            self.assertIn("--offline", gargs)
            rev_args = [a for a in gargs if a.startswith("-PtalosRevision=")]
            dirty_args = [a for a in gargs if a.startswith("-PtalosDirty=")]
            if rev_args:
                self.assertRegex(
                    rev_args[0], r"^-PtalosRevision=[0-9a-f]{16}$"
                )
                self.assertIn(
                    dirty_args[0], ("-PtalosDirty=true", "-PtalosDirty=false")
                )

    # REQ-DEV-30, REQ-OPS-25
    def test_forwards_caller_wpilib_preferences_and_resets_when_absent(self):
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            caller = os.path.join(tmp, "robot_repo")
            os.makedirs(os.path.join(caller, ".wpilib"))
            with open(
                os.path.join(caller, ".wpilib", "wpilib_preferences.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump({"projectYear": "2026", "teamNumber": 1234}, f)

            self._run_gradlew(
                tmp, ["deploy", "-Pcompanion=10.12.34.11"], workspace_dir=caller, cache_dir=cache
            )
            staged_prefs = os.path.join(
                cache,
                "controller_processor",
                "rio",
                ".wpilib",
                "wpilib_preferences.json",
            )
            with open(staged_prefs, encoding="utf-8") as f:
                self.assertEqual(json.load(f)["teamNumber"], 1234)

            # A subsequent run from a workspace without preferences must not
            # keep the previous workspace's team number.
            empty_caller = os.path.join(tmp, "other_repo")
            os.makedirs(empty_caller)
            self._run_gradlew(
                tmp, ["phoenixTest"], workspace_dir=empty_caller, cache_dir=cache
            )
            with open(staged_prefs, encoding="utf-8") as f:
                self.assertNotIn("teamNumber", json.load(f))

    # REQ-CONTRACT-30
    def test_explicit_caller_stamp_is_not_duplicated(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, _ = self._run_gradlew(
                tmp,
                [
                    "buildRio",
                    "-Pcompanion=10.0.0.2",
                    "-PtalosRevision=0123456789abcdef",
                    "-PtalosDirty=false",
                ],
            )
            revs = [
                a
                for a in result["gradle_args"]
                if a.startswith("-PtalosRevision=")
            ]
            dirties = [
                a
                for a in result["gradle_args"]
                if a.startswith("-PtalosDirty=")
            ]
            self.assertEqual(revs, ["-PtalosRevision=0123456789abcdef"])
            self.assertEqual(dirties, ["-PtalosDirty=false"])


if __name__ == "__main__":
    unittest.main()
