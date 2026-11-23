"""Tests //controller_processor/rio:gradlew staging, stamping and forwarding.

REQ-DEV-30, REQ-OPS-25, REQ-CONTRACT-30: the Bazel-exported gradlew runner
stages controller_processor/rio and its talOS sources into a cache directory
without mutating the checkout, stamps -PtalosRevision/-PtalosDirty from the
talOS checkout, forwards the caller's .wpilib/wpilib_preferences.json and CLI
arguments, and preserves mtimes for unchanged files.
"""

import fcntl
import json
import os
import stat
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def _make_fake_jdk(root, extra_script=""):
    """Creates a minimal fake JAVA_HOME whose bin/java records PWD and gradle
    arguments (everything after GradleWrapperMain) as JSON."""
    jdk = os.path.join(root, "fake_jdk")
    bin_dir = os.path.join(jdk, "bin")
    os.makedirs(bin_dir, exist_ok=True)
    java = os.path.join(bin_dir, "java")
    with open(java, "w", encoding="utf-8") as f:
        f.write(
            "#!/usr/bin/env python3\n"
            "import fcntl, json, os, signal, sys\n"
            "args = sys.argv[1:]\n"
            "marker = 'org.gradle.wrapper.GradleWrapperMain'\n"
            "idx = args.index(marker) + 1 if marker in args else 0\n"
            "payload = {'cwd': os.getcwd(), 'gradle_args': args[idx:]}\n"
            + extra_script
            + "with open(os.environ['FAKE_JAVA_OUT'], 'w', encoding='utf-8') as out:\n"
            "    json.dump(payload, out)\n"
        )
    os.chmod(java, 0o755)
    return jdk


class GradlewRunnerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = runfiles.Create().Rlocation(os.environ["RIO_GRADLEW"])
        assert cls.runner and os.path.isfile(cls.runner), cls.runner

    def _run_gradlew_raw(
        self,
        tmp,
        args,
        workspace_dir=None,
        cache_dir=None,
        xdg_cache_home=None,
        extra_env=None,
        extra_script="",
    ):
        jdk = _make_fake_jdk(tmp, extra_script=extra_script)
        out_json = os.path.join(tmp, "java_out.json")
        if os.path.exists(out_json):
            os.unlink(out_json)
        env = dict(os.environ)
        env["JAVA_HOME"] = jdk
        env["FAKE_JAVA_OUT"] = out_json
        if cache_dir is not None:
            env["TALOS_RIO_CACHE_DIR"] = cache_dir
        else:
            env.pop("TALOS_RIO_CACHE_DIR", None)
        if xdg_cache_home is not None:
            env["XDG_CACHE_HOME"] = xdg_cache_home
        if workspace_dir is not None:
            env["BUILD_WORKSPACE_DIRECTORY"] = workspace_dir
        else:
            env.pop("BUILD_WORKSPACE_DIRECTORY", None)
        if extra_env:
            env.update(extra_env)
        proc = subprocess.run(
            [self.runner, *args],
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        return proc, out_json

    def _run_gradlew(
        self,
        tmp,
        args,
        workspace_dir=None,
        cache_dir=None,
        xdg_cache_home=None,
        extra_env=None,
        extra_script="",
    ):
        if cache_dir is None and xdg_cache_home is None:
            cache_dir = os.path.join(tmp, "cache")
        proc, out_json = self._run_gradlew_raw(
            tmp,
            args,
            workspace_dir=workspace_dir,
            cache_dir=cache_dir,
            xdg_cache_home=xdg_cache_home,
            extra_env=extra_env,
            extra_script=extra_script,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        with open(out_json, encoding="utf-8") as f:
            return json.load(f), cache_dir

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
                # Transitive deps of //talOS/hardware must also be staged.
                "talOS/schema/identity.h",
                "talOS/schema/descriptor.h",
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
    def test_refuses_caller_stamp_override_in_git_checkout(self):
        spellings = [
            ["buildRio", "-PtalosRevision=0123456789abcdef", "-PtalosDirty=false"],
            ["buildRio", "-P", "talosRevision=0123456789abcdef"],
            ["buildRio", "-P", "talosDirty=false"],
            ["buildRio", "-P=talosRevision=0123456789abcdef"],
            ["buildRio", "-P=talosDirty=false"],
            ["buildRio", "--project-prop=talosRevision=0123456789abcdef"],
            ["buildRio", "--project-prop", "talosDirty=false"],
            ["buildRio", "-Dorg.gradle.project.talosRevision=0123456789abcdef"],
            ["buildRio", "-D", "org.gradle.project.talosDirty=false"],
            ["buildRio", "-D=org.gradle.project.talosRevision=0123456789abcdef"],
            ["buildRio", "-D=org.gradle.project.talosDirty=false"],
            [
                "buildRio",
                "--system-prop=org.gradle.project.talosRevision=0123456789abcdef",
            ],
            ["buildRio", "--system-prop", "org.gradle.project.talosDirty=false"],
        ]
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            for cli_args in spellings:
                with self.subTest(args=cli_args):
                    proc, out_json = self._run_gradlew_raw(
                        tmp, cli_args, cache_dir=cache
                    )
                    self.assertNotEqual(
                        proc.returncode,
                        0,
                        f"expected refusal for {cli_args}, got exit 0",
                    )
                    self.assertIn("talosRevision", proc.stderr)
                    self.assertIn("talosDirty", proc.stderr)
                    self.assertFalse(
                        os.path.exists(out_json),
                        f"Gradle should not have been invoked for {cli_args}",
                    )

    # REQ-DEV-30, REQ-OPS-25
    def test_distinct_cache_dirs_per_caller_workspace_and_prefs(self):
        with tempfile.TemporaryDirectory() as tmp:
            xdg = os.path.join(tmp, "xdg")
            caller_a = os.path.join(tmp, "robot_a")
            caller_b = os.path.join(tmp, "robot_b")
            caller_c = os.path.join(tmp, "robot_c")
            os.makedirs(os.path.join(caller_a, ".wpilib"))
            os.makedirs(os.path.join(caller_b, ".wpilib"))
            os.makedirs(caller_c)
            with open(
                os.path.join(caller_a, ".wpilib", "wpilib_preferences.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump({"projectYear": "2026", "teamNumber": 1111}, f)
            with open(
                os.path.join(caller_b, ".wpilib", "wpilib_preferences.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump({"projectYear": "2026", "teamNumber": 2222}, f)

            res_a, _ = self._run_gradlew(
                tmp, ["buildRio"], workspace_dir=caller_a, xdg_cache_home=xdg
            )
            res_b, _ = self._run_gradlew(
                tmp, ["buildRio"], workspace_dir=caller_b, xdg_cache_home=xdg
            )
            res_c, _ = self._run_gradlew(
                tmp, ["buildRio"], workspace_dir=caller_c, xdg_cache_home=xdg
            )
            res_a2, _ = self._run_gradlew(
                tmp, ["buildRio"], workspace_dir=caller_a, xdg_cache_home=xdg
            )

            cwd_a = os.path.realpath(res_a["cwd"])
            cwd_b = os.path.realpath(res_b["cwd"])
            cwd_c = os.path.realpath(res_c["cwd"])
            self.assertEqual(cwd_a, os.path.realpath(res_a2["cwd"]))
            self.assertEqual(
                len({cwd_a, cwd_b, cwd_c}),
                3,
                f"expected distinct cache dirs, got {cwd_a}, {cwd_b}, {cwd_c}",
            )

            # Each workspace's staged preferences remain intact after the others run.
            with open(
                os.path.join(cwd_a, ".wpilib", "wpilib_preferences.json"),
                encoding="utf-8",
            ) as f:
                self.assertEqual(json.load(f)["teamNumber"], 1111)
            with open(
                os.path.join(cwd_b, ".wpilib", "wpilib_preferences.json"),
                encoding="utf-8",
            ) as f:
                self.assertEqual(json.load(f)["teamNumber"], 2222)
            with open(
                os.path.join(cwd_c, ".wpilib", "wpilib_preferences.json"),
                encoding="utf-8",
            ) as f:
                self.assertNotIn("teamNumber", json.load(f))

    # REQ-DEV-25, REQ-DEV-30
    def test_flock_held_while_gradle_runs_and_released_on_exit(self):
        probe_lock_script = (
            "lock_path = os.environ['EXPECTED_LOCK_PATH']\n"
            "with open(lock_path, 'a+', encoding='utf-8') as lf:\n"
            "    try:\n"
            "        fcntl.flock(lf.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)\n"
            "        payload['lock_held'] = False\n"
            "        fcntl.flock(lf.fileno(), fcntl.LOCK_UN)\n"
            "    except BlockingIOError:\n"
            "        payload['lock_held'] = True\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            lock_path = cache + ".lock"
            result, _ = self._run_gradlew(
                tmp,
                ["phoenixTest"],
                cache_dir=cache,
                extra_env={"EXPECTED_LOCK_PATH": lock_path},
                extra_script=probe_lock_script,
            )
            self.assertTrue(
                result["lock_held"],
                "cache flock must remain held while Gradle is running",
            )
            # After gradlew exits, the lock must be released.
            with open(lock_path, "a+", encoding="utf-8") as lf:
                fcntl.flock(lf.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(lf.fileno(), fcntl.LOCK_UN)

    # REQ-DEV-25, REQ-DEV-30
    def test_sigterm_forwarded_to_gradle_and_releases_lock(self):
        # Block SIGTERM before signalling readiness and consume it with
        # sigwait() rather than signal.pause(): pause() sleeps forever if
        # SIGTERM trips CPython's C handler just before the pause() syscall.
        signal_script = (
            "signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM})\n"
            "signal.alarm(10)\n"
            "print('FAKE_JAVA_READY', flush=True)\n"
            "signum = signal.sigwait({signal.SIGTERM})\n"
            "payload['received_signal'] = signum\n"
            "with open(os.environ['FAKE_JAVA_OUT'], 'w', encoding='utf-8') as out:\n"
            "    json.dump(payload, out)\n"
            "sys.exit(128 + signum)\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            cache = os.path.join(tmp, "cache")
            lock_path = cache + ".lock"
            jdk = _make_fake_jdk(tmp, extra_script=signal_script)
            out_json = os.path.join(tmp, "java_out.json")
            env = dict(os.environ)
            env["JAVA_HOME"] = jdk
            env["FAKE_JAVA_OUT"] = out_json
            env["TALOS_RIO_CACHE_DIR"] = cache
            env.pop("BUILD_WORKSPACE_DIRECTORY", None)
            proc = subprocess.Popen(
                [self.runner, "phoenixTest"],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                ready = False
                for line in proc.stdout:
                    if line.strip() == "FAKE_JAVA_READY":
                        ready = True
                        break
                self.assertTrue(ready, "fake java did not reach ready state")
                proc.terminate()
                rc = proc.wait(timeout=10)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=5)
                proc.stdout.close()
                proc.stderr.close()

            self.assertEqual(rc, 143)
            with open(out_json, encoding="utf-8") as f:
                result = json.load(f)
            self.assertEqual(result["received_signal"], 15)
            with open(lock_path, "a+", encoding="utf-8") as lf:
                fcntl.flock(lf.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                fcntl.flock(lf.fileno(), fcntl.LOCK_UN)


if __name__ == "__main__":
    unittest.main()
