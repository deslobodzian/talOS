"""Tests for sim.sh --config and failed-build diagnostics.

sim.sh runs from a copy at the root of a scratch workspace, with a `bazel`
on PATH that runs the real manifest check (//tools:sync_manifest --check)
and a resolver stub that stops sim.sh right after it (or stubs a failing
build into a test-local TALOS_SIM_LOG_DIR), so no simulation starts and
nothing is written outside the workspace.
"""

import glob
import json
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles

from tools import sharded_test_loader, sync_manifest

CONFIG = "bot/robot.toml"
ROBOT = """\
[robot]
name = "bot"

[subsystems.arm]
node = "//bot:node"

[subsystems.arm.motors.arm_motor]
can_id = 1
"""


def put(path, text, mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    if mode is not None:
        os.chmod(path, mode)


class SimShConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._runfiles = runfiles.Create()
        cls._sim_src = cls._runfiles.Rlocation("_main/sim.sh")
        cls._manifest_rel = sync_manifest.manifest_path(CONFIG)
        # Generate the current manifest once per shard rather than spawning
        # the C++ resolver in every test's setUp().
        with tempfile.TemporaryDirectory() as tmpl:
            put(os.path.join(tmpl, "bot", "BUILD"), "")
            put(os.path.join(tmpl, CONFIG), ROBOT)
            sync_manifest.write(tmpl, CONFIG)
            with open(os.path.join(tmpl, cls._manifest_rel),
                      encoding="utf-8") as f:
                cls._manifest_text = f.read()

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)
        self.root = os.path.join(self.tmp, "ws")
        self.sim = os.path.join(self.root, "sim.sh")
        os.makedirs(self.root)
        shutil.copy(self._sim_src, self.sim)
        put(os.path.join(self.root, "bot", "BUILD"), "")
        put(os.path.join(self.root, CONFIG), ROBOT)
        put(os.path.join(self.root, self._manifest_rel), self._manifest_text)
        os.makedirs(os.path.join(self.root, "sub"))

        self.log = os.path.join(self.tmp, "checked")
        check = ("import sys; from tools import sync_manifest; "
                 "sys.exit(sync_manifest.main(sys.argv[1:]))")
        bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            '[ "$1" = run ] || { echo "bazel stub: $*" >&2; exit 97; }\n'
            'while [ "$1" != -- ]; do shift; done; shift\n'
            f'printf "%s\\n" "$*" >> {shlex.quote(self.log)}\n'
            f"BUILD_WORKSPACE_DIRECTORY={shlex.quote(self.root)} "
            f"PYTHONPATH={shlex.quote(os.pathsep.join(sys.path[1:]))} "
            f'exec {shlex.quote(sys.executable)} -c {shlex.quote(check)} "$@"\n',
            0o755)
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            '#!/bin/sh\necho "resolver stub: stop here" >&2\nexit 1\n',
            0o755)
        self.env = dict(os.environ, **self._runfiles.EnvVars())
        self.env["PATH"] = bin_dir + os.pathsep + os.environ.get("PATH", "")
        self.env.pop("BUILD_WORKSPACE_DIRECTORY", None)

    def run_sim(self, config, cwd):
        """sim.sh --config `config` from `cwd`: (exit status, output, the
        configs the manifest check was run on)."""
        if os.path.exists(self.log):
            os.remove(self.log)
        proc = subprocess.run(
            ["bash", self.sim, "--config", config, "--sim-gateway",
             "--no-studio"],
            cwd=cwd, env=dict(self.env, PWD=cwd), capture_output=True,
            text=True, timeout=60)
        checked = []
        if os.path.exists(self.log):
            with open(self.log, encoding="utf-8") as f:
                checked = f.read().split()
        return proc.returncode, proc.stdout + proc.stderr, checked

    # REQ-DEV-23
    def test_a_config_path_names_one_file_in_the_repository(self):
        # Relative to the root or the caller's directory, or absolute: the
        # manifest check and the launcher get it relative to the root, and
        # a current manifest passes the check (the resolver stub then stops
        # sim.sh).
        for config, cwd in ((CONFIG, self.root),
                            (CONFIG, os.path.join(self.root, "sub")),
                            ("robot.toml", os.path.join(self.root, "bot")),
                            (os.path.join(self.root, CONFIG), self.tmp)):
            with self.subTest(config=config, cwd=cwd):
                status, output, checked = self.run_sim(config, cwd)
                self.assertEqual(checked, ["--check", CONFIG], output)
                self.assertNotEqual(status, 0)
                self.assertIn(f"cannot resolve {CONFIG}", output)

    # REQ-DEV-23
    def test_a_resolver_failure_is_one_line_with_its_reason(self):
        # The resolver stub fails as the real one does on a config it cannot
        # load: its reason on stderr, nothing on stdout, exit 1. sim.sh
        # reports that reason in its own one line and exits 1, without
        # handing the empty output to its JSON reader.
        status, output, _ = self.run_sim(CONFIG, self.root)
        self.assertEqual(status, 1, output)
        lines = [line for line in output.splitlines()
                 if line.strip() and not line.startswith("\x1b[1;36m==>")]
        self.assertEqual(len(lines), 1, output)
        self.assertIn(f"cannot resolve {CONFIG}", lines[0])
        self.assertIn("resolver stub: stop here", lines[0])

    # REQ-DEV-23
    def test_refuses_an_ambiguous_missing_or_outside_config_unchecked(self):
        # A relative path naming a different file under the root and under
        # the caller's directory would silently run the root's.
        shadow = os.path.join(self.root, "sub", CONFIG)
        put(shadow, ROBOT)
        outside = os.path.join(self.tmp, "elsewhere", "robot.toml")
        put(outside, ROBOT)
        for config, cwd, named in (
                (CONFIG, os.path.join(self.root, "sub"), shadow),
                ("missing.toml", self.root, "missing.toml"),
                (outside, self.root, outside)):
            with self.subTest(config=config):
                status, output, checked = self.run_sim(config, cwd)
                self.assertEqual(status, 1, output)
                self.assertEqual(checked, [], output)
                self.assertIn(named, output)

    # REQ-DEV-23
    def test_refuses_a_stale_manifest_naming_the_regenerate_command(self):
        with open(os.path.join(self.root, CONFIG), "a",
                  encoding="utf-8") as f:
            f.write('\n[subsystems.wrist]\nnode = "//bot:wrist"\n')
        status, output, checked = self.run_sim(CONFIG, self.root)
        self.assertEqual(status, 1, output)
        self.assertEqual(checked, ["--check", CONFIG])
        self.assertIn(f"bazel run //tools:sync_manifest -- {CONFIG}", output)
        self.assertNotIn("cannot resolve", output)

    # REQ-DEV-05
    def test_failed_build_prints_last_60_lines_and_build_log_path(self):
        # A failing `bazel build` in sim.sh writes its full output to
        # $TALOS_SIM_LOG_DIR/<session_id>/sim/build.log, prints only the last
        # 60 lines to stderr, and names the log's path in the fatal error.
        resolved = json.dumps({
            "nodes": ["//bot:node"],
            "effective": {
                "robot": {"sim_gateway": "//talOS/simulation:sim_gateway"},
                "studio": {},
            },
        })
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(resolved)}\n",
            0o755)
        bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            'if [ "$1" = build ]; then\n'
            '  i=1\n'
            '  while [ "$i" -le 80 ]; do\n'
            '    printf "build-line-%02d\\n" "$i"\n'
            '    i=$((i + 1))\n'
            '  done\n'
            '  exit 1\n'
            'fi\n'
            '[ "$1" = run ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)

        log_root = os.path.join(self.tmp, "sim_logs")
        env = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root)
        proc = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--sim-gateway",
             "--no-studio"],
            cwd=self.root, env=env, capture_output=True, text=True,
            timeout=60)
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        logs = glob.glob(os.path.join(log_root, "*", "sim", "build.log"))
        self.assertEqual(len(logs), 1,
                         f"expected build.log under {log_root}: {proc.stderr}")
        build_log = logs[0]
        self.assertIn(build_log, proc.stderr)
        self.assertNotIn("build-line-20", proc.stderr)
        self.assertIn("build-line-21", proc.stderr)
        self.assertIn("build-line-80", proc.stderr)
        with open(build_log, encoding="utf-8") as f:
            recorded = f.read().splitlines()
        self.assertEqual(len(recorded), 80)
        self.assertEqual(recorded[0], "build-line-01")
        self.assertEqual(recorded[-1], "build-line-80")

    # REQ-OPS-20
    def test_wpilib_mode_checks_session_lock_before_build_and_before_gradlew(self):
        # TOOLS-7: in WPILib mode, sim.sh runs `launcher --check-session-lock`
        # before `bazel build` if the launcher binary already exists, and again
        # right before starting `./gradlew simulateNative` after the build.
        resolved = json.dumps({
            "nodes": ["//bot:node"],
            "effective": {"robot": {}, "studio": {}},
        })
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(resolved)}\n",
            0o755)
        launcher_path = os.path.join(self.root, "bazel-bin", "talOS",
                                     "launcher", "launcher")
        build_marker = os.path.join(self.tmp, "bazel_build_called")
        gradle_marker = os.path.join(self.tmp, "gradlew_called")
        put(os.path.join(self.root, "controller_processor", "rio", "gradlew"),
            f"#!/bin/sh\necho called > {shlex.quote(gradle_marker)}\nexit 0\n",
            0o755)
        bin_dir = os.path.join(self.tmp, "bin")

        # Case 1: launcher already exists and reports a held lock -> fails
        # before `bazel build` and before `gradlew`.
        put(launcher_path,
            '#!/bin/sh\n'
            'if [ "$1" = "--check-session-lock" ]; then\n'
            '  echo "launcher: another launcher (pid 4242) is running a session" >&2\n'
            '  exit 1\n'
            'fi\n'
            'exit 0\n',
            0o755)
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            f'if [ "$1" = build ]; then echo build > {shlex.quote(build_marker)}; exit 0; fi\n'
            '[ "$1" = run ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)
        log_root = os.path.join(self.tmp, "sim_logs_lock1")
        env = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root)
        proc = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--headless", "--no-studio"],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=60)
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        self.assertIn("pid 4242", proc.stderr)
        self.assertFalse(os.path.exists(build_marker),
                         "bazel build should not run when prebuilt launcher sees held lock")
        self.assertFalse(os.path.exists(gradle_marker),
                         "gradlew should not start when session lock is held")

        # Case 2: launcher did not exist before build; build creates it and
        # post-build lock check fails before `gradlew simulateNative`.
        os.remove(launcher_path)
        launcher_script = (
            "#!/bin/sh\n"
            'if [ "$1" = "--check-session-lock" ]; then\n'
            '  echo "launcher: another launcher (pid 5151) is running a session" >&2\n'
            '  exit 1\n'
            'fi\n'
            'exit 0\n'
        )
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            'if [ "$1" = build ]; then\n'
            f'  echo build > {shlex.quote(build_marker)}\n'
            f'  mkdir -p {shlex.quote(os.path.dirname(launcher_path))}\n'
            f'  printf %s {shlex.quote(launcher_script)} > {shlex.quote(launcher_path)}\n'
            f'  chmod 755 {shlex.quote(launcher_path)}\n'
            '  exit 0\n'
            'fi\n'
            '[ "$1" = run ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)
        log_root2 = os.path.join(self.tmp, "sim_logs_lock2")
        env2 = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root2)
        proc2 = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--headless", "--no-studio"],
            cwd=self.root, env=env2, capture_output=True, text=True, timeout=60)
        self.assertEqual(proc2.returncode, 1, proc2.stdout + proc2.stderr)
        self.assertTrue(os.path.exists(build_marker),
                        "bazel build should run when launcher is not yet built")
        self.assertIn("pid 5151", proc2.stderr)
        self.assertFalse(os.path.exists(gradle_marker),
                         "gradlew must not start when post-build session lock check fails")

    # REQ-OPS-20
    def test_wpilib_mode_stale_prebuilt_launcher_falls_through_to_build(self):
        # A stale pre-build launcher that rejects --check-session-lock (or
        # fails for a non-holder reason) must fall through to `bazel build`
        # instead of aborting before rebuilding; the post-build check remains
        # strict and reports non-holder failures accurately.
        resolved = json.dumps({
            "nodes": ["//bot:node"],
            "effective": {"robot": {}, "studio": {}},
        })
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(resolved)}\n",
            0o755)
        launcher_path = os.path.join(self.root, "bazel-bin", "talOS",
                                     "launcher", "launcher")
        build_marker = os.path.join(self.tmp, "bazel_build_stale")
        gradle_marker = os.path.join(self.tmp, "gradlew_stale")
        put(os.path.join(self.root, "controller_processor", "rio", "gradlew"),
            f"#!/bin/sh\necho called > {shlex.quote(gradle_marker)}\nexit 0\n",
            0o755)
        bin_dir = os.path.join(self.tmp, "bin")

        # Stale pre-build launcher rejects --check-session-lock as unknown.
        put(launcher_path,
            '#!/bin/sh\n'
            'echo "launcher error: unknown argument: --check-session-lock" >&2\n'
            'exit 1\n',
            0o755)
        # Build replaces it with a current launcher that fails the post-build
        # check with a non-holder error (e.g. unwritable lock file).
        rebuilt_launcher = (
            "#!/bin/sh\n"
            'if [ "$1" = "--check-session-lock" ]; then\n'
            '  echo "launcher: cannot open the session lock /tmp/lock: Permission denied" >&2\n'
            '  exit 1\n'
            'fi\n'
            'exit 0\n'
        )
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            'if [ "$1" = build ]; then\n'
            f'  echo build > {shlex.quote(build_marker)}\n'
            f'  printf %s {shlex.quote(rebuilt_launcher)} > {shlex.quote(launcher_path)}\n'
            f'  chmod 755 {shlex.quote(launcher_path)}\n'
            '  exit 0\n'
            'fi\n'
            '[ "$1" = run ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)

        log_root = os.path.join(self.tmp, "sim_logs_stale")
        env = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root)
        proc = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--headless", "--no-studio"],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=60)
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        self.assertTrue(os.path.exists(build_marker),
                        "stale pre-build launcher must fall through to bazel build")
        self.assertFalse(os.path.exists(gradle_marker),
                         "post-build check must remain strict before gradlew")
        self.assertIn("Permission denied", proc.stderr)
        self.assertIn("session lock check failed", proc.stderr)
        self.assertNotIn("another session holds the session lock", proc.stderr)

    # REQ-DEV-05
    def test_missing_value_for_duration_s_or_http_port_exits_2(self):
        for flag in ("--duration-s", "--http-port"):
            with self.subTest(flag=flag):
                proc = subprocess.run(
                    ["bash", self.sim, flag],
                    cwd=self.root, env=dict(self.env, PWD=self.root),
                    capture_output=True, text=True, timeout=15)
                self.assertEqual(proc.returncode, 2, proc.stdout + proc.stderr)
                self.assertIn(f"{flag} needs an argument", proc.stderr)
                self.assertNotIn("unbound variable", proc.stderr)

    # REQ-DEV-05, REQ-DEV-23
    def test_defaults_http_port_from_resolved_studio_config(self):
        sock_cfg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_cfg.bind(("127.0.0.1", 0))
        sock_cfg.listen(4)
        self.addCleanup(sock_cfg.close)
        port_cfg = sock_cfg.getsockname()[1]

        sock_cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_cli.bind(("127.0.0.1", 0))
        sock_cli.listen(4)
        self.addCleanup(sock_cli.close)
        port_cli = sock_cli.getsockname()[1]

        resolved = json.dumps({
            "nodes": ["//bot:node"],
            "effective": {
                "robot": {"sim_gateway": "//talOS/simulation:sim_gateway"},
                "studio": {"http_port": port_cfg},
            },
        })
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(resolved)}\n",
            0o755)
        bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(bin_dir, "bazel"),
            '#!/bin/sh\n[ "$1" = run ] || [ "$1" = build ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)
        launcher_args = os.path.join(self.tmp, "launcher_args")
        put(os.path.join(self.root, "bazel-bin", "talOS", "launcher",
                         "launcher"),
            "#!/bin/sh\n"
            f'printf "%s\\n" "$*" > {shlex.quote(launcher_args)}\n'
            'out_dir=""\n'
            'session_id=""\n'
            'while [ $# -gt 0 ]; do\n'
            '  case "$1" in\n'
            '    --output-dir) out_dir="$2"; shift ;;\n'
            '    --session-id) session_id="$2"; shift ;;\n'
            '  esac\n'
            '  shift\n'
            'done\n'
            'mkdir -p "$out_dir/$session_id"\n'
            'printf "{}\\n" > "$out_dir/$session_id/graph.json"\n'
            'exit 0\n',
            0o755)

        log_root = os.path.join(self.tmp, "sim_logs_studio_port")
        env = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root)

        # Without --http-port: sim.sh must poll the config's [studio] http_port
        # (and not override it on the launcher command line).
        proc = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--sim-gateway",
             "--no-open"],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=8)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn(f"Studio on http://localhost:{port_cfg}", proc.stdout)
        with open(launcher_args, encoding="utf-8") as f:
            args_default = f.read()
        self.assertNotIn("--studio-http-port", args_default)

        # With explicit --http-port: sim.sh polls and forwards that port.
        proc2 = subprocess.run(
            ["bash", self.sim, "--config", CONFIG, "--sim-gateway",
             "--no-open", "--http-port", str(port_cli)],
            cwd=self.root, env=env, capture_output=True, text=True, timeout=8)
        self.assertEqual(proc2.returncode, 0, proc2.stdout + proc2.stderr)
        self.assertIn(f"Studio on http://localhost:{port_cli}", proc2.stdout)
        with open(launcher_args, encoding="utf-8") as f:
            args_cli = f.read()
        self.assertIn(f"--studio-http-port {port_cli}", args_cli)

    def _assert_signal_during_wpilib_wait_exits(self, sig, expected_rc):
        resolved = json.dumps({
            "nodes": ["//bot:node"],
            "effective": {"robot": {}, "studio": {}},
        })
        put(os.path.join(self.root, "bazel-bin", "talOS", "configuration",
                         "resolve"),
            f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(resolved)}\n",
            0o755)
        bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(bin_dir, "bazel"),
            '#!/bin/sh\n[ "$1" = run ] || [ "$1" = build ] && exit 0\n'
            'echo "bazel stub: $*" >&2; exit 97\n',
            0o755)
        launcher_marker = os.path.join(self.tmp, f"launcher_started_{sig.name}")
        put(os.path.join(self.root, "bazel-bin", "talOS", "launcher",
                         "launcher"),
            "#!/bin/sh\n"
            '[ "$1" = "--check-session-lock" ] && exit 0\n'
            f'echo started > {shlex.quote(launcher_marker)}\n'
            'exit 0\n',
            0o755)
        put(os.path.join(self.root, "controller_processor", "rio", "gradlew"),
            "#!/bin/sh\nexec sleep 60\n",
            0o755)

        log_root = os.path.join(self.tmp, f"sim_logs_sig_{sig.name}")
        env = dict(self.env, PWD=self.root, TALOS_SIM_LOG_DIR=log_root)
        proc = subprocess.Popen(
            ["bash", self.sim, "--config", CONFIG, "--headless", "--no-studio"],
            cwd=self.root, env=env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        seen = []
        try:
            for line in proc.stdout:
                seen.append(line)
                if "waiting for the simulation to come up" in line:
                    break
            else:
                self.fail(f"sim.sh exited before WPILib wait: {''.join(seen)}")
            proc.send_signal(sig)
            rest, _ = proc.communicate(timeout=8)
            out = "".join(seen) + (rest or "")
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            self.fail(
                f"sim.sh did not exit after {sig.name} during WPILib wait loop")
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        self.assertEqual(proc.returncode, expected_rc, out)
        self.assertIn("sim.sh: shutting down", out)
        self.assertFalse(os.path.exists(launcher_marker),
                         "launcher must not start after signal during WPILib wait")

    # REQ-DEV-05
    def test_sigterm_during_wpilib_wait_exits_143(self):
        self._assert_signal_during_wpilib_wait_exits(signal.SIGTERM, 143)

    # REQ-DEV-05
    def test_sigint_during_wpilib_wait_exits_130(self):
        self._assert_signal_during_wpilib_wait_exits(signal.SIGINT, 130)


if __name__ == "__main__":
    unittest.main(testLoader=sharded_test_loader.bazel_test_loader())
