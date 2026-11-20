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
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles

from tools import sync_manifest

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
    def setUp(self):
        r = runfiles.Create()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)
        self.root = os.path.join(self.tmp, "ws")
        self.sim = os.path.join(self.root, "sim.sh")
        os.makedirs(self.root)
        shutil.copy(r.Rlocation("_main/sim.sh"), self.sim)
        put(os.path.join(self.root, "bot", "BUILD"), "")
        put(os.path.join(self.root, CONFIG), ROBOT)
        sync_manifest.write(self.root, CONFIG)
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
        self.env = dict(os.environ, **r.EnvVars())
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
        check = ("import sys; from tools import sync_manifest; "
                 "sys.exit(sync_manifest.main(sys.argv[1:]))")
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
            '[ "$1" = run ] || { echo "bazel stub: $*" >&2; exit 97; }\n'
            'while [ "$1" != -- ]; do shift; done; shift\n'
            f'printf "%s\\n" "$*" >> {shlex.quote(self.log)}\n'
            f"BUILD_WORKSPACE_DIRECTORY={shlex.quote(self.root)} "
            f"PYTHONPATH={shlex.quote(os.pathsep.join(sys.path[1:]))} "
            f'exec {shlex.quote(sys.executable)} -c {shlex.quote(check)} "$@"\n',
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


if __name__ == "__main__":
    unittest.main()
