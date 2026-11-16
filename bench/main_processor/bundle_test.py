"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it: its files copied as
real files (Bazel's sandbox hands the test a tree of links into the build
tree, which deploy.sh would copy as links), then tools/deploy.sh install
into a fresh prefix, which runs every member once without a warning. Then
--describe runs every node binary from there, so a C++ node that cannot
load a shared library it links, or a Python node that cannot import its
wheels or start its interpreter (as when Bazel restored the bundle from its
cache with links copied), fails here. A Python node's native wheels (numpy's,
gtsam's and robotpy-wpimath's extension modules and the libraries the
wheels bundle or depend on) must also load from the installed files and
nowhere else. The subjects are the bench robot's
bundles: the deployable one, which carries Studio and which link and stop
qualification install; the Python bench robot's, with two Python nodes;
and a test config that reaches its files through `include`, a
parent-directory `defaults`, a custom `path` and an entry `node`, and adds
a Python node importing numpy, gtsam and robotpy-wpimath (the bench robot's
own nodes import no native wheel, and no other talOS target loads the gtsam
and robotpy-wpimath talOS pins for robot repositories). A bundle that carries Studio is described by the command the boot service
runs (deploy.sh enable-boot's systemd ExecStart and launchd
ProgramArguments), which must run its bridge from the install, found through
the manifest's [studio] table, beside the index page of the webroot that
table names; one that does not is asked for Studio with
--start-studio-bridge and must describe without a bridge rather than fail on
its absence. Every Python node's venv runs its runtime's one interpreter,
which its bundle holds once under python/<repository>/. Install refuses a
bundle whose graph its own launcher refuses. Install's warm-up, and the
order of its graph check, are checked on a bundle of stubs that record each
run.

On macOS each newly written executable's first exec costs about 100 ms, and
seconds under parallel load (TODO.md MAC-2), and every installed stub is a
new file. So a stub runs under the default 60 s bound, except in the one
case that waits out a short bound: there every member has already run once
from its installed path, so only the stub that never exits can reach it."""

import filecmp
import glob
import json
import os
import plistlib
import select
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

BUNDLES = (
    "bench/main_processor/bench_bundle",
    "bench/main_processor/bench_py_bundle",
    "bench/main_processor/relocation_bundle",
)

# Python nodes whose bundle holds native wheels, and the modules of them
# that must load from the installed files: each package, and the extension
# module that links the libraries the wheel bundles (gtsam.libs/) or its
# robotpy-native wheel holds (native/wpimath/, native/wpiutil/). On Linux
# each module's first name must also name a mapped file.
NATIVE_IMPORTS = {
    "native_wheel": ("numpy", "numpy._core._multiarray_umath",
                     "gtsam", "gtsam.gtsam",
                     "wpimath.geometry", "wpimath.geometry._geometry",
                     "wpiutil._wpiutil"),
}

# What a Python node's interpreter must not inherit from the test: the
# installed venv alone decides what it imports.
CLEAN_ENV = {k: v for k, v in os.environ.items()
             if k not in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV",
                          "RUNFILES_DIR", "RUNFILES_MANIFEST_FILE",
                          "PYTHON_RUNFILES")}

# Run by a node's venv interpreter: adds the site-packages directory it is
# given, imports each module named after it and prints, as JSON, the real
# path of each module's file and (where /proc exists) of every mapped file
# whose path names one of their packages.
IMPORT_PROBE = """
import importlib, json, os, site, sys
site.addsitedir(sys.argv[1])
modules = sys.argv[2:]
files = [os.path.realpath(importlib.import_module(m).__file__)
         for m in modules]
mapped = set()
if os.path.exists("/proc/self/maps"):
    roots = {m.split(".")[0] for m in modules}
    with open("/proc/self/maps") as maps:
        for line in maps:
            fields = line.split(maxsplit=5)
            path = fields[5].strip() if len(fields) == 6 else ""
            if path.startswith("/") and any(r in path for r in roots):
                mapped.add(os.path.realpath(path))
print(json.dumps({"files": files, "mapped": sorted(mapped)}))
"""


def deploy_script():
    return runfiles.Create().Rlocation("_main/tools/deploy.sh")


def enable_boot(prefix, system):
    """What `deploy.sh enable-boot --dry-run` prints on `system` ("Linux"
    or "Darwin"), with uname and FileVault faked."""
    with tempfile.TemporaryDirectory() as fake:
        for name, body in (("uname", f"echo {system}"),
                           ("fdesetup", "echo 'FileVault is Off.'")):
            with open(os.path.join(fake, name), "w", encoding="utf-8") as f:
                f.write(f"#!/bin/sh\n{body}\n")
            os.chmod(os.path.join(fake, name), 0o755)
        env = dict(os.environ, PATH=fake + os.pathsep + os.environ["PATH"])
        return subprocess.run(
            ["bash", deploy_script(), "enable-boot", "--dry-run",
             "--prefix", prefix],
            check=True, capture_output=True, text=True, env=env,
            timeout=60).stdout


def boot_command(prefix, system):
    """The argv enable-boot writes into the boot service on `system`
    ("Linux": the systemd unit's ExecStart; "Darwin": the LaunchDaemon's
    ProgramArguments)."""
    out = enable_boot(prefix, system)
    if system == "Darwin":
        plist = plistlib.loads(
            out[out.index("<?xml"):out.index("</plist>") + 8].encode())
        return plist["ProgramArguments"]
    line = next(line for line in out.splitlines()
                if line.startswith("ExecStart="))
    return shlex.split(line.removeprefix("ExecStart="))


def as_shipped(src, tmp):
    """`src` as real files. Bazel's Linux sandbox links each file of a
    bundle into the test's runfiles, and deploy.sh install copies links as
    links (cp -RP), so installing `src` itself would run the build tree's
    files; a talos_bundle holds no links of its own (REQ-OPS-48)."""
    if not os.path.islink(os.path.join(src, "run.sh")):
        return src
    copy = os.path.join(tmp, os.path.basename(src))
    shutil.copytree(src, copy, symlinks=False)
    return copy


def python_venvs(bundle, node):
    """The venv interpreters under a Python node's runfiles in `bundle`."""
    return [os.path.join(parent, "python3")
            for parent, _, files in os.walk(os.path.join(
                bundle, node["binary"] + ".runfiles"))
            if parent.endswith(".venv/bin") and "python3" in files]


def describe(bundle, prefix, command, *args):
    """Installs `bundle` under `prefix` with deploy.sh and runs `command`
    (an argv starting with the installed run.sh) with --describe-only and
    `args`. Returns the process, the session's graph, the installed
    bundle's directory and the install's stderr."""
    install = subprocess.run(["bash", deploy_script(), "install", bundle,
                              "--prefix", prefix], check=True,
                             capture_output=True, text=True, timeout=120)
    where = os.path.join(prefix, "current")
    log_dir = os.path.join(prefix, "logs")
    env = dict(os.environ, TALOS_LOG_DIR=log_dir)
    proc = subprocess.run([*command, "--describe-only", *args], cwd=where,
                          env=env, capture_output=True, text=True, timeout=120)
    sessions = sorted(
        (os.path.join(log_dir, name) for name in os.listdir(log_dir)
         if os.path.isdir(os.path.join(log_dir, name))),
        key=os.path.getmtime, reverse=True)
    graph = os.path.join(sessions[0], "graph.json") if sessions else ""
    if not os.path.isfile(graph):
        return proc, None, where, install.stderr
    with open(graph, encoding="utf-8") as f:
        return proc, json.load(f), where, install.stderr


class BundleTest(unittest.TestCase):
    def test_installed_bundles_describe_with_a_stable_identity(self):
        r = runfiles.Create()
        for name in BUNDLES:
            # deploy.sh installs read-only, so cleanup cannot remove it all;
            # Bazel clears TEST_TMPDIR.
            with self.subTest(name), tempfile.TemporaryDirectory(
                    ignore_cleanup_errors=True) as tmp:
                src = r.Rlocation(f"_main/{name}")
                self.assertTrue(src and os.path.isdir(src), src)
                src = as_shipped(src, tmp)
                with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                    manifest = tomllib.load(f)
                studio = manifest.get("studio")
                prefix = os.path.join(tmp, "prefix")
                if studio:
                    # REQ-OPS-34: a booted robot serves Studio. Both boot
                    # services run the same command, with no argument that
                    # asks for or declines the bridge.
                    command = boot_command(prefix, "Linux")
                    self.assertEqual(command, boot_command(prefix, "Darwin"))
                    args = ()
                else:
                    # Asked for Studio, a bundle without it still describes.
                    command = [os.path.join(prefix, "current", "run.sh")]
                    args = ("--start-studio-bridge",)
                proc, graph, where, installed = describe(
                    src, prefix, command, *args)
                out = proc.stdout + "\n" + proc.stderr
                # Install ran every member the way the probe will, so each
                # one's first exec is paid before the first launch.
                self.assertNotIn("warning: warm-up", installed)
                self.assertEqual(proc.returncode, 0, out)
                self.assertIsNotNone(graph, out)
                # The bundle holds exactly the nodes its config launches, so
                # the graph lists each of them, and its Studio bridge.
                self.assertEqual(len(graph["nodes"]),
                                 len(manifest["nodes"]) + bool(studio), out)
                if studio:
                    self.assertIn("studio_bridge",
                                  [node.get("name") for node in graph["nodes"]],
                                  out)
                    self.assertTrue(os.path.isfile(os.path.join(
                        where, studio["webroot"], "index.html")), studio)
                for node in graph["nodes"]:
                    self.assertTrue(node.get("described"),
                                    f"{node.get('name')}: {node.get('describe_error')}\n{out}")
                # The copy keeps the identity the bundle was built with.
                self.assertTrue(manifest["config_sha256"])
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"])
                # REQ-OPS-48
                self.assert_native_wheels_load_from(
                    os.path.realpath(where), manifest)

    def assert_native_wheels_load_from(self, installed, manifest):
        """Each NATIVE_IMPORTS node's modules, imported by its installed venv
        with the import path its bootstrap gives it (rules_python's stage 2:
        RUNFILES_DIR, then the venv's site-packages, whose bazel.pth adds the
        node's imports), load from files under `installed`, as does every
        library of the wheel they map."""
        for node in manifest["nodes"]:
            modules = NATIVE_IMPORTS.get(node["name"])
            if not modules:
                continue
            venvs = python_venvs(installed, node)
            self.assertEqual(len(venvs), 1, (node["name"], venvs))
            sites = glob.glob(os.path.join(
                os.path.dirname(os.path.dirname(venvs[0])),
                "lib", "python3.*", "site-packages"))
            self.assertEqual(len(sites), 1, (node["name"], sites))
            proc = subprocess.run(
                [venvs[0], "-c", IMPORT_PROBE, sites[0], *modules],
                capture_output=True, text=True, timeout=120,
                env=dict(CLEAN_ENV, RUNFILES_DIR=os.path.join(
                    installed, node["binary"] + ".runfiles")),
                cwd=os.path.dirname(installed))
            self.assertEqual(proc.returncode, 0,
                             f"{node['name']}: {proc.stderr}")
            loaded = json.loads(proc.stdout)
            if sys.platform == "linux":
                for root in {module.split(".")[0] for module in modules}:
                    self.assertTrue(
                        [path for path in loaded["mapped"] if root in path],
                        (node["name"], root, loaded))
            for path in loaded["files"] + loaded["mapped"]:
                self.assertTrue(path.startswith(installed + os.sep),
                                f"{node['name']}: {path} is not under "
                                f"{installed}")

    # REQ-OPS-48
    def test_every_python_node_runs_its_runtimes_one_interpreter(self):
        # The interpreter is most of a bundle's bytes, which a deploy copies
        # to the robot: one copy per Python node, and per name rules_python
        # gives it, made a 14-member robot bundle 749 MB. bench_py_bundle
        # runs two Python nodes, so a second copy fails here.
        r = runfiles.Create()
        env = {k: v for k, v in os.environ.items()
               if k not in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV")}
        for name in BUNDLES:
            with self.subTest(name):
                src = r.Rlocation(f"_main/{name}")
                with open(os.path.join(src, "bundle_manifest.toml"), "rb") as f:
                    manifest = tomllib.load(f)
                ran = set()
                for node in manifest["nodes"]:
                    if node["language"] != "python":
                        continue
                    venvs = python_venvs(src, node)
                    self.assertTrue(venvs, node["name"])
                    for venv in venvs:
                        proc = subprocess.run(
                            [venv, "-I", "-S", "-c",
                             "import os, sys; "
                             "print(os.path.normpath(sys.executable))"],
                            capture_output=True, text=True, env=env,
                            timeout=60)
                        self.assertEqual(proc.returncode, 0, proc.stderr)
                        ran.add(proc.stdout.strip())
                # Each runtime repository the nodes run is held once, under
                # python/<repo>/, and all its nodes run that one file.
                by_repo = {}
                for interpreter in ran:
                    rel = os.path.relpath(interpreter, src)
                    parts = rel.split(os.sep)
                    self.assertEqual(parts[0], "python", rel)
                    by_repo.setdefault(parts[1], set()).add(interpreter)
                for repo, interpreters in by_repo.items():
                    self.assertEqual(len(interpreters), 1, (repo, interpreters))
                    interpreter = interpreters.pop()
                    size = os.path.getsize(interpreter)
                    copies = [
                        os.path.relpath(path, src)
                        for parent, _, files in os.walk(src)
                        for path in (os.path.join(parent, f) for f in files)
                        if os.path.getsize(path) == size
                        and filecmp.cmp(path, interpreter, shallow=False)]
                    self.assertEqual(len(copies), 1, (repo, copies))


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


def launcher_stop_timeout_s():
    """The launcher's orderly stop: SIGINT to every member, SIGKILL to any
    still alive kStopTimeout (talOS/launcher/launcher.h) later, then the
    final manifest. Printed by a binary built from that constant."""
    path = runfiles.Create().Rlocation(
        "_main/bench/main_processor/bundle_test_stop_timeout")
    return int(subprocess.run([path], check=True, capture_output=True,
                              text=True, timeout=60).stdout)


class BootServiceTest(unittest.TestCase):
    # REQ-OPS-06
    def test_systemd_stops_through_the_launcher_alone(self):
        # Under systemd's default KillMode=control-group, `systemctl stop`
        # SIGTERMs every member with the launcher: a member that does not
        # declare holds_stop (hardware_node, the simulated gateway, the
        # Studio bridge; a framework node holds it) dies of a signal the
        # launcher did not send, the session exits 255 and the unit is recorded failed.
        # Only the launcher may get the stop; systemd's
        # SIGKILL must come after the launcher's own escalation.
        with tempfile.TemporaryDirectory() as tmp:
            out = enable_boot(os.path.join(tmp, "prefix"), "Linux")
        service = out[out.index("[Service]"):out.index("[Install]")]
        unit = dict(line.split("=", 1) for line in service.splitlines()
                    if "=" in line)
        self.assertEqual(unit.get("KillMode"), "mixed", service)
        stop_s = unit.get("TimeoutStopSec", "")
        self.assertRegex(stop_s, r"^\d+s?$", service)
        self.assertGreater(int(stop_s.rstrip("s")), launcher_stop_timeout_s(),
                           service)


# Each stub records, under its installed path, "start" with its pid, what
# $STUB_PREFIX/current selected then ("-" for nothing) and its argv before
# acting, and "end" after, so the log orders every warm-up run.
STUB = """#!/usr/bin/env bash
me="$(cd "$(dirname "$0")" && pwd -P)/$(basename "$0")"
current="$(readlink "$STUB_PREFIX/current" || echo -)"
printf 'start %s %s %s %s\\n' "$me" "$$" "$current" "$*" >> "$STUB_LOG"
case "$(basename "$0")" in
  fails) printf 'end %s\\n' "$me" >> "$STUB_LOG"; exit 3 ;;
  hangs)
    # STUB_PREWARM: run once to pay a first exec, then exit. STUB_FIFO: say
    # it is running.
    [ -z "${STUB_PREWARM:-}" ] || exit 0
    [ -z "${STUB_FIFO:-}" ] || echo up > "$STUB_FIFO"
    exec tail -f /dev/null ;;
esac
printf 'end %s\\n' "$me" >> "$STUB_LOG"
"""

def stub_bundle(root, nodes):
    """A bundle laid out as talos_bundle writes one, every executable a
    stub: `nodes` (`fails` exits 3, `hangs` never exits, any other exits
    0), a Studio bridge, the launcher and run.sh."""
    for sub in ("bin", "config", "studio"):
        os.makedirs(os.path.join(root, sub))
    with open(os.path.join(root, "config", "robot.toml"), "w",
              encoding="utf-8") as f:
        f.write("[robot]\n")
    for rel in [f"bin/{n}" for n in nodes] + [
            "studio/studio_bridge", "launcher", "run.sh"]:
        with open(os.path.join(root, rel), "w", encoding="utf-8") as f:
            f.write(STUB)
        os.chmod(os.path.join(root, rel), 0o755)
    lines = ['bundle_version = 1', 'config_path = "config/robot.toml"',
             'config_sha256 = "0"', '', '[studio]',
             'bridge = "studio/studio_bridge"', 'bridge_sha256 = "0"',
             'targets = ["//studio/bridge:studio_bridge"]',
             'webroot = "studio/webroot"']
    for n in nodes:
        lines += ['', '[[nodes]]', f'name = "{n}"',
                  f'targets = ["//x:{n}"]', f'binary = "bin/{n}"',
                  'binary_sha256 = "0"', 'language = "cpp"']
    with open(os.path.join(root, "bundle_manifest.toml"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


class WarmUpTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        self.prefix = os.path.join(self.tmp, "prefix")
        self.log = os.path.join(self.tmp, "stub.log")
        # The default bound: no fresh stub is killed for its first exec.
        self.env = dict(os.environ, STUB_LOG=self.log, STUB_PREFIX=self.prefix)
        self.env.pop("TALOS_DEPLOY_WARM_UP_TIMEOUT_S", None)

    def deploy(self, *args):
        proc = subprocess.run(
            ["bash", deploy_script(), *args, "--prefix", self.prefix],
            capture_output=True, text=True, env=self.env, timeout=60)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        return proc

    def install(self, name, nodes, *args):
        src = os.path.join(self.tmp, name)
        stub_bundle(src, nodes)
        return self.deploy("install", src, *args)

    def records(self, check=False):
        """The warm-up's records, or with `check` install's graph check's
        (the installed run.sh, which the warm-up never runs)."""
        if not os.path.exists(self.log):
            return []
        with open(self.log, encoding="utf-8") as f:
            return [r for r in (line.split(" ", 4)
                                for line in f.read().splitlines())
                    if (os.path.basename(r[1]) == "run.sh") == check]

    def assert_checked_after_warm_up(self, bundle):
        """The installed run.sh ran once, with --describe-only, after every
        warm-up run and before `current` selected anything."""
        log = self.records(check=True)
        self.assertEqual([r[0] for r in log], ["start", "end"], log)
        self.assertEqual(log[0][1], os.path.join(bundle, "run.sh"), log)
        self.assertEqual(log[0][3], "-", log)
        self.assertTrue(log[0][4].startswith("--describe-only --output-dir "),
                        log)
        with open(self.log, encoding="utf-8") as f:
            last = f.read().splitlines()[-1]
        self.assertEqual(last, f"end {os.path.join(bundle, 'run.sh')}")

    def assert_warmed_in_turn(self, bundle, nodes, selected, proc):
        """Every member of the installed `bundle` ran exactly once, one at a
        time, while `current` still selected `selected`, and nothing else
        ran; a member that failed or hung is named in a warning."""
        records = self.records()
        out = proc.stdout + proc.stderr
        describe = "--describe --config config/robot.toml"
        # The installed files (the macOS charge is per file); run.sh is the
        # graph check's, which records() leaves out.
        started = sorted((r[1], r[4] if len(r) > 4 else "")
                         for r in records if r[0] == "start")
        want = sorted(
            [(os.path.join(bundle, "bin", n), describe) for n in nodes]
            + [(os.path.join(bundle, "studio", "studio_bridge"), describe),
               (os.path.join(bundle, "launcher"), "--help")])
        self.assertEqual(started, want, out)
        hung_pids = []
        for i, r in enumerate(records):
            if r[0] != "start":
                continue
            self.assertEqual(r[3], selected, records)
            if os.path.basename(r[1]) == "hangs":
                # Killed before the next starts, its pid gone by the return.
                hung_pids.append(int(r[2]))
                self.assertNotEqual(records[i + 1:i + 2], [["end", r[1]]])
                continue
            self.assertEqual(records[i + 1][:2], ["end", r[1]], records)
        self.assertEqual(len(hung_pids), "hangs" in nodes, records)
        for pid in hung_pids:
            with self.assertRaises(ProcessLookupError, msg="hung member alive"):
                os.kill(pid, 0)
        for name in ("fails", "hangs"):
            self.assertEqual(name in proc.stderr, name in nodes, out)

    # REQ-OPS-47
    def test_install_runs_each_member_once_in_turn_before_selecting(self):
        nodes = ("alpha", "fails", "omega")
        proc = self.install("bundle", nodes)
        bundle = os.path.realpath(os.path.join(self.prefix, "current"))
        self.assert_warmed_in_turn(bundle, nodes, "-", proc)
        self.assert_checked_after_warm_up(bundle)

    # REQ-OPS-47
    def test_no_warm_up_runs_nothing(self):
        self.install("bundle", ("alpha",), "--no-warm-up")
        self.assertEqual(self.records(), [])
        # The graph check is no part of the warm-up.
        self.assert_checked_after_warm_up(
            os.path.realpath(os.path.join(self.prefix, "current")))

    # REQ-OPS-47
    def test_rollback_warms_the_previous_bundle_before_selecting_it(self):
        # `previous` as a deploy.sh that predates the warm-up leaves it.
        nodes = ("alpha", "fails")
        self.install("old", nodes, "--no-warm-up")
        old = os.readlink(os.path.join(self.prefix, "current"))
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        proc = self.deploy("rollback")
        self.assertEqual(os.readlink(os.path.join(self.prefix, "current")),
                         old)
        self.assert_warmed_in_turn(os.path.realpath(old), nodes, new, proc)


    # REQ-OPS-47
    @unittest.skipUnless(sys.platform == "darwin",
                         "only macOS charges a native library's first load")
    def test_install_loads_each_native_library_once_before_selecting(self):
        src = os.path.join(self.tmp, "bundle")
        stub_bundle(src, ("alpha",))
        rel = "bin/alpha.runfiles/site/mark.cpython-311-darwin.so"
        os.makedirs(os.path.dirname(os.path.join(src, rel)))
        shutil.copy(runfiles.Create().Rlocation(
            "_main/bench/main_processor/bundle_test_native_mark.so"),
            os.path.join(src, rel))
        # A library that fails to load (sorted first) stops nothing.
        with open(os.path.join(os.path.dirname(os.path.join(src, rel)),
                               "broken.so"), "wb") as f:
            f.write(b"not a Mach-O file")
        proc = self.deploy("install", src)
        bundle = os.path.realpath(os.path.join(self.prefix, "current"))
        # The installed copy (the charge is per file), loaded once, while
        # nothing was selected yet.
        marks = [(os.path.realpath(r[1]), r[2])
                 for r in self.records() if r[0] == "native"]
        self.assertEqual(marks, [(os.path.join(bundle, rel), "-")],
                         proc.stdout + proc.stderr)

    # REQ-OPS-47
    def test_a_hung_member_is_killed_at_its_bound(self):
        # Every member of `previous` has run once from its installed path,
        # so within the short bound only `hangs` is slow.
        nodes = ("alpha", "hangs", "omega")
        self.install("old", nodes, "--no-warm-up")
        old = os.path.realpath(os.path.join(self.prefix, "current"))
        prewarm = dict(self.env, STUB_PREWARM="1",
                       STUB_LOG=os.path.join(self.tmp, "prewarm.log"))
        for rel in [f"bin/{n}" for n in nodes] + [
                "studio/studio_bridge", "launcher"]:
            subprocess.run([os.path.join(old, rel)], env=prewarm, check=True,
                           timeout=60)
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        self.env["TALOS_DEPLOY_WARM_UP_TIMEOUT_S"] = "3"
        proc = self.deploy("rollback")
        self.assert_warmed_in_turn(old, nodes, new, proc)

    # REQ-OPS-47
    def test_a_stopped_warm_up_kills_its_member_and_selects_nothing(self):
        src = os.path.join(self.tmp, "bundle")
        stub_bundle(src, ("alpha", "hangs", "omega"))
        fifo = os.path.join(self.tmp, "running")
        os.mkfifo(fifo)
        # Read and write: the stub's open never blocks, and no EOF wakes us.
        reader = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
        self.addCleanup(os.close, reader)
        deploy = subprocess.Popen(
            ["bash", deploy_script(), "install", src, "--prefix", self.prefix],
            env=dict(self.env, STUB_FIFO=fifo), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        self.addCleanup(deploy.kill)
        ready, _, _ = select.select([reader], [], [], 60)
        self.assertTrue(ready, "the hung member never started")
        deploy.send_signal(signal.SIGTERM)
        out, err = deploy.communicate(timeout=60)
        self.assertNotEqual(deploy.returncode, 0, out + err)
        self.assertFalse(os.path.lexists(os.path.join(self.prefix, "current")),
                         out + err)
        records = self.records()
        started = [os.path.basename(r[1]) for r in records if r[0] == "start"]
        self.assertEqual(started, ["studio_bridge", "alpha", "hangs"], records)
        with self.assertRaises(ProcessLookupError, msg="hung member alive"):
            os.kill(int(records[-1][2]), 0)

if __name__ == "__main__":
    unittest.main()
