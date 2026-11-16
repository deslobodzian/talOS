"""Installed bundles describe their graph and keep their config identity.

Each bundle is installed the way the robot gets it: tools/deploy.sh install
into a fresh prefix, which copies it as real files (Bazel's sandbox hands
the test a tree of links into the build tree, and install copies a link
that leaves the bundle as the file it names) and runs every member once
without a warning. Then
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
which its bundle holds once under python/<repository>/. The boot service
stops the session through the launcher alone. Install's refusal of a graph
its launcher refuses is install_check_test's; install's warm-up is
warm_up_test's."""

import filecmp
import glob
import json
import os
import plistlib
import shlex
import subprocess
import sys
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

from bench.main_processor.deploy_test_support import deploy_script

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


if __name__ == "__main__":
    unittest.main()
