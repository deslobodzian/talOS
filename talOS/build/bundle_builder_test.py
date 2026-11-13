"""Tests bundle_builder.py's check of a bundle's nodes against its config,
its config staging (relative layout and identity), its manifest's binary
digests and stamped revision, its staging of runfiles (the Python
runtime once, in python/) and of C++ nodes' shared libraries in lib/, its check of a bundled Studio
against the config, and its run.sh (including the Studio it starts), and
tools/deploy.sh's bundle selection, pruning and session commands."""

import json
import os
import plistlib
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles


def put(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


HARDWARE_NODE = "//talOS/bridge:hardware_node"
# A Python runtime's repository as a node's runfiles name it.
RUNTIME = "rules_python++python+python_3_11"
# Where a bundle holds it (bundle_builder.RUNTIME_DIR).
RUNTIME_DIR = "python"


def node(tmp, name, target=None):
    """The spec of a C++ node `name`, whose binary is an empty file."""
    binary = os.path.join(tmp, "src", "bin", name)
    put(binary, "")
    return {"name": name, "target": target or f"//{name}:node",
            "language": "cpp", "src_path": binary,
            "runfiles": [], "shared_libs": []}


class BundleBuilderTest(unittest.TestCase):
    def assemble(self, tmp, files, nodes, launches, robot=None, extra=(),
                 studio=None, effective=None):
        """Runs the builder on `files` ({short_path: text}; the first is the
        config) and the node specs `nodes`, for a config whose subsystems
        launch the labels `launches` and whose effective [robot] table is
        `robot` (other effective tables: `effective`), with `extra`
        arguments and the Studio spec `studio`. Returns the finished
        process."""
        src = os.path.join(tmp, "src")
        entries = []
        for short_path, text in files.items():
            put(os.path.join(src, short_path), text)
            entries.append({"src_path": os.path.join(src, short_path),
                            "short_path": short_path})
        launcher = os.path.join(src, "launcher")
        put(launcher, "")
        spec = os.path.join(tmp, "spec.json")
        with open(spec, "w", encoding="utf-8") as f:
            json.dump({"launcher_path": launcher,
                       "config": entries[0], "config_files": entries[1:],
                       "nodes": list(nodes),
                       "default_hardware_node": HARDWARE_NODE,
                       "studio": studio}, f)
        resolved = os.path.join(tmp, "resolved.json")
        with open(resolved, "w", encoding="utf-8") as f:
            json.dump({"nodes": list(launches),
                       "effective": {"robot": robot or {},
                                     **(effective or {})}}, f)
        builder = runfiles.Create().Rlocation("_main/talOS/build/bundle_builder.py")
        return subprocess.run([sys.executable, builder, "--spec", spec,
                               "--resolved", resolved,
                               "--out", os.path.join(tmp, "out"), *extra],
                              capture_output=True, text=True)

    def build(self, tmp, files, nodes=(), check=True, extra=()):
        """Bundles `files` ({short_path: text}); the first is the config.
        `nodes`: {name: {library basename: bytes}} C++ nodes and the shared
        libraries each links, which the config launches beside the default
        hardware node."""
        specs = []
        for name, libs in dict(nodes).items():
            spec = node(tmp, name)
            for lib, text in libs.items():
                path = os.path.join(tmp, "src", "solib", name, lib)
                put(path, text)
                spec["shared_libs"].append({"src_path": path, "name": lib})
            specs.append(spec)
        proc = self.assemble(tmp, files,
                             [node(tmp, "hardware_node", HARDWARE_NODE)] + specs,
                             [spec["target"] for spec in specs], extra=extra)
        if not check:
            return proc
        self.assertEqual(proc.returncode, 0, proc.stderr)
        out = os.path.join(tmp, "out")
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

    def test_manifest_records_binary_digests_and_the_stamped_revision(self):
        empty_sha256 = ("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b"
                        "7852b855")
        with tempfile.TemporaryDirectory() as tmp:
            _, manifest = self.build(tmp, {"c/robot.toml": "[robot]\n"})
            self.assertIn(f'binary_sha256 = "{empty_sha256}"', manifest)
            # Unstamped: no revision rather than a stale or made-up one.
            self.assertNotIn("git_revision", manifest)
        with tempfile.TemporaryDirectory() as tmp:
            status = os.path.join(tmp, "stable-status.txt")
            put(status, "BUILD_HOST h\nSTABLE_GIT_COMMIT 0123abcd\n"
                "STABLE_GIT_DIRTY 1\n")
            _, manifest = self.build(tmp, {"c/robot.toml": "[robot]\n"},
                                     extra=("--stable-status", status))
            self.assertIn('git_revision = "0123abcd"', manifest)
            self.assertIn("git_dirty = true", manifest)

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

    def test_nodes_are_exactly_what_the_config_launches(self):
        # The launcher finds a node's binary by label, else by its name: a
        # bundle short of a node stops the robot at boot, or runs the default
        # bridge in place of the config's [robot] hardware_node.
        bridge = {"hardware_node": "//bot:bridge"}
        for bundled, launches, robot, problem in (
                ((HARDWARE_NODE,), ("//arm:node",), {}, "//arm:node"),
                ((HARDWARE_NODE, "//arm:node"), (), {}, "//arm:node"),
                ((HARDWARE_NODE, "//arm:node"), ("//arm:node",), bridge,
                 "//bot:bridge"),
                (("//bot:bridge", "//arm:node"), ("//arm:node",), bridge, None)):
            with self.subTest(bundled=bundled, launches=launches, robot=robot), \
                    tempfile.TemporaryDirectory() as tmp:
                nodes = [node(tmp, label.split(":")[1], label) for label in bundled]
                proc = self.assemble(tmp, {"c/robot.toml": "[robot]\n"}, nodes,
                                     launches, robot)
                if problem is None:
                    self.assertEqual(proc.returncode, 0, proc.stderr)
                else:
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn(problem, proc.stderr)

    def test_studio_must_be_the_configs_bridge_and_a_built_site(self):
        # A bundle whose bridge is not the one [studio] names would run
        # another viewer, found by name; a site with no index page serves
        # nothing.
        for configured, index, problem in (
                (None, True, None),
                ("//viewer:bridge", True, None),
                ("//other:bridge", True, "//other:bridge"),
                (None, False, "index.html")):
            with self.subTest(configured=configured, index=index), \
                    tempfile.TemporaryDirectory() as tmp:
                bridge = os.path.join(tmp, "src", "studio_bridge")
                put(bridge, "")
                webroot = os.path.join(tmp, "src", "dist")
                os.makedirs(webroot)
                if index:
                    put(os.path.join(webroot, "index.html"), "<html>")
                proc = self.assemble(
                    tmp, {"c/robot.toml": "[robot]\n"},
                    [node(tmp, "hardware_node", HARDWARE_NODE)], (),
                    studio={"bridge_path": bridge,
                            "bridge_label": "//viewer:bridge",
                            "webroot_path": webroot},
                    effective={"studio": {"bridge": configured}
                               if configured else {}})
                if problem is not None:
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn(problem, proc.stderr)
                    continue
                self.assertEqual(proc.returncode, 0, proc.stderr)
                out = os.path.join(tmp, "out")
                with open(os.path.join(out, "bundle_manifest.toml"), "rb") as f:
                    studio = tomllib.load(f)["studio"]
                self.assertTrue(os.access(os.path.join(out, studio["bridge"]),
                                          os.X_OK))
                self.assertTrue(os.path.isfile(
                    os.path.join(out, studio["webroot"], "index.html")))

    def test_runfiles_the_bundle_cannot_hold_fail(self):
        # A Python node imports from its runfiles only on the robot. A link
        # Bazel left unresolved (a py_binary venv's bin/python3) points into
        # the runfiles tree, so what it names must be in the bundle.
        interpreter = "rules_python++python+python_3_11/bin/python3"
        venv_python = "_main/py/_node_bin.venv/bin/python3"
        gone = "_main/py/gone.py"
        for staged, problem in (((venv_python,), venv_python),
                                ((gone,), gone)):
            with self.subTest(staged=staged), tempfile.TemporaryDirectory() as tmp:
                put(os.path.join(tmp, "src", "python3"), "")
                # Up from the venv's bin/ to the runfiles root, as Bazel
                # declares it; from src/venv/bin it resolves nowhere.
                link = os.path.join(tmp, "src", "venv", "bin", "python3")
                os.makedirs(os.path.dirname(link))
                os.symlink("../../../../" + interpreter, link)
                sources = {interpreter: os.path.join(tmp, "src", "python3"),
                           venv_python: link,
                           gone: os.path.join(tmp, "src", "gone.py")}
                py = dict(node(tmp, "py"), language="python", runfiles=[
                    {"src_path": sources[rel], "rf_rel": rel} for rel in staged])
                proc = self.assemble(tmp, {"c/robot.toml": "[robot]\n"},
                                     [node(tmp, "hardware_node", HARDWARE_NODE), py],
                                     ["//py:node"])
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn(problem, proc.stderr)

    # REQ-OPS-48
    def test_a_bundle_restored_from_a_cache_runs_its_python_nodes(self):
        # A bundle is a tree artifact, and Bazel may hand one back from its
        # disk or remote cache with each symlink inside it replaced by a copy
        # of its target (a `bazel test` that restores the bundle as a test's
        # input did). The venv's bin/python3, a link to the
        # bundled interpreter, then became a copy that looks for its standard
        # library beside itself, and every Python node died at start with
        # "No module named 'encodings'". The bundle must run the same whether
        # built or restored: it holds no symlink, and the venv's interpreter,
        # run from a copy that dereferences every link, finds its library.
        interpreter = "rules_python++python+python_3_11/bin/python3"
        stdlib = "rules_python++python+python_3_11/lib/python3.11/encodings/__init__.py"
        venv_python = "_main/py/_node_bin.venv/bin/python3"
        with tempfile.TemporaryDirectory() as tmp:
            # As CPython does, the stand-in resolves its own links and looks
            # for its library from where its file is.
            real = os.path.join(tmp, "src", "python3")
            put(real, '#!/bin/sh\nhome="$(dirname "$(realpath "$0")")/.."\n'
                'test -d "$home/lib/python3.11/encodings" || {\n'
                '  echo "No module named encodings" >&2; exit 1; }\n'
                'echo "ran $*"\n')
            os.chmod(real, 0o755)
            put(os.path.join(tmp, "src", "encodings.py"), "")
            # Up from the venv's bin/ to the runfiles root, as Bazel
            # declares it; from src/venv/bin it resolves nowhere.
            link = os.path.join(tmp, "src", "venv", "bin", "python3")
            os.makedirs(os.path.dirname(link))
            os.symlink("../../../../" + interpreter, link)
            py = dict(node(tmp, "py"), language="python", runfiles=[
                {"src_path": real, "rf_rel": interpreter},
                {"src_path": os.path.join(tmp, "src", "encodings.py"),
                 "rf_rel": stdlib},
                {"src_path": link, "rf_rel": venv_python}])
            proc = self.assemble(tmp, {"c/robot.toml": "[robot]\n"},
                                 [node(tmp, "hardware_node", HARDWARE_NODE), py],
                                 ["//py:node"])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            built = os.path.join(tmp, "out")
            links = [os.path.relpath(os.path.join(parent, name), built)
                     for parent, dirs, files in os.walk(built)
                     for name in dirs + files
                     if os.path.islink(os.path.join(parent, name))]
            self.assertEqual(links, [])
            restored = os.path.join(tmp, "restored")
            shutil.copytree(built, restored, symlinks=False)
            for tree in (built, restored):
                with self.subTest(tree=os.path.basename(tree)):
                    run = subprocess.run(
                        [os.path.join(tree, "bin", "py.runfiles", venv_python),
                         "-c", "pass"],
                        capture_output=True, text=True, timeout=60)
                    self.assertEqual((run.returncode, run.stdout),
                                     (0, "ran -c pass\n"), run.stderr)

    def python_nodes(self, tmp, interpreters):
        """Specs of Python nodes, one per {name: interpreter text}, each
        carrying the runtime repository RUNTIME as rules_python's runfiles
        do: the interpreter as bin/python3 and again, as copies, under its
        other names, its standard library, and a venv whose bin/python3 is a
        link Bazel declared to it."""
        venv_python = "_main/{0}/_node_bin.venv/bin/python3"
        specs = []
        for name, text in interpreters.items():
            src = os.path.join(tmp, "src", name)
            real = os.path.join(src, "python3")
            put(real, text)
            os.chmod(real, 0o755)
            put(os.path.join(src, "encodings.py"), "")
            link = os.path.join(src, "venv", "bin", "python3")
            os.makedirs(os.path.dirname(link))
            os.symlink("../../../../" + RUNTIME + "/bin/python3", link)
            rf = [{"src_path": real, "rf_rel": f"{RUNTIME}/bin/{alias}"}
                  for alias in ("python", "python3", "python3.11")]
            rf += [{"src_path": os.path.join(src, "encodings.py"),
                    "rf_rel": RUNTIME + "/lib/python3.11/encodings/__init__.py"},
                   {"src_path": link, "rf_rel": venv_python.format(name)}]
            specs.append(dict(node(tmp, name, f"//{name}:node"),
                              language="python", runfiles=rf))
        return specs

    # REQ-OPS-48
    def test_python_nodes_share_one_interpreter(self):
        # Each Python node's runfiles carry the whole interpreter (rules_python
        # copies it under python, python3 and python3.11): a 52 MB file six
        # times over for two nodes, most of a 749 MB bundle copied to the
        # robot on every deploy. The bundle holds its bytes once, and each
        # node's venv runs that file in the built tree.
        interpreter = ('#!/bin/sh\nhome="$(dirname "$(realpath "$0")")/.."\n'
                       'test -d "$home/lib/python3.11/encodings" || {\n'
                       '  echo "No module named encodings" >&2; exit 1; }\n'
                       'echo "ran $0"\n')
        with tempfile.TemporaryDirectory() as tmp:
            nodes = self.python_nodes(tmp, {"a": interpreter, "b": interpreter})
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE)] + nodes,
                ["//a:node", "//b:node"])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            built = os.path.join(tmp, "out")
            held = []
            for parent, _, files in os.walk(built):
                for name in files:
                    path = os.path.join(parent, name)
                    with open(path, encoding="utf-8", errors="replace") as f:
                        if f.read() == interpreter:
                            held.append(os.path.relpath(path, built))
            self.assertEqual(held, [RUNTIME_DIR + "/" + RUNTIME + "/bin/python3"])
            for name in ("a", "b"):
                venv = os.path.join(built, "bin", f"{name}.runfiles", "_main",
                                    name, "_node_bin.venv", "bin", "python3")
                with self.subTest(node=name):
                    run = subprocess.run([venv], capture_output=True,
                                         text=True, timeout=60)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    self.assertEqual(
                        os.path.realpath(run.stdout.split(" ", 1)[1].strip()),
                        os.path.realpath(os.path.join(built, held[0])))

    # REQ-OPS-48
    def test_a_runtime_no_venv_link_selects_fails(self):
        # A runtime repository a venv link selects is staged once, in
        # python/. One that no link selects (a venv built without its
        # declared symlink) was staged whole inside the node's runfiles
        # again: the build passed and the bundle grew by the runtime, 242 MB
        # per node.
        with tempfile.TemporaryDirectory() as tmp:
            (py,) = self.python_nodes(tmp, {"a": "#!/bin/sh\n"})
            py["runfiles"] = [rf for rf in py["runfiles"]
                              if not rf["rf_rel"].startswith("_main/")]
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE), py], ["//a:node"])
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("node 'a'", proc.stderr)
            self.assertIn(RUNTIME, proc.stderr)
            self.assertIn("no venv link", proc.stderr)

    # REQ-OPS-48
    def test_two_different_runtimes_under_one_name_fail(self):
        # One runtime directory serves every node, so two nodes that carry
        # different interpreters under one repository name cannot share it.
        with tempfile.TemporaryDirectory() as tmp:
            nodes = self.python_nodes(tmp, {"a": "#!/bin/sh\necho 3.11.1\n",
                                            "b": "#!/bin/sh\necho 3.11.2\n"})
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE)] + nodes,
                ["//a:node", "//b:node"])
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn(RUNTIME, proc.stderr)
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

    # REQ-OPS-34
    def test_run_script_starts_a_carried_studio_unless_declined(self):
        # The boot service runs run.sh bare, so the default is what a booted
        # robot gets. --no-studio is run.sh's own and must not reach the
        # launcher, which would refuse it.
        for carried, given, starts in ((True, [], True),
                                       (True, ["--no-studio"], False),
                                       (False, [], False)):
            with self.subTest(carried=carried, given=given), \
                    tempfile.TemporaryDirectory() as tmp:
                studio = None
                if carried:
                    bridge = os.path.join(tmp, "src", "studio_bridge")
                    put(bridge, "")
                    webroot = os.path.join(tmp, "src", "dist")
                    put(os.path.join(webroot, "index.html"), "<html>")
                    studio = {"bridge_path": bridge,
                              "bridge_label": "//viewer:bridge",
                              "webroot_path": webroot}
                proc = self.assemble(
                    tmp, {"c/robot.toml": "[robot]\n"},
                    [node(tmp, "hardware_node", HARDWARE_NODE)], (),
                    studio=studio)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                out = os.path.join(tmp, "out")
                self.echo_launcher(out)
                proc = subprocess.run(
                    [os.path.join(out, "run.sh"), *given, "--duration-s", "1"],
                    capture_output=True, text=True)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                args = json.loads(proc.stdout)
                self.assertEqual("--start-studio-bridge" in args, starts, args)
                self.assertNotIn("--no-studio", args)
                self.assertEqual(args[-2:], ["--duration-s", "1"])


class DeployTest(unittest.TestCase):
    def deploy(self, prefix, *args, cwd=None, path=None):
        script = os.path.abspath(
            runfiles.Create().Rlocation("_main/tools/deploy.sh"))
        env = dict(os.environ)
        # Never the machine's own boot service log.
        env["TALOS_LAUNCHD_STDERR"] = os.path.join(prefix, "launchd.stderr.log")
        if path:
            env["PATH"] = path + os.pathsep + env["PATH"]
        return subprocess.run(["bash", script, *args, "--prefix", prefix],
                              cwd=cwd, capture_output=True, text=True, env=env)

    def macos(self, tmp, filevault="FileVault is Off.", autorestart="1",
              sleep="0"):
        """A PATH directory faking macOS and its power settings. `sudo`
        records its arguments and fails, so nothing privileged runs."""
        bin_dir = os.path.join(tmp, "fake_bin")
        scripts = {
            "uname": "echo Darwin",
            "fdesetup": f"echo '{filevault}'",
            "pmset": ("printf 'Currently in use:\\n"
                      f" autorestart          {autorestart}\\n"
                      f" sleep                {sleep} (sleep prevented by x)"
                      "\\n'"),
            "defaults": "exit 1",
            "sudo": f'echo "$*" >> "{tmp}/sudo.log"; exit 1',
        }
        for name, body in scripts.items():
            put(os.path.join(bin_dir, name), f"#!/bin/sh\n{body}\n")
            os.chmod(os.path.join(bin_dir, name), 0o755)
        return bin_dir

    def session(self, prefix, name, timestamp, start_ns=0, **fields):
        path = os.path.join(prefix, "logs", name)
        put(os.path.join(path, "manifest.json"), json.dumps({
            "session_id": name, "config_path": "config/robot.toml",
            "config_sha256": "abcdef", "start_wall_ns": start_ns,
            "nodes": [], **fields}))
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

    def test_logs_orders_sessions_by_start_time_and_reports_each(self):
        second = 1_000_000_000
        with tempfile.TemporaryDirectory() as tmp:
            # Names and manifest modification times both sort opposite to
            # the recorded start times; an id is not a time.
            newest = self.session(
                tmp, "a_new", 1 * second, start_ns=1_700_000_300 * second,
                git_revision="0123456789abcdef", git_dirty=True,
                nodes=[{"name": "arm", "exit_code": 0, "restart_count": 1,
                        "runs": [{"reason": "crash"}, {"reason": "stop"}]},
                       {"name": "shooter", "exit_code": 0,
                        "runs": [{"reason": "stop"}]}])
            put(os.path.join(newest, "session_class"), "match 30\n")
            oldest = self.session(tmp, "z_old", 3 * second,
                                  start_ns=1_700_000_100 * second)
            # No start time: ordered by the directory's modification time.
            middle = self.session(tmp, "m_mid", 2 * second)
            os.utime(middle, ns=(1_700_000_200 * second,) * 2)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            out = proc.stdout
            self.assertLess(out.index(oldest), out.index(middle))
            self.assertLess(out.index(middle), out.index(newest))
            self.assertIn(f"newest session manifest: {newest}/manifest.json",
                          out)

            def fields(prefix):
                return next(line.split() for line in out.splitlines()
                            if line.strip().startswith(prefix))

            summary = fields("session a_new")
            for field in ("0123456789ab", "dirty", "match"):
                self.assertIn(field, summary)
            self.assertRegex(" ".join(fields(newest)), r"\b1 crash, 2 stop\b")
            self.assertIn("unstamped", fields(oldest))
            self.assertIn("unclassified", fields(oldest))
            self.assertIn("last=stop", fields("arm:"))
            self.assertIn("restarts=1", fields("arm:"))
            self.assertNotIn("left no manifest", out)

    def test_logs_shows_launcher_stderr_a_failed_launch_left(self):
        second = 1_000_000_000
        with tempfile.TemporaryDirectory() as tmp:
            self.session(tmp, "one", 1, start_ns=1_700_000_000 * second)
            stderr_log = os.path.join(tmp, "launchd.stderr.log")
            put(stderr_log, "older line\nconfig error: no [robot] table\n")
            # Written before the newest session started: stale, not shown.
            os.utime(stderr_log, ns=(1_600_000_000 * second,) * 2)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn(f"launcher stderr under launchd: {stderr_log}",
                          proc.stdout)
            self.assertNotIn("config error", proc.stdout)
            # A later launch failed before it wrote any manifest.
            os.utime(stderr_log, ns=(1_700_000_100 * second,) * 2)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("  | config error: no [robot] table", proc.stdout)

    def test_enable_boot_on_macos_is_unthrottled_with_bounded_stderr(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = os.path.join(tmp, "prefix")
            proc = self.deploy(prefix, "enable-boot", "--dry-run",
                               path=self.macos(tmp))
            self.assertEqual(proc.returncode, 0, proc.stderr)
            out = proc.stdout
            plist = plistlib.loads(out[out.index("<?xml"):
                                       out.index("</plist>") + 8].encode())
            # Unset, launchd throttles the job's CPU and I/O.
            self.assertEqual(plist["ProcessType"], "Interactive")
            # Outside the log root the service account owns: root's
            # newsyslog chowns rotated copies there by path.
            stderr_log = "/Library/Logs/talos/launchd.stderr.log"
            self.assertEqual(plist["StandardErrorPath"], stderr_log)
            self.assertEqual(plist["StandardOutPath"], "/dev/null")
            # newsyslog rotates it past 1024 KiB, keeping four copies, with
            # no "turned over" line (B) and no process to signal (N).
            self.assertRegex(
                out, rf"(?m)^{re.escape(stderr_log)} \S+ 644 4 1024 \* NB$")
            self.assertIn("+ sudo pmset -a autorestart 1 sleep 0", out)

    def test_enable_boot_refuses_filevault_before_writing_anything(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = self.deploy(os.path.join(tmp, "prefix"), "enable-boot",
                               path=self.macos(tmp, "FileVault is On."))
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("FileVault is On.", proc.stderr)
            self.assertFalse(os.path.exists(os.path.join(tmp, "sudo.log")))

    def test_boot_check_fails_unless_each_boot_setting_holds(self):
        cases = {
            "ok": ({}, 0),
            "filevault pending": (
                {"filevault": "FileVault is Off, but will be enabled after "
                              "the next restart."}, 1),
            "no autorestart": ({"autorestart": "0"}, 1),
            "sleeps": ({"sleep": "10"}, 1),
        }
        for name, (settings, status) in cases.items():
            with self.subTest(name), tempfile.TemporaryDirectory() as tmp:
                proc = self.deploy(tmp, "boot-check",
                                   path=self.macos(tmp, **settings))
                self.assertEqual(proc.returncode, status, proc.stdout)

    def test_logs_with_no_sessions_says_so(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.makedirs(os.path.join(tmp, "logs"))
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn("no sessions yet", proc.stdout)
            self.assertNotIn("left no manifest", proc.stdout)

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
