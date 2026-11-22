"""Tests bundle_builder.py's check of a bundle's nodes against its config,
its config staging (relative layout and identity), its manifest's binary
digests and stamped revision, its staging of runfiles (the Python
runtime once, in python/) and of C++ nodes' shared libraries in lib/, its check of a bundled Studio
against the config, and its run.sh (including the Studio it starts), and
tools/deploy.sh's bundle selection, pruning and session commands."""

import importlib.util
import json
import os
import plistlib
import re
import shutil
import struct
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


def put_bytes(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


def make_elf(needed=(), *, bits=64, endian="<", e_type=3, vaddr_base=0x400000):
    """Builds a minimal valid ELF32/ELF64 image with PT_LOAD + PT_DYNAMIC
    listing `needed` as DT_NEEDED entries (or an ET_REL object with no
    program headers when e_type == 1)."""
    ei_class = 2 if bits == 64 else 1
    ei_data = 1 if endian == "<" else 2
    ident = b"\x7fELF" + bytes([ei_class, ei_data, 1]) + b"\x00" * 9
    ehdr_size = 64 if bits == 64 else 52
    phentsize = 56 if bits == 64 else 32
    if e_type == 1:
        if bits == 64:
            hdr = struct.pack(endian + "HHIQQQIHHHHHH",
                              e_type, 62, 1, 0, 0, 0, 0, ehdr_size, 0, 0, 0, 0, 0)
        else:
            hdr = struct.pack(endian + "HHIIIIIHHHHHH",
                              e_type, 3, 1, 0, 0, 0, 0, ehdr_size, 0, 0, 0, 0, 0)
        return ident + hdr

    dynstr = bytearray(b"\x00")
    offsets = []
    for name in needed:
        offsets.append(len(dynstr))
        dynstr.extend(name.encode("utf-8") + b"\x00")

    phoff = ehdr_size
    phnum = 2
    dyn_off = phoff + phnum * phentsize
    dyn_entry_size = 16 if bits == 64 else 8
    dyn_entries = [(1, off) for off in offsets]  # DT_NEEDED
    dyn_entries.append((5, 0))   # DT_STRTAB placeholder
    dyn_entries.append((10, len(dynstr)))  # DT_STRSZ
    dyn_entries.append((0, 0))   # DT_NULL
    dyn_size = len(dyn_entries) * dyn_entry_size
    dynstr_off = dyn_off + dyn_size
    dyn_entries[-3] = (5, vaddr_base + dynstr_off)
    total = dynstr_off + len(dynstr)

    if bits == 64:
        hdr = struct.pack(endian + "HHIQQQIHHHHHH",
                          e_type, 62, 1, 0, phoff, 0, 0,
                          ehdr_size, phentsize, phnum, 0, 0, 0)
        ph_load = struct.pack(endian + "IIQQQQQQ",
                              1, 5, 0, vaddr_base, vaddr_base, total, total, 0x1000)
        ph_dyn = struct.pack(endian + "IIQQQQQQ",
                             2, 6, dyn_off, vaddr_base + dyn_off,
                             vaddr_base + dyn_off, dyn_size, dyn_size, 8)
        dyn_bytes = b"".join(struct.pack(endian + "qQ", t, v)
                             for t, v in dyn_entries)
    else:
        hdr = struct.pack(endian + "HHIIIIIHHHHHH",
                          e_type, 3, 1, 0, phoff, 0, 0,
                          ehdr_size, phentsize, phnum, 0, 0, 0)
        ph_load = struct.pack(endian + "IIIIIIII",
                              1, 0, vaddr_base, vaddr_base, total, total, 5, 0x1000)
        ph_dyn = struct.pack(endian + "IIIIIIII",
                             2, dyn_off, vaddr_base + dyn_off,
                             vaddr_base + dyn_off, dyn_size, dyn_size, 6, 4)
        dyn_bytes = b"".join(struct.pack(endian + "iI", t, v)
                             for t, v in dyn_entries)
    return ident + hdr + ph_load + ph_dyn + dyn_bytes + bytes(dynstr)


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
            self.assertNotIn("talos_git_revision", manifest)
        with tempfile.TemporaryDirectory() as tmp:
            status = os.path.join(tmp, "stable-status.txt")
            put(status, "BUILD_HOST h\nSTABLE_GIT_COMMIT 0123abcd\n"
                "STABLE_GIT_DIRTY 1\n")
            _, manifest = self.build(tmp, {"c/robot.toml": "[robot]\n"},
                                     extra=("--stable-status", status))
            self.assertIn('git_revision = "0123abcd"', manifest)
            self.assertIn("git_dirty = true", manifest)
            self.assertNotIn("talos_git_revision", manifest)
        # Dual-revision stamp from a separate robot repository (REPO-4).
        with tempfile.TemporaryDirectory() as tmp:
            status = os.path.join(tmp, "stable-status.txt")
            put(status,
                "STABLE_GIT_COMMIT 1111aaaa\nSTABLE_GIT_DIRTY 0\n"
                "STABLE_TALOS_GIT_COMMIT 2222bbbb\nSTABLE_TALOS_GIT_DIRTY 1\n")
            _, manifest = self.build(tmp, {"c/robot.toml": "[robot]\n"},
                                     extra=("--stable-status", status))
            self.assertIn('git_revision = "1111aaaa"', manifest)
            self.assertIn("git_dirty = false", manifest)
            self.assertIn('talos_git_revision = "2222bbbb"', manifest)
            self.assertIn("talos_git_dirty = true", manifest)
        # talOS checkout stamped even when the robot directory is not a git repo.
        with tempfile.TemporaryDirectory() as tmp:
            status = os.path.join(tmp, "stable-status.txt")
            put(status,
                "STABLE_TALOS_GIT_COMMIT 3333cccc\nSTABLE_TALOS_GIT_DIRTY 0\n")
            _, manifest = self.build(tmp, {"c/robot.toml": "[robot]\n"},
                                     extra=("--stable-status", status))
            self.assertNotIn("\ngit_revision =", "\n" + manifest)
            self.assertIn('talos_git_revision = "3333cccc"', manifest)
            self.assertIn("talos_git_dirty = false", manifest)

    # REQ-OPS-10, REQ-DEV-30 (REPO-4): workspace_status.sh stamps both the
    # robot repo and talOS when invoked from a separate robot repository,
    # omits STABLE_TALOS_GIT_* in a single repo or vendored tree, and still
    # stamps talOS when the robot directory is outside git.
    def test_workspace_status_stamps_separate_robot_and_talos_repos(self):
        status_src = os.path.abspath(
            runfiles.Create().Rlocation("_main/tools/workspace_status.sh"))

        def git(repo, *args):
            env = dict(os.environ,
                       GIT_AUTHOR_NAME="Test", GIT_AUTHOR_EMAIL="t@example.com",
                       GIT_COMMITTER_NAME="Test",
                       GIT_COMMITTER_EMAIL="t@example.com")
            for k in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE"):
                env.pop(k, None)
            proc = subprocess.run(
                ["git", "-C", repo, *args],
                capture_output=True, text=True, env=env)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            return proc.stdout.strip()

        def init_repo(path, files, executable=()):
            os.makedirs(path, exist_ok=True)
            git(path, "init", "-q")
            for rel, content in files.items():
                full = os.path.join(path, rel)
                put(full, content)
                if rel in executable:
                    os.chmod(full, 0o755)
            git(path, "add", ".")
            git(path, "commit", "-q", "-m", "init")
            return git(path, "rev-parse", "HEAD")

        def run_status(script, cwd):
            env = {k: v for k, v in os.environ.items()
                   if k not in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE")}
            proc = subprocess.run(["bash", script], cwd=cwd,
                                  capture_output=True, text=True, env=env)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            parsed = {}
            for line in proc.stdout.splitlines():
                k, _, v = line.partition(" ")
                parsed[k] = v
            return parsed

        with tempfile.TemporaryDirectory() as tmp:
            with open(status_src, encoding="utf-8") as f:
                script_body = f.read()
            talos_dir = os.path.join(tmp, "talos")
            talos_rev = init_repo(
                talos_dir, {"tools/workspace_status.sh": script_body},
                executable=("tools/workspace_status.sh",))
            talos_script = os.path.join(talos_dir, "tools", "workspace_status.sh")

            # 1. Single repo (inside talOS): emits only STABLE_GIT_*.
            self.assertEqual(run_status(talos_script, talos_dir), {
                "STABLE_GIT_COMMIT": talos_rev,
                "STABLE_GIT_DIRTY": "0",
            })

            # 2. Separate clean robot repo: emits both revisions, both clean.
            robot_dir = os.path.join(tmp, "robot")
            robot_rev = init_repo(robot_dir, {"robot.toml": "[robot]\n"})
            self.assertEqual(run_status(talos_script, robot_dir), {
                "STABLE_GIT_COMMIT": robot_rev,
                "STABLE_GIT_DIRTY": "0",
                "STABLE_TALOS_GIT_COMMIT": talos_rev,
                "STABLE_TALOS_GIT_DIRTY": "0",
            })

            # 3. Dirty robot + dirty talOS: both dirty bits flip to 1.
            put(os.path.join(robot_dir, "untracked.txt"), "dirty\n")
            put(os.path.join(talos_dir, "untracked.txt"), "dirty\n")
            self.assertEqual(run_status(talos_script, robot_dir), {
                "STABLE_GIT_COMMIT": robot_rev,
                "STABLE_GIT_DIRTY": "1",
                "STABLE_TALOS_GIT_COMMIT": talos_rev,
                "STABLE_TALOS_GIT_DIRTY": "1",
            })

            # 4. Non-git robot directory: does not exit early; stamps talOS.
            nongit_dir = os.path.join(tmp, "nongit")
            os.makedirs(nongit_dir)
            self.assertEqual(run_status(talos_script, nongit_dir), {
                "STABLE_TALOS_GIT_COMMIT": talos_rev,
                "STABLE_TALOS_GIT_DIRTY": "1",
            })

            # 5. Vendored copy inside robot repo (no separate talOS .git):
            # toplevel != TALOS_DIR, so STABLE_TALOS_GIT_* is not emitted.
            vendored = os.path.join(
                robot_dir, "vendor", "talos", "tools", "workspace_status.sh")
            put(vendored, script_body)
            os.chmod(vendored, 0o755)
            self.assertEqual(run_status(vendored, robot_dir), {
                "STABLE_GIT_COMMIT": robot_rev,
                "STABLE_GIT_DIRTY": "1",
            })

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

    # REQ-OPS-34
    def test_studio_must_be_the_configs_bridge_and_a_built_site(self):
        # A bundle whose bridge is not the one [studio] names would run
        # another viewer, found by name; a site with no index page serves
        # nothing; one with no driver dashboard leaves the drive team without
        # `/drive` (REQ-OPS-34). Either refusal names the build command.
        for configured, pages, problem in (
                (None, ("index.html", "drive.html"), None),
                ("//viewer:bridge", ("index.html", "drive.html"), None),
                ("//other:bridge", ("index.html", "drive.html"),
                 "//other:bridge"),
                (None, ("drive.html",), "index.html"),
                (None, ("index.html",), "drive.html")):
            with self.subTest(configured=configured, pages=pages), \
                    tempfile.TemporaryDirectory() as tmp:
                bridge = os.path.join(tmp, "src", "studio_bridge")
                put(bridge, "")
                webroot = os.path.join(tmp, "src", "dist")
                os.makedirs(webroot)
                for page in pages:
                    put(os.path.join(webroot, page), "<html>")
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
                    if problem.endswith(".html"):
                        self.assertIn("bazel build //studio:web_dist",
                                      proc.stderr)
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

    # REQ-OPS-48
    def test_strips_test_only_wheels_from_python_node_runfiles(self):
        # gtsam's wheel metadata pulls in pytest and its four dependencies
        # (pluggy, iniconfig, pygments, packaging) as test-only wheels (~7 MB).
        # The bundler strips those external site-packages wheel directories while
        # keeping real wheels (including numpy/_pytesttester.py), runtime files,
        # and any user code under _main/.
        with tempfile.TemporaryDirectory() as tmp:
            (py,) = self.python_nodes(tmp, {"a": "#!/bin/sh\n"})
            stripped = (
                "rules_python++pip+pypi_311_pytest_py3_none_any_07acdcf8/site-packages/pytest/__init__.py",
                "rules_python++pip+pypi_311_pytest_py3_none_any_07acdcf8/site-packages/_pytest/main.py",
                "rules_python++pip+pypi_311_pytest_py3_none_any_07acdcf8/site-packages/py.py",
                "rules_python++pip+pypi_311_pytest_py3_none_any_07acdcf8/site-packages/pytest-9.2.0.dist-info/METADATA",
                "rules_python++pip+pypi_311_pluggy_py3_none_any_7dd7b0d8/site-packages/pluggy/__init__.py",
                "rules_python++pip+pypi_311_pluggy_py3_none_any_7dd7b0d8/site-packages/pluggy-1.7.0.dist-info/METADATA",
                "rules_python++pip+pypi_311_iniconfig_py3_none_any_9121e2c1/site-packages/iniconfig/__init__.py",
                "rules_python++pip+pypi_311_iniconfig_py3_none_any_9121e2c1/site-packages/iniconfig-2.3.1.dist-info/METADATA",
                "rules_python++pip+pypi_311_pygments_py3_none_any_2363c69b/site-packages/pygments/__init__.py",
                "rules_python++pip+pypi_311_pygments_py3_none_any_2363c69b/site-packages/pygments-2.21.0.dist-info/METADATA",
                "rules_python++pip+pypi_311_packaging_py3_none_any_d7193f7c/site-packages/packaging/__init__.py",
                "rules_python++pip+pypi_311_packaging_py3_none_any_d7193f7c/site-packages/packaging-26.3.dist-info/METADATA",
            )
            kept_in_runfiles = (
                "rules_python++pip+pypi_311_gtsam_cp311_manylinux/site-packages/gtsam/__init__.py",
                "rules_python++pip+pypi_311_numpy_cp311_manylinux/site-packages/numpy/_pytesttester.py",
                "_main/packaging/node.py",
                "_main/site-packages/pytest/user_code.py",
            )
            kept_in_runtime = (
                RUNTIME + "/lib/python3.11/site-packages/pip/_vendor/packaging/__init__.py",
            )
            for idx, rf_rel in enumerate(stripped + kept_in_runfiles + kept_in_runtime):
                src = os.path.join(tmp, "src", "extra", f"f{idx}.py")
                put(src, f"# {rf_rel}\n")
                py["runfiles"].append({"src_path": src, "rf_rel": rf_rel})
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE), py], ["//a:node"])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            built = os.path.join(tmp, "out")
            rf_root = os.path.join(built, "bin", "a.runfiles")
            for rf_rel in stripped:
                self.assertFalse(
                    os.path.exists(os.path.join(rf_root, rf_rel)),
                    f"test-only wheel file was not stripped: {rf_rel}")
            for rf_rel in kept_in_runfiles:
                self.assertTrue(
                    os.path.isfile(os.path.join(rf_root, rf_rel)),
                    f"non-test runfile was stripped: {rf_rel}")
            for rf_rel in kept_in_runtime:
                self.assertTrue(
                    os.path.isfile(os.path.join(built, RUNTIME_DIR, rf_rel)),
                    f"runtime file was stripped: {rf_rel}")

    def _elf_python_node(self, tmp, interpreter_bytes, extra_runfiles=()):
        """Builds a single Python node spec whose bundled interpreter carries
        `interpreter_bytes` (under python, python3 and python3.11) plus
        libpython3.11.so.1.0, libpython3.so (which itself needs
        libpython3.11.so.1.0), and libother.so in the runtime's lib/."""
        src = os.path.join(tmp, "src", "py")
        real = os.path.join(src, "python3")
        put_bytes(real, interpreter_bytes)
        os.chmod(real, 0o755)
        put(os.path.join(src, "encodings.py"), "")
        link = os.path.join(src, "venv", "bin", "python3")
        os.makedirs(os.path.dirname(link))
        os.symlink("../../../../" + RUNTIME + "/bin/python3", link)
        libpy11 = os.path.join(src, "libpython3.11.so.1.0")
        put_bytes(libpy11, make_elf(["libc.so.6"]))
        os.chmod(libpy11, 0o755)
        libpy3 = os.path.join(src, "libpython3.so")
        put_bytes(libpy3, make_elf(["$ORIGIN/../lib/libpython3.11.so.1.0"]))
        os.chmod(libpy3, 0o755)
        libother = os.path.join(src, "libother.so")
        put_bytes(libother, make_elf(["libc.so.6"]))
        os.chmod(libother, 0o755)
        rf = [{"src_path": real, "rf_rel": f"{RUNTIME}/bin/{alias}"}
              for alias in ("python", "python3", "python3.11")]
        rf += [
            {"src_path": os.path.join(src, "encodings.py"),
             "rf_rel": f"{RUNTIME}/lib/python3.11/encodings/__init__.py"},
            {"src_path": libpy11,
             "rf_rel": f"{RUNTIME}/lib/libpython3.11.so.1.0"},
            {"src_path": libpy3,
             "rf_rel": f"{RUNTIME}/lib/libpython3.so"},
            {"src_path": libother,
             "rf_rel": f"{RUNTIME}/lib/libother.so"},
            {"src_path": link,
             "rf_rel": "_main/a/_node_bin.venv/bin/python3"},
        ]
        for idx, (rf_rel, data) in enumerate(extra_runfiles):
            p = os.path.join(src, f"extra_{idx}")
            put_bytes(p, data)
            rf.append({"src_path": p, "rf_rel": rf_rel})
        return dict(node(tmp, "a", "//a:node"), language="python", runfiles=rf)

    # REQ-OPS-48
    def test_strips_unused_libpython_shared_libs_when_statically_linked(self):
        # On Linux, rules_python's python3 binary statically links libpython
        # (no libpython* in DT_NEEDED) yet ships a 51 MB libpython3.11.so.1.0
        # and libpython3.so beside it. When neither python3 nor any other ELF
        # file in the bundle needs libpython*, the bundler omits them while
        # keeping unrelated runtime and extension .so files.
        with tempfile.TemporaryDirectory() as tmp:
            py = self._elf_python_node(
                tmp, make_elf(["libpthread.so.0", "libc.so.6"]),
                extra_runfiles=[(
                    "wheel/site-packages/pkg/_ext.cpython-311.so",
                    make_elf(["libc.so.6", "libm.so.6"]),
                )])
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE), py], ["//a:node"])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            built = os.path.join(tmp, "out")
            rt_lib = os.path.join(built, RUNTIME_DIR, RUNTIME, "lib")
            self.assertFalse(os.path.exists(
                os.path.join(rt_lib, "libpython3.11.so.1.0")))
            self.assertFalse(os.path.exists(
                os.path.join(rt_lib, "libpython3.so")))
            self.assertTrue(os.path.isfile(
                os.path.join(rt_lib, "libother.so")))
            self.assertTrue(os.path.isfile(os.path.join(
                built, "bin", "a.runfiles",
                "wheel/site-packages/pkg/_ext.cpython-311.so")))
            # Interpreter copies are still aliased after stripping.
            with open(os.path.join(built, RUNTIME_DIR, RUNTIME, "bin",
                                   "python3.11"), "rb") as f:
                self.assertTrue(f.read(2) == b"#!")

    # REQ-OPS-48
    def test_keeps_libpython_when_needed_or_non_elf_or_corrupt_elf(self):
        # Fail-closed safety checks: keep libpython3*.so* whenever python3
        # itself links libpython, any extension or shared library in the bundle
        # links libpython, the runtime is non-ELF (macOS Mach-O + .dylib), or
        # any ELF file in the bundle cannot be parsed.
        macho = b"\xcf\xfa\xed\xfe" + b"\x00" * 60
        cases = (
            ("dynamic python3",
             make_elf(["$ORIGIN/../lib/libpython3.11.so.1.0", "libc.so.6"]),
             (), (), True),
            ("wheel needs libpython3.so",
             make_elf(["libc.so.6"]),
             [("wheel/site-packages/abi3/_ext.abi3.so",
               make_elf(["libpython3.so", "libc.so.6"]))],
             (), True),
            ("cpp shared_lib needs libpython",
             make_elf(["libc.so.6"]),
             (), [("libembed.so", make_elf(["libpython3.11.so.1.0"]))], True),
            ("macos macho and dylib",
             macho,
             [(f"{RUNTIME}/lib/libpython3.11.dylib", macho)],
             (), True),
            ("corrupt elf fails closed",
             make_elf(["libc.so.6"]),
             [("wheel/site-packages/pkg/bad.so", b"\x7fELF\x02\x01\x01")],
             (), True),
        )
        for label, interp, extra_rf, cpp_libs, kept in cases:
            with self.subTest(label), tempfile.TemporaryDirectory() as tmp:
                py = self._elf_python_node(tmp, interp, extra_rf)
                hw = node(tmp, "hardware_node", HARDWARE_NODE)
                for lib_name, lib_bytes in cpp_libs:
                    p = os.path.join(tmp, "src", "solib", lib_name)
                    put_bytes(p, lib_bytes)
                    hw["shared_libs"].append({"src_path": p, "name": lib_name})
                proc = self.assemble(
                    tmp, {"c/robot.toml": "[robot]\n"}, [hw, py], ["//a:node"])
                self.assertEqual(proc.returncode, 0, proc.stderr)
                rt_lib = os.path.join(tmp, "out", RUNTIME_DIR, RUNTIME, "lib")
                self.assertEqual(
                    os.path.isfile(os.path.join(rt_lib, "libpython3.11.so.1.0")),
                    kept, label)
                self.assertEqual(
                    os.path.isfile(os.path.join(rt_lib, "libpython3.so")),
                    kept, label)
                if label == "macos macho and dylib":
                    self.assertTrue(os.path.isfile(
                        os.path.join(rt_lib, "libpython3.11.dylib")))

    # REQ-OPS-48
    def test_elf_needed_libraries_parses_formats_and_fails_closed(self):
        builder_path = runfiles.Create().Rlocation(
            "_main/talOS/build/bundle_builder.py")
        spec = importlib.util.spec_from_file_location(
            "bundle_builder", builder_path)
        bb = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bb)
        with tempfile.TemporaryDirectory() as tmp:
            p64 = os.path.join(tmp, "e64")
            put_bytes(p64, make_elf(["libc.so.6", "libm.so.6"], bits=64, endian="<"))
            self.assertEqual(bb.elf_needed_libraries(p64), ["libc.so.6", "libm.so.6"])

            p32 = os.path.join(tmp, "e32be")
            put_bytes(p32, make_elf(["libpython3.so"], bits=32, endian=">"))
            self.assertEqual(bb.elf_needed_libraries(p32), ["libpython3.so"])

            prel = os.path.join(tmp, "obj.o")
            put_bytes(prel, make_elf(e_type=1))
            self.assertEqual(bb.elf_needed_libraries(prel), [])

            ptext = os.path.join(tmp, "script.sh")
            put(ptext, "#!/bin/sh\n")
            self.assertIsNone(bb.elf_needed_libraries(ptext))

            pbad = os.path.join(tmp, "bad.elf")
            put_bytes(pbad, b"\x7fELF" + b"\x00" * 12)
            with self.assertRaises(bb.ElfParseError):
                bb.elf_needed_libraries(pbad)

            # Corrupt ELF with zero-length PT_DYNAMIC must fail closed.
            valid = bytearray(make_elf(["libc.so.6"], bits=64, endian="<"))
            # Second program header (index 1, offset 64 + 56 = 120) is PT_DYNAMIC;
            # p_filesz is at +32 within the 56-byte Elf64_Phdr.
            pzero = os.path.join(tmp, "zero_dyn.elf")
            z = bytearray(valid)
            z[64 + 56 + 32 : 64 + 56 + 40] = (0).to_bytes(8, "little")
            put_bytes(pzero, bytes(z))
            with self.assertRaises(bb.ElfParseError):
                bb.elf_needed_libraries(pzero)

            # Truncated PT_DYNAMIC without DT_NULL terminator must fail closed.
            pnoterm = os.path.join(tmp, "noterm_dyn.elf")
            nt = bytearray(valid)
            # Shrink PT_DYNAMIC p_filesz to 16 bytes (1 entry, before DT_NULL).
            nt[64 + 56 + 32 : 64 + 56 + 40] = (16).to_bytes(8, "little")
            put_bytes(pnoterm, bytes(nt))
            with self.assertRaises(bb.ElfParseError):
                bb.elf_needed_libraries(pnoterm)

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
        # The boot service runs run.sh --serve-refusal without --no-studio, so
        # the default is what a booted robot gets. --no-studio is run.sh's own
        # and must not reach the launcher, which would refuse it.
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
                    put(os.path.join(webroot, "drive.html"), "<html>")
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

    # REQ-DEV-30
    def test_repo_mapping_maps_custom_apparent_names_for_nodes_and_studio(self):
        # A robot repository with `bazel_dep(name = "talos", repo_name =
        # "my_talos")` names talOS nodes as `@my_talos//...` in its config while
        # Bazel's canonical labels are `@@talos+//...` (Bazel 8) or
        # `@@talos~//...` (Bazel 7). `_repo_mapping` supplies the real apparent
        # name to check_nodes, check_studio, and bundle_manifest.toml.
        for canonical, apparent in (("talos+", "my_talos"),
                                    ("talos~", "my_talos7")):
            with self.subTest(canonical=canonical, apparent=apparent), \
                    tempfile.TemporaryDirectory() as tmp:
                repo_mapping = os.path.join(tmp, "_repo_mapping")
                put(repo_mapping,
                    f",{apparent},{canonical}\n"
                    f"{canonical},rules_cc,rules_cc+\n")
                bridge = os.path.join(tmp, "src", "studio_bridge")
                put(bridge, "")
                webroot = os.path.join(tmp, "src", "dist")
                put(os.path.join(webroot, "index.html"), "<html>")
                put(os.path.join(webroot, "drive.html"), "<html>")
                hw_canon = f"@@{canonical}//talOS/bridge:hardware_node"
                teleop_canon = f"@@{canonical}//talOS/teleop:node"
                studio_canon = f"@@{canonical}//studio/bridge:studio_bridge"
                proc = self.assemble(
                    tmp, {"c/robot.toml": "[robot]\n"},
                    [node(tmp, "hardware_node", hw_canon),
                     node(tmp, "teleop", teleop_canon)],
                    [f"@{apparent}//talOS/teleop:node"],
                    robot={"hardware_node":
                           f"@{apparent}//talOS/bridge:hardware_node"},
                    extra=("--repo-mapping", repo_mapping),
                    studio={"bridge_path": bridge,
                            "bridge_label": studio_canon,
                            "webroot_path": webroot},
                    effective={"studio": {
                        "bridge":
                        f"@{apparent}//studio/bridge:studio_bridge"}})
                self.assertEqual(proc.returncode, 0, proc.stderr)
                with open(os.path.join(tmp, "out", "bundle_manifest.toml"),
                          "rb") as f:
                    manifest = tomllib.load(f)
                by_name = {n["name"]: n["targets"] for n in manifest["nodes"]}
                self.assertEqual(
                    by_name["teleop"],
                    [teleop_canon, f"@{apparent}//talOS/teleop:node"])
                self.assertEqual(
                    manifest["studio"]["targets"],
                    [studio_canon, f"@{apparent}//studio/bridge:studio_bridge"])

    # REQ-DEV-30
    def test_apparent_repo_maps_external_targets_without_main_repo_mapping_rows(self):
        # When 100% of a bundle's targets live in an external repo (`@my_talos`),
        # their `repo_mapping_manifest` files carry no `source == ""` main-repo
        # rows. The `talos_bundle` macro extracts `apparent=canonical` pairs in
        # the caller's repo context and passes `--apparent-repo`, so `check_nodes`,
        # `check_studio`, and `bundle_manifest.toml` still resolve `@my_talos`.
        for canonical, apparent in (("talos+", "my_talos"),
                                    ("talos~", "my_talos7")):
            with self.subTest(canonical=canonical, apparent=apparent), \
                    tempfile.TemporaryDirectory() as tmp:
                # External-repo manifest only: zero `source == ""` rows.
                repo_mapping = os.path.join(tmp, "_repo_mapping")
                put(repo_mapping,
                    f"{canonical},rules_cc,rules_cc+\n"
                    f"{canonical},rules_python,rules_python+\n")
                bridge = os.path.join(tmp, "src", "studio_bridge")
                put(bridge, "")
                webroot = os.path.join(tmp, "src", "dist")
                put(os.path.join(webroot, "index.html"), "<html>")
                put(os.path.join(webroot, "drive.html"), "<html>")
                hw_canon = f"@@{canonical}//talOS/bridge:hardware_node"
                teleop_canon = f"@@{canonical}//talOS/teleop:node"
                studio_canon = f"@@{canonical}//studio/bridge:studio_bridge"
                proc = self.assemble(
                    tmp, {"c/robot.toml": "[robot]\n"},
                    [node(tmp, "hardware_node", hw_canon),
                     node(tmp, "teleop", teleop_canon)],
                    [f"@{apparent}//talOS/teleop:node"],
                    robot={"hardware_node":
                           f"@{apparent}//talOS/bridge:hardware_node"},
                    extra=("--repo-mapping", repo_mapping,
                           "--apparent-repo", f"{apparent}={canonical}"),
                    studio={"bridge_path": bridge,
                            "bridge_label": studio_canon,
                            "webroot_path": webroot},
                    effective={"studio": {
                        "bridge":
                        f"@{apparent}//studio/bridge:studio_bridge"}})
                self.assertEqual(proc.returncode, 0, proc.stderr)
                with open(os.path.join(tmp, "out", "bundle_manifest.toml"),
                          "rb") as f:
                    manifest = tomllib.load(f)
                by_name = {n["name"]: n["targets"] for n in manifest["nodes"]}
                self.assertEqual(
                    by_name["hardware_node"],
                    [hw_canon, f"@{apparent}//talOS/bridge:hardware_node"])
                self.assertEqual(
                    by_name["teleop"],
                    [teleop_canon, f"@{apparent}//talOS/teleop:node"])
                self.assertEqual(
                    manifest["studio"]["targets"],
                    [studio_canon, f"@{apparent}//studio/bridge:studio_bridge"])

        # Canonical main-repo `@@//pkg:node` normalizes to `//pkg:node` cleanly.
        with tempfile.TemporaryDirectory() as tmp:
            proc = self.assemble(
                tmp, {"c/robot.toml": "[robot]\n"},
                [node(tmp, "hardware_node", HARDWARE_NODE),
                 node(tmp, "arm", "@@//arm:node")],
                ["//arm:node"])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            with open(os.path.join(tmp, "out", "bundle_manifest.toml"),
                      "rb") as f:
                manifest = tomllib.load(f)
            by_name = {n["name"]: n["targets"] for n in manifest["nodes"]}
            self.assertEqual(by_name["arm"], ["@@//arm:node", "//arm:node"])

    # REQ-DEV-30
    def test_launcher_runner_derives_main_repo_without_hardcoding_main(self):
        runner_path = runfiles.Create().Rlocation(
            "_main/talOS/build/launcher_runner.py")
        spec = importlib.util.spec_from_file_location(
            "launcher_runner", runner_path)
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        with tempfile.TemporaryDirectory() as tmp:
            root = os.path.join(tmp, "runfiles")
            put(os.path.join(root, "_repo_mapping"),
                ",my_robot,custom_main\n"
                ",my_talos,talos+\n"
                "+_repo_rules+toml,talos,_main\n"
                "talos+,rules_cc,rules_cc+\n"
                "talos+,transitive_plain,transitive_plain\n")
            put(os.path.join(root, "custom_main", "robot.toml"), "[robot]\n")
            launcher = os.path.join(
                root, "talos+", "talOS", "launcher", "launcher")
            put(launcher, "")
            self.assertEqual(
                runner.split_config("_main/robot.toml", root),
                ("_main", "robot.toml"))
            self.assertEqual(
                runner.split_config("custom_main/robot.toml", root),
                ("custom_main", "robot.toml"))
            for bad in ("talos+/bench/robot.toml",
                        "rules_cc+/bench/robot.toml",
                        "legacy~dep/bench/robot.toml",
                        "transitive_plain/bench/robot.toml"):
                with self.subTest(bad=bad):
                    with self.assertRaises(SystemExit) as caught:
                        runner.split_config(bad, root)
                    self.assertIn("not in the main repository",
                                  str(caught.exception))
            old_rf = os.environ.get("RUNFILES_DIR")
            os.environ["RUNFILES_DIR"] = root
            try:
                work_dir, config, env = runner.workspace(
                    os.path.join(tmp, "work"), launcher,
                    "custom_main/robot.toml")
                # Guard against runfiles.Create() returning None.
                orig_create = runner.runfiles.Create
                runner.runfiles.Create = lambda: None
                try:
                    work_dir2, config2, env2 = runner.workspace(
                        os.path.join(tmp, "work2"), launcher,
                        "custom_main/robot.toml")
                finally:
                    runner.runfiles.Create = orig_create
            finally:
                if old_rf is None:
                    os.environ.pop("RUNFILES_DIR", None)
                else:
                    os.environ["RUNFILES_DIR"] = old_rf
            self.assertEqual(config, "robot.toml")
            self.assertEqual(env["RUNFILES_DIR"], root)
            self.assertTrue(os.path.isfile(os.path.join(work_dir, "robot.toml")))
            self.assertTrue(os.path.isdir(
                os.path.join(work_dir, "bazel-bin", "external", "talos+")))
            self.assertEqual(config2, "robot.toml")
            self.assertEqual(env2["RUNFILES_DIR"], root)
            self.assertTrue(os.path.isfile(os.path.join(work_dir2, "robot.toml")))


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
            self.assertNotIn("talos", summary)

    # REQ-OPS-10 (REPO-4): deploy.sh logs surfaces talos_git_revision when
    # present in a session's manifest.json and omits it otherwise.
    def test_logs_reports_talos_revision_when_present(self):
        second = 1_000_000_000
        with tempfile.TemporaryDirectory() as tmp:
            self.session(
                tmp, "dual", 1 * second, start_ns=1_700_000_100 * second,
                git_revision="1111222233334444", git_dirty=False,
                talos_git_revision="abcdef0123456789", talos_git_dirty=True)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            header = next(line for line in proc.stdout.splitlines()
                          if line.startswith("session dual"))
            self.assertIn("revision 111122223333 talos abcdef012345 dirty",
                          header)

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

    def test_logs_with_an_empty_log_root_lists_no_session(self):
        # A fresh install's log root exists before its first session: the
        # status command succeeds and names no session or manifest in it.
        with tempfile.TemporaryDirectory() as tmp:
            root = os.path.join(tmp, "logs")
            os.makedirs(root)
            proc = self.deploy(tmp, "logs")
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertNotIn(root + os.sep, proc.stdout)

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

    # REQ-OPS-53
    def test_install_refuses_a_testonly_bundle_outside_a_test(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            def stub(name, manifest):
                path = os.path.join(tmp, name)
                put(os.path.join(path, "run.sh"), "#!/bin/sh\n")
                os.chmod(os.path.join(path, "run.sh"), 0o755)
                put(os.path.join(path, "bundle_manifest.toml"), manifest)
                return path

            header = 'bundle_version = 1\nconfig_path = "config/robot.toml"\n'
            fixture = stub("fixture", header + "testonly = true\n")
            robot = stub("robot", header)
            # As a person on a robot runs it: no Bazel test around it.
            robot_env = {k: v for k, v in os.environ.items()
                         if k not in ("TEST_TMPDIR", "TEST_TARGET")}
            test_dir = os.path.join(tmp, "x")
            test_env = dict(robot_env, TEST_TMPDIR=test_dir,
                            TEST_TARGET="//talOS/build:bundle_builder_test")
            stale_env = dict(robot_env, TEST_TMPDIR=test_dir)
            inside = os.path.join(test_dir, "prefix")
            for name, bundle, env, prefix, installs in (
                    ("test-only, on a robot", fixture, robot_env,
                     os.path.join(tmp, "robot_prefix"), False),
                    ("ordinary, on a robot", robot, robot_env,
                     os.path.join(tmp, "ordinary_prefix"), True),
                    # A sibling whose name extends TEST_TMPDIR's is outside.
                    ("test-only, beside the test's directory", fixture,
                     test_env, os.path.join(tmp, "xy", "prefix"), False),
                    # A stale TEST_TMPDIR alone is not a Bazel test.
                    ("test-only, TEST_TARGET unset", fixture, stale_env,
                     inside, False),
                    ("test-only, in a test", fixture, test_env, inside,
                     True)):
                with self.subTest(name):
                    proc = subprocess.run(
                        ["bash", os.path.abspath(runfiles.Create().Rlocation(
                            "_main/tools/deploy.sh")),
                         "install", bundle, "--prefix", prefix,
                         "--no-warm-up"],
                        capture_output=True, text=True, env=env)
                    if installs:
                        self.assertEqual(proc.returncode, 0, proc.stderr)
                        self.assertTrue(os.path.isfile(
                            os.path.join(prefix, "current", "run.sh")))
                        continue
                    self.assertNotEqual(proc.returncode, 0, proc.stdout)
                    self.assertIn(bundle, proc.stderr)
                    self.assertIn("testonly", proc.stderr)
                    # Refused before anything is written.
                    self.assertFalse(os.path.lexists(prefix), proc.stdout)

    def selected(self, tmp):
        """A prefix whose `current` selects a stub bundle whose run.sh
        records its pid, TALOS_LOG_DIR and arguments, one a line, in the
        returned file."""
        prefix = os.path.join(tmp, "prefix")
        record = os.path.join(tmp, "ran")
        bundle = os.path.join(prefix, "bundles", "20260101_000000_1")
        put(os.path.join(bundle, "run.sh"),
            "#!/bin/sh\n{ echo \"$$\"; echo \"$TALOS_LOG_DIR\"; "
            "for a in \"$@\"; do echo \"$a\"; done; } > " + record + "\n")
        os.chmod(os.path.join(bundle, "run.sh"), 0o755)
        os.symlink(bundle, os.path.join(prefix, "current"))
        return prefix, record

    def host(self, tmp, system, running=None):
        """A PATH directory faking `uname` as `system` ("Linux" or
        "Darwin") and a boot service (systemd's talos.service, launchd's
        org.talos.robot) running the program `running`, or none."""
        bin_dir = os.path.join(tmp, "fake_host")
        if running:
            systemctl = (
                'case "$*" in *ActiveState*) echo active ;; '
                f'*ExecStart*) echo "{{ path={running} ; argv[]={running} ; '
                'ignore_errors=no }" ;; esac')
            launchctl = (f'printf "system/org.talos.robot = {{\\n\\tstate = '
                         f'running\\n\\tprogram = {running}\\n}}\\n"')
        else:
            systemctl = 'case "$*" in *ActiveState*) echo inactive ;; esac'
            launchctl = "exit 113"
        for name, body in (("uname", f"echo {system}"),
                           ("systemctl", systemctl),
                           ("launchctl", launchctl)):
            put(os.path.join(bin_dir, name), f"#!/bin/sh\n{body}\n")
            os.chmod(os.path.join(bin_dir, name), 0o755)
        return bin_dir

    # REQ-OPS-54
    def test_run_dry_run_takes_the_boot_units_limits_on_linux_only(self):
        for system in ("Linux", "Darwin"):
            with self.subTest(system), tempfile.TemporaryDirectory() as tmp:
                prefix, record = self.selected(tmp)
                # Another prefix's boot service does not refuse this one.
                path = self.host(tmp, system, "/elsewhere/current/run.sh")
                proc = self.deploy(prefix, "run", "--dry-run", "--remote",
                                   "10.0.0.9", path=path)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                lines = [line for line in proc.stdout.splitlines()
                         if line.startswith("+ ")]
                launch = (f"+ exec env TALOS_LOG_DIR={prefix}/logs "
                          f"{prefix}/current/run.sh --remote 10.0.0.9")
                limits = ["+ ulimit -r 49", "+ ulimit -l unlimited"]
                self.assertEqual(
                    lines, (limits if system == "Linux" else []) + [launch],
                    proc.stdout)
                self.assertFalse(os.path.exists(record))

    # REQ-OPS-54
    def test_run_becomes_the_launcher_with_its_arguments(self):
        # The macOS form, which takes no limit, so it runs on any host.
        with tempfile.TemporaryDirectory() as tmp:
            prefix, record = self.selected(tmp)
            env = dict(os.environ, PATH=self.host(tmp, "Darwin") + os.pathsep
                       + os.environ["PATH"])
            # After `--`, even deploy.sh's own flags go to run.sh.
            proc = subprocess.run(
                ["bash", os.path.abspath(runfiles.Create().Rlocation(
                    "_main/tools/deploy.sh")), "run", "--prefix", prefix,
                 "--remote", "10.0.0.9", "--", "--dry-run"],
                capture_output=True, text=True, env=env)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            with open(record, encoding="utf-8") as f:
                pid, log_dir, *args = f.read().splitlines()
            # run.sh, and the launcher it execs, run in deploy.sh's process,
            # whose pid it printed.
            self.assertRegex(proc.stdout, rf"\bpid {pid}\b")
            self.assertEqual(log_dir, os.path.join(prefix, "logs"))
            self.assertIn(log_dir, proc.stdout)
            self.assertEqual(args, ["--remote", "10.0.0.9", "--dry-run"])

    # REQ-OPS-54
    def test_run_refuses_while_the_boot_service_runs_the_same_bundle(self):
        for system in ("Linux", "Darwin"):
            with self.subTest(system), tempfile.TemporaryDirectory() as tmp:
                prefix, record = self.selected(tmp)
                path = self.host(tmp, system,
                                 os.path.join(prefix, "current", "run.sh"))
                proc = self.deploy(prefix, "run", path=path)
                self.assertNotEqual(proc.returncode, 0, proc.stdout)
                self.assertIn(os.path.join(prefix, "current", "run.sh"),
                              proc.stderr)
                self.assertFalse(os.path.exists(record), proc.stdout)

    # REQ-OPS-54
    @unittest.skipUnless(sys.platform.startswith("linux"),
                         "RLIMIT_RTPRIO is Linux's")
    def test_run_on_linux_starts_nothing_without_the_limit_grant(self):
        import resource

        def lowered():
            resource.setrlimit(resource.RLIMIT_RTPRIO, (0, 0))

        with tempfile.TemporaryDirectory() as tmp:
            prefix, record = self.selected(tmp)
            env = dict(os.environ, PATH=self.host(tmp, "Linux") + os.pathsep
                       + os.environ["PATH"])
            proc = subprocess.run(
                ["bash", os.path.abspath(runfiles.Create().Rlocation(
                    "_main/tools/deploy.sh")), "run", "--prefix", prefix],
                capture_output=True, text=True, env=env, preexec_fn=lowered)
            self.assertNotEqual(proc.returncode, 0, proc.stdout)
            self.assertIn("rtprio", proc.stderr)
            self.assertFalse(os.path.exists(record), proc.stdout)

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
