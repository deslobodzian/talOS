"""This repository's bundle installs and runs away from the build that made it.

REQ-DEV-30, REQ-OPS-47, REQ-OPS-50: the bundle a repository depending on
talOS builds (`//:robot_bundle`, its nodes under @talos's external
repository name and this repository's) is installed the way the robot gets
it, with talOS's `deploy.sh install` into a fresh prefix:

- install exits 0 without a warm-up warning: every member ran once from its
  installed path, and the bundle's own launcher accepted the graph
  (`run.sh --describe-only`, which install runs before selecting it);
- the installed `run.sh --describe-only` describes every node the manifest
  launches, plus the hardware bridge (and the Studio bridge, when the
  bundle carries Studio), from the install;
- nothing in the install leads back to the build: it is installed from a
  copy of the bundle that is removed before the describe, which runs with no
  Bazel runfiles variables in its environment, and no link in the install
  resolves outside it, nor does any file name Bazel's output base, this
  repository's checkout or talOS's.

About 2 s: two copies of the bundle, the install's warm-up and graph check,
and one more describe.
"""

import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

# Bounds only a hang; each step takes about a second.
STEP_TIMEOUT_S = 50

# What a Bazel test or developer shell hands its children that a boot service
# never sets: runfiles, Python environment, and talOS session overrides.
# Dynamic linker search/preload variables (`LD_*`, `DYLD_*`) are also stripped.
RUNFILES_VARIABLES = ("RUNFILES_DIR", "RUNFILES_MANIFEST_FILE",
                      "RUNFILES_MANIFEST_ONLY", "TEST_SRCDIR",
                      "TEST_WORKSPACE", "JAVA_RUNFILES", "PYTHON_RUNFILES",
                      "PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE",
                      "VIRTUAL_ENV", "BUILD_WORKSPACE_DIRECTORY",
                      "BUILD_WORKING_DIRECTORY", "TALOS_SHM_NAMESPACE",
                      "TALOS_LOG_DIR")


def clean_env():
    """Environment stripped of Bazel runfiles, Python, linker, and talOS
    session variables so the installed bundle cannot resolve anything from the
    build tree or host overrides."""
    return {k: v for k, v in os.environ.items()
            if k not in RUNFILES_VARIABLES
            and not k.startswith(("LD_", "DYLD_"))}


def location(path):
    found = runfiles.Create().Rlocation(path)
    if not found or not os.path.exists(found):
        raise FileNotFoundError(f"{path} is not in the runfiles ({found})")
    return found


def build_tree_paths(bundle, deploy):
    """Directories the installed bundle must not name: Bazel's output tree
    and external repositories, this repository's checkout and talOS's."""
    paths = set()
    # Resolve a file inside the bundle, not the bundle directory itself: inside
    # Bazel's Linux sandbox the directory is a real sandbox directory holding
    # symlinks to the output base, so realpath(bundle) stays in sandbox/ instead
    # of reaching the real output_base/execroot/.
    manifest = os.path.join(bundle, "bundle_manifest.toml")
    for candidate in (
        os.path.dirname(os.path.realpath(manifest)),
        os.path.realpath(bundle),
    ):
        if "/execroot/" in candidate:
            output_base = candidate.split("/execroot/", 1)[0]
            paths |= {output_base + "/execroot/", output_base + "/external/"}
    paths.add(os.path.dirname(os.path.realpath(
        location("_main/bundle_install_test.py"))) + "/")
    paths.add(os.path.dirname(os.path.dirname(os.path.realpath(deploy))) + "/")
    return sorted(paths)


def manifest_nodes():
    """NODES of robot_manifest.bzl: what robot.toml launches."""
    with open(location("_main/robot_manifest.bzl"), encoding="utf-8") as f:
        module = ast.parse(f.read())
    for statement in module.body:
        if (isinstance(statement, ast.Assign)
                and [t.id for t in statement.targets] == ["NODES"]):
            return ast.literal_eval(statement.value)
    raise AssertionError("robot_manifest.bzl has no NODES")


def remove(tree):
    """Removes a copied Bazel output, whose directories are read-only."""
    for parent, _, _ in os.walk(tree):
        os.chmod(parent, 0o755)
    shutil.rmtree(tree)


def _copy_tree_best_effort(src, dst):
    if not os.path.isdir(src):
        return
    try:
        os.makedirs(dst, exist_ok=True)
    except OSError:
        return
    for root, dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        target_root = dst if rel == "." else os.path.join(dst, rel)
        for name in dirs:
            try:
                os.makedirs(os.path.join(target_root, name), exist_ok=True)
            except OSError:
                pass
        for name in files:
            src_path = os.path.join(root, name)
            dst_path = os.path.join(target_root, name)
            if os.path.islink(src_path) and not os.path.exists(src_path):
                continue
            try:
                os.makedirs(os.path.dirname(dst_path), exist_ok=True)
                shutil.copyfile(src_path, dst_path)
            except OSError:
                pass


def preserve_failure_artifacts(logs_dir, install_output="",
                               describe_output="",
                               subdir="bundle_install_failure"):
    """Best-effort copy of command output and session logs into
    `TEST_UNDECLARED_OUTPUTS_DIR` on failure (docs/TESTING.md §8.2). Never
    raises so copy errors cannot mask the test failure."""
    outputs_dir = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")
    if not outputs_dir:
        return None
    has_logs = False
    if logs_dir and os.path.isdir(logs_dir):
        try:
            has_logs = bool(os.listdir(logs_dir))
        except OSError:
            has_logs = False
    if not has_logs and not install_output and not describe_output:
        return None
    try:
        os.makedirs(outputs_dir, exist_ok=True)
        dest = None
        for index in range(1, 1000):
            name = subdir if index == 1 else f"{subdir}_{index}"
            candidate = os.path.join(outputs_dir, name)
            try:
                os.makedirs(candidate, exist_ok=False)
                dest = candidate
                break
            except FileExistsError:
                continue
        if dest is None:
            dest = tempfile.mkdtemp(prefix=f"{subdir}_", dir=outputs_dir)
    except OSError:
        return None
    sections = []
    if install_output:
        sections.append(f"=== deploy.sh install ===\n{install_output}")
    if describe_output:
        sections.append(f"=== run.sh --describe-only ===\n{describe_output}")
    if sections:
        try:
            with open(os.path.join(dest, "launch.log"), "w",
                      encoding="utf-8", errors="replace") as f:
                f.write("\n".join(sections))
        except OSError:
            pass
    if has_logs:
        _copy_tree_best_effort(logs_dir, os.path.join(dest, "sessions"))
    return dest


class BundleInstallTest(unittest.TestCase):
    def test_installed_bundle_describes_without_the_build(self):
        bundle = location("_main/robot_bundle")
        deploy = location("talos/tools/deploy.sh")
        avoid = build_tree_paths(bundle, deploy)
        env = clean_env()
        # Keep scratch directories inside TEST_TMPDIR and restore write bits on
        # deploy.sh's read-only install tree before teardown so nothing leaks.
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR"),
                ignore_cleanup_errors=True) as tmp:
            logs = os.path.join(tmp, "logs")
            staging_root = os.path.join(tmp, "staging")
            prefix = os.path.join(tmp, "prefix")
            step_env = dict(env, TMPDIR=tmp)
            installed = ""
            out = ""
            try:
                # The bundle as Bazel wrote it (bazel-bin/robot_bundle), not
                # the sandbox's tree of links into that directory: this is the
                # directory a person installs from.
                built = os.path.dirname(os.path.realpath(
                    os.path.join(bundle, "bundle_manifest.toml")))
                staging = os.path.join(staging_root, "robot_bundle")
                shutil.copytree(built, staging, symlinks=True)
                install = subprocess.run(
                    ["bash", deploy, "install", staging, "--prefix", prefix],
                    capture_output=True, text=True, env=step_env, cwd=tmp,
                    timeout=STEP_TIMEOUT_S)
                installed = install.stdout + install.stderr
                self.assertEqual(install.returncode, 0, installed)
                self.assertNotIn("warning: warm-up", installed)
                remove(staging_root)

                where = os.path.realpath(os.path.join(prefix, "current"))
                for parent, dirs, files in os.walk(where):
                    for name in dirs + files:
                        path = os.path.join(parent, name)
                        if os.path.islink(path):
                            self.assertTrue(
                                os.path.exists(path),
                                f"{path} is a dangling link to "
                                f"{os.readlink(path)}")
                            target = os.path.realpath(path)
                            self.assertTrue(
                                target == where
                                or target.startswith(where + os.sep),
                                f"{path} links to {os.readlink(path)}, "
                                "outside the install")
                        elif name in files:
                            with open(path, "rb") as f:
                                data = f.read()
                            for avoided in avoid:
                                self.assertNotIn(
                                    avoided.encode(), data,
                                    f"{path} names {avoided}, which the robot "
                                    "does not have")

                proc = subprocess.run(
                    [os.path.join(prefix, "current", "run.sh"),
                     "--describe-only"],
                    capture_output=True, text=True, cwd=tmp,
                    env=dict(step_env, TALOS_LOG_DIR=logs),
                    timeout=STEP_TIMEOUT_S)
                out = proc.stdout + proc.stderr
                self.assertEqual(proc.returncode, 0, out)
                (session,) = os.listdir(logs)
                with open(os.path.join(logs, session, "graph.json"),
                          encoding="utf-8") as f:
                    graph = json.load(f)
                with open(os.path.join(where, "bundle_manifest.toml"),
                          "rb") as f:
                    manifest = tomllib.load(f)
                names = sorted(node["name"] for node in graph["nodes"])
                # The config's nodes and the hardware bridge, as the bundle
                # holds them, and its Studio bridge once its talos_bundle names
                # one (README.md, "Studio").
                studio = ["studio_bridge"] if "studio" in manifest else []
                self.assertEqual(len(names),
                                 len(manifest_nodes()) + 1 + len(studio), out)
                self.assertEqual(names, sorted(
                    [node["name"] for node in manifest["nodes"]] + studio),
                    out)
                for node in graph["nodes"]:
                    self.assertTrue(
                        node.get("described"),
                        f"{node.get('name')}: "
                        f"{node.get('describe_error')}\n{out}")
                self.assertEqual(graph["config_sha256"],
                                 manifest["config_sha256"], out)
            except BaseException:
                kept = preserve_failure_artifacts(logs, installed, out)
                if kept:
                    print(f"bundle_install_test: failure artifacts preserved: "
                          f"{kept}", file=sys.stderr)
                raise
            finally:
                for tree in (staging_root, prefix):
                    if os.path.isdir(tree):
                        try:
                            remove(tree)
                        except OSError:
                            pass


if __name__ == "__main__":
    unittest.main()
