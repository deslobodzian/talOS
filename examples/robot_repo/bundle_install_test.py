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
import tempfile
import tomllib
import unittest

from python.runfiles import runfiles

# Bounds only a hang; each step takes about a second.
STEP_TIMEOUT_S = 50

# What a Bazel test hands its children so they find their runfiles; a
# boot service sets none of them.
RUNFILES_VARIABLES = ("RUNFILES_DIR", "RUNFILES_MANIFEST_FILE",
                      "RUNFILES_MANIFEST_ONLY", "TEST_SRCDIR",
                      "JAVA_RUNFILES", "PYTHON_RUNFILES", "PYTHONPATH",
                      "PYTHONHOME", "VIRTUAL_ENV", "BUILD_WORKSPACE_DIRECTORY",
                      "BUILD_WORKING_DIRECTORY")


def location(path):
    found = runfiles.Create().Rlocation(path)
    if not found or not os.path.exists(found):
        raise FileNotFoundError(f"{path} is not in the runfiles ({found})")
    return found


def build_tree_paths(bundle, deploy):
    """Directories the installed bundle must not name: Bazel's output tree
    and external repositories, this repository's checkout and talOS's."""
    paths = set()
    built = os.path.realpath(bundle)
    if "/execroot/" in built:
        output_base = built.split("/execroot/", 1)[0]
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


class BundleInstallTest(unittest.TestCase):
    def test_installed_bundle_describes_without_the_build(self):
        bundle = location("_main/robot_bundle")
        deploy = location("talos/tools/deploy.sh")
        avoid = build_tree_paths(bundle, deploy)
        env = {k: v for k, v in os.environ.items()
               if k not in RUNFILES_VARIABLES}
        # deploy.sh installs read-only, so cleanup cannot remove it all;
        # Bazel clears TEST_TMPDIR.
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            # The bundle as Bazel wrote it (bazel-bin/robot_bundle), not the
            # sandbox's tree of links into that directory: this is the
            # directory a person installs from.
            built = os.path.dirname(os.path.realpath(
                os.path.join(bundle, "bundle_manifest.toml")))
            staging = os.path.join(tmp, "staging", "robot_bundle")
            shutil.copytree(built, staging, symlinks=True)
            prefix = os.path.join(tmp, "prefix")
            install = subprocess.run(
                ["bash", deploy, "install", staging, "--prefix", prefix],
                capture_output=True, text=True, env=env, cwd=tmp,
                timeout=STEP_TIMEOUT_S)
            installed = install.stdout + install.stderr
            self.assertEqual(install.returncode, 0, installed)
            self.assertNotIn("warning: warm-up", installed)
            remove(os.path.dirname(staging))

            where = os.path.realpath(os.path.join(prefix, "current"))
            for parent, dirs, files in os.walk(where):
                for name in dirs + files:
                    path = os.path.join(parent, name)
                    if os.path.islink(path):
                        target = os.path.realpath(path)
                        self.assertTrue(
                            target == where
                            or target.startswith(where + os.sep),
                            f"{path} links to {os.readlink(path)}, outside "
                            "the install")
                    elif name in files:
                        with open(path, "rb") as f:
                            data = f.read()
                        for avoided in avoid:
                            self.assertNotIn(
                                avoided.encode(), data,
                                f"{path} names {avoided}, which the robot "
                                "does not have")

            logs = os.path.join(tmp, "logs")
            proc = subprocess.run(
                [os.path.join(prefix, "current", "run.sh"), "--describe-only"],
                capture_output=True, text=True, cwd=tmp,
                env=dict(env, TALOS_LOG_DIR=logs), timeout=STEP_TIMEOUT_S)
            out = proc.stdout + proc.stderr
            self.assertEqual(proc.returncode, 0, out)
            (session,) = os.listdir(logs)
            with open(os.path.join(logs, session, "graph.json"),
                      encoding="utf-8") as f:
                graph = json.load(f)
            with open(os.path.join(where, "bundle_manifest.toml"), "rb") as f:
                manifest = tomllib.load(f)
            names = sorted(node["name"] for node in graph["nodes"])
            # The config's nodes and the hardware bridge, as the bundle
            # holds them, and its Studio bridge once its talos_bundle names
            # one (README.md, "Studio").
            studio = ["studio_bridge"] if "studio" in manifest else []
            self.assertEqual(len(names),
                             len(manifest_nodes()) + 1 + len(studio), out)
            self.assertEqual(names, sorted(
                [node["name"] for node in manifest["nodes"]] + studio), out)
            for node in graph["nodes"]:
                self.assertTrue(
                    node.get("described"),
                    f"{node.get('name')}: {node.get('describe_error')}\n{out}")
            self.assertEqual(graph["config_sha256"],
                             manifest["config_sha256"], out)


if __name__ == "__main__":
    unittest.main()
