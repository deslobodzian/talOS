"""This repository builds talOS the way talOS builds itself.

REQ-DEV-30: a robot repository that depends on talOS through @talos builds
and tests it. Three things that decide how talOS's code compiles and runs
belong to the workspace, not to the @talos module, so a dependent repository
names them itself; each must agree with the talOS that MODULE.bazel's
override makes @talos:

- `.bazelrc` imports that talOS's `talos.bazelrc` (C++23 for target and
  exec configuration, the Python runfiles flags, libjpeg-turbo's `noasm`,
  the sanitizer, release and lane configs): `import <talOS>/talos.bazelrc`,
  the same checkout `local_path_override`'s `path` names. Where @talos is
  Bazel's fetched copy (`git_override`), there is no checkout to import from,
  and the imported file must hold the same text as that talOS's;
- `.bazelversion` is talOS's: another Bazel can resolve the module graph
  differently;
- the default Python toolchain (`python.toolchain(is_default = True)`) is
  talOS's version: the root module picks the interpreter, and talOS's pip
  wheels are built for its own.

The truth is the talOS @talos is built from, found through its MODULE.bazel
in this test's runfiles; reading it does not run Bazel. Bazel does not see
that checkout's `talos.bazelrc` and `.bazelversion` as inputs, so the target
is tagged `external` (never cached). Every `--workspace_status_command` the
`.bazelrc` names must also exist, so `--config=release` can stamp a bundle.
"""

import os
import re
import shlex
import unittest

from python.runfiles import runfiles


def location(path):
    found = runfiles.Create().Rlocation(path)
    if not found or not os.path.exists(found):
        raise FileNotFoundError(f"{path} is not in the runfiles ({found})")
    return found


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def default_python(module):
    """python_version of the module's `python.toolchain(...)` call that sets
    is_default = True, or None."""
    for call in re.finditer(r"python\.toolchain\(([^)]*)\)", module):
        if re.search(r"is_default\s*=\s*True", call[1]):
            version = re.search(r'python_version\s*=\s*"([^"]+)"', call[1])
            return version and version[1]
    return None


def rc_lines(text):
    """The .bazelrc's lines as argument lists, comments and blanks left out."""
    for line in text.splitlines():
        if line.strip() and not line.lstrip().startswith("#"):
            yield shlex.split(line, comments=True)


class TalosPinTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # This repository's files, through the runfiles links to its
        # checkout; %workspace% is the directory they are in.
        cls.bazelrc = os.path.realpath(location("_main/.bazelrc"))
        cls.workspace = os.path.dirname(cls.bazelrc)
        # The talOS @talos is built from: a checkout (local_path_override),
        # or Bazel's copy under the output base's external/ (git_override).
        cls.talos = os.path.dirname(
            os.path.realpath(location("talos/MODULE.bazel")))
        cls.fetched = os.path.basename(os.path.dirname(cls.talos)) == \
            "external"

    def resolve(self, path):
        """A .bazelrc path, %workspace% replaced, made absolute."""
        path = path.replace("%workspace%", self.workspace)
        return os.path.realpath(os.path.join(self.workspace, path))

    def test_bazelrc_imports_the_talos_bazelrc_of_the_pinned_talos(self):
        expected = os.path.join(self.talos, "talos.bazelrc")
        self.assertTrue(os.path.isfile(expected),
                        f"{self.talos} (@talos) has no talos.bazelrc")
        imports = [self.resolve(args[1])
                   for args in rc_lines(read(self.bazelrc))
                   if args[0] == "import" and len(args) == 2]
        if self.fetched:
            # No checkout to point at: the file imported is a copy.
            matching = [p for p in imports
                        if os.path.isfile(p) and read(p) == read(expected)]
        else:
            matching = [p for p in imports if p == expected]
        self.assertTrue(
            matching,
            f"{self.bazelrc} imports {imports or 'nothing'}; it must "
            f"`import` {expected}, talOS's flags from the checkout "
            "MODULE.bazel's local_path_override names (a copy of it when "
            "@talos is fetched by git_override). try-import does not count: "
            "it skips a missing file without a word")

    def test_bazel_version_is_talos_own(self):
        ours = read(location("_main/.bazelversion")).strip()
        talos = read(os.path.join(self.talos, ".bazelversion")).strip()
        self.assertEqual(ours, talos,
                         f".bazelversion is {ours}; talOS at {self.talos} "
                         f"is built with Bazel {talos}")

    def test_default_python_is_talos_own(self):
        ours = default_python(read(location("_main/MODULE.bazel")))
        talos = default_python(read(location("talos/MODULE.bazel")))
        self.assertIsNotNone(talos, f"{self.talos}/MODULE.bazel names no "
                             "default Python toolchain")
        self.assertEqual(ours, talos,
                         "MODULE.bazel's python.toolchain(is_default = True) "
                         f"is {ours}; talOS's Python nodes and wheels are "
                         f"built for {talos}")

    def test_workspace_status_command_exists(self):
        for args in rc_lines(read(self.bazelrc)):
            for arg in args[1:]:
                if arg.startswith("--workspace_status_command="):
                    command = self.resolve(arg.split("=", 1)[1])
                    self.assertTrue(os.access(command, os.X_OK),
                                    f"{arg}: {command} is not an executable "
                                    "file")


if __name__ == "__main__":
    unittest.main()
