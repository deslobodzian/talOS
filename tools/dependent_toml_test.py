"""Contract tests for REQ-DEV-29: patched toml++ in dependent repositories.

REQ-DEV-29: a build that uses talOS as a dependency compiles talOS's patched
toml++; the consuming repository carries no override or patch file for it.

Bazel ignores module overrides (`single_version_override`, `archive_override`,
etc.) declared in non-root modules, so talOS must fetch `tomlplusplus` through
its own non-dev `http_archive` repository rule with
`//third_party:tomlplusplus_parser_ub.patch` applied, export it publicly via
`//third_party:tomlplusplus`, keep every consuming repository
(`examples/robot_repo`, `examples/wrist_repo`, and standalone scaffolds) free
of toml++ overrides or patch files, and test
`@talos//talOS/configuration:config_fuzz_test` in both dependents in CI and
`tools/prepush.sh`.
"""

import ast
import os
import re
import tempfile
import unittest

from tools import new_subsystem

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

OVERRIDE_CALLS = frozenset({
    "single_version_override",
    "archive_override",
    "git_override",
    "local_path_override",
    "multiple_version_override",
})
TOML_PATCH = "//third_party:tomlplusplus_parser_ub.patch"
TOML_BUILD_FILE = "//third_party:tomlplusplus.BUILD.bazel"
CONFIG_FUZZ_TARGET = "@talos//talOS/configuration:config_fuzz_test"


def _kwarg(call, name):
    for kw in call.keywords:
        if kw.arg == name:
            return kw.value
    return None


def _literal_str(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _literal_str_list(node):
    if not isinstance(node, (ast.List, ast.Tuple)):
        return None
    out = []
    for elt in node.elts:
        val = _literal_str(elt)
        if val is None:
            return None
        out.append(val)
    return out


def _literal_bool(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, bool):
        return node.value
    return None


def check_talos_module(text):
    """Returns a list of REQ-DEV-29 violations in talOS's root MODULE.bazel."""
    errors = []
    tree = ast.parse(text, filename="MODULE.bazel")

    # Track non-dev `http_archive` bindings from `use_repo_rule`.
    http_archive_rules = {}
    for stmt in tree.body:
        if (isinstance(stmt, ast.Assign) and len(stmt.targets) == 1
                and isinstance(stmt.targets[0], ast.Name)
                and isinstance(stmt.value, ast.Call)
                and isinstance(stmt.value.func, ast.Name)
                and stmt.value.func.id == "use_repo_rule"):
            call = stmt.value
            args = [_literal_str(a) for a in call.args]
            if (len(args) == 2
                    and args[0] == "@bazel_tools//tools/build_defs/repo:http.bzl"
                    and args[1] == "http_archive"):
                is_dev = _literal_bool(_kwarg(call, "dev_dependency")) is True
                http_archive_rules[stmt.targets[0].id] = is_dev

    found_archive = False
    for stmt in tree.body:
        if not (isinstance(stmt, ast.Expr)
                and isinstance(stmt.value, ast.Call)
                and isinstance(stmt.value.func, ast.Name)):
            continue
        call = stmt.value
        func = call.func.id
        if func == "bazel_dep":
            dep_name = _literal_str(_kwarg(call, "name"))
            if dep_name is None and call.args:
                dep_name = _literal_str(call.args[0])
            if dep_name == "tomlplusplus":
                errors.append(
                    "tomlplusplus must not be a bazel_dep (root-only overrides "
                    "do not reach dependent repositories)")
        elif func in OVERRIDE_CALLS:
            mod_name = _literal_str(_kwarg(call, "module_name"))
            if mod_name is None and call.args:
                mod_name = _literal_str(call.args[0])
            if mod_name == "tomlplusplus":
                errors.append(
                    f"tomlplusplus must not use root-only {func}")
        elif func in http_archive_rules:
            if _literal_str(_kwarg(call, "name")) == "tomlplusplus":
                found_archive = True
                if http_archive_rules[func]:
                    errors.append(
                        f"{func} for tomlplusplus must not be dev_dependency=True")
                if _literal_bool(_kwarg(call, "dev_dependency")) is True:
                    errors.append(
                        "tomlplusplus http_archive call must not be "
                        "dev_dependency=True")
                patches = _literal_str_list(_kwarg(call, "patches")) or []
                if TOML_PATCH not in patches:
                    errors.append(
                        f"tomlplusplus http_archive is missing patch "
                        f"{TOML_PATCH}: {patches}")
                build_file = _literal_str(_kwarg(call, "build_file"))
                if build_file != TOML_BUILD_FILE:
                    errors.append(
                        f"tomlplusplus http_archive build_file is "
                        f"{build_file!r}, want {TOML_BUILD_FILE!r}")

    if not found_archive:
        errors.append(
            "MODULE.bazel does not declare tomlplusplus via http_archive "
            "use_repo_rule")
    return errors


def check_dependent_module(text, filename="MODULE.bazel"):
    """Returns a list of REQ-DEV-29 violations in a consuming repo's MODULE.bazel."""
    errors = []
    tree = ast.parse(text, filename=filename)
    depends_on_talos = False
    for stmt in tree.body:
        if not (isinstance(stmt, ast.Expr)
                and isinstance(stmt.value, ast.Call)
                and isinstance(stmt.value.func, ast.Name)):
            continue
        call = stmt.value
        func = call.func.id
        name = _literal_str(_kwarg(call, "name"))
        if name is None and call.args:
            name = _literal_str(call.args[0])
        mod_name = _literal_str(_kwarg(call, "module_name"))
        if func == "bazel_dep" and name == "talos":
            depends_on_talos = True
        for val in (name, mod_name):
            if val and "toml" in val.lower():
                errors.append(
                    f"{filename}: carries {func} for {val!r}; dependent "
                    "repositories must inherit talOS's patched toml++")
        patches = _literal_str_list(_kwarg(call, "patches")) or []
        for patch in patches:
            if "toml" in patch.lower():
                errors.append(
                    f"{filename}: carries toml++ patch {patch!r} in {func}")
    if not depends_on_talos:
        errors.append(f"{filename}: missing bazel_dep(name = 'talos')")
    if re.search(r"tomlplusplus|toml\+\+", text, re.IGNORECASE):
        errors.append(
            f"{filename}: mentions toml++/tomlplusplus; dependent repositories "
            "carry no override or patch for it")
    return errors


def find_toml_patch_files(root):
    """Returns any patch/diff files for toml++ inside a dependent repository."""
    found = []
    for current, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if not d.startswith("bazel-")]
        for name in files:
            low = name.lower()
            if low.endswith((".patch", ".diff")) and "toml" in low:
                found.append(
                    os.path.relpath(os.path.join(current, name), root))
    return sorted(found)


class DependentTomlTest(unittest.TestCase):
    # REQ-DEV-29
    def test_validator_rejects_root_only_overrides_and_dependent_patches(self):
        # Proof that the checks below fail closed on every regression mode.
        root_override = (
            'bazel_dep(name = "tomlplusplus", version = "3.4.0")\n'
            'single_version_override(\n'
            '    module_name = "tomlplusplus",\n'
            f'    patches = ["{TOML_PATCH}"],\n'
            ')\n'
        )
        errs = check_talos_module(root_override)
        self.assertTrue(
            any("bazel_dep" in e for e in errs)
            and any("single_version_override" in e for e in errs),
            errs)

        dev_only_archive = (
            'archive = use_repo_rule(\n'
            '    "@bazel_tools//tools/build_defs/repo:http.bzl",\n'
            '    "http_archive",\n'
            '    dev_dependency = True,\n'
            ')\n'
            'archive(\n'
            '    name = "tomlplusplus",\n'
            f'    build_file = "{TOML_BUILD_FILE}",\n'
            f'    patches = ["{TOML_PATCH}"],\n'
            ')\n'
        )
        self.assertTrue(
            any("dev_dependency" in e
                for e in check_talos_module(dev_only_archive)))

        unpatched_archive = (
            'archive = use_repo_rule(\n'
            '    "@bazel_tools//tools/build_defs/repo:http.bzl",\n'
            '    "http_archive",\n'
            ')\n'
            'archive(\n'
            '    name = "tomlplusplus",\n'
            f'    build_file = "{TOML_BUILD_FILE}",\n'
            ')\n'
        )
        self.assertTrue(
            any("missing patch" in e
                for e in check_talos_module(unpatched_archive)))

        dependent_with_override = (
            'module(name = "robot")\n'
            'bazel_dep(name = "talos", version = "0.1.0")\n'
            'single_version_override(\n'
            '    module_name = "tomlplusplus",\n'
            '    patches = ["//:toml.patch"],\n'
            ')\n'
        )
        dep_errs = check_dependent_module(dependent_with_override)
        self.assertTrue(
            any("single_version_override" in e for e in dep_errs), dep_errs)

    # REQ-DEV-29
    def test_talos_fetches_patched_tomlplusplus_via_module_http_archive(self):
        with open(os.path.join(REPO_ROOT, "MODULE.bazel"),
                  encoding="utf-8") as f:
            module_text = f.read()
        self.assertEqual(check_talos_module(module_text), [])
        self.assertTrue(os.path.isfile(
            os.path.join(REPO_ROOT, "third_party",
                         "tomlplusplus_parser_ub.patch")))
        self.assertTrue(os.path.isfile(
            os.path.join(REPO_ROOT, "third_party", "tomlplusplus.BUILD.bazel")))

        # third_party/BUILD exports @talos//third_party:tomlplusplus publicly
        # so dependent repositories compile against talOS's patched archive.
        with open(os.path.join(REPO_ROOT, "third_party", "BUILD"),
                  encoding="utf-8") as f:
            build_tree = ast.parse(f.read(), filename="third_party/BUILD")
        default_public = False
        found_alias = False
        alias_public = False
        for stmt in build_tree.body:
            if not (isinstance(stmt, ast.Expr)
                    and isinstance(stmt.value, ast.Call)
                    and isinstance(stmt.value.func, ast.Name)):
                continue
            call = stmt.value
            if call.func.id == "package":
                vis = _literal_str_list(_kwarg(call, "default_visibility")) or []
                if "//visibility:public" in vis:
                    default_public = True
            elif (call.func.id == "alias"
                  and _literal_str(_kwarg(call, "name")) == "tomlplusplus"):
                self.assertEqual(
                    _literal_str(_kwarg(call, "actual")),
                    "@tomlplusplus//:toml++")
                vis = _literal_str_list(_kwarg(call, "visibility"))
                alias_public = (
                    "//visibility:public" in vis if vis is not None
                    else default_public)
                found_alias = True
        self.assertTrue(
            found_alias and alias_public,
            "third_party/BUILD must publicly expose alias(name = "
            "'tomlplusplus', actual = '@tomlplusplus//:toml++')")

    # REQ-DEV-29
    def test_dependent_repositories_carry_no_tomlplusplus_override_or_patch(
            self):
        for rel in ("examples/robot_repo", "examples/wrist_repo"):
            dep_root = os.path.join(REPO_ROOT, rel)
            module_path = os.path.join(dep_root, "MODULE.bazel")
            with self.subTest(dependent=rel):
                with open(module_path, encoding="utf-8") as f:
                    text = f.read()
                self.assertEqual(
                    check_dependent_module(text, filename=f"{rel}/MODULE.bazel"),
                    [])
                self.assertEqual(find_toml_patch_files(dep_root), [])

        # examples/robot_repo/talos_checks compiles directly against
        # @talos//third_party:tomlplusplus.
        checks_build = os.path.join(
            REPO_ROOT, "examples", "robot_repo", "talos_checks", "BUILD.bazel")
        with open(checks_build, encoding="utf-8") as f:
            checks_tree = ast.parse(f.read(), filename=checks_build)
        found_compile_check = False
        for stmt in checks_tree.body:
            if (isinstance(stmt, ast.Expr)
                    and isinstance(stmt.value, ast.Call)
                    and isinstance(stmt.value.func, ast.Name)
                    and stmt.value.func.id == "cc_library"
                    and _literal_str(_kwarg(stmt.value, "name"))
                    == "tomlplusplus_compile_check"):
                deps = _literal_str_list(_kwarg(stmt.value, "deps")) or []
                self.assertIn("@talos//third_party:tomlplusplus", deps)
                found_compile_check = True
        self.assertTrue(found_compile_check)

        # Standalone scaffolds generated by new_subsystem also carry no
        # toml++ override or patch file.
        for lang in ("cpp", "py"):
            with self.subTest(standalone_lang=lang), \
                    tempfile.TemporaryDirectory() as tmp:
                target = os.path.join(tmp, "arm_repo")
                writes = new_subsystem.plan_standalone(
                    target, "arm", lang, REPO_ROOT)
                by_rel = {
                    os.path.relpath(path, target).replace(os.sep, "/"): content
                    for path, _, content in writes
                }
                self.assertEqual(
                    check_dependent_module(
                        by_rel["MODULE.bazel"],
                        filename=f"standalone_{lang}/MODULE.bazel"),
                    [])
                for path in by_rel:
                    self.assertFalse(
                        path.lower().endswith((".patch", ".diff"))
                        and "toml" in path.lower(),
                        path)

    # REQ-DEV-29
    def test_ci_and_prepush_test_config_fuzz_test_in_dependent_repositories(
            self):
        # CI's robot-repo job runs @talos//talOS/configuration:config_fuzz_test
        # in both examples/robot_repo and examples/wrist_repo.
        ci_path = os.path.join(
            REPO_ROOT, ".github", "workflows", "continuous-integration.yaml")
        with open(ci_path, encoding="utf-8") as f:
            ci = f.read()
        for repo in ("examples/robot_repo", "examples/wrist_repo"):
            with self.subTest(ci_dependent=repo):
                pattern = (
                    rf"working-directory:\s*{re.escape(repo)}\n"
                    r"(?:[ \t]+.*\n)*?"
                    rf"[ \t]+.*{re.escape(CONFIG_FUZZ_TARGET)}"
                )
                self.assertRegex(
                    ci, pattern,
                    f"CI robot-repo job does not run {CONFIG_FUZZ_TARGET} in "
                    f"{repo}")

        # tools/prepush.sh runs the same target in both dependent repositories.
        with open(os.path.join(REPO_ROOT, "tools", "prepush.sh"),
                  encoding="utf-8") as f:
            prepush = f.read()
        self.assertIn("for dependent in robot_repo wrist_repo; do", prepush)
        self.assertIn(CONFIG_FUZZ_TARGET, prepush)


if __name__ == "__main__":
    unittest.main()
