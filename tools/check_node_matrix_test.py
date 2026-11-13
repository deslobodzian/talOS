"""Unit tests for //tools:check_node_matrix."""

import contextlib
import io
import os
import tempfile
import unittest
from unittest import mock

from tools import check_node_matrix
from tools import new_subsystem

# Every archetype //tools:new_subsystem generates, with its matrix role.
SCAFFOLDS = (("gripper", "cpp", "velocity", "actuator"),
             ("scaffold_py", "py", "velocity", "actuator"),
             ("position_cpp", "cpp", "position", "actuator"),
             ("position_py", "py", "position", "actuator"),
             ("localizer", "cpp", "estimator", "observer"),
             ("py_localizer", "py", "estimator", "observer"))


def scaffold(name, lang, kind):
    """(node test text, [non-test sources]) of a generated package."""
    files = new_subsystem.package_files(name, lang, kind=kind)
    test = files.pop("node_test.cc" if lang == "cpp" else "node_test.py")
    return test, [text for path, text in files.items()
                  if path.endswith((".h", ".cc", ".py"))]


def missing(result):
    return {case: detail for case, (status, detail) in result.items()
            if status == "missing"}


class CheckNodeMatrixTest(unittest.TestCase):
    def test_every_scaffold_has_its_whole_matrix(self):
        # The templates are the matrix's reference: each generated test file
        # has every case its inferred role requires, or a reasoned n/a.
        for name, lang, kind, role in SCAFFOLDS:
            with self.subTest(name):
                test, sources = scaffold(name, lang, kind)
                self.assertEqual(check_node_matrix.infer_role(sources), role)
                _, result = check_node_matrix.check(test, lang, role)
                self.assertEqual(missing(result), {})

    def test_a_scaffold_without_case_11_fails(self):
        # A node test that loses the earlier-generation case (REQ-SAFE-09)
        # fails the lint, naming it.
        for name, lang, kind, role in SCAFFOLDS[:4]:
            with self.subTest(name):
                test, _ = scaffold(name, lang, kind)
                case_11 = ("EarlierGenerationTargetIsIgnored" if lang == "cpp"
                           else "test_earlier_generation_target_is_ignored")
                self.assertIn(case_11, test)
                _, result = check_node_matrix.check(
                    test.replace(case_11, "Renamed"), lang, role)
                self.assertEqual(missing(result), {11: case_11})

    def test_infers_the_role_from_the_package_sources(self):
        for sources, role in (
                (["from talos.devices import (\n    DigitalInput,\n"
                  "    Motor,\n)\n"], "actuator"),
                (["from talos import Motor\n"], "actuator"),
                # examples/superstructure: the Motor is the block's.
                (["talos::mechanisms::PositionMechanism lift_;\n"],
                 "actuator"),
                # talOS/teleop writes its lanes through the framework hooks.
                (["arming_{talos::detail::TargetAccess::open_arming(node)}"],
                 "producer"),
                (["from talos import testing\nMotorState = 1\n"],
                 "observer"),
                (['Elevator = talos.cpp_node("lib.so", "elevator")\n'],
                 "hosted")):
            with self.subTest(role=role, sources=sources):
                self.assertEqual(check_node_matrix.infer_role(sources), role)

    def test_a_role_override_needs_a_reason(self):
        with self.assertRaises(check_node_matrix.MatrixError):
            check_node_matrix.check("// matrix: role producer\n", "cpp",
                                    "observer")
        role, result = check_node_matrix.check(
            "// matrix: role producer (sends through a helper)\n", "cpp",
            "observer")
        self.assertEqual(role, "producer")
        self.assertIn(11, missing(result))

    def test_an_actuator_cannot_be_overridden_to_another_role(self):
        for override in ("observer", "producer", "hosted"):
            with self.subTest(override), self.assertRaises(
                    check_node_matrix.MatrixError):
                check_node_matrix.check(
                    f"// matrix: role {override} (fewer cases)\n", "cpp",
                    "actuator")

    def test_hosted_is_refused_in_a_file_that_defines_tests(self):
        with self.assertRaises(check_node_matrix.MatrixError):
            check_node_matrix.check(
                "# matrix: role hosted (wraps a C++ node)\n"
                "    def test_publishes(self):\n", "py", "observer")

    def test_a_directive_inside_a_string_is_not_read(self):
        _, result = check_node_matrix.check(
            'kText = "// matrix: n/a 9 (quoted)";\n', "cpp", "observer")
        self.assertIn(9, missing(result))

    def test_not_applicable_needs_a_reason(self):
        names = check_node_matrix.CASES["observer"]
        body = "".join(f"TEST(T, {names[case]}) {{}}\n" for case in (1, 3))
        _, result = check_node_matrix.check(
            body + "// matrix: n/a 9, 12 (reads no parameters)\n", "cpp",
            "observer")
        self.assertEqual(result[9], ("n/a", "reads no parameters"))
        self.assertEqual(missing(result), {})
        _, result = check_node_matrix.check(
            body + "// matrix: n/a 9, 12\n", "cpp", "observer")
        self.assertEqual(sorted(missing(result)), [9, 12])

    def test_finds_node_packages_and_the_nodes_a_manifest_launches(self):
        # bot/arm is under the scanned root; lib/teleop only in its manifest.
        with tempfile.TemporaryDirectory() as root:
            for path, text in (
                    ("bot/arm/BUILD", "talos_cc_node(\n)\n"),
                    ("bot/arm/arm.h", "talos::Motor motor_;\n"),
                    ("bot/config/robot_manifest.bzl",
                     'NODES = [\n    "//bot/arm:node",\n'
                     '    "//lib/teleop:node",\n]\n'),
                    ("lib/teleop/BUILD", "talos_cc_node(\n)\n"),
                    ("lib/teleop/node.h", "talos::TargetLane<M> lane_;\n"),
                    ("lib/teleop/node_test.cc", "")):
                os.makedirs(os.path.dirname(os.path.join(root, path)),
                            exist_ok=True)
                with open(os.path.join(root, path), "w",
                          encoding="utf-8") as f:
                    f.write(text)
            self.assertEqual(
                check_node_matrix.targets(root, ["bot"]),
                [(os.path.join(root, "bot", "arm"), "actuator"),
                 (os.path.join(root, "lib", "teleop", "node_test.cc"),
                  "producer")])
            with mock.patch.dict(os.environ,
                                 {"BUILD_WORKSPACE_DIRECTORY": root}):
                self.assertEqual(check_node_matrix.main(["bot"]), 1)

    def test_baseline_fails_on_a_new_missing_case_and_a_stale_entry(self):
        # CI runs the lint over a baseline of the cases missing today
        # (MATRIX-1), which may only shrink: an unlisted missing case fails,
        # and so does a listed case that is present or a path that is gone.
        names = check_node_matrix.CASES["actuator"]
        body = "".join(f"TEST(Arm, {names[case]}) {{}}\n"
                       for case in range(1, 11))
        with tempfile.TemporaryDirectory() as root:
            for path, text in (
                    ("bot/arm/BUILD", "talos_cc_node(\n)\n"),
                    ("bot/arm/arm.h", "talos::Motor motor_;\n"),
                    ("bot/arm/node_test.cc", body)):
                os.makedirs(os.path.dirname(os.path.join(root, path)),
                            exist_ok=True)
                with open(os.path.join(root, path), "w",
                          encoding="utf-8") as f:
                    f.write(text)
            for baseline, code in (
                    ("bot/arm/node_test.cc 11 12  # MATRIX-1\n", 0),
                    ("bot/arm/node_test.cc 12\n", 1),
                    ("bot/arm/node_test.cc 10 11 12\n", 1),
                    ("bot/arm/node_test.cc 11 12\nbot/gone_test.cc 1\n", 1),
                    ("bot/arm/node_test.cc eleven\n", 2)):
                with self.subTest(baseline=baseline):
                    with open(os.path.join(root, "baseline.txt"), "w",
                              encoding="utf-8") as f:
                        f.write(baseline)
                    with mock.patch.dict(
                            os.environ, {"BUILD_WORKSPACE_DIRECTORY": root}), \
                            contextlib.redirect_stdout(io.StringIO()), \
                            contextlib.redirect_stderr(io.StringIO()):
                        self.assertEqual(check_node_matrix.main(
                            ["bot", "--baseline", "baseline.txt"]), code)


if __name__ == "__main__":
    unittest.main()
