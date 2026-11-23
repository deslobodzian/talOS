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
SCAFFOLDS = (("gripper", "cpp", "velocity", False, "actuator"),
             ("scaffold_py", "py", "velocity", False, "actuator"),
             ("position_cpp", "cpp", "position", False, "actuator"),
             ("position_py", "py", "position", False, "actuator"),
             ("position_raw_cpp", "cpp", "position", True, "actuator"),
             ("position_raw_py", "py", "position", True, "actuator"),
             ("localizer", "cpp", "estimator", False, "observer"),
             ("py_localizer", "py", "estimator", False, "observer"))


def scaffold(name, lang, kind, raw):
    """(node test text, [non-test sources]) of a generated package."""
    files = new_subsystem.package_files(name, lang, kind=kind, raw=raw)
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
        for name, lang, kind, raw, role in SCAFFOLDS:
            with self.subTest(name):
                test, sources = scaffold(name, lang, kind, raw)
                self.assertEqual(check_node_matrix.infer_role(sources), role)
                _, result = check_node_matrix.check(test, lang, role)
                self.assertEqual(missing(result), {})

    def test_a_scaffold_without_case_11_fails(self):
        # A node test that loses the earlier-generation case (REQ-SAFE-09)
        # fails the lint, naming it.
        for name, lang, kind, raw, role in SCAFFOLDS:
            if role != "actuator":
                continue
            with self.subTest(name):
                test, _ = scaffold(name, lang, kind, raw)
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

    # REQ-CONTRACT-14 (a parity claim stands for a C++ case only when the
    # Python node really runs that step)
    def test_a_parity_claim_holds_only_with_its_run_cpp_case_and_steps(self):
        # A Python twin may cover a case by its talos.parity scenario only
        # when the file runs one, the C++ test carries the case, and the
        # steps it names are in the scenario its test passes (names resolved
        # in the calling function first) and that test is not skipped;
        # otherwise the lint fails naming the case, so a case cannot be
        # dropped by a bare comment.
        cpp = "TEST_F(ArmTest, StaleLaneGoesNeutral) {}\n"
        run = ('SCENARIO = [\n    ("stale lane", wait(0.2)),\n]\n'
               "def test_parity():\n"
               "    parity.assert_same_behaviour(Cpp, Py, CONFIG, \"arm\",\n"
               "                                 SCENARIO)\n")
        claim = '# matrix: parity 3 node_test.cc "stale lane"\n'

        def sibling(files):
            return files.get

        _, result = check_node_matrix.check(
            claim + run, "py", "actuator", sibling({"node_test.cc": cpp}))
        self.assertEqual(result[3][0], "parity")
        # A test's own scenario is the one its call runs.
        local = run.replace('("stale lane"', '("other"', 1).replace(
            "    parity.assert_same",
            '    SCENARIO = [("stale lane", wait(0.2))]\n'
            "    parity.assert_same")
        _, result = check_node_matrix.check(
            claim + local, "py", "actuator", sibling({"node_test.cc": cpp}))
        self.assertEqual(result[3][0], "parity")
        # A second run on a scenario built from the first (the same steps at
        # another period) counts no steps of its own but refuses nothing.
        derived = run + (
            "def test_parity_slow():\n"
            "    slow = [(name, *a, wait(0.01)) for name, *a in SCENARIO]\n"
            "    parity.assert_same_behaviour(Cpp, Py, SLOW, \"arm\", slow)\n")
        _, result = check_node_matrix.check(
            claim + derived, "py", "actuator", sibling({"node_test.cc": cpp}))
        self.assertEqual(result[3][0], "parity")
        for why, text, files in (
                ("no parity run", claim + run.split("def test_parity")[0],
                 {"node_test.cc": cpp}),
                ("C++ file lacks the case", claim + run,
                 {"node_test.cc": "TEST_F(ArmTest, Other) {}\n"}),
                ("no C++ file", claim + run, {}),
                ("missing step",
                 claim.replace("stale lane", "stale target") + run,
                 {"node_test.cc": cpp}),
                # The step is a tuple in the file, but not in the scenario
                # the parity run is given.
                ("step outside the scenario",
                 claim.replace("stale lane", "stale target") + run
                 + 'UNUSED = [("stale target", wait(0.2))]\n'
                 + 'check(("stale target", 1))\n',
                 {"node_test.cc": cpp}),
                ("construction cannot be compared",
                 claim.replace("parity 3", "parity 9") + run,
                 {"node_test.cc": cpp}),
                # The run that names the step never runs.
                ("skipped test",
                 claim + run.replace("def test_parity",
                                     '@unittest.skip("later")\n'
                                     "def test_parity"),
                 {"node_test.cc": cpp}),
                ("skipped class",
                 claim + run.split("def test_parity")[0]
                 + '@unittest.skipIf(True, "later")\n'
                 + "class ParityTest(unittest.TestCase):\n"
                 + "  def test_parity(self):\n"
                 + "    parity.assert_same_behaviour(Cpp, Py, CONFIG, "
                 + '"arm", SCENARIO)\n',
                 {"node_test.cc": cpp}),
                ("skipTest in the test",
                 claim + run.replace("    parity.assert_same",
                                     '    self.skipTest("later")\n'
                                     "    parity.assert_same"),
                 {"node_test.cc": cpp}),
                # The test passes its own SCENARIO, which has no such step.
                ("function-local scenario shadows the module's",
                 claim + run.replace(
                     "    parity.assert_same",
                     '    SCENARIO = [("other", wait(0.2))]\n'
                     "    parity.assert_same"),
                 {"node_test.cc": cpp}),
                # A helper's scenario is its parameter, not the module's.
                ("scenario is a parameter",
                 claim + run.replace("def test_parity():",
                                     "def run(SCENARIO):"),
                 {"node_test.cc": cpp})):
            with self.subTest(why), self.assertRaisesRegex(
                    check_node_matrix.MatrixError, r"parity [39]:"):
                check_node_matrix.check(text, "py", "actuator",
                                        sibling(files))

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

    def test_a_package_of_testonly_nodes_is_skipped_and_listed(self):
        # A fixture node no robot bundle can carry (bundle_testdata's
        # native_wheel) needs no matrix, and the run says it skipped it; a
        # package with one ordinary node rule, or testonly = False, is
        # still checked.
        packages = {
            "fixture": "talos_py_node(\n    name = \"a\",\n"
                       "    testonly = True,\n)\n",
            "mixed": "talos_cc_node(\n    name = \"node\",\n)\n"
                     "talos_cc_shared_node(\n    name = \"lib.so\",\n"
                     "    testonly = True,\n)\n",
            "explicit": "talos_cc_node(\n    name = \"node\",\n"
                        "    testonly = False,\n)\n",
            # A testonly inside another call's arguments is not the node's.
            "nested": "talos_cc_node(\n    name = \"node\",\n"
                      "    deps = select({\"x\": [f(testonly = True)]}),\n"
                      ")\n"}
        with tempfile.TemporaryDirectory() as root:
            for name, build in packages.items():
                os.makedirs(os.path.join(root, "bot", name))
                with open(os.path.join(root, "bot", name, "BUILD"), "w",
                          encoding="utf-8") as f:
                    f.write(build)
            self.assertEqual(
                check_node_matrix.targets(root, ["bot"]),
                [(os.path.join(root, "bot", name), role) for name, role in (
                    ("explicit", "observer"), ("fixture", "fixture"),
                    ("mixed", "observer"), ("nested", "observer"))])
            out = io.StringIO()
            with mock.patch.dict(os.environ,
                                 {"BUILD_WORKSPACE_DIRECTORY": root}), \
                    contextlib.redirect_stdout(out):
                self.assertEqual(check_node_matrix.main(["bot/fixture"]), 0)
            self.assertIn("bot/fixture  [fixture]  skipped", out.getvalue())

    # REQ-SAFE-38: BUILD parsing uses AST, not regex/string-tracking, so
    # escaped backslashes, triple-quoted strings, comments, or syntax errors
    # cannot spoof `testonly = True` or bypass matrix checks.
    def test_testonly_cannot_be_spoofed_by_strings_and_syntax_error_fails_closed(
            self):
        packages = {
            "escaped_backslash": (
                'talos_cc_node(\n'
                '    name = "node",\n'
                '    desc = "\\\\",\n'
                '    note = "testonly = True",\n'
                ')\n'
            ),
            "triple_quote": (
                'talos_cc_node(\n'
                '    name = "node",\n'
                '    doc = """hello "testonly = True" world""",\n'
                ')\n'
            ),
            "fake_in_string": (
                'DOC = """\n'
                'talos_cc_node(\n'
                '    name = "fake",\n'
                '    testonly = True,\n'
                ')\n'
                '"""\n'
            ),
            "aliased_load": (
                'load("//talOS/build:contract.bzl", my_node = "talos_cc_node")\n'
                'my_node(name = "node")\n'
            ),
        }
        with tempfile.TemporaryDirectory() as root:
            for name, build in packages.items():
                os.makedirs(os.path.join(root, "bot", name))
                with open(os.path.join(root, "bot", name, "BUILD"), "w",
                          encoding="utf-8") as f:
                    f.write(build)
            self.assertEqual(
                check_node_matrix.targets(root, ["bot"]),
                [(os.path.join(root, "bot", name), "observer")
                 for name in ("aliased_load", "escaped_backslash",
                              "triple_quote")])

            # A BUILD file with a syntax error fails closed.
            os.makedirs(os.path.join(root, "broken"))
            with open(os.path.join(root, "broken", "BUILD"), "w",
                      encoding="utf-8") as f:
                f.write('talos_cc_node(name = "node",\n')
            with self.assertRaises(check_node_matrix.MatrixError):
                check_node_matrix.targets(root, ["broken"])
            err = io.StringIO()
            with mock.patch.dict(os.environ,
                                 {"BUILD_WORKSPACE_DIRECTORY": root}), \
                    contextlib.redirect_stderr(err):
                self.assertEqual(check_node_matrix.main(["broken"]), 2)
            self.assertIn("broken/BUILD", err.getvalue())

    def test_baseline_fails_on_a_new_missing_case_and_a_stale_entry(self):
        # CI runs the lint over a baseline of the cases missing today,
        # which may only shrink: an unlisted missing case fails,
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
                    ("bot/arm/node_test.cc 11 12  # known gap\n", 0),
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

    # REQ-CONTRACT-14, REQ-SAFE-38: commented-out C++ tests, skipped or
    # docstring Python tests, and aliased Python actuator imports must not
    # bypass the matrix check.
    def test_commented_skipped_tests_and_aliased_actuator_imports_fail_closed(
            self):
        # Commented-out and raw-string C++ tests do not count as present.
        cpp_commented = (
            "/*\n"
            "TEST(Obs, PublishesFromALiveInput) {}\n"
            "*/\n"
            'const char* s = R"(\n'
            "TEST(Obs, StaleInputClearsValidity) {}\n"
            ')";\n'
        )
        _, result = check_node_matrix.check(cpp_commented, "cpp", "observer")
        self.assertIn(1, missing(result))
        self.assertIn(3, missing(result))

        # A commented-out C++ test must not satisfy a Python parity claim.
        claim = '# matrix: parity 3 node_test.cc "stale lane"\n'
        run = (
            'SCENARIO = [("stale lane", wait(0.2))]\n'
            'def test_parity():\n'
            '    parity.assert_same_behaviour(Cpp, Py, CONFIG, "arm", SCENARIO)\n'
        )
        with self.assertRaisesRegex(check_node_matrix.MatrixError, r"parity 3:"):
            check_node_matrix.check(
                claim + run, "py", "actuator",
                lambda name: (
                    "/*\nTEST_F(ArmTest, StaleLaneGoesNeutral) {}\n*/\n"
                    if name == "node_test.cc" else None
                ),
            )

        # Skipped Python tests (method decorator, class decorator, skipTest)
        # and defs inside docstrings do not count as present.
        py_skipped = (
            '"""\n'
            'def test_publishes_from_a_live_input(self):\n'
            '    pass\n'
            '"""\n'
            'import unittest\n'
            'class ObsTest(unittest.TestCase):\n'
            '    @unittest.skip("broken")\n'
            '    def test_stale_input_clears_validity(self):\n'
            '        pass\n'
            '    def test_bad_parameter_fails_construction(self):\n'
            '        self.skipTest("later")\n'
        )
        _, result = check_node_matrix.check(py_skipped, "py", "observer")
        self.assertEqual(sorted(missing(result)), [1, 3, 9])

        # A Python actuator importing talos or talos.devices under an alias
        # is still inferred as actuator (12 cases, not 4).
        for src in (
            "import talos as t\nm = t.Motor(node, 'm')\n",
            "import talos.devices as dev\nm = dev.PwmOutput(node, 'p')\n",
        ):
            with self.subTest(src=src):
                self.assertEqual(check_node_matrix.infer_role([src]), "actuator")


if __name__ == "__main__":
    unittest.main()
