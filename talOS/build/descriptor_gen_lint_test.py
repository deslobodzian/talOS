"""The talos_messages field-name lint (descriptor_gen lint) on fixtures.

bad.fbs mixes fields without a unit suffix with every exempt kind (bool,
enum, enum array, struct, struct array) and suffixed numbers; only the
unsuffixed numbers may be reported. good.fbs embeds included.fbs's
unsuffixed struct: a file answers for its own declarations only.
"""

import ast
import os
import re
import shutil
import subprocess
import tempfile
import types
import unittest

from python.runfiles import runfiles

_RUNFILES = runfiles.Create()


def _path(env):
    return _RUNFILES.Rlocation(os.environ[env])


def _lint(env, file):
    return subprocess.run(
        [_path("TALOS_DESCRIPTOR_GEN"), "lint", _path(env), file],
        capture_output=True, text=True, check=False)


def _reported(stderr):
    """{Struct.field} names the lint reported."""
    return set(re.findall(r"lint_test\.(\w+\.\w+)", stderr))


class _StarlarkFail(AssertionError):
    """Raised by the mock Starlark `fail()` builtin."""


class _ElemsToList(ast.NodeTransformer):
    """Rewrites Starlark's `str.elems()` into Python's `list(str)`."""

    def visit_Call(self, node):
        self.generic_visit(node)
        if (isinstance(node.func, ast.Attribute)
                and node.func.attr == "elems"
                and not node.args
                and not node.keywords):
            return ast.copy_location(
                ast.Call(
                    func=ast.Name(id="list", ctx=ast.Load()),
                    args=[node.func.value],
                    keywords=[],
                ),
                node,
            )
        return node


def _load_contract_bzl():
    """Evaluates talOS/build/contract.bzl with lightweight Starlark stubs so
    talos_messages and _talos_bfbs_impl can be executed directly."""
    recorded = {}

    def make_recorder(kind):
        def _record(**kwargs):
            recorded.setdefault(kind, []).append(kwargs)
        return _record

    class _Rule:
        def __init__(self, implementation, attrs):
            self.implementation = implementation
            self.attrs = attrs

        def __call__(self, **kwargs):
            recorded.setdefault(self.implementation.__name__, []).append(kwargs)

    class _Attr:
        def __getattr__(self, kind):
            return lambda **kwargs: {"kind": kind, **kwargs}

    class _Native:
        def package_name(self):
            return "test_pkg"

        def package_relative_label(self, name):
            return f"//test_pkg:{name}"

        def genrule(self, **kwargs):
            recorded.setdefault("genrule", []).append(kwargs)

    env = {
        "DefaultInfo": lambda **kwargs: types.SimpleNamespace(**kwargs),
        "Label": lambda s: s,
        "attr": _Attr(),
        "depset": lambda items=(), **_kw: list(items),
        "fail": lambda msg: (_ for _ in ()).throw(_StarlarkFail(msg)),
        "native": _Native(),
        "provider": lambda **_kw: (lambda **f: types.SimpleNamespace(**f)),
        "rule": lambda implementation, attrs=None, **_kw: _Rule(
            implementation, attrs or {}),
        "select": lambda d: d,
        "struct": lambda **kwargs: types.SimpleNamespace(**kwargs),
    }

    def _load(_bzl, *symbols):
        for sym in symbols:
            if sym == "STAMP_ATTRS":
                env[sym] = {}
            elif sym == "DEFAULT_INCLUDE_PATHS":
                env[sym] = []
            elif sym in ("PyInfo", "maybe_stamp", "manifest_freshness_stamp"):
                env[sym] = lambda *a, **kw: None
            else:
                env[sym] = make_recorder(sym)

    env["load"] = _load

    bzl_path = _path("TALOS_CONTRACT_BZL")
    with open(bzl_path, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=bzl_path)
    tree = _ElemsToList().visit(tree)
    ast.fix_missing_locations(tree)
    exec(compile(tree, bzl_path, "exec"), env)  # noqa: S102
    return env, recorded


def _lint_fbs(fbs_text):
    """Compiles an inline lint_test/case.fbs schema and runs descriptor_gen lint."""
    with tempfile.TemporaryDirectory() as tmp:
        pkg_dir = os.path.join(tmp, "lint_test")
        out_dir = os.path.join(tmp, "out")
        os.makedirs(pkg_dir)
        os.makedirs(out_dir)
        fbs_path = os.path.join(pkg_dir, "case.fbs")
        with open(fbs_path, "w", encoding="utf-8") as f:
            f.write(fbs_text)
        flatc = subprocess.run(
            [_path("TALOS_FLATC"), "--binary", "--schema",
             "--bfbs-filenames", tmp, "-I", tmp, "-o", out_dir, fbs_path],
            capture_output=True, text=True, check=False)
        if flatc.returncode != 0:
            raise AssertionError(f"flatc failed: {flatc.stderr}")
        return subprocess.run(
            [_path("TALOS_DESCRIPTOR_GEN"), "lint",
             os.path.join(out_dir, "case.bfbs"), "//lint_test/case.fbs"],
            capture_output=True, text=True, check=False)


class FieldNameLintTest(unittest.TestCase):
    # REQ-CONTRACT-08
    def test_unsuffixed_numbers_fail_and_exempt_kinds_pass(self):
        result = _lint("TALOS_LINT_BAD", "//lint_test/bad.fbs")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(_reported(result.stderr),
                         {"Bad.speed_bad", "Bad.count", "Bad.samples_bad"})

    def test_suffixed_struct_passes(self):
        result = _lint("TALOS_LINT_GOOD", "//lint_test/good.fbs")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_included_struct_fails_in_its_own_file(self):
        result = _lint("TALOS_LINT_INCLUDED", "//lint_test/included.fbs")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(_reported(result.stderr), {"Included.raw"})

    # REQ-CONTRACT-08
    def test_valid_field_names_pass(self):
        result = _lint_fbs("""
            namespace lint_test;
            enum Mode : ubyte { OFF, ON }
            struct Inner { width_m:float; }
            struct Valid {
              x_m:double;
              y_m:double;
              z_m:double;
              q_w:double;
              v_mps:double;
              a_mps2:double;
              x_std_m:double;
              y_err_rad:double;
              module0:Inner;
              slot0_v:double;
              pose2d:Inner;
              accel_mps2:double;
              ok:bool;
              x:bool;
              mode:Mode;
            }
        """)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")

    # REQ-CONTRACT-08
    def test_non_lower_snake_case_fields_fail(self):
        result = _lint_fbs("""
            namespace lint_test;
            enum Mode : ubyte { OFF, ON }
            struct BadCase {
              Position_m:double;
              camelCase:bool;
              ModeField:Mode;
              bad__name_m:double;
              _leading_m:double;
              trailing_m_:double;
              flag_:bool;
              position_m:double;
              valid:bool;
            }
        """)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(
            _reported(result.stderr),
            {
                "BadCase.Position_m",
                "BadCase.camelCase",
                "BadCase.ModeField",
                "BadCase.bad__name_m",
                "BadCase._leading_m",
                "BadCase.trailing_m_",
                "BadCase.flag_",
            },
        )
        self.assertIn("must be lower snake_case", result.stderr)

    # REQ-CONTRACT-08
    def test_digit_segment_fields_fail(self):
        result = _lint_fbs("""
            namespace lint_test;
            struct Inner { width_m:float; }
            struct BadDigit {
              module_0:Inner;
              motor_1_v:double;
              sensor_2_ok:bool;
              samples_0_m:[float:2];
              module0:Inner;
              slot0_v:double;
              accel_mps2:double;
            }
        """)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(
            _reported(result.stderr),
            {
                "BadDigit.module_0",
                "BadDigit.motor_1_v",
                "BadDigit.sensor_2_ok",
                "BadDigit.samples_0_m",
            },
        )
        self.assertIn("'_<digit>'", result.stderr)
        self.assertNotIn("end every numeric field with a unit suffix",
                         result.stderr)

    # REQ-CONTRACT-08
    def test_interior_one_letter_before_unit_suffix_fails(self):
        result = _lint_fbs("""
            namespace lint_test;
            struct BadOneLetter {
              std_x_m:double;
              err_y_rad:double;
              pos_z_mps2:double;
              a_b_m:double;
              arr_x_v:[float:2];
              flag_a_id:bool;
              x_m:double;
              y_m:double;
              z_m:double;
              q_w:double;
              v_mps:double;
              x_std_m:double;
              y_err_rad:double;
            }
        """)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(
            _reported(result.stderr),
            {
                "BadOneLetter.std_x_m",
                "BadOneLetter.err_y_rad",
                "BadOneLetter.pos_z_mps2",
                "BadOneLetter.a_b_m",
                "BadOneLetter.arr_x_v",
                "BadOneLetter.flag_a_id",
            },
        )
        self.assertIn("one-letter word before its unit suffix", result.stderr)
        self.assertNotIn("end every numeric field with a unit suffix",
                         result.stderr)


class TalosMessagesMacroContractTest(unittest.TestCase):
    # REQ-CONTRACT-08
    def test_talos_messages_enforces_check_topics_at_load(self):
        env, recorded = _load_contract_bzl()
        talos_messages = env["talos_messages"]

        # Valid topics pass.
        talos_messages(
            name="good_msgs",
            srcs=["good.fbs"],
            topics={
                "state": "/intake/state",
                "teleop_target": "/intake/target/teleop",
            },
            include_prefix="lint_test",
        )
        self.assertEqual(len(recorded["cc_library"]), 1)

        # Malformed topics and keys must fail inside talos_messages itself
        # (deleting `_check_topics(topics)` from talos_messages fails here).
        bad_cases = [
            {"bad": "/drive/state"},        # owner abbreviation
            {"bad": "/intake_out/state"},   # owner direction suffix
            {"bad": "/hw/state/vel"},       # qualifier abbreviation
            {"bad": "/intake/pose"},        # invalid role
            {"BadKey": "/intake/state"},    # non-snake_case topic key
        ]
        for topics in bad_cases:
            with self.subTest(topics=topics):
                with self.assertRaises(_StarlarkFail):
                    talos_messages(
                        name="bad_msgs",
                        srcs=["good.fbs"],
                        topics=topics,
                        include_prefix="lint_test",
                    )

    # REQ-CONTRACT-08
    def test_talos_messages_wires_and_executes_descriptor_gen_lint(self):
        env, recorded = _load_contract_bzl()
        talos_messages = env["talos_messages"]
        talos_bfbs_rule = env["_talos_bfbs"]

        # Verify talos_messages instantiates _talos_bfbs and wires its output
        # into both the C++ library (via _talos_cc_descriptors) and the Python
        # library (via _flatc_py_tree).
        talos_messages(
            name="lint_pkg",
            srcs=["bad.fbs", "good.fbs", "included.fbs"],
            include_prefix="lint_test",
        )
        bfbs_calls = recorded.get("_talos_bfbs_impl", [])
        self.assertEqual(len(bfbs_calls), 1)
        bfbs_name = bfbs_calls[0]["name"]
        self.assertEqual(bfbs_calls[0]["srcs"],
                         ["bad.fbs", "good.fbs", "included.fbs"])
        self.assertEqual(bfbs_calls[0]["include_prefix"], "lint_test")

        cc_desc_calls = recorded.get("_talos_cc_descriptors_impl", [])
        self.assertEqual(len(cc_desc_calls), 1)
        self.assertEqual(cc_desc_calls[0]["bfbs"], ":" + bfbs_name)
        cc_lib_calls = recorded.get("cc_library", [])
        self.assertEqual(len(cc_lib_calls), 1)
        self.assertIn(":" + cc_desc_calls[0]["name"], cc_lib_calls[0]["hdrs"])

        py_tree_calls = recorded.get("_flatc_py_tree_impl", [])
        self.assertEqual(len(py_tree_calls), 1)
        self.assertEqual(py_tree_calls[0]["bfbs"], ":" + bfbs_name)
        py_lib_calls = recorded.get("py_library", [])
        self.assertEqual(len(py_lib_calls), 1)
        self.assertIn(":" + py_tree_calls[0]["name"], py_lib_calls[0]["deps"])

        # Verify _talos_bfbs binds _gen to //talOS/build:descriptor_gen.
        self.assertEqual(talos_bfbs_rule.attrs["_gen"]["default"],
                         "//talOS/build:descriptor_gen")

        # Execute _talos_bfbs_impl and actually run its emitted shell command
        # with real flatc and real descriptor_gen against bad/good/included.fbs.
        # Deleting `"$5" lint "$4" "//$3"` from _talos_bfbs_impl makes bad.fbs
        # exit 0 and fails this test.
        with tempfile.TemporaryDirectory() as tmp:
            tree_dir = os.path.join(tmp, "tree")
            os.makedirs(os.path.join(tree_dir, "lint_test"))
            out_dir = os.path.join(tmp, "out")
            os.makedirs(out_dir)

            src_files = []
            for env_key, basename in (
                ("TALOS_LINT_BAD_FBS", "bad.fbs"),
                ("TALOS_LINT_GOOD_FBS", "good.fbs"),
                ("TALOS_LINT_INCLUDED_FBS", "included.fbs"),
            ):
                dst = os.path.join(tree_dir, "lint_test", basename)
                shutil.copyfile(_path(env_key), dst)
                src_files.append(types.SimpleNamespace(
                    basename=basename, path=dst))

            actions = []

            class _FakeActions:
                def declare_file(self, rel):
                    full = os.path.join(out_dir, rel)
                    os.makedirs(os.path.dirname(full), exist_ok=True)
                    return types.SimpleNamespace(path=full)

                def run_shell(self, **kwargs):
                    actions.append(kwargs)

            schemas_provider = env["_TalosMessagesInfo"]
            gen_exec = types.SimpleNamespace(path=_path("TALOS_DESCRIPTOR_GEN"))
            flatc_exec = types.SimpleNamespace(path=_path("TALOS_FLATC"))
            ctx = types.SimpleNamespace(
                label=types.SimpleNamespace(name=bfbs_name),
                attr=types.SimpleNamespace(
                    schemas={schemas_provider: types.SimpleNamespace(
                        tree=types.SimpleNamespace(path=tree_dir))},
                    include_prefix="lint_test",
                ),
                files=types.SimpleNamespace(srcs=src_files),
                executable=types.SimpleNamespace(
                    _flatc=flatc_exec, _gen=gen_exec),
                actions=_FakeActions(),
            )
            talos_bfbs_rule.implementation(ctx)
            self.assertEqual(len(actions), 3)

            by_stem = {}
            for action in actions:
                self.assertIn(gen_exec, action["tools"])
                out_path = action["outputs"][0].path
                stem = os.path.basename(out_path).removesuffix(".bfbs")
                by_stem[stem] = (action, out_path)

            bad_action, _ = by_stem["bad"]
            bad_res = subprocess.run(
                ["bash", "-c", bad_action["command"], "talos_bfbs",
                 *bad_action["arguments"]],
                capture_output=True, text=True, check=False)
            self.assertEqual(bad_res.returncode, 1, bad_res.stderr)
            self.assertEqual(_reported(bad_res.stderr),
                             {"Bad.speed_bad", "Bad.count", "Bad.samples_bad"})

            good_action, good_out = by_stem["good"]
            good_res = subprocess.run(
                ["bash", "-c", good_action["command"], "talos_bfbs",
                 *good_action["arguments"]],
                capture_output=True, text=True, check=False)
            self.assertEqual(good_res.returncode, 0, good_res.stderr)
            self.assertTrue(os.path.isfile(good_out))

            inc_action, _ = by_stem["included"]
            inc_res = subprocess.run(
                ["bash", "-c", inc_action["command"], "talos_bfbs",
                 *inc_action["arguments"]],
                capture_output=True, text=True, check=False)
            self.assertEqual(inc_res.returncode, 1, inc_res.stderr)
            self.assertEqual(_reported(inc_res.stderr), {"Included.raw"})


if __name__ == "__main__":
    unittest.main()
