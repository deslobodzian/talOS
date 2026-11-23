"""Fast unit tests for tools/check_commissioning.py (no git subprocesses).

REQ-DEV-33 (RULES.md 4): directly exercises TOML parsing, guarded-key
extraction (`commissioned`, `can_id`, `bus`, `include`, `[[subsystems]]`
`path`/`defaults`, `[subsystem]` `defaults`), 2-way and 3-way merge diffing,
path exemptions (`examples`, `tests`, `fixtures`, `*testdata`), raw diff-tree
parsing, fail-closed error handling, and `Commissioned-By:` trailer checking
without spawning git. Full end-to-end git-repository tests live in
`//tools:check_commissioning_test` (integration lane).
"""

import tomllib
import unittest
from unittest import mock

from tools import check_commissioning as cc


def _guarded(toml_text):
    return cc.extract_guarded(tomllib.loads(toml_text))


class CheckCommissioningUnitTest(unittest.TestCase):

    # REQ-DEV-33, RULES.md 4: path exemptions are exact-case directories and
    # `*testdata`, while `.toml` matching is case-insensitive.
    def test_path_exemption_and_robot_config_matching(self):
        for path in (
            "bench/main_processor/configuration/robot.toml",
            "bench/main_processor/configuration/EXTRA.TOML",
            "bench/main_processor/configuration/Motor.Toml",
            "robot.toml",
            "examples.toml",
            "robot/examples.toml",
            "EXAMPLES/robot/robot.toml",
            "TESTS/robot/robot.toml",
            "FIXTURES/robot/robot.toml",
            "bench/TESTDATA/robot.toml",
        ):
            with self.subTest(guarded=path):
                self.assertFalse(cc.is_exempt_path(path))
                self.assertTrue(cc.is_robot_config(path))

        for path in (
            "examples/robot/robot.toml",
            "examples/robot/EXTRA.TOML",
            "tests/robot/robot.toml",
            "fixtures/robot/robot.toml",
            "a/b/examples/c.toml",
            "a/b/tests/c.toml",
            "a/b/fixtures/c.toml",
            "tools/scaffold_testdata/gripper/robot.toml",
            "bench/testdata/robot.toml",
        ):
            with self.subTest(exempt=path):
                self.assertTrue(cc.is_exempt_path(path))
                self.assertFalse(cc.is_robot_config(path))

        for path in (
            "bench/main_processor/README.md",
            "bench/main_processor/robot.toml.bak",
            "tools/check_commissioning.py",
        ):
            with self.subTest(non_toml=path):
                self.assertFalse(cc.is_exempt_path(path))
                self.assertFalse(cc.is_robot_config(path))

    # REQ-DEV-33, RULES.md 4: every valid TOML spelling of commissioned,
    # can_id, bus, include and subsystem pointers is extracted and diffed.
    def test_guarded_key_extraction_and_diffing(self):
        base = _guarded(
            '[robot]\ncommissioned = false\n'
            '[motors.drive]\ntype = "TalonFX"\ncan_id = 1\n'
        )
        edited = _guarded(
            '[robot]\ncommissioned = true\n'
            '[motors.drive]\ntype = "TalonFX"\ncan_id = 7\nbus = "Shooter"\n'
        )
        findings = cc.compare_guarded(base, edited)
        self.assertEqual(
            findings,
            [
                'sets a CAN bus: motors.drive.bus = "Shooter"',
                "changes a CAN id: motors.drive.can_id from 1 to 7",
                "changes commissioned at robot.commissioned from false to true",
            ],
        )

        # Removing guarded keys is also reported.
        removed = cc.compare_guarded(edited, _guarded('[motors.drive]\n'))
        self.assertEqual(
            removed,
            [
                'removes a CAN bus: motors.drive.bus = "Shooter"',
                "removes a CAN id: motors.drive.can_id = 7",
                "removes commissioned = true at robot.commissioned",
            ],
        )

        # Dotted, inline-table, quoted, unicode-escaped, and multiline strings.
        tricky = (
            'robot.commissioned = true\n'
            'a = { type = """\nTalonFX""", bus = "Drive", can_id = 3 }\n'
            r'b."\u0063an_id" = 4' + "\n"
            r'b."\U00000062us" = "canivore"' + "\n"
            'include = "other.toml"\n'
        )
        tricky_findings = "\n".join(cc.compare_guarded({}, _guarded(tricky)))
        for needle in (
            "sets commissioned = true at robot.commissioned",
            'sets a CAN bus: a.bus = "Drive"',
            "sets a CAN id: a.can_id = 3",
            "sets a CAN id: b.can_id = 4",
            'sets a CAN bus: b.bus = "canivore"',
            'sets include: include = "other.toml"',
        ):
            self.assertIn(needle, tricky_findings)

        # Comments, string values, unrelated keys, container tables named like
        # a guarded key, and literal single-quoted escapes are not guarded.
        innocent = (
            '# commissioned = true\n'
            'note = "can_id = 3, bus = Shooter"\n'
            'can_bus = "rio"\n'
            'bus_name = "Shooter"\n'
            r"'\u0063an_id' = 5" + "\n"
            '[other]\n'
            r'"\\u0063an_id" = 6' + "\n"
            '[subsystems.bus]\nlabel = "ok"\n'
            '[teleop.defaults]\ndeadband = 0.05\n'
        )
        self.assertEqual(_guarded(innocent), {})

    # REQ-DEV-33, RULES.md 4: swapping table headers or subsystem pointers
    # changes the path->value mapping and is caught.
    def test_table_header_swap_and_subsystem_pointers(self):
        before_motors = _guarded(
            '[motors.lead]\ncan_id = 1\nbus = "Drive"\n'
            '[motors.follower]\ncan_id = 2\n'
        )
        after_motors = _guarded(
            '[motors.follower]\ncan_id = 1\nbus = "Drive"\n'
            '[motors.lead]\ncan_id = 2\n'
        )
        motor_findings = "\n".join(
            cc.compare_guarded(before_motors, after_motors))
        self.assertIn("motors.lead.can_id from 1 to 2", motor_findings)
        self.assertIn("motors.follower.can_id from 2 to 1", motor_findings)
        self.assertIn("removes a CAN bus: motors.lead.bus", motor_findings)
        self.assertIn("sets a CAN bus: motors.follower.bus", motor_findings)

        before_manifest = _guarded(
            'include = "base.toml"\n'
            '[[subsystems]]\nname = "arm"\npath = "subsystems/arm.toml"\n'
            'defaults = "shared/arm.toml"\n'
            '[[subsystems]]\nname = "wrist"\npath = "subsystems/wrist.toml"\n'
        )
        after_manifest = _guarded(
            'include = "alt.toml"\n'
            '[[subsystems]]\nname = "wrist"\npath = "subsystems/arm.toml"\n'
            'defaults = "shared/other.toml"\n'
            '[[subsystems]]\nname = "arm"\npath = "subsystems/wrist.toml"\n'
        )
        manifest_findings = "\n".join(
            cc.compare_guarded(before_manifest, after_manifest))
        for needle in (
            'changes include: include from "base.toml" to "alt.toml"',
            "subsystems[arm].path",
            "subsystems[arm].defaults",
            "subsystems[wrist].path",
            "subsystems[wrist].defaults",
        ):
            self.assertIn(needle, manifest_findings)

        # `[subsystem] defaults` is guarded; duplicate subsystem names fall
        # back to index-keyed pointer tracking.
        sub_before = _guarded('[subsystem]\nname = "arm"\ndefaults = "a.toml"\n')
        sub_after = _guarded('[subsystem]\nname = "arm"\ndefaults = "b.toml"\n')
        self.assertEqual(
            cc.compare_guarded(sub_before, sub_after),
            ['changes a subsystem config pointer: subsystem.defaults '
             'from "a.toml" to "b.toml"'],
        )
        dup_before = _guarded(
            '[[subsystems]]\nname = "a"\npath = "1.toml"\n'
            '[[subsystems]]\nname = "a"\npath = "2.toml"\n'
        )
        dup_after = _guarded(
            '[[subsystems]]\nname = "a"\npath = "1.toml"\n'
            '[[subsystems]]\nname = "b"\npath = "2.toml"\n'
        )
        self.assertTrue(cc.compare_guarded(dup_before, dup_after))

    # REQ-DEV-33, RULES.md 4: canonical() preserves TOML value types.
    def test_canonical_distinguishes_bool_int_float_and_nan(self):
        self.assertNotEqual(cc.canonical(True), cc.canonical(1))
        self.assertNotEqual(cc.canonical(False), cc.canonical(0))
        self.assertNotEqual(cc.canonical(1), cc.canonical(1.0))
        self.assertEqual(cc.canonical(float("nan")),
                         cc.canonical(float("nan")))

    # REQ-DEV-33, RULES.md 4: 3-way merge diff catches merge-introduced edits,
    # stale reverts and conflict resolutions while accepting clean merges.
    def test_compare_merge_guarded(self):
        base = _guarded("[m]\ncan_id = 1\n")
        p1_changed = _guarded("[m]\ncan_id = 5\n")
        p2_changed = _guarded("[m]\ncan_id = 6\n")
        other = _guarded("[m]\ncan_id = 9\n")

        # Clean merges where one parent changed and merge kept that change.
        self.assertEqual(
            cc.compare_merge_guarded(base, p1_changed, base, p1_changed), [])
        self.assertEqual(
            cc.compare_merge_guarded(base, base, p2_changed, p2_changed), [])

        # Merge introduces a brand new edit when neither parent changed.
        self.assertEqual(
            cc.compare_merge_guarded(base, base, base, other),
            ["changes a CAN id: m.can_id from 1 to 9"],
        )

        # Merge reverts p1's change back to base (stale copy).
        self.assertEqual(
            cc.compare_merge_guarded(base, p1_changed, base, base),
            ["changes a CAN id: m.can_id from 5 to 1"],
        )

        # Both parents changed the same key differently: conflict resolution
        # requires human sign-off regardless of which value was chosen.
        conflict = cc.compare_merge_guarded(
            base, p1_changed, p2_changed, p1_changed)
        self.assertEqual(len(conflict), 1)
        self.assertIn("resolves a merge conflict on a CAN id: m.can_id",
                      conflict[0])

    # REQ-DEV-33, RULES.md 4: raw diff-tree parser filters configs/symlinks
    # and fails closed on unexpected records or rename/copy status.
    def test_parse_raw_diff_tree_and_fail_closed_loaders(self):
        oid1 = "1" * 40
        oid2 = "2" * 40
        zero = "0" * 40
        raw = b"\0".join([
            f":100644 100644 {oid1} {oid2} M".encode(),
            "bench/configuración/motor.toml".encode("utf-8"),
            f":000000 100644 {zero} {oid2} A".encode(),
            b"examples/robot/robot.toml",
            f":000000 100644 {zero} {oid2} A".encode(),
            b"bench/README.md",
            f":000000 120000 {zero} {oid2} A".encode(),
            b"robot/subsystems",
            f":000000 120000 {zero} {oid2} A".encode(),
            b"examples/robot/link",
            b"",
        ])
        parsed = cc.parse_raw_diff_tree(raw)
        self.assertEqual(
            parsed,
            {
                "bench/configuración/motor.toml": (
                    ("100644", oid1), ("100644", oid2)),
                "robot/subsystems": (None, ("120000", oid2)),
            },
        )

        for bad_raw in (
            b"not-a-record\0foo.toml\0",
            f":100644 100644 {oid1} {oid2} R100\0a.toml\0b.toml\0".encode(),
            f":100644 100644 {oid1} {oid2} M".encode(),
        ):
            with self.subTest(bad_raw=bad_raw):
                with self.assertRaises(cc.CannotCheck):
                    cc.parse_raw_diff_tree(bad_raw)

        # Non-regular modes (symlink, gitlink) and invalid UTF-8/TOML fail
        # closed without calling real git.
        self.assertEqual(
            cc.load_guarded(("120000", oid1)),
            (None, "non-regular file mode 120000"),
        )
        self.assertEqual(
            cc.load_guarded(("160000", oid1), side_label="parent"),
            (None, "non-regular file mode 160000 (parent)"),
        )
        with mock.patch.object(cc, "git_bytes", return_value=b"\xff\xfe"):
            m, err = cc.load_guarded(("100644", oid1))
            self.assertIsNone(m)
            self.assertIn("invalid UTF-8", err)
        with mock.patch.object(cc, "git_bytes", return_value=b"[broken\n"):
            m, err = cc.load_guarded(("100644", oid1))
            self.assertIsNone(m)
            self.assertIn("invalid TOML", err)

        # Replacing a non-exempt non-TOML symlink with a regular non-TOML file
        # reports the parent symlink without parsing the regular file as TOML.
        with mock.patch.object(cc, "git_bytes") as mocked_cat:
            findings = cc.pair_findings(
                "robot/subsystems", ("120000", oid1), ("100644", oid2))
        mocked_cat.assert_not_called()
        self.assertEqual(
            findings,
            [("robot/subsystems", "non-regular file mode 120000 (parent)")],
        )

    # REQ-DEV-33, RULES.md 4: `Commissioned-By:` exempts only its own commit,
    # and a blank trailer value does not exempt a commit.
    def test_commissioned_by_trailer_enforcement(self):
        c1 = "a" * 40
        c2 = "b" * 40
        c3 = "c" * 40
        base = "0" * 40
        trailers = {
            c1: "A Person\n",
            c2: "\n",
            c3: "   \n",
        }

        def fake_git(*args):
            if args[:2] == ("merge-base", base):
                return base + "\n"
            if args[:3] == ("rev-list", "--parents", "--reverse"):
                return f"{c1} {base}\n{c2} {c1}\n{c3} {c2}\n"
            if args[0] == "log":
                return trailers[args[-1]]
            raise AssertionError(f"unexpected git call: {args}")

        def fake_findings(_pinned, commit, _parents):
            return [(f"bench/{commit[:4]}.toml", "sets commissioned = true")]

        with mock.patch.object(cc, "pinned_git", return_value=()), \
                mock.patch.object(cc, "git", side_effect=fake_git), \
                mock.patch.object(cc, "commit_findings",
                                  side_effect=fake_findings):
            found = cc.check(base)

        self.assertEqual(
            found,
            [
                f"{c2[:12]} bench/{c2[:4]}.toml: sets commissioned = true",
                f"{c3[:12]} bench/{c3[:4]}.toml: sets commissioned = true",
            ],
        )


if __name__ == "__main__":
    unittest.main()
