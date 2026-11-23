"""Unit tests for tools/check_commissioning.py on a scratch git repository.

REQ-DEV-33 (RULES.md 4): an agent's commit that sets or changes `commissioned`,
changes a CAN id or bus (including via table header swaps, multiline/inline
tables, or unicode-escaped keys), re-points `include` or subsystem config
paths/defaults, or introduces unparseable TOML in a robot config fails;
the same change in an example, or by a person's commit carrying
`Commissioned-By: <name>`, passes.
"""

import io
import os
import subprocess
import tempfile
import unittest
from unittest import mock

from tools import check_commissioning

MOTOR = "[motors.drive]\ntype = \"TalonFX\"\ncan_id = 1\n"
ROBOT = "[robot]\ncommissioned = false\n"


class CheckCommissioningTest(unittest.TestCase):
    def setUp(self):
        self.repo = tempfile.mkdtemp(dir=os.environ.get("TEST_TMPDIR"))
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith("GIT_")}
        self.env.update(GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1",
                        GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@example.com",
                        GIT_COMMITTER_NAME="t",
                        GIT_COMMITTER_EMAIL="t@example.com")
        self.git("init", "-q")
        for root in ("bench/main_processor/configuration", "examples/robot"):
            self.put(f"{root}/robot.toml", ROBOT)
            self.put(f"{root}/subsystems/motor.toml", MOTOR)
        self.commit("base")
        self.base = self.git("rev-parse", "HEAD").strip()
        cwd = os.getcwd()
        os.chdir(self.repo)
        self.addCleanup(os.chdir, cwd)

    def git(self, *args):
        return subprocess.run(["git", *args], cwd=self.repo, env=self.env,
                              capture_output=True, text=True,
                              check=True).stdout

    def put(self, path, text):
        full = os.path.join(self.repo, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "w", encoding="utf-8") as f:
            f.write(text)

    def commit(self, message, *trailers):
        self.git("add", "-A")
        args = ["commit", "-qm", message]
        for trailer in trailers:
            args += ["--trailer", trailer]
        self.git(*args)

    def edit_configs(self, root):
        self.put(f"{root}/robot.toml", "[robot]\ncommissioned = true\n")
        self.put(f"{root}/subsystems/motor.toml", MOTOR.replace("= 1", "= 7"))

    # REQ-DEV-33, RULES.md 4
    def test_robot_config_commissioning_or_can_id_change_fails(self):
        self.edit_configs("bench/main_processor/configuration")
        self.commit("bench: tidy the config")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 2, found)
        text = "\n".join(found)
        self.assertIn("bench/main_processor/configuration/robot.toml: "
                      "changes commissioned at robot.commissioned from false "
                      "to true", text)
        self.assertIn("motor.toml: changes a CAN id: motors.drive.can_id "
                      "from 1 to 7", text)

        # Removing or flipping commissioned back to false also fails.
        self.git("reset", "--hard", self.base)
        self.put("bench/main_processor/configuration/robot.toml", "[robot]\n")
        self.commit("bench: remove commissioned")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 1, found)
        self.assertIn("removes commissioned = false at robot.commissioned",
                      found[0])

    # REQ-DEV-33, RULES.md 4
    def test_trailer_covers_only_its_own_commit(self):
        self.edit_configs("bench/main_processor/configuration")
        self.commit("bench: commission at the robot",
                    "Commissioned-By: A Person")
        self.assertEqual(check_commissioning.check(self.base), [])
        # A later commit without the trailer is still checked.
        self.put("bench/main_processor/configuration/subsystems/motor.toml",
                 MOTOR.replace("= 1", "= 8"))
        self.commit("bench: renumber")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 1, found)
        self.assertIn("changes a CAN id: motors.drive.can_id from 7 to 8",
                      found[0])

    # REQ-DEV-33, RULES.md 4: every spelling the TOML parser accepts is seen.
    def test_dotted_inline_quoted_and_commented_keys_fail(self):
        config = "bench/main_processor/configuration/extra.toml"
        cases = (
            ("robot.commissioned = true", "sets commissioned = true"),
            ("commissioned=true", "sets commissioned = true"),
            ("\"commissioned\" = true  # at the robot",
             "sets commissioned = true"),
            ("drive = { type = \"TalonFX\", can_id = 3 }", "sets a CAN id"),
            ("'can_id' = 3", "sets a CAN id"),
            ("motors.drive.can_id=3 # moved", "sets a CAN id"),
            ("bus = \"Shooter\"", "sets a CAN bus"),
            ("bus=\"canivore\"", "sets a CAN bus"),
            ("motors.drive.bus = \"Shooter\" # moved", "sets a CAN bus"),
            ("drive = { type = \"TalonFX\", bus = \"Shooter\" }",
             "sets a CAN bus"),
            ("\"bus\" = \"Shooter\"", "sets a CAN bus"),
            ("'bus' = \"Shooter\"", "sets a CAN bus"),
        )
        for line, expected in cases:
            with self.subTest(line=line):
                self.git("reset", "--hard", self.base)
                self.put(config, f"{line}\n")
                self.commit("bench: respell")
                found = check_commissioning.check(self.base)
                self.assertIn(expected, "\n".join(found), found)

    # REQ-DEV-33, RULES.md 4: moving a device to another CAN bus changes its
    # physical address without touching can_id, including adding/removing bus.
    def test_can_bus_add_change_remove_and_merge_fail(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        # Adding bus moves the device off the default "rio" bus.
        self.put(motor, MOTOR + "bus = \"Shooter\"\n")
        self.commit("bench: add bus")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 1, found)
        self.assertIn("motor.toml: sets a CAN bus: motors.drive.bus = "
                      "\"Shooter\"", found[0])

        # Changing bus between two values reports the change.
        before = self.git("rev-parse", "HEAD").strip()
        self.put(motor, MOTOR + "bus = \"rio\"\n")
        self.commit("bench: move bus")
        found = check_commissioning.check(before)
        self.assertEqual(len(found), 1, found)
        self.assertIn("motor.toml: changes a CAN bus: motors.drive.bus from "
                      "\"Shooter\" to \"rio\"", found[0])

        # Removing bus falls back to default "rio" and still fails.
        before = self.git("rev-parse", "HEAD").strip()
        self.put(motor, MOTOR)
        self.commit("bench: remove bus")
        found = check_commissioning.check(before)
        self.assertEqual(len(found), 1, found)
        self.assertIn("motor.toml: removes a CAN bus: motors.drive.bus = "
                      "\"rio\"", found[0])

        # A bus edit introduced during merge resolution is caught too.
        before = self.git("rev-parse", "HEAD").strip()
        self.branch_and_merge(
            lambda: self.put("bench/main_processor/configuration/side.toml",
                             "b = 2\n"),
            lambda: self.put(motor, MOTOR + "bus = \"canivore\"\n"))
        merge = self.git("rev-parse", "HEAD").strip()[:12]
        found = check_commissioning.check(before)
        self.assertEqual(found, [
            f"{merge} {motor}: sets a CAN bus: motors.drive.bus = "
            "\"canivore\""], found)

    # REQ-DEV-33, RULES.md 4: TOML basic-string unicode escapes in double-quoted
    # keys decode before key matching, both when added and when removed.
    def test_unicode_escaped_quoted_keys_fail(self):
        config = "bench/main_processor/configuration/extra.toml"
        cases = (
            (r'"\u0063an_id" = 5', "sets a CAN id"),
            (r'"\U00000063an_id" = 6', "sets a CAN id"),
            (r'"\u0063an\u005Fid" = 7', "sets a CAN id"),
            (r'motors.drive."\u0063an_id" = 8', "sets a CAN id"),
            (r'"\u0063ommissioned" = true', "sets commissioned = true"),
            (r'"\U00000063ommissioned" = true', "sets commissioned = true"),
            (r'robot = { "\u0063ommissioned" = true }',
             "sets commissioned = true"),
            (r'"\u0062us" = "canivore"', "sets a CAN bus"),
            (r'"\U00000062us" = "rio"', "sets a CAN bus"),
            (r'drive = { type = "TalonFX", "\u0062us" = "Shooter" }',
             "sets a CAN bus"),
        )
        for line, expected in cases:
            with self.subTest(line=line):
                self.git("reset", "--hard", self.base)
                self.put(config, f"{line}\n")
                self.commit("bench: unicode escape key")
                found = check_commissioning.check(self.base)
                self.assertIn(expected, "\n".join(found), found)

        # Removing a unicode-escaped can_id or bus key is also caught.
        for line, expected in (
            (r'"\u0063an_id" = 5', "removes a CAN id"),
            (r'"\u0062us" = "canivore"', "removes a CAN bus"),
        ):
            with self.subTest(remove=line):
                self.git("reset", "--hard", self.base)
                self.put(config, f"{line}\n")
                self.commit("bench: seed", "Commissioned-By: A Person")
                before = self.git("rev-parse", "HEAD").strip()
                self.put(config, "ok = 1\n")
                self.commit("bench: remove escaped key")
                found = check_commissioning.check(before)
                self.assertIn(expected, "\n".join(found), found)

    # REQ-DEV-33, RULES.md 4: single-quoted literal keys do not process TOML
    # escapes and escaped backslashes stay literal (valid TOML, unrelated keys).
    def test_literal_single_quoted_escapes_pass(self):
        self.put(
            "bench/main_processor/configuration/robot.toml",
            ROBOT
            + r"'\u0063an_id' = 5" + "\n"
            + r"'\u0062us' = 'canivore'" + "\n"
            + r"'\u0063ommissioned' = true" + "\n"
            + "[other]\n"
            + r'"\\u0063an_id" = 5' + "\n",
        )
        self.commit("bench: literal escapes")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: a comment, string value or unrelated key naming
    # a guarded word is not a guarded key.
    def test_comment_string_and_unrelated_keys_pass(self):
        self.put("bench/main_processor/configuration/robot.toml",
                 ROBOT + "# set commissioned = true or bus = \"Shooter\"\n"
                 "note = \"can_id = 3, bus = Shooter\"\n"
                 "can_bus = \"rio\"\n"
                 "bus_name = \"Shooter\"\n"
                 "can_bus_count = 2\n"
                 "[subsystems.bus]\n")
        self.commit("bench: comment and unrelated keys")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: the developer's git configuration changes nothing.
    def test_no_prefix_diff_config_still_fails(self):
        self.git("config", "diff.noprefix", "true")
        self.edit_configs("bench/main_processor/configuration")
        self.commit("bench: tidy the config")
        self.assertEqual(len(check_commissioning.check(self.base)), 2)

    # REQ-DEV-33, RULES.md 4: git quotes a path with a non-ASCII byte and
    # tab-terminates one with a space; neither hides the config.
    def test_config_path_git_quotes_or_tab_terminates_fails(self):
        for root in ("bench/main_processor/configuración", "bench/robot two"):
            with self.subTest(root=root):
                self.put(f"{root}/motor.toml", MOTOR)
                self.commit("bench: add")
                before = self.git("rev-parse", "HEAD").strip()
                self.put(f"{root}/motor.toml", MOTOR.replace("= 1", "= 7"))
                self.commit("bench: renumber")
                found = check_commissioning.check(before)
                self.assertEqual(len(found), 1, found)
                self.assertIn(f" {root}/motor.toml: ", found[0])

    # REQ-DEV-33, RULES.md 4: a gitattributes `-diff` (binary) mark, which
    # the same commit can add, does not hide a changed line.
    def test_config_marked_binary_still_fails(self):
        self.put(".gitattributes", "*.toml -diff\n")
        self.edit_configs("bench/main_processor/configuration")
        self.commit("bench: tidy the config")
        self.assertEqual(len(check_commissioning.check(self.base)), 2)

    def branch_and_merge(self, side_edit, resolve=None):
        """Commits side_edit on a branch and an unrelated change here, then
        merges the branch; resolve() edits the merge result before its
        commit. Returns the branch's commit."""
        here = self.git("rev-parse", "--abbrev-ref", "HEAD").strip()
        self.git("checkout", "-q", "-b", "side")
        side_edit()
        self.commit("side: change")
        side = self.git("rev-parse", "HEAD").strip()
        self.git("checkout", "-q", here)
        self.put("bench/main_processor/configuration/notes.toml", "a = 1\n")
        self.commit("bench: notes")
        self.git("merge", "-q", "--no-ff", "--no-commit", "side")
        if resolve:
            resolve()
        self.commit("merge side")
        return side

    # REQ-DEV-33, RULES.md 4: a CAN id written while resolving a merge, in
    # neither parent, is the merge's own change.
    def test_can_id_added_by_a_merge_resolution_fails(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        self.branch_and_merge(
            lambda: self.put("bench/main_processor/configuration/side.toml",
                             "b = 2\n"),
            lambda: self.put(motor, MOTOR.replace("= 1", "= 9")))
        merge = self.git("rev-parse", "HEAD").strip()[:12]
        found = check_commissioning.check(self.base)
        self.assertEqual(found, [
            f"{merge} {motor}: changes a CAN id: motors.drive.can_id from "
            "1 to 9"], found)

    # REQ-DEV-33, RULES.md 4: a clean merge repeats no finding; the merged
    # branch's own commit is what is checked.
    def test_clean_merge_reports_only_the_branch_commit(self):
        side = self.branch_and_merge(
            lambda: self.edit_configs("bench/main_processor/configuration"))
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 2, found)
        for line in found:
            self.assertTrue(line.startswith(side[:12]), found)

    def merge_keeps_the_old_id(self, merge_main_into_branch):
        """A person renumbers a motor here (with the trailer) after branch
        `old` forked; a merge of the two keeps `old`'s copy of the file.
        Returns (the person's commit, the merge, the config)."""
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        here = self.git("rev-parse", "--abbrev-ref", "HEAD").strip()
        self.git("checkout", "-q", "-b", "old")
        self.put("bench/main_processor/configuration/side.toml", "b = 2\n")
        self.commit("old: unrelated")
        self.git("checkout", "-q", here)
        self.put(motor, MOTOR.replace("= 1", "= 5"))
        self.commit("bench: renumber at the robot",
                    "Commissioned-By: A Person")
        person = self.git("rev-parse", "HEAD").strip()
        if merge_main_into_branch:
            self.git("checkout", "-q", "old")
            self.git("merge", "-q", "--no-commit", here)
            self.git("checkout", "HEAD", "--", motor)
        else:
            self.git("checkout", "-q", "-b", "feature")
            self.git("merge", "-q", "--no-ff", "--no-commit", "old")
            self.git("checkout", "old", "--", motor)
        self.commit("merge")
        return person, self.git("rev-parse", "HEAD").strip()[:12], motor

    # REQ-DEV-33, RULES.md 4: a merge that takes a branch's stale copy of a
    # config reverts a person's CAN id; the revert is the merge's change.
    def test_merge_reverting_a_commissioned_can_id_fails(self):
        for main_into_branch in (False, True):
            with self.subTest(merge_main_into_branch=main_into_branch):
                self.setUp()
                person, merge, motor = self.merge_keeps_the_old_id(
                    main_into_branch)
                found = check_commissioning.check(person)
                self.assertEqual(found, [
                    f"{merge} {motor}: changes a CAN id: motors.drive.can_id "
                    "from 5 to 1"], found)

    # REQ-DEV-33, RULES.md 4: a `-merge` attribute committed on the branch
    # (a .gitattributes, which nothing else checks) does not hide a reverted id.
    def test_merge_attribute_does_not_hide_a_reverted_can_id(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        here = self.git("rev-parse", "--abbrev-ref", "HEAD").strip()
        self.git("checkout", "-q", "-b", "f")
        self.put(".gitattributes", "*.toml -merge\n")
        self.put(motor, MOTOR.replace("TalonFX", "Kraken"))
        self.commit("f: attributes and a type")
        self.git("checkout", "-q", here)
        self.put(motor, MOTOR.replace("= 1", "= 5"))
        self.commit("bench: renumber at the robot",
                    "Commissioned-By: A Person")
        person = self.git("rev-parse", "HEAD").strip()
        self.git("checkout", "-q", "f")
        subprocess.run(["git", "merge", "-q", "--no-commit", here],
                       cwd=self.repo, env=self.env, capture_output=True,
                       check=False)  # Conflicts: the file merges as binary.
        self.git("checkout", "--ours", "--", motor)
        self.commit("merge")
        found = check_commissioning.check(person)
        self.assertIn("changes a CAN id: motors.drive.can_id from 5 to 1",
                      "\n".join(found))

    # REQ-DEV-33, RULES.md 4: resolving a merge conflict where both parents
    # changed the same CAN id requires human commissioning sign-off.
    def test_merge_conflict_on_can_id_fails(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        here = self.git("rev-parse", "--abbrev-ref", "HEAD").strip()
        self.git("checkout", "-q", "-b", "side")
        self.put(motor, MOTOR.replace("= 1", "= 6"))
        self.commit("side: renumber", "Commissioned-By: A Person")
        self.git("checkout", "-q", here)
        self.put(motor, MOTOR.replace("= 1", "= 5"))
        self.commit("main: renumber", "Commissioned-By: A Person")
        person = self.git("rev-parse", "HEAD").strip()
        subprocess.run(["git", "merge", "-q", "--no-commit", "side"],
                       cwd=self.repo, env=self.env, capture_output=True,
                       check=False)
        self.put(motor, MOTOR.replace("= 1", "= 5"))
        self.commit("merge picking main")
        found = check_commissioning.check(person)
        self.assertEqual(len(found), 1, found)
        self.assertIn("resolves a merge conflict on a CAN id", found[0])

    # REQ-DEV-33, RULES.md 4: .git/info/attributes applies whatever the
    # attribute source, so a rule there fails the check closed.
    def test_local_info_attributes_fail_closed(self):
        info = os.path.join(self.repo, ".git", "info", "attributes")
        os.makedirs(os.path.dirname(info), exist_ok=True)
        with open(info, "w", encoding="utf-8") as f:
            f.write("# a comment is fine\n*.toml -merge\n")
        with mock.patch("sys.stderr", new_callable=io.StringIO) as err:
            self.assertEqual(check_commissioning.main(["--base", self.base]),
                             2)
        self.assertIn("info/attributes sets attributes", err.getvalue())

    # REQ-DEV-33, RULES.md 4: a git without --attr-source fails closed.
    def test_merge_under_a_git_without_remerge_diff_fails_closed(self):
        self.branch_and_merge(
            lambda: self.put("bench/main_processor/configuration/side.toml",
                             "b = 2\n"))
        real = check_commissioning.git

        def old_git(*args):
            return "git version 2.35.8\n" if args == ("version",) else real(
                *args)

        with mock.patch.object(check_commissioning, "git", old_git), \
                mock.patch("sys.stderr", new_callable=io.StringIO) as err:
            self.assertEqual(check_commissioning.main(["--base", self.base]),
                             2)
        self.assertIn("needs git 2.40 or later", err.getvalue())

    # REQ-DEV-33, RULES.md 4: git cannot re-merge an octopus merge, so one
    # that changes a robot config is reported rather than skipped.
    def test_octopus_merge_changing_a_config_fails(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        here = self.git("rev-parse", "--abbrev-ref", "HEAD").strip()
        for name in ("a", "b"):
            self.git("checkout", "-q", "-b", name, here)
            self.put(f"bench/main_processor/configuration/{name}.toml",
                     "x = 1\n")
            self.commit(name)
        self.git("checkout", "-q", here)
        self.git("merge", "-q", "--no-ff", "--no-commit", "a", "b")
        self.put(motor, MOTOR.replace("= 1", "= 9"))
        self.commit("merge a and b")
        merge = self.git("rev-parse", "HEAD").strip()[:12]
        found = check_commissioning.check(self.base)
        self.assertTrue(found, found)
        self.assertTrue(all(line.startswith(merge) for line in found), found)
        self.assertIn(motor, "\n".join(found))

    # REQ-DEV-33, RULES.md 4: examples carry placeholder ids.
    def test_example_config_passes(self):
        self.edit_configs("examples/robot")
        self.commit("examples: renumber the placeholder")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: multiline strings inside inline tables cannot
    # hide bus or can_id additions or later edits.
    def test_multiline_string_in_inline_table_hiding_can_id_and_bus_fails(self):
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        body = (
            "[subsystems.a.motors]\n"
            'lead = {type = """\n'
            'TalonFX""", bus = "Drive", can_id = 7}\n'
        )
        self.put(motor, body)
        self.commit("bench: add multiline inline table motor")
        found = check_commissioning.check(self.base)
        text = "\n".join(found)
        self.assertIn("subsystems.a.motors.lead.bus", text, found)
        self.assertIn("subsystems.a.motors.lead.can_id", text, found)

        # Once present, editing bus or can_id on line 2 must also fail.
        self.git("reset", "--hard", self.base)
        self.put(motor, body)
        self.commit("bench: seed multiline inline table",
                    "Commissioned-By: A Person")
        before = self.git("rev-parse", "HEAD").strip()
        self.put(motor, body.replace("can_id = 7", "can_id = 9"))
        self.commit("bench: edit line 2 can_id")
        found = check_commissioning.check(before)
        self.assertIn("subsystems.a.motors.lead.can_id", "\n".join(found),
                      found)

    # REQ-DEV-33, RULES.md 4: single-line triple-quoted strings inside inline
    # tables cannot hide bus or can_id keys.
    def test_single_line_triple_quoted_strings_in_inline_table_fail(self):
        config = "bench/main_processor/configuration/extra.toml"
        for line in (
            "lead = {note='''it's''', bus = \"Drive\", can_id = 7}",
            'lead = {note="""a"b""", bus = "Shooter", can_id = 8}',
        ):
            with self.subTest(line=line):
                self.git("reset", "--hard", self.base)
                self.put(config, f"[motors]\n{line}\n")
                self.commit("bench: triple-quoted inline table")
                found = check_commissioning.check(self.base)
                text = "\n".join(found)
                self.assertIn("motors.lead.bus", text, found)
                self.assertIn("motors.lead.can_id", text, found)

    # REQ-DEV-33, RULES.md 4: changing include or subsystem config pointers
    # re-points device configs and must fail; unrelated [teleop.defaults] passes.
    def test_changing_include_or_subsystem_paths_fails(self):
        robot = "bench/main_processor/configuration/robot.toml"
        motor = "bench/main_processor/configuration/subsystems/motor.toml"
        base_manifest = (
            'include = "base.toml"\n'
            + ROBOT
            + '\n[[subsystems]]\nname = "arm"\npath = "subsystems/arm.toml"\n'
            + 'defaults = "shared/arm_defaults.toml"\n'
            + '\n[[subsystems]]\nname = "wrist"\npath = "subsystems/wrist.toml"\n'
        )
        self.put(robot, base_manifest)
        self.put(motor,
                 '[subsystem]\nname = "motor"\ndefaults = "shared/d.toml"\n'
                 + MOTOR)
        self.commit("bench: seed manifest", "Commissioned-By: A Person")
        seeded = self.git("rev-parse", "HEAD").strip()

        cases = (
            ("change include", robot,
             base_manifest.replace('"base.toml"', '"other.toml"'), "include"),
            ("remove include", robot,
             base_manifest.replace('include = "base.toml"\n', ""), "include"),
            ("change subsystem path", robot,
             base_manifest.replace('"subsystems/arm.toml"',
                                   '"subsystems/arm_alt.toml"'),
             "subsystems[arm].path"),
            ("change subsystem defaults", robot,
             base_manifest.replace('"shared/arm_defaults.toml"',
                                   '"shared/other.toml"'),
             "subsystems[arm].defaults"),
            ("swap subsystem names", robot,
             base_manifest.replace('name = "arm"', 'name = "__tmp__"')
                          .replace('name = "wrist"', 'name = "arm"')
                          .replace('name = "__tmp__"', 'name = "wrist"'),
             "subsystems[arm].path"),
            ("change [subsystem] defaults", motor,
             '[subsystem]\nname = "motor"\ndefaults = "shared/alt.toml"\n'
             + MOTOR,
             "subsystem.defaults"),
        )
        for label, target, content, expected in cases:
            with self.subTest(case=label):
                self.git("reset", "--hard", seeded)
                self.put(target, content)
                self.commit(f"bench: {label}")
                found = check_commissioning.check(seeded)
                self.assertIn(expected, "\n".join(found), found)

        # Unrelated [teleop.defaults] table is not a config pointer.
        self.git("reset", "--hard", seeded)
        self.put("bench/main_processor/configuration/subsystems/teleop.toml",
                 '[subsystem]\nname = "teleop"\n'
                 '[teleop.defaults]\ndeadband = 0.05\n')
        self.commit("bench: add teleop defaults")
        self.assertEqual(check_commissioning.check(seeded), [])

    # REQ-DEV-33, RULES.md 4: swapping two device table headers remaps physical
    # CAN ids/buses without touching any can_id or bus line.
    def test_swapping_device_table_headers_fails(self):
        config = "bench/main_processor/configuration/subsystems/motor.toml"
        original = (
            "[subsystems.a.motors.lead]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
            'bus = "Drive"\n\n'
            "[subsystems.a.motors.follower]\n"
            'type = "TalonFX"\n'
            "can_id = 2\n"
            'bus = "Drive"\n'
        )
        self.put(config, original)
        self.commit("bench: seed two motors", "Commissioned-By: A Person")
        seeded = self.git("rev-parse", "HEAD").strip()

        swapped = (
            "[subsystems.a.motors.follower]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
            'bus = "Drive"\n\n'
            "[subsystems.a.motors.lead]\n"
            'type = "TalonFX"\n'
            "can_id = 2\n"
            'bus = "Drive"\n'
        )
        self.put(config, swapped)
        self.commit("bench: swap table headers")
        found = check_commissioning.check(seeded)
        text = "\n".join(found)
        self.assertIn("subsystems.a.motors.lead.can_id from 1 to 2", text,
                      found)
        self.assertIn("subsystems.a.motors.follower.can_id from 2 to 1", text,
                      found)

        # Same can_id on different buses (explicit vs default) swapped fails too.
        self.git("reset", "--hard", self.base)
        bus_orig = (
            "[subsystems.a.motors.lead]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
            'bus = "Drive"\n\n'
            "[subsystems.a.motors.follower]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
        )
        self.put(config, bus_orig)
        self.commit("bench: seed two buses", "Commissioned-By: A Person")
        seeded_bus = self.git("rev-parse", "HEAD").strip()
        bus_swap = (
            "[subsystems.a.motors.follower]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
            'bus = "Drive"\n\n'
            "[subsystems.a.motors.lead]\n"
            'type = "TalonFX"\n'
            "can_id = 1\n"
        )
        self.put(config, bus_swap)
        self.commit("bench: swap bus headers")
        found = check_commissioning.check(seeded_bus)
        text = "\n".join(found)
        self.assertIn("removes a CAN bus: subsystems.a.motors.lead.bus", text,
                      found)
        self.assertIn("sets a CAN bus: subsystems.a.motors.follower.bus", text,
                      found)

    # REQ-DEV-33, RULES.md 4: unparseable TOML, invalid UTF-8, or a symlink in
    # a robot config fails closed naming the file.
    def test_invalid_toml_and_symlink_fail_closed(self):
        config = "bench/main_processor/configuration/robot.toml"
        for label, raw in (
            ("unclosed header", b"[motors.drive\ncan_id = 1\n"),
            ("invalid escape", b'"\\uD800" = 1\n'),
            ("unknown escape", b'"\\q" = 1\n'),
            ("invalid utf-8", b"\xff\xfe[robot]\n"),
        ):
            with self.subTest(case=label):
                self.git("reset", "--hard", self.base)
                full = os.path.join(self.repo, config)
                with open(full, "wb") as f:
                    f.write(raw)
                self.commit(f"bench: broken {label}")
                found = check_commissioning.check(self.base)
                self.assertTrue(found, label)
                self.assertIn(config, found[0])
                self.assertIn("invalid TOML", found[0])

        # Symlinked .toml fails closed (could point to an exempt directory).
        self.git("reset", "--hard", self.base)
        link = os.path.join(self.repo,
                            "bench/main_processor/configuration/link.toml")
        os.symlink("../../../examples/robot/robot.toml", link)
        self.commit("bench: symlink config")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 1, found)
        self.assertIn("non-regular file mode 120000", found[0])

        # Invalid TOML under an exempt directory still passes.
        self.git("reset", "--hard", self.base)
        self.put("examples/robot/robot.toml", "[broken\n")
        self.commit("examples: broken")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: uppercase/mixed-case .toml extensions are checked,
    # while EXEMPT_DIRECTORIES stay exact-case so exemptions are not widened.
    def test_case_insensitive_toml_extensions_fail(self):
        # Use filenames that do not case-collide with setUp()'s robot.toml or
        # examples/ directory so the test exercises genuine uppercase/mixed
        # paths on case-insensitive filesystems (macOS APFS) as well as Linux.
        for rel in (
            "bench/main_processor/configuration/EXTRA.TOML",
            "bench/main_processor/configuration/Motor.Toml",
            "TESTS/robot/Config.TOML",
            "FIXTURES/robot/Config.TOML",
        ):
            with self.subTest(path=rel):
                self.git("reset", "--hard", self.base)
                self.put(rel, "[robot]\ncommissioned = true\n")
                self.commit(f"bench: add {rel}")
                found = check_commissioning.check(self.base)
                self.assertEqual(len(found), 1, found)
                self.assertIn(f" {rel}: sets commissioned = true", found[0])

        # Verify uppercase EXAMPLES/ is not exempted (remove lowercase
        # examples/ first so macOS APFS creates a real uppercase directory).
        self.git("reset", "--hard", self.base)
        self.git("rm", "-rq", "examples")
        self.commit("remove lowercase examples fixture")
        no_examples_base = self.git("rev-parse", "HEAD").strip()
        upper_ex = "EXAMPLES/robot/extra.toml"
        self.put(upper_ex, "[robot]\ncommissioned = true\n")
        self.commit(f"bench: add {upper_ex}")
        found = check_commissioning.check(no_examples_base)
        self.assertEqual(len(found), 1, found)
        self.assertIn(f" {upper_ex}: sets commissioned = true", found[0])

        # Editing a CAN id in a mixed-case .Toml file fails too.
        self.git("reset", "--hard", self.base)
        mixed = "bench/main_processor/configuration/subsystems/Drive.Toml"
        self.put(mixed, MOTOR)
        self.commit("bench: seed mixed-case config",
                    "Commissioned-By: A Person")
        seeded = self.git("rev-parse", "HEAD").strip()
        self.put(mixed, MOTOR.replace("= 1", "= 9"))
        self.commit("bench: edit mixed-case can_id")
        found = check_commissioning.check(seeded)
        self.assertEqual(len(found), 1, found)
        self.assertIn(f" {mixed}: changes a CAN id: motors.drive.can_id "
                      "from 1 to 9", found[0])

        # Exact-case exempt directory with uppercase .TOML still passes.
        self.git("reset", "--hard", self.base)
        self.put("examples/robot/EXTRA.TOML", "[robot]\ncommissioned = true\n")
        self.commit("examples: uppercase config")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: any non-exempt symlink (including a directory
    # symlink with no .toml suffix) fails closed; exempt symlinks pass.
    def test_non_exempt_directory_symlink_fails_closed(self):
        # Adding a non-exempt directory symlink without a .toml suffix fails.
        link_rel = "robot/subsystems"
        link_full = os.path.join(self.repo, link_rel)
        os.makedirs(os.path.dirname(link_full), exist_ok=True)
        os.symlink("../examples/robot/subsystems", link_full)
        self.commit("robot: symlink subsystems directory")
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 1, found)
        self.assertIn(f" {link_rel}: non-regular file mode 120000", found[0])

        # Replacing a non-exempt symlink with a regular non-TOML file fails
        # closed on the parent symlink without parsing the regular file as TOML.
        self.git("reset", "--hard", self.base)
        os.makedirs(os.path.dirname(link_full), exist_ok=True)
        os.symlink("../examples/robot/subsystems", link_full)
        self.commit("robot: seed symlink", "Commissioned-By: A Person")
        seeded = self.git("rev-parse", "HEAD").strip()
        os.unlink(link_full)
        self.put(link_rel, "not valid toml = = =\n")
        self.commit("robot: replace symlink with regular file")
        found = check_commissioning.check(seeded)
        self.assertEqual(len(found), 1, found)
        self.assertIn(f" {link_rel}: non-regular file mode 120000 (parent)",
                      found[0])

        # Introducing a non-exempt directory symlink in a merge fails closed.
        self.git("reset", "--hard", self.base)

        def add_merge_symlink():
            target = os.path.join(self.repo, "robot/subsystems")
            os.makedirs(os.path.dirname(target), exist_ok=True)
            os.symlink("../examples/robot/subsystems", target)

        self.branch_and_merge(
            lambda: self.put("bench/main_processor/configuration/side.toml",
                             "b = 2\n"),
            add_merge_symlink)
        merge = self.git("rev-parse", "HEAD").strip()[:12]
        found = check_commissioning.check(self.base)
        self.assertEqual(found, [
            f"{merge} robot/subsystems: non-regular file mode 120000"], found)

        # A directory symlink under an exempt directory still passes.
        self.git("reset", "--hard", self.base)
        exempt_link = os.path.join(self.repo, "examples/robot/subsystems_link")
        os.symlink("subsystems", exempt_link)
        self.commit("examples: symlink inside exempt dir")
        self.assertEqual(check_commissioning.check(self.base), [])


if __name__ == "__main__":
    unittest.main()
