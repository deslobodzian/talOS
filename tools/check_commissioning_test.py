"""Unit tests for tools/check_commissioning.py on a scratch git repository.

REQ-DEV-33 (RULES.md 4): an agent's commit that sets
`commissioned = true` or changes a CAN id in a robot config fails; the same
change in an example, or by a person's commit carrying
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
        self.assertEqual(len(found), 3, found)
        text = "\n".join(found)
        self.assertIn("bench/main_processor/configuration/robot.toml:2: "
                      "sets commissioned = true", text)
        self.assertIn("motor.toml:3: removes a CAN id: can_id = 1", text)
        self.assertIn("motor.toml:3: sets a CAN id: can_id = 7", text)

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
        self.assertEqual(len(found), 2, found)

    # REQ-DEV-33, RULES.md 4: every spelling the TOML parser accepts is seen.
    def test_dotted_inline_quoted_and_commented_keys_fail(self):
        config = "bench/main_processor/configuration/robot.toml"
        for line in ("robot.commissioned = true",
                     "commissioned=true",
                     "\"commissioned\" = true  # at the robot",
                     "drive = { type = \"TalonFX\", can_id = 3 }",
                     "'can_id' = 3",
                     "motors.drive.can_id=3 # moved"):
            with self.subTest(line=line):
                before = self.git("rev-parse", "HEAD").strip()
                self.put(config, f"{line}\n")
                self.commit("bench: respell")
                self.assertTrue(check_commissioning.check(before), line)

    # REQ-DEV-33, RULES.md 4: a comment or string naming the key is not the key.
    def test_comment_mentioning_the_key_passes(self):
        self.put("bench/main_processor/configuration/robot.toml",
                 ROBOT + "# set commissioned = true at the robot\n"
                 "note = \"can_id = 3\"\n")
        self.commit("bench: comment")
        self.assertEqual(check_commissioning.check(self.base), [])

    # REQ-DEV-33, RULES.md 4: the developer's git configuration changes nothing.
    def test_no_prefix_diff_config_still_fails(self):
        self.git("config", "diff.noprefix", "true")
        self.edit_configs("bench/main_processor/configuration")
        self.commit("bench: tidy the config")
        self.assertEqual(len(check_commissioning.check(self.base)), 3)

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
        self.assertEqual(sorted(found), [
            f"{merge} {motor}:3: removes a CAN id: can_id = 1",
            f"{merge} {motor}:3: sets a CAN id: can_id = 9"], found)

    # REQ-DEV-33, RULES.md 4: a clean merge repeats no finding; the merged
    # branch's own commit is what is checked.
    def test_clean_merge_reports_only_the_branch_commit(self):
        side = self.branch_and_merge(
            lambda: self.edit_configs("bench/main_processor/configuration"))
        found = check_commissioning.check(self.base)
        self.assertEqual(len(found), 3, found)
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
                self.assertEqual(sorted(found), [
                    f"{merge} {motor}:3: removes a CAN id: can_id = 5",
                    f"{merge} {motor}:3: sets a CAN id: can_id = 1"], found)

    # REQ-DEV-33, RULES.md 4: a git without --remerge-diff cannot tell a
    # merge's own change, so the check fails closed instead of skipping it.
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
        self.assertIn("needs git 2.36 or later", err.getvalue())

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


if __name__ == "__main__":
    unittest.main()
