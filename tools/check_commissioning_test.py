"""Unit tests for tools/check_commissioning.py on a scratch git repository.

REQ-DEV-33 (RULES.md 4): an agent's commit that sets
`commissioned = true` or changes a CAN id in a robot config fails; the same
change in an example, or by a person's commit carrying
`Commissioned-By: <name>`, passes.
"""

import os
import subprocess
import tempfile
import unittest

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

    # REQ-DEV-33, RULES.md 4: examples carry placeholder ids.
    def test_example_config_passes(self):
        self.edit_configs("examples/robot")
        self.commit("examples: renumber the placeholder")
        self.assertEqual(check_commissioning.check(self.base), [])


if __name__ == "__main__":
    unittest.main()
