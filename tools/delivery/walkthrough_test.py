"""Executable docs (item 59): the walkthrough and deployment commands in
docs/NODE_DEVELOPMENT_GUIDE.md and docs/DEPLOYMENT.md are tested, not read.

  - every beginner-walkthrough scaffold command executes for real (the
    scaffold binary under test, output rewritten to an isolated temp dir)
  - every CLI flag named in either doc exists in the real binary's --help
    (scaffold, launcher) or the real script's argparse (readiness,
    compat_record) / subcommand list (deploy.sh)
  - every TOML snippet in DEPLOYMENT.md is marked example vs competition

A renamed flag or a stale command fails here instead of in the pit.
"""

import os
import re
import subprocess
import sys
import tempfile
import unittest

import compat_record
import readiness
from python.runfiles import runfiles

_R = runfiles.Create()


def rloc(key):
    return _R.Rlocation(os.environ[key])


SCAFFOLD_BIN = rloc("SCAFFOLD_BIN")
LAUNCHER_BIN = rloc("LAUNCHER_BIN")
DEPLOY_SH = rloc("DEPLOY_SH")
GUIDE_MD = rloc("GUIDE_MD")
DEPLOYMENT_MD = rloc("DEPLOYMENT_MD")


def code_blocks(markdown):
    # Fences anchored at line starts: a bare closing ``` must never parse
    # as an opening fence and swallow the block that follows it.
    return re.findall(r"^```(?:bash|sh|toml|starlark|python)\n(.*?)^```",
                      markdown, re.DOTALL | re.MULTILINE)


def command_lines(markdown):
    cmds = []
    for block in code_blocks(markdown):
        for line in block.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            line = re.sub(r"^\$\s*", "", line)
            cmds.append(line)
    return cmds


def help_text(binary, *args):
    proc = subprocess.run([binary, *args], capture_output=True, text=True,
                          timeout=120)
    return proc.returncode, proc.stdout + proc.stderr


class WalkthroughTest(unittest.TestCase):
    def test_scaffold_help_lists_documented_flags(self):
        rc, text = help_text(sys.executable, SCAFFOLD_BIN, "--help")
        self.assertEqual(rc, 0, text)
        for flag in ("--name", "--lang", "--hardware", "--out", "--sdk-path",
                     "--force"):
            self.assertIn(flag, text)

    def test_walkthrough_scaffold_commands_execute(self):
        # The exact scaffold invocations from the beginner walkthrough, with
        # only --out rewritten to an isolated temp dir. Template lines with
        # <placeholders> document the CLI shape; only concrete commands run.
        guide = Path_read(GUIDE_MD)
        ran = 0
        with tempfile.TemporaryDirectory(prefix="talos_walk_") as tmp:
            for cmd in command_lines(guide):
                if "tools/node_scaffold/main.py" not in cmd:
                    continue
                if "<" in cmd:
                    continue
                argv = self._rewrite_out(cmd, tmp, ran)
                proc = subprocess.run(
                    [sys.executable, SCAFFOLD_BIN, *argv],
                    capture_output=True, text=True, timeout=120)
                self.assertEqual(proc.returncode, 0, f"{cmd}\n{proc.stderr}")
                ran += 1
        self.assertGreater(ran, 0, "walkthrough scaffold command not found")

    def _rewrite_out(self, cmd, tmp, idx):
        # Drop the `python3 tools/.../main.py` head; keep flags only.
        argv = cmd.split("tools/node_scaffold/main.py", 1)[1].split()
        # Support `&&` chains: only the scaffold invocation itself.
        if "&&" in argv:
            argv = argv[:argv.index("&&")]
        out = [i for i, p in enumerate(argv) if p == "--out"]
        self.assertTrue(out, cmd)
        argv[out[0] + 1] = os.path.join(tmp, f"walk_{idx}")
        return argv

    def test_launcher_flags_in_docs_exist(self):
        rc, text = help_text(LAUNCHER_BIN, "--help")
        self.assertEqual(rc, 0, text)
        for md in (Path_read(GUIDE_MD), Path_read(DEPLOYMENT_MD)):
            for cmd in command_lines(md):
                if "launcher" not in cmd:
                    continue
                tail = cmd.split("--", 1)[-1] if " -- " in f" {cmd}" else cmd
                for flag in re.findall(r"--[\w-]+", tail):
                    if flag == "--":
                        continue
                    self.assertIn(flag, text, f"{flag} from: {cmd}")

    def test_deploy_subcommands_in_docs_exist(self):
        source = Path_read(DEPLOY_SH)
        for md in (Path_read(DEPLOYMENT_MD),):
            for cmd in command_lines(md):
                if "deploy.sh" not in cmd:
                    continue
                m = re.search(r"deploy\.sh\s+([\w-]+)", cmd)
                self.assertIsNotNone(m, cmd)
                sub = m.group(1)
                self.assertTrue(
                    f"{sub})" in source or f"cmd_{sub}" in source,
                    f"deploy.sh subcommand {sub!r} from: {cmd}")

    def test_python_tool_flags_in_docs_exist(self):
        modules = {"readiness.py": readiness.__file__,
                   "compat_record.py": compat_record.__file__}
        for md in (Path_read(DEPLOYMENT_MD),):
            for cmd in command_lines(md):
                for tool, path in modules.items():
                    if tool not in cmd:
                        continue
                    with open(path) as f:
                        source = f.read()
                    for flag in re.findall(r"--[\w-]+", cmd):
                        self.assertIn(flag, source, f"{flag} from: {cmd}")

    def test_deployment_toml_is_marked_example_or_competition(self):
        doc = Path_read(DEPLOYMENT_MD)
        self.assertIn("example", doc.lower())
        self.assertIn("competition", doc.lower())
        blocks = re.findall(
            r"^```toml\n(.*?)^```", doc, re.DOTALL | re.MULTILINE)
        self.assertGreater(len(blocks), 0)
        cursor = 0
        for block in blocks:
            start = doc.find(block, cursor)
            cursor = start + len(block)
            context = doc[max(0, start - 600):start].lower()
            self.assertTrue("example" in context or "competition" in context,
                            f"unmarked TOML block: {block[:80]!r}")


def Path_read(path):
    with open(path) as f:
        return f.read()


if __name__ == "__main__":
    unittest.main()
