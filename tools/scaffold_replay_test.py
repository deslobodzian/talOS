"""integration: a scaffolded node's binary records a simulated run with --log
and replays that log with --replay without divergence. One target per
language runs it (BUILD: scaffold_replay_cc_test on the gripper golden,
scaffold_replay_py_test on scaffold_py), so the generated node tests run
the node's class only and spawn no binary (TODO.md TEST-5).

NODE_BINARY is the golden node's runfiles path; its config is the golden
package's testdata/robot.toml beside it."""

import os
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles

# A one-second simulated run of a single node, recorded, then replayed:
# a bound on a hang, not on a run.
RUN_TIMEOUT_S = 30


class ScaffoldReplayTest(unittest.TestCase):
    def run_node(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["NODE_BINARY"])
        config = os.path.join(os.path.dirname(binary), "testdata",
                              "robot.toml")
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)  # the node's own, not this test's
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv, "--config", config],
                                  cwd=cwd, env=env, capture_output=True,
                                  text=True, timeout=RUN_TIMEOUT_S)
        output = proc.stdout + proc.stderr
        self.assertEqual(proc.returncode, 0, output)
        return output

    # REQ-OPS-13
    def test_logged_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node.tlog")
            self.run_node("--sim", "--duration-s", "1", "--log", log)
            self.assertIn("diverged=0 clean_exit=1",
                          self.run_node("--replay", log))


if __name__ == "__main__":
    unittest.main()
