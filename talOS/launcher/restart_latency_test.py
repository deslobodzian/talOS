import subprocess
import tempfile
import unittest

from python.runfiles import runfiles
from talOS.launcher import restart_latency


class RestartStagesTest(unittest.TestCase):
    def test_stages_of_a_recorded_restart(self):
        # The fixture's real LogWriter output: run() is the tick deadline
        # less one period, and the request counts from the first tick even
        # when that tick sends nothing.
        fixture = runfiles.Create().Rlocation("_main/talOS/launcher/restart_latency_fixture")
        with tempfile.TemporaryDirectory() as directory:
            path = f"{directory}/drivetrain.restart1.tlog"
            subprocess.run([fixture, path], check=True)
            log = restart_latency.read_tlog(path)
        stages = restart_latency.restart_stages(log, 996_000_000, "drivetrain")
        expected = {"kill_to_origin": 4.0, "origin_to_run": 1.5, "run_to_first_tick": 5.1,
                    "first_tick_to_request": 5.0, "total": 15.6}
        self.assertEqual(stages.keys(), expected.keys())
        for key, value in expected.items():
            self.assertAlmostEqual(stages[key], value, places=6, msg=key)
        with self.assertRaisesRegex(ValueError, "no /hw/request/intake send"):
            log["sources"][0]["name"] = "intake"
            restart_latency.restart_stages(log, 0, "intake")


if __name__ == "__main__":
    unittest.main()
