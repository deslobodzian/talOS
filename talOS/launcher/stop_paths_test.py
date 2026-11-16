import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def line(ms, process, topic, text):
    """One stop_paths line, in the columns BENCH.md's trials read."""
    return f"{ms:12.3f} {process:<14} {topic:<26} {text}"


class StopPathsTest(unittest.TestCase):
    def run_tool(self, mode=None):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/stop_paths_fixture")
        binary = files.Rlocation("_main/talOS/launcher/stop_paths")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory] + ([mode] if mode else []), check=True)
            return subprocess.run([binary, directory], capture_output=True, text=True)

    # REQ-SAFE-07, REQ-SAFE-02 (bench evidence, docs/BENCH.md "Reading a
    # trial"): a recorded session of stop path 2 (teleop stopped 300 ms,
    # stick held) and stop path 1 (a Driver Station disable) reads as the
    # lines each trial's Pass column measures between: the lane's PAUSE and
    # the first neutral request, the disabling packet and State, the first
    # State at 0 V; and a log's lost records read as a LOG GAP, with the
    # PAUSE it causes, not as the robot's silence.
    def test_recorded_stop_paths(self):
        result = self.run_tool()
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        lane = "/motor/target/teleop"
        request = "/hw/request/motor"
        expected = [
            # Stop path 2: 150 - 50 = 100 ms from the PAUSE's start (the
            # lane's last message) to the first neutral request.
            line(155, "motor", request,
                 "observed_us=1155000 kraken=neutral 0 (previous value last at 150.000)"),
            line(156, "hardware_node", "/hw/command",
                 "observed_us=1155000 kraken=neutral 0 (previous value last at 151.000)"),
            line(350, "teleop", lane, "PAUSE 300.0 ms after 50.000"),
            line(350, "teleop", lane,
                 "{mode_since_us: 900000, active: false, message: "
                 "{run_index: 1, sequence_count: 2}} (previous value last at 50.000)"),
            # Stop path 1: the disabling packet, the first State with
            # enabled=False (already inactive), the first at 0 V.
            line(400, "hardware_node", "/hw/state/driver_station",
                 "enabled=False estop=False mode=teleop (previous value last at 0.000)"),
            line(400, "hardware_node", "/hw/state",
                 "sample_us=1400000 enabled=False active=False epoch=1 since_us=1400000 "
                 "kraken_v=1 kraken_rps=10 (previous value last at 395.000)"),
            line(410, "hardware_node", "/hw/state",
                 "sample_us=1410000 enabled=False active=False epoch=1 since_us=1410000 "
                 "kraken_v=0 kraken_rps=0 (previous value last at 405.000)"),
            # The motor log's lost requests, then the 45 ms they leave.
            f"{200:12.3f} {'motor':<14} LOG GAP to 240.000: 8 records lost (pool_full)",
            line(240, "motor", request, "PAUSE 45.0 ms after 195.000"),
        ]
        for want in expected:
            with self.subTest(line=want):
                self.assertIn(want, lines)
        # Only the gap and the stopped lane pause: no stream ticking every
        # 5 ms reads a PAUSE, and nothing is UNDECODED.
        self.assertEqual(sum("PAUSE" in text for text in lines), 2, result.stdout)
        self.assertNotIn("UNDECODED", result.stdout)

    # REQ-OPS-51: a session of another hardware protocol is refused, not read
    # with this build's layout.
    def test_refuses_undecoded_hardware_packets(self):
        result = self.run_tool("foreign-protocol")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(result.stdout, "")
        self.assertIn("stop_paths: hardware packets not decoded", result.stderr)


if __name__ == "__main__":
    unittest.main()
