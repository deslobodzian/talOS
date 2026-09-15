"""Fresh generated projects: generate, describe, codec, nonzero device ID (item 51).

Scaffolds real standalone modules with tools/node_scaffold (the exact
beginner-walkthrough entry point) into an isolated temp dir and executes
what can run hermetically:
  - generate: layout, MODULE minimality, subsystem/testdata contents
  - describe: the generated node's describe_source_rows() executes and
    matches packet.describe_sources() row-for-row (no shared memory: rows
    are pure data, the same rows --describe would print)
  - codec connect: generated packet encode/decode round-trips, and
    resolve_ids() against the generated testdata robot.toml returns the
    nonzero device ID the node would connect with (zero raises)

Full `bazel test //...` inside the fresh checkout needs module toolchains
from the network, so it stays a documented manual step (DEPLOYMENT.md);
what runs here fails first on any generator drift that would break it.
"""

import importlib.util
import os
import py_compile
import re
import sys
import tempfile
import unittest
from pathlib import Path

from scaffold import scaffold_node

FORBIDDEN_MODULE_SIBLINGS = (
    "talos_driver_station",
    "talos_operator_interface",
    "talos_odometry",
    "talos_telemetry",
    "talos_chassis_contracts",
)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


class FreshProjectTest(unittest.TestCase):
    def test_generate_python_hardware_layout(self):
        with tempfile.TemporaryDirectory(prefix="talos_fresh_py_") as tmp:
            # scaffold_node resolves out (macOS /tmp is a symlink), so
            # resolve here too before comparing subpaths.
            out = (Path(tmp) / "talos_tprobe").resolve()
            written = scaffold_node(name="tprobe", lang="python", out=out,
                                    hardware=True)
            names = {str(p.relative_to(out)) for p in written}
            for expected in ("MODULE.bazel", "BUILD", "packet.py", "node.py",
                             "node_test.py", "subsystem.toml",
                             "testdata/robot.toml", "testdata/tprobe.toml",
                             "tprobe_message.fbs", "test.sh", "README.md"):
                self.assertIn(expected, names, names)

    def test_module_bazel_is_minimal_sdk_only(self):
        # A minimal consumer pins talos + core toolchain pins only: platform
        # siblings are dev-only assembly deps and must not leak into the
        # generated MODULE (see external_consumer fixture).
        with tempfile.TemporaryDirectory(prefix="talos_fresh_mod_") as tmp:
            out = Path(tmp) / "talos_tprobe"
            scaffold_node(name="tprobe", lang="python", out=out, hardware=True)
            module = (out / "MODULE.bazel").read_text()
            self.assertIn('bazel_dep(name = "talos"', module)
            self.assertIn("local_path_override", module)
            for sibling in FORBIDDEN_MODULE_SIBLINGS:
                self.assertNotIn(sibling, module)

    def test_example_hardware_config_uses_nonzero_device_id(self):
        with tempfile.TemporaryDirectory(prefix="talos_fresh_cfg_") as tmp:
            out = Path(tmp) / "talos_tprobe"
            scaffold_node(name="tprobe", lang="python", out=out, hardware=True)
            subsys = (out / "subsystem.toml").read_text()
            m = re.search(r"can_id\s*=\s*(\d+)", subsys)
            self.assertIsNotNone(m, subsys)
            self.assertNotEqual(int(m.group(1)), 0)

    def test_generated_python_describes_codecs_and_resolves_id(self):
        with tempfile.TemporaryDirectory(prefix="talos_fresh_exec_") as tmp:
            out = Path(tmp) / "talos_tprobe"
            scaffold_node(name="tprobe", lang="python", out=out, hardware=True)
            for src in ("packet.py", "node.py"):
                py_compile.compile(str(out / src), doraise=True)
            sys.path.insert(0, str(out))
            try:
                packet = load_module("tprobe_packet", out / "packet.py")
                node = load_module("tprobe_node", out / "node.py")
            finally:
                sys.path.remove(str(out))
            # Describe: node rows match the packet's declared sources exactly.
            self.assertEqual(list(node.describe_source_rows()),
                             list(packet.describe_sources(hardware=True)))
            self.assertGreater(len(node.describe_source_rows()), 0)
            # Codec: generated struct codecs round-trip.
            wire = packet.encode_target(123456789, 12.5, True)
            self.assertEqual(len(wire), packet.TARGET_SIZE)
            issued_ns, vel, enabled = packet.decode_target(wire)
            self.assertEqual(issued_ns, 123456789)
            self.assertAlmostEqual(vel, 12.5)
            self.assertTrue(enabled)
            # Connect: the device ID resolves nonzero from the generated
            # testdata robot config; a zero ID raises instead of connecting.
            motor_id, period_us = node.resolve_ids(
                str(out / "testdata" / "robot.toml"))
            self.assertNotEqual(motor_id, 0)
            self.assertGreater(period_us, 0)
            with self.assertRaises(Exception):
                node.resolve_ids(str(out / "testdata" / "robot.toml"),
                                 subsystem="no_such_subsystem")

    def test_generate_cpp_hardware_topics(self):
        with tempfile.TemporaryDirectory(prefix="talos_fresh_cpp_") as tmp:
            out = Path(tmp) / "talos_tarm"
            scaffold_node(name="tarm", lang="cpp", out=out, hardware=True)
            packet_h = (out / "packet.h").read_text()
            self.assertIn("/tarm/state", packet_h)
            self.assertIn("/tarm/target", packet_h)
            module = (out / "MODULE.bazel").read_text()
            for sibling in FORBIDDEN_MODULE_SIBLINGS:
                self.assertNotIn(sibling, module)
            subsys = (out / "subsystem.toml").read_text()
            m = re.search(r"can_id\s*=\s*(\d+)", subsys)
            self.assertIsNotNone(m, subsys)
            self.assertNotEqual(int(m.group(1)), 0)

    def test_scaffold_refuses_to_clobber_and_rejects_bad_names(self):
        with tempfile.TemporaryDirectory(prefix="talos_fresh_neg_") as tmp:
            out = Path(tmp) / "talos_tprobe"
            scaffold_node(name="tprobe", lang="python", out=out, hardware=True)
            with self.assertRaises(FileExistsError):
                scaffold_node(name="tprobe", lang="python", out=out,
                              hardware=True)
            with self.assertRaises(ValueError):
                scaffold_node(name="NotSnake", lang="python",
                              out=Path(tmp) / "other", hardware=True)


if __name__ == "__main__":
    unittest.main()
