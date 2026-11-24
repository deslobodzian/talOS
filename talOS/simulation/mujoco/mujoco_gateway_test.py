"""//talOS/simulation/mujoco:mujoco_gateway as a child process (REQ-DEV-37).

Its --describe against sim_gateway --physics's, its refusals before the
socket opens, a model reading a file robot_model_files omits, a diverging
model, and a short run on the 2026 field.
"""

import json
import os
import shutil
import socket
import subprocess
import tempfile
import unittest
from pathlib import Path

from python.runfiles import runfiles
from talOS.build.udp_ports import udp_ports

_RUNFILES = runfiles.Create()


def _path(variable):
    return Path(_RUNFILES.Rlocation(os.environ[variable]))


def _run(*args, timeout=30):
    return subprocess.run([str(_path("MUJOCO_GATEWAY")), *args],
                          capture_output=True, text=True, timeout=timeout)


_ROBOT = """[robot]
name = "mujoco_gateway_test"
period_us = 5000
command_timeout_us = 100000
controller_address = "127.0.0.1"

[subsystems.frame]
"""

# A flywheel carrying its inlet and numeric but no `flywheel_exit` site: a
# scripted shooter's contract, partly.
_INLET_ONLY = """<mujoco><compiler angle="radian"/><worldbody>
  <body name="frame" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
    <site name="flywheel_inlet" type="box" size="0.1 0.1 0.1"/>
    <body name="wheel">
      <joint name="flywheel_joint" type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.0025"/>
    </body>
  </body>
</worldbody>
<custom><numeric name="flywheel_shooter" data="0.5 0.0508"/></custom>
<actuator><motor name="flywheel" joint="flywheel_joint"/></actuator>
</mujoco>
"""

# A hinge sprung 1e12 N*m/rad towards 1 rad on 1e-3 kg*m^2: the explicit
# spring diverges within a few 5 ms steps.
_DIVERGING = """<mujoco><compiler angle="radian"/><worldbody>
  <body name="post" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="flap">
      <joint name="flap_joint" type="hinge" axis="0 0 1" stiffness="1e12"
             springref="1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="1e-3 1e-3 1e-3"/>
    </body>
  </body>
</worldbody></mujoco>
"""


class MujocoGatewayTest(unittest.TestCase):

    # REQ-DEV-37, REQ-DEV-40: the same roster member as sim_gateway
    # --physics (name, /sim/state, schema) plus /sim/state/match, so the
    # launcher and the graph lint see the program's label and the match.
    def test_describes_as_sim_gateway_physics_with_the_match(self):
        mujoco = _run("--describe")
        self.assertEqual(mujoco.returncode, 0, mujoco.stderr)
        plain = subprocess.run(
            [str(_path("SIM_GATEWAY")), "--describe", "--physics"],
            capture_output=True, text=True, timeout=30)
        self.assertEqual(plain.returncode, 0, plain.stderr)
        mujoco_json = json.loads(mujoco.stdout)
        plain_json = json.loads(plain.stdout)
        self.assertEqual(mujoco_json.pop("target"),
                         "//talOS/simulation/mujoco:mujoco_gateway")
        plain_json.pop("target")
        match = [source for source in mujoco_json["sources"]
                 if source["name"] == "/sim/state/match"]
        self.assertEqual(len(match), 1, mujoco_json["sources"])
        self.assertEqual(match[0]["schema"], "sim.MatchState")
        self.assertEqual(match[0]["kind"], "SENDER")
        mujoco_json["sources"].remove(match[0])
        self.assertEqual(mujoco_json, plain_json)

    # REQ-DEV-37: refused before the socket opens, exit 1, naming the key or
    # flag and the motor or input: the extension's own refusals, one the
    # backend makes when the gateway binds the plant, and one the plant's
    # Bind makes.
    def test_refuses_naming_the_key_or_flag(self):
        arm = (
            '[subsystems.arm.motors.arm]\ncan_id = 1\n'
            'sensor_to_mechanism_ratio = 50.0\n')
        mujoco = '\n[simulation.mujoco]\nrobot_model = "arm.xml"\n'
        cases = (
            ("no [simulation.mujoco]", _ROBOT, None,
             ["[simulation.mujoco] robot_model"]),
            ("wheel slip with a free root", None,
             ["--physics", "--wheel-slip-gain", "0.2"], ["--wheel-slip-gain"]),
            ("backend: a mechanism on a bound motor",
             _ROBOT + arm + '[[simulation.mechanisms]]\nmotor = "arm"\n'
             'inertia_kg_m2 = 0.1\n' + mujoco, None,
             ["[[simulation.mechanisms]]", "arm"]),
            ("plant: an input on a missing sensor",
             _ROBOT + arm + '[subsystems.arm.digital_inputs.arm_home]\n'
             'dio = 0\nsim_sensor = "missing_sensor"\n' + mujoco, None,
             ["arm_home", "missing_sensor"]),
            ("a scripted shooter's inlet without its exit",
             _ROBOT + '[subsystems.frame.motors.flywheel]\ncan_id = 1\n'
             + mujoco, None, ["flywheel_exit"]),
        )
        for what, toml, flags, names in cases:
            with self.subTest(what), tempfile.TemporaryDirectory() as tmp:
                if toml is None:
                    config = _path("ROVER_CONFIG")
                else:
                    if "flywheel" in toml:
                        (Path(tmp) / "arm.xml").write_text(
                            _INLET_ONLY, encoding="utf-8")
                    else:
                        shutil.copy(_path("ARM_XML"), Path(tmp) / "arm.xml")
                    config = Path(tmp) / "robot.toml"
                    config.write_text(toml, encoding="utf-8")
                port, remote = udp_ports()
                proc = _run("--config", str(config), *(flags or []),
                            "--port", str(port), "--remote-port", str(remote),
                            "--duration-s", "5")
                self.assertEqual(proc.returncode, 1,
                                 proc.stdout + proc.stderr)
                for name in names:
                    self.assertIn(name, proc.stderr)
                self.assertNotIn("active_ticks=", proc.stdout)

    # REQ-DEV-37: a model file the config does not list is absent where only
    # the listed files are (a talos_sim_test's runfiles), and the program
    # exits 1 naming it; from the source tree, where the file is, the same
    # config runs. Config load cannot see it (MuJoCo opens the file), so
    # this is the only place it fails.
    def test_unlisted_model_file_fails_where_only_listed_files_are(self):
        source = _path("UNLISTED_MESH_CONFIG")
        port, remote = udp_ports()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.bind(("127.0.0.1", remote))
            present = _run("--config", str(source), "--port", str(port),
                           "--remote-port", str(remote),
                           "--duration-s", "0.02")
        self.assertEqual(present.returncode, 0,
                         present.stdout + present.stderr)
        with tempfile.TemporaryDirectory() as tmp:
            for name in ("robot.toml", "robot.xml"):  # what the config lists
                shutil.copy(source.parent / name, Path(tmp) / name)
            port, remote = udp_ports()
            proc = _run("--config", str(Path(tmp) / "robot.toml"),
                        "--port", str(port), "--remote-port", str(remote),
                        "--duration-s", "5")
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        self.assertIn("part.obj", proc.stderr)

    # REQ-DEV-37: a world that diverges ends the program: exit 1, the MuJoCo
    # warning on stderr, and no summary (the outputs went neutral with the
    # gateway's total loss first).
    def test_diverging_model_exits_naming_the_warning(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "robot.xml").write_text(_DIVERGING, encoding="utf-8")
            config = Path(tmp) / "robot.toml"
            config.write_text(
                _ROBOT + '\n[simulation.mujoco]\nrobot_model = "robot.xml"\n',
                encoding="utf-8")
            port, remote = udp_ports()
            proc = _run("--config", str(config), "--port", str(port),
                        "--remote-port", str(remote), "--duration-s", "10")
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        self.assertRegex(proc.stderr, r"mjWARN_BAD(QPOS|QVEL|QACC)")
        self.assertIn("flap_joint", proc.stderr)
        self.assertNotIn("active_ticks=", proc.stdout)
        self.assertNotIn("stepped_s=", proc.stdout)

    # REQ-DEV-37: a free-root robot on the 2026 field (504 FUEL) runs for
    # 50 ms with --physics given and ends normally, having printed its
    # binding table and the plant's clock.
    def test_short_run_on_the_field(self):
        port, remote = udp_ports()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.bind(("127.0.0.1", remote))
            proc = _run("--config", str(_path("ROVER_CONFIG")), "--physics",
                        "--port", str(port), "--remote-port", str(remote),
                        "--duration-s", "0.05")
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        self.assertIn("roller", proc.stderr)  # its binding table row
        self.assertIn("stepped_s=", proc.stdout)
        self.assertIn("overspeed_excursions=0", proc.stdout)


if __name__ == "__main__":
    unittest.main()
