"""//talOS/simulation/mujoco:sim_gateway, the MuJoCo gateway, as a child
process (REQ-DEV-37).

Its --describe against sim_gateway --physics's, its refusals before the
socket opens, a model reading a file robot_model_files omits, a diverging
model, and a short run on the 2026 field.
"""

import json
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

from _posixshmem import shm_unlink
from python.runfiles import runfiles
from talOS.build.udp_ports import udp_ports

_RUNFILES = runfiles.Create()

# talOS/protocol/frame.h and talOS/protocol/types.h.
_FRAME_MAGIC = 0x4F4952534F4C4154
_FRAME_HEADER_BYTES = 40
_FRAME_TYPE_HARDWARE_STATE = 20

# talOS/hardware/sim_rings.h kPiecesSlots and MAX_SLOTS.
_PIECES_SLOTS = 8
_DEFAULT_SLOTS = 1024

# Shared-memory segments //talOS/simulation/mujoco:sim_gateway can create.
_GATEWAY_SHM_TOPICS = (
    "/sim/state",
    "/sim/state/match",
    "/sim/telemetry/pieces",
    "/talos_registry.6",
)

# Hang bounds only (docs/TESTING.md §6.3): 300 x 50 ms = 15 s without a UDP
# frame, and 15 s for SIGINT shutdown.
_RECV_POLL_S = 0.05
_MAX_IDLE_POLLS = 300
_STOP_TIMEOUT_S = 15
_HANG_DURATION_S = "10"


def _shm_name(topic, namespace):
    """Matches talos::ShmObjectName (talOS/memory/shared_memory_ptr.h)."""
    if topic.startswith("/~"):
        return topic
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    if not namespace:
        return mapped
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"


def _unlink_shm(topic, namespace):
    try:
        shm_unlink(_shm_name(topic, namespace))
    except FileNotFoundError:
        pass


def _isolated_env(test_case, namespace):
    """Isolate SHM segments and registry lock files in `namespace` and unlink
    every segment the gateway creates when the test finishes."""
    for topic in _GATEWAY_SHM_TOPICS:
        test_case.addCleanup(_unlink_shm, topic, namespace)
    env = os.environ.copy()
    # Unset TALOS_SHM_NAMESPACE and set TEST_TMPDIR so both ShmObjectName and
    # LockDirectory() (talOS/introspection/lock_file.h) use `namespace`.
    env.pop("TALOS_SHM_NAMESPACE", None)
    env["TEST_TMPDIR"] = namespace
    return env


def _bind_peer_and_port():
    """Bind a loopback UDP receiver at port 0 (no TOCTOU) and pick a distinct
    gateway listen port below the OS ephemeral range."""
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        peer.bind(("127.0.0.1", 0))
        remote = peer.getsockname()[1]
        p0, p1 = udp_ports()
        port = p0 if p0 != remote else p1
        return peer, port, remote
    except BaseException:
        peer.close()
        raise


def _is_hardware_state_frame(packet):
    if len(packet) < _FRAME_HEADER_BYTES:
        return False
    magic = struct.unpack_from("<Q", packet, 0)[0]
    frame_type = struct.unpack_from("<H", packet, 12)[0]
    return magic == _FRAME_MAGIC and frame_type == _FRAME_TYPE_HARDWARE_STATE


def _path(variable):
    resolved = _RUNFILES.Rlocation(os.environ[variable])
    if resolved is None:
        raise RuntimeError(f"runfile unavailable for {variable}")
    return Path(resolved)


def _run(*args, env=None, timeout=30):
    return subprocess.run([str(_path("MUJOCO_GATEWAY")), *args],
                          env=env, capture_output=True, text=True,
                          timeout=timeout)


def _run_for_ticks(args, env, work_dir, min_state_frames):
    """Run the MuJoCo gateway until `min_state_frames` State datagrams arrive
    (event-driven, avoiding fixed --duration-s races under host stalls), then
    stop it cleanly with SIGINT and return (returncode, stdout, stderr,
    state_frames)."""
    out_path = Path(work_dir) / "gateway.stdout"
    err_path = Path(work_dir) / "gateway.stderr"
    peer, port, remote = _bind_peer_and_port()
    state_frames = 0
    with peer, open(out_path, "w+", encoding="utf-8") as out_f, \
            open(err_path, "w+", encoding="utf-8") as err_f:
        peer.settimeout(_RECV_POLL_S)
        proc = subprocess.Popen(
            [str(_path("MUJOCO_GATEWAY")), *args,
             "--port", str(port), "--remote-port", str(remote),
             "--duration-s", _HANG_DURATION_S],
            env=env, stdout=out_f, stderr=err_f)
        try:
            idle = 0
            received = 0
            while (state_frames < min_state_frames and
                   idle < _MAX_IDLE_POLLS and received < 20000):
                try:
                    packet = peer.recv(4096)
                except socket.timeout:
                    idle += 1
                    if proc.poll() is not None:
                        break
                    continue
                idle = 0
                received += 1
                if _is_hardware_state_frame(packet):
                    state_frames += 1
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=_STOP_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        out_f.seek(0)
        err_f.seek(0)
        return proc.returncode, out_f.read(), err_f.read(), state_frames


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

    # REQ-DEV-37, REQ-DEV-40, REQ-DEV-43: the same roster member as
    # sim_gateway --physics (name, /sim/state, schema) plus /sim/state/match
    # and /sim/telemetry/pieces, so the launcher and the graph lint see the
    # program's label, the match and the game pieces.
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
                         "//talOS/simulation/mujoco:sim_gateway")
        plain_json.pop("target")
        match = [source for source in mujoco_json["sources"]
                 if source["name"] == "/sim/state/match"]
        self.assertEqual(len(match), 1, mujoco_json["sources"])
        self.assertEqual(match[0]["schema"], "sim.MatchState")
        self.assertEqual(match[0]["kind"], "SENDER")
        self.assertEqual(match[0]["slots"], _DEFAULT_SLOTS)
        mujoco_json["sources"].remove(match[0])
        pieces = [source for source in mujoco_json["sources"]
                  if source["name"] == "/sim/telemetry/pieces"]
        self.assertEqual(len(pieces), 1, mujoco_json["sources"])
        self.assertEqual(pieces[0]["schema"], "sim.GamePieces")
        self.assertEqual(pieces[0]["kind"], "SENDER")
        self.assertEqual(pieces[0]["slots"], _PIECES_SLOTS)
        mujoco_json["sources"].remove(pieces[0])
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
            ("wheel radius error without --physics implies --physics", None,
             ["--wheel-radius-error", "0.1"],
             ["--wheel-radius-error", "freejoint"]),
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
                peer, port, remote = _bind_peer_and_port()
                with peer:
                    proc = _run(
                        "--config", str(config), *(flags or []),
                        "--port", str(port), "--remote-port", str(remote),
                        "--duration-s", "5",
                        env=_isolated_env(self, tmp))
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
        with tempfile.TemporaryDirectory() as tmp:
            rc, stdout, stderr, frames = _run_for_ticks(
                ["--config", str(source)],
                env=_isolated_env(self, tmp),
                work_dir=tmp,
                min_state_frames=2)
        self.assertEqual(rc, 0, stdout + stderr)
        self.assertGreaterEqual(frames, 2, stdout + stderr)
        with tempfile.TemporaryDirectory() as tmp:
            for name in ("robot.toml", "robot.xml"):  # what the config lists
                shutil.copy(source.parent / name, Path(tmp) / name)
            peer, port, remote = _bind_peer_and_port()
            with peer:
                proc = _run("--config", str(Path(tmp) / "robot.toml"),
                            "--port", str(port), "--remote-port", str(remote),
                            "--duration-s", "5",
                            env=_isolated_env(self, tmp))
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
            peer, port, remote = _bind_peer_and_port()
            with peer:
                proc = _run("--config", str(config), "--port", str(port),
                            "--remote-port", str(remote), "--duration-s", "10",
                            env=_isolated_env(self, tmp))
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        self.assertRegex(proc.stderr, r"mjWARN_BAD(QPOS|QVEL|QACC)")
        self.assertIn("flap_joint", proc.stderr)
        self.assertNotIn("active_ticks=", proc.stdout)
        self.assertNotIn("stepped_s=", proc.stdout)

    # REQ-DEV-37: a free-root robot on the 2026 field (504 FUEL) runs for at
    # least 10 ticks (50 ms) with --physics given and ends normally, having
    # printed its binding table, the plant's clock with positive stepped time,
    # and on its backend line the start pose its config's
    # [simulation.start_pose] sets.
    def test_short_run_on_the_field(self):
        source = _path("ROVER_CONFIG")
        with tempfile.TemporaryDirectory() as tmp:
            shutil.copy(source.parent / "rover.xml", Path(tmp) / "rover.xml")
            config = Path(tmp) / "robot.toml"
            config.write_text(
                source.read_text(encoding="utf-8")
                + "\n[simulation.start_pose]\nx_m = 1.907\ny_m = 5.302\n"
                "yaw_rad = 2.705\n", encoding="utf-8")
            rc, stdout, stderr, frames = _run_for_ticks(
                ["--config", str(config), "--physics"],
                env=_isolated_env(self, tmp),
                work_dir=tmp,
                min_state_frames=10)
        self.assertEqual(rc, 0, stdout + stderr)
        self.assertGreaterEqual(frames, 10, stdout + stderr)
        self.assertIn("backend=mujoco start_pose=1.907,5.302,2.705 (config)",
                      stderr)
        self.assertIn("roller", stderr)  # its binding table row
        self.assertIn("sim_gateway: active_ticks=", stdout)
        stepped = re.search(r"stepped_s=(\d+\.\d+)", stdout)
        self.assertIsNotNone(stepped, stdout)
        self.assertGreater(float(stepped.group(1)), 0.0, stdout)
        self.assertIn("overspeed_excursions=0", stdout)


if __name__ == "__main__":
    unittest.main()
