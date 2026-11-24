"""generate_robot: a bad parameter file is refused naming its key, a name a
hardware config accepts is accepted, the model carries the parameter file's
scripted-shooter values, the output is the same bytes whatever the
interpreter's hash seed, and --check passes on a current model and fails on
an edited parameter."""

import copy
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stderr, redirect_stdout

from talOS.simulation.mujoco.robots import generate_robot

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = os.path.join(HERE, "examples")


def example(name):
    with open(os.path.join(EXAMPLES, name + ".toml"), "rb") as f:
        return tomllib.load(f)


def _set(path, value):
    def edit(p):
        *parents, last = path
        for k in parents:
            p = p[k]
        p[last] = value
    return edit


def _delete(path):
    def edit(p):
        *parents, last = path
        for k in parents:
            p = p[k]
        del p[last]
    return edit


class RefusalTest(unittest.TestCase):
    # (what, example, edit, the key the refusal names)
    CASES = [
        ("unknown key", "shooter_rig", _set(["flywheel", "colour"], "red"),
         "flywheel.colour"),
        ("missing key", "shooter_rig", _delete(["flywheel", "radius_m"]),
         "flywheel.radius_m"),
        ("not a number", "shooter_rig", _set(["frame", "width_m"], "wide"),
         "frame.width_m"),
        ("bumper above the piece's equator", "shooter_rig",
         _set(["bumpers", "bottom_m"], 0.08), "bumpers.bottom_m"),
        ("nine preloads", "shooter_rig",
         lambda p: p["preload"]["positions_m"].append([0.25, 0.18, 0.32]),
         "preload.positions_m"),
        ("a preload in the hopper floor", "shooter_rig",
         _set(["preload", "positions_m", 1, 2], 0.15),
         "preload.positions_m[1]"),
        ("two preloads overlapping", "shooter_rig",
         _set(["preload", "positions_m", 7], [0.065, -0.06, 0.353]),
         "preload.positions_m[7]"),
        ("no room for the pocket behind the feeder", "shooter_rig",
         _set(["feeder", "x_m"], -0.25), "feeder.x_m"),
        ("an intake wider than the hopper", "swerve_robot",
         _set(["intake", "width_m"], 0.6), "intake.width_m"),
        ("a step without an intake", "shooter_rig",
         _set(["hopper", "step_m"], 0.05), "hopper.step_m"),
        ("a wall too low for the fence", "swerve_robot",
         _set(["hopper", "wall_top_m"], 0.33), "hopper.wall_top_m"),
        ("a floor that falls below the belly pan", "shooter_rig",
         _set(["hopper", "floor_slope_deg"], 30.0), "hopper.floor_slope_deg"),
        ("an exit inside the hood", "shooter_rig",
         _set(["flywheel", "exit_offset_m"], 0.0), "flywheel.exit_offset_m"),
        ("a name with a newline", "shooter_rig",
         _set(["beam_breaks", 0, "name"], "intake\nbeam"),
         "beam_breaks[0].name"),
        ("a preload outside the hopper", "shooter_rig",
         _set(["preload", "positions_m", 0], [2.5, 0.0, 0.3]),
         "preload.positions_m[0]"),
        ("a beam named as a motor", "shooter_rig",
         _set(["beam_breaks", 1, "name"], "flywheel"), "beam_breaks[1].name"),
        ("two modules at one position", "swerve_robot",
         _set(["swerve", "modules", 1, "y_m"], 0.30), "swerve.modules[1]"),
        ("two wheels overlapping", "swerve_robot",
         _set(["swerve", "modules", 1, "y_m"], 0.22), "swerve.modules[1]"),
        ("two swerve modules", "swerve_robot",
         lambda p: p["swerve"]["modules"].__delitem__(slice(2, None)),
         "swerve.modules"),
        ("modules on one line", "swerve_robot",
         lambda p: [m.update(y_m=0.0, x_m=x) for m, x in
                    zip(p["swerve"]["modules"], (0.3, 0.1, -0.1, -0.3))],
         "swerve.modules"),
        ("two modules of one name", "swerve_robot",
         _set(["swerve", "modules", 1, "name"], "front_left"),
         "swerve.modules[1].name"),
        ("a motor no part drives", "shooter_rig",
         _set(["motors", "climber"], {"gear": 1.0,
                                      "rotor_inertia_kg_m2": 1e-4}),
         "motors.climber"),
        ("a part naming an undeclared motor", "shooter_rig",
         _set(["flywheel", "motor"], "shooter"), "flywheel.motor"),
        ("parts heavier than the robot", "swerve_robot",
         _set(["robot", "mass_kg"], 5.0), "robot.mass_kg"),
        ("a welded root with swerve", "swerve_robot",
         _set(["robot", "root"], "welded"), "swerve"),
        ("hood start at exit end", "shooter_rig",
         _set(["flywheel", "hood_start_deg"], 30.0),
         "flywheel.hood_start_deg"),
        ("hood start past exit end", "shooter_rig",
         _set(["flywheel", "hood_start_deg"], 45.0),
         "flywheel.hood_start_deg"),
        ("hopper wall below floor front", "shooter_rig",
         lambda p: p["hopper"].update(
             floor_front_m=0.80, floor_slope_deg=40.0, wall_top_m=0.70),
         "hopper.wall_top_m"),
        ("belly clearance at chassis box top", "shooter_rig",
         _set(["frame", "belly_clearance_m"], 0.30),
         "frame.belly_clearance_m"),
        ("non-finite mass", "shooter_rig",
         _set(["robot", "mass_kg"], math.nan), "robot.mass_kg"),
        ("negative motor gear", "shooter_rig",
         _set(["motors", "flywheel", "gear"], -1.0), "motors.flywheel.gear"),
        ("rotor armature overflow", "shooter_rig",
         _set(["motors", "flywheel", "gear"], 1e200), "motors.flywheel"),
        ("preload underground below hopper floor", "shooter_rig",
         _set(["preload", "positions_m", 0], [0.05, 0.0, -0.5]),
         "preload.positions_m[0]"),
        ("XML non-character in name", "shooter_rig",
         _set(["beam_breaks", 0, "name"], "beam\ufffe"),
         "beam_breaks[0].name"),
        ("hopper too narrow for feeder roller", "shooter_rig",
         lambda p: (
             p["game_piece"].update(radius_m=0.002),
             p["bumpers"].update(bottom_m=0.001),
             p["hopper"].update(width_m=0.01),
             p["feeder"].update(compression_m=0.001),
             p["flywheel"].update(compression_m=0.001),
         ), "hopper.width_m"),
    ]

    # REQ-DEV-41: a parameter file the generator cannot honour is refused,
    # naming the key to fix.
    def test_each_bad_file_is_refused_naming_its_key(self):
        for what, name, edit, key in self.CASES:
            with self.subTest(what):
                params = copy.deepcopy(example(name))
                edit(params)
                with self.assertRaises(generate_robot.ParameterError) as err:
                    generate_robot.generate(params, name + ".toml")
                self.assertEqual(err.exception.key, key, str(err.exception))


class NameTest(unittest.TestCase):

    # REQ-DEV-41: one actuator per named motor: a motor or input name a
    # hardware config accepts (non-empty, at most 63 bytes) names the
    # model's actuator and rangefinder unchanged.
    def test_names_a_config_accepts_name_the_model(self):
        params = copy.deepcopy(example("shooter_rig"))
        params["motors"]["shooter-Left &1"] = params["motors"].pop("flywheel")
        params["flywheel"]["motor"] = "shooter-Left &1"
        params["beam_breaks"][0]["name"] = "intakeBeam"
        model = ET.fromstring(generate_robot.generate(params, "x.toml"))
        self.assertEqual(
            [m.get("name") for m in model.iter("motor")],
            ["shooter-Left &1", "roller"])
        self.assertIn("intakeBeam",
                      [r.get("name") for r in model.iter("rangefinder")])

    def test_source_name_with_comment_metacharacters_emits_valid_xml(self):
        params = copy.deepcopy(example("shooter_rig"))
        text = generate_robot.generate(params, "robot--v2-->evil\n.toml")
        header, body = text.split("\n", 1)
        self.assertTrue(header.startswith("<!-- "))
        self.assertTrue(header.endswith(" -->"))
        self.assertNotIn("--", header[4:-3])
        model = ET.fromstring(text)
        self.assertEqual(model.tag, "mujoco")
        self.assertEqual(ET.fromstring(body).tag, "mujoco")

    def test_tiny_positives_and_huge_beam_direction_stay_positive_finite(self):
        params = copy.deepcopy(example("shooter_rig"))
        params["flywheel"]["efficiency"] = 1e-13
        params["beam_breaks"][0]["direction"] = [1.5e308, 1.5e308, 0.0]
        params["beam_breaks"][0]["cutoff_m"] = 1e-13
        model = ET.fromstring(generate_robot.generate(params, "x.toml"))
        numerics = {n.get("name"): n.get("data")
                    for n in model.iter("numeric")}
        efficiency, _ = map(float, numerics["flywheel_shooter"].split())
        self.assertGreater(efficiency, 0.0)
        ranges = {r.get("name"): float(r.get("cutoff"))
                  for r in model.iter("rangefinder")}
        self.assertGreater(ranges["intake_beam"], 0.0)
        sites = {s.get("name"): s for s in model.iter("site")}
        zaxis = tuple(map(float, sites["intake_beam"].get("zaxis").split()))
        self.assertAlmostEqual(math.hypot(*zaxis), 1.0, places=5)

        tiny_piece = copy.deepcopy(example("shooter_rig"))
        tiny_piece.pop("preload", None)
        tiny_piece["game_piece"]["radius_m"] = 1e-13
        tiny_piece["bumpers"]["bottom_m"] = 0.0
        tiny_piece["feeder"]["compression_m"] = 0.0
        tiny_piece["flywheel"]["compression_m"] = 0.0
        tiny_model = ET.fromstring(generate_robot.generate(tiny_piece, "y.toml"))
        tiny_sites = {s.get("name"): s for s in tiny_model.iter("site")}
        inlet_half = tuple(
            map(float, tiny_sites["flywheel_inlet"].get("size").split())
        )
        self.assertTrue(all(v > 0.0 for v in inlet_half), inlet_half)


class ContractTest(unittest.TestCase):

    # REQ-DEV-41: the scripted-shooter contract carries the parameter file's
    # efficiency and wheel radius, in that order, and its exit's x axis
    # leaves at the parameter file's elevation over the robot's rear. Read
    # from the TOML and the generated XML, not through the generator's code.
    def test_shooter_contract_carries_the_parameters(self):
        for name in ("swerve_robot", "shooter_rig"):
            with self.subTest(name):
                flywheel = example(name)["flywheel"]
                with open(os.path.join(EXAMPLES, name + ".xml"),
                          encoding="utf-8") as f:
                    model = ET.fromstring(f.read().split("\n", 1)[1])
                motor = flywheel["motor"]
                numeric = {n.get("name"): n.get("data")
                           for n in model.iter("numeric")}
                efficiency, radius = map(float,
                                         numeric[motor + "_shooter"].split())
                self.assertAlmostEqual(efficiency, flywheel["efficiency"])
                self.assertAlmostEqual(radius, flywheel["radius_m"])
                sites = {s.get("name"): s for s in model.iter("site")}
                x, _, z = map(float,
                              sites[motor + "_exit"].get("xyaxes").split()[:3])
                self.assertLess(x, 0.0)  # over the rear
                self.assertAlmostEqual(
                    math.degrees(math.atan2(z, -x)),
                    flywheel["exit_elevation_deg"], places=4)
                self.assertEqual(sites[motor + "_inlet"].get("type"), "box")


class OutputTest(unittest.TestCase):

    def run_generator(self, parameters, out, hash_seed):
        env = dict(os.environ, PYTHONHASHSEED=str(hash_seed))
        env.pop("BUILD_WORKSPACE_DIRECTORY", None)
        subprocess.run([sys.executable, os.path.join(HERE, "generate_robot.py"),
                        "--parameters", parameters, "--out", out],
                       env=env, check=True, capture_output=True)
        with open(out, "rb") as f:
            return f.read()

    # REQ-DEV-41
    def test_same_bytes_under_two_hash_seeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ("swerve_robot", "shooter_rig"):
                parameters = os.path.join(EXAMPLES, name + ".toml")
                first = self.run_generator(
                    parameters, os.path.join(tmp, name + "_1.xml"), 1)
                second = self.run_generator(
                    parameters, os.path.join(tmp, name + "_2.xml"), 2)
                self.assertEqual(first, second, name)

    # REQ-DEV-41: a robot's checked-in model fails its sync test when it no
    # longer matches its parameter file.
    def test_check_fails_on_an_edited_parameter(self):
        with tempfile.TemporaryDirectory() as tmp:
            parameters = os.path.join(tmp, "robot_params.toml")
            model = os.path.join(tmp, "robot.xml")
            # copyfile: the contents without the mode, since a checkout's
            # tracked files are read-only under CI and prepush.
            shutil.copyfile(
                os.path.join(EXAMPLES, "shooter_rig.toml"), parameters)
            args = ["--parameters", parameters, "--out", model]
            quiet = io.StringIO()
            with redirect_stdout(quiet), redirect_stderr(quiet):
                self.assertEqual(generate_robot.main(args), 0)
                self.assertEqual(generate_robot.main(["--check"] + args), 0)
            with open(parameters, encoding="utf-8") as f:
                text = f.read()
            with open(parameters, "w", encoding="utf-8") as f:
                f.write(text.replace("efficiency = 0.5", "efficiency = 0.55"))
            err = io.StringIO()
            with redirect_stdout(quiet), redirect_stderr(err):
                self.assertEqual(generate_robot.main(["--check"] + args), 1)
            self.assertIn("generate_robot -- --parameters", err.getvalue())

    def test_out_in_a_missing_directory_is_one_error_line(self):
        err = io.StringIO()
        with redirect_stderr(err):
            status = generate_robot.main([
                "--parameters", os.path.join(EXAMPLES, "shooter_rig.toml"),
                "--out", os.path.join(tempfile.gettempdir(), "missing_dir",
                                      "robot.xml")])
        self.assertEqual(status, 1)
        self.assertEqual(len(err.getvalue().splitlines()), 1)

    def test_check_reports_non_utf8_model_as_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            parameters = os.path.join(tmp, "robot_params.toml")
            model = os.path.join(tmp, "robot.xml")
            shutil.copyfile(
                os.path.join(EXAMPLES, "shooter_rig.toml"), parameters)
            with open(model, "wb") as f:
                f.write(b"\xff\xfe\x00not-utf8")
            err = io.StringIO()
            with redirect_stderr(err):
                status = generate_robot.main([
                    "--check", "--parameters", parameters, "--out", model])
            self.assertEqual(status, 1)
            self.assertIn("UTF-8", err.getvalue())
            self.assertIn("generate_robot -- --parameters", err.getvalue())


if __name__ == "__main__":
    unittest.main()
