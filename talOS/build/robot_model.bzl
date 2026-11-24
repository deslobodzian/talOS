"""The freshness test for a robot's checked-in MuJoCo model.

The generator and its parameter file are described in
talOS/simulation/mujoco/robots/README.md.
"""

load("@rules_python//python:defs.bzl", "py_test")

_GENERATOR_SCRIPT = Label("//talOS/simulation/mujoco/robots:generate_robot.py")

def _workspace_path(file):
    label = native.package_relative_label(file)
    return (label.package + "/" + label.name) if label.package else label.name

def robot_model_sync_test(name, parameters, model, **kwargs):
    """Runs `generate_robot --check` on a robot's parameter file and model.

    Fails, printing the regenerate command and a diff, when `model` (the
    checked-in MJCF a config's `[simulation.mujoco] robot_model` names) is
    not what `parameters` generates.

    Args:
      name: test target name.
      parameters: the robot's parameter file, e.g. "robot_params.toml", a
        source file of the calling package.
      model: its generated model, e.g. "robot.xml", a source file of the
        calling package.
      **kwargs: passed to py_test.
    """
    py_test(
        name = name,
        size = kwargs.pop("size", "small"),
        srcs = [_GENERATOR_SCRIPT],
        main = _GENERATOR_SCRIPT,
        args = [
            "--check",
            "--parameters",
            _workspace_path(parameters),
            "--out",
            _workspace_path(model),
            # The canonical name; the test finds what the main repository
            # calls it in its runfiles' repository mapping.
            "--talos-repository=" + _GENERATOR_SCRIPT.repo_name,
        ],
        data = [parameters, model],
        legacy_create_init = False,
        **kwargs
    )
