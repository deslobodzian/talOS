"""`manifest_sync_test`: fails when a robot config's generated manifest is stale."""

load("@rules_python//python:defs.bzl", "py_test")

def manifest_sync_test(name, config, manifest, config_files, **kwargs):
    """Runs `//tools:sync_manifest --check` on `config`.

    Fails, printing the regenerate command and a diff, when `manifest` (the
    `<stem>_manifest.bzl` beside `config`) no longer matches it.

    Args:
      name: test target name.
      config: the robot config, e.g. "robot.toml".
      manifest: its generated manifest, e.g. "robot_manifest.bzl".
      config_files: the manifest's CONFIG_FILES. The test sees only these, so
        a config that reads a file they do not list fails as stale.
      **kwargs: passed to py_test.
    """
    config_label = native.package_relative_label(config)
    config_path = (config_label.package + "/" + config_label.name) if config_label.package else config_label.name
    py_test(
        name = name,
        size = kwargs.pop("size", "small"),
        srcs = [Label("//tools:sync_manifest.py")],
        main = Label("//tools:sync_manifest.py"),
        args = ["--check", config_path],
        data = [config, manifest] + list(config_files),
        legacy_create_init = False,
        **kwargs
    )
