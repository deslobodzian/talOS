"""Build and test gates for a robot config's generated manifest freshness."""

load("@rules_python//python:defs.bzl", "py_test")

def manifest_freshness_stamp(ctx, config, manifest, config_files, tool):
    """Registers a check-mode action and returns its success stamp.

    Args:
      ctx: bundle rule context.
      config: robot TOML File.
      manifest: generated manifest File beside config.
      config_files: Files listed by CONFIG_FILES.
      tool: sync_manifest FilesToRunProvider, including its Python runfiles.

    Returns:
      A stamp File that a bundle action must consume.
    """
    stamp = ctx.actions.declare_file(ctx.label.name + ".manifest_current")

    # Manifest labels and generated comments are relative to the config's own
    # repository, including when a standalone robot loads @talos's macros.
    workspace_root = ctx.attr.config.label.workspace_root
    config_path = config.path
    if workspace_root:
        config_path = config_path.removeprefix(workspace_root + "/")
    ctx.actions.run_shell(
        inputs = depset([config, manifest] + config_files),
        tools = [tool],
        outputs = [stamp],
        command = (
            'root="$PWD"; tool="$root/$1"; config="$2"; stamp="$root/$3"; ' +
            'cd "$root/$4" && "$tool" --check "$config" && : > "$stamp"'
        ),
        arguments = [tool.executable.path, config_path, stamp.path, workspace_root],
        mnemonic = "TalosManifestCheck",
        progress_message = "Checking config manifest for %{label}",
    )
    return stamp

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
        data = [config, manifest, Label("//talOS/configuration:resolve")] + list(config_files),
        deps = [Label("@rules_python//python/runfiles")],
        legacy_create_init = False,
        **kwargs
    )
