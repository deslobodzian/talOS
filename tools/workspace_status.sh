#!/usr/bin/env bash
# Bazel --workspace_status_command for --config=release (.bazelrc): the
# source revision talos_bundle records in bundle_manifest.toml. Prints
# nothing outside a git checkout; a status command that fails fails the build.
revision="$(git rev-parse HEAD 2>/dev/null)" || exit 0
echo "STABLE_GIT_COMMIT ${revision}"
# Untracked files count: a new source file is part of the build.
if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
  echo "STABLE_GIT_DIRTY 1"
else
  echo "STABLE_GIT_DIRTY 0"
fi
