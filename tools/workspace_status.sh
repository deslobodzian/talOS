#!/usr/bin/env bash
# Bazel --workspace_status_command for --config=release (.bazelrc): the
# source revision talos_bundle records in bundle_manifest.toml. When run
# from a separate robot repository against a talOS git checkout, also
# records talOS's revision. Prints nothing outside a git checkout; a status
# command that fails fails the build.

canon_dir() {
  (cd "$1" 2>/dev/null && pwd -P) || return 1
}

TALOS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

pwd_top=""
if revision="$(git rev-parse HEAD 2>/dev/null)"; then
  echo "STABLE_GIT_COMMIT ${revision}"
  # Untracked files count: a new source file is part of the build.
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    echo "STABLE_GIT_DIRTY 1"
  else
    echo "STABLE_GIT_DIRTY 0"
  fi
  if raw_pwd_top="$(git rev-parse --show-toplevel 2>/dev/null)"; then
    pwd_top="$(canon_dir "$raw_pwd_top" || true)"
  fi
fi

if raw_talos_top="$(git -C "$TALOS_DIR" rev-parse --show-toplevel 2>/dev/null)" \
  && talos_top="$(canon_dir "$raw_talos_top")" \
  && [ "$talos_top" = "$TALOS_DIR" ] \
  && [ "$talos_top" != "$pwd_top" ] \
  && talos_rev="$(git -C "$TALOS_DIR" rev-parse HEAD 2>/dev/null)"; then
  echo "STABLE_TALOS_GIT_COMMIT ${talos_rev}"
  if [ -n "$(git -C "$TALOS_DIR" status --porcelain 2>/dev/null)" ]; then
    echo "STABLE_TALOS_GIT_DIRTY 1"
  else
    echo "STABLE_TALOS_GIT_DIRTY 0"
  fi
fi
exit 0
