#!/usr/bin/env bash
# Bazel --workspace_status_command for --config=release (.bazelrc): the
# source revision talos_bundle records in bundle_manifest.toml. When run
# from a separate robot repository against a talOS git checkout, also
# records talOS's revision. Prints nothing outside a git checkout; a status
# command that fails fails the build.

canon_dir() {
  (cd "$1" 2>/dev/null && pwd -P) || return 1
}

script_path="${BASH_SOURCE[0]}"
hops=0
while [ -L "$script_path" ] && [ "$hops" -lt 40 ]; do
  hops=$((hops + 1))
  link_dir="$(cd "$(dirname "$script_path")" 2>/dev/null && pwd -P)" || break
  target="$(readlink "$script_path")" || break
  case "$target" in
    /*) script_path="$target" ;;
    *) script_path="$link_dir/$target" ;;
  esac
done
TALOS_DIR="$(cd "$(dirname "$script_path")/.." && pwd -P)"

pwd_top=""
if revision="$(git rev-parse HEAD 2>/dev/null)"; then
  echo "STABLE_GIT_COMMIT ${revision}"
  # Untracked files count: a new source file is part of the build.
  # Fail closed to dirty if `git status` itself fails.
  if ! status_out="$(git status --porcelain 2>/dev/null)" || [ -n "$status_out" ]; then
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
  if ! talos_status="$(git -C "$TALOS_DIR" status --porcelain 2>/dev/null)" || [ -n "$talos_status" ]; then
    echo "STABLE_TALOS_GIT_DIRTY 1"
  else
    echo "STABLE_TALOS_GIT_DIRTY 0"
  fi
fi
exit 0
