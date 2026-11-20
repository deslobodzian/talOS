# Sourced by tools/check_robot_repo_scaffold.sh and
# tools/check_robot_repository.sh, which each build a copy of a robot
# repository against this talOS checkout. The caller defines die().
#
#   take_copy_lock LOCK WHAT   mkdir LOCK, or take it from a dead holder;
#                              dies naming WHAT when a live run holds it.
#                              One run at a time owns a fixed scratch path
#                              (so every run reuses one Bazel output base).
#   point_rc_at_talos RC FROM TALOS
#                              rewrites the copy's .bazelrc RC so that its
#                              `import %workspace%/FROM/talos.bazelrc` and a
#                              `--workspace_status_command=FROM/tools/...`
#                              name the checkout TALOS instead (FROM: where
#                              the repository keeps talOS, `talos` for the
#                              submodule, `../..` for examples/robot_repo);
#                              otherwise the copy would read the flags of the
#                              talOS it pins, or of an empty submodule
#                              directory. Returns 1 when RC has no such
#                              import (RC is then unchanged but for the
#                              status command), 2 when it cannot rewrite RC.
#
# The pid file appears whole (written beside it, then renamed), as in
# tools/prepush.sh, so a lock without one is being taken, or its run was
# killed while taking it.
take_copy_lock() {
  local lock="$1" what="$2" holder
  if ! mkdir "$lock" 2>/dev/null; then
    [ -f "$lock/pid" ] ||
      die "$lock has no pid yet: another check is starting (if none is, rmdir $lock)"
    holder="$(cat "$lock/pid" 2>/dev/null || true)"
    if [ -n "$holder" ] && kill -0 "$holder" 2>/dev/null; then
      die "another check (pid $holder) is using $what"
    fi
    rm -rf "$lock"
    mkdir "$lock" 2>/dev/null || die "cannot take $lock"
  fi
  { echo $$ > "$lock/pid.$$" && mv -f "$lock/pid.$$" "$lock/pid"; } ||
    die "cannot write $lock/pid"
}

point_rc_at_talos() {
  local rc="$1" from to found=1
  # FROM as a sed pattern, TALOS as a sed replacement, `|` delimited.
  from="$(printf '%s' "$2" | sed 's/[][\.*^$|]/\\&/g')"
  to="$(printf '%s' "$3" | sed 's/[\&|]/\\&/g')"
  [ -f "$rc" ] || return 1
  grep -qxF "import %workspace%/$2/talos.bazelrc" "$rc" && found=0
  sed -e "s|^import %workspace%/$from/talos\\.bazelrc\$|import $to/talos.bazelrc|" \
    -e "s|--workspace_status_command=$from/tools/|--workspace_status_command=$to/tools/|" \
    "$rc" > "$rc.new" && mv "$rc.new" "$rc" || return 2
  return "$found"
}

# Bazel 8's Linux sandbox mounts a private tmpfs over /tmp
# (--incompatible_sandbox_hermetic_tmp), hiding any local_path_override or
# --override_module checkout under /tmp (such as tools/prepush.sh's worktree).
# Prints the --sandbox_add_mount_pair flag needed to expose TALOS inside the
# sandbox when running on Linux with TALOS under /tmp; prints nothing otherwise.
sandbox_mount_talos() {
  local talos_dir="$1"
  if [ "$(uname -s)" = Linux ]; then
    case "$talos_dir" in
      /tmp/*) printf '%s\n' "--sandbox_add_mount_pair=$talos_dir" ;;
    esac
  fi
}
