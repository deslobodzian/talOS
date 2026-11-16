# Sourced by tools/check_robot_repo_scaffold.sh and
# tools/check_robot_repository.sh: one run at a time owns a fixed scratch
# path (so every run reuses one Bazel output base). The caller defines
# die().
#
#   take_copy_lock LOCK WHAT   mkdir LOCK, or take it from a dead holder;
#                              dies naming WHAT when a live run holds it.
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
