# Sourced by tools/prepush.sh: whether its robot-repository step runs, is
# skipped or fails, from TALOS_ROBOT_REPOSITORY (REQ-DEV-32).
#
#   robot_repository_decision
#       prints one line and returns
#       0  run: the line is the directory's physical path
#       1  skipped: the line is why (the variable is unset)
#       2  FAILED: the line is why (the variable is set, empty included,
#          to something that is not a directory: it asks for the check
#          and names no robot repository)
robot_repository_decision() {
  if [ -z "${TALOS_ROBOT_REPOSITORY+set}" ]; then
    echo "TALOS_ROBOT_REPOSITORY unset"
    return 1
  fi
  local path
  # Not empty: `cd ''` stays where it is, and would check the directory
  # prepush was started in.
  if [ -n "$TALOS_ROBOT_REPOSITORY" ] &&
      path="$(CDPATH='' cd -- "$TALOS_ROBOT_REPOSITORY" 2>/dev/null && pwd -P)"; then
    echo "$path"
    return 0
  fi
  echo "TALOS_ROBOT_REPOSITORY names no robot repository: '$TALOS_ROBOT_REPOSITORY' is not a directory"
  return 2
}
