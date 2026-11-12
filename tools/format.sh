#!/usr/bin/env bash
#
# Run clang-format 18, the version CI runs, on the C/C++ files changed against
# a base. Same selection as CI's clang-format job: files changed since the
# base (deletions excluded), *.c *.cc *.cpp *.h *.hpp, third_party/ excluded.
# Uncommitted edits are included too, since this runs before committing.
# Tool setup: tools/clang18.sh.
#
#   tools/format.sh [--check | --fix] [--base <ref> | --all]
#
#   --check   (default) list files that need formatting; exit 1 if any
#   --fix     rewrite them in place
#   --base    diff against the merge base with <ref> (default origin/main)
#   --all     every tracked C/C++ file
#
# Exit: 0 clean, 1 files need formatting, 2 usage or tool error.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
. "$(dirname "${BASH_SOURCE[0]}")/clang18.sh"

mode=check
base_arg=origin/main
while [ $# -gt 0 ]; do
  case "$1" in
    --check) mode=check ;;
    --fix) mode=fix ;;
    --all) base_arg=--all ;;
    --base) [ $# -ge 2 ] || clang18_die "--base needs a ref"; base_arg="$2"; shift ;;
    -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) clang18_die "unknown argument: $1" ;;
  esac
  shift
done

clang18_setup
clang18_base "$base_arg"

pat=('*.c' '*.cc' '*.cpp' '*.h' '*.hpp' ':!third_party/')
files=()
if [ -z "$BASE" ]; then
  while IFS= read -r -d '' f; do files+=("$f"); done < <(git ls-files -z -- "${pat[@]}")
else
  while IFS= read -r -d '' f; do
    [ -f "$f" ] && files+=("$f")
  done < <(git diff --name-only --diff-filter=d -z "$BASE" -- "${pat[@]}")
fi
[ ${#files[@]} -gt 0 ] || { echo "format: no C/C++ files to check"; exit 0; }

if [ "$mode" = fix ]; then
  clang-format-18 -i "${files[@]}"
  echo "format: formatted ${#files[@]} file(s)"
  exit 0
fi

bad=()
for f in "${files[@]}"; do
  clang-format-18 --dry-run --Werror "$f" >/dev/null 2>&1 || bad+=("$f")
done
if [ ${#bad[@]} -gt 0 ]; then
  echo "format: needs clang-format 18:"
  printf '  %s\n' "${bad[@]}"
  echo "fix with: tools/format.sh --fix --base $base_arg"
  exit 1
fi
echo "format: ${#files[@]} file(s) clean"
