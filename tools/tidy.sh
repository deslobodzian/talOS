#!/usr/bin/env bash
#
# Run clang-tidy 18, the version CI runs, on the C++ files changed against a
# base. The selection logic is not copied: this extracts the Python step
# "Run clang-tidy on changed sources and headers" from
# .github/workflows/continuous-integration.yaml and runs it, so the changed
# sources, the includers analysed for a changed header, the test-code check
# exemption and the OFF_GRAPH list are CI's own. It sees committed changes
# only (git diff BASE HEAD, as in CI): commit first. Tool setup:
# tools/clang18.sh.
#
#   tools/tidy.sh [--base <ref> | --all] [--refresh]
#
#   --base     diff against the merge base with <ref> (default origin/main)
#   --all      every tracked C++ file (slow)
#   --refresh  regenerate compile_commands.json even if it exists
#
# compile_commands.json comes from CI's command, `bazel run
# @hedron_compile_commands//:refresh_all`, when missing. Headers generated
# by Bazel (flatbuffers, topics.h) must exist under bazel-out, so build the
# touched packages first (`bazel build //talOS/...` or the package's
# targets), as CI's build step does before tidy.
#
# On macOS, when clang 18 cannot parse the default SDK's libc++, it analyses
# against the newest installed SDK that it can parse, and says which.
#
# Exit: 0 clean, 1 findings, 2 usage or tool error, 3 cannot run on this host
# (no bazel, the database could not be produced, or clang 18 cannot parse the
# host's C++ standard library: a libstdc++ too new, or no macOS SDK it parses).
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
. "$(dirname "${BASH_SOURCE[0]}")/clang18.sh"

base_arg=origin/main
refresh=0
while [ $# -gt 0 ]; do
  case "$1" in
    --all) base_arg=--all ;;
    --base) [ $# -ge 2 ] || clang18_die "--base needs a ref"; base_arg="$2"; shift ;;
    --refresh) refresh=1 ;;
    -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) clang18_die "unknown argument: $1" ;;
  esac
  shift
done

wf=.github/workflows/continuous-integration.yaml
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
sed -n "/python3 - \"\\\$BASE\" <<'PYEOF'/,/^ *PYEOF\$/p" "$wf" | sed '1d;$d;s/^        //' > "$tmp/tidy.py"
grep -q 'clang-tidy-18' "$tmp/tidy.py" ||
  clang18_die "could not extract the clang-tidy step from $wf; update tools/tidy.sh"

command -v python3 >/dev/null 2>&1 || clang18_die "need python3"
clang18_setup
clang-tidy-18 --version >/dev/null 2>&1 || clang18_die "clang-tidy-18 does not run"

# clang 18 cannot parse a standard library newer than its era (GCC 16's
# libstdc++ on Arch: atomic_wait.h, stl_iterator.h; the macOS 26 SDK's libc++:
# __builtin_clzg), which makes every finding noise. Probe one translation unit
# that includes standard headers, with one real check enabled (a list of only
# clang-diagnostic-* is "no checks enabled", an error on every host).
printf '#include <atomic>\n#include <string>\n#include <vector>\nint main() { return 0; }\n' > "$tmp/probe.cc"
# probe_std [extra clang args...]: 0 parses; 1 the standard library does not
# (an error outside the probe, or a standard header not found); 2 anything
# else, with clang-tidy's output in $tmp/probe.out.
probe_std() {
  if clang-tidy-18 --checks='-*,readability-braces-around-statements' --quiet \
      "$tmp/probe.cc" -- -std=c++23 "$@" >"$tmp/probe.out" 2>&1; then
    return 0
  fi
  if grep -Eq "^[^ ]+:[0-9]+:[0-9]+: (fatal )?error: " "$tmp/probe.out" &&
     ! grep -E "^[^ ]+:[0-9]+:[0-9]+: (fatal )?error: " "$tmp/probe.out" |
       grep -v "file not found" | grep -q "^$tmp/probe.cc:"; then
    return 1
  fi
  return 2
}
probe_failed() {  # probe_failed <status> <what was probed>
  if [ "$1" = 2 ]; then
    echo "tidy: the clang-tidy-18 probe failed for another reason ($2):" >&2
    head -20 "$tmp/probe.out" | sed 's/^/  /' >&2
    exit 2
  fi
}

# CI's step runs clang-tidy-18 by name, so extra arguments go through a
# wrapper ahead of it on PATH. (Arrays are not used: macOS's bash 3.2 treats
# an empty one as unbound under set -u.)
sysroot=""
if [ "$(uname -s)" = Darwin ]; then
  # The compilation database builds against the default SDK; Apple's newest
  # may need a newer clang. Then try the installed SDKs, newest first (the
  # unversioned and major-only names are links to one of them).
  default_sdk="$(xcrun --show-sdk-path 2>/dev/null || true)"
  status=0; probe_std ${default_sdk:+-isysroot "$default_sdk"} || status=$?
  probe_failed "$status" "default SDK ${default_sdk:-(none)}"
  if [ "$status" != 0 ]; then
    tried="${default_sdk:-(no default SDK)}"
    dev="$(xcode-select -p 2>/dev/null || true)"
    for sdk in $( { ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX[0-9]*.[0-9]*.sdk
                    [ -n "$dev" ] && ls -d "$dev"/Platforms/MacOSX.platform/Developer/SDKs/MacOSX[0-9]*.[0-9]*.sdk
                  } 2>/dev/null |
                  awk -F/ '{v=$NF; sub(/^MacOSX/, "", v); sub(/\.sdk$/, "", v); split(v, n, ".");
                            printf "%06d.%06d\t%s\n", n[1], n[2], $0}' |
                  sort -r | cut -f2 | awk '!seen[$0]++'); do
      tried="$tried $sdk"
      status=0; probe_std -isysroot "$sdk" || status=$?
      probe_failed "$status" "$sdk"
      if [ "$status" = 0 ]; then sysroot="$sdk"; break; fi
    done
    if [ -z "$sysroot" ]; then
      echo "tidy: cannot run here: clang 18 cannot parse the C++ standard library of any installed macOS SDK; tried:" >&2
      for sdk in $tried; do echo "  $sdk" >&2; done
      grep -E "error: " "$tmp/probe.out" | head -3 | sed 's/^/  last: /' >&2
      exit 3
    fi
    echo "tidy: clang 18 cannot parse the default SDK ($default_sdk); using $sysroot" >&2
  fi
else
  status=0; probe_std || status=$?
  probe_failed "$status" "host standard library"
  if [ "$status" != 0 ]; then
    echo "tidy: cannot run here: clang 18 cannot parse this host's C++ standard library:" >&2
    grep -E "error: " "$tmp/probe.out" | head -3 | sed 's/^/  /' >&2
    echo "tidy: (CI uses Ubuntu's GCC 13/14 libstdc++.) Run it on macOS or rely on CI." >&2
    exit 3
  fi
fi
if [ -n "$sysroot" ]; then
  mkdir -p "$tmp/bin"
  printf '#!/bin/sh\nexec %q --extra-arg=-isysroot --extra-arg=%q "$@"\n' \
    "$(command -v clang-tidy-18)" "$sysroot" > "$tmp/bin/clang-tidy-18"
  chmod +x "$tmp/bin/clang-tidy-18"
  export PATH="$tmp/bin:$PATH"
fi

if [ "$refresh" = 1 ] || [ ! -f compile_commands.json ]; then
  if ! command -v bazel >/dev/null 2>&1; then
    echo "tidy: cannot run here: no compile_commands.json and no bazel to make it" >&2
    exit 3
  fi
  bazel run --disk_cache="$HOME/.cache/bazel-disk" --noshow_progress \
    @hedron_compile_commands//:refresh_all ||
    { echo "tidy: cannot run here: compilation database refresh failed" >&2; exit 3; }
fi

if [ "$base_arg" = --all ]; then
  # git's empty tree: everything counts as changed.
  BASE="$(git hash-object -t tree /dev/null)"
else
  clang18_base "$base_arg"
fi
python3 "$tmp/tidy.py" "$BASE"
