#!/usr/bin/env bash
#
# Run clang-tidy 18, the version CI runs, on the C++ files changed against a
# base. Both this script and CI invoke tools/run_clang_tidy.py, so the changed
# sources, the includers analysed for a changed header, the test-code check
# exemption and the OFF_GRAPH list are shared. It sees committed changes
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
# against the newest installed SDK that it can parse, and says which. On Linux
# x86_64, when clang 18 cannot parse the host's libstdc++ (e.g. GCC 16 on
# Arch), it fetches pinned Debian GCC 14 libstdc++/libgcc headers into
# ${XDG_CACHE_HOME:-~/.cache}/talos/gcc-14-sysroot-<version> (verified by
# SHA-256) and analyses against them with --gcc-install-dir.
#
# Exit: 0 clean, 1 findings, 2 usage or tool error, 3 cannot run on this host
# (no bazel, the database could not be produced, or clang 18 cannot parse the
# host's C++ standard library and no fallback is available).
set -euo pipefail
script_dir="$(CDPATH='' cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$(git rev-parse --show-toplevel)"
. "$script_dir/clang18.sh"

# Pinned Debian GCC 14 headers for Linux x86_64 hosts whose system libstdc++
# is newer than clang 18 can parse (e.g. Arch Linux GCC 16).
GCC14_DEB_VERSION="14.2.0-19"
GCC14_SNAPSHOT_BASE="https://snapshot.debian.org/archive/debian/20250315T204837Z/pool/main/g/gcc-14"
GCC14_LIBSTDCXX_DEB="libstdc++-14-dev_${GCC14_DEB_VERSION}_amd64.deb"
GCC14_LIBSTDCXX_SHA256="4b962fac5f1af0bd8b1b3f97e0f47b9d8e9a79e88802143f00ebfbd78ad87a7b"
GCC14_LIBGCC_DEB="libgcc-14-dev_${GCC14_DEB_VERSION}_amd64.deb"
GCC14_LIBGCC_SHA256="ca6f2d36d96b19b3eb71405b0b80134d8c89380b02204a2512e5c58ceb090628"

base_arg=origin/main
refresh=0
while [ $# -gt 0 ]; do
  case "$1" in
    --all) base_arg=--all ;;
    --base) [ $# -ge 2 ] || clang18_die "--base needs a ref"; base_arg="$2"; shift ;;
    --refresh) refresh=1 ;;
    -h|--help) sed -n '2,/^set /p' "$script_dir/tidy.sh" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) clang18_die "unknown argument: $1" ;;
  esac
  shift
done

tmp="$(mktemp -d "${TMPDIR:-/tmp}/talos-tidy.XXXXXX")"
staging=""
cleanup() {
  rm -rf "$tmp"
  if [ -n "${staging:-}" ]; then
    rm -rf "$staging"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

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
  if grep -Eq '^.+:[0-9]+:[0-9]+: (fatal )?error: ' "$tmp/probe.out" &&
     ! { grep -E '^.+:[0-9]+:[0-9]+: (fatal )?error: ' "$tmp/probe.out" |
         PROBE_PREFIX="$tmp/probe.cc:" awk \
           'index($0, "file not found") == 0 && index($0, ENVIRON["PROBE_PREFIX"]) == 1 { found = 1 }
            END { exit !found }'; }; then
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
sha256_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    python3 -c 'import hashlib, sys; print(hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest())' "$1"
  fi
}

extract_deb() {
  local deb="$1" dest="$2"
  if command -v dpkg-deb >/dev/null 2>&1; then
    dpkg-deb -x "$deb" "$dest"
  elif command -v bsdtar >/dev/null 2>&1; then
    bsdtar -xOf "$deb" 'data.tar.*' | bsdtar -xf - -C "$dest"
  elif command -v ar >/dev/null 2>&1 && command -v tar >/dev/null 2>&1; then
    local member
    member="$(ar t "$deb" | grep -m1 '^data\.tar\.')" || return 1
    [ -n "$member" ] || return 1
    case "$member" in
      *.xz) ar p "$deb" "$member" | tar -xJf - -C "$dest" ;;
      *.gz) ar p "$deb" "$member" | tar -xzf - -C "$dest" ;;
      *.zst) ar p "$deb" "$member" | tar --zstd -xf - -C "$dest" ;;
      *) ar p "$deb" "$member" | tar -xf - -C "$dest" ;;
    esac
  else
    return 127
  fi
}

gcc14_ready() {
  [ -f "$gcc14_marker" ] &&
    [ "$(cat "$gcc14_marker" 2>/dev/null || true)" = "$gcc14_fingerprint" ] &&
    [ -d "$gcc14_root/usr/lib/gcc/x86_64-linux-gnu/14" ]
}

fetch_and_extract_deb() {
  local deb="$1" expected_sha="$2" dest="$3"
  local url="$GCC14_SNAPSHOT_BASE/$deb"
  local archive="$tmp/$deb"
  local err actual_sha
  if ! curl -fsSL --connect-timeout 15 --max-time 120 -o "$archive" "$url" 2>"$tmp/curl.err"; then
    err="$(head -n 1 "$tmp/curl.err" 2>/dev/null || true)"
    fallback_reason="failed to download $deb (${err:-curl failed})"
    return 1
  fi
  actual_sha="$(sha256_file "$archive")" || {
    fallback_reason="failed to compute SHA-256 for $deb"
    return 1
  }
  if [ "$actual_sha" != "$expected_sha" ]; then
    fallback_reason="SHA-256 mismatch for $deb (expected $expected_sha, got $actual_sha)"
    return 1
  fi
  if ! extract_deb "$archive" "$dest" 2>"$tmp/extract.err"; then
    err="$(head -n 1 "$tmp/extract.err" 2>/dev/null || true)"
    fallback_reason="failed to extract $deb (${err:-extraction failed})"
    return 1
  fi
}

populate_gcc14_sysroot() {
  if gcc14_ready; then
    return 0
  fi
  if ! command -v curl >/dev/null 2>&1; then
    fallback_reason="need curl to fetch Debian GCC 14 packages"
    return 1
  fi
  if ! command -v dpkg-deb >/dev/null 2>&1 &&
     ! command -v bsdtar >/dev/null 2>&1 &&
     { ! command -v ar >/dev/null 2>&1 || ! command -v tar >/dev/null 2>&1; }; then
    fallback_reason="need dpkg-deb, bsdtar, or ar+tar to extract Debian GCC 14 packages"
    return 1
  fi
  # Clean up any stale staging directory left behind by a prior SIGKILL/SIGTERM
  # (safe here: populate_gcc14_sysroot runs under the cache flock when available).
  if command -v flock >/dev/null 2>&1; then
    rm -rf "$cache_root/.gcc-14-sysroot-${GCC14_DEB_VERSION}".staging.* 2>/dev/null || true
  fi
  staging="$(mktemp -d "$cache_root/.gcc-14-sysroot-${GCC14_DEB_VERSION}.staging.XXXXXX")" || {
    staging=""
    fallback_reason="cannot create staging directory in $cache_root"
    return 1
  }
  echo "tidy: fetching pinned Debian GCC 14 libstdc++ ($GCC14_DEB_VERSION) into $gcc14_root" >&2
  if ! fetch_and_extract_deb "$GCC14_LIBSTDCXX_DEB" "$GCC14_LIBSTDCXX_SHA256" "$staging" ||
     ! fetch_and_extract_deb "$GCC14_LIBGCC_DEB" "$GCC14_LIBGCC_SHA256" "$staging"; then
    rm -rf "$staging"
    staging=""
    return 1
  fi
  if [ ! -d "$staging/usr/lib/gcc/x86_64-linux-gnu/14" ]; then
    rm -rf "$staging"
    staging=""
    fallback_reason="extracted GCC 14 packages missing usr/lib/gcc/x86_64-linux-gnu/14"
    return 1
  fi
  if ! printf '%s\n' "$gcc14_fingerprint" > "$staging/.ready"; then
    fallback_reason="cannot write readiness marker in $staging"
    rm -rf "$staging"
    staging=""
    return 1
  fi
  if [ -e "$gcc14_root" ] && ! gcc14_ready; then
    rm -rf "$gcc14_root" || {
      rm -rf "$staging"
      staging=""
      fallback_reason="cannot replace incomplete cache $gcc14_root"
      return 1
    }
  fi
  if python3 -c 'import os, sys; os.rename(sys.argv[1], sys.argv[2])' "$staging" "$gcc14_root" 2>/dev/null; then
    staging=""
    return 0
  fi
  rm -rf "$staging"
  staging=""
  if gcc14_ready; then
    return 0
  fi
  fallback_reason="failed to move $gcc14_root into place"
  return 1
}

ensure_gcc14_sysroot() {
  if gcc14_ready; then
    return 0
  fi
  if ! mkdir -p "$cache_root" 2>/dev/null || [ ! -w "$cache_root" ]; then
    fallback_reason="cache directory $cache_root is not writable"
    return 1
  fi
  if command -v flock >/dev/null 2>&1; then
    local lock_file="$cache_root/.gcc-14-sysroot-${GCC14_DEB_VERSION}.lock"
    { flock 9 && populate_gcc14_sysroot; } 9>"$lock_file"
  else
    populate_gcc14_sysroot
  fi
}

sysroot=""
gcc_install_dir=""
os_name="$(uname -s)"
if [ "$os_name" = Darwin ]; then
  # The compilation database builds against the default SDK; Apple's newest
  # may need a newer clang. Then try the installed SDKs, newest first (the
  # unversioned and major-only names are links to one of them).
  default_sdk="$(xcrun --show-sdk-path 2>/dev/null || true)"
  status=0; probe_std ${default_sdk:+-isysroot "$default_sdk"} || status=$?
  probe_failed "$status" "default SDK ${default_sdk:-(none)}"
  if [ "$status" != 0 ]; then
    tried="${default_sdk:-(no default SDK)}"
    dev="$(xcode-select -p 2>/dev/null || true)"
    { ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX[0-9]*.[0-9]*.sdk 2>/dev/null || true
      if [ -n "$dev" ]; then
        ls -d "$dev"/Platforms/MacOSX.platform/Developer/SDKs/MacOSX[0-9]*.[0-9]*.sdk 2>/dev/null || true
      fi
    } | awk -F/ '{v=$NF; sub(/^MacOSX/, "", v); sub(/\.sdk$/, "", v); split(v, n, ".");
                  printf "%06d.%06d\t%s\n", n[1], n[2], $0}' |
        sort -r | cut -f2- | awk '!seen[$0]++' > "$tmp/sdks"
    while IFS= read -r sdk; do
      [ -n "$sdk" ] || continue
      tried="${tried}"$'\n'"${sdk}"
      status=0; probe_std -isysroot "$sdk" || status=$?
      probe_failed "$status" "$sdk"
      if [ "$status" = 0 ]; then sysroot="$sdk"; break; fi
    done < "$tmp/sdks"
    if [ -z "$sysroot" ]; then
      echo "tidy: cannot run here: clang 18 cannot parse the C++ standard library of any installed macOS SDK; tried:" >&2
      printf '%s\n' "$tried" | sed 's/^/  /' >&2
      grep -E "error: " "$tmp/probe.out" | head -3 | sed 's/^/  last: /' >&2
      exit 3
    fi
    echo "tidy: clang 18 cannot parse the default SDK ($default_sdk); using $sysroot" >&2
  fi
else
  status=0; probe_std || status=$?
  probe_failed "$status" "host standard library"
  if [ "$status" != 0 ]; then
    native_errors="$(grep -E "error: " "$tmp/probe.out" | head -3 || true)"
    arch_name="$(uname -m)"
    if [ "$os_name" = Linux ] && [ "$arch_name" = x86_64 ]; then
      cache_root="${XDG_CACHE_HOME:-$HOME/.cache}/talos"
      gcc14_root="$cache_root/gcc-14-sysroot-${GCC14_DEB_VERSION}"
      gcc14_marker="$gcc14_root/.ready"
      gcc14_fingerprint="${GCC14_DEB_VERSION}:${GCC14_LIBSTDCXX_SHA256}:${GCC14_LIBGCC_SHA256}"
      fallback_reason=""
      if ! ensure_gcc14_sysroot; then
        echo "tidy: cannot run here: clang 18 cannot parse this host's C++ standard library, and GCC 14 fallback failed (${fallback_reason:-unknown error}):" >&2
        [ -n "$native_errors" ] && printf '%s\n' "$native_errors" | sed 's/^/  /' >&2
        echo "tidy: (CI uses Ubuntu's GCC 13/14 libstdc++.) Rely on CI or check network access." >&2
        exit 3
      fi
      candidate_gcc_dir="$gcc14_root/usr/lib/gcc/x86_64-linux-gnu/14"
      status=0; probe_std "--gcc-install-dir=$candidate_gcc_dir" || status=$?
      if [ "$status" != 0 ]; then
        echo "tidy: cannot run here: clang 18 cannot parse this host's C++ standard library, and GCC 14 fallback ($candidate_gcc_dir) failed to parse:" >&2
        grep -E "error: " "$tmp/probe.out" | head -3 | sed 's/^/  /' >&2
        exit 3
      fi
      gcc_install_dir="$candidate_gcc_dir"
      echo "tidy: clang 18 cannot parse this host's C++ standard library; using $gcc_install_dir" >&2
    else
      echo "tidy: cannot run here: clang 18 cannot parse this host's C++ standard library:" >&2
      [ -n "$native_errors" ] && printf '%s\n' "$native_errors" | sed 's/^/  /' >&2
      echo "tidy: (CI uses Ubuntu's GCC 13/14 libstdc++; automatic fallback is Linux x86_64 only.) Run it on macOS or rely on CI." >&2
      exit 3
    fi
  fi
fi
if [ -n "$sysroot" ] || [ -n "$gcc_install_dir" ]; then
  mkdir -p "$tmp/bin"
  if [ -n "$sysroot" ]; then
    printf '#!/usr/bin/env bash\nexec %q --extra-arg=-isysroot --extra-arg=%q "$@"\n' \
      "$(command -v clang-tidy-18)" "$sysroot" > "$tmp/bin/clang-tidy-18"
  else
    printf '#!/usr/bin/env bash\nexec %q --extra-arg=%q "$@"\n' \
      "$(command -v clang-tidy-18)" "--gcc-install-dir=$gcc_install_dir" > "$tmp/bin/clang-tidy-18"
  fi
  chmod +x "$tmp/bin/clang-tidy-18"
  export PATH="$tmp/bin:$PATH"
fi

if [ "$refresh" = 1 ] || [ ! -f compile_commands.json ]; then
  if ! command -v bazel >/dev/null 2>&1; then
    echo "tidy: cannot run here: no compile_commands.json and no bazel to make it" >&2
    exit 3
  fi
  disk_cache="${TALOS_BAZEL_DISK_CACHE:-$HOME/.cache/talos-bazel-disk}"
  bazel run --disk_cache="$disk_cache" --noshow_progress \
    @hedron_compile_commands//:refresh_all ||
    { echo "tidy: cannot run here: compilation database refresh failed" >&2; exit 3; }
fi

if [ "$base_arg" = --all ]; then
  # git's empty tree: everything counts as changed.
  BASE="$(git hash-object -t tree /dev/null)"
else
  clang18_base "$base_arg"
fi
python3 tools/run_clang_tidy.py "$BASE"
