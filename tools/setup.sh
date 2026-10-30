#!/usr/bin/env bash
#
# talos developer environment: install (or just check) what building, testing
# and simulating this repo needs, on Linux (apt, dnf, pacman) and macOS
# (Homebrew). Development only; robot deployment is tools/deploy.sh. Windows
# runs this inside WSL2 via tools/setup.ps1. Reference: docs/INSTALL.md.
#
#   ./tools/setup.sh [--check] [--yes] [--no-studio] [--no-rio] [--no-lint] [--verify]
#
#   --check      report every component; install nothing; exit 1 if a
#                required one is missing or the wrong version
#   --yes        no prompts (passes -y / --noconfirm to the package manager)
#   --no-studio  skip Node 22 + pnpm (Studio's pnpm loop); Tauri's system
#                libs stay, since `bazel build //...` builds the shell
#   --no-rio     skip JDK 17 and ./gradlew installRoboRioToolchain
#   --no-lint    skip clang-format-18, clang-tidy-18, buildifier
#   --verify     afterwards build the launcher and the robot's nodes, then run
#                the launcher with --describe-only on the 2026 robot config
#   -h, --help   this text
#
# Default installs everything. Re-running is safe: present components at an
# acceptable version are left alone.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RIO_DIR="$ROOT/2026-robot/controller_processor/rio"
ROBOT_CONFIG="2026-robot/main_processor/configuration/robot.toml"

# Pins. Bazel itself, Python 3.11, Node 22.14.0, pnpm and Rust inside Bazel
# are hermetic (MODULE.bazel); these are the tools that live outside it.
BAZELISK_VERSION="v1.30.0"
BUILDIFIER_VERSION="8.2.1"
# sha256 of the GitHub release assets, checked before anything is installed.
BAZELISK_SHA256_amd64="a248ae2b2b62b5baebb748f6fba7203bff1a27e5d01b55399a8406744cb539e8"
BAZELISK_SHA256_arm64="b2bcc040c213b44586b121bda6d983c35df121b917c3a7a66fb16f2fa0df1a4f"
BUILDIFIER_SHA256_amd64="6ceb7b0ab7cf66fceccc56a027d21d9cc557a7f34af37d2101edb56b92fcfa1a"
BUILDIFIER_SHA256_arm64="3baa1cf7eb41d51f462fdd1fff3a6a4d81d757275d05b2dd5f48671284e9a1a5"
PNPM_VERSION="9.15.9"
NODE_MAJOR=22
JDK_MAJOR=17
LLVM_MAJOR=18
GCC_MIN=13
FRC_YEAR=2026

CHECK=0
YES=0
STUDIO=1
RIO=1
LINT=1
VERIFY=0

OS=""
PM=""
ARCH=""
YFLAG=""
DISTRO=""
APT_UPDATED=0
CLT_OK=1
JDK17_HOME=""
LLVM18_BIN=""
ORIG_PATH="$PATH"

NAMES=()
STATES=()
NOTES=()
HINTS=()
DETAIL=""
MISSING_PKGS=""

say() { printf 'setup: %s\n' "$*"; }
warn() { printf 'setup: warning: %s\n' "$*" >&2; }
die() { printf 'setup: error: %s\n' "$*" >&2; exit 1; }
# Every mutation goes through run, so --check can't install by accident.
run() {
  [ "$CHECK" = 0 ] || die "internal: refusing to run under --check: $*"
  printf '+ %s\n' "$*"
  "$@"
}
have() { command -v "$1" >/dev/null 2>&1; }

as_root() {
  if [ "$(id -u)" = 0 ]; then run "$@"
  elif have sudo; then run sudo "$@"
  else die "root needed for: $* (install sudo or re-run as root)"
  fi
}

confirm() {
  [ "$YES" = 1 ] && return 0
  [ -t 0 ] || return 1
  local a
  read -r -p "setup: $1 [y/N] " a
  case "$a" in y|Y|yes|YES) return 0 ;; esac
  return 1
}

usage() { sed -n '2,/^set -euo pipefail/{/^set /d;s/^# \{0,1\}//;p;}' "$0"; }

record() { NAMES+=("$1"); STATES+=("$2"); NOTES+=("${3:-}"); }
hint() {
  local h
  for h in ${HINTS[@]+"${HINTS[@]}"}; do [ "$h" = "$*" ] && return 0; done
  HINTS+=("$*")
}

# --- arguments ---------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK=1 ;;
    --yes|-y) YES=1 ;;
    --no-studio) STUDIO=0 ;;
    --no-rio) RIO=0 ;;
    --no-lint) LINT=0 ;;
    --verify) VERIFY=1 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "unknown argument: $1" ;;
  esac
  shift
done

# --- platform ----------------------------------------------------------------
detect_os() {
  case "$(uname -m)" in
    x86_64|amd64) ARCH=amd64 ;;
    aarch64|arm64) ARCH=arm64 ;;
    *) ARCH="$(uname -m)" ;;
  esac
  case "$(uname -s)" in
    Darwin)
      OS=macos; PM=brew
      local v major rest minor
      v="$(sw_vers -productVersion)"
      DISTRO="macOS $v"
      major="${v%%.*}"; rest="${v#*.}"; minor="${rest%%.*}"
      [ "$rest" = "$v" ] && minor=0
      # .bazelrc sets -mmacosx-version-min=13.4 for C++23 libc++.
      if [ "$major" -lt 13 ] || { [ "$major" -eq 13 ] && [ "$minor" -lt 4 ]; }; then
        die "macOS 13.4 or newer is required (C++23 libc++); this is $v"
      fi
      ;;
    Linux)
      OS=linux
      [ -r /etc/os-release ] || die "no /etc/os-release; cannot tell which package manager to use"
      local id like
      # Subshells: os-release defines NAME, VERSION, ... which must not leak.
      id="$(. /etc/os-release && printf '%s' "${ID:-}")"
      like="$(. /etc/os-release && printf '%s' "${ID_LIKE:-}")"
      DISTRO="$(. /etc/os-release && printf '%s' "${PRETTY_NAME:-$id}")"
      case " $id $like " in
        *" ubuntu "*|*" debian "*) PM=apt; YFLAG="-y" ;;
        *" fedora "*) PM=dnf; YFLAG="-y" ;;
        *" arch "*) PM=pacman; YFLAG="--noconfirm" ;;
        *) die "unsupported distribution: $DISTRO. Supported: Ubuntu 24.04+, Debian 13+, Fedora 39+, Arch (apt/dnf/pacman), macOS (brew). Elsewhere install by hand: git curl unzip zip, g++ >= $GCC_MIN, python3, bazelisk as 'bazel', Tauri's GTK/WebKit2GTK 4.1 dev libs, Node $NODE_MAJOR + pnpm $PNPM_VERSION (corepack), JDK $JDK_MAJOR, clang-format/clang-tidy $LLVM_MAJOR, buildifier $BUILDIFIER_VERSION — see docs/INSTALL.md" ;;
      esac
      ;;
    *) die "unsupported OS: $(uname -s). Windows: run tools/setup.ps1 (sets up WSL2)" ;;
  esac
  [ "$YES" = 1 ] || YFLAG=""
}

# --- package manager ---------------------------------------------------------
pkg_has() {
  case "$PM" in
    apt) dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed' ;;
    dnf) rpm -q --whatprovides "$1" >/dev/null 2>&1 ;;
    pacman) pacman -Qi "$1" >/dev/null 2>&1 || pacman -Qg "$1" >/dev/null 2>&1 ;;
    brew) have brew && brew list --versions "$1" >/dev/null 2>&1 ;;
  esac
}

# Sets MISSING_PKGS to the subset of "$@" not installed; returns 1 if any.
pkgs_missing() {
  local p
  MISSING_PKGS=""
  for p in "$@"; do pkg_has "$p" || MISSING_PKGS="$MISSING_PKGS $p"; done
  MISSING_PKGS="${MISSING_PKGS# }"
  [ -z "$MISSING_PKGS" ]
}

pkg_install() {
  case "$PM" in
    apt)
      if [ "$APT_UPDATED" = 0 ]; then
        as_root apt-get update || return 1
        APT_UPDATED=1
      fi
      if [ "$YES" = 1 ]; then
        as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
      else
        as_root apt-get install "$@"
      fi
      ;;
    dnf) as_root dnf install $YFLAG "$@" ;;
    pacman) as_root pacman -S --needed $YFLAG "$@" ;;
    brew) have brew || { warn "Homebrew missing"; return 1; }; run brew install "$@" ;;
  esac
}

# Fetches a single-file release binary, checks its sha256, installs it into a
# system bin dir.
download_bin() {
  local url="$1" sha="$2" dest="$3" tmp rc
  [ -n "$sha" ] || { warn "no pinned sha256 for $url (arch $ARCH)"; return 1; }
  tmp="$(mktemp)" || return 1
  say "downloading $url"
  curl -fL --retry 3 -o "$tmp" "$url" \
    && printf '%s  %s\n' "$sha" "$tmp" | sha256sum -c - \
    && chmod 0755 "$tmp" \
    && as_root mkdir -p "$(dirname "$dest")" && as_root install -m 0755 "$tmp" "$dest"
  rc=$?
  rm -f "$tmp"
  return $rc
}
release_sha() {
  case "$1-$ARCH" in
    bazelisk-amd64) printf '%s' "$BAZELISK_SHA256_amd64" ;;
    bazelisk-arm64) printf '%s' "$BAZELISK_SHA256_arm64" ;;
    buildifier-amd64) printf '%s' "$BUILDIFIER_SHA256_amd64" ;;
    buildifier-arm64) printf '%s' "$BUILDIFIER_SHA256_arm64" ;;
  esac
}

# --- component driver --------------------------------------------------------
# component NAME DETECT INSTALL
# DETECT sets DETAIL and returns 0 ok, 1 missing, 2 wrong version, 3 usable
# but not what CI uses (warning only, nothing to install).
component() {
  local name="$1" detect="$2" install="$3" rc=0 state
  DETAIL=""
  "$detect" || rc=$?
  case "$rc" in
    0) record "$name" ok "$DETAIL"; return 0 ;;
    3) record "$name" warn "$DETAIL"; return 0 ;;
    2) state=wrong-version ;;
    *) state=missing ;;
  esac
  if [ "$CHECK" = 1 ]; then record "$name" "$state" "$DETAIL"; return 0; fi
  say "--- $name: ${DETAIL:-$state} ---"
  if ! "$install"; then
    record "$name" failed "install failed; ${DETAIL:-$state}"
    return 0
  fi
  hash -r
  rc=0; DETAIL=""
  "$detect" || rc=$?
  case "$rc" in
    0) record "$name" installed "$DETAIL" ;;
    3) record "$name" warn "$DETAIL" ;;
    *) record "$name" failed "still not acceptable after install: $DETAIL" ;;
  esac
}

# --- homebrew / xcode command line tools (macOS) ----------------------------
load_brew_env() {
  local b
  for b in /opt/homebrew/bin/brew /usr/local/bin/brew; do
    if [ -x "$b" ]; then eval "$("$b" shellenv)"; return 0; fi
  done
  return 1
}

detect_clt() {
  local p
  if p="$(xcode-select -p 2>/dev/null)" && [ -d "$p" ]; then DETAIL="$p"; CLT_OK=1; return 0; fi
  CLT_OK=0; DETAIL="xcode-select -p: not installed"; return 1
}
install_clt() {
  run xcode-select --install || true
  die "finish the Command Line Tools installer that just opened, then re-run tools/setup.sh"
}

detect_brew() {
  have brew || load_brew_env || { DETAIL="brew not found"; return 1; }
  DETAIL="$(brew --version 2>/dev/null | sed -n '1p')"
}
install_brew() {
  confirm "Homebrew is not installed. Install it from https://brew.sh?" \
    || { hint "Install Homebrew (https://brew.sh) or re-run with --yes"; return 1; }
  # Homebrew treats any set NONINTERACTIVE as "don't ask"; set it only for
  # --yes, after priming sudo so its installer doesn't prompt mid-way.
  if [ "$YES" = 1 ]; then
    run sudo -v && run env NONINTERACTIVE=1 /bin/bash -c \
      "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" || return 1
  else
    run /bin/bash -c \
      "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" || return 1
  fi
  load_brew_env || return 1
  hint "Add Homebrew to your shell: eval \"\$($(command -v brew) shellenv)\" in ~/.zprofile"
}

# --- base tools / compiler (Linux) -------------------------------------------
detect_base() {
  local c missing=""
  for c in git curl unzip zip; do have "$c" || missing="$missing $c"; done
  if [ -n "$missing" ]; then DETAIL="missing:$missing"; return 1; fi
  DETAIL="git $(git --version | sed 's/^git version //'), curl, unzip, zip"
}
install_base() {
  case "$PM" in
    apt) pkg_install git curl unzip zip ca-certificates gnupg ;;
    *) pkg_install git curl unzip zip ;;
  esac
}

gcc_too_old_msg() {
  printf '%s' "GCC >= $GCC_MIN is required (C++23: std::format, std::jthread); found $1 on $DISTRO. Supported out of the box: Ubuntu 24.04+, Debian 13+, Fedora 39+, Arch. Elsewhere install g++-$GCC_MIN (Ubuntu 22.04: ppa:ubuntu-toolchain-r/test) and export CC=gcc-$GCC_MIN CXX=g++-$GCC_MIN before bazel, or upgrade the distro."
}
# Bazel's auto-configured toolchain honours CC/CXX, so check what it will use.
detect_cxx() {
  local cxx="${CXX:-g++}" v n
  have "$cxx" || { DETAIL="$cxx not found"; return 1; }
  v="$("$cxx" -dumpfullversion -dumpversion 2>/dev/null || true)"
  DETAIL="$cxx $v"
  [ "${v%%.*}" -ge "$GCC_MIN" ] 2>/dev/null && return 0
  DETAIL="$cxx $v (need >= $GCC_MIN)"
  if [ -z "${CXX:-}" ]; then
    for n in 15 14 13; do
      if have "g++-$n"; then
        DETAIL="$DETAIL; g++-$n present"
        hint "Default g++ is $v; export CC=gcc-$n CXX=g++-$n (e.g. in ~/.bashrc) so Bazel uses g++-$n"
        return 3
      fi
    done
  fi
  hint "$(gcc_too_old_msg "$cxx $v")"
  return 2
}
install_cxx() {
  # A present-but-old compiler can't be fixed by the package manager on this
  # release; the hint from detect_cxx says what to do.
  have "${CXX:-g++}" && return 1
  case "$PM" in
    apt) pkg_install build-essential ;;
    dnf) pkg_install gcc-c++ make ;;
    pacman) pkg_install base-devel ;;
  esac
}

# --- python3 -----------------------------------------------------------------
# rules_python bootstraps with a system python3; tools/test_drivetrain.py and
# the CI clang-tidy script use it directly.
detect_python() {
  have python3 || { DETAIL="python3 not found"; return 1; }
  # /usr/bin/python3 on macOS is a stub that pops the CLT installer.
  if [ "$OS" = macos ] && [ "$CLT_OK" = 0 ] && [ "$(command -v python3)" = /usr/bin/python3 ]; then
    DETAIL="needs Xcode Command Line Tools"; return 1
  fi
  local v
  v="$(python3 -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])' 2>/dev/null)" \
    || { DETAIL="python3 does not run"; return 1; }
  DETAIL="python $v"
  python3 -c 'import sys; sys.exit(sys.version_info < (3, 8))' || { DETAIL="python $v (need >= 3.8)"; return 2; }
}
install_python() {
  case "$PM" in
    apt|dnf) pkg_install python3 ;;
    pacman) pkg_install python ;;
    brew) pkg_install python ;;
  esac
}

# --- bazel (bazelisk) --------------------------------------------------------
# Bazelisk reads .bazelversion; a plain Bazel install would ignore the pin.
# Detected without running it, since running bazelisk downloads Bazel.
is_bazelisk() {
  local real
  real="$(readlink -f "$1" 2>/dev/null || printf '%s' "$1")"
  case "$(basename "$real")" in *bazelisk*) return 0 ;; esac
  LC_ALL=C grep -aqm1 'BAZELISK_SKIP_WRAPPER' "$real" 2>/dev/null
}
detect_bazel() {
  local p pin
  pin="$(tr -d '[:space:]' < "$ROOT/.bazelversion" 2>/dev/null || true)"
  pin="${pin:-?}"
  p="$(command -v bazel 2>/dev/null)" || { DETAIL="bazel not on PATH"; return 1; }
  if is_bazelisk "$p"; then DETAIL="bazelisk at $p (runs Bazel $pin from .bazelversion)"; return 0; fi
  DETAIL="$p is not bazelisk (.bazelversion pins $pin)"
  return 2
}
install_bazel() {
  if [ "$OS" = macos ]; then pkg_install bazelisk; return; fi
  local dest=/usr/local/bin/bazel
  if [ -e "$dest" ] && ! is_bazelisk "$dest"; then
    confirm "replace non-bazelisk $dest with bazelisk?" || { hint "Remove the plain Bazel at $dest; bazelisk must be 'bazel'"; return 1; }
  fi
  download_bin "https://github.com/bazelbuild/bazelisk/releases/download/$BAZELISK_VERSION/bazelisk-linux-$ARCH" \
    "$(release_sha bazelisk)" "$dest" || return 1
  local p
  p="$(command -v bazel 2>/dev/null || true)"
  [ "$p" = "$dest" ] || hint "'bazel' resolves to $p ahead of $dest; remove it or fix PATH"
}

# --- tauri libs (Linux), studio: node, pnpm ---------------------------------
tauri_pkgs() {
  case "$PM" in
    apt) printf '%s\n' pkg-config libdbus-1-dev libglib2.0-dev libgtk-3-dev libwebkit2gtk-4.1-dev \
      libayatana-appindicator3-dev librsvg2-dev libssl-dev libxdo-dev patchelf ;;
    dnf) printf '%s\n' pkgconf-pkg-config webkit2gtk4.1-devel gtk3-devel libappindicator-gtk3-devel \
      librsvg2-devel openssl-devel libxdo-devel dbus-devel patchelf ;;
    pacman) printf '%s\n' pkgconf webkit2gtk-4.1 gtk3 libappindicator-gtk3 librsvg openssl \
      xdotool dbus patchelf base-devel ;;
  esac
}
detect_tauri() {
  # shellcheck disable=SC2046
  if pkgs_missing $(tauri_pkgs); then DETAIL="$(tauri_pkgs | wc -l | tr -d ' ') packages present"; return 0; fi
  DETAIL="missing: $MISSING_PKGS"
  return 1
}
install_tauri() {
  # shellcheck disable=SC2086
  pkg_install $MISSING_PKGS
}

# brew's node@22 is keg-only; use it for this run and tell the user.
use_node_keg() {
  [ "$OS" = macos ] && have brew || return 0
  local keg
  keg="$(brew --prefix node@$NODE_MAJOR 2>/dev/null)/bin"
  [ -x "$keg/node" ] || return 0
  case ":$PATH:" in *":$keg:"*) ;; *) PATH="$keg:$PATH" ;; esac
  case ":$ORIG_PATH:" in
    *":$keg:"*) ;;
    *) hint "node@$NODE_MAJOR is keg-only: add 'export PATH=\"$keg:\$PATH\"' to your shell profile" ;;
  esac
}
detect_node() {
  use_node_keg
  have node || { DETAIL="node not found"; return 1; }
  local v
  v="$(node --version 2>/dev/null)"
  DETAIL="node $v"
  v="${v#v}"
  [ "${v%%.*}" -ge "$NODE_MAJOR" ] 2>/dev/null || { DETAIL="node v$v (need >= $NODE_MAJOR)"; return 2; }
}
install_node() {
  local tmp rc
  case "$PM" in
    # Distro Node is older than 22 on most releases; NodeSource carries 22.x.
    # Fetched as the user and checked before root runs it.
    apt|dnf)
      local host=deb.nodesource.com
      [ "$PM" = dnf ] && host=rpm.nodesource.com
      tmp="$(mktemp)" || return 1
      curl -fsSL -o "$tmp" "https://$host/setup_$NODE_MAJOR.x" && [ -s "$tmp" ] \
        && as_root bash "$tmp" && pkg_install nodejs
      rc=$?
      rm -f "$tmp"
      return $rc
      ;;
    # nodejs-lts-jod is the 22.x line; plain nodejs tracks current.
    pacman) pkg_install nodejs-lts-jod npm ;;
    brew) pkg_install "node@$NODE_MAJOR" ;;
  esac
}

corepack_home() { printf '%s' "${COREPACK_HOME:-${XDG_CACHE_HOME:-$HOME/.cache}/node/corepack}"; }
detect_pnpm() {
  have node || { DETAIL="needs node"; return 1; }
  local p real v
  p="$(command -v pnpm 2>/dev/null)" || { DETAIL="pnpm not on PATH (corepack not enabled)"; return 1; }
  real="$(readlink -f "$p" 2>/dev/null || printf '%s' "$p")"
  case "$real" in
    *corepack*)
      # Running the shim would download pnpm; read corepack's pin instead.
      if grep -Eq "\"pnpm\" *: *\"$PNPM_VERSION[+\"]" "$(corepack_home)/lastKnownGood.json" 2>/dev/null; then
        DETAIL="pnpm $PNPM_VERSION via corepack"; return 0
      fi
      DETAIL="corepack pnpm not pinned to $PNPM_VERSION"; return 2 ;;
  esac
  v="$(pnpm --version 2>/dev/null || true)"
  DETAIL="pnpm $v at $p"
  [ "$v" = "$PNPM_VERSION" ] || { DETAIL="pnpm $v at $p (want $PNPM_VERSION via corepack)"; return 2; }
}
# corepack writes shims next to node, which may be root-owned.
node_dir_run() {
  local bin
  bin="$(command -v "$1")" || return 1
  shift
  if [ -w "$(dirname "$bin")" ]; then run "$bin" "$@"; else as_root "$bin" "$@"; fi
}
install_pnpm() {
  have node || return 1
  if ! have corepack; then
    # Node >= 25 and some distro builds ship without corepack.
    have npm || { warn "neither corepack nor npm found; reinstall Node $NODE_MAJOR"; return 1; }
    node_dir_run npm install -g corepack || return 1
    hash -r
  fi
  node_dir_run corepack enable \
    && run env COREPACK_ENABLE_DOWNLOAD_PROMPT=0 corepack prepare "pnpm@$PNPM_VERSION" --activate
}

# --- rio: jdk 17, roborio toolchain -----------------------------------------
java_major() { "$1" -version 2>&1 | sed -n 's/.*version "\([0-9][0-9]*\).*/\1/p' | sed -n '1p'; }
find_jdk17() {
  local c cands=()
  [ -n "${JAVA_HOME:-}" ] && cands+=("$JAVA_HOME")
  cands+=("$HOME/wpilib/$FRC_YEAR/jdk")
  if [ "$OS" = macos ]; then
    c="$(/usr/libexec/java_home -v "$JDK_MAJOR" 2>/dev/null)" && cands+=("$c")
  else
    for c in /usr/lib/jvm/*; do [ -d "$c" ] && cands+=("$c"); done
  fi
  if have java; then
    c="$(readlink -f "$(command -v java)" 2>/dev/null)" && cands+=("${c%/bin/java}")
  fi
  for c in "${cands[@]}"; do
    if [ -x "$c/bin/java" ] && [ -x "$c/bin/javac" ] && [ "$(java_major "$c/bin/java")" = "$JDK_MAJOR" ]; then
      printf '%s\n' "$c"
      return 0
    fi
  done
  return 1
}
detect_jdk() {
  JDK17_HOME="$(find_jdk17)" || { JDK17_HOME=""; DETAIL="no JDK $JDK_MAJOR found"; return 1; }
  DETAIL="$JDK17_HOME"
  # gradlew uses JAVA_HOME, else the java on PATH.
  local active=""
  if [ -n "${JAVA_HOME:-}" ]; then active="$JAVA_HOME"
  elif have java; then active="$(readlink -f "$(command -v java)" 2>/dev/null)"; active="${active%/bin/java}"
  fi
  if [ "$active" != "$JDK17_HOME" ] && { [ -z "$active" ] || [ "$(java_major "$active/bin/java" 2>/dev/null)" != "$JDK_MAJOR" ]; }; then
    hint "GradleRIO needs JDK $JDK_MAJOR: export JAVA_HOME=$JDK17_HOME before ./gradlew or ./sim.sh"
  fi
}
install_jdk() {
  local ok=0
  case "$PM" in
    apt) pkg_install openjdk-17-jdk && ok=1 ;;
    dnf) pkg_install java-17-openjdk-devel && ok=1 ;;
    pacman) pkg_install jdk17-openjdk && ok=1 ;;
    brew) run brew install --cask temurin@17 && ok=1 ;;
  esac
  [ "$ok" = 1 ] && return 0
  hint "No JDK $JDK_MAJOR package here: install Temurin 17 (https://adoptium.net/installation/) or the WPILib $FRC_YEAR installer (bundles one in ~/wpilib/$FRC_YEAR/jdk)"
  return 1
}

# GradleRIO names the toolchain dir after its toolchain year, which lags the
# season (GradleRIO 2026.x installs frc/2025), so accept any year.
detect_rio() {
  local base g
  base="${GRADLE_USER_HOME:-$HOME/.gradle}/toolchains/frc"
  for g in "$base"/*/roborio/bin/*-g++; do
    if [ -x "$g" ]; then DETAIL="${g%/bin/*}"; return 0; fi
  done
  DETAIL="not installed in $base/<year>/roborio"
  return 1
}
install_rio() {
  [ -n "$JDK17_HOME" ] || { warn "RoboRIO toolchain needs JDK $JDK_MAJOR first"; return 1; }
  if LC_ALL=C grep -q $'\r' "$RIO_DIR/gradlew"; then
    hint "gradlew has CRLF line endings (Windows checkout): run git ls-files -z -- '*.sh' '*gradlew' | xargs -0 sed -i 's/\\r\$//' from the repo root (or rerun tools\\setup.ps1 on Windows)"
    return 1
  fi
  (cd "$RIO_DIR" && run env JAVA_HOME="$JDK17_HOME" bash ./gradlew installRoboRioToolchain)
}

# --- lint: clang-format/clang-tidy 18, buildifier ---------------------------
llvm_major() { "$1" --version 2>/dev/null | sed -n 's/.*version \([0-9][0-9]*\).*/\1/p' | sed -n '1p'; }
set_llvm18_bin() {
  LLVM18_BIN=""
  [ "$OS" = macos ] && have brew || return 0
  local d
  d="$(brew --prefix llvm@$LLVM_MAJOR 2>/dev/null)/bin"
  [ -x "$d/clang-format" ] && LLVM18_BIN="$d"
  return 0
}
find_llvm_tool() {
  if have "$1-$LLVM_MAJOR"; then command -v "$1-$LLVM_MAJOR"; return 0; fi
  if [ -n "$LLVM18_BIN" ] && [ -x "$LLVM18_BIN/$1" ]; then printf '%s\n' "$LLVM18_BIN/$1"; return 0; fi
  if have "$1"; then command -v "$1"; return 0; fi
  return 1
}
detect_llvm() {
  set_llvm18_bin
  local n p m missing=0 other=0 out=""
  for n in clang-format clang-tidy; do
    if ! p="$(find_llvm_tool "$n")"; then out="$out, $n missing"; missing=1; continue; fi
    m="$(llvm_major "$p")"
    out="$out, $n ${m:-?} ($p)"
    [ "$m" = "$LLVM_MAJOR" ] || other=1
  done
  DETAIL="${out#, }"
  [ "$missing" = 0 ] || return 1
  [ "$other" = 1 ] || {
    case "$LLVM18_BIN" in "") ;; *) hint "clang-format/clang-tidy $LLVM_MAJOR are keg-only in $LLVM18_BIN; add it to PATH" ;; esac
    return 0
  }
  # apt and brew can provide 18; elsewhere it's the distro's LLVM or nothing.
  case "$PM" in
    apt|brew) DETAIL="$DETAIL; CI uses $LLVM_MAJOR"; return 2 ;;
    *) DETAIL="$DETAIL; CI uses $LLVM_MAJOR, results may differ"; return 3 ;;
  esac
}
install_llvm() {
  case "$PM" in
    apt) pkg_install "clang-format-$LLVM_MAJOR" "clang-tidy-$LLVM_MAJOR" ;;
    dnf) pkg_install clang-tools-extra ;;
    pacman) pkg_install clang ;;
    brew) pkg_install "llvm@$LLVM_MAJOR" ;;
  esac
}

detect_buildifier() {
  have buildifier || { DETAIL="buildifier not found"; return 1; }
  local v
  v="$(buildifier --version 2>/dev/null | sed -n 's/^buildifier version: *//p' | tr -d '[:space:]')"
  DETAIL="buildifier ${v:-?}"
  [ "$v" = "$BUILDIFIER_VERSION" ] && return 0
  DETAIL="$DETAIL (CI uses $BUILDIFIER_VERSION)"
  # brew only has its current release; that's close enough to lint locally.
  [ "$OS" = macos ] && return 3
  return 2
}
install_buildifier() {
  if [ "$OS" = macos ]; then pkg_install buildifier; return; fi
  download_bin "https://github.com/bazelbuild/buildtools/releases/download/v$BUILDIFIER_VERSION/buildifier-linux-$ARCH" \
    "$(release_sha buildifier)" /usr/local/bin/buildifier
}

# --- verify ------------------------------------------------------------------
# --describe-only asks every node for its description, so the nodes the config
# names must be built too, not just the launcher. The hardware node is implied
# ([robot] hardware_node, default //talOS/bridge:hardware_node).
verify() {
  local labels
  labels="$( { printf '%s\n' '//talOS/bridge:hardware_node'
    sed 's/#.*//' "$ROOT/$ROBOT_CONFIG" "$(dirname "$ROOT/$ROBOT_CONFIG")"/subsystems/*.toml \
      | grep -o 'node *= *"//[^"]*"' | sed 's/.*"\(\/\/[^"]*\)"/\1/'; } | sort -u)"
  # shellcheck disable=SC2086
  (cd "$ROOT" && run bazel build //talOS/launcher:launcher $labels \
    && run bazel run //talOS/launcher:launcher -- --config "$ROOT/$ROBOT_CONFIG" --describe-only)
}

# --- summary -----------------------------------------------------------------
summary() {
  local i bad=0 s
  printf '\n%-22s %-14s %s\n' COMPONENT STATUS DETAIL
  printf '%-22s %-14s %s\n' --------- ------ ------
  i=0
  while [ "$i" -lt "${#NAMES[@]}" ]; do
    s="${STATES[$i]}"
    case "$s" in missing|wrong-version|failed) bad=$((bad + 1)) ;; esac
    printf '%-22s %-14s %s\n' "${NAMES[$i]}" "$s" "${NOTES[$i]}"
    i=$((i + 1))
  done
  if [ "${#HINTS[@]}" -gt 0 ]; then
    printf '\nNotes:\n'
    for s in "${HINTS[@]}"; do printf '  - %s\n' "$s"; done
  fi
  if [ "$bad" -gt 0 ]; then
    if [ "$CHECK" = 1 ]; then
      printf '\n%d component(s) need attention; run tools/setup.sh without --check to install them.\n' "$bad"
    else
      printf '\n%d component(s) failed; see above and docs/INSTALL.md.\n' "$bad"
    fi
    return 1
  fi
  printf '\nNext steps (from %s):\n' "$ROOT"
  printf '  bazel build //...\n'
  printf '  bazel test //...\n'
  printf '  ./sim.sh --sim-gateway --no-studio\n'
}

# --- main --------------------------------------------------------------------
detect_os
say "$DISTRO ($OS/$ARCH, $PM)$([ "$CHECK" = 1 ] && printf ', check only')"
if [ "$OS" = linux ] && grep -qi microsoft /proc/version 2>/dev/null; then
  case "$ROOT" in
    /mnt/*) warn "repo is on the Windows filesystem ($ROOT); Bazel is much faster on a clone inside the WSL home" ;;
  esac
fi

if [ "$OS" = macos ]; then
  component "xcode clt" detect_clt install_clt
  component "homebrew" detect_brew install_brew
else
  component "git/curl/unzip/zip" detect_base install_base
  component "c++ compiler" detect_cxx install_cxx
fi
component "python3" detect_python install_python
component "bazel (bazelisk)" detect_bazel install_bazel

# Linux always: //studio/src-tauri:desktop is part of `bazel build //...`.
[ "$OS" = linux ] && component "tauri system libs" detect_tauri install_tauri
if [ "$STUDIO" = 1 ]; then
  component "node $NODE_MAJOR" detect_node install_node
  component "pnpm $PNPM_VERSION" detect_pnpm install_pnpm
else
  record "studio node/pnpm" skipped "--no-studio"
fi

if [ "$RIO" = 1 ]; then
  component "jdk $JDK_MAJOR" detect_jdk install_jdk
  component "roborio toolchain" detect_rio install_rio
else
  record "rio deps" skipped "--no-rio"
fi

if [ "$LINT" = 1 ]; then
  component "clang-format/tidy $LLVM_MAJOR" detect_llvm install_llvm
  component "buildifier" detect_buildifier install_buildifier
else
  record "lint tools" skipped "--no-lint"
fi

if [ "$VERIFY" = 1 ]; then
  if [ "$CHECK" = 1 ]; then
    record "verify" skipped "not with --check"
  elif verify; then
    record "verify" ok "launcher --describe-only on $ROBOT_CONFIG"
  else
    record "verify" failed "bazel build or --describe-only failed (output above)"
  fi
fi

summary
