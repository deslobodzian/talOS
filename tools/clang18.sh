# Sourced by tools/format.sh and tools/tidy.sh: puts clang-format-18 and
# clang-tidy-18 on PATH, the same binaries CI runs, and picks the diff base.
#
# The tools are the PyPI wheels clang-format / clang-tidy (the upstream LLVM
# 18 binaries for Linux x86_64/aarch64 and macOS arm64) installed into a
# private venv under ${XDG_CACHE_HOME:-~/.cache}/talos/clang-18, so no system
# package is needed. CI installs Ubuntu's clang-{format,tidy}-18 packages;
# the wheels are 18.1.8, the same major release.

CLANG18_VERSION="18.1.8"

clang18_die() { printf '%s: error: %s\n' "$(basename "$0")" "$*" >&2; exit 2; }

clang18_setup() {
  local home="${XDG_CACHE_HOME:-$HOME/.cache}/talos/clang-18"
  local bin="$home/venv/bin" shim="$home/shim"
  if [ ! -x "$bin/clang-format" ] || [ ! -x "$bin/clang-tidy" ] ||
     [ "$(cat "$home/version" 2>/dev/null)" != "$CLANG18_VERSION" ]; then
    echo "installing clang-format/clang-tidy $CLANG18_VERSION into $home" >&2
    rm -rf "$home"
    mkdir -p "$home"
    if command -v uv >/dev/null 2>&1; then
      { uv venv -q "$home/venv" &&
          uv pip install -q --python "$bin/python" \
            "clang-format==$CLANG18_VERSION" "clang-tidy==$CLANG18_VERSION"; } ||
        { rm -rf "$home"; clang18_die "uv could not install the wheels"; }
    else
      command -v python3 >/dev/null 2>&1 || clang18_die "need uv or python3"
      { python3 -m venv "$home/venv" &&
          "$bin/pip" install -q \
            "clang-format==$CLANG18_VERSION" "clang-tidy==$CLANG18_VERSION"; } ||
        { rm -rf "$home"; clang18_die "pip could not install the wheels"; }
    fi
    printf '%s\n' "$CLANG18_VERSION" > "$home/version"
  fi
  # CI invokes the tools by their versioned names.
  mkdir -p "$shim"
  ln -sf "$bin/clang-format" "$shim/clang-format-18"
  ln -sf "$bin/clang-tidy" "$shim/clang-tidy-18"
  export PATH="$shim:$PATH"
  local v
  v="$(clang-format-18 --version 2>&1)" || clang18_die "clang-format-18 does not run: $v"
  case "$v" in *"version 18."*) ;; *) clang18_die "unexpected clang-format: $v" ;; esac
}

# clang18_base <ref|--all>: sets BASE (empty for --all) to the merge base of
# the ref and HEAD, as CI computes it for a pull request.
clang18_base() {
  BASE=""
  [ "$1" = "--all" ] && return 0
  BASE="$(git merge-base "$1" HEAD 2>/dev/null)" ||
    clang18_die "no merge base with $1 (git fetch origin?)"
}
