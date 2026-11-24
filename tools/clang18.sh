# Sourced by tools/format.sh and tools/tidy.sh: puts clang-format-18 and
# clang-tidy-18 on PATH, the same binaries CI runs, and picks the diff base.
#
# The tools are the PyPI wheels clang-format / clang-tidy (the upstream LLVM
# 18 binaries for Linux x86_64/aarch64 and macOS arm64) installed into a
# private venv under ${XDG_CACHE_HOME:-~/.cache}/talos/clang-18, so no system
# package is needed. CI installs Ubuntu's clang-{format,tidy}-18 packages;
# the wheels are 18.1.8, the same major release.

CLANG18_VERSION="18.1.8"
_CLANG18_SH="$(CDPATH='' cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/$(basename "${BASH_SOURCE[0]}")"

clang18_die() { printf '%s: error: %s\n' "$(basename "$0")" "$*" >&2; exit 2; }

clang18_ready() {
  [ -x "$1/venv/bin/clang-format" ] &&
    [ -x "$1/venv/bin/clang-tidy" ] &&
    [ "$(cat "$1/version" 2>/dev/null || true)" = "$CLANG18_VERSION" ]
}

_clang18_install_locked() {
  local cache_root="$1" home="$2" bin="$3"
  local self_pid="${BASHPID:-}"
  [ -n "$self_pid" ] || self_pid="$(exec sh -c 'echo "$PPID"')"
  _clang18_staging=""
  _clang18_old=""
  cleanup_clang18_install() {
    if [ -n "${_clang18_staging:-}" ]; then rm -rf "$_clang18_staging"; fi
    if [ -n "${_clang18_old:-}" ]; then rm -rf "$_clang18_old"; fi
  }
  trap cleanup_clang18_install EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP

  if clang18_ready "$home"; then
    return 0
  fi

  echo "installing clang-format/clang-tidy $CLANG18_VERSION into $home" >&2
  _clang18_staging="$(mktemp -d "$cache_root/.clang-18.staging.XXXXXX")" ||
    clang18_die "cannot create staging directory in $cache_root"
  local staging="$_clang18_staging"
  local sbin="$staging/venv/bin"
  if command -v uv >/dev/null 2>&1; then
    { uv venv -q "$staging/venv" &&
        uv pip install -q --python "$sbin/python" \
          "clang-format==$CLANG18_VERSION" "clang-tidy==$CLANG18_VERSION"; } ||
      clang18_die "uv could not install the wheels"
  else
    command -v python3 >/dev/null 2>&1 || clang18_die "need uv or python3"
    { python3 -m venv "$staging/venv" &&
        "$sbin/pip" install -q \
          "clang-format==$CLANG18_VERSION" "clang-tidy==$CLANG18_VERSION"; } ||
      clang18_die "pip could not install the wheels"
  fi
  # Console scripts written by pip/uv embed the staging venv path in their
  # shebang (or long-path sh exec wrapper); rewrite it to the final $home
  # path before swapping into place. Avoid head | grep under pipefail (SIGPIPE).
  local py="$sbin/python" f
  [ -x "$py" ] || py="$(command -v python3 2>/dev/null || true)"
  for f in "$sbin/clang-format" "$sbin/clang-tidy"; do
    if [ -f "$f" ] && [ -n "$py" ]; then
      "$py" -c '
import os, sys
path, old_dir, new_dir = sys.argv[1:4]
with open(path, "rb") as fp:
    data = fp.read()
old_b, new_b = old_dir.encode(), new_dir.encode()
if data.startswith(b"#!") and old_b in data[:512]:
    lines = data.splitlines(keepends=True)
    changed = False
    for i in range(min(2, len(lines))):
        updated = lines[i].replace(old_b, new_b)
        if updated != lines[i]:
            lines[i] = updated
            changed = True
    if changed:
        st = os.stat(path)
        tmp_path = path + ".tmp"
        with open(tmp_path, "wb") as fp:
            fp.writelines(lines)
        os.chmod(tmp_path, st.st_mode)
        os.replace(tmp_path, path)
' "$f" "$staging" "$home" || clang18_die "cannot rewrite shebang in $f"
    fi
  done
  mkdir -p "$staging/shim"
  ln -sf "$bin/clang-format" "$staging/shim/clang-format-18"
  ln -sf "$bin/clang-tidy" "$staging/shim/clang-tidy-18"
  printf '%s\n' "$CLANG18_VERSION" > "$staging/version"

  if [ -e "$home" ]; then
    _clang18_old="$cache_root/.clang-18.old.$self_pid"
    rm -rf "$_clang18_old"
    mv "$home" "$_clang18_old" || clang18_die "cannot replace existing $home"
    if ! mv "$staging" "$home"; then
      mv "$_clang18_old" "$home" 2>/dev/null || true
      _clang18_old=""
      clang18_die "cannot move $staging into $home"
    fi
    _clang18_staging=""
    rm -rf "$_clang18_old"
    _clang18_old=""
  else
    mv "$staging" "$home" || clang18_die "cannot move $staging into $home"
    _clang18_staging=""
  fi
}

clang18_setup() {
  local cache_root="${XDG_CACHE_HOME:-$HOME/.cache}/talos"
  local home="$cache_root/clang-18"
  local bin="$home/venv/bin" shim="$home/shim"
  if ! clang18_ready "$home"; then
    mkdir -p "$cache_root" || clang18_die "cannot create cache directory $cache_root"
    local lock_file="$cache_root/.clang-18.flock"
    if command -v flock >/dev/null 2>&1; then
      (
        exec 9>"$lock_file" || clang18_die "cannot open $lock_file"
        flock 9 || clang18_die "cannot lock $lock_file"
        _clang18_install_locked "$cache_root" "$home" "$bin"
      )
    else
      command -v python3 >/dev/null 2>&1 || clang18_die "need flock or python3"
      python3 -c '
import fcntl, subprocess, sys
lock_path, caller, script, cache_root, home, bin_dir = sys.argv[1:7]
with open(lock_path, "a+") as f:
    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
    r = subprocess.run(
        ["bash", "-euo", "pipefail", "-c",
         ". \"$1\" && _clang18_install_locked \"$2\" \"$3\" \"$4\"",
         caller, script, cache_root, home, bin_dir],
    )
sys.exit(r.returncode)
' "$lock_file" "$0" "$_CLANG18_SH" "$cache_root" "$home" "$bin"
    fi
  fi
  # CI invokes the tools by their versioned names. Pre-populated caches (such as
  # test fixtures) may lack shim/; populate atomically to avoid concurrent races.
  if [ ! -x "$shim/clang-format-18" ] || [ ! -x "$shim/clang-tidy-18" ]; then
    local shim_pid="${BASHPID:-}"
    [ -n "$shim_pid" ] || shim_pid="$(exec sh -c 'echo "$PPID"')"
    mkdir -p "$shim"
    ln -sf "$bin/clang-format" "$shim/.clang-format-18.$shim_pid" &&
      mv -f "$shim/.clang-format-18.$shim_pid" "$shim/clang-format-18"
    ln -sf "$bin/clang-tidy" "$shim/.clang-tidy-18.$shim_pid" &&
      mv -f "$shim/.clang-tidy-18.$shim_pid" "$shim/clang-tidy-18"
  fi
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
