# Installing a development environment

Development only, plus the [realtime grant](#realtime-grant-for-deploysh-run) a
hand-run bundle needs. Deploying to the robot is [DEPLOYMENT.md](DEPLOYMENT.md) / `tools/deploy.sh`.

## Quick start

| OS | Command (from the repo root) |
|---|---|
| Linux, macOS | `./tools/setup.sh` |
| Windows | inside WSL2 Ubuntu 24.04: `./tools/setup.sh` |

On Windows, install WSL2 first from an admin PowerShell with `wsl --install -d Ubuntu-24.04`, then clone the repo inside the WSL home and run the script there.

The script checks installed versions before installing components and prints
next-step commands only when no required component failed.

## What Bazel provides vs what the host needs

Hermetic (downloaded by Bazel, see `MODULE.bazel`): Python 3.11, Node 22.14.0,
pnpm 9.15.9, all C++/JS dependencies. Bazel itself is pinned
by `.bazelversion` (8.6.0) and fetched by bazelisk.

| Component | Why | Linux | macOS | Windows |
|---|---|---|---|---|
| bazelisk (as `bazel`) | runs pinned Bazel 8.6.0 | release binary (sha256-verified) → `/usr/local/bin/bazel` | `brew install bazelisk` | in WSL |
| C++23 compiler | talOS (`std::format`, `std::jthread`) | setup checks GCC ≥ 13 | Xcode CLT | in WSL |
| git, curl, unzip, zip, build-essential | Bazel fetches, builds | apt/dnf/pacman | Xcode CLT / brew | in WSL |
| system `python3` | rules_python bootstrap, `tools/test_rio_in_the_loop.py`, clang-tidy script | package | Xcode CLT / brew | in WSL |
| JDK 17 | RoboRIO Gradle project, `./sim.sh` WPILib sim | package (OpenJDK) | `brew install --cask temurin@17` | in WSL |
| Node 22 + pnpm 9.15.9 (corepack) | Studio dev loop outside Bazel | package | brew | in WSL |
| `flatc` | none: `pnpm generate` runs Bazel's pinned `@flatbuffers//:flatc` (same as the `//studio:telemetry_ts` genrule) | – | – | run `pnpm generate` in WSL |
| clang-format 18, clang-tidy 18 | lint (`tools/format.sh`, `tools/tidy.sh`) | PyPI wheels `==18.1.8` in a private venv under `~/.cache/talos/clang-18` (via `uv` if present, else `python3 -m venv`); no system package | same | in WSL |
| buildifier | lint | GitHub release (sha256-verified) | brew | in WSL |
| WPILib 2026 | sim GUI | optional | optional | manual installer (not on winget) |

CI uses clang-format/clang-tidy 18 and buildifier 8.2.1. `tools/setup.sh`
installs the 18.1.8 wheels, which are the same upstream release line as CI's
`clang-{format,tidy}-18` packages and need no system LLVM. Before committing:

```sh
tools/format.sh --check    # or --fix; changed and untracked C/C++ files vs the merge base with origin/main
tools/tidy.sh              # clang-tidy over the files CI would analyze (commit first)
```

Both take `--base <ref>` and `--all`. `tools/tidy.sh` runs CI's own selection
script, and needs `compile_commands.json` (it runs
`bazel run @hedron_compile_commands//:refresh_all` when missing, or with
`--refresh`) and the generated headers from a prior `bazel build` of the
touched packages. It first checks that clang 18 parses the host's C++
standard library. On macOS the newest SDK's libc++ may need a newer clang
(the 26.x SDKs do); it then analyses against the newest installed SDK
(Command Line Tools or Xcode) that clang 18 parses, and prints which. On
Linux x86_64, when clang 18 cannot parse the host's libstdc++ (such as
Arch's GCC 16), it fetches pinned Debian GCC 14 `libstdc++-14-dev` and
`libgcc-14-dev` headers (SHA-256 verified) into
`${XDG_CACHE_HOME:-~/.cache}/talos/gcc-14-sysroot-14.2.0-19` and analyses against them with
`--gcc-install-dir`. It exits 3 when it cannot run: no bazel to make the
database, or a standard library clang 18 cannot parse when no fallback is
available (e.g. an offline host before the GCC 14 cache is populated,
non-x86_64 Linux, or no installed macOS SDK it parses), where it says so
and names what it tried.
Homebrew buildifier and the distribution's buildifier may differ from 8.2.1;
setup warns about this.

## Flags

| Flag | Effect |
|---|---|
| `--check` | report each component ok/missing/wrong-version; install nothing; exit 1 if anything required is missing |
| `--yes` | non-interactive: answers prompts (incl. Homebrew install on macOS), passes `-y`/`--noconfirm` to package managers |
| `--no-studio` | skip Node 22/pnpm |
| `--no-rio` | skip JDK 17 and `./gradlew installRoboRioToolchain` |
| `--no-lint` | skip clang-format/clang-tidy 18 wheels, buildifier |
| `--verify` | build the launcher, `//talOS/bridge:hardware_node` and every node the bench robot's config (`bench/main_processor/configuration/robot.toml`) names, then run `--describe-only` on it |
| `-h`, `--help` | usage |

Package managers: apt (Ubuntu/Debian), dnf (Fedora), pacman (Arch), brew (macOS).

## Supported platforms

| Platform | Notes |
|---|---|
| Ubuntu 24.04+, Debian 13+, Fedora 39+, Arch | Ubuntu 22.04 ships GCC 11/12: too old. Where the distro has no JDK 17 package, install Temurin 17 by hand (or `--no-rio`), then rerun |
| macOS 13.4+ | Xcode Command Line Tools (`xcode-select --install`) |
| Windows 11, Windows 10 22H2 | via WSL2 `Ubuntu-24.04` |

talOS does not build on native Windows: it uses POSIX shared memory
(`shm_open`/`mmap`) and signals. On Windows the Bazel tree is built inside WSL2.

## Windows notes

- Clone inside the WSL home (`~/talos`), not `/mnt/c/...`: Bazel on `/mnt/c` is very
  slow. The script warns when the repo is on `/mnt/c`.
- The WPILib GUI sim (`./sim.sh`) inside WSL displays through WSLg (Windows 11).
- To run the WPILib sim natively on Windows talking UDP to nodes in WSL, enable
  mirrored networking in `%UserProfile%\.wslconfig`, then `wsl --shutdown`:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  The script does not edit `.wslconfig`.
- Natively on Windows these work: the RIO Gradle project
  (`controller_processor\rio`, `gradlew.bat`), the Studio pnpm loop, WPILib tools.

## Verify

```sh
./tools/setup.sh --verify
bazel test //...
./sim.sh --sim-gateway --no-studio
```

talOS's `.bazelrc` enables a shared Bazel disk cache at
`~/.cache/talos-bazel-disk` (40 GiB GC cap) so worktrees, `tools/prepush.sh`,
and `tools/tidy.sh` reuse compiled artifacts; an explicit `--disk_cache` flag
or `~/.bazelrc` entry overrides it. [`talos.bazelrc`](../talos.bazelrc)
(imported by talOS's `.bazelrc` and dependent robot repositories) sets
`--incompatible_strict_action_env` for shell-`PATH`-independent cache keys
and defines `--config=dev` (`-O1` + the fast test lane; see
[TESTING.md §9](TESTING.md#9-lanes-and-ci) for the analysis-cache switch
caveat when alternating with default `-O3` builds or `tools/tidy.sh`).

CI runs the C++ suite with AddressSanitizer + UndefinedBehaviorSanitizer and
ThreadSanitizer, one leg per sanitizer: on Linux on every push
(`continuous-integration.yaml`), on macOS nightly (`nightly.yaml`). Run sanitizer builds in their own output
bases so they do not evict the default build's cache (`--config=asan`/`tsan` already filter the
`no_asan`/`no_tsan` tags):

```sh
bazel --output_base=$HOME/.cache/bazel-talos-asan test --config=asan --test_output=errors //...
bazel --output_base=$HOME/.cache/bazel-talos-tsan test --config=tsan --test_output=errors //...
```

The decoder `*_fuzz_test` gtests run deterministic, bounded mutation corpora
(`talOS/protocol/fuzz_corpus.h`) under ordinary `bazel test` too: every
truncation, count/length-byte edits, bit flips, insertions, erasures and seeded
random byte replacements. Successful binary decodes must survive a canonical
encode/decode/encode round trip. The hardware codec test also feeds each input
to one `ConfigAssembler` twice and then shifted by one byte, so chunks persist
across calls. This is not coverage-guided fuzzing.

Tests loading native libraries into an uninstrumented Python process via ctypes
carry `no_asan` and `no_tsan`, with the reason beside each BUILD tag.
`talOS/launcher` session tests that time fork-heavy launches carry
`no_macos_sanitizer`: under macOS ASan/TSan one `fork()` costs about 30/375 ms,
so their windows cannot hold. The nightly macOS sanitizer legs filter that tag (locally, add
`--test_tag_filters=-no_asan,-no_macos_sanitizer` or the `tsan` equivalent);
the Linux legs run them. No
project-code sanitizer findings are suppressed. Linux `metal` tests exercise CPU
fallback, not Apple Metal execution; GPU-only tests are skipped or compiled out.

Third-party patches in `third_party/` that sanitizer runs depend on:

- `third_party/apriltag_unionfind_atomic.patch`: AprilTag 3.4.5's union-find parents use
  relaxed atomic accesses, so parallel gradient row bands may compress
  overlapping paths. `//talOS/vision/sanitizer:parallel_detector_test` runs the
  Linux production CPU settings (two detector workers, decimation 2, sigma 0,
  edge refinement) under TSan.
- `third_party/opencv_thread_pool_atomic.patch`: OpenCV 4.12's pthread pool `isActive` flag
  is atomic. The same test's OpenCV case requests four workers and converts
  colour through `CpuAprilTagBackend::Execute`'s `cvtColor`; Apple's GCD
  backend ignores the requested size, so only Linux runs exercise the patch.
  It is a `single_version_override`, which Bazel applies only in the root
  module: a repository that depends on talOS builds OpenCV unpatched, and
  only its `--config=tsan` runs report that flag's race. (talOS fetches
  toml++ as its own patched archive instead, so its parser patch reaches
  every repository that depends on talOS.)
- `third_party/apriltag_pjpeg_bounds.patch`: bounds and overflow fixes in AprilTag's JPEG
  decoder (`pjpeg.c`), used only by the offline `DetectImageFile` API; live
  capture passes raw GREY/YUYV buffers to `Detect`.
  `//talOS/vision/sanitizer:jpeg_loader_test` runs only 64 deterministic
  corrupt-scan variants of one fixture photograph under ASan/UBSan (memory
  safety only); intact photographs are decoded in
  `//talOS/vision/apriltag:cpu_apriltag_backend_test`.

## Building without a network

An event's pit may have no internet. Before going offline, in the checkout
you will build there and at the commit you will build (and the same in a
robot repository's checkout):

```sh
bazel fetch //...
```

This downloads every repository `//...` needs and runs rules_python's `pip`
module extension. Afterwards, in a fresh output base with the network off,
that checkout downloads nothing to analyse all of `//...` (`bazel build
--nobuild //...`), build `//talOS/mechanisms:all` and `//talOS/python/...`,
or test a package with a changed `--test_env`, also after a Bazel server
restart. Unchecked offline: a full `bazel build //...` and `bazel test
//...`, a changed `--config`, the RoboRIO program's Gradle build and
Studio's pnpm build.

The `pip` extension is the step that needs the network again. Each time it
runs, it reads two PyPI index pages, `https://pypi.org/simple/gevent/` and
`https://pypi.org/simple/protobuf/`, for grpc's Python requirements (grpc is a
dependency of flatbuffers), and `MODULE.bazel.lock` cannot store them. Bazel
keeps the extension's result in the checkout's output base, and runs it again
when:

- the output base is new: a fresh clone, a moved or renamed checkout, a new
  `git worktree`, or after `bazel clean --expunge`;
- a requirements file changes (a changed comment in
  `talOS/python/requirements.txt` counts, so a `git checkout` or rebase onto
  such a commit does too), or `MODULE.bazel` changes a `pip.parse` or a
  module version.

Each of these needs the network: do it, then run `bazel fetch //...` again,
before leaving. This documented preparation in the target checkout and output
base is the supported offline workflow; talOS does not patch `rules_python` or
`grpc` to suppress those PyPI index queries.

## Realtime grant for deploy.sh run

On Linux, [`deploy.sh run`](DEPLOYMENT.md#on-the-main-processor) starts a
bundle by hand with the boot service's realtime limits (`RLIMIT_RTPRIO` 49,
the bridge's `SCHED_FIFO` priority; `RLIMIT_MEMLOCK` unlimited, for
`mlockall`), which an account may take only when its hard limits already
allow them. Grant them once, for the account that runs it, and log in again:

```sh
printf '%s\n' "$(id -un) - rtprio 49" "$(id -un) - memlock unlimited" |
  sudo tee /etc/security/limits.d/talos.conf
```

Afterwards `ulimit -H -r` prints `49` and `ulimit -H -l` prints `unlimited`
in a new login (pam_limits applies the file at login, ssh included). For one
terminal only, without the file: `sudo prlimit --pid $$ --rtprio=49:49
--memlock=unlimited:unlimited`. The boot service needs neither: its unit
sets the limits itself. macOS needs no grant.

## Editor setup

clangd needs a compilation database:

```sh
bazel run @hedron_compile_commands//:refresh_all
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `std::format` / `jthread` not found, `-std=c++23` unknown | GCC < 13. Install `g++-13`+ or use a supported distro |
| macOS libc++ / availability errors | `-fexperimental-library` and min macOS 13.4 are already in `talos.bazelrc` (which `.bazelrc` imports); update Xcode CLT |
| RIO build can't find the roboRIO compiler | `cd controller_processor/rio && ./gradlew installRoboRioToolchain` (JDK 17). Setup also recognizes a toolchain from the WPILib installer; see `tools/setup.sh` for discovery paths |
| `$'\r': command not found`, `/usr/bin/env: 'bash\r'` | CRLF working copy. WSL/Linux: `git ls-files -z -- '*.sh' '*gradlew' \| xargs -0 sed -i 's/\r$//'` |
