# Installing a development environment

Development only. Deploying to the robot is [DEPLOYMENT.md](DEPLOYMENT.md) / `tools/deploy.sh`.

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
| system `python3` | rules_python bootstrap, `tools/test_drivetrain.py`, clang-tidy script | package | Xcode CLT / brew | in WSL |
| JDK 17 | RoboRIO Gradle project, `./sim.sh` WPILib sim | package (OpenJDK) | `brew install --cask temurin@17` | in WSL |
| Node 22 + pnpm 9.15.9 (corepack) | Studio dev loop outside Bazel | package | brew | in WSL |
| `flatc` | none: `pnpm generate` runs Bazel's pinned `@flatbuffers//:flatc` (same as the `//studio:telemetry_ts` genrule) | – | – | run `pnpm generate` in WSL |
| clang-format-18, clang-tidy-18, buildifier | lint | package / GitHub release (sha256-verified) | brew (`llvm@18`, `buildifier`) | in WSL |
| WPILib 2026 | Glass, AdvantageScope, sim GUI | optional | optional | manual installer (not on winget) |

CI uses clang-format/clang-tidy 18 and buildifier 8.2.1. Homebrew buildifier and
the distribution LLVM on Arch/Fedora may differ; setup warns about these.
For clang-format 18, use the pinned wheel:
`python3 -m venv ~/.local/share/clang-format-18 && ~/.local/share/clang-format-18/bin/pip install clang-format==18.1.8`,
then format with `~/.local/share/clang-format-18/bin/clang-format -i <files>`.

## Flags

| Flag | Effect |
|---|---|
| `--check` | report each component ok/missing/wrong-version; install nothing; exit 1 if anything required is missing |
| `--yes` | non-interactive: answers prompts (incl. Homebrew install on macOS), passes `-y`/`--noconfirm` to package managers |
| `--no-studio` | skip Node 22/pnpm |
| `--no-rio` | skip JDK 17 and `./gradlew installRoboRioToolchain` |
| `--no-lint` | skip clang-format-18, clang-tidy-18, buildifier |
| `--verify` | build the launcher, `//talOS/bridge:hardware_node` and every node the 2026 config names, then run `--describe-only` on it |
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
  (`2026-robot\controller_processor\rio`, `gradlew.bat`), the Studio pnpm loop, WPILib tools.

## Verify

```sh
./tools/setup.sh --verify
bazel test //...
./sim.sh --sim-gateway --no-studio
```

The CI workflow's `sanitizers` job runs the C++ suite with AddressSanitizer +
UndefinedBehaviorSanitizer and ThreadSanitizer, with one leg per sanitizer on
Linux and on macOS. Run sanitizer builds in their own output
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
so their windows cannot hold. The macOS CI legs filter that tag (locally, add
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
- `third_party/apriltag_pjpeg_bounds.patch`: bounds and overflow fixes in AprilTag's JPEG
  decoder (`pjpeg.c`), used only by the offline `DetectImageFile` API; live
  capture passes raw GREY/YUYV buffers to `Detect`.
  `//talOS/vision/sanitizer:jpeg_loader_test` runs only 64 deterministic
  corrupt-scan variants of one fixture photograph under ASan/UBSan (memory
  safety only); intact photographs are decoded in
  `//talOS/vision/apriltag:cpu_apriltag_backend_test`.

## Editor setup

clangd needs a compilation database:

```sh
bazel run @hedron_compile_commands//:refresh_all
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `std::format` / `jthread` not found, `-std=c++23` unknown | GCC < 13. Install `g++-13`+ or use a supported distro |
| macOS libc++ / availability errors | `-fexperimental-library` and min macOS 13.4 are already in `.bazelrc`; update Xcode CLT |
| RIO build can't find the roboRIO compiler | `cd 2026-robot/controller_processor/rio && ./gradlew installRoboRioToolchain` (JDK 17). Setup also recognizes a toolchain from the WPILib installer; see `tools/setup.sh` for discovery paths |
| `$'\r': command not found`, `/usr/bin/env: 'bash\r'` | CRLF working copy. WSL/Linux: `git ls-files -z -- '*.sh' '*gradlew' \| xargs -0 sed -i 's/\r$//'` |
