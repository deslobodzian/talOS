# Installing a development environment

Development only. Deploying to the robot is [DEPLOYMENT.md](DEPLOYMENT.md) / `tools/deploy.sh`.

## Quick start

| OS | Command (from the repo root) |
|---|---|
| Linux, macOS | `./tools/setup.sh` |
| Windows | `powershell -ExecutionPolicy Bypass -File tools\setup.ps1` |

Both are idempotent: anything already present at an acceptable version is skipped.
Each run ends with a summary table and next-step commands.

## What Bazel provides vs what the host needs

Hermetic (downloaded by Bazel, see `MODULE.bazel`): Python 3.11, Node 22.14.0,
pnpm 9.15.9, all C++/JS dependencies. Bazel itself is pinned
by `.bazelversion` (8.6.0) and fetched by bazelisk.

| Component | Why | Linux | macOS | Windows |
|---|---|---|---|---|
| bazelisk (as `bazel`) | runs pinned Bazel 8.6.0 | release binary (sha256-verified) → `/usr/local/bin/bazel` | `brew install bazelisk` | in WSL |
| C++23 compiler | talOS (`std::format`, `std::jthread`) | GCC ≥ 13 (or clang ≥ 17 + libstdc++ 13) | Xcode CLT | in WSL |
| git, curl, unzip, zip, build-essential | Bazel fetches, builds | apt/dnf/pacman | Xcode CLT / brew | WSL + `Git.Git` (winget) |
| system `python3` | rules_python bootstrap, `tools/test_drivetrain.py`, clang-tidy script | package | Xcode CLT / brew | in WSL |
| JDK 17 (Temurin) | RoboRIO Gradle project, `./sim.sh` WPILib sim | package | brew | in WSL + `EclipseAdoptium.Temurin.17.JDK` |
| Node 22 + pnpm 9.15.9 (corepack) | Studio dev loop outside Bazel | package | brew | in WSL + `OpenJS.NodeJS.22` |
| `flatc` | none: `pnpm generate` runs Bazel's pinned `@flatbuffers//:flatc` (same as the `//studio:telemetry_ts` genrule) | – | – | run `pnpm generate` in WSL |
| clang-format-18, clang-tidy-18, buildifier 8.2.1 | lint, matches CI | package / GitHub release (sha256-verified) | brew / GitHub release | in WSL |
| WPILib 2026 | Glass, AdvantageScope, sim GUI | optional | optional | manual installer (not on winget) |

## Flags

| `setup.sh` | `setup.ps1` | Effect |
|---|---|---|
| `--check` | `-Check` | report each component ok/missing/wrong-version; install nothing; exit 1 if anything required is missing |
| `--yes` | `-Yes` | non-interactive: answers prompts (incl. Homebrew install on macOS), passes `-y` to package managers |
| `--no-studio` | `-NoStudio` | skip Node 22/pnpm |
| `--no-rio` | `-NoRio` | skip JDK 17 and `./gradlew installRoboRioToolchain` |
| `--no-lint` | `-NoLint` | skip clang-format-18, clang-tidy-18, buildifier |
| `--verify` | `-Verify` | build the launcher, `//talOS/bridge:hardware_node` and every node the 2026 config names, then run `--describe-only` on it |
| | `-Distro <name>` | WSL distro (default `Ubuntu-24.04`) |
| | `-NoNative` | skip winget installs on the Windows side |
| `-h`, `--help` | | usage |

Package managers: apt (Ubuntu/Debian), dnf (Fedora), pacman (Arch), brew (macOS).

`setup.ps1` exit codes: 0 ok; 1 something missing/failed (or `setup.sh`'s code);
2 reboot / first WSL launch / user creation needed, then rerun.

## Supported platforms

| Platform | Notes |
|---|---|
| Ubuntu 24.04+, Debian 13+, Fedora 39+, Arch | Ubuntu 22.04 ships GCC 11/12: too old |
| macOS 13.4+ | Xcode Command Line Tools (`xcode-select --install`) |
| Windows 11, Windows 10 22H2 | via WSL2 `Ubuntu-24.04` |

talOS does not build on native Windows: it uses POSIX shared memory
(`shm_open`/`mmap`) and signals. On Windows the Bazel tree is built inside WSL2.

## Windows notes

- `setup.ps1` installs WSL2 + the distro (`wsl --install -d Ubuntu-24.04`). If that
  needs a reboot or the first-launch user creation, it stops and says so; rerun after.
- Clone inside the WSL home (`~/talos`), not `/mnt/c/...`: Bazel on `/mnt/c` is very
  slow. The script warns when the repo is on `/mnt/c`.
- Line endings: `.gitattributes` forces LF on `*.sh` and `gradlew`; `setup.ps1` also
  converts existing tracked `*.sh`/`gradlew` to LF in place (local edits kept).
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
| RIO build can't find the roboRIO compiler | `cd 2026-robot/controller_processor/rio && ./gradlew installRoboRioToolchain` (JDK 17). It lands in `~/.gradle/toolchains/frc/2025/roborio` (toolchain year lags GradleRIO) |
| `$'\r': command not found`, `/usr/bin/env: 'bash\r'` | CRLF working copy. Windows: rerun `tools\setup.ps1`. WSL/Linux: `git ls-files -z -- '*.sh' '*gradlew' \| xargs -0 sed -i 's/\r$//'` |
