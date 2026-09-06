# Desktop shell

Build from the repository root with `bazel build //studio/src-tauri:desktop` and launch with `bazel run //studio/src-tauri:desktop`. The target compiles Rust through rules_rust and embeds `//studio:web_dist`; Cargo.toml and Cargo.lock are dependency metadata for crate_universe. No standalone Cargo or Tauri CLI build is needed.

The build script stages Tauri configuration, icons, and capability files within Bazel's output tree and generates the application context there. Rust uses Tauri v2 binary channels for UDP payloads. The frontend acknowledges each packet; native in-flight capacity is 32 packets, with excess datagrams dropped. The socket read timeout bounds shutdown at 50 ms.

Native platform development libraries remain prerequisites: Apple SDK/Xcode tools on macOS; GTK 3, WebKitGTK 4.1 and associated development packages on Linux; MSVC tools, Windows SDK and WebView2 on Windows. Cross-platform native builds have not yet been validated. Crate-universe resolution and the Tauri build-script integration still need a successful full Bazel build. The installed system Cargo 1.82 cannot compile resolved edition-2024 dependencies; Bazel declares Rust 1.94.1 independently.

This target produces an executable; installers, signing, and application bundles are not configured.
