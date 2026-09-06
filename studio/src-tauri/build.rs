use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn resolve_bazel_path(raw_path: &str, current_out_dir: &str) -> PathBuf {
    let original = PathBuf::from(raw_path);
    if original.exists() {
        return original;
    }

    if let (Some(out_idx), Some(path_idx)) =
        (current_out_dir.find("/execroot/"), raw_path.find("/execroot/"))
    {
        let in_sandbox = format!("{}{}", &current_out_dir[..out_idx], &raw_path[path_idx..]);
        let cand1 = PathBuf::from(&in_sandbox);
        if cand1.exists() {
            return cand1;
        }

        if let Some(sb_idx) = current_out_dir.find("/sandbox/") {
            let unsandboxed = format!("{}{}", &current_out_dir[..sb_idx], &raw_path[path_idx..]);
            let cand2 = PathBuf::from(&unsandboxed);
            if cand2.exists() {
                return cand2;
            }
        }
    }

    original
}

fn fix_bazel_sandbox_paths(output: &Path) {
    let out_s = output.to_string_lossy();
    for (key, value) in env::vars_os() {
        let key_str = key.to_string_lossy();
        if key_str.contains("PERMISSION_FILES_PATH") || key_str.contains("GLOBAL_SCOPE_SCHEMA_PATH") {
            let val_str = value.to_string_lossy();
            let resolved_path = resolve_bazel_path(&val_str, &out_s);

            if key_str.contains("PERMISSION_FILES_PATH") {
                if let Ok(content) = fs::read_to_string(&resolved_path) {
                    if let Ok(paths) = serde_json::from_str::<Vec<PathBuf>>(&content) {
                        let mut fixed_paths = Vec::new();
                        for p in paths {
                            let ps = p.to_string_lossy();
                            let fixed_p = resolve_bazel_path(&ps, &out_s);
                            fixed_paths.push(fixed_p);
                        }
                        let new_file = output.join(format!("fixed_{}", key_str.replace(':', "_")));
                        let new_json = serde_json::to_string(&fixed_paths).unwrap();
                        fs::write(&new_file, new_json).expect("write fixed permission files list");
                        env::set_var(&key, &new_file);
                        continue;
                    }
                }
            }

            env::set_var(&key, &resolved_path);
        }
    }
}

fn copy_tree(source: &Path, destination: &Path) {
    fs::create_dir_all(destination).expect("create build directory");
    for entry in fs::read_dir(source).expect("read build input") {
        let entry = entry.unwrap();
        let path = entry.path();
        let target = destination.join(entry.file_name());
        if path.is_dir() {
            copy_tree(&path, &target);
        } else {
            fs::copy(&path, target).expect("copy build input");
        }
    }
}

fn main() {
    let output = fs::canonicalize(env::var_os("OUT_DIR").unwrap()).unwrap();
    if let Some(dist) = env::var_os("STUDIO_FRONTEND_DIST") {
        // Tauri writes ACL schemas beside its manifest. Stage inputs inside the
        // Bazel output tree so build scripts never modify workspace sources.
        let dist = fs::canonicalize(dist).expect("Bazel web_dist output");
        let source = env::current_dir().unwrap();
        let project = output.join("project");
        fs::create_dir_all(&project).unwrap();
        fs::copy(source.join("Cargo.toml"), project.join("Cargo.toml")).unwrap();
        if source.join("Cargo.lock").exists() {
            fs::copy(source.join("Cargo.lock"), project.join("Cargo.lock")).unwrap();
        }
        // cargo_toml discovers the binary when processing package metadata.
        copy_tree(&source.join("src"), &project.join("src"));
        copy_tree(&source.join("icons"), &project.join("icons"));
        copy_tree(&source.join("capabilities"), &project.join("capabilities"));
        let mut config: serde_json::Value = serde_json::from_slice(&fs::read(source.join("tauri.conf.json")).unwrap()).unwrap();
        config["build"]["frontendDist"] = serde_json::Value::String(dist.to_string_lossy().into_owned());
        config["build"]["devUrl"] = serde_json::Value::Null;
        fs::write(project.join("tauri.conf.json"), serde_json::to_vec(&config).unwrap()).unwrap();
        env::set_current_dir(&project).unwrap();
        env::set_var("CARGO_MANIFEST_DIR", &project);
        // Tauri derives a Cargo-shaped target directory from OUT_DIR.
        let nested = output.join("build/studio/out");
        fs::create_dir_all(&nested).unwrap();
        env::set_var("OUT_DIR", &nested);
        fix_bazel_sandbox_paths(&output);
        tauri_build::try_build(tauri_build::Attributes::new().codegen(tauri_build::CodegenContext::new())).expect("generate desktop context");
        for entry in fs::read_dir(&nested).expect("read nested output directory") {
            let entry = entry.expect("read nested entry");
            let path = entry.path();
            let dest = output.join(entry.file_name());
            if path.is_dir() {
                copy_tree(&path, &dest);
            } else {
                fs::copy(&path, dest).expect("copy nested build output");
            }
        }
    } else {
        fix_bazel_sandbox_paths(&output);
        tauri_build::try_build(tauri_build::Attributes::new().codegen(tauri_build::CodegenContext::new())).expect("generate desktop context");
    }
    patch_tauri_build_context(&output);
}

fn patch_tauri_build_context(output: &Path) {
    let context_file = output.join("tauri-build-context.rs");
    if context_file.exists() {
        if let Ok(mut code) = fs::read_to_string(&context_file) {
            code = code.replace(
                "ResolvedCommand { context :",
                "ResolvedCommand { #[cfg(debug_assertions)] referenced_by : ::core::default::Default::default() , context :",
            );
            let out_s = output.to_string_lossy();
            if let Some(sb_idx) = out_s.find("/sandbox/") {
                if let Some(exec_idx) = out_s.find("/execroot/") {
                    let sandbox_segment = &out_s[sb_idx..exec_idx];
                    code = code.replace(sandbox_segment, "");
                }
            }
            fs::write(&context_file, code).expect("write patched tauri-build-context.rs");
        }
    }
}
