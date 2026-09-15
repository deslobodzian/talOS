"""Scaffolding logic for standalone talOS nodes."""

import os
import re
from pathlib import Path
from typing import List, Optional

from templates import (
    class_name,
    render_bazelrc,
    render_build,
    render_message_fbs,
    render_module_bazel,
    render_node_h,
    render_node_py,
    render_node_test_cc,
    render_node_test_py,
    render_packet_h,
    render_packet_py,
    render_readme,
    render_subsystem_toml,
    render_test_robot_toml,
    render_test_sh,
)

NODE_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def validate_node_name(name: str) -> None:
    if not NODE_NAME_RE.match(name):
        raise ValueError(
            f"Invalid node name '{name}': must start with a lowercase letter "
            "and contain only lowercase letters, digits, and underscores."
        )


def find_talos_root() -> Optional[Path]:
    """Traverse upwards to find the talos workspace root (containing REPO.bazel or MODULE.bazel)."""
    current = Path(__file__).resolve().parent
    for parent in [current] + list(current.parents):
        if (parent / "MODULE.bazel").exists() and (parent / "talOS").exists():
            return parent
    return None


def resolve_sdk_path(out_dir: Path, explicit_sdk_path: Optional[str] = None) -> str:
    """Resolve the SDK path for local_path_override."""
    if explicit_sdk_path:
        return explicit_sdk_path

    talos_root = find_talos_root()
    if talos_root:
        try:
            rel = os.path.relpath(talos_root, out_dir.resolve())
            return rel
        except ValueError:
            pass
    return "../talos"


def scaffold_node(
    name: str,
    lang: str,
    out: Path | str,
    hardware: bool = False,
    sdk_path: Optional[str] = None,
    force: bool = False,
) -> List[Path]:
    """Scaffold a standalone talOS node repository."""
    validate_node_name(name)

    if lang not in ("cpp", "python"):
        raise ValueError(f"Unsupported language '{lang}': must be 'cpp' or 'python'")

    out_dir = Path(out).resolve()
    if out_dir.exists() and any(out_dir.iterdir()) and not force:
        raise FileExistsError(
            f"Refusing to overwrite existing files in '{out_dir}'. "
            "Pass --force to overwrite author-owned source."
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    testdata_dir = out_dir / "testdata"
    testdata_dir.mkdir(parents=True, exist_ok=True)

    resolved_sdk = resolve_sdk_path(out_dir, sdk_path)

    files = {
        "MODULE.bazel": render_module_bazel(name, lang, hardware, resolved_sdk),
        ".bazelrc": render_bazelrc(),
        "BUILD": render_build(name, lang, hardware),
        f"{name}_message.fbs": render_message_fbs(name),
        "packet.h": render_packet_h(name),
        "subsystem.toml": render_subsystem_toml(name, hardware),
        "testdata/robot.toml": render_test_robot_toml(name),
        f"testdata/{name}.toml": render_subsystem_toml(name, hardware),
        "test.sh": render_test_sh(),
        "README.md": render_readme(name, lang, hardware),
    }

    if lang == "cpp":
        files["node.h"] = render_node_h(name, hardware)
        files["node_test.cc"] = render_node_test_cc(name, hardware)
    elif lang == "python":
        files["packet.py"] = render_packet_py(name, hardware)
        files["node.py"] = render_node_py(name, hardware)
        files["node_test.py"] = render_node_test_py(name, hardware)

    written: List[Path] = []
    for rel_path, content in files.items():
        file_path = out_dir / rel_path
        file_path.parent.mkdir(parents=True, exist_ok=True)
        file_path.write_text(content, encoding="utf-8")
        if rel_path == "test.sh":
            os.chmod(file_path, 0o755)
        written.append(file_path)

    return written
