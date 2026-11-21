"""Fails a commit that commissions a robot or changes a physical CAN address.

    python3 tools/check_commissioning.py --base <rev>   # from the workspace

RULES.md 4: an agent never sets `commissioned = true` or changes a physical
CAN id (or moves a device to another CAN bus, which changes its physical CAN
address) in a robot config; a person does that at the robot
(docs/BRINGUP.md "Human commissioning"). This checks each commit in
`merge-base(<rev>, HEAD)..HEAD` (a merged branch's commits included) on its
own. For every non-exempt tracked `*.toml` file (matched case-insensitively)
that commit changes, it reads the parent and commit blobs directly from git
(`git cat-file blob`), parses both as TOML 1.0 with `tomllib` (failing closed
on invalid UTF-8, TOML parse errors, or non-regular file modes), and reports
any addition, change or removal of:

- `commissioned` at any table path (non-table values),
- `can_id` or `bus` at any table path (non-table values), including table
  header swaps (`[...motors.lead]` <-> `[...motors.follower]`), inline tables,
  multiline or triple-quoted strings, dotted keys and quoted/unicode-escaped
  keys,
- `include` at any table path (non-table values), or
- `[[subsystems]]` `path` or `defaults` entries (and `[subsystem]` `defaults`),
  which re-point device configs.

Because each file is parsed as TOML rather than scanned line by line, comments,
string values, unrelated keys, and tables named like a guarded key (such as
`[subsystems.bus]`) do not trigger findings, while any valid TOML 1.0 spelling
of a guarded key is seen. A config file moved, renamed, deleted, or added
whole with guarded keys is also reported.

For a two-parent merge commit with a single merge base, the check compares the
merged commit's guarded keys against the 3-way merge of its parents relative
to their merge base: a clean merge reports nothing (each parent branch's own
commits are checked on their own), while a resolution that writes or edits a
guarded key, keeps a stale copy that reverts the other side's change, or
resolves a conflict on a guarded key is reported. For a criss-cross merge
(multiple merge bases), an unrelated-history merge (no merge base), or an
octopus merge (more than two parents), the check compares the merge commit
against every parent and reports any guarded key differing from any parent.

Every git tree diff reads attributes from the empty tree and no attributes
file (`--attr-source`), so a committed `.gitattributes` (`-diff`, `binary`,
`-merge`, `merge=<driver>`) has no effect. A local `.git/info/attributes`,
which git reads whatever the attribute source, must hold no rule. The check
needs git 2.40 or later and Python 3.11 or later (`tomllib`); otherwise it
exits 2 naming why.

A robot or bench config is any tracked `*.toml` (matched case-insensitively)
outside an exact-case `examples`, `tests` or `fixtures` directory and outside
any directory whose name ends in `testdata`: examples carry placeholder ids
(ARCHITECTURE.md "Configuration"), and test fixtures are not loaded onto
hardware. Any non-exempt tracked symlink (file or directory symlink, with or
without a `.toml` suffix) also fails closed with `non-regular file mode 120000`,
since a directory symlink could redirect a config tree into an exempt path.

The override is the person's own: the commit that makes the change carries
the trailer `Commissioned-By: <name>` (`git commit --trailer
"Commissioned-By: <name>"`). A trailer on a later commit does not cover an
earlier one, so it marks exactly the commit a person checked at the robot.
Agents never write this trailer.

Accepted bounds:

- Non-exempt `*.toml` files are matched case-insensitively (e.g. `.TOML`,
  `.Toml`), and any non-exempt symlink (file or directory symlink, regardless
  of extension) fails closed; regular non-TOML files are not inspected (robot
  and bench configs are TOML).
- Any path under exact-case `EXEMPT_DIRECTORIES` (`examples`, `tests`,
  `fixtures`) or a `*testdata` directory is exempt per REQ-DEV-33, even if real
  config were placed there.
- No diff tells it apart: the trailer vouches for its whole commit, so a
  squash or amend that keeps a person's `Commissioned-By` in its last trailer
  block (the only one read) exempts everything folded into it, whoever wrote
  it. A person's commissioning commit therefore holds nothing but what that
  person checked at the robot.
- Each changed file is compared on its own parsed TOML rather than resolving
  cross-file targets; re-pointing `include`, `[[subsystems]]` `path`/`defaults`
  or `[subsystem]` `defaults` is guarded directly instead.

Prints each finding as `<commit> <file>: <what>`; exits 1 when there is one,
2 on a usage, environment or git error. CI's lint job and tools/prepush.sh run
it through tools/source_lints.sh --base. CI's base covers every commit the
event adds (a pull request's or topic branch's merge base with main; on
main, the previous tip, or after a force push its merge base with HEAD),
and the job fails, naming why, when no such base exists
(.github/workflows/continuous-integration.yaml, "Choose the diff base").
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11 fails closed
    tomllib = None

TRAILER = "Commissioned-By"
EXEMPT_DIRECTORIES = {"examples", "tests", "fixtures"}
GUARDED_KEYS = frozenset({"commissioned", "can_id", "bus", "include"})
REGULAR_MODES = frozenset({"100644", "100755"})
SYMLINK_MODE = "120000"
BARE_KEY = re.compile(r"^[A-Za-z0-9_-]+$")
KIND_LABEL = {
    "commissioned": "commissioned",
    "can_id": "a CAN id",
    "bus": "a CAN bus",
    "include": "include",
    "subsystem_pointer": "a subsystem config pointer",
}

# `--attr-source` (2.40) reads attributes from a tree instead of the work
# tree, which is HEAD, the commit being checked.
NEEDED_GIT = (2, 40)


class CannotCheck(Exception):
    pass


def is_exempt_path(path):
    """Whether a tracked path lies under an exempt directory (exact case)."""
    directories = path.split("/")[:-1]
    return any(d in EXEMPT_DIRECTORIES or d.endswith("testdata")
               for d in directories)


def is_robot_config(path):
    """Whether a tracked path is a robot or bench config this rule guards."""
    if not path.lower().endswith(".toml"):
        return False
    return not is_exempt_path(path)


def git(*args):
    return subprocess.run(["git", *args], capture_output=True, text=True,
                          check=True).stdout


def git_bytes(*args):
    return subprocess.run(["git", *args], capture_output=True,
                          check=True).stdout


def pinned_git():
    """The git options every tree diff here runs with: attributes from the
    empty tree (git knows its id without storing it, so nothing is written)
    and from no user file, so neither a committed `.gitattributes` nor
    core.attributesFile affects git's behavior; also checks git >= 2.40 and
    Python tomllib availability."""
    if tomllib is None:
        raise CannotCheck("needs Python 3.11 or later (tomllib)")
    version = git("version")
    match = re.search(r"(\d+)\.(\d+)", version)
    if not match or tuple(map(int, match.groups())) < NEEDED_GIT:
        raise CannotCheck(
            f"needs git {'.'.join(map(str, NEEDED_GIT))} or later "
            f"(--attr-source); this is {version.strip()}")
    info = git("rev-parse", "--git-path", "info/attributes").strip()
    try:
        with open(info, encoding="utf-8", errors="replace") as f:
            rules = [r for r in f.read().splitlines()
                     if r.strip() and not r.lstrip().startswith("#")]
    except FileNotFoundError:
        rules = []
    if rules:
        raise CannotCheck(f"{info} sets attributes, which git applies to every "
                          f"diff whatever the attribute source; empty it")
    empty_tree = git("hash-object", "-t", "tree", os.devnull).strip()
    return (f"--attr-source={empty_tree}",
            "-c", f"core.attributesFile={os.devnull}",
            "-c", "merge.renormalize=false",
            "-c", "merge.conflictStyle=merge")


def canonical(value):
    """Type-preserving canonical key for comparing parsed TOML values
    (prevents Python's True == 1, False == 0, 1 == 1.0, and NaN != NaN)."""
    if isinstance(value, bool):
        return ("bool", value)
    if isinstance(value, int):
        return ("int", value)
    if isinstance(value, float):
        if math.isnan(value):
            return ("float", "nan")
        return ("float", value, math.copysign(1.0, value))
    if isinstance(value, str):
        return ("str", value)
    if isinstance(value, dict):
        return ("table", tuple(sorted((k, canonical(v))
                                      for k, v in value.items())))
    if isinstance(value, list):
        return ("array", tuple(canonical(v) for v in value))
    return (type(value).__name__, repr(value))


def format_path(path):
    """Human-readable TOML path from tagged path segments."""
    parts = []
    for tag, val in path:
        if tag == "key":
            seg = val if BARE_KEY.match(val) else json.dumps(val)
            parts.append(f".{seg}" if parts else seg)
        elif tag == "item":
            seg = val if BARE_KEY.match(val) else json.dumps(val)
            parts.append(f"[{seg}]")
        else:
            parts.append(f"[{val}]")
    return "".join(parts)


def format_value(val):
    if isinstance(val, bool):
        return "true" if val else "false"
    if isinstance(val, str):
        return json.dumps(val)
    return repr(val)


def extract_guarded(doc):
    """Map tagged key path -> (kind, canonical_value, raw_value) for every
    guarded setting in a parsed TOML document."""
    out = {}

    def record(path, kind, val):
        out[path] = (kind, canonical(val), val)

    def walk_subsystems_manifest(items, base_path):
        names = [
            item.get("name")
            for item in items
            if isinstance(item, dict)
            and isinstance(item.get("name"), str)
            and item.get("name")
        ]
        unique_names = (len(names) == len(items)
                        and len(set(names)) == len(names))
        for idx, item in enumerate(items):
            selector = (("item", item["name"]) if unique_names
                        else ("index", idx))
            entry_path = base_path + (selector,)
            if isinstance(item, dict):
                pointer_keys = (("path", "defaults") if unique_names
                                else ("name", "path", "defaults"))
                for pkey in pointer_keys:
                    if pkey in item:
                        record(entry_path + (("key", pkey),),
                               "subsystem_pointer", item[pkey])
                walk(item, entry_path)
            else:
                record(entry_path, "subsystem_pointer", item)

    def walk(node, path):
        if isinstance(node, dict):
            for key, val in node.items():
                child_path = path + (("key", key),)
                # Tables named like a guarded key (e.g. [subsystems.bus]) are
                # containers, not guarded scalar values; still recurse into
                # them so any guarded keys inside are checked.
                if key in GUARDED_KEYS and not isinstance(val, dict):
                    record(child_path, key, val)
                if path == () and key == "subsystem" and isinstance(val, dict):
                    if "defaults" in val:
                        record(child_path + (("key", "defaults"),),
                               "subsystem_pointer", val["defaults"])
                if path == () and key == "subsystems" and isinstance(val, list):
                    walk_subsystems_manifest(val, child_path)
                    continue
                if isinstance(val, (dict, list)):
                    walk(val, child_path)
        elif isinstance(node, list):
            for idx, item in enumerate(node):
                if isinstance(item, (dict, list)):
                    walk(item, path + (("index", idx),))

    walk(doc, ())
    return out


def rec_equal(a, b):
    if a is None or b is None:
        return a is b
    return a[0] == b[0] and a[1] == b[1]


def describe_change(key_path, old_rec, new_rec):
    where = format_path(key_path)
    if old_rec is None:
        kind, _, new_val = new_rec
        if kind == "commissioned":
            return f"sets commissioned = {format_value(new_val)} at {where}"
        return f"sets {KIND_LABEL[kind]}: {where} = {format_value(new_val)}"
    if new_rec is None:
        kind, _, old_val = old_rec
        if kind == "commissioned":
            return f"removes commissioned = {format_value(old_val)} at {where}"
        return f"removes {KIND_LABEL[kind]}: {where} = {format_value(old_val)}"
    kind, _, new_val = new_rec
    _, _, old_val = old_rec
    if kind == "commissioned":
        return (f"changes commissioned at {where} from "
                f"{format_value(old_val)} to {format_value(new_val)}")
    return (f"changes {KIND_LABEL[kind]}: {where} from "
            f"{format_value(old_val)} to {format_value(new_val)}")


def compare_guarded(before, after):
    findings = []
    for key_path in sorted(set(before) | set(after), key=format_path):
        old_rec = before.get(key_path)
        new_rec = after.get(key_path)
        if not rec_equal(old_rec, new_rec):
            findings.append(describe_change(key_path, old_rec, new_rec))
    return findings


def compare_merge_guarded(base_map, p1_map, p2_map, merged_map):
    findings = []
    keys = set(base_map) | set(p1_map) | set(p2_map) | set(merged_map)
    for key_path in sorted(keys, key=format_path):
        vb = base_map.get(key_path)
        v1 = p1_map.get(key_path)
        v2 = p2_map.get(key_path)
        vm = merged_map.get(key_path)
        if rec_equal(v1, v2):
            if not rec_equal(vm, v1):
                findings.append(describe_change(key_path, v1, vm))
        elif not rec_equal(v1, vb) and rec_equal(v2, vb):
            if not rec_equal(vm, v1):
                findings.append(describe_change(key_path, v1, vm))
        elif not rec_equal(v2, vb) and rec_equal(v1, vb):
            if not rec_equal(vm, v2):
                findings.append(describe_change(key_path, v2, vm))
        else:
            kind = (vm or v1 or v2 or vb)[0]
            where = format_path(key_path)
            s1 = "<absent>" if v1 is None else format_value(v1[2])
            s2 = "<absent>" if v2 is None else format_value(v2[2])
            sm = "<absent>" if vm is None else format_value(vm[2])
            findings.append(
                f"resolves a merge conflict on {KIND_LABEL[kind]}: {where} "
                f"(parents: {s1} vs {s2}, merged: {sm})")
    return findings


def parse_raw_diff_tree(raw):
    """Parse `git diff-tree --no-commit-id -r --no-renames --raw -z
    --no-abbrev` output into {path: (old_entry, new_entry)} for robot configs
    and non-exempt symlinks."""
    tokens = raw.split(b"\0")
    entries = {}
    i = 0
    while i < len(tokens):
        meta = tokens[i]
        i += 1
        if not meta:
            continue
        if not meta.startswith(b":"):
            raise CannotCheck(
                f"unexpected diff-tree record: {meta.decode('utf-8', 'replace')!r}")
        fields = meta[1:].decode("ascii").split(" ")
        if len(fields) != 5:
            raise CannotCheck(f"unexpected diff-tree fields: {fields!r}")
        old_mode, new_mode, old_oid, new_oid, status = fields
        if status.startswith(("R", "C")):
            raise CannotCheck(f"unexpected rename/copy status {status!r}")
        if i >= len(tokens):
            raise CannotCheck("truncated diff-tree output")
        path = tokens[i].decode("utf-8", "surrogateescape")
        i += 1
        is_symlink = old_mode == SYMLINK_MODE or new_mode == SYMLINK_MODE
        if not (is_robot_config(path)
                or (is_symlink and not is_exempt_path(path))):
            continue
        old_entry = (None if old_mode == "000000" or set(old_oid) == {"0"}
                     else (old_mode, old_oid))
        new_entry = (None if new_mode == "000000" or set(new_oid) == {"0"}
                     else (new_mode, new_oid))
        entries[path] = (old_entry, new_entry)
    return entries


def changed_toml_entries(pinned, rev_a, rev_b):
    """Map changed non-exempt *.toml (case-insensitive) or symlink path ->
    (old_entry, new_entry). When rev_a is None, diffs root commit rev_b
    against the empty tree."""
    args = [*pinned, "diff-tree", "--no-commit-id", "-r", "--no-renames",
            "--raw", "-z", "--no-abbrev"]
    if rev_a is None:
        args.extend(["--root", rev_b])
    else:
        args.extend([rev_a, rev_b])
    return parse_raw_diff_tree(git_bytes(*args))


def tree_entry(pinned, rev, path):
    """Return (mode, oid) for `path` at `rev`, or None if absent or a tree."""
    raw = git_bytes(*pinned, "--literal-pathspecs", "ls-tree", "-z",
                    rev, "--", path)
    if not raw:
        return None
    for record in raw.split(b"\0"):
        if not record:
            continue
        meta, entry_path = record.split(b"\t", 1)
        if entry_path.decode("utf-8", "surrogateescape") != path:
            continue
        mode, obj_type, oid = meta.decode("ascii").split(" ")
        if obj_type == "tree":
            return None
        return mode, oid
    return None


def load_guarded(entry, side_label=None, parse_toml=True):
    """Load and parse a git tree entry (mode, oid) into (guarded_map, error).
    Fails closed on non-regular file modes, invalid UTF-8, or invalid TOML."""
    if entry is None:
        return {}, None
    mode, oid = entry
    suffix = f" ({side_label})" if side_label else ""
    if mode not in REGULAR_MODES:
        return None, f"non-regular file mode {mode}{suffix}"
    if not parse_toml:
        return {}, None
    raw = git_bytes("cat-file", "blob", oid)
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as err:
        return None, f"invalid TOML{suffix}: invalid UTF-8: {err}"
    try:
        doc = tomllib.loads(text)
    except tomllib.TOMLDecodeError as err:
        return None, f"invalid TOML{suffix}: {err}"
    return extract_guarded(doc), None


def pair_findings(path, old_entry, new_entry):
    """Findings for one changed config between `old_entry` and `new_entry`."""
    if old_entry == new_entry:
        return []
    parse_toml = is_robot_config(path)
    new_map, new_err = load_guarded(new_entry, parse_toml=parse_toml)
    old_map, old_err = load_guarded(old_entry, side_label="parent",
                                    parse_toml=parse_toml)
    errors = [err for err in (new_err, old_err) if err is not None]
    if errors:
        return [(path, err) for err in errors]
    return [(path, what) for what in compare_guarded(old_map, new_map)]


def merge_bases(p1, p2):
    proc = subprocess.run(["git", "merge-base", "--all", p1, p2],
                          capture_output=True, text=True, check=False)
    if proc.returncode == 0:
        return proc.stdout.splitlines()
    if proc.returncode == 1:
        return []
    raise subprocess.CalledProcessError(proc.returncode, proc.args,
                                        proc.stdout, proc.stderr)


def merge_findings(pinned, commit, parents):
    """The merge's own changes to guarded TOML keys."""
    per_parent = [changed_toml_entries(pinned, parent, commit)
                  for parent in parents]
    paths = sorted({path for entries in per_parent for path in entries})
    if not paths:
        return []

    if len(parents) == 2:
        bases = merge_bases(parents[0], parents[1])
        if len(bases) == 1:
            base_rev = bases[0]
            out = []
            for path in paths:
                if path in per_parent[0]:
                    e1, em = per_parent[0][path]
                    e2 = (per_parent[1][path][0] if path in per_parent[1]
                          else em)
                else:
                    e2, em = per_parent[1][path]
                    e1 = em
                eb = tree_entry(pinned, base_rev, path)
                if (em == e1 and e2 == eb) or (em == e2 and e1 == eb):
                    continue
                parse_toml = is_robot_config(path)
                m_map, m_err = load_guarded(em, parse_toml=parse_toml)
                p1_map, p1_err = load_guarded(e1, side_label="first parent",
                                              parse_toml=parse_toml)
                p2_map, p2_err = load_guarded(e2, side_label="second parent",
                                              parse_toml=parse_toml)
                b_map, b_err = load_guarded(eb, side_label="merge base",
                                            parse_toml=parse_toml)
                errors = [e for e in (m_err, p1_err, p2_err, b_err)
                          if e is not None]
                if errors:
                    for err in dict.fromkeys(errors):
                        out.append((path, err))
                    continue
                for what in compare_merge_guarded(b_map, p1_map, p2_map, m_map):
                    out.append((path, what))
            return out

    # Octopus merge (>2 parents), criss-cross merge (>1 merge base), or
    # unrelated-history merge (0 merge bases): compare against every parent.
    seen = set()
    out = []
    for entries in per_parent:
        for path, (old_entry, new_entry) in sorted(entries.items()):
            for finding in pair_findings(path, old_entry, new_entry):
                if finding not in seen:
                    seen.add(finding)
                    out.append(finding)
    return out


def commit_findings(pinned, commit, parents):
    if len(parents) > 1:
        return merge_findings(pinned, commit, parents)
    parent = parents[0] if parents else None
    entries = changed_toml_entries(pinned, parent, commit)
    out = []
    for path in sorted(entries):
        old_entry, new_entry = entries[path]
        out.extend(pair_findings(path, old_entry, new_entry))
    return out


def check(base):
    """Every finding in merge-base(base, HEAD)..HEAD not covered by a trailer."""
    pinned = pinned_git()
    start = git("merge-base", base, "HEAD").strip()
    found = []
    for entry in git("rev-list", "--parents", "--reverse",
                     f"{start}..HEAD").splitlines():
        commit, *parents = entry.split()
        trailer = git("log", "-1",
                      f"--format=%(trailers:key={TRAILER},valueonly)",
                      commit).strip()
        if trailer:
            continue
        for path, what in commit_findings(pinned, commit, parents):
            found.append(f"{commit[:12]} {path}: {what}")
    return found


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", required=True,
                        help="check the commits since the merge base with this")
    args = parser.parse_args(argv)
    try:
        found = check(args.base)
    except CannotCheck as error:
        print(f"check_commissioning: {error}", file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as error:
        stderr = (error.stderr.decode("utf-8", "replace")
                  if isinstance(error.stderr, bytes) else error.stderr)
        print(f"check_commissioning: git {' '.join(error.cmd[1:])}: "
              f"{stderr.strip()}", file=sys.stderr)
        return 2
    for line in found:
        print(line)
    if found:
        print(f"check_commissioning: RULES.md 4: only a person commissions a "
              f"robot or changes a CAN id or bus, in a commit carrying the "
              f"trailer `{TRAILER}: <name>` (docs/BRINGUP.md "
              f"\"Human commissioning\"); a person also adds it to a commit "
              f"that moves, renames or adds a whole config, or re-points an "
              f"`include` or subsystem config path/defaults")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
