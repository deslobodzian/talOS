"""Fails a commit that commissions a robot or changes a physical CAN id.

    python3 tools/check_commissioning.py --base <rev>   # from the workspace

RULES.md 4: an agent never sets `commissioned = true` or changes a physical
CAN id in a robot config; a person does that at the robot
(docs/BRINGUP.md "Human commissioning"). This checks each commit in
`merge-base(<rev>, HEAD)..HEAD` (a merged branch's commits included) on its
own and reports every robot or bench config line it

- adds that sets `commissioned = true`, or
- adds or removes that holds a `can_id` key (the only key talOS/hardware
  reads a CAN id from),

in any spelling TOML accepts (bare, quoted or dotted key, inline table,
trailing comment), but not in a comment or a string value. It over-reports
rather than miss one: a config file moved, renamed or added whole, or a
`can_id` line moved between files, also needs the trailer below. A merge
commit's own change is its diff against git's automatic re-merge of its
parents (`git show --remerge-diff`). A clean merge has none; a line
written while resolving a conflict, or a stale copy of a config kept from
one side (which can revert a person's change made on the other), shows.
It over-reports a conflict resolved by picking one side of a `can_id`
line: the side not picked reads as removed, and the merge then carries the
trailer. git cannot re-merge an octopus merge, so one reports what its
diff against any parent shows.

Every diff reads attributes from the empty tree and no attributes file
(`--attr-source`), so a committed `.gitattributes` (`-diff`, `binary`,
`-merge`, `merge=<driver>`) neither hides a line nor makes the re-merge
keep the side the merge kept; external diff, textconv and renormalization
are off. A local `.git/info/attributes`, which git reads whatever the
attribute source, must hold no rule. The check needs git 2.40 or later;
otherwise it exits 2 naming why.

A robot or bench config is any tracked `.toml` outside an `examples`,
`tests` or `fixtures` directory and outside any directory whose name ends
in `testdata`: examples carry placeholder ids (ARCHITECTURE.md
"Configuration"), and test fixtures are not loaded onto hardware.

The override is the person's own: the commit that makes the change carries
the trailer `Commissioned-By: <name>` (`git commit --trailer
"Commissioned-By: <name>"`). A trailer on a later commit does not cover an
earlier one, so it marks exactly the commit a person checked at the robot.
Agents never write this trailer.

Accepted, since no diff tells it apart: the trailer vouches for its whole
commit, so a squash or amend that keeps a person's `Commissioned-By` in its
last trailer block (the only one read) exempts everything folded into it,
whoever wrote it. A person's commissioning commit therefore holds nothing
but what that person checked at the robot.

Prints each finding as `<commit> <file>:<line>: <what>`; exits 1 when there
is one, 2 on a usage or git error. CI's lint job and tools/prepush.sh run it
through tools/source_lints.sh --base. CI's base covers every commit the
event adds (a pull request's or topic branch's merge base with main; on
main, the previous tip, or after a force push its merge base with HEAD),
and the job fails, naming why, when no such base exists
(.github/workflows/continuous-integration.yaml, "Choose the diff base").
"""

import argparse
import codecs
import os
import re
import subprocess
import sys

TRAILER = "Commissioned-By"
# A key can stand at the line's start, after a dot (dotted key) or after
# `{` or `,` (inline table); keys are matched on code() of the line.
COMMISSIONED_TRUE = re.compile(r"(?:^|[\s.{,])commissioned\s*=\s*true\b")
CAN_ID = re.compile(r"(?:^|[\s.{,])can_id\s*=")
STRING = re.compile(r"\"(?:[^\"\\]|\\.)*\"|'[^']*'")
EXEMPT_DIRECTORIES = {"examples", "tests", "fixtures"}
HUNK = re.compile(r"^@@ -(\d+)(?:,\d+)? \+(\d+)(?:,\d+)? @@")


def is_robot_config(path):
    """Whether a tracked path is a robot or bench config this rule guards."""
    if not path.endswith(".toml"):
        return False
    directories = path.split("/")[:-1]
    return not any(d in EXEMPT_DIRECTORIES or d.endswith("testdata")
                   for d in directories)


def code(line):
    """A TOML line without its comment, a quoted key unquoted and every other
    string emptied, so a key matches however it is spelled and a comment or
    string value that mentions one does not."""
    out, at = [], 0
    while at < len(line):
        char = line[at]
        if char == "#":
            break
        if char in "\"'":
            match = STRING.match(line, at)
            if not match:
                break  # An unterminated string: the rest is its text.
            after = line[match.end():].lstrip()
            is_key = after.startswith(("=", "."))
            out.append(match.group(0)[1:-1] if is_key else '""')
            at = match.end()
            continue
        out.append(char)
        at += 1
    return "".join(out)


def git(*args):
    return subprocess.run(["git", *args], capture_output=True, text=True,
                          check=True).stdout


def diff_name(name):
    """A `--- `/`+++ ` line's file name as a path: git ends a name holding
    a space with a tab and C-quotes one holding a non-ASCII or control byte
    (core.quotePath), so a config named either way is still checked."""
    name = name.removesuffix("\t")
    if name.startswith('"') and name.endswith('"'):
        raw = codecs.escape_decode(name[1:-1].encode("utf-8"))[0]
        name = raw.decode("utf-8", "surrogateescape")
    return name


def findings_in_diff(diff):
    """(file, line, what) for each guarded line a unified -U0 diff changes."""
    path, old, new = None, 0, 0
    for line in diff.splitlines():
        if line.startswith(("--- ", "+++ ")):
            # `--- a/<path>` then `+++ b/<path>`; a created or deleted
            # file has /dev/null on one side and keeps the other's path.
            name = diff_name(line[4:])
            if name.startswith(("a/", "b/")):
                path = name[2:]
            continue
        if line.startswith("diff --git "):
            path = None
            continue
        match = HUNK.match(line)
        if match:
            old, new = int(match.group(1)), int(match.group(2))
            continue
        if path is None or not is_robot_config(path):
            continue
        if CONFLICT_MARKER.match(line):
            # A re-merge's conflict marker: not config, but a line.
            if line.startswith("+"):
                new += 1
            else:
                old += 1
            continue
        if line.startswith("+"):
            text = code(line[1:])
            if COMMISSIONED_TRUE.search(text):
                yield path, new, "sets commissioned = true"
            elif CAN_ID.search(text):
                yield path, new, f"sets a CAN id: {line[1:].strip()}"
            new += 1
        elif line.startswith("-"):
            if CAN_ID.search(code(line[1:])):
                yield path, old, f"removes a CAN id: {line[1:].strip()}"
            old += 1


# Fixed prefixes, no external diff or text conversion, and every file read
# as text: the output must not depend on the user's git configuration
# (diff.noprefix, diff.external, a textconv driver) or on a gitattributes
# `-diff`/`binary` mark, which the checked commit itself can add.
DIFF_FLAGS = ("-U0", "--no-color", "--no-ext-diff", "--no-textconv", "--text",
              "--no-renames", "--src-prefix=a/", "--dst-prefix=b/")


# `git show --remerge-diff` (2.36) re-merges a merge's parents and diffs
# the merge against that: exactly what the merge itself changed.
# `--attr-source` (2.40) reads attributes from a tree instead of the work
# tree, which is HEAD, the commit being checked.
NEEDED_GIT = (2, 40)
# A conflict marker in the re-merge, never a TOML line; the plain style,
# whatever merge.conflictStyle says, so no base section is shown.
CONFLICT_MARKER = re.compile(r"^[-+](?:<{7}|>{7}|\|{7})(?: |$)|^[-+]={7}$")


class CannotCheck(Exception):
    pass


def pinned_git():
    """The git options every diff here runs with: attributes from the empty
    tree (git knows its id without storing it, so nothing is written) and
    from no user file, so neither a committed `.gitattributes` (`-merge`,
    `merge=<driver>`, `binary`) nor core.attributesFile changes a diff or
    makes the re-merge keep the side a merge kept; no renormalization and
    the plain conflict style in the re-merge."""
    version = git("version")
    match = re.search(r"(\d+)\.(\d+)", version)
    if not match or tuple(map(int, match.groups())) < NEEDED_GIT:
        raise CannotCheck(
            f"needs git {'.'.join(map(str, NEEDED_GIT))} or later "
            f"(--attr-source, --remerge-diff); this is {version.strip()}")
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


def merge_findings(pinned, commit, parents):
    """The merge's own changes: its diff against git's automatic re-merge
    of its parents. A clean merge's is empty; a resolution that writes a
    new line, or keeps one side's stale copy of a file the other side
    changed, shows. git cannot re-merge more than two parents, so an
    octopus merge reports what its diff against any parent shows."""
    if len(parents) > 2:
        found = {}
        for parent in parents:
            diff = git(*pinned, "diff", *DIFF_FLAGS, parent, commit, "--",
                       "*.toml")
            for path, line, what in findings_in_diff(diff):
                found.setdefault((path, what), line)
        return [(path, line, what) for (path, what), line in found.items()]
    diff = git(*pinned, "show", "--remerge-diff", "--format=", *DIFF_FLAGS,
               commit, "--", "*.toml")
    return findings_in_diff(diff)


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
        if len(parents) > 1:
            findings = merge_findings(pinned, commit, parents)
        else:
            findings = findings_in_diff(git(*pinned, "show", "--format=",
                                            *DIFF_FLAGS, commit, "--",
                                            "*.toml"))
        for path, line, what in findings:
            found.append(f"{commit[:12]} {path}:{line}: {what}")
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
        print(f"check_commissioning: git {' '.join(error.cmd[1:])}: "
              f"{error.stderr.strip()}", file=sys.stderr)
        return 2
    for line in found:
        print(line)
    if found:
        print(f"check_commissioning: RULES.md 4: only a person commissions a "
              f"robot or changes a CAN id, in a commit carrying the trailer "
              f"`{TRAILER}: <name>` (docs/BRINGUP.md \"Human commissioning\"); "
              f"a person also adds it to a commit that moves, renames or adds "
              f"a whole config, or moves a can_id line between files")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
