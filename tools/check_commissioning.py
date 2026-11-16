"""Fails a commit that commissions a robot or changes a physical CAN id.

    python3 tools/check_commissioning.py --base <rev>   # from the workspace

RULES.md 4: an agent never sets `commissioned = true` or changes a physical
CAN id in a robot config; a person does that at the robot
(docs/BRINGUP.md "Human commissioning"). This checks each non-merge commit
in `merge-base(<rev>, HEAD)..HEAD` on its own and reports every robot or
bench config line it

- adds that sets `commissioned = true`, or
- adds or removes that holds a `can_id` key (the only key talOS/hardware
  reads a CAN id from),

in any spelling TOML accepts (bare, quoted or dotted key, inline table,
trailing comment), but not in a comment or a string value. It over-reports
rather than miss one: a config file moved, renamed or added whole, or a
`can_id` line moved between files, also needs the trailer below.

A robot or bench config is any tracked `.toml` outside an `examples`,
`tests` or `fixtures` directory and outside any directory whose name ends
in `testdata`: examples carry placeholder ids (ARCHITECTURE.md
"Configuration"), and test fixtures are not loaded onto hardware.

The override is the person's own: the commit that makes the change carries
the trailer `Commissioned-By: <name>` (`git commit --trailer
"Commissioned-By: <name>"`). A trailer on a later commit does not cover an
earlier one, so it marks exactly the commit a person checked at the robot.
Agents never write this trailer.

Not checked: a change made while resolving a conflict in a merge commit
(merges are skipped); a squash that keeps a person's `Commissioned-By`
trailer exempts everything folded into it (only the commit's last trailer
block is read); a force push to `main` checks only its last commit (CI's
diff base for main is that push's parent).

Prints each finding as `<commit> <file>:<line>: <what>`; exits 1 when there
is one, 2 on a usage or git error. CI's lint job and tools/prepush.sh run it
through tools/source_lints.sh --base.
"""

import argparse
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


def findings_in_diff(diff):
    """(file, line, what) for each guarded line a unified -U0 diff changes."""
    path, old, new = None, 0, 0
    for line in diff.splitlines():
        if line.startswith(("--- ", "+++ ")):
            # `--- a/<path>` then `+++ b/<path>`; a created or deleted
            # file has /dev/null on one side and keeps the other's path.
            name = line[4:]
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


def check(base):
    """Every finding in merge-base(base, HEAD)..HEAD not covered by a trailer."""
    start = git("merge-base", base, "HEAD").strip()
    commits = git("rev-list", "--no-merges", "--reverse",
                  f"{start}..HEAD").split()
    found = []
    for commit in commits:
        trailer = git("log", "-1",
                      f"--format=%(trailers:key={TRAILER},valueonly)",
                      commit).strip()
        if trailer:
            continue
        # Fixed prefixes and no external diff: the output must not depend
        # on the user's git configuration (diff.noprefix, diff.external).
        diff = git("show", "--format=", "-U0", "--no-color", "--no-ext-diff",
                   "--no-renames", "--src-prefix=a/", "--dst-prefix=b/",
                   commit, "--", "*.toml")
        for path, line, what in findings_in_diff(diff):
            found.append(f"{commit[:12]} {path}:{line}: {what}")
    return found


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", required=True,
                        help="check the commits since the merge base with this")
    args = parser.parse_args(argv)
    try:
        found = check(args.base)
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
