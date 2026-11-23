"""Checks that every relative link in the tracked markdown files resolves.

    python3 tools/check_links.py   # from the workspace root

For each `[text](target)` and `[label]: target` outside code, a relative
target must name a tracked file or directory, and a `#anchor` (on its own,
or after a markdown file) must name a heading of that file, by GitHub's
rules: the heading's text, lowercased, with every character but letters,
digits, spaces, `-` and `_` dropped and each space made a `-`; a repeated
heading gets `-1`, `-2`, ... in order. `<a id>`/`<a name>` anchors count
too. Links with a scheme (`https:`, `mailto:`) are not checked. Prints each
broken link as `file:line: target: why`; exits 1 when there is one.
"""

import os
import re
import subprocess
import sys
import urllib.parse

OPEN_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})(.*)$")
CLOSE_FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})[ \t]*$")
INLINE_CODE = re.compile(r"(`+)(?:(?!\1).)+?\1")
INLINE_LINK = re.compile(
    r"!?\[(?:[^\[\]]|\[[^\[\]]*\])*\]\(\s*(<[^>]*>|[^\s()]*(?:\([^\s()]*\)"
    r"[^\s()]*)*)(?:\s+(?:\"[^\"]*\"|'[^']*'|\([^)]*\)))?\s*\)")
REFERENCE = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(<[^>]*>|\S+)")
HEADING = re.compile(r"^\s{0,3}(#{1,6})\s+(.*?)\s*#*\s*$")
HTML_ANCHOR = re.compile(r"<a\s+(?:[^>]*\s)?(?:id|name)\s*=\s*[\"']([^\"']+)")
SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")


def lines_outside_code(text):
    """(line number, line) for each line outside fenced code."""
    fence_char, fence_len = None, 0
    for number, line in enumerate(text.splitlines(), 1):
        if fence_char is not None:
            close = CLOSE_FENCE.match(line)
            if (close and close.group(1)[0] == fence_char and
                    len(close.group(1)) >= fence_len):
                fence_char, fence_len = None, 0
            continue
        open_m = OPEN_FENCE.match(line)
        if open_m:
            marker, info = open_m.group(1), open_m.group(2)
            # In CommonMark, a backtick fence's info string cannot contain
            # backticks (so ```code``` at line start is inline code, not a fence).
            if not (marker[0] == "`" and "`" in info):
                fence_char, fence_len = marker[0], len(marker)
                continue
        yield number, line


def slug(heading):
    """GitHub's anchor for a heading's markdown text."""
    text = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", heading)  # links
    text = re.sub(r"<[^>]+>", "", text)  # inline HTML
    text = text.replace("`", "").replace("*", "")
    text = re.sub(r"(?<!\w)_+|_+(?!\w)", "", text)  # _emphasis_, not snake_case
    text = text.strip().lower()
    text = "".join(c for c in text if c.isalnum() or c in " -_")
    return text.replace(" ", "-")


def anchors(text):
    """Every anchor a markdown file defines outside fenced code."""
    found, seen = set(), {}
    for _, line in lines_outside_code(text):
        found.update(a.lower() for a in HTML_ANCHOR.findall(line))
        match = HEADING.match(line)
        if not match:
            continue
        base = slug(match.group(2))
        count = seen.get(base, 0)
        seen[base] = count + 1
        found.add(base if count == 0 else f"{base}-{count}")
    return found


def links(text):
    """(line number, target) for each link in a markdown file."""
    for number, line in lines_outside_code(text):
        line = INLINE_CODE.sub("", line)
        for match in INLINE_LINK.finditer(line):
            yield number, match.group(1)
        match = REFERENCE.match(line)
        if match:
            yield number, match.group(1)


def check(files, read, out=sys.stdout):
    """Prints each broken link among the markdown `files`; returns the
    number of them. `files` is every tracked path; `read` returns a file's
    text."""
    tracked = set(files)
    directories = {os.path.dirname(p) for p in files}
    for path in list(directories):
        while path:
            path = os.path.dirname(path)
            directories.add(path)
    cache = {}

    def anchors_of(path):
        if path not in cache:
            cache[path] = anchors(read(path))
        return cache[path]

    broken = 0
    for source in sorted(p for p in files if p.endswith(".md")):
        for number, target in links(read(source)):
            target = target.strip("<>")
            if not target or SCHEME.match(target):
                continue
            path, _, fragment = target.partition("#")
            path = urllib.parse.unquote(path.split("?")[0])
            fragment = urllib.parse.unquote(fragment)
            if path:
                resolved = os.path.normpath(
                    os.path.join(os.path.dirname(source), path))
                if path.startswith("/"):
                    resolved = os.path.normpath(path.lstrip("/"))
            else:
                resolved = source
            why = None
            if resolved == ".." or resolved.startswith("../"):
                why = "leaves the repository"
            elif resolved not in tracked and resolved not in directories:
                why = "no such tracked file or directory"
            elif (fragment and resolved.endswith(".md") and
                  resolved in tracked and
                  fragment.lower() not in anchors_of(resolved)):
                why = f"no heading or anchor #{fragment} in {resolved}"
            if why:
                broken += 1
                print(f"{source}:{number}: {target}: {why}", file=out)
    print(f"{broken} broken link(s)" if broken else "every relative link "
          "resolves", file=out)
    return broken


def tracked_files():
    listing = subprocess.run(["git", "ls-files", "-z"], capture_output=True,
                             check=True).stdout.decode()
    return [p for p in listing.split("\0") if p]


def read_file(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def main():
    return 1 if check(tracked_files(), read_file) else 0


if __name__ == "__main__":
    sys.exit(main())
