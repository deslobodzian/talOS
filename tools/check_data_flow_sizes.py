"""Checks that docs/DATA_FLOW.md states the sizes the code defines.

    bazel test //tools:check_data_flow_sizes_test        # CI: bazel test //...
    python3 tools/check_data_flow_sizes.py SIZES [DOC]   # SIZES: the output
        # of `bazel run //tools:data_flow_sizes`, or - for stdin

RULES.md 9: a change to a UDP frame, payload codec, device cap,
shared-memory slot size or telemetry frame budget updates docs/DATA_FLOW.md
with its compile-time size assert. //tools:data_flow_sizes prints each such
size as `name value`, from the headers that define and assert it. In the
doc, a checked number follows the marker `<!-- size: name -->`, optionally
with `**` between (`<!-- size: frame.max_payload_bytes -->**1432 B**`);
thousands may carry commas (10,227). This prints

    docs/DATA_FLOW.md:<line>: <name> is <doc value>, the code's is <value>
    docs/DATA_FLOW.md:<line>: <name> is no size the code prints
    docs/DATA_FLOW.md:<line>: <name>'s marker has no number
    docs/DATA_FLOW.md: no marker states <name> (<value>)

and exits 1 when there is any. An unmarked number is not checked.
"""

import re
import sys

DOC = "docs/DATA_FLOW.md"
MARKER = re.compile(
    r"<!--\s*size\s*:([^>]*?)-->(?:\*\*)?[ \t]*([0-9]+(?:,[0-9]+)*(?:\.[0-9]+)?)?"
)
VALID_INT_TOKEN = re.compile(r"^(?:[0-9]+|[0-9]{1,3}(?:,[0-9]{3})+)$")


def parse_sizes(text):
    """`name value` lines as {name: int}."""
    sizes = {}
    for line in text.splitlines():
        if line.strip():
            name, value = line.split()
            if name in sizes:
                raise ValueError(f"duplicate size entry: {name}")
            sizes[name] = int(value)
    return sizes


def markers(doc_text):
    """(line number, name, stated value or None or raw bad string) per marker."""
    found = []
    for number, line in enumerate(doc_text.splitlines(), start=1):
        for match in MARKER.finditer(line):
            name = match.group(1).strip()
            raw = match.group(2)
            if raw is None:
                value = None
            elif VALID_INT_TOKEN.fullmatch(raw):
                value = int(raw.replace(",", ""))
            else:
                value = raw
            found.append((number, name, value))
    return found


def check(sizes, doc_text, doc=DOC):
    """Each problem, as a line to print."""
    if not sizes:
        return [f"{doc}: no sizes provided by data_flow_sizes"]
    problems, marked = [], set()
    for line, name, stated in markers(doc_text):
        marked.add(name)
        if name not in sizes:
            problems.append(f"{doc}:{line}: {name} is no size the code prints")
        elif stated is None:
            problems.append(f"{doc}:{line}: {name}'s marker has no number")
        elif stated != sizes[name]:
            problems.append(f"{doc}:{line}: {name} is {stated}, "
                            f"the code's is {sizes[name]}")
    problems += [f"{doc}: no marker states {name} ({value})"
                 for name, value in sorted(sizes.items())
                 if name not in marked]
    return problems


def main(argv):
    if len(argv) not in (2, 3):
        print(__doc__, file=sys.stderr)
        return 2
    if argv[1] == "-":
        sizes_text = sys.stdin.read()
    else:
        with open(argv[1], encoding="utf-8") as f:
            sizes_text = f.read()
    doc = argv[2] if len(argv) == 3 else DOC
    with open(doc, encoding="utf-8") as f:
        doc_text = f.read()
    problems = check(parse_sizes(sizes_text), doc_text, doc)
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
