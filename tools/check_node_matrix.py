"""Checks node tests against the node test matrix (docs/TESTING.md §7).

    bazel run //tools:check_node_matrix -- [PATH...]

Each PATH is a node test file (`*_test.cc`, `*_test.py`) or a directory,
relative to the workspace. With none it checks every node package under
2026-robot/main_processor, examples, bench and tools/scaffold_testdata, and
every package a config's manifest (`*_manifest.bzl` `NODES`) under them
launches, such as //talOS/teleop and //talOS/telemetry. A node package is a
directory whose BUILD file declares a node (talos_cc_node, talos_py_node,
talos_cc_shared_node). Each of its test files in the language of its nodes (a
Python test of a C++ node tests the binding), or each file given, is checked
for the cases its role requires, by the matrix's names (Python:
test_<snake_case>). The role comes from the package's non-test sources:

- actuator: it constructs a Motor, DigitalOutput or PwmOutput;
- producer: otherwise it writes target lanes (TargetLane, OperatorArming,
  detail::TargetAccess);
- observer: otherwise;
- hosted: a Python package whose nodes are only `talos.cpp_node` wrappers,
  which is checked in the C++ package it hosts, so it requires no case.

Comments in the test file, each on its own line (`//` in C++, `#` in Python):

- `matrix: n/a <case>[, <case>] (reason)`: those cases do not apply.
- `matrix: role <role> (reason)`: overrides the inferred role. An actuator
  stays an actuator, and `hosted` is refused in a file that defines tests.

Case 12 never applies to a Python node. Prints one row per test file, then
each missing case or bad comment; exits 1 when there is any, 2 on a bad
argument.
"""

import os
import re
import sys

DEFAULT_ROOTS = ("2026-robot/main_processor", "examples", "bench",
                 "tools/scaffold_testdata")

# Case number -> name, per role. The observer's stale-input case takes the
# number of the actuator's stale case.
CASES = {
    "actuator": {
        1: "DrivesAtALiveLane",
        2: "DisabledRobotGoesNeutral",
        3: "StaleLaneGoesNeutral",
        4: "ReleasedLaneGoesNeutral",
        5: "NeverSentGoesNeutral",
        6: "DriverStationLossGoesNeutral",
        7: "DisconnectedFeedbackGoesNeutral",
        8: "NonFiniteDemandGoesNeutral",
        9: "BadParameterFailsConstruction",
        10: "OtherModesLaneIsIgnored",
        11: "EarlierGenerationTargetIsIgnored",
        12: "UpdateDoesNotAllocate",
    },
    "producer": {
        1: "SendsAtALiveInput",
        2: "DisabledRobotReleasesItsLanes",
        6: "DriverStationLossReleasesItsLanes",
        9: "BadParameterFailsConstruction",
        11: "NewGenerationDisarms",
        12: "UpdateDoesNotAllocate",
    },
    "observer": {
        1: "PublishesFromALiveInput",
        3: "StaleInputClearsValidity",
        9: "BadParameterFailsConstruction",
        12: "UpdateDoesNotAllocate",
    },
    "hosted": {},
}
PYTHON_EXEMPT = {12: "a Python tick allocates in the interpreter"}
CELL = {"present": "ok", "n/a": "na", "missing": "MISSING"}

NODE_RULE = re.compile(r"^\s*talos_(cc|py|cc_shared)_node\(", re.MULTILINE)
NODES = re.compile(r"^NODES = \[(.*?)\]", re.MULTILINE | re.DOTALL)
LABEL = re.compile(r'"//([^:"]*):[^"]*"')
ACTUATORS = r"(?:Motor|DigitalOutput|PwmOutput)"
ACTUATOR = (
    re.compile(rf"\btalos(?:::|\.(?:devices\.)?){ACTUATORS}\b"),
    # Python: from talos import Motor / from talos.devices import (Motor, ...)
    re.compile(rf"^\s*from\s+talos(?:\.devices)?\s+import\s+"
               rf"(?:\([^)]*|[^\n(]*)\b{ACTUATORS}\b", re.MULTILINE),
)
PRODUCER = re.compile(r"\b(?:TargetLane|OperatorArming|TargetAccess)\b")
HOSTED = re.compile(r"\btalos\.cpp_node\(")
CLASS = re.compile(r"^class\s", re.MULTILINE)
# Directives are whole-line comments, so a string or docstring that quotes
# one is not read as one.
ROLE = re.compile(r"^[ \t]*(?://|#)[ \t]*matrix:[ \t]*role[ \t]+(\w+)(.*)$",
                  re.MULTILINE)
NOT_APPLICABLE = re.compile(
    r"^[ \t]*(?://|#)[ \t]*matrix:[ \t]*n/a[ \t]+(\d+(?:[ \t]*,[ \t]*\d+)*)"
    r"(.*)$", re.MULTILINE)
CC_TEST = re.compile(r"^\s*TEST(?:_F|_P)?\(\s*\w+\s*,\s*(\w+)\s*\)",
                     re.MULTILINE)
PY_TEST = re.compile(r"^\s*def\s+(test_\w+)\s*\(", re.MULTILINE)


class MatrixError(Exception):
    """A test file's matrix comments cannot be honoured."""


def snake(name):
    """DrivesAtALiveLane -> drives_at_a_live_lane."""
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def language(path):
    return "py" if path.endswith(".py") else "cpp"


def reason(text):
    """The reason a directive gives after its operands, or ''."""
    return text.strip().lstrip("(-:–—").rstrip(")").strip()


def infer_role(sources):
    """The role of a node package from the text of its non-test sources."""
    if any(p.search(text) for p in ACTUATOR for text in sources):
        return "actuator"
    if any(PRODUCER.search(text) for text in sources):
        return "producer"
    if any(HOSTED.search(text) for text in sources) and not any(
            CLASS.search(text) for text in sources):
        return "hosted"
    return "observer"


def check(text, lang, role):
    """(role, {case: (status, detail)}) for one test file's text, `role`
    being the package's inferred role: status is "present", "n/a" or
    "missing"; detail is the case's name in `lang`, or the n/a reason.
    Raises MatrixError for a role override the rules refuse."""
    names = set((PY_TEST if lang == "py" else CC_TEST).findall(text))
    for match in ROLE.finditer(text):
        override, why = match.group(1), reason(match.group(2))
        if override not in CASES:
            raise MatrixError(f"unknown matrix role {override!r}")
        if not why:
            raise MatrixError(f"matrix: role {override} gives no reason")
        if role == "actuator" and override != "actuator":
            raise MatrixError(
                f"matrix: role {override} on an actuator node: its sources "
                f"construct an output device")
        if override == "hosted" and names:
            raise MatrixError("matrix: role hosted in a file that defines "
                              "tests")
        role = override
    not_applicable = {}
    for match in NOT_APPLICABLE.finditer(text):
        for case in re.findall(r"\d+", match.group(1)):
            not_applicable[int(case)] = reason(match.group(2))
    if lang == "py":
        for case, why in PYTHON_EXEMPT.items():
            not_applicable.setdefault(case, why)
    result = {}
    for case, name in CASES[role].items():
        name = f"test_{snake(name)}" if lang == "py" else name
        if name in names:
            result[case] = ("present", name)
        elif not_applicable.get(case):
            result[case] = ("n/a", not_applicable[case])
        elif case in not_applicable:
            result[case] = ("missing", f"{name} (n/a without a reason)")
        else:
            result[case] = ("missing", name)
    return role, result


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def is_test(name):
    return name.endswith(("_test.cc", "_test.py"))


def package_sources(directory):
    return [read(os.path.join(directory, f))
            for f in sorted(os.listdir(directory))
            if f.endswith((".h", ".cc", ".py")) and not is_test(f)
            and os.path.isfile(os.path.join(directory, f))]


def node_languages(directory):
    """The languages of the nodes `directory`'s BUILD file declares."""
    for build in ("BUILD", "BUILD.bazel"):
        path = os.path.join(directory, build)
        if os.path.isfile(path):
            return {"py" if rule == "py" else "cpp"
                    for rule in NODE_RULE.findall(read(path))}
    return set()


def declares_node(directory):
    return bool(node_languages(directory))


def walk(root):
    for directory, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if not d.startswith((".", "bazel-")))
        yield directory, files


def node_packages(workspace, root):
    """Node packages under `root`, then those a manifest under it launches
    from elsewhere in the workspace."""
    found = []
    for directory, files in walk(root):
        if declares_node(directory):
            found.append(directory)
        for f in files:
            if f.endswith("_manifest.bzl"):
                nodes = NODES.search(read(os.path.join(directory, f)))
                for package in LABEL.findall(nodes.group(1) if nodes else ""):
                    full = os.path.join(workspace, package)
                    if os.path.isdir(full) and declares_node(full):
                        found.append(full)
    return found


def targets(workspace, paths):
    """[(test file, or a node package without one, its inferred role)]."""
    found, seen = [], set()
    for path in paths or DEFAULT_ROOTS:
        full = os.path.join(workspace, path)
        if os.path.isfile(full):
            found.append((full, infer_role(
                package_sources(os.path.dirname(full)))))
        elif os.path.isdir(full):
            for package in node_packages(workspace, full):
                key = os.path.realpath(package)
                if key in seen:
                    continue
                seen.add(key)
                role = infer_role(package_sources(package))
                # A test in another language than the package's nodes (a
                # Python test of a C++ node's binding) is not a node test.
                languages = node_languages(package)
                tests = [os.path.join(package, f)
                         for f in sorted(os.listdir(package))
                         if is_test(f) and language(f) in languages]
                found.extend((test, role) for test in tests or [package])
        else:
            raise FileNotFoundError(path)
    return found


def main(argv):
    workspace = os.environ.get("BUILD_WORKSPACE_DIRECTORY", os.getcwd())
    try:
        found = targets(workspace, argv)
    except FileNotFoundError as missing:
        print(f"check_node_matrix: no such file or directory: {missing}",
              file=sys.stderr)
        return 2
    failed = False
    for path, role in found:
        shown = os.path.relpath(path, workspace)
        if os.path.isdir(path):
            # A node package with no test file has no case at all.
            result = {case: ("missing", "no test file")
                      for case in CASES[role]}
        else:
            try:
                role, result = check(read(path), language(path), role)
            except MatrixError as error:
                failed = True
                print(f"{shown}  [{role}]  {error}")
                continue
        cells = " ".join(f"{case}:{CELL[status]}"
                         for case, (status, _) in result.items())
        print(f"{shown}  [{role}]  {cells}")
        for case, (status, detail) in result.items():
            if status == "missing":
                failed = True
                print(f"  missing case {case}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
