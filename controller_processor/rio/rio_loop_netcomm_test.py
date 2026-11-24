"""Source-level guard that the RoboRIO loop makes no Driver Station NetComm call.

REQ-RT-24: the RoboRIO loop (Robot.cpp) reads whole Driver Station snapshots
from its DriverStationReader helper thread and makes no Driver Station NetComm
refresh or mode-observation call on the loop thread.
"""

import os
import re
import unittest

from python.runfiles import runfiles

FORBIDDEN_NETCOMM_SYMBOLS = (
    "HAL_RefreshDSData",
    "RefreshData",
    "HAL_ObserveUserProgramDisabled",
    "HAL_ObserveUserProgramAutonomous",
    "HAL_ObserveUserProgramTeleop",
    "HAL_ObserveUserProgramTest",
)

FORBIDDEN_PATTERNS = tuple(
    (sym, re.compile(rf"\b{re.escape(sym)}\b"))
    for sym in FORBIDDEN_NETCOMM_SYMBOLS
)


def _is_ident_char(ch):
    return ch == "_" or ("a" <= ch <= "z") or ("A" <= ch <= "Z") or ("0" <= ch <= "9")


def _strip_cpp_comments_and_strings(source):
    """Returns `(code_only, line_numbers)` with comments, string literals, and
    character literals blanked out while preserving line numbers.

    Handles C++ line splices (`\\` + newline), `//` and `/* ... */` comments,
    raw string literals (`R"delim(...)delim"`), quoted string/char literals,
    and C++14 numeric digit separators (`1'000'000`) so a digit separator does
    not open a fake character literal.
    """
    # Phase 1: splice backslash-newlines while recording each character's
    # original 1-based line number.
    spliced = []
    spliced_lines = []
    line = 1
    i = 0
    n = len(source)
    while i < n:
        ch = source[i]
        if ch == "\\" and i + 1 < n:
            if source[i + 1] == "\n":
                line += 1
                i += 2
                continue
            if source[i + 1] == "\r" and i + 2 < n and source[i + 2] == "\n":
                line += 1
                i += 3
                continue
        if ch == "\r":
            if i + 1 < n and source[i + 1] == "\n":
                i += 1
            spliced.append("\n")
            spliced_lines.append(line)
            line += 1
            i += 1
            continue
        spliced.append(ch)
        spliced_lines.append(line)
        if ch == "\n":
            line += 1
        i += 1

    src = "".join(spliced)
    m = len(src)
    out = []
    out_lines = []

    def emit(ch, lineno):
        out.append(ch)
        out_lines.append(lineno)

    i = 0
    while i < m:
        ch = src[i]
        nxt = src[i + 1] if i + 1 < m else ""

        # Line comment.
        if ch == "/" and nxt == "/":
            emit(" ", spliced_lines[i])
            i += 2
            while i < m and src[i] != "\n":
                i += 1
            continue

        # Block comment.
        if ch == "/" and nxt == "*":
            end = src.find("*/", i + 2)
            stop = m if end == -1 else end + 2
            for k in range(i, stop):
                emit("\n" if src[k] == "\n" else " ", spliced_lines[k])
            i = stop
            continue

        # Identifier or raw string literal ([u8|u|U|L]R"delim(...)delim").
        if ch == "_" or ("a" <= ch <= "z") or ("A" <= ch <= "Z"):
            j = i + 1
            while j < m and _is_ident_char(src[j]):
                j += 1
            ident = src[i:j]
            if ident in ("R", "u8R", "uR", "UR", "LR") and j < m and src[j] == '"':
                paren = src.find("(", j + 1)
                if paren != -1:
                    delim = src[j + 1 : paren]
                    closing = ")" + delim + '"'
                    end = src.find(closing, paren + 1)
                    stop = m if end == -1 else end + len(closing)
                    for k in range(i, stop):
                        emit("\n" if src[k] == "\n" else " ", spliced_lines[k])
                    i = stop
                    continue
            for k in range(i, j):
                emit(src[k], spliced_lines[k])
            i = j
            continue

        # pp-number (consumes C++14 digit separators such as 1'000'000).
        if ("0" <= ch <= "9") or (ch == "." and "0" <= nxt <= "9"):
            if ch == ".":
                emit(ch, spliced_lines[i])
                i += 1
            emit(src[i], spliced_lines[i])
            i += 1
            while i < m:
                c = src[i]
                c_next = src[i + 1] if i + 1 < m else ""
                if c == "'" and c_next and _is_ident_char(c_next):
                    emit(c, spliced_lines[i])
                    emit(c_next, spliced_lines[i + 1])
                    i += 2
                    continue
                if c in ("e", "E", "p", "P") and c_next in ("+", "-"):
                    emit(c, spliced_lines[i])
                    emit(c_next, spliced_lines[i + 1])
                    i += 2
                    continue
                if _is_ident_char(c) or c == ".":
                    emit(c, spliced_lines[i])
                    i += 1
                    continue
                break
            continue

        # String or character literal.
        if ch in ('"', "'"):
            quote = ch
            emit(" ", spliced_lines[i])
            i += 1
            while i < m:
                c = src[i]
                if c == "\\":
                    emit(" ", spliced_lines[i])
                    i += 1
                    if i < m and src[i] != "\n":
                        emit(" ", spliced_lines[i])
                        i += 1
                    continue
                if c == quote:
                    emit(" ", spliced_lines[i])
                    i += 1
                    break
                if c == "\n":
                    emit("\n", spliced_lines[i])
                    i += 1
                    break
                emit(" ", spliced_lines[i])
                i += 1
            continue

        emit(ch, spliced_lines[i])
        i += 1

    return "".join(out), out_lines


def find_forbidden_netcomm_calls(source):
    """Returns a sorted list of `(lineno, symbol)` hits in `source`."""
    code, line_numbers = _strip_cpp_comments_and_strings(source)
    hits = []
    for sym, pattern in FORBIDDEN_PATTERNS:
        for match in pattern.finditer(code):
            lineno = line_numbers[match.start()] if line_numbers else 1
            hits.append((lineno, sym))
    hits.sort()
    return hits


class RioLoopNetcommTest(unittest.TestCase):
    # REQ-RT-24
    def test_robot_cpp_makes_no_driver_station_netcomm_call(self):
        path = runfiles.Create().Rlocation(os.environ["RIO_ROBOT_CPP"])
        self.assertTrue(path and os.path.isfile(path), f"missing Robot.cpp: {path}")
        with open(path, encoding="utf-8") as f:
            source = f.read()
        self.assertIn(
            "DriverStationReader",
            source,
            "Robot.cpp should delegate Driver Station NetComm calls to DriverStationReader",
        )
        hits = find_forbidden_netcomm_calls(source)
        self.assertEqual(
            hits,
            [],
            "Robot.cpp must not call Driver Station NetComm APIs on the loop thread: "
            + ", ".join(f"{sym} at line {line}" for line, sym in hits),
        )

    # REQ-RT-24
    def test_scanner_catches_calls_and_ignores_comments_and_strings(self):
        clean = (
            'constexpr uint64_t kMicros = 1\'000\'000;\n'
            '// HAL_RefreshDSData(); RefreshData();\n'
            '/* HAL_ObserveUserProgramTeleop(); */\n'
            'const char* a = "HAL_ObserveUserProgramDisabled";\n'
            'const char* b = R"delim(HAL_ObserveUserProgramAutonomous)delim";\n'
        )
        self.assertEqual(find_forbidden_netcomm_calls(clean), [])

        for sym in FORBIDDEN_NETCOMM_SYMBOLS:
            with self.subTest(symbol=sym):
                mutated = clean + f"  {sym}();\n"
                self.assertEqual(find_forbidden_netcomm_calls(mutated), [(6, sym)])

        spliced = "HAL_Refresh\\\nDSData();\n"
        self.assertEqual(
            find_forbidden_netcomm_calls(spliced),
            [(1, "HAL_RefreshDSData")],
        )


if __name__ == "__main__":
    unittest.main()
