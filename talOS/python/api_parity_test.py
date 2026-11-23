"""Compare the C ABI in node_api.h with its ctypes mirror, without loading it.

Checks the version, every exported function's restype and argtypes (types,
not only counts), every TALOS_* enum/#define value against _abi.py, that
every Python IntEnum across talos (__init__.py, devices.py, mechanisms.py)
takes each member from _abi.<PREFIX>_*, and that every talos/*.py module stays
within the talOS/node_api C ABI boundary (REQ-CONTRACT-15).
"""

import ast
import os
from pathlib import Path
import re
import unittest

from python.runfiles import runfiles

# C type (after normalization) -> the ctypes expression _abi.py must use.
_SCALARS = {
    "int8_t": "c_int8", "uint8_t": "c_uint8", "int16_t": "c_int16",
    "int32_t": "c_int32", "int64_t": "c_int64", "uint32_t": "c_uint32",
    "uint64_t": "c_uint64", "double": "c_double", "float": "c_float",
    "char*": "c_char_p", "void": "None", "void*": "c_void_p",
    "TalosCallback": "CALLBACK", "TalosMessageCallback": "MESSAGE_CALLBACK",
    "TalosFactory": "FACTORY",
    # Opaque handles pass as c_void_p.
    "TalosNode*": "c_void_p", "TalosSim*": "c_void_p",
    "TalosSimHardware*": "c_void_p",
}
# A const input buffer may also be bytes (c_char_p).
_ALSO_ACCEPTED = {("void*", "c_char_p")}


def _ctypes_of(c_type):
    c_type = re.sub(r"\bconst\b", "", c_type)
    c_type = re.sub(r"\s+", "", c_type)
    if c_type in _SCALARS:
        return c_type, _SCALARS[c_type]
    if c_type.endswith("*"):
        pointee = c_type[:-1]
        if pointee.startswith("Talos"):
            return c_type, f"_P({pointee[len('Talos'):]})"
        return c_type, f"_P({_ctypes_of(pointee)[1]})"
    raise AssertionError(f"no ctypes mapping for C type {c_type!r}")


def _parameter_type(parameter):
    # Drop the parameter name: the last identifier unless the text is a bare
    # type (one token, or ending in '*').
    parameter = parameter.strip()
    match = re.fullmatch(r"(.*?[\s*])(\w+)", parameter)
    return match.group(1) if match else parameter


def _header_constants(header):
    constants = {}
    for match in re.finditer(r"\bTALOS_(\w+)\s*=\s*([^,}\n]+)", header):
        constants[match.group(1)] = _value(
            ast.parse(match.group(2).strip(), mode="eval").body)
    for match in re.finditer(r"^#define\s+TALOS_(\w+)\s+(\d+)\s*$", header,
                             re.MULTILINE):
        constants[match.group(1)] = int(match.group(2))
    return constants


def _value(node):
    # Integer literals and shifts such as 1 << 3; no names or calls.
    for child in ast.walk(node):
        if isinstance(child, (ast.Name, ast.Call, ast.Attribute)):
            raise AssertionError(f"not a literal: {ast.unparse(node)}")
    return eval(compile(ast.Expression(node), "<abi>", "eval"),  # noqa: S307
                {"__builtins__": {}})


# Every IntEnum across talos/*.py and the TALOS_<PREFIX>* family in node_api.h
# its members must bind from _abi.
_ENUM_PREFIXES = {
    ("TALOS_ABI_INIT", "Wake"): "WAKE_",
    ("TALOS_ABI_DEVICES", "Mode"): "MODE_",
    ("TALOS_ABI_DEVICES", "Alliance"): "ALLIANCE_",
    ("TALOS_ABI_DEVICES", "ArmingInputKind"): "ARMING_INPUT_",
    ("TALOS_ABI_MECHANISMS", "PositionPhase"): "POSITION_PHASE_",
    ("TALOS_ABI_MECHANISMS", "PositionFault"): "POSITION_FAULT_",
    ("TALOS_ABI_MECHANISMS", "VelocityPhase"): "VELOCITY_PHASE_",
    ("TALOS_ABI_MECHANISMS", "RollerCommand"): "ROLLER_COMMAND_",
    ("TALOS_ABI_MECHANISMS", "RollerPhase"): "ROLLER_PHASE_",
}


def _is_int_enum(node):
    for base in node.bases:
        if isinstance(base, ast.Name) and base.id == "IntEnum":
            return True
        if isinstance(base, ast.Attribute) and base.attr == "IntEnum":
            return True
    return False


# REQ-CONTRACT-15: Python reaches talOS only through talOS/node_api and
# reimplements no wire format, transport, or OS IPC boundary.
_FORBIDDEN_IMPORTS = frozenset({
    "socket",
    "_socket",
    "mmap",
    "multiprocessing",
    "select",
    "selectors",
    "fcntl",
    "zlib",
    "binascii",
    "asyncio",
    "http",
    "urllib",
    "subprocess",
    "posix_ipc",
    "_struct",
    "_ctypes",
})
_DYNAMIC_IMPORT_SYMBOLS = frozenset({"__import__", "import_module"})
_DYNAMIC_EXEC_SYMBOLS = frozenset({"eval", "exec"})
_DYNAMIC_SYMBOLS = _DYNAMIC_IMPORT_SYMBOLS | _DYNAMIC_EXEC_SYMBOLS
_CTYPES_DLL_SYMBOLS = frozenset({
    "CDLL",
    "cdll",
    "PyDLL",
    "pydll",
    "WinDLL",
    "windll",
    "OleDLL",
    "oledll",
    "LibraryLoader",
})
# Relative paths under talOS/python/talos/ granted narrow exceptions.
_STRUCT_ALLOWED = frozenset({"_messages.py", "parity.py"})
_CDLL_ALLOWED = frozenset({"_abi.py", "__init__.py"})
_IMPORTLIB_UTIL_ALLOWED = frozenset({"__init__.py"})


def _check_python_abi_boundary(rel_path, source):
    """Return boundary violations for one talOS/python/talos/ module."""
    tree = ast.parse(source, filename=rel_path)
    violations = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                name = alias.name
                root = name.split(".", 1)[0]
                if root in _FORBIDDEN_IMPORTS:
                    violations.append(
                        f"{rel_path}:{node.lineno}: forbidden import {name!r}")
                elif root == "struct" and rel_path not in _STRUCT_ALLOWED:
                    violations.append(
                        f"{rel_path}:{node.lineno}: struct import is only "
                        f"allowed in {sorted(_STRUCT_ALLOWED)}")
                elif root == "importlib":
                    if not (rel_path in _IMPORTLIB_UTIL_ALLOWED
                            and name == "importlib.util"):
                        violations.append(
                            f"{rel_path}:{node.lineno}: forbidden importlib "
                            f"import {name!r}")
                elif ((name == "ctypes.util"
                       or name.startswith("ctypes.util."))
                      and rel_path not in _CDLL_ALLOWED):
                    violations.append(
                        f"{rel_path}:{node.lineno}: ctypes.util is only "
                        f"allowed in {sorted(_CDLL_ALLOWED)}")
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name in _DYNAMIC_SYMBOLS:
                    violations.append(
                        f"{rel_path}:{node.lineno}: forbidden dynamic "
                        f"import/exec symbol {alias.name!r}")
            if node.level == 0 and node.module:
                mod = node.module
                root = mod.split(".", 1)[0]
                if root in _FORBIDDEN_IMPORTS:
                    violations.append(
                        f"{rel_path}:{node.lineno}: forbidden import from "
                        f"{mod!r}")
                elif root == "struct" and rel_path not in _STRUCT_ALLOWED:
                    violations.append(
                        f"{rel_path}:{node.lineno}: struct import is only "
                        f"allowed in {sorted(_STRUCT_ALLOWED)}")
                elif root == "importlib":
                    if not (rel_path in _IMPORTLIB_UTIL_ALLOWED
                            and mod == "importlib.util"):
                        violations.append(
                            f"{rel_path}:{node.lineno}: forbidden importlib "
                            f"import from {mod!r}")
                    else:
                        for alias in node.names:
                            if alias.name == "*":
                                violations.append(
                                    f"{rel_path}:{node.lineno}: star import "
                                    f"from {mod!r} is forbidden")
                elif root == "ctypes" and rel_path not in _CDLL_ALLOWED:
                    if mod == "ctypes.util" or mod.startswith("ctypes.util."):
                        violations.append(
                            f"{rel_path}:{node.lineno}: ctypes.util is only "
                            f"allowed in {sorted(_CDLL_ALLOWED)}")
                    for alias in node.names:
                        if (alias.name == "*"
                                or alias.name in _CTYPES_DLL_SYMBOLS):
                            violations.append(
                                f"{rel_path}:{node.lineno}: ctypes "
                                f"{alias.name!r} is only allowed in "
                                f"{sorted(_CDLL_ALLOWED)}")
        elif isinstance(node, ast.Name):
            if node.id in _DYNAMIC_SYMBOLS:
                violations.append(
                    f"{rel_path}:{node.lineno}: forbidden dynamic "
                    f"import/exec reference {node.id!r}")
        elif isinstance(node, ast.Attribute):
            if node.attr in _DYNAMIC_SYMBOLS:
                violations.append(
                    f"{rel_path}:{node.lineno}: forbidden dynamic "
                    f"import/exec attribute {node.attr!r}")
            elif (node.attr in _CTYPES_DLL_SYMBOLS
                  and rel_path not in _CDLL_ALLOWED):
                violations.append(
                    f"{rel_path}:{node.lineno}: ctypes {node.attr!r} is only "
                    f"allowed in {sorted(_CDLL_ALLOWED)}")
            elif node.attr == "struct" and rel_path not in _STRUCT_ALLOWED:
                violations.append(
                    f"{rel_path}:{node.lineno}: struct attribute access is "
                    f"only allowed in {sorted(_STRUCT_ALLOWED)}")
        elif isinstance(node, ast.Call):
            func = node.func
            is_getattr = ((isinstance(func, ast.Name) and func.id == "getattr")
                          or (isinstance(func, ast.Attribute)
                              and func.attr == "getattr"))
            if (is_getattr and len(node.args) >= 2
                    and isinstance(node.args[1], ast.Constant)
                    and isinstance(node.args[1].value, str)):
                attr = node.args[1].value
                if attr in _DYNAMIC_SYMBOLS:
                    violations.append(
                        f"{rel_path}:{node.lineno}: forbidden "
                        f"getattr({attr!r})")
                elif (attr in _CTYPES_DLL_SYMBOLS
                      and rel_path not in _CDLL_ALLOWED):
                    violations.append(
                        f"{rel_path}:{node.lineno}: getattr({attr!r}) is only "
                        f"allowed in {sorted(_CDLL_ALLOWED)}")
                elif attr == "struct" and rel_path not in _STRUCT_ALLOWED:
                    violations.append(
                        f"{rel_path}:{node.lineno}: getattr('struct') is only "
                        f"allowed in {sorted(_STRUCT_ALLOWED)}")
    return violations


class ApiParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        files = runfiles.Create()
        read = lambda key: Path(files.Rlocation(os.environ[key])).read_text()
        cls.raw_header = read("TALOS_ABI_HEADER")
        cls.abi_tree = ast.parse(read("TALOS_ABI_PYTHON"))
        cls.module_trees = {
            key: ast.parse(read(key))
            for key in ("TALOS_ABI_INIT", "TALOS_ABI_DEVICES",
                        "TALOS_ABI_MECHANISMS")
        }
        cls.talos_sources = {}
        raw_srcs = os.environ["TALOS_PY_SRCS"].split(":")
        for rlocation in raw_srcs:
            if not rlocation:
                continue
            _, sep, rel_path = rlocation.partition("/talOS/python/talos/")
            if not sep:
                raise AssertionError(
                    f"unexpected talos source rlocation: {rlocation!r}")
            cls.talos_sources[rel_path] = Path(
                files.Rlocation(rlocation)).read_text()
        cls.header = re.sub(r"/\*.*?\*/|//[^\n]*", "", cls.raw_header,
                            flags=re.DOTALL)

    def _abi_assignments(self):
        values = {}
        for statement in self.abi_tree.body:
            if isinstance(statement, ast.Assign):
                for target in statement.targets:
                    if isinstance(target, ast.Name):
                        values[target.id] = statement.value
        return values

    # REQ-CONTRACT-13: the Python ABI table matches node_api.h
    def test_functions_version_and_types(self):
        assignments = self._abi_assignments()
        version = ast.literal_eval(assignments["_EXPECTED_ABI_VERSION"])
        bindings = {}
        for table in ("_SIGNATURES", "_TESTING_SIGNATURES"):
            node = assignments[table]
            self.assertIsInstance(node, ast.Dict)
            for key, value in zip(node.keys, node.values):
                name = ast.literal_eval(key)
                self.assertNotIn(name, bindings)
                self.assertIsInstance(value, ast.Tuple)
                self.assertIsInstance(value.elts[1], ast.List)
                bindings[name] = (ast.unparse(value.elts[0]),
                                  [ast.unparse(a) for a in value.elts[1].elts])
        declared = re.search(r"^#define\s+TALOS_NODE_ABI_VERSION\s+(\d+)u?\s*$",
                             self.raw_header, re.MULTILINE)
        self.assertIsNotNone(declared)
        self.assertEqual(int(declared.group(1)), version)
        body = self.header.split('extern "C" {', 1)[1].rsplit("}", 1)[0]
        body = re.sub(r"typedef[^;]*;", "", body)
        declarations = {}
        for match in re.finditer(
                r"([\w\s*]*?[\s*])(talos_\w+)\s*\(([^()]*)\)\s*;", body):
            result, name, parameters = match.groups()
            self.assertNotIn(name, declarations)
            parameters = parameters.strip()
            parameters = ([] if parameters in ("", "void")
                          else parameters.split(","))
            declarations[name] = (_ctypes_of(result),
                                  [_ctypes_of(_parameter_type(p))
                                   for p in parameters])
        self.assertTrue(declarations)
        self.assertEqual(declarations.keys(), bindings.keys())
        for name, (result, parameters) in declarations.items():
            with self.subTest(function=name):
                restype, argtypes = bindings[name]
                self.assertEqual(result[1], restype)
                self.assertEqual(len(parameters), len(argtypes))
                for index, ((c_type, expected), actual) in enumerate(
                        zip(parameters, argtypes)):
                    if (c_type, actual) in _ALSO_ACCEPTED:
                        continue
                    self.assertEqual(expected, actual,
                                     f"argument {index} ({c_type})")

    def test_constants_match_header(self):
        constants = _header_constants(self.header)
        constants.pop("NODE_ABI_VERSION", None)  # checked as the version
        self.assertIn("MODE_VELOCITY", constants)
        assignments = self._abi_assignments()
        for name, value in constants.items():
            with self.subTest(constant=name):
                self.assertIn(name, assignments)
                self.assertEqual(_value(assignments[name]), value)

    def test_all_int_enums_come_from_abi(self):
        constants = _header_constants(self.header)
        found = {}
        for key, tree in self.module_trees.items():
            for statement in tree.body:
                if (isinstance(statement, ast.ClassDef) and
                        _is_int_enum(statement)):
                    members = {}
                    for item in statement.body:
                        if isinstance(item, ast.Assign):
                            members[item.targets[0].id] = ast.unparse(
                                item.value)
                    found[(key, statement.name)] = members
        self.assertEqual(found.keys(), _ENUM_PREFIXES.keys())
        for (key, class_name), prefix in _ENUM_PREFIXES.items():
            with self.subTest(enum=class_name):
                expected_names = {
                    name[len(prefix):]
                    for name in constants
                    if name.startswith(prefix)
                }
                self.assertTrue(expected_names, f"no TALOS_{prefix}* in header")
                self.assertEqual(
                    found[(key, class_name)],
                    {m: f"_abi.{prefix}{m}" for m in expected_names})

    # REQ-CONTRACT-15: Python reaches talOS only through talOS/node_api and
    # reimplements no wire format.
    def test_python_abi_boundary(self):
        self.assertTrue(self.talos_sources, "no talOS/python/talos/*.py files")
        expected_allowed = (_STRUCT_ALLOWED | _CDLL_ALLOWED
                            | _IMPORTLIB_UTIL_ALLOWED)
        self.assertTrue(
            expected_allowed.issubset(self.talos_sources),
            f"allowlist names missing files: "
            f"{sorted(expected_allowed - set(self.talos_sources))}")
        violations = []
        for rel_path, source in sorted(self.talos_sources.items()):
            violations.extend(_check_python_abi_boundary(rel_path, source))
        self.assertEqual(violations, [], "\n".join(violations))

    # REQ-CONTRACT-15: boundary check fails closed on forbidden imports,
    # disallowed struct/CDLL usage, and dynamic import escapes.
    def test_python_abi_boundary_rejects_violations(self):
        for mod in sorted(_FORBIDDEN_IMPORTS):
            for snippet in (
                f"import {mod}\n",
                f"import {mod}.sub as alias\n",
                f"from {mod} import thing\n",
                f"def f():\n    import {mod}\n",
            ):
                with self.subTest(module=mod, snippet=snippet.strip()):
                    self.assertTrue(
                        _check_python_abi_boundary("devices.py", snippet),
                        f"expected violation for {snippet!r}")

        dynamic_escapes = [
            ("__init__.py", "__import__('socket')\n"),
            ("devices.py", "builtins.__import__('socket')\n"),
            ("devices.py", "from builtins import __import__ as imp\n"),
            ("devices.py", "getattr(builtins, '__import__')('socket')\n"),
            ("devices.py", "import importlib\nimportlib.import_module('socket')\n"),
            ("__init__.py", "import importlib\nimportlib.import_module('socket')\n"),
            ("__init__.py", "from importlib import import_module as im\n"),
            ("__init__.py", "from importlib import *\n"),
            ("__init__.py", "getattr(importlib, 'import_module')('socket')\n"),
            ("devices.py", "import importlib.util\n"),
            ("__init__.py", "import importlib.machinery\n"),
            ("devices.py", "exec('import socket')\n"),
            ("devices.py", "eval(\"__import__('socket')\")\n"),
        ]
        for rel_path, snippet in dynamic_escapes:
            with self.subTest(escape=snippet.strip(), file=rel_path):
                self.assertTrue(
                    _check_python_abi_boundary(rel_path, snippet),
                    f"expected violation in {rel_path} for {snippet!r}")

        disallowed_struct = [
            ("devices.py", "import struct\n"),
            ("testing.py", "import struct as s\n"),
            ("_abi.py", "from struct import pack\n"),
            ("__init__.py", "from struct import Struct\n"),
            ("mechanisms.py", "from talos import _messages\n_messages.struct.pack('<d', 1.0)\n"),
            ("devices.py", "getattr(mod, 'struct')\n"),
            ("sub/_messages.py", "import struct\n"),
            ("_messages.py", "import _struct\n"),
        ]
        for rel_path, snippet in disallowed_struct:
            with self.subTest(disallowed_struct=snippet.strip(), file=rel_path):
                self.assertTrue(
                    _check_python_abi_boundary(rel_path, snippet),
                    f"expected struct violation in {rel_path} for {snippet!r}")

        for rel_path in sorted(_STRUCT_ALLOWED):
            with self.subTest(allowed_struct=rel_path):
                self.assertEqual(
                    _check_python_abi_boundary(rel_path, "import struct\n"),
                    [])

        disallowed_cdll = [
            ("devices.py", "import ctypes\nctypes.CDLL('libfoo.so')\n"),
            ("testing.py", "import ctypes as c\nc.CDLL('libfoo.so')\n"),
            ("mechanisms.py", "import ctypes\nctypes.cdll.LoadLibrary('libfoo.so')\n"),
            ("devices.py", "from ctypes import CDLL as L\nL('libfoo.so')\n"),
            ("_messages.py", "from ctypes import cdll\n"),
            ("devices.py", "from ctypes import *\n"),
            ("devices.py", "getattr(ctypes, 'CDLL')('libfoo.so')\n"),
            ("devices.py", "builtins.getattr(ctypes, 'cdll')\n"),
            ("devices.py", "import ctypes\nctypes.PyDLL('libfoo.so')\n"),
            ("devices.py", "import ctypes\nctypes.LibraryLoader(object)\n"),
            ("devices.py", "import ctypes.util\n"),
            ("sub/_abi.py", "import ctypes\nctypes.CDLL('libfoo.so')\n"),
            ("_abi.py", "import _ctypes\n"),
        ]
        for rel_path, snippet in disallowed_cdll:
            with self.subTest(disallowed_cdll=snippet.strip(), file=rel_path):
                self.assertTrue(
                    _check_python_abi_boundary(rel_path, snippet),
                    f"expected CDLL violation in {rel_path} for {snippet!r}")

        for rel_path in sorted(_CDLL_ALLOWED):
            with self.subTest(allowed_cdll=rel_path):
                self.assertEqual(
                    _check_python_abi_boundary(
                        rel_path, "import ctypes\nctypes.CDLL('libtalos.so')\n"),
                    [])
        self.assertEqual(
            _check_python_abi_boundary(
                "devices.py", "import ctypes\nctypes.c_int32(0)\n"),
            [])


if __name__ == "__main__":
    unittest.main()
