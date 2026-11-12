"""Compare the C ABI in node_api.h with its ctypes mirror, without loading it.

Checks the version, every exported function's restype and argtypes (types,
not only counts), every TALOS_* enum/#define value against _abi.py, and that
devices.Mode takes each member from _abi.MODE_*.
"""

import ast
import os
from pathlib import Path
import re
import unittest

from python.runfiles import runfiles

# C type (after normalization) -> the ctypes expression _abi.py must use.
_SCALARS = {
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


class ApiParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        files = runfiles.Create()
        read = lambda key: Path(files.Rlocation(os.environ[key])).read_text()
        cls.raw_header = read("TALOS_ABI_HEADER")
        cls.abi_tree = ast.parse(read("TALOS_ABI_PYTHON"))
        cls.devices_tree = ast.parse(read("TALOS_ABI_DEVICES"))
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

    def test_mode_members_come_from_abi(self):
        modes = {name[len("MODE_"):]
                 for name in _header_constants(self.header)
                 if name.startswith("MODE_")}
        members = {}
        for statement in self.devices_tree.body:
            if isinstance(statement, ast.ClassDef) and statement.name == "Mode":
                for item in statement.body:
                    if isinstance(item, ast.Assign):
                        members[item.targets[0].id] = ast.unparse(item.value)
        self.assertEqual(members, {m: f"_abi.MODE_{m}" for m in modes})


if __name__ == "__main__":
    unittest.main()
