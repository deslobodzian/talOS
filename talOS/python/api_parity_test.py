"""Compare the C ABI declarations with the ctypes signature tables, without loading it."""

import ast
import os
from pathlib import Path
import re
import unittest

from python.runfiles import runfiles


class ApiParityTest(unittest.TestCase):
    def test_exported_signatures(self):
        files = runfiles.Create()
        header = Path(files.Rlocation(os.environ["TALOS_ABI_HEADER"])).read_text()
        source = Path(files.Rlocation(os.environ["TALOS_ABI_PYTHON"])).read_text()
        tree = ast.parse(source)
        bindings = {}
        version = None
        # _abi.py declares signatures as data and applies each with getattr(lib,
        # name). Inspect the AST rather than importing (and loading a dylib).
        for statement in tree.body:
            if not isinstance(statement, ast.Assign):
                continue
            names = {target.id for target in statement.targets
                     if isinstance(target, ast.Name)}
            if "_EXPECTED_ABI_VERSION" in names:
                version = ast.literal_eval(statement.value)
            if names & {"_SIGNATURES", "_TESTING_SIGNATURES"}:
                self.assertIsInstance(statement.value, ast.Dict)
                for key, value in zip(statement.value.keys, statement.value.values):
                    name = ast.literal_eval(key)
                    self.assertNotIn(name, bindings)
                    self.assertIsInstance(value, ast.Tuple)
                    self.assertIsInstance(value.elts[1], ast.List)
                    bindings[name] = len(value.elts[1].elts)
        declared_version = re.search(
            r"^#define\s+TALOS_NODE_ABI_VERSION\s+(\d+)u?\s*$",
            header, re.MULTILINE)
        self.assertIsNotNone(declared_version)
        self.assertIsNotNone(version)
        self.assertEqual(int(declared_version.group(1)), version)
        header = re.sub(r"/\*.*?\*/|//[^\n]*", "", header, flags=re.DOTALL)
        header = header.split('extern "C" {', 1)[1].rsplit("}", 1)[0]
        declarations = {}
        for match in re.finditer(r"\b(talos_\w+)\s*\(([^()]*)\)\s*;", header):
            name, parameters = match.groups()
            self.assertNotIn(name, declarations)
            parameters = parameters.strip()
            declarations[name] = 0 if parameters in ("", "void") else len(parameters.split(","))
        self.assertTrue(declarations)
        self.assertTrue(bindings)
        self.assertEqual(declarations.keys(), bindings.keys())
        for name, count in declarations.items():
            with self.subTest(function=name):
                self.assertEqual(count, bindings[name])


if __name__ == "__main__":
    unittest.main()
