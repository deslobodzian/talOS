"""Differential test: talos_schema.struct_hash (Python) against
talos::schema::struct_hash (C++) on generated schemas.

A seeded generator writes FlatBuffers schema files of structs and enums
(every scalar base type and its aliases, enums of every integer width with
gaps, negative, 64-bit and bit_flags values, nested structs, fixed arrays of
scalars, enums and structs, force_align, field and struct names that share
prefixes, and structs that reference an included file's types). flatc
compiles them to .bfbs as talos_messages does, descriptor_gen (the C++
struct_hash talos_messages bakes into Descriptor<M>::kHash) hashes every
struct, and Python must give the same hash for each.
"""

import os
import random
import re
import subprocess
import tempfile
import unittest

import talos_schema
from python.runfiles import runfiles

_RUNFILES = runfiles.Create()
_SEED = 20261123
_FILES = 40
_DESCRIPTOR_HASH_RE = re.compile(
    r'kName = "([^"]+)";\s*static constexpr std::uint64_t kHash = '
    r'(0x[0-9a-fA-F]+)ULL;')

# (spellings, size); the first spelling is the canonical one.
_SCALARS = (
    (("bool",), 1),
    (("byte", "int8"), 1),
    (("ubyte", "uint8"), 1),
    (("short", "int16"), 2),
    (("ushort", "uint16"), 2),
    (("int", "int32"), 4),
    (("uint", "uint32"), 4),
    (("long", "int64"), 8),
    (("ulong", "uint64"), 8),
    (("float", "float32"), 4),
    (("double", "float64"), 8),
)
_ENUM_TYPES = (
    ("byte", 1, True), ("ubyte", 1, False), ("short", 2, True),
    ("ushort", 2, False), ("int", 4, True), ("uint", 4, False),
    ("long", 8, True), ("ulong", 8, False),
)
# Names that share prefixes, so a canonical text that ran one name into the
# next (no separator) would collide.
_FIELD_STEMS = ("a", "ab", "ab_c", "abc", "x", "x_m", "x_mm", "speed",
                "speed_m", "speed_mps", "level", "level2", "mode", "modes")
_STRUCT_STEMS = ("S", "Sa", "Sab", "Pose", "Pose2", "PoseA", "Inner", "In")


class _File:
    def __init__(self, index, rng, previous):
        self.index = index
        self.namespace = f"gen.f{index}" if index % 3 else f"gen_f{index}"
        self.path = f"gen/f{index}.fbs"
        self.include = previous if previous and rng.random() < 0.5 else None
        # name -> (qualified name, size, align) of types usable in a field.
        self.enums = []  # (qualified, size)
        self.structs = []  # (qualified, size, align)
        if self.include is not None:
            self.enums.extend(self.include.enums)
            self.structs.extend(self.include.structs)
        self.own_structs = []  # qualified names declared here
        lines = []
        if self.include is not None:
            lines.append(f'include "{self.include.path}";')
        lines.append(f"namespace {self.namespace};")
        for e in range(rng.randint(0, 3)):
            lines.extend(self._enum(rng, e))
        stems = list(_STRUCT_STEMS)
        rng.shuffle(stems)
        for s in range(rng.randint(5, 10)):
            lines.extend(self._struct(rng, f"{stems[s % len(stems)]}{s}"))
        self.text = "\n".join(lines) + "\n"

    def _enum(self, rng, e):
        spelling, size, signed = rng.choice(_ENUM_TYPES)
        bits = size * 8
        lo, hi = (-(1 << (bits - 1)), (1 << (bits - 1)) - 1) if signed else (
            0, (1 << bits) - 1)
        name = f"Kind{e}" if e % 2 else f"K{e}"
        count = rng.randint(1, 6)
        if not signed and rng.random() < 0.25:
            positions = sorted(rng.sample(range(bits), min(count, bits)))
            values = ", ".join(f"F{p} = {p}" for p in positions)
            self.enums.append((f"{self.namespace}.{name}", size))
            return [f"enum {name} : {spelling} (bit_flags) {{ {values} }}"]
        values = sorted(rng.sample(range(max(lo, -1000), min(hi, 1000) + 1),
                                   count))
        if rng.random() < 0.3:
            values[-1] = hi  # the type's extreme, 2^64 - 1 for ulong
        if signed and rng.random() < 0.3:
            values[0] = lo
        # A struct field's enum must hold its default, 0.
        values = sorted(set(values) | {0})
        spelled = []
        for i, v in enumerate(values):
            # Implicit values (previous + 1) and explicit ones, mixed.
            implicit = i > 0 and values[i - 1] + 1 == v and rng.random() < 0.5
            spelled.append(f"V{i}" if implicit else f"V{i} = {v}")
        self.enums.append((f"{self.namespace}.{name}", size))
        return [f"enum {name} : {spelling} {{ {', '.join(spelled)} }}"]

    def _field_type(self, rng):
        """(spelling, size, align) of a non-array field type."""
        pick = rng.random()
        if pick < 0.2 and self.enums:
            qualified, size = rng.choice(self.enums)
            return self._spell(qualified), size, size
        if pick < 0.4 and self.structs:
            qualified, size, align = rng.choice(self.structs)
            return self._spell(qualified), size, align
        spellings, size = rng.choice(_SCALARS)
        return rng.choice(spellings), size, size

    def _spell(self, qualified):
        # Own types by leaf name, an included file's fully qualified.
        prefix = self.namespace + "."
        if qualified.startswith(prefix) and "." not in qualified[len(prefix):]:
            return qualified[len(prefix):]
        return qualified

    def _struct(self, rng, name):
        stems = list(_FIELD_STEMS)
        rng.shuffle(stems)
        fields = []
        offset = 0
        align = 1
        for f in range(rng.randint(1, 7)):
            spelling, size, field_align = self._field_type(rng)
            if spelling != "bool" and rng.random() < 0.2:
                length = rng.randint(1, 5)
                spelling = f"[{spelling}:{length}]"
                size *= length
            offset = (offset + field_align - 1) // field_align * field_align
            offset += size
            align = max(align, field_align)
            fields.append(f"  {stems[f]}:{spelling};")
        attribute = ""
        if rng.random() < 0.15:
            forced = rng.choice([a for a in (1, 2, 4, 8, 16) if a >= align])
            attribute = f" (force_align: {forced})"
            align = forced
        size = (offset + align - 1) // align * align
        qualified = f"{self.namespace}.{name}"
        self.structs.append((qualified, size, align))
        self.own_structs.append(qualified)
        return [f"struct {name}{attribute} {{", *fields, "}"]


def _generate(seed):
    rng = random.Random(seed)
    files = []
    previous = None
    for index in range(_FILES):
        previous = _File(index, rng, previous)
        files.append(previous)
    return files


def _run(args):
    result = subprocess.run(args, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise AssertionError(f"{args[0]} failed: {result.stderr}")


class StructHashDifferentialTest(unittest.TestCase):
    # REQ-CONTRACT-07
    def test_python_hash_equals_cpp_on_generated_schemas(self):
        flatc = _RUNFILES.Rlocation(os.environ["TALOS_FLATC"])
        gen = _RUNFILES.Rlocation(os.environ["TALOS_DESCRIPTOR_GEN"])
        files = _generate(_SEED)
        compared = 0
        with tempfile.TemporaryDirectory() as tmp:
            os.mkdir(os.path.join(tmp, "gen"))
            out = os.path.join(tmp, "out")
            for f in files:
                with open(os.path.join(tmp, f.path), "w",
                          encoding="utf-8") as fbs:
                    fbs.write(f.text)
            # As talos_messages compiles: declaration files relative to the
            # include root.
            _run([flatc, "--binary", "--schema", "--bfbs-filenames", tmp,
                  "-I", tmp, "-o", out,
                  *(os.path.join(tmp, f.path) for f in files)])
            for f in files:
                bfbs_path = os.path.join(out, f"f{f.index}.bfbs")
                header = os.path.join(tmp, f"f{f.index}.h")
                _run([gen, "cc", bfbs_path, "//" + f.path, header,
                      "//talOS/schema:differential"])
                with open(header, encoding="utf-8") as h:
                    cpp = {m.group(1): int(m.group(2), 16)
                           for m in _DESCRIPTOR_HASH_RE.finditer(h.read())}
                with open(bfbs_path, "rb") as b:
                    bfbs = b.read()
                # Every struct the file declares was hashed by C++, so a
                # declaration-file mismatch cannot silently compare nothing.
                self.assertEqual(sorted(cpp), sorted(f.own_structs), f.path)
                for name in f.own_structs:
                    python = talos_schema.struct_hash(bfbs, name)
                    context = (
                        f"seed {_SEED}, {f.path}, struct {name}\n"
                        f"C++ {cpp[name]:#018x}, Python "
                        f"{python if python is None else hex(python)}\n"
                        f"Python canonical: "
                        f"{talos_schema.canonical_struct(bfbs, name)}\n"
                        f"--- {f.path}\n{f.text}"
                        + (f"--- {f.include.path}\n{f.include.text}"
                           if f.include else ""))
                    self.assertNotEqual(cpp[name], 0, context)
                    self.assertEqual(python, cpp[name], context)
                    compared += 1
        # The generator's reach, so a change that shrinks it is seen.
        self.assertGreater(compared, 250)


if __name__ == "__main__":
    unittest.main()
