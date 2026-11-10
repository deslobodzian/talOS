"""talos.cpp_node hosts an exported C++ node on the one framework in the
process: the library's node and devices calls run in the libtalos_node image
Python drives, not in a private copy of their own."""

import ctypes
import ctypes.util
import os
import shutil
import subprocess
import sys
import unittest

from python.runfiles import runfiles
import talos
from talos import _abi

# Set in the child process test_production_load_order starts.
_CHILD = "TALOS_CPP_NODE_TEST_CHILD"

# Framework functions the probe library calls, each returning its address as
# resolved from the library. talos_probe_hardware_function names one node_api
# never calls.
_PROBE_FUNCTIONS = ("talos_probe_node_function", "talos_probe_devices_function",
                    "talos_probe_hardware_function")


class _DlInfo(ctypes.Structure):
    _fields_ = [("dli_fname", ctypes.c_char_p), ("dli_fbase", ctypes.c_void_p),
                ("dli_sname", ctypes.c_char_p), ("dli_saddr", ctypes.c_void_p)]


def _dladdr():
    for library in (None, ctypes.util.find_library("dl")):
        try:
            function = ctypes.CDLL(library).dladdr
        except (AttributeError, OSError, TypeError):
            continue
        function.argtypes = [ctypes.c_void_p, ctypes.POINTER(_DlInfo)]
        function.restype = ctypes.c_int
        return function
    raise unittest.SkipTest("no dladdr")


def _image(address):
    """The base address and path of the loaded image holding address."""
    info = _DlInfo()
    if not _dladdr()(address, ctypes.byref(info)):
        raise AssertionError(f"dladdr found no image for {address:#x}")
    return info.dli_fbase, info.dli_fname.decode()


def _address(function):
    return ctypes.cast(function, ctypes.c_void_p).value


def _foreign_images(probe_path):
    """Hosts the probe, then returns each framework function it calls that
    resolves outside the image Python's node_api calls run in, with that
    image's path; empty when the process has one framework."""
    talos.cpp_node(probe_path, "probe")
    framework = _image(_address(_abi.lib.talos_abi_version))
    library = ctypes.CDLL(probe_path)
    foreign = []
    for symbol in _PROBE_FUNCTIONS:
        function = getattr(library, symbol)
        function.restype = ctypes.c_void_p
        image = _image(function())
        if image[0] != framework[0]:
            foreign.append(f"{symbol} is in {image[1]}, not {framework[1]}")
    return framework, foreign


def _child():
    """Runs with TALOS_NODE_LIB naming a copy of libtalos_node.so: exits 0
    when the probe runs on that copy."""
    framework, foreign = _foreign_images(os.environ["TALOS_PROBE_PATH"])
    expected = os.path.realpath(os.environ["TALOS_NODE_LIB"])
    if os.path.realpath(framework[1]) != expected:
        foreign.append(f"node_api is in {framework[1]}, not {expected}")
    for line in foreign:
        print(line, file=sys.stderr)
    return 1 if foreign else 0


class CppNodeTest(unittest.TestCase):

    def test_hosted_library_runs_on_the_host_framework(self):
        # A library linking its own copy of talOS node and devices runs those
        # copies against libtalos_node's Node: two sets of statics and of
        # extension type identities in one node.
        files = runfiles.Create()
        path = files.Rlocation(os.environ["TALOS_PROBE_LIB"])
        framework, foreign = _foreign_images(path)
        self.assertEqual(foreign, [])

        # talos.run loads libtalos_node.so from the runfiles, and a library
        # its node hosts finds that file through its own search path: one
        # image.
        production = ctypes.CDLL(
            files.Rlocation(os.environ["TALOS_FRAMEWORK_LIB"]))
        self.assertEqual(_image(_address(production.talos_abi_version))[0],
                         framework[0])

    def test_production_load_order(self):
        # A deployed Python node loads libtalos_node.so first and by path, from
        # a file other than the one the hosted library's search path reaches
        # (talos_bundle copies the runfiles, so the runfiles and _solib paths
        # are two files). The loader must still bind the library's
        # libtalos_node.so to the loaded image: by soname on Linux, by install
        # name on macOS. Without that, the library loads a second framework.
        files = runfiles.Create()
        copy = os.path.join(os.environ["TEST_TMPDIR"], "lib", "libtalos_node.so")
        os.makedirs(os.path.dirname(copy), exist_ok=True)
        shutil.copyfile(files.Rlocation(os.environ["TALOS_FRAMEWORK_LIB"]),
                        copy)
        env = dict(os.environ)
        env.update({
            _CHILD: "1",
            "TALOS_NODE_LIB": copy,
            "TALOS_PROBE_PATH": files.Rlocation(os.environ["TALOS_PROBE_LIB"]),
            "PYTHONPATH": os.pathsep.join(sys.path),
        })
        # The import of talos loads the framework, so the order needs a fresh
        # process.
        child = subprocess.run([sys.executable, __file__], env=env,
                               capture_output=True, text=True, check=False)
        self.assertEqual(child.returncode, 0, child.stdout + child.stderr)


if __name__ == "__main__":
    if os.environ.get(_CHILD):
        sys.exit(_child())
    unittest.main()
