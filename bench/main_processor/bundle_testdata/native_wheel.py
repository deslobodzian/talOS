"""NativeWheel: //bench/main_processor:bundle_test's Python node that
imports a third-party native wheel (numpy, from @pypi), so the installed
bundle carries a wheel's extension modules and the shared libraries the
wheel bundles. Not a robot node: it owns no device and publishes nothing.

numpy is imported with the module, so `--describe` (install's warm-up and
graph check) loads it as a start would.
"""

import numpy


class NativeWheel:
    def __init__(self, node):
        self.node = node
        self.ticks = numpy.zeros(1, dtype=numpy.int64)

    def update(self):
        self.ticks += 1
