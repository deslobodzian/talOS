"""NativeWheel: //bench/main_processor:bundle_test's Python node that
imports third-party native wheels from @pypi, so the installed bundle
carries wheels' extension modules and the shared libraries they bundle or
depend on: numpy; gtsam; and robotpy-wpimath with its robotpy-native
wpimath and wpiutil libraries. talOS pins gtsam and robotpy-wpimath for
robot repositories and no other talOS target loads them, so this is where a
toolchain or lock change that leaves either without a loadable wheel fails.
Not a robot node: it owns no device and publishes nothing.

The wheels are imported, and each native module called once, with the
module, so `--describe` (install's warm-up and graph check) loads their
shared libraries as a start would.
"""

import gtsam
import numpy
import wpimath.geometry

# One call into each native module: an import alone may leave a lazily
# loaded library unopened.
HEADING_RAD = (gtsam.Pose2(1.0, 2.0, 0.5).theta()
               + wpimath.geometry.Rotation2d(0.5).radians())


class NativeWheel:
    def __init__(self, node):
        self.node = node
        self.ticks = numpy.zeros(1, dtype=numpy.int64)

    def update(self):
        self.ticks += 1
