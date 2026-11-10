"""One motion-magic motor homed on a bottom switch: the C++ demo::Elevator
node wrapped for Python."""

import talos

Elevator = talos.cpp_node(
    "examples/superstructure/cpp/libmechanisms.so", "elevator")
