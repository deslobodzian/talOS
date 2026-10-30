"""One motion-magic motor: the C++ demo::Elevator node wrapped for Python."""

import talos

Elevator = talos.cpp_node(
    "examples/superstructure/cpp/libmechanisms.so", "elevator")
