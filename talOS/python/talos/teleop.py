"""The framework teleop node, shared with C++ robots."""

import talos

Teleop = talos.cpp_node("talOS/teleop/libteleop.so", "teleop")
