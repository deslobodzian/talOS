"""Sequences the intake and the elevator: the C++ demo::Superstructure node
wrapped for Python."""

import talos

Superstructure = talos.cpp_node(
    "examples/superstructure/cpp/libmechanisms.so", "superstructure")
