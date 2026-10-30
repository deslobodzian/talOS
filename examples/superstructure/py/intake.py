"""Roller + beam break: the C++ demo::Intake node wrapped for Python."""

import talos

Intake = talos.cpp_node("examples/superstructure/cpp/libmechanisms.so", "intake")
