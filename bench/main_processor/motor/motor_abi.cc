#include "motor/motor.h"

// The C++ node hosted in Python (talos.cpp_node), for node_py_test.py's
// parity case against motor.py.
TALOS_EXPORT_NODE(motor, motor::BenchMotor)
