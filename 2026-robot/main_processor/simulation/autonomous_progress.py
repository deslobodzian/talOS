"""Removes the autonomous node's progress record after a namespaced sim run.

The record is a shared-memory object that lasts until reboot so a match
survives a launcher restart (2026-robot/README.md); the launcher unlinks only
declared topics. A harness that runs a session under its own
TALOS_SHM_NAMESPACE owns that namespace, so it removes the record when done.
"""

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses

# autonomous/progress.h kProgressObject.
PROGRESS_OBJECT = "/autonomous/progress"


def shm_name(name, namespace):
    """talos::ShmObjectName (talOS/memory/shared_memory_ptr.h) in a namespace."""
    mapped = "/" + name.lstrip("/").replace("/", ".")
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"


def remove(namespace):
    """Unlinks the record in `namespace`, if the run left one."""
    try:
        shm_unlink(shm_name(PROGRESS_OBJECT, namespace))
    except FileNotFoundError:
        pass
