"""talos::ShmObjectName (talOS/memory/shared_memory_ptr.h) in a namespace,
for Python tools and tests that open or unlink a session's segments.

Held to the C++ by shm_name_test.
"""


def shm_name(topic, namespace):
    """The OS name of `topic`'s segment under TALOS_SHM_NAMESPACE (or a
    test's TEST_TMPDIR) `namespace`: "/~" and 16 lowercase hex digits of
    FNV-1a-64 over the namespace, a NUL, and the topic with its leading
    slashes collapsed to one and the others turned to dots."""
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"
