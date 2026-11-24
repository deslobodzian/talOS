"""talos::ShmObjectName (talOS/memory/shared_memory_ptr.h) in a namespace,
for Python tools and tests that open or unlink a session's segments.

Held to the C++ by shm_name_test.
"""


def shm_name(topic, namespace):
    """The OS name of `topic`'s segment under TALOS_SHM_NAMESPACE (or a
    test's TEST_TMPDIR) `namespace`: unchanged when already a `/~...` hashed
    name; otherwise the topic with leading slashes collapsed to one and
    interior slashes turned to dots, and when `namespace` is non-empty,
    "/~" and 16 lowercase hex digits of FNV-1a-64 over the namespace, a NUL,
    and that mapped name."""
    if topic.startswith("/~"):
        return topic
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    if not namespace:
        return mapped
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"
