"""Sample standalone Python node consuming the talOS SDK."""
from talos_runtime import node_api


def inspect_runtime():
    """Verify the native node ABI can be loaded and bound."""
    lib_path = node_api.find_lib()
    if not lib_path:
        raise RuntimeError("Native runtime libtalos_node.so not found")
    version = node_api.abi_version()
    if version < node_api.TALOS_NODE_ABI_VERSION:
        raise RuntimeError(f"ABI version mismatch: got {version}, want {node_api.TALOS_NODE_ABI_VERSION}")
    return lib_path, version
