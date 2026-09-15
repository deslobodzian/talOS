"""Intake node entry point. Mirrors the talOS/process/node_main.h flag set.

--describe is import-light on purpose (stdlib + json + sizes only): it must
run from the launcher probe without touching shared memory, hardware, or
the network, exactly like the C++ describe branch (node_main.h: describe
builds the node on a simulated loop and prints its manifest). In particular
--describe never imports node.py (which pulls in rtms + flatbuffers); the
source table below mirrors node.describe_sources() and node_test.py asserts
the two agree.
"""

import sys

from talos_nodes.intake import packet
from talos_runtime import node_api, runner


def _default_config():
    return runner.resolve_config_path("talos_nodes.intake:subsystem.toml")


def describe_source_rows():
    """Import-light source table. One line over the single copy in packet.py
    (rule 11) -- the same rows node.describe_sources() reports."""
    return packet.describe_sources(node_api.TIMER, node_api.WATCHER,
                                   node_api.SENDER)


def run_describe():
    print(node_api.describe_json(packet.NODE_NAME, packet.NODE_TARGET,
                                 describe_source_rows()))
    return 0


def main(argv=None):
    return runner.run_node_main(
        node_name=packet.NODE_NAME,
        node_target=packet.NODE_TARGET,
        entry_point="talos_nodes.intake.node:IntakeNode",
        describe_sources_fn=describe_source_rows,
        default_config=_default_config(),
        argv=argv,
    )


if __name__ == "__main__":
    sys.exit(main())
