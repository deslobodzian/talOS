"""Shared Bazel unittest sharding loader for tools/ Python test suites."""

import os
import unittest


class ShardedTestLoader(unittest.TestLoader):
    """Partitions test methods deterministically across Bazel test shards."""

    def __init__(self, shard_index, total_shards):
        super().__init__()
        self._shard_index = shard_index
        self._total_shards = total_shards

    def getTestCaseNames(self, testCaseClass):
        names = super().getTestCaseNames(testCaseClass)
        return [
            name
            for i, name in enumerate(names)
            if i % self._total_shards == self._shard_index
        ]


def bazel_test_loader():
    status_file = os.environ.get("TEST_SHARD_STATUS_FILE")
    if status_file:
        with open(status_file, "a", encoding="utf-8"):
            pass
    total = int(os.environ.get("TEST_TOTAL_SHARDS") or "1")
    index = int(os.environ.get("TEST_SHARD_INDEX") or "0")
    if total <= 1:
        return unittest.defaultTestLoader
    if not 0 <= index < total:
        raise ValueError(
            f"TEST_SHARD_INDEX={index} out of range for "
            f"TEST_TOTAL_SHARDS={total}"
        )
    return ShardedTestLoader(index, total)
