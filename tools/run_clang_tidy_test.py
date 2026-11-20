"""Unit tests for tools/run_clang_tidy.py header includer selection."""

import unittest

from tools import run_clang_tidy


class SelectIncludersTest(unittest.TestCase):
    def test_production_header_prefers_production_includers(self):
        includers = {
            "talOS/bridge/foo.h": {
                "talOS/bridge/foo_test.cc",
                "talOS/bridge/foo.cc",
                "talOS/launcher/launcher.cc",
            },
        }
        tus = {
            "talOS/bridge/foo_test.cc",
            "talOS/bridge/foo.cc",
            "talOS/launcher/launcher.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/foo.h", includers, tus
        )
        self.assertEqual(
            found, ["talOS/bridge/foo.cc", "talOS/launcher/launcher.cc"]
        )
        self.assertFalse(fallback)

    def test_production_header_with_only_test_includers_falls_back(self):
        includers = {
            "talOS/bridge/in_process_gateway.h": {
                "talOS/bridge/node_test_support.h",
                "talOS/bridge/in_process_gateway_test.cc",
            },
            "talOS/bridge/node_test_support.h": {
                "talOS/bridge/node_test.cc",
                "talOS/other/integration_test.cc",
            },
        }
        tus = {
            "talOS/bridge/in_process_gateway_test.cc",
            "talOS/bridge/node_test.cc",
            "talOS/other/integration_test.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/in_process_gateway.h", includers, tus
        )
        self.assertEqual(
            found,
            [
                "talOS/bridge/in_process_gateway_test.cc",
                "talOS/bridge/node_test.cc",
                "talOS/other/integration_test.cc",
            ],
        )
        self.assertTrue(fallback)

    def test_production_header_with_transitive_production_includer_skips_test_fallback(self):
        includers = {
            "talOS/bridge/detail.h": {
                "talOS/bridge/detail_test.cc",
                "talOS/bridge/api.h",
            },
            "talOS/bridge/api.h": {
                "talOS/bridge/node.cc",
            },
        }
        tus = {
            "talOS/bridge/detail_test.cc",
            "talOS/bridge/node.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/detail.h", includers, tus
        )
        self.assertEqual(found, ["talOS/bridge/node.cc"])
        self.assertFalse(fallback)

    def test_test_header_selects_test_includers_without_fallback_flag(self):
        includers = {
            "talOS/bridge/testing.h": {
                "talOS/bridge/node_test.cc",
            },
        }
        tus = {"talOS/bridge/node_test.cc"}
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/testing.h", includers, tus
        )
        self.assertEqual(found, ["talOS/bridge/node_test.cc"])
        self.assertFalse(fallback)

    def test_unincluded_header_returns_empty(self):
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/orphan.h", {}, {"talOS/bridge/node.cc"}
        )
        self.assertEqual(found, [])
        self.assertFalse(fallback)

    def test_fallback_respects_tus_per_header_limit(self):
        tests = {f"talOS/bridge/t{i}_test.cc" for i in range(6)}
        includers = {"talOS/bridge/in_process_gateway.h": tests}
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/in_process_gateway.h", includers, tests, limit=4
        )
        self.assertEqual(len(found), 4)
        self.assertTrue(fallback)


if __name__ == "__main__":
    unittest.main()
