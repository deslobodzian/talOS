"""Unit tests for tools/check_links.py on in-memory files."""

import io
import unittest

from tools import check_links

GUIDE = """# Guide

## 1. What a test is for
## Commit log (the project's reasoning record)
## `Target::get()` and `kAtDrain`
## Notes
## Notes
<a id="custom-anchor"></a>

```sh
[not a link](missing.md)
```
"""


def check(files):
    out = io.StringIO()
    broken = check_links.check(sorted(files), files.get, out)
    return broken, out.getvalue()


class CheckLinksTest(unittest.TestCase):
    def test_resolving_links_pass(self):
        # GitHub's anchor rules: punctuation dropped, spaces to hyphens,
        # code spans unwrapped, a repeated heading numbered.
        doc = ("[a](GUIDE.md#1-what-a-test-is-for) "
               "[b](GUIDE.md#commit-log-the-projects-reasoning-record) "
               "[c](GUIDE.md#targetget-and-katdrain) [d](GUIDE.md#notes-1) "
               "[e](GUIDE.md#custom-anchor) [f](talOS/) [g](#readme) "
               "[h](https://example.com/x.md#nowhere) `[i](missing.md)`\n"
               "[ref]: talOS/node/node.h\n# Readme\n")
        broken, out = check({"README.md": doc, "GUIDE.md": GUIDE,
                             "talOS/node/node.h": ""})
        self.assertEqual(broken, 0, out)

    def test_broken_link_or_anchor_fails_naming_it(self):
        for target in ("missing.md", "GUIDE.md#no-such-heading",
                       "GUIDE.md#notes-2", "#nowhere", "../outside.md",
                       "docs/"):
            with self.subTest(target):
                broken, out = check({"README.md": f"x\n[link]({target})\n",
                                     "GUIDE.md": GUIDE})
                self.assertEqual(broken, 1, out)
                self.assertIn(f"README.md:2: {target}:", out)

    def test_inline_triple_backticks_fenced_anchors_and_title_parens(self):
        # An inline triple-backtick code span at line start must not open an
        # unclosed fenced code block that hides every later link.
        doc = "```code```\n[broken](missing.md)\n"
        broken, out = check({"README.md": doc})
        self.assertEqual(broken, 1, out)
        self.assertIn("README.md:2: missing.md:", out)

        # A 3-backtick line inside a 4-backtick fence must not close it, and a
        # closing fence with trailing text must not close a 3-backtick fence.
        nested = (
            "````markdown\n"
            "```\n"
            "[inside-fence](missing1.md)\n"
            "````\n"
            "[after-fence](missing2.md)\n"
        )
        broken, out = check({"README.md": nested})
        self.assertEqual(broken, 1, out)
        self.assertIn("missing2.md", out)
        self.assertNotIn("missing1.md", out)

        # An HTML <a id="..."> inside a fenced code block is not a real anchor,
        # while a mixed-case real <a id="MixedCase"> outside code resolves.
        guide = (
            '```html\n<a id="fenced-only"></a>\n```\n'
            '<a id="MixedCase"></a>\n'
        )
        broken, out = check({
            "README.md": "[bad](GUIDE.md#fenced-only) [ok](GUIDE.md#MixedCase)\n",
            "GUIDE.md": guide,
        })
        self.assertEqual(broken, 1, out)
        self.assertIn("GUIDE.md#fenced-only", out)
        self.assertNotIn("MixedCase", out)

        # A broken link whose quoted title contains parentheses is still caught.
        broken, out = check({
            "README.md": '[x](missing.md "see (details)")\n',
        })
        self.assertEqual(broken, 1, out)
        self.assertIn("missing.md", out)


if __name__ == "__main__":
    unittest.main()
