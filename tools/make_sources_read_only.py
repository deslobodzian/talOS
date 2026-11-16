#!/usr/bin/env python3
"""Clear the write bits of every tracked regular file in the current checkout.

CI's "Make tracked source files read-only" step and tools/prepush.sh both run
this, from the checkout's root, before the tests; the reason is the step's
comment in .github/workflows/continuous-integration.yaml. Directories stay
writable (Bazel's convenience symlinks), symlinks and submodules are left
alone, and MODULE.bazel.lock stays writable.

  python3 tools/make_sources_read_only.py
"""

import os
import subprocess
import sys


def main():
    listing = subprocess.run(["git", "ls-files", "-s", "-z"],
                             capture_output=True, check=True).stdout
    for entry in filter(None, listing.split(b"\0")):
        meta, path = entry.split(b"\t", 1)
        if meta.startswith((b"100644", b"100755")) and path != b"MODULE.bazel.lock":
            os.chmod(path, os.stat(path).st_mode & ~0o222)
    return 0


if __name__ == "__main__":
    sys.exit(main())
