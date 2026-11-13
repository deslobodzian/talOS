#!/usr/bin/env python3
"""Fail on a C/C++ compile that read a header from the host's include tree.

Bazel's autodetected toolchain lists /usr/include (and /usr/local/include)
among its built-in include directories, so an action that resolves
<toml++/toml.hpp> from a host package passes Bazel's undeclared-inclusion
check on a workstation that has the package and fails to compile where it is
absent (CI; REQ-DEV-25). Every compile writes the headers it read to its
`.d` file; this reads those files and reports each header under a host
include prefix that is not the C library's, the kernel's or the C++ standard
library's, naming the source file that read it.

  python3 tools/check_host_includes.py [--allow PATH]... DIR_OR_FILE...

Each argument is a `.d` file, or a directory searched for `*.d` files
(`bazel info output_path` after a build with
--experimental_inmemory_dotd_files=false and
--remote_download_regex='.*\\.d$': without them a compile that hit a cache
leaves no `.d` file on disk). Paths outside the host prefixes
(the compiler's own directories, a macOS SDK, Bazel's external/ and
bazel-out/) are never reported. --allow adds a header path, or a directory
ending in '/', to the allowlist.

Exit: 0 clean, 1 host headers found, 2 usage error or no `.d` file found.
"""

import argparse
import os
import sys

# Include trees a compiler searches by default that host packages install
# into. A macOS SDK's usr/include is under /Library or /Applications, so it
# is not one of these.
HOST_PREFIXES = (
    "/usr/include/",
    "/usr/local/include/",
    "/opt/homebrew/include/",
    "/opt/local/include/",
)

# Directories under a host prefix that belong to the C library, the kernel
# UAPI or the C++ standard library (Debian's multiarch <triple>/ prefix is
# stripped first).
SYSTEM_DIRS = frozenset({
    # glibc
    "arpa", "bits", "gnu", "net", "netash", "netatalk", "netax25",
    "neteconet", "netinet", "netipx", "netiucv", "netpacket", "netrom",
    "netrose", "nfs", "protocols", "rpc", "scsi", "sys",
    # Linux UAPI (linux-api-headers, linux-libc-dev)
    "asm", "asm-generic", "drm", "linux", "misc", "mtd", "rdma", "sound",
    "video", "xen",
    # libstdc++ / libc++
    "c++",
})

# glibc's top-level headers.
SYSTEM_HEADERS = frozenset("""
aio.h aliases.h alloca.h a.out.h ar.h argp.h argz.h assert.h byteswap.h
complex.h cpio.h crypt.h ctype.h dirent.h dlfcn.h elf.h endian.h envz.h err.h
errno.h error.h execinfo.h fcntl.h features.h features-time64.h fenv.h
fmtmsg.h fnmatch.h fpu_control.h fstab.h fts.h ftw.h gconv.h getopt.h glob.h
gnu-versions.h grp.h gshadow.h iconv.h ieee754.h ifaddrs.h inttypes.h
langinfo.h lastlog.h libgen.h libintl.h limits.h link.h locale.h malloc.h
math.h mcheck.h memory.h mntent.h monetary.h mqueue.h netdb.h nl_types.h
nss.h obstack.h paths.h poll.h printf.h proc_service.h pthread.h pty.h pwd.h
re_comp.h regex.h regexp.h resolv.h sched.h search.h semaphore.h setjmp.h
sgtty.h shadow.h signal.h spawn.h stab.h stdbit.h stdc-predef.h stdint.h
stdio.h stdio_ext.h stdlib.h string.h strings.h syscall.h sysexits.h syslog.h
tar.h termio.h termios.h tgmath.h thread_db.h threads.h time.h ttyent.h
uchar.h ucontext.h ulimit.h unistd.h utime.h utmp.h utmpx.h values.h wait.h
wchar.h wctype.h wordexp.h
""".split())


def parse_depfile(text):
    """Returns the rules of a Make-style dependency file as lists of their
    prerequisites, the compiled source first; a phony rule (-MP) has none."""
    rules = []
    for line in text.replace("\\\n", " ").splitlines():
        _, sep, rest = line.partition(": ")
        if sep:
            # A '\ ' is an escaped space inside a path.
            tokens = rest.replace("\\ ", "\0").split()
            rules.append([t.replace("\0", " ") for t in tokens])
    return rules


def is_multiarch(segment):
    return segment.endswith(("-linux-gnu", "-linux-gnueabihf", "-linux-musl"))


def host_violation(path, allow):
    """The normalized host path if `path` is a forbidden host header."""
    if not path.startswith("/"):
        return None
    norm = os.path.normpath(path)
    prefix = next((p for p in HOST_PREFIXES if norm.startswith(p)), None)
    if prefix is None:
        return None
    if any(norm == a or (a.endswith("/") and norm.startswith(a)) for a in allow):
        return None
    parts = norm[len(prefix):].split("/")
    if prefix == "/usr/include/":
        if len(parts) > 1 and is_multiarch(parts[0]):
            parts = parts[1:]
        if len(parts) > 1 and parts[0] in SYSTEM_DIRS:
            return None
        if len(parts) == 1 and parts[0] in SYSTEM_HEADERS:
            return None
    return norm


def depfiles(paths):
    for path in paths:
        if os.path.isdir(path):
            for root, _, files in os.walk(path, followlinks=False):
                for name in sorted(files):
                    if name.endswith(".d"):
                        yield os.path.join(root, name)
        else:
            yield path


def check(paths, allow):
    """Returns ([(source, header)], depfile count)."""
    found = []
    count = 0
    for depfile in depfiles(paths):
        try:
            with open(depfile, encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            continue
        rules = parse_depfile(text)
        if not rules:
            continue
        count += 1
        for prerequisites in rules:
            for header in prerequisites:
                bad = host_violation(header, allow)
                if bad is not None:
                    found.append((prerequisites[0], bad))
    return sorted(set(found)), count


def main(argv):
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--allow", action="append", default=[])
    args = parser.parse_args(argv)
    found, count = check(args.paths, args.allow)
    if count == 0:
        # Not a pass: nothing was checked.
        print("check_host_includes: no .d files under "
              + " ".join(args.paths) + ": the build compiled nothing, or "
              "its compiles hit a cache and Bazel kept their .d files in "
              "memory; build with --experimental_inmemory_dotd_files=false "
              "'--remote_download_regex=.*\\.d$' (tools/prepush.sh does)",
              file=sys.stderr)
        return 2
    if found:
        print("check_host_includes: compiled against host headers no "
              "declared dependency provides:")
        by_source = {}
        for source, header in found:
            by_source.setdefault(source, []).append(header)
        for source, headers in by_source.items():
            # The directly included header sorts first among its own tree's.
            shown = sorted(headers, key=lambda h: (h.count("/"), h))[:3]
            more = len(headers) - len(shown)
            for header in shown:
                print(f"  {source}: {header}")
            if more:
                print(f"  {source}: ... and {more} more")
        return 1
    print(f"check_host_includes: {count} .d files, no host headers")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
