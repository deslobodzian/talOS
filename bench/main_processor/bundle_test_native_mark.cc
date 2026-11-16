// warm_up_test only: a native library whose load leaves a mark. Its
// constructor appends `native <its path> <what $STUB_PREFIX/current selects,
// or ->` to $STUB_LOG, as warm_up_test.py's stub executables record a run.
// It then writes two lines to its process's stdout, as a library that
// prints when loaded does, and with $STUB_NATIVE_KILL set kills the loading
// process.
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

#include <climits>
#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace {

__attribute__((constructor)) void Mark() {
  const char* log = std::getenv("STUB_LOG");
  const char* prefix = std::getenv("STUB_PREFIX");
  Dl_info info;
  if (log == nullptr || prefix == nullptr ||
      dladdr(reinterpret_cast<void*>(&Mark), &info) == 0) {
    return;
  }
  char link[PATH_MAX];
  char selected[PATH_MAX] = "-";
  std::snprintf(link, sizeof link, "%s/current", prefix);
  const ssize_t length = readlink(link, selected, sizeof selected - 1);
  if (length > 0) selected[length] = '\0';
  char line[3 * PATH_MAX];
  const int size = std::snprintf(line, sizeof line, "native %s %s\n",
                                 info.dli_fname, selected);
  const int fd = open(log, O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC, 0644);
  if (fd < 0) return;
  if (size > 0 && write(fd, line, static_cast<size_t>(size)) < 0) {
    // Nothing to report to: the test then finds no mark.
  }
  close(fd);
  constexpr char kPrinted[] = "loaded\nloaded\n";
  if (write(STDOUT_FILENO, kPrinted, sizeof kPrinted - 1) < 0) {
    // Nobody reads it: the count must not either.
  }
  if (std::getenv("STUB_NATIVE_KILL") != nullptr) std::raise(SIGKILL);
}

}  // namespace
