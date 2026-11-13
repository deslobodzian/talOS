// bundle_test only: a native library whose load leaves a mark. Its
// constructor appends `native <its path> <what $STUB_PREFIX/current selects,
// or ->` to $STUB_LOG, as bundle_test.py's stub executables record a run.
#define _GNU_SOURCE  // dladdr and Dl_info, under glibc
#include <dlfcn.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

__attribute__((constructor)) static void Mark(void) {
  const char* log = getenv("STUB_LOG");
  const char* prefix = getenv("STUB_PREFIX");
  Dl_info info;
  if (log == NULL || prefix == NULL || dladdr((void*)&Mark, &info) == 0) return;
  char link[PATH_MAX];
  char selected[PATH_MAX] = "-";
  snprintf(link, sizeof link, "%s/current", prefix);
  const ssize_t length = readlink(link, selected, sizeof selected - 1);
  if (length > 0) selected[length] = '\0';
  char line[3 * PATH_MAX];
  const int size =
      snprintf(line, sizeof line, "native %s %s\n", info.dli_fname, selected);
  const int fd = open(log, O_WRONLY | O_APPEND | O_CREAT, 0644);
  if (fd < 0) return;
  if (size > 0) (void)!write(fd, line, (size_t)size);
  close(fd);
}
