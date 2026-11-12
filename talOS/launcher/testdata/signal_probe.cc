// Prints the signals this process started with ignored, then those it
// started with blocked, as `ignored: <n> ...` and `blocked: <n> ...` lines,
// for closed_output_test's check of what SpawnProcess passes on.

#include <signal.h>

#include <cstdio>

int main() {
  sigset_t mask;
  sigemptyset(&mask);
  ::sigprocmask(SIG_SETMASK, nullptr, &mask);
  std::printf("ignored:");
  for (int signal = 1; signal < NSIG; ++signal) {
    struct sigaction action {};
    if (::sigaction(signal, nullptr, &action) == 0 &&
        action.sa_handler == SIG_IGN)
      std::printf(" %d", signal);
  }
  std::printf("\nblocked:");
  for (int signal = 1; signal < NSIG; ++signal) {
    if (sigismember(&mask, signal) == 1) std::printf(" %d", signal);
  }
  std::printf("\n");
  return 0;
}
