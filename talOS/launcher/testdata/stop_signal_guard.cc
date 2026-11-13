// Linked into every launcher test that runs a session in its own process
// (alwayslink): before main runs, SIGINT, SIGTERM and SIGHUP get a handler
// that does nothing.
//
// A roster Launcher::Run takes those signals for its orderly stop and puts
// back the dispositions it found when it returns. A test that signals its
// own process to stop a session (RunSessionUntil's SIGTERM, a stopper
// thread's kill after a wait) may, on a failing path, signal after Run has
// returned; with the default action that would end the whole test binary,
// hiding the failure's messages and skipping the rest of the shard. With
// this handler the late signal does nothing. A launcher forked from the test
// inherits the handler and installs its own over it, as it would over any;
// exec, which a spawned member or probe goes through, resets it to the
// default.

#include <signal.h>

#include <initializer_list>

namespace {

void Ignore(int /*signal*/) {}

bool InstallDoNothingHandlers() noexcept {
  struct sigaction action {};
  action.sa_handler = Ignore;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  bool installed = true;
  for (const int signal : {SIGINT, SIGTERM, SIGHUP})
    installed = ::sigaction(signal, &action, nullptr) == 0 && installed;
  return installed;
}

// At static initialization, so before gtest runs any test.
[[maybe_unused]] const bool kStopSignalGuard = InstallDoNothingHandlers();

}  // namespace
