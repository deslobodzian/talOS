#include "talOS/launcher/sleep_guard.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

#ifdef __APPLE__
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOReturn.h>
#include <IOKit/pwr_mgt/IOPMLib.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <sstream>
#endif

namespace talos::launcher {
namespace {

#ifdef __APPLE__
// CoreFoundation and IOKit power-management assertions cannot be called in a
// process forked without exec() after they have been initialized in the parent
// (macOS aborts or hangs in CoreFoundation/libxpc). Track the process that
// first touched IOKit plus a pthread_atfork child flag so forked children
// (such as in-process test forks) skip IOKit cleanly, while exec'd launcher
// binaries start fresh and always acquire their assertion.
std::atomic<bool> g_forked_after_iokit{false};
std::atomic<pid_t> g_iokit_owner_pid{0};
std::once_flag g_atfork_once;

void EnsureForkGuardRegistered() {
  std::call_once(g_atfork_once, [] {
    g_iokit_owner_pid.store(::getpid(), std::memory_order_relaxed);
    (void)::pthread_atfork(nullptr, nullptr, [] {
      g_forked_after_iokit.store(true, std::memory_order_relaxed);
    });
  });
}

bool CanUseIoKitInThisProcess() noexcept {
  if (g_forked_after_iokit.load(std::memory_order_relaxed)) return false;
  const pid_t owner = g_iokit_owner_pid.load(std::memory_order_relaxed);
  return owner == 0 || owner == ::getpid();
}

bool CreateMacOsAssertion(std::string_view reason, std::uint32_t& assertion_id,
                          bool& active, std::string& error, void* /*context*/) {
  static_assert(sizeof(IOPMAssertionID) == sizeof(std::uint32_t));
  if (!CanUseIoKitInThisProcess()) {
    assertion_id = 0;
    active = false;
    return true;
  }
  EnsureForkGuardRegistered();
  CFStringRef owned_name = nullptr;
  CFStringRef name = CFSTR("talOS robot session");
  if (!reason.empty()) {
    owned_name = CFStringCreateWithBytes(
        kCFAllocatorDefault, reinterpret_cast<const UInt8*>(reason.data()),
        static_cast<CFIndex>(reason.size()), kCFStringEncodingUTF8,
        /*isExternalRepresentation=*/false);
    if (owned_name != nullptr) name = owned_name;
  }
  IOPMAssertionID id = kIOPMNullAssertionID;
  const IOReturn kr = IOPMAssertionCreateWithName(
      kIOPMAssertionTypePreventSystemSleep, kIOPMAssertionLevelOn, name, &id);
  if (owned_name != nullptr) CFRelease(owned_name);
  if (kr != kIOReturnSuccess) {
    std::ostringstream msg;
    msg << "IOPMAssertionCreateWithName(PreventSystemSleep) failed (IOReturn "
           "0x"
        << std::hex << static_cast<unsigned int>(kr) << ")";
    error = msg.str();
    active = false;
    assertion_id = 0;
    return false;
  }
  assertion_id = static_cast<std::uint32_t>(id);
  active = true;
  return true;
}

void ReleaseMacOsAssertion(std::uint32_t assertion_id,
                           void* /*context*/) noexcept {
  if (!CanUseIoKitInThisProcess()) return;
  (void)IOPMAssertionRelease(static_cast<IOPMAssertionID>(assertion_id));
}

constexpr SleepGuard::Ops kPlatformOps{
    .create = &CreateMacOsAssertion,
    .release = &ReleaseMacOsAssertion,
    .context = nullptr,
};
#else
bool CreateNoopAssertion(std::string_view /*reason*/,
                         std::uint32_t& assertion_id, bool& active,
                         std::string& /*error*/, void* /*context*/) {
  assertion_id = 0;
  active = false;
  return true;
}

constexpr SleepGuard::Ops kPlatformOps{
    .create = &CreateNoopAssertion,
    .release = nullptr,
    .context = nullptr,
};
#endif

}  // namespace

SleepGuard::SleepGuard(SleepGuard&& other) noexcept
    : active_{std::exchange(other.active_, false)},
      assertion_id_{std::exchange(other.assertion_id_, 0)},
      ops_{std::exchange(other.ops_, Ops{})} {}

SleepGuard& SleepGuard::operator=(SleepGuard&& other) noexcept {
  if (this != &other) {
    Release();
    active_ = std::exchange(other.active_, false);
    assertion_id_ = std::exchange(other.assertion_id_, 0);
    ops_ = std::exchange(other.ops_, Ops{});
  }
  return *this;
}

SleepGuard SleepGuard::Acquire(std::string_view reason, std::string* error) {
  return AcquireWithOps(reason, kPlatformOps, error);
}

SleepGuard SleepGuard::AcquireWithOps(std::string_view reason, Ops ops,
                                      std::string* error) {
  if (error != nullptr) error->clear();
  if (ops.create == nullptr) {
    if (error != nullptr) *error = "sleep assertion backend has no create hook";
    return {};
  }
  std::uint32_t assertion_id = 0;
  bool active = false;
  std::string local_error;
  if (!ops.create(reason, assertion_id, active, local_error, ops.context)) {
    if (error != nullptr) *error = std::move(local_error);
    return {};
  }
  if (!active) return {};
  return SleepGuard{assertion_id, ops};
}

void SleepGuard::Release() noexcept {
  if (!active_) return;
  const std::uint32_t id = std::exchange(assertion_id_, 0);
  const Ops ops = std::exchange(ops_, Ops{});
  active_ = false;
  if (ops.release != nullptr) ops.release(id, ops.context);
}

}  // namespace talos::launcher
