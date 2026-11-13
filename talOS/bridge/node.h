#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "talOS/bridge/clock_offset.h"
#include "talOS/bridge/merge.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/os/poller.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/protocol/udp.h"
#include "talOS/rtms/wake.h"

namespace talos::hardware {

namespace bridge_test {
class InProcessGateway;  // in_process_gateway.h, the bridge's own tests
}  // namespace bridge_test

// This node's identity, in one place: the registry row, the `--describe`
// output and the launcher's config all have to agree on both strings, and a
// second copy of either is a second thing to forget.
inline constexpr const char* kNodeName = "hardware_node";
inline constexpr const char* kNodeTarget = "//talOS/bridge:hardware_node";

struct SubsystemTracker {
  std::string name;
  Devices devices;
  uint32_t period_us{5000};
  // Its standing in the merge (merge.h): the owner timeout
  // (OwnerTimeoutUs), last accepted request on the bridge clock and its
  // observation on the gateway clock, reset on every controller transition.
  MergeOwner merge;
  // Owner silence as /hw/status reports it (timed_out_mask and
  // alert_warning_owner_timed_out, HardwareNode::OwnerSilent), tracked apart
  // from `merge`, whose Reset() at every controller transition must keep
  // dropping the owner's demands without the status reporting a timeout.
  // An owner is reported only once `heard`: this bridge process has accepted
  // a request from it (one refused for identity or ownership is not heard).
  // Nothing clears it, a gateway reboot or link loss included, so a session's
  // start, however late a node's first request, reports no owner, and one
  // whose node never sends is left to the launcher (REQ-OPS-05). It gates
  // only the status and the exit summary: the merge, and so the owner
  // timeout's neutral, treats a never-heard owner as before.
  // Silence counts on the bridge clock from `silence_from_us`: the link
  // becoming ready (a State accepted and the current boot configured: the
  // session's start, or a gateway reboot's end), the last accepted request,
  // or the first controller transition since that request while the owner
  // was not silent (its requests in flight then carry the old identity and
  // are refused, so its next one comes up to a period later). One such
  // transition per accepted request, so a gateway that keeps transitioning
  // cannot hide an owner none of whose requests is accepted.
  bool heard{false};
  std::optional<uint64_t> silence_from_us;
  bool transition_since_accepted{false};
  // Whether OwnerSilent held at this subsystem's last evaluation in Tick()
  // (the exit summary's timed_out=), and the rising edges seen: distinct
  // owner-timeout episodes, the exit summary's timeouts=.
  bool silent{false};
  uint64_t timeouts{0};
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> subscriber;

  // Requests actually decoded off this subsystem's topic, and where that topic
  // sits in the introspection manifest. Kept here because the registry reports
  // per-topic traffic, and a subsystem that has gone quiet is the single most
  // useful thing to see when actuators stop responding.
  uint64_t requests{0};
  std::size_t source_index{0};
  // Requests the latest-value read passed over: published while an earlier
  // one waited unread, the sum of ipc::Received::dropped over this topic's
  // reads. Its registry source row's `dropped`. Normally 0; nonzero when the
  // node's wakes to the bridge fail or the bridge falls behind, and this
  // subsystem merges at a fraction of its rate (TODO.md RT-7).
  uint64_t requests_skipped{0};
  // HardwareNode::Pass()'s idle-pass guard (kMaxIdlePasses): whether this
  // topic's arm failed in the last pass, and whether the guard has counted
  // it in Wakeups::stalled_arms since its arm last succeeded or a read last
  // returned a request from it.
  bool arm_failed{false};
  bool arm_stalled{false};

  // Positions in the merged command of the actuators this subsystem owns,
  // resolved from `devices` by HardwareNode::IndexActuators so a timeout
  // neutralizes without searching. Ids absent from the merge are left out.
  std::vector<uint16_t> motor_slots;
  std::vector<uint16_t> digital_output_slots;
  std::vector<uint16_t> pwm_output_slots;
};

class HardwareNode {
 public:
  explicit HardwareNode(config::RobotConfig robot_config,
                        std::string remote_ip = "127.0.0.1",
                        uint16_t remote_port = 5802, uint16_t local_port = 5803,
                        bool simulation = false, uint64_t session_id = 0,
                        std::string log_path = {}, uint64_t log_max_bytes = 0,
                        std::chrono::nanoseconds log_window = {});

  ~HardwareNode() = default;

  bool Open();
  // One merge step at `now_us` (the bridge clock): receives every waiting
  // gateway datagram up to kReceiveBudget, reads each request topic, merges,
  // sends the command when it changed or the heartbeat is due, and runs the
  // time-driven duties (clock probe, owner timeouts, /hw/status). Run() calls
  // it on every wakeup; tests call it with synthetic time.
  void Tick(uint64_t now_us);
  // The earliest bridge-clock instant after a Tick(now_us) at which Tick has
  // time-driven work: the next clock probe or its retry, the completion of an
  // echoed probe (now_us itself), /hw/status, each merged owner's timeout,
  // and the command heartbeat while one would be sent. Each is solved from
  // the condition Tick tests. A duty already due at now_us, which that Tick
  // could not do (a failed send), is retried kRetryUs later. While the UDP
  // socket is closed (SocketCounters) the probe and the command, which
  // cannot be sent, give way to its reopen.
  uint64_t NextDutyUs(uint64_t now_us) const;
  // What Run() does after a Pass(): wait until `until_us` (bridge clock),
  // returning early on input unless `ignore_input`, when the CPU budget is
  // spent and it sleeps through any input.
  struct Wait {
    uint64_t until_us;
    bool ignore_input;
  };
  // Run()'s loop body, without its wait: Tick(now_us), the registry's
  // liveness sample (on every pass, whatever woke it), then arming every
  // request subscriber's wake. Waits until now_us when input is known to be
  // waiting (an arm found an unread request, or Tick stopped at
  // kReceiveBudget), otherwise until the next duty, the full registry
  // sample, the liveness backstop or the longest wait. Once the CPU budget
  // is spent (see kCpuWindowUs) it sleeps through input until the window
  // ends, or until the next duty or registry sample if that is sooner.
  Wait Pass(uint64_t now_us);
  int Run(int duration_s = 0);

  // Run()'s waits. OpenWaits() builds the poller and watches the UDP socket,
  // a wake receiver every request subscriber names and the stop pipe
  // (process::StopFd(), when it exists); when any of the first two
  // fails it says so on stderr, counts a fallback, and the loop polls its
  // inputs every kRetryUs instead (false). CloseWaits() undoes it. Run()
  // calls both; they are public so a test can reach the fallback.
  bool OpenWaits();
  void CloseWaits();

  // Test seams, null in production. `make_wake_receiver` builds the request
  // wake receiver instead of WakeReceiver's constructor (a test throws from
  // it to reach the polling fallback). `arm_request` stands in for
  // Subscriber::arm_wake in Pass() (a test reports unread data no read
  // returns, to reach the kMaxIdlePasses guard).
  // `send_error`, when set, is asked before each send to the gateway: a
  // nonzero errno fails that send with it, unsent, as if the OS had
  // returned it (a test reaches the errors a loopback socket never gives).
  struct Seams {
    WakeReceiver (*make_wake_receiver)(){nullptr};
    bool (*arm_request)(ipc::Subscriber<Packet>& subscriber){nullptr};
    int (*send_error)(){nullptr};
  };
  void set_seams(Seams seams) { seams_ = seams; }

  // Run()'s CPU budget. In each kCpuWindowUs window (starting at the first
  // pass after the last one ended) the loop makes at most kPassBudget passes
  // and receives at most kDatagramBudget datagrams before it sleeps through
  // input to the window's end: a gateway flooding datagrams, or a node
  // publishing requests without pause, cannot hold the highest-priority
  // loop runnable. Duties are not delayed: the sleep ends at the next one.
  // Normal traffic (a State every 5 ms, a request per actuator subsystem per
  // period, Driver Station packets every 20 ms) stays far below both.
  static constexpr uint64_t kCpuWindowUs = 1'000;
  static constexpr uint64_t kPassBudget = 8;
  static constexpr uint64_t kDatagramBudget = 64;
  // Datagrams one Tick receives at most, so a burst delays rather than
  // stalls the merge; Run() then ticks again without waiting, within the
  // CPU budget.
  static constexpr int kReceiveBudget = 32;
  // Passes that may skip the wait on a failed arm without consuming input
  // (a read the writer keeps tearing), before the loop waits anyway: the
  // arm still requested a notification, and the longest wait bounds it.
  static constexpr uint32_t kMaxIdlePasses = 3;

  // Run()'s wakeups, by what the loop then found: a gateway datagram, a
  // request, or neither and the wait reached its deadline. A wakeup that
  // found both counts in both. Read only while Run() is not running.
  struct Wakeups {
    uint64_t passes{0};    // every Pass(), immediate ones included
    uint64_t udp{0};       // woken readable; Tick received a datagram
    uint64_t request{0};   // woken readable; Tick read a request
    uint64_t deadline{0};  // the wait reached its deadline
    // Passes that spent the CPU budget, each followed by a sleep through
    // input. Stays 0 under normal traffic.
    uint64_t throttled{0};
    // OpenWaits() calls that fell back to polling. Stays 0.
    uint64_t wait_fallbacks{0};
    // Request topics whose failed arms the kMaxIdlePasses guard stopped
    // holding the loop awake, each counted once until its arm succeeds or a
    // read returns a request from it again; the registry row's
    // stalled_arms. Stays 0.
    uint64_t stalled_arms{0};
  };
  const Wakeups& wakeups() const { return wakeups_; }

  // The UDP socket to the gateway. A send that fails with an error the
  // socket may never recover from (protocol::SendErrorNeedsNewSocket: a
  // socket shut down or defunct, or its source address gone from the host)
  // closes it; Tick() then opens a new one on the same ports
  // (RuntimeUdpPeer::Reopen): the first attempt on the next Tick
  // (NextDutyUs: kRetryUs later at the latest), then at most once per
  // kReopenRetryUs, which is also the cadence while the network has no
  // route to the gateway and each attempt fails.
  // While it is closed nothing is sent or received, so the gateway times
  // the command out and /hw/status's state_age_us grows, as for any lost
  // link. Any other send error (ECONNREFUSED while the gateway is down,
  // ENOBUFS, ...) keeps the socket: it clears by itself, and the failed duty
  // is retried kRetryUs later.
  // stderr gets one line when an outage starts, with its errno, and one when
  // a send succeeds again. Read only while Run() is not running.
  struct SocketCounters {
    uint64_t send_failures{0};  // sends that failed, of either kind
    int last_send_error{0};     // the errno of the last; 0 before one
    uint64_t reopens{0};        // sockets opened to replace a lost one
  };
  const SocketCounters& socket_counters() const { return socket_counters_; }
  // The socket's fd, -1 while it is closed (a test breaks it through this).
  int udp_fd() const { return peer_.NativeFd(); }
  static constexpr uint64_t kReopenRetryUs = 100'000;

  bool is_configured() const { return is_configured_; }
  uint64_t states_received() const { return states_received_; }
  uint64_t commands_sent() const { return commands_sent_; }
  uint64_t driver_station_packets_received() const {
    return driver_station_packets_received_;
  }
  bool is_subsystem_timed_out(const std::string& name) const {
    auto it = subsystems_.find(name);
    return it != subsystems_.end() && it->second.merge.timed_out;
  }
  // Owner-timeout episodes of subsystem `name` so far (SubsystemTracker),
  // as the exit summary prints them; 0 for a name that owns no actuator.
  uint64_t owner_timeouts(const std::string& name) const {
    auto it = subsystems_.find(name);
    return it != subsystems_.end() ? it->second.timeouts : 0;
  }
  const Command& current_command() const { return full_cmd_; }
  uint64_t identity_rejected() const { return identity_rejected_; }
  uint64_t ownership_rejected() const { return ownership_rejected_; }
  // Bounded-heartbeat interval for unchanged merges. Derived from the
  // configured command timeout (timeout/5, clamped to [5, 20] ms) so a live
  // merge always refreshes the gateway well before it expires.
  uint64_t heartbeat_us() const { return heartbeat_us_; }
  // The gateway-to-node clock mapping, fed by every decoded /hw/state frame.
  // It is also the State ordering gate: a frame it ignores is dropped.
  const ClockOffsetEstimator& clock() const { return clock_; }

  // The /hw/status message as of `now_us` (the bridge's monotonic clock, the
  // same one Tick() receives). Tick() publishes it every kStatusPeriodUs.
  // Run() reads that clock from talos::event::Poller::now(), the clock every
  // node's Node::now() runs on, so the clock_* mapping lands in node time.
  bridge::BridgeStatus Status(uint64_t now_us) const;

  void PushConfig(uint64_t now_us);

  // The shape this node would take, answered without opening a socket, a
  // shared-memory segment or any hardware. `--describe` prints this; the
  // launcher assembles the declared graph out of it before it spawns anything.
  // It comes from the same BuildIntrospectionManifest() that Open() registers,
  // so the declared graph and the running one differ only when the node does.
  introspect::Description Describe();

  // Copies the counters above into this node's registry slot: each topic's
  // traffic, each subsystem's skipped requests (SubsystemTracker), and the
  // wake loss (the wake notifications its publishers failed to send, and
  // OpenWaits() fallbacks). Called from Run(); exposed so a test can drive
  // it without a real event loop.
  void PublishIntrospection();

 private:
  // The in-process gateway the bridge's component tests drive this node
  // through instead of a socket: a testonly library private to talOS/bridge
  // (in_process_gateway.h). It calls OpenTopics() and HandleGatewayFrame()
  // and sets frame_sink_; nothing else can.
  friend class bridge_test::InProcessGateway;

  // Open() after its socket: the shared-memory topics, the packet log and
  // the registry row.
  void OpenTopics();
  // One decoded datagram from the gateway (a State, a Driver Station packet
  // or a config acknowledgement), received at `now_us`: what Tick()'s
  // receive loop does with each frame of this protocol version it reads.
  void HandleGatewayFrame(const protocol::DecodedFrame& frame, uint64_t now_us);

  // The sum of the per-topic counters: the registry's dispatch count.
  std::uint64_t DispatchCount() const;
  // Heartbeat and dispatch count alone, one registry sample; Pass() calls it
  // on every pass that does not call PublishIntrospection().
  void RefreshLiveness();

  // Describes this node to the live registry. The hardware node predates the
  // event loop and drives its own tick, so it has no manifest to hand over --
  // it declares one, in the order Open() creates the transports.
  void BuildIntrospectionManifest();

  // How this node's ends behave, by topic. The manifest cannot say it: to the
  // loop, /hw/command is a sender like any other. Without this the graph
  // reports the RoboRIO link as a broken topic, which is how a report stops
  // being read.
  static const std::vector<introspect::EndpointAttribute>& EndpointAttributes();

  // Whether a subsystem request may merge: same configuration, boot, and
  // epoch as last observed (exact match), and an observation its MergeOwner
  // admits against last_sample_time_us_ (MergeOwner::Admits: younger than
  // command_timeout_us, never future, monotonic per subsystem). The gateway
  // enforces the same age against its own clock, which is later than the
  // latest State's sample, so it remains the safety backstop.
  bool HasRequestIdentity(const SubsystemTracker& tracker,
                          const Command& partial) const;
  // Whether every actuator a request addresses belongs to the subsystem that
  // published it. One stray id rejects the whole request: a partial merge
  // would let a buggy subsystem drive hardware it does not own.
  bool OwnsRequestDevices(const SubsystemTracker& tracker,
                          const Command& partial) const;
  // Drops every cached merge and forces all subsystems back to timed-out, so
  // nothing is sent until each publishes a fresh request. Called on every
  // controller transition: reboot, config change, disable/re-enable, and fault
  // recovery (all of which the gateway reports as a new boot, config, epoch,
  // or enabled/fault edge), at `now_us`, which starts the silence of each
  // owner not yet silent (SubsystemTracker::silence_from_us).
  void InvalidateRequests(uint64_t now_us);
  // Whether the link is ready for requests (a State accepted and the
  // current gateway boot has acknowledged this configuration), after a
  // gateway frame received at `now_us`. Becoming ready starts every owner's
  // silence at `now_us`, as the session's start does.
  void UpdateLinkReady(uint64_t now_us);
  // Whether `tracker`'s owner is silent at `now_us` as /hw/status reports
  // it: the link is ready, the owner has been heard, and nothing restarted
  // its silence (silence_from_us) for longer than its owner timeout. False
  // while the link is not ready, when alert_error_link_lost or
  // alert_error_not_configured says why nothing merges, and for an owner
  // never heard (SubsystemTracker::heard).
  bool OwnerSilent(const SubsystemTracker& tracker, uint64_t now_us) const;
  void NeutralizeAll();
  // Rebuilds the id -> position tables below and every tracker's slots from
  // full_cmd_'s actuator layout. The first occurrence of an id wins, matching
  // a front-to-back search.
  void IndexActuators();
  // When the merged command is next due on the wire, the one rule Tick()
  // sends by and NextDutyUs() wakes for: `now_us` when it changed since the
  // last send (or none was sent since the last controller transition), else
  // the heartbeat, last_send_us_ + heartbeat_us_. While the latest State
  // says the gateway cannot take a command (GatewayReady false: not
  // configured and commissioned, not enabled, or faulted), a change of the
  // observation alone is no change. The demands it would carry are neutral:
  // the gateway starts a new epoch whenever it stops being ready, so every
  // request of the latest State's epoch was built from a State that was not
  // ready either, and nodes flush neutral then (InvalidateRequests dropped
  // every earlier one). Holding it only lets the gateway's held command look
  // older, never drive. Every edge back to ready is a controller transition
  // (a new epoch, or the fault edge), which sends the next merge at once.
  // The heartbeat carries the newer observation instead, so the rate is one
  // command per heartbeat whatever the owners' phases. A change of a demand,
  // an output or an identity word still sends at once. While the gateway can
  // take a command, every change sends at once: holding the observation
  // would age the command early at the gateway (ARCHITECTURE.md "The
  // tick").
  uint64_t CommandDueUs(uint64_t now_us) const;
  // Every frame to the gateway leaves here: a send that fails with an error
  // the socket never recovers from closes it (SocketCounters). Returns the
  // send's status; kSocketError while the socket is closed.
  protocol::UdpStatus SendToGateway(protocol::FrameType type, uint64_t now_us,
                                    const void* payload,
                                    std::size_t payload_size);
  // Opens the replacement socket once one is due (SocketCounters), and
  // watches it in Run()'s poller.
  void ReopenSocket(uint64_t now_us);
  // OpenWaits()'s fallback: no poller, inputs polled every kRetryUs.
  void FallBackToPolling(const char* reason);
  // Publishes `packet` on `publisher` stamped with the Poller clock now, and
  // logs it as a SEND of manifest source `source_id` carrying that stamp and
  // the transport sequence, which every reader's MESSAGE record of it also
  // carries (so a replay or a merge counts it once), with `now_us` (the pass
  // that produced it) as its event time.
  void Publish(ipc::Publisher<Packet>& publisher, uint16_t source_id,
               const Packet& packet, uint64_t now_us);
  // One stderr line per gateway boot naming its program and build (the
  // launcher keeps it in the node's session log; /hw/status carries it too).
  static void ReportGatewayBuild(const State& state);
  // Prints the packet log's error once, whether capture stopped or the log
  // failed (e.g. its file could not be created).
  void ReportLogFailure();

  config::RobotConfig robot_config_;
  Config config_;
  uint64_t config_id_{0};
  std::string remote_ip_;
  uint16_t remote_port_;
  uint16_t local_port_;
  bool simulation_;

  protocol::RuntimeUdpPeer peer_;
  // Where SendToGateway() hands every frame instead of peer_, each send
  // succeeding: set only by bridge_test::InProcessGateway, unset in
  // production.
  struct FrameSink {
    void (*send)(void* context, protocol::FrameType type, uint64_t now_us,
                 std::span<const uint8_t> payload){nullptr};
    void* context{nullptr};
  };
  FrameSink frame_sink_{};
  // What every request subscriber's reader slot names while Run() waits.
  // Declared before subsystems_, so their slots are released first and no
  // publisher notifies a closed receiver.
  std::optional<WakeReceiver> request_wake_;
  // Run()'s waits (OpenWaits); empty while it cannot build one.
  std::optional<event::Poller> poller_;
  Seams seams_{};
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> state_pub_;
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> cmd_pub_;
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> ds_pub_;
  std::unique_ptr<ipc::Publisher<bridge::BridgeStatus>> status_pub_;

  std::map<std::string, SubsystemTracker> subsystems_;
  Command full_cmd_{};
  // Where each actuator id sits in full_cmd_, so merging a request is one
  // lookup per addressed actuator. Ids not in the table are ignored.
  std::unordered_map<uint16_t, uint16_t> motor_index_;
  std::unordered_map<uint16_t, uint16_t> digital_output_index_;
  std::unordered_map<uint16_t, uint16_t> pwm_output_index_;

  event::Manifest introspection_manifest_;
  std::optional<introspect::NodeRegistration> registration_;
  uint64_t session_id_{0};
  std::string log_path_;
  uint64_t log_max_bytes_{0};
  std::chrono::nanoseconds log_window_{0};  // LogWriterOptions::window
  std::unique_ptr<event::log::LogWriter> log_;
  bool log_failure_reported_{false};
  State latest_state_{};
  // Run()'s registry refreshes, on the bridge clock.
  uint64_t next_introspection_us_{0};
  uint64_t next_liveness_us_{0};
  // What the last Tick did: datagrams it received, requests it read, and
  // whether it stopped at kReceiveBudget rather than an empty socket.
  uint32_t tick_datagrams_{0};
  uint32_t tick_requests_{0};
  bool tick_receive_capped_{false};
  // Consecutive passes that found input waiting but consumed none.
  uint32_t idle_passes_{0};
  // The current CPU budget window: its start, and its passes and datagrams.
  uint64_t cpu_window_start_us_{0};
  uint64_t cpu_window_passes_{0};
  uint64_t cpu_window_datagrams_{0};
  // The longest single wait: kMaxWaitUs, or kRetryUs when Run() cannot wait
  // on its inputs and polls them instead.
  uint64_t max_wait_us_{kMaxWaitUs};
  Wakeups wakeups_{};
  SocketCounters socket_counters_{};
  // An outage: from a send that closed the socket to the first send that
  // succeeds after it.
  bool socket_lost_{false};
  // The last reopen attempt, on the bridge clock; none yet while
  // !reopen_attempted_.
  bool reopen_attempted_{false};
  uint64_t last_reopen_us_{0};

  bool is_configured_{false};
  uint64_t last_boot_id_{0};
  uint64_t last_ds_sample_time_us_{0};
  uint64_t last_epoch_{0};
  uint64_t last_sample_time_us_{0};
  uint32_t last_state_flags_{0};
  uint64_t states_received_{0};
  uint64_t identity_rejected_{0};
  uint64_t ownership_rejected_{0};
  uint64_t commands_sent_{0};
  uint64_t driver_station_packets_received_{0};
  // Bridge-clock receipt time of the last /hw/state frame; meaningful once
  // states_received_ is non-zero.
  uint64_t last_state_receipt_us_{0};
  // Whether a State has been accepted (past the ordering gate) yet.
  bool state_accepted_{false};
  // UpdateLinkReady()'s last answer.
  bool link_ready_{false};
  ClockOffsetEstimator clock_;
  uint64_t last_probe_send_us_{};
  bool probe_waiting_{false};
  // Set once a frame of another protocol version is logged, until a frame
  // of this version arrives.
  bool version_mismatch_logged_{false};
  // Set once an undecodable State is logged, until a State decodes.
  bool state_decode_failure_logged_{false};
  uint64_t pending_probe_send_us_{};
  uint64_t pending_probe_sample_us_{};
  uint64_t pending_probe_boot_id_{};
  uint64_t statuses_published_{0};
  uint64_t last_status_us_{0};
  uint64_t last_push_time_us_{0};
  uint64_t config_push_backoff_us_{50000};

  // Run() sleeps until a gateway datagram, a request publication, a stop (the
  // poller watches process::StopFd()) or the next time-driven duty.
  // kMaxWaitUs caps one wait as a backstop: the loop still checks
  // process::stop_requested at least this often should the stop pipe be
  // missing or a stop leave it unreadable.
  static constexpr uint64_t kMaxWaitUs = 10'000;
  // A duty Tick could not do (a failed send) is retried this much later,
  // so a refused send costs at most a 1 kHz retry, never a busy loop.
  static constexpr uint64_t kRetryUs = 1'000;
  // The macOS time-constraint period ConfigureRealtime is given: after each
  // wakeup the bridge's work (computation half of it) is due within it. Not
  // a polling period; the kernel orders time-constraint threads by deadline,
  // so a longer one would put the bridge behind the control nodes.
  static constexpr uint64_t kWakeBudgetUs = 1'000;
  // Registry: every pass stores the heartbeat and dispatch count, so a wake
  // for them alone is due only once no pass has come for
  // kLivenessBackstopUs (never while waits work, kMaxWaitUs being shorter),
  // far inside the launcher's 500 ms minimum stall timeout.
  static constexpr uint64_t kLivenessBackstopUs = 50'000;
  static_assert(kMaxWaitUs <= kLivenessBackstopUs);
  // The full sample, per-source counters included.
  static constexpr uint64_t kIntrospectionPeriodUs = 250'000;
  static constexpr uint64_t kStatusPeriodUs = 20000;
  // Clock probes: one outstanding, at most every kProbePeriodUs. An
  // unanswered probe (or a lost echo) is replaced after kProbeRetryUs, well
  // inside ClockOffsetEstimator::kExchangeLifetimeUs, so one lost datagram
  // never leaves the estimator without an unexpired exchange. An echo whose
  // round trip exceeds the retry interval is therefore never used.
  static constexpr uint64_t kProbePeriodUs = 20'000;
  static constexpr uint64_t kProbeRetryUs = 100'000;
  static_assert(kProbeRetryUs * 5 <= ClockOffsetEstimator::kExchangeLifetimeUs);
  uint64_t heartbeat_us_{20000};
  Command last_sent_cmd_{};
  uint64_t last_send_us_{0};
  bool have_last_sent_{false};
  bool last_sent_had_active_subsystem_{false};
};

}  // namespace talos::hardware
