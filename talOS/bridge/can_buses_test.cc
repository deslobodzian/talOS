// The /hw/status CAN bus table the bridge (and SimHardware) assembles from
// the one bus slot each State reports: unit tier, no transport.
#include "talOS/bridge/can_buses.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace talos::hardware {
namespace {

constexpr uint64_t kConfigId = 0xc0ffee;

State Report(uint64_t boot_id, uint64_t config_id, uint8_t count, uint8_t index,
             CanBusSample bus) {
  State state;
  state.boot_id = boot_id;
  state.config_id = config_id;
  state.health.can_bus_count = count;
  state.health.can_bus_index = index;
  state.health.can_bus = bus;
  return state;
}

constexpr CanBusSample kOk{CanBusReadState::kOk, 0.16F, 1, 2, 3, 4, 0};

// Each State fills only its own slot, and /hw/status shows every slot below
// the count with the figures its last report carried.
// REQ-OPS-20
TEST(CanBusTable, KeepsEachBusFromTheStateThatReportedIt) {
  CanBusTable table{kConfigId};
  table.Observe(Report(7, kConfigId, 2, 0, kOk));
  table.Observe(Report(7, kConfigId, 2, 1,
                       {CanBusReadState::kOk, 0.5F, 9, 8, 255, 128, 3}));
  bridge::BridgeStatus status{};
  table.Fill(status);
  EXPECT_EQ(status.can_bus_count(), 2);
  const auto* slots = status.can_buses();
  EXPECT_FLOAT_EQ(slots->Get(0)->utilization_frac(), 0.16F);
  EXPECT_EQ(slots->Get(1)->bus_off_count(), 9u);
  EXPECT_EQ(slots->Get(1)->transmit_buffer_full_count(), 8u);
  EXPECT_EQ(slots->Get(1)->receive_error_count(), 255);
  EXPECT_EQ(slots->Get(1)->transmit_error_count(), 128);
  EXPECT_EQ(slots->Get(1)->read_failure_count(), 3u);
  EXPECT_EQ(slots->Get(1)->read_state(), bridge::CanBusReadState::OK);
  EXPECT_EQ(slots->Get(2)->read_state(), bridge::CanBusReadState::NONE);
  EXPECT_FALSE(status.alert_warning_can_bus_unread());
}

// A slot index means a bus only under the configuration both ends listed it
// from: a State of another configuration (a stale RoboRIO config, before the
// bridge's push lands) stores nothing and empties the table, and a new
// gateway boot starts it over, so no slot shows another configuration's bus
// or a retired boot's read.
// REQ-OPS-20
TEST(CanBusTable, IgnoresAnotherConfigurationAndStartsOverOnANewBoot) {
  CanBusTable table{kConfigId};
  table.Observe(Report(7, kConfigId + 1, 2, 0, kOk));
  EXPECT_EQ(table.count(), 0);
  EXPECT_EQ(table.slot(0).read_state, CanBusReadState::kNone);

  table.Observe(Report(7, kConfigId, 2, 0, kOk));
  table.Observe(Report(7, kConfigId, 2, 1, kOk));
  table.Observe(Report(7, kConfigId + 1, 2, 0, kOk));
  EXPECT_EQ(table.count(), 0) << "another configuration's State kept the table";
  EXPECT_EQ(table.slot(1).read_state, CanBusReadState::kNone);

  table.Observe(Report(7, kConfigId, 2, 0, kOk));
  table.Observe(Report(7, kConfigId, 2, 1, kOk));
  table.Observe(Report(8, kConfigId, 2, 0, kOk));
  EXPECT_EQ(table.count(), 2);
  EXPECT_EQ(table.slot(0).read_state, CanBusReadState::kOk);
  EXPECT_EQ(table.slot(1).read_state, CanBusReadState::kNone)
      << "the previous boot's read of bus 1 survived the reboot";
}

// alert_warning_can_bus_unread holds while a configured bus's last report
// reads FAILED or STALE, and clears once that bus reports OK; a bus not yet
// reported (NONE), still pending or simulated raises nothing.
// REQ-OPS-20
TEST(CanBusTable, AlertsOnAFailedOrStaleBusUntilItReadsAgain) {
  for (const auto unread :
       {CanBusReadState::kFailed, CanBusReadState::kStale}) {
    SCOPED_TRACE(static_cast<int>(unread));
    CanBusTable table{kConfigId};
    table.Observe(Report(7, kConfigId, 3, 0, kOk));
    table.Observe(
        Report(7, kConfigId, 3, 1, {.read_state = CanBusReadState::kPending}));
    EXPECT_FALSE(table.unread());
    table.Observe(Report(7, kConfigId, 3, 2, {.read_state = unread}));
    EXPECT_TRUE(table.unread());
    bridge::BridgeStatus status{};
    table.Fill(status);
    EXPECT_TRUE(status.alert_warning_can_bus_unread());
    table.Observe(Report(7, kConfigId, 3, 0, kOk));
    EXPECT_TRUE(table.unread()) << "another bus's report cleared it";
    table.Observe(Report(7, kConfigId, 3, 2, kOk));
    EXPECT_FALSE(table.unread());
  }
  CanBusTable simulated{kConfigId};
  simulated.Observe(
      Report(7, kConfigId, 1, 0, {.read_state = CanBusReadState::kSimulated}));
  EXPECT_FALSE(simulated.unread());
}

}  // namespace
}  // namespace talos::hardware
