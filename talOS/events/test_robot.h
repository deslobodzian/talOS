#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/events_test_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/time.h"

namespace talos::event::testing {

// Topic names used by the determinism tests. The realtime loop turns these
// into shared-memory objects, so they carry a pid suffix at construction.
struct Topics {
  std::string sensor;
  std::string setpoint;
  std::string command;
};

// A minimal robot exercising every kind of event source: a periodic control
// timer, a watcher that must not miss a message, a fetcher that samples the
// newest value, and a sender whose output is what replay verifies.
//
// Templated on the loop rather than taking an interface, so the identical
// handler code runs on the realtime, simulated and replay loops with no
// virtual dispatch. Every input it reads comes from the loop, which is what
// makes its output a pure function of the recorded inputs.
template <typename Loop>
class TestRobot {
 public:
  TestRobot(Loop& loop, const Topics& topics, float gain)
      : loop_{&loop}, gain_{gain} {
    // Registration order defines source ids and therefore the shape of the
    // log. Keep it stable.
    watch<EventsTest::SensorMessage, &TestRobot::on_sensor>(loop, topics.sensor,
                                                            this);
    setpoint_ =
        make_fetcher<EventsTest::SetpointMessage>(loop, topics.setpoint);
    command_ = make_sender<EventsTest::CommandMessage>(loop, topics.command);
    control_ = make_timer<&TestRobot::on_control>(loop, "control", this);
  }

  Timer<Loop>& control_timer() { return control_; }

  // Observed history, for tests that want to assert on behaviour rather than
  // just on log equality.
  const std::vector<float>& efforts() const { return efforts_; }
  std::uint64_t sensor_count() const { return sensor_count_; }
  std::uint64_t control_count() const { return control_count_; }
  std::uint64_t dropped_sensors() const { return dropped_sensors_; }

 private:
  void on_sensor(const Context& context,
                 const EventsTest::SensorMessage& message) {
    position_ = message.position();
    velocity_ = message.velocity();
    last_sensor_id_ = message.id();
    dropped_sensors_ += context.dropped;
    ++sensor_count_;
  }

  void on_control(const Context& context) {
    ++control_count_;

    if (const auto setpoint = setpoint_.fetch()) {
      target_ = setpoint->target();
    }

    // Deliberately reads the clock through the loop: on replay this returns
    // the recorded time, so the output stays a function of the log.
    const float damping = static_cast<float>(context.latency().count()) * 1e-9F;

    const float effort =
        gain_ * (target_ - position_) - (0.1F * velocity_) - damping;

    efforts_.push_back(effort);

    command_.send(
        EventsTest::CommandMessage{last_sensor_id_, effort, context.sequence});
  }

  Loop* loop_;
  float gain_;

  Fetcher<Loop, EventsTest::SetpointMessage> setpoint_{};
  Sender<Loop, EventsTest::CommandMessage> command_{};
  Timer<Loop> control_{};

  float position_{0.0F};
  float velocity_{0.0F};
  float target_{0.0F};
  std::int32_t last_sensor_id_{0};

  std::vector<float> efforts_;
  std::uint64_t sensor_count_{0};
  std::uint64_t control_count_{0};
  std::uint64_t dropped_sensors_{0};
};

}  // namespace talos::event::testing
