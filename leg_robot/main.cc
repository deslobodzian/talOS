#include <chrono>
#include <iostream>
#include <cmath>
#include <numbers>
#include <thread>

#include "third_party/mjbots/moteus/moteus.h"
#include "third_party/mjbots/moteus/moteus_protocol.h"

namespace moteus = mjbots::moteus;

int main() {
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;

    moteus::Controller::Options options;
    options.id = 1;

    moteus::Controller motor{options};
    moteus::PositionMode::Command command;

    auto duration = Clock::now() + 4s;
    constexpr auto freq = 1.0 / 2.0;

    while (Clock::now() < duration) {

        auto seconds = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        auto sin_out = sin(2.0 * std::numbers::pi * freq * seconds);
        command.position = static_cast<double>(sin_out);

        //std::printf("Desired Position: %f\n", sin_out);
        const auto maybe_result = motor.SetPosition(command);

        if (maybe_result) {
            const auto& v = maybe_result->values;
            const auto error = sin_out - v.position;
            std::printf(
            "Mode: %i\nPosition: %f\nVelocity: %f\nDesired: %f\nError: %f",
                v.mode, v.position, v.velocity, sin_out, error
            );
        }
        std::this_thread::sleep_for(1us);
    }

    motor.SetStop();

    std::cout << "Let robot\n";
    return 0;
}
