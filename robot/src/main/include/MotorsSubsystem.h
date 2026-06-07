#pragma once

#include <vector>
#include "motor.h"

// This will handle motor aggregation and commands
class MotorsSubsystem {
public:
    MotorsSubsystem();
private:
    std::vector<Motor> motors_;
};
