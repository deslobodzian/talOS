#pragma once

#include <cstdint>

#include "driver_station/driver_station.h"

namespace talos::driver_station {

DriverStationData SampleDriverStation(uint64_t now_us);

}  // namespace talos::driver_station
