#ifndef DJIOSDK_INFLUX_METRICS_MISSION_HPP
#define DJIOSDK_INFLUX_METRICS_MISSION_HPP

// System Includes
#include <iostream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

namespace mission
{
bool
subscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout);
bool
unsubscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout);
}

#endif // DJIOSDK_INFLUX_METRICS_MISSION_HPP