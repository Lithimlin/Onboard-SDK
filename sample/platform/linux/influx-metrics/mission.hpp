#ifndef DJIOSDK_INFLUX_METRICS_MISSION_HPP
#define DJIOSDK_INFLUX_METRICS_MISSION_HPP

// System Includes
#include <boost/asio.hpp>
#include <iostream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

namespace mission
{
bool
runHotpointMission(boost::asio::steady_timer* timer,
                   DJI::OSDK::Vehicle*        vehiclePtr,
                   float                      radius,
                   float                      altitude,
                   int                        numStops,
                   int                        waitTime,
                   int                        responseTimeout);
}

#endif // DJIOSDK_INFLUX_METRICS_MISSION_HPP