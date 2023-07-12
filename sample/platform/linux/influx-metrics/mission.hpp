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
/**
 * @brief Run a new waypoint mission.
 * @param timer           currently unused
 * @param vehiclePtr      Pointer to the vehicle
 * @param radius          The radius of the waypoints in meters
 * @param altitude        The altitude of the mission in meters
 * @param numStops        The number of waypoints in the mission
 * @param waitTime        The time to wait between waypoints in seconds
 * @param responseTimeout The response timeout in seconds
 * @return true if the mission was successfully started, false otherwise
 */
bool
runWaypointMission(boost::asio::steady_timer* timer,
                   DJI::OSDK::Vehicle*        vehiclePtr,
                   float                      radius,
                   float                      altitude,
                   int                        numStops,
                   int                        waitTime,
                   int                        responseTimeout);

void
waypointEventCallback(Vehicle*      vehiclePtr,
                      RecvContainer recvFrame,
                      UserData      userData);
}

#endif // DJIOSDK_INFLUX_METRICS_MISSION_HPP