#pragma once

// System Includes
#include <boost/asio.hpp>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// InfluxDB
#include <InfluxDBFactory.h>

struct MissionConfig
{
  float   altitude;
  float   radius;
  uint8_t numStops;
  uint8_t waitTime;

  MissionConfig(float   altitude,
                float   radius,
                uint8_t numStops,
                uint8_t waitTime)
    : altitude(altitude)
    , radius(radius)
    , numStops(numStops)
    , waitTime(waitTime)
  {
  }

  std::string toString() const;
};

std::ostream&
operator<<(std::ostream& o, const MissionConfig& mc);

enum PointType
{
  waypoint,
  hotpoint
};

enum MissionStatus
{
  undefined,
  enRoute,
  waiting,
  completed
};

class MetricsMission
{
public:
  static const int PACKAGE_ID    = 0;
  MissionStatus    missionStatus = MissionStatus::undefined;

  MetricsMission(Vehicle*            vehiclePtr,
                 influxdb::InfluxDB* influxDBPtr,
                 PointType           missionType,
                 int                 responseTimeout);

  ~MetricsMission();

  bool runMission(MissionConfig* mission);
  void runContext();
  bool stopMission();
  bool flyToCenter(float altitude = 5.0f);

private:
  Vehicle*                vehiclePtr;
  influxdb::InfluxDB*     influxDBPtr;
  PointType               missionType;
  int                     responseTimeout;
  WayPointSettings        centerPoint;
  boost::asio::io_context ctx;

public:
  boost::asio::steady_timer metricsTimer;
  void                      commitMetrics();
  void                      flushMetrics();

private:
  WayPointSettings getCurrentPoint();

  bool runWaypointMission(MissionConfig* mission);
  bool runHotpointMission(MissionConfig* mission);
  bool stopWaypointMission();
  bool stopHotpointMission();

  bool subscribe();
  bool unsubscribe();

  static void setWaypointDefaults(WayPointSettings* wp);
  static void copyWaypointSettings(WayPointSettings*       dst,
                                   const WayPointSettings* src);
  static void setWaypointInitDefaults(WayPointInitSettings* fdata);

  std::vector<WayPointSettings> createWaypoints(MissionConfig* mission);
  /**
   * @brief This function is used to generate a new waypoint that is displaced
   * from another waypoint by a radius and angle.
   * @param oldWp Pointer to the old waypoint to take as a base
   * @param radius The radius of the displacement in meters
   * @param angle The angle of the displacement in degrees
   * @return A new waypoint struct containing the displacement
   */
  WayPointSettings newDisplacedWaypoint(WayPointSettings* oldWp,
                                        float             radius,
                                        float             angle);
  void             uploadWaypoints(std::vector<WayPointSettings>& waypoints);

  bool initHotpointMission(MissionConfig* mission);
  bool isInAir();
  bool takeOff();

}; // class MetricsMission