#include "mission.hpp"
// #include <boost/asio.hpp>
// #include <boost/bind/bind.hpp>
#include <dji_telemetry.hpp>
#include <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace mission
{

Telemetry::TypeMap<TopicName::TOPIC_GPS_FUSED>::type gpsPosition;
static bool                                          startPositionSetup = false;
static const float RADIUS_OF_EARTH_IN_METERS = 6371000.0f; // meters

bool
subscribe(Vehicle* vehicle, int responseTimeout);
bool
unsubscribe(Vehicle* vehicle, int responseTimeout);

void
setWaypointDefaults(WayPointSettings* wp);

void
copyWaypointSettings(WayPointSettings* dst, const WayPointSettings* src);

void
setWaypointInitDefaults(WayPointInitSettings* fdata);

std::vector<WayPointSettings>
createWaypoints(Vehicle* vehicle,
                float    radius,
                float    altitude,
                int      numWaypoints,
                int      waitTime);

std::vector<WayPointSettings>
generateWaypoints(WayPointSettings* centerPoint,
                  float             radius,
                  int               numWaypoints);

/**
 * @brief This function is used to generate a new waypoint that is displaced
 * from another waypoint by a radius and angle.
 * @param oldWp Pointer to the old waypoint to take as a base
 * @param radius The radius of the displacement in meters
 * @param angle The angle of the displacement in degrees
 * @return A new waypoint struct containing the displacement
 */
WayPointSettings
newDisplacedWaypoint(WayPointSettings* oldWp, float radius, float angle);

void
uploadWaypoints(Vehicle*                       vehicle,
                std::vector<WayPointSettings>& waypoints,
                int                            responseTimeout);

bool
isInAir(Vehicle* vehicle);

bool
takeOff(Vehicle* vehicle, int responseTimeout);

bool
waitTakeoffFinished(Vehicle* vehicle);

static void
WaypointEventCallBack(Vehicle*      vehicle,
                      RecvContainer recvFrame,
                      UserData      userData);

bool
runWaypointMission(boost::asio::steady_timer* timer,
                   Vehicle*                   vehicle,
                   float                      radius,
                   float                      altitude,
                   int                        numStops,
                   int                        waitTime,
                   int                        responseTimeout)
{
  if (!vehicle->isM210V2() && !vehicle->isM300())
  {
    std::cout << "This waypoint mission is only supported "
              << "on M210V2 and M300." << std::endl;
    return false;
  }

  if (!subscribe(vehicle, responseTimeout))
  {
    std::cout << "Failed to set up subscription!" << std::endl;
    return false;
  }
  sleep(1);

  // init mission
  std::cout << "Initializing waypoint mission..." << std::endl;
  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);

  fdata.indexNumber = numStops;

  ACK::ErrorCode ack = vehicle->missionManager->init(
    DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);

  vehicle->missionManager->wpMission->setWaypointEventCallback(
    &WaypointEventCallBack, vehicle);

  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
  vehicle->missionManager->printInfo();

  std::vector<WayPointSettings> waypoints =
    createWaypoints(vehicle, radius, altitude, numStops, waitTime);

  uploadWaypoints(vehicle, waypoints, responseTimeout);

  // // Optional takeoff
  // if (!isInAir(vehicle))
  // {
  //   bool status = takeOff(vehicle, responseTimeout);
  //   if (!status)
  //   {
  //     return false;
  //   }
  // }

  // metrics no longer needed
  unsubscribe(vehicle, responseTimeout);

  std::cout << "Starting waypoint mission..." << std::endl;
  ack = vehicle->missionManager->wpMission->start(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }

  return true;
}

void
setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  memset(wp->commandList, WP_ACTION_STAY, sizeof(wp->commandList));
  memset(wp->commandParameter, 0, sizeof(wp->commandParameter));
}

void
copyWaypointSettings(WayPointSettings* dst, const WayPointSettings* src)
{
  dst->longitude       = src->longitude;
  dst->latitude        = src->latitude;
  dst->altitude        = src->altitude;
  dst->damping         = src->damping;
  dst->yaw             = src->yaw;
  dst->gimbalPitch     = src->gimbalPitch;
  dst->turnMode        = src->turnMode;
  dst->hasAction       = src->hasAction;
  dst->actionTimeLimit = src->actionTimeLimit;
  dst->actionNumber    = src->actionNumber;
  dst->actionRepeat    = src->actionRepeat;
  memcpy(dst->commandList, src->commandList, sizeof(src->commandList));
  memcpy(dst->commandParameter,
         src->commandParameter,
         sizeof(src->commandParameter));
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
  fdata->maxVelocity    = 10;
  fdata->idleVelocity   = 5;
  fdata->finishAction   = 0;
  fdata->executiveTimes = 1;
  fdata->yawMode        = 0;
  fdata->traceMode      = 0;
  fdata->RCLostAction   = 1;
  fdata->gimbalPitch    = 0;
  fdata->latitude       = 0;
  fdata->longitude      = 0;
  fdata->altitude       = 0;
}

std::vector<WayPointSettings>
createWaypoints(Vehicle* vehicle,
                float    radius,
                float    altitude,
                int      numWaypoints,
                int      waitTime)
{
  std::cout << "Creating waypoints..." << std::endl;
  WayPointSettings centerPoint;
  setWaypointDefaults(&centerPoint);

  Telemetry::TypeMap<TopicName::TOPIC_GPS_FUSED>::type gpsPosition;
  gpsPosition = vehicle->subscribe->getValue<TopicName::TOPIC_GPS_FUSED>();
  centerPoint.longitude           = gpsPosition.longitude;
  centerPoint.latitude            = gpsPosition.latitude;
  centerPoint.altitude            = altitude;
  centerPoint.actionNumber        = 1;
  centerPoint.actionRepeat        = 0;
  centerPoint.commandList[0]      = WP_ACTION_STAY;
  centerPoint.commandParameter[0] = waitTime * 1000;
  printf("Creating %d waypoints around center (LLA): %f\t%f\t%f\n",
         numWaypoints,
         centerPoint.longitude,
         centerPoint.latitude,
         centerPoint.altitude);

  return generateWaypoints(&centerPoint, radius, numWaypoints);
}

std::vector<WayPointSettings>
generateWaypoints(WayPointSettings* centerPoint, float radius, int numWaypoints)
{
  std::cout << "Generating waypoints..." << std::endl;
  std::vector<WayPointSettings> waypoints;
  waypoints.reserve(numWaypoints);

  for (int i = 0; i < numWaypoints; i++)
  {
    WayPointSettings wp =
      newDisplacedWaypoint(centerPoint, radius, 360.0f / numWaypoints * i);
    wp.index = i;

    waypoints.push_back(wp);
  }

  return waypoints;
}

WayPointSettings
newDisplacedWaypoint(WayPointSettings* oldWp, float radius, float angle)
{
  WayPointSettings newWp;
  copyWaypointSettings(&newWp, oldWp);
  float dx = cos(angle) * radius;
  float dy = sin(angle) * radius;
  newWp.latitude += (dx / RADIUS_OF_EARTH_IN_METERS) * (180 / M_PI);
  newWp.longitude += (dy / RADIUS_OF_EARTH_IN_METERS) * (180 / M_PI) /
                     cos(newWp.latitude * M_PI / 180);
  return newWp;
}

void
uploadWaypoints(Vehicle*                       vehicle,
                std::vector<WayPointSettings>& waypoints,
                int                            responseTimeout)
{
  std::cout << "Uploading waypoints..." << std::endl;
  for (auto waypoint : waypoints)
  {
    printf("Uploading waypoint %d at (LLA): %f\t%f\t%f\n",
           waypoint.index,
           waypoint.longitude,
           waypoint.latitude,
           waypoint.altitude);
    ACK::WayPointIndex wpIndexACK =
      vehicle->missionManager->wpMission->uploadIndexData(&waypoint,
                                                          responseTimeout);
    ACK::getErrorCodeMessage(wpIndexACK.ack, __func__);
  }
}

bool
isInAir(Vehicle* vehicle)
{
  return vehicle->subscribe->getValue<TopicName::TOPIC_STATUS_FLIGHT>() == 2;
}

bool
takeOff(Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Taking off..." << std::endl;
  ErrorCode::ErrorCodeType err =
    vehicle->flightController->startTakeoffSync(responseTimeout);
  if (err != ErrorCode::SysCommonErr::Success)
  {
    ErrorCode::getErrorCodeMsg(err);
    return false;
  }
  return waitTakeoffFinished(vehicle);
}

bool
waitTakeoffFinished(Vehicle* vehicle)
{
  TypeMap<TopicName::TOPIC_STATUS_DISPLAYMODE>::type displayMode =
    vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
  while (displayMode == VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
         displayMode == VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
  {
    sleep(1);
    displayMode = vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();
  }
  return (displayMode == VehicleStatus::DisplayMode::MODE_P_GPS ||
          displayMode == VehicleStatus::DisplayMode::MODE_ATTITUDE)
           ? true
           : false;
}

void
WaypointEventCallBack(Vehicle*      vehicle,
                      RecvContainer recvFrame,
                      UserData      userData)
{
  DSTATUS("%s", __func__);
  DSTATUS("Reached waypoint %d.\n",
          recvFrame.recvData.wayPointReachedData.waypoint_index);
  DSTATUS("Current status is %d.\n",
          recvFrame.recvData.wayPointReachedData.current_status);
  DSTATUS("Incident type is %d.\n",
          recvFrame.recvData.wayPointReachedData.incident_type);
}

bool
subscribe(Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Subscribing to topics..." << std::endl;
  ACK::ErrorCode status;
  status = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(status, __func__);
    return false;
  }

  int       freq            = 10;
  TopicName topicList[]     = { TopicName::TOPIC_GPS_FUSED,
                                TopicName::TOPIC_STATUS_FLIGHT,
                                TopicName::TOPIC_STATUS_DISPLAYMODE };
  int       numTopics       = sizeof(topicList) / sizeof(topicList[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    1, numTopics, topicList, enableTimestamp, freq);
  if (!pkgStatus)
  {
    return false;
  }

  status = vehicle->subscribe->startPackage(1, responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(status, __func__);
    unsubscribe(vehicle, responseTimeout);
    return false;
  }
  return true;
}

bool
unsubscribe(Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Unsubscribing from topics..." << std::endl;
  ACK::ErrorCode status;
  status = vehicle->subscribe->removePackage(1, responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    std::cout << "Error unsubscribing; please restart the drone/FC to get "
                 "back to a clean state.\n";
    return false;
  }
  return true;
}
}