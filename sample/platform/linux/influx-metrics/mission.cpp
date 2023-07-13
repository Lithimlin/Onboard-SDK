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
static const float RADIUS_OF_EARTH_IN_METERS = 6371000.0f;
static const float METERS_PER_DEGREE =
  RADIUS_OF_EARTH_IN_METERS * M_PI / 180.0f;

bool
subscribe(Vehicle* vehiclePtr, int responseTimeout);
bool
unsubscribe(Vehicle* vehiclePtr, int responseTimeout);

void
setWaypointDefaults(WayPointSettings* wp);

void
copyWaypointSettings(WayPointSettings* dst, const WayPointSettings* src);

void
setWaypointInitDefaults(WayPointInitSettings* fdata);

std::vector<WayPointSettings>
createWaypoints(Vehicle* vehiclePtr,
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
uploadWaypoints(Vehicle*                       vehiclePtr,
                std::vector<WayPointSettings>& waypoints,
                int                            responseTimeout);

static void
waypointReachedCallback(Vehicle*      vehiclePtr,
                        RecvContainer recvFrame,
                        UserData      userData);

/**********************************************************************/

bool
runWaypointMission(boost::asio::steady_timer* timer,
                   Vehicle*                   vehiclePtr,
                   float                      radius,
                   float                      altitude,
                   int                        numStops,
                   int                        waitTime,
                   int                        responseTimeout)
{
  if (!vehiclePtr->isM210V2() && !vehiclePtr->isM300())
  {
    std::cout << "This waypoint mission is only supported "
              << "on M210V2 and M300." << std::endl;
    return false;
  }

  if (!subscribe(vehiclePtr, responseTimeout))
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

  ACK::ErrorCode ack = vehiclePtr->missionManager->init(
    DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);

  vehiclePtr->missionManager->wpMission->setWaypointEventCallback(
    &waypointReachedCallback, vehiclePtr);

  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
  vehiclePtr->missionManager->printInfo();

  std::vector<WayPointSettings> waypoints =
    createWaypoints(vehiclePtr, radius, altitude, numStops, waitTime);

  uploadWaypoints(vehiclePtr, waypoints, responseTimeout);

  // metrics no longer needed
  unsubscribe(vehiclePtr, responseTimeout);

  std::cout << "Starting waypoint mission..." << std::endl;
  ack = vehiclePtr->missionManager->wpMission->start(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }

  return true;
}

/*****************************************************************************/

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
createWaypoints(Vehicle* vehiclePtr,
                float    radius,
                float    altitude,
                int      numWaypoints,
                int      waitTime)
{
  std::cout << "Creating waypoints..." << std::endl;
  WayPointSettings centerPoint;
  setWaypointDefaults(&centerPoint);

  Telemetry::TypeMap<TopicName::TOPIC_GPS_FUSED>::type gpsPosition;
  gpsPosition = vehiclePtr->subscribe->getValue<TopicName::TOPIC_GPS_FUSED>();
  centerPoint.longitude           = gpsPosition.longitude;
  centerPoint.latitude            = gpsPosition.latitude;
  centerPoint.altitude            = altitude;
  centerPoint.hasAction           = 1;
  centerPoint.actionNumber        = 1;
  centerPoint.actionRepeat        = 1;
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
  float dx = cos(angle * M_PI / 180) * radius;
  float dy = sin(angle * M_PI / 180) * radius;
  printf("Displacing center by (%f, %f)\n", dx, dy);
  newWp.latitude += dx / METERS_PER_DEGREE / 56;
  newWp.longitude +=
    dy / METERS_PER_DEGREE / 41 / cos(newWp.latitude * M_PI / 180);
  return newWp;
}

void
uploadWaypoints(Vehicle*                       vehiclePtr,
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
      vehiclePtr->missionManager->wpMission->uploadIndexData(&waypoint,
                                                             responseTimeout);
    ACK::getErrorCodeMessage(wpIndexACK.ack, __func__);
  }
}

void
waypointReachedCallback(Vehicle*      vehiclePtr,
                        RecvContainer recvFrame,
                        UserData      userData)
{
  if (recvFrame.recvData.wayPointReachedData.incident_type ==
        WayPointIncidentType::NAVI_MISSION_FINISH &&
      recvFrame.recvData.wayPointReachedData.current_status == 0)
  {
    DSTATUS("Mission finished.");
    return;
  }

  DSTATUS("Reached waypoint %d.",
          recvFrame.recvData.wayPointReachedData.waypoint_index);
  DSTATUS("Current status is %d.",
          recvFrame.recvData.wayPointReachedData.current_status);
  DSTATUS("Incident type is %d.\n",
          recvFrame.recvData.wayPointReachedData.incident_type);
}

bool
subscribe(Vehicle* vehiclePtr, int responseTimeout)
{
  std::cout << "Subscribing to topics..." << std::endl;
  ACK::ErrorCode status;
  status = vehiclePtr->subscribe->verify(responseTimeout);
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

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    1, numTopics, topicList, enableTimestamp, freq);
  if (!pkgStatus)
  {
    return false;
  }

  status = vehiclePtr->subscribe->startPackage(1, responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(status, __func__);
    unsubscribe(vehiclePtr, responseTimeout);
    return false;
  }
  return true;
}

bool
unsubscribe(Vehicle* vehiclePtr, int responseTimeout)
{
  std::cout << "Unsubscribing from topics..." << std::endl;
  ACK::ErrorCode status;
  status = vehiclePtr->subscribe->removePackage(1, responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    std::cout << "Error unsubscribing; please restart the drone/FC to get "
                 "back to a clean state.\n";
    return false;
  }
  return true;
}
}