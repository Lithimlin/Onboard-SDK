#include "metrics-mission.hpp"
#include "dotenv.h"

#include <boost/bind/bind.hpp>

#include <dji_telemetry.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

const float RADIUS_OF_EARTH_IN_METERS = 6371000.0f;
const float METERS_PER_DEGREE = RADIUS_OF_EARTH_IN_METERS * M_PI / 180.0f;

const float LAT_QUOTIENT =
  56.0f; // magic number that is needed for correct conversion.
const float LON_QUOTIENT =
  41.0f; // magic number that is needed for correct conversion.

int       metricsFreq = 1; // Hz
TopicName topicList[] = {
  TopicName::TOPIC_VELOCITY,
  TopicName::TOPIC_GPS_FUSED,
  TopicName::TOPIC_RTK_CONNECT_STATUS,
  TopicName::TOPIC_RTK_POSITION,
  TopicName::TOPIC_RTK_VELOCITY,
  TopicName::TOPIC_RTK_POSITION_INFO,
  TopicName::TOPIC_HEIGHT_FUSION,
  TopicName::TOPIC_ALTITUDE_FUSIONED,
  TopicName::TOPIC_ALTITUDE_OF_HOMEPOINT,
  TopicName::TOPIC_STATUS_FLIGHT,
};
int  numTopics       = sizeof(topicList) / sizeof(topicList[0]);
bool enableTimestamp = true;

// subscribed variables
// clang-format off
TypeMap<TopicName::TOPIC_VELOCITY>::type                velocity; // in m/s
TypeMap<TopicName::TOPIC_GPS_FUSED>::type               gpsFused; // in rad (Lat,Lon), m (Alt)
TypeMap<TopicName::TOPIC_RTK_CONNECT_STATUS>::type      rtkConnectStatus; // bool
TypeMap<TopicName::TOPIC_RTK_POSITION>::type            rtkPosition; // in deg (x, y), m(z)
TypeMap<TopicName::TOPIC_RTK_VELOCITY>::type            rtkVelocity; // in cm/s
TypeMap<TopicName::TOPIC_RTK_POSITION_INFO>::type       rtkPositionInfo; // enum, see
                    // https://developer.dji.com/onboard-api-reference/group__telem.html
TypeMap<TopicName::TOPIC_HEIGHT_FUSION>::type           heightFusion; // in m, estimate of current height from ground
TypeMap<TopicName::TOPIC_ALTITUDE_FUSIONED>::type       altitudeFusioned; // in m
TypeMap<TopicName::TOPIC_ALTITUDE_OF_HOMEPOINT>::type   altitudeOfHomepoint; // in m
TypeMap<TopicName::TOPIC_STATUS_FLIGHT>::type           statusFlight; // 0: stopped, 1: on ground, 2: in air
// clang-format on

void
waypointEventCallback(Vehicle*      vehiclePtr,
                      RecvContainer recvFrame,
                      UserData      userData);

void
commitMetricsTimerCallback(const boost::system::error_code& ec,
                           MetricsMission*                  ref);

MetricsMission::MetricsMission(Vehicle*            vehiclePtr,
                               influxdb::InfluxDB* influxDBPtr,
                               PointType           missionType,
                               int                 responseTimeout)
  : vehiclePtr(vehiclePtr)
  , influxDBPtr(influxDBPtr)
  , missionType(missionType)
  , responseTimeout(responseTimeout)
  , metricsTimer(ctx, boost::asio::chrono::seconds(1))
{
  if (!this->influxDBPtr)
  {
    std::cerr << "InfluxDB pointer is null" << std::endl;
    exit(-1);
  }
  this->influxDBPtr->batchOf(50);

  if (!subscribe())
  {
    std::cerr << "Failed to subscribe in MetricsMission..." << std::endl;
    exit(-1);
  }
  sleep(1);

  std::cout << "Recording position..." << std::endl;
  centerPoint = getCurrentPoint();
  std::cout << "Center point is (" << centerPoint.latitude << ", "
            << centerPoint.longitude << ")" << std::endl;

  std::cout << "Starting metrics timer..." << std::endl;
  metricsTimer.async_wait(boost::bind(
    commitMetricsTimerCallback, boost::asio::placeholders::error, this));

  std::cout << "MetricsMission initialized..." << std::endl;
}

MetricsMission::~MetricsMission()
{
  unsubscribe();
  std::cout << "Cancelling metrics timer..." << std::endl;
  metricsTimer.expires_after(boost::asio::chrono::milliseconds(50));
  std::cout << "Cancelling mission..." << std::endl;
  stopMission();
}

bool
MetricsMission::runMission(MissionConfig* mission)
{
  if (!vehiclePtr->isM210V2() && !vehiclePtr->isM300())
  {
    std::cout << "This metrics mission is only supported "
              << "on M210V2 and M300." << std::endl;
    return false;
  }

  switch (missionType)
  {
    case PointType::waypoint:
      return runWaypointMission(mission);
    case PointType::hotpoint:
      return runHotpointMission(mission);
    default:
      return false;
  }
}

void
MetricsMission::runContext()
{
  ctx.run();
}

bool
MetricsMission::flyToCenter(float altitude)
{
  MissionConfig mission = { altitude, 0.0f, 1, 0 };
  return runWaypointMission(&mission);
}

void
MetricsMission::commitMetrics()
{
  velocity = vehiclePtr->subscribe->getValue<TopicName::TOPIC_VELOCITY>();
  gpsFused = vehiclePtr->subscribe->getValue<TopicName::TOPIC_GPS_FUSED>();
  rtkConnectStatus =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_CONNECT_STATUS>();
  rtkPosition =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_POSITION>();
  rtkVelocity =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_VELOCITY>();
  rtkPositionInfo =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_POSITION_INFO>();
  heightFusion =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_HEIGHT_FUSION>();
  altitudeFusioned =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_ALTITUDE_FUSIONED>();
  altitudeOfHomepoint =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_ALTITUDE_OF_HOMEPOINT>();
  statusFlight =
    vehiclePtr->subscribe->getValue<TopicName::TOPIC_STATUS_FLIGHT>();

  std::string hostname = dotenv::env["HOST"];

  influxDBPtr->write(
    influxdb::Point{ "drone_metrics" }
      .addField("velocity_x", velocity.data.x)
      .addField("velocity_y", velocity.data.y)
      .addField("velocity_z", velocity.data.z)
      .addField("gps_lat", gpsFused.latitude)
      .addField("gps_lon", gpsFused.longitude)
      .addField("gps_alt", gpsFused.altitude)
      .addField("rtk_connect_status", (uint8_t)rtkConnectStatus.rtkConnected)
      .addField("rtk_lat", rtkPosition.latitude)
      .addField("rtk_lon", rtkPosition.longitude)
      .addField("rtk_height_from_sea", rtkPosition.HFSL)
      .addField("rtk_velocity_x", rtkVelocity.x)
      .addField("rtk_velocity_y", rtkVelocity.y)
      .addField("rtk_velocity_z", rtkVelocity.z)
      .addField("rtk_position_info", rtkPositionInfo)
      .addField("height_fusion", heightFusion)
      .addField("altitude_fusion", altitudeFusioned)
      .addField("altitude_of_homepoint", altitudeOfHomepoint)
      .addField("status_flight", statusFlight)
      .addField("mission_status", (uint8_t)missionStatus)
      .addField("mission_type", (uint8_t)missionType)
      .addTag("hostname", hostname));

  std::cout << "Committing" << std::endl;
}

void
MetricsMission::flushMetrics()
{
  influxDBPtr->flushBatch();
}

WayPointSettings
MetricsMission::getCurrentPoint()
{
  WayPointSettings point;
  setWaypointDefaults(&point);

  if (vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_CONNECT_STATUS>()
        .rtkConnected)
  {
    std::cout << "Getting RTK position..." << std::endl;
    rtkPosition =
      vehiclePtr->subscribe->getValue<TopicName::TOPIC_RTK_POSITION>();
    point.latitude  = rtkPosition.latitude;
    point.longitude = rtkPosition.longitude;
  }
  else
  {
    std::cout << "Getting GPS position..." << std::endl;
    gpsFused = vehiclePtr->subscribe->getValue<TopicName::TOPIC_GPS_FUSED>();
    point.latitude  = gpsFused.latitude;
    point.longitude = gpsFused.longitude;
  }

  point.altitude            = 0;
  point.hasAction           = 1;
  point.actionNumber        = 1;
  point.actionRepeat        = 4;
  point.commandList[0]      = WP_ACTION_STAY;
  point.commandParameter[0] = 0 * 250;

  return point;
}

bool
MetricsMission::runWaypointMission(MissionConfig* mission)
{
  // init mission
  std::cout << "Initializing waypoint mission..." << std::endl;
  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);
  fdata.indexNumber = mission->numStops;

  ACK::ErrorCode ack = vehiclePtr->missionManager->init(
    DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);

  vehiclePtr->missionManager->wpMission->setWaypointEventCallback(
    waypointEventCallback, this);

  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
  vehiclePtr->missionManager->printInfo();

  std::vector<WayPointSettings> waypoints = createWaypoints(mission);

  uploadWaypoints(waypoints);

  std::cout << "Starting waypoint mission..." << std::endl;
  ack = vehiclePtr->missionManager->wpMission->start(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }

  return true;
}

bool
MetricsMission::runHotpointMission(MissionConfig* mission)
{
  // init mission
  bool status = initHotpointMission(mission);
  if (!status)
  {
    return false;
  }
  // Optional takeoff
  if (!isInAir())
  {
    status = takeOff();
    if (!status)
    {
      return false;
    }
  }

  std::cout << "Starting hotpoint mission..." << std::endl;
  ACK::ErrorCode ack =
    vehiclePtr->missionManager->hpMission->start(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  missionStatus = MissionStatus::enRoute;

  return true;
}

bool
MetricsMission::stopMission()
{
  switch (missionType)
  {
    case PointType::waypoint:
      return stopWaypointMission();
    case PointType::hotpoint:
      return stopHotpointMission();
    default:
      return false;
  }
  return false;
}

bool
MetricsMission::stopWaypointMission()
{
  std::cout << "Stopping waypoint mission..." << std::endl;
  vehiclePtr->missionManager->wpMission->stop(responseTimeout);
  missionStatus = MissionStatus::completed;
  return true;
}

bool
MetricsMission::stopHotpointMission()
{
  std::cout << "Stopping hotpoint mission..." << std::endl;
  vehiclePtr->missionManager->hpMission->stop(responseTimeout);
  missionStatus = MissionStatus::completed;
  return true;
}

bool
MetricsMission::subscribe()
{
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehiclePtr->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }
  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    PACKAGE_ID, numTopics, topicList, enableTimestamp, metricsFreq);

  std::cout << "Subscribing to topics..." << std::endl;

  if (!(pkgStatus))
  {
    DERROR("Subscription failed");
    return false;
  }

  subscribeStatus =
    vehiclePtr->subscribe->startPackage(PACKAGE_ID, responseTimeout);

  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    vehiclePtr->subscribe->removePackage(PACKAGE_ID, responseTimeout);
    return false;
  }
  return true;
}

bool
MetricsMission::unsubscribe()
{
  std::cout << "Unsubscribing from topics..." << std::endl;
  ACK::ErrorCode status;
  status = vehiclePtr->subscribe->removePackage(PACKAGE_ID, responseTimeout);
  if (ACK::getError(status) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(status, __func__);
    std::cout << "Error unsubscribing; please restart the drone/FC to get "
                 "back to a clean state.\n";
    return false;
  }
  return true;
}

void
MetricsMission::setWaypointDefaults(WayPointSettings* wp)
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
MetricsMission::copyWaypointSettings(WayPointSettings*       dst,
                                     const WayPointSettings* src)
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
MetricsMission::setWaypointInitDefaults(WayPointInitSettings* fdata)
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
MetricsMission::createWaypoints(MissionConfig* mission)
{
  std::cout << "Generating waypoints..." << std::endl;
  std::vector<WayPointSettings> waypoints;
  waypoints.reserve(mission->numStops);

  float angleIncrement = 360.0f / mission->numStops;

  for (size_t i = 0; i < mission->numStops; ++i)
  {
    WayPointSettings wp =
      newDisplacedWaypoint(&centerPoint, mission->radius, angleIncrement * i);
    wp.index               = i;
    wp.altitude            = mission->altitude;
    wp.commandParameter[0] = mission->waitTime * 250;

    waypoints.push_back(wp);
  }
  return waypoints;
}

WayPointSettings
MetricsMission::newDisplacedWaypoint(WayPointSettings* oldWp,
                                     float             radius,
                                     float             angle)
{
  WayPointSettings newWp;
  copyWaypointSettings(&newWp, oldWp);
  float dx = cos(angle * M_PI / 180) * radius;
  float dy = sin(angle * M_PI / 180) * radius;

  newWp.latitude += dx / METERS_PER_DEGREE / LAT_QUOTIENT;
  newWp.longitude +=
    dy / METERS_PER_DEGREE / LON_QUOTIENT / cos(newWp.latitude * M_PI / 180);
  return newWp;
}

void
MetricsMission::uploadWaypoints(std::vector<WayPointSettings>& waypoints)
{
  std::cout << "Uploading waypoints..." << std::endl;
  for (auto waypoint : waypoints)
  {
    ACK::WayPointIndex wpIndexACK =
      vehiclePtr->missionManager->wpMission->uploadIndexData(&waypoint,
                                                             responseTimeout);
    if (ACK::getError(wpIndexACK.ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(wpIndexACK.ack, __func__);
    }
  }
}

bool
MetricsMission::initHotpointMission(MissionConfig* mission)
{
  vehiclePtr->missionManager->init(
    DJI_MISSION_TYPE::HOTPOINT, responseTimeout, NULL);
  vehiclePtr->missionManager->printInfo();

  vehiclePtr->missionManager->hpMission->setHotPoint(
    centerPoint.longitude, centerPoint.latitude, mission->altitude);

  vehiclePtr->missionManager->hpMission->setRadius(mission->radius);
  vehiclePtr->missionManager->hpMission->setYawMode(
    HotpointMission::YawMode::YAW_INSIDE);
  vehiclePtr->missionManager->hpMission->setYawRate(15.0f); // deg/s

  return true;
}

bool
MetricsMission::isInAir()
{
  return vehiclePtr->subscribe->getValue<TopicName::TOPIC_STATUS_FLIGHT>() == 2;
}

bool
MetricsMission::takeOff()
{
  std::cout << "Taking off..." << std::endl;
  ACK::ErrorCode ack = vehiclePtr->control->takeoff(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);

    if (ack.info.cmd_set == OpenProtocolCMD::CMDSet::control &&
        ack.data == ErrorCode::CommonACK::START_MOTOR_FAIL_MOTOR_STARTED)
    {
      DSTATUS("Take off command sent failed. Please Land the drone and disarm "
              "the motors first.\n");
    }
    return false;
  }
  sleep(10);
  return true;
}

void
waypointEventCallback(Vehicle*      vehiclePtr,
                      RecvContainer recvFrame,
                      UserData      userData)
{
  if (recvFrame.recvData.wayPointReachedData.incident_type ==
        WayPointIncidentType::NAVI_MISSION_FINISH &&
      recvFrame.recvData.wayPointReachedData.current_status == 0)
  {
    DSTATUS("Mission finished.");
    ((MetricsMission*)userData)->missionStatus = MissionStatus::completed;
    return;
  }

  if (recvFrame.recvData.wayPointReachedData.incident_type !=
      WayPointIncidentType::NAVI_MISSION_WP_REACH_POINT)
  {
    return;
  }

  if (recvFrame.recvData.wayPointReachedData.current_status == 4)
  {
    DSTATUS("Waiting at waypoint %d",
            recvFrame.recvData.wayPointReachedData.waypoint_index);
    ((MetricsMission*)userData)->missionStatus = MissionStatus::waiting;
    return;
  }
  else if (recvFrame.recvData.wayPointReachedData.current_status == 6)
  {
    DSTATUS("Departing from waypoint %d",
            recvFrame.recvData.wayPointReachedData.waypoint_index);
    ((MetricsMission*)userData)->missionStatus = MissionStatus::enRoute;
    return;
  }
}

void
commitMetricsTimerCallback(const boost::system::error_code& ec,
                           MetricsMission*                  ref)
{
  ref->commitMetrics();

  if (ec == boost::asio::error::operation_aborted)
  {
    std::cout << "\nCtrl+C pressed, quit metrics loop" << std::endl;
    ref->flushMetrics();
    return;
  }

  ref->metricsTimer.expires_at(
    ref->metricsTimer.expiry() +
    boost::asio::chrono::milliseconds((int)1e3 / metricsFreq));

  ref->metricsTimer.async_wait(boost::bind(
    commitMetricsTimerCallback, boost::asio::placeholders::error, ref));
}

std::string
MissionConfig::toString() const
{
  std::stringstream ss;
  ss << "Altitude: " << to_string(altitude) << ", Radius: " << to_string(radius)
     << ", NumStops: " << to_string(numStops)
     << ", WaitTime: " << to_string(waitTime);
  return ss.str();
}

std::ostream&
operator<<(std::ostream& o, const MissionConfig& mission)
{
  o << mission.toString();
  return o;
}
