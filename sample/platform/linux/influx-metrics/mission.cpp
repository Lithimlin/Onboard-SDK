#include "mission.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <dji_telemetry.hpp>
#include <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace mission
{

Telemetry::TypeMap<TopicName::TOPIC_GPS_FUSED>::type gpsPosition;
static bool                                          startPositionSetup = false;
static const float                                   yawRate = 15.0f; // deg/sec

bool
subscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout);
bool
unsubscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout);

bool
initHotpointMission(DJI::OSDK::Vehicle* vehicle,
                    float               radius,
                    float               altitude,
                    int                 responseTimeout);

bool
isInAir(DJI::OSDK::Vehicle* vehicle);

bool
takeOff(DJI::OSDK::Vehicle* vehicle, int responseTimeout);

bool
pauseHotpointMission(const boost::system::error_code& ec,
                     boost::asio::steady_timer*       timer,
                     DJI::OSDK::Vehicle*              vehicle,
                     int                              stopIndex,
                     int                              numStops,
                     int                              waitTime,
                     int                              responseTimeout);

bool
resumeHotpointMission(const boost::system::error_code& ec,
                      boost::asio::steady_timer*       timer,
                      DJI::OSDK::Vehicle*              vehicle,
                      int                              stopIndex,
                      int                              numStops,
                      int                              waitTime,
                      int                              responseTimeout);

bool
stopHotpointMission(DJI::OSDK::Vehicle* vehicle, int responseTimeout);

bool
runHotpointMission(boost::asio::steady_timer* timer,
                   DJI::OSDK::Vehicle*        vehicle,
                   float                      radius,   // in meters
                   float                      altitude, // in meters
                   int                        numStops, // in a full circle
                   int                        waitTime, // in seconds
                   int                        responseTimeout)
{
  if (!vehicle->isM210V2() && !vehicle->isM300())
  {
    std::cout << "This hotpoint mission is only supported "
              << "on M210V2 and M300." << std::endl;
    return false;
  }
  // init mission
  bool status = initHotpointMission(vehicle, radius, altitude, responseTimeout);
  if (!status)
  {
    return false;
  }
  // Optional takeoff
  if (!isInAir(vehicle))
  {
    status = takeOff(vehicle, responseTimeout);
    if (!status)
    {
      return false;
    }
  }
  // metrics no longer needed
  unsubscribe(vehicle, responseTimeout);

  std::cout << "Starting hotpoint mission..." << std::endl;
  ACK::ErrorCode ack =
    vehicle->missionManager->hpMission->start(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  float angleIncrements = 360.0f / numStops;
  timer->expires_after(boost::asio::chrono::milliseconds(
    (int)(angleIncrements * 1000 / yawRate) + 3000));

  timer->async_wait(boost::bind(pauseHotpointMission,
                                boost::asio::placeholders::error,
                                timer,
                                vehicle,
                                0,
                                numStops,
                                waitTime,
                                responseTimeout));

  return true;
}

bool
initHotpointMission(DJI::OSDK::Vehicle* vehicle,
                    float               radius,
                    float               altitude,
                    int                 responseTimeout)
{
  std::cout << "Initializing hotpoint mission..." << std::endl;
  if (!subscribe(vehicle, responseTimeout))
  {
    std::cout << "Failed to set up subscription!" << std::endl;
    return false;
  }
  sleep(1);

  vehicle->missionManager->init(
    DJI_MISSION_TYPE::HOTPOINT, responseTimeout, NULL);
  vehicle->missionManager->printInfo();

  gpsPosition = vehicle->subscribe->getValue<TopicName::TOPIC_GPS_FUSED>();
  vehicle->missionManager->hpMission->setHotPoint(
    gpsPosition.longitude, gpsPosition.latitude, altitude);

  vehicle->missionManager->hpMission->setRadius(radius);
  vehicle->missionManager->hpMission->setYawRate(yawRate);
  vehicle->missionManager->hpMission->setClockwise(false);

  return true;
}

bool
isInAir(DJI::OSDK::Vehicle* vehicle)
{
  return vehicle->subscribe->getValue<TopicName::TOPIC_STATUS_FLIGHT>() == 2;
}

bool
takeOff(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Taking off..." << std::endl;
  ACK::ErrorCode ack = vehicle->control->takeoff(responseTimeout);
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

bool
pauseHotpointMission(const boost::system::error_code& ec,
                     boost::asio::steady_timer*       timer,
                     DJI::OSDK::Vehicle*              vehicle,
                     int                              stopIndex,
                     int                              numStops,
                     int                              waitTime,
                     int                              responseTimeout)
{
  if (++stopIndex == numStops || ec == boost::asio::error::operation_aborted)
  {
    stopHotpointMission(vehicle, responseTimeout);
    return true;
  }

  std::cout << "Pausing hotpoint mission..." << std::endl;
  ACK::ErrorCode ack =
    vehicle->missionManager->hpMission->pause(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    // return false;
  }

  timer->expires_after(boost::asio::chrono::seconds(waitTime));
  timer->async_wait(boost::bind(resumeHotpointMission,
                                boost::asio::placeholders::error,
                                timer,
                                vehicle,
                                stopIndex,
                                numStops,
                                waitTime,
                                responseTimeout));
  return true;
}
// target: 4
// start (0), pause (0->1), resume (1), pause (1->2), resume (2), pause (2->2),
// resume (3), pause (3->4)
bool
resumeHotpointMission(const boost::system::error_code& ec,
                      boost::asio::steady_timer*       timer,
                      DJI::OSDK::Vehicle*              vehicle,
                      int                              stopIndex,
                      int                              numStops,
                      int                              waitTime,
                      int                              responseTimeout)
{
  std::cout << "Resuming hotpoint mission..." << std::endl;
  if (ec == boost::asio::error::operation_aborted)
  {
    stopHotpointMission(vehicle, responseTimeout);
    return true;
  }
  ACK::ErrorCode ack =
    vehicle->missionManager->hpMission->resume(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    // return false;
  }
  float angleIncrements = 360.0f / numStops;

  timer->expires_after(
    boost::asio::chrono::milliseconds((int)(angleIncrements * 1000 / yawRate)));
  timer->async_wait(boost::bind(pauseHotpointMission,
                                boost::asio::placeholders::error,
                                timer,
                                vehicle,
                                stopIndex,
                                numStops,
                                waitTime,
                                responseTimeout));
  return true;
}

bool
stopHotpointMission(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Stopping hotpoint mission..." << std::endl;
  ACK::ErrorCode ack =
    vehicle->missionManager->hpMission->stop(responseTimeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  return true;
}

bool
subscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
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
                                TopicName::TOPIC_STATUS_FLIGHT };
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
unsubscribe(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
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