
#include "influx-metrics.hpp"
#include <dji_telemetry.hpp>
#include <signal.h>
#include <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void
            INThandler(int);
static bool quit = false;

bool
subscribeAndWriteToInflux(DJI::OSDK::Vehicle* vehiclePtr,
                          influxdb::InfluxDB* influxDB,
                          int                 responseTimeout)
{
  signal(SIGINT, INThandler);
  // put influxDB into batch mode
  influxDB->batchOf(50);

  // subscribe to vehicle telemetry
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehiclePtr->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  int       pkgIndex    = 0;
  int       freq        = 1; // Hz
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

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    pkgIndex, numTopics, topicList, enableTimestamp, freq);

  DDEBUG("subscribe status: %d", subscribeStatus);
  std::cout << "Subscribing to topics..." << std::endl;

  if (!(pkgStatus))
  {
    DERROR("Subscription failed");
    return false;
  }

  subscribeStatus =
    vehiclePtr->subscribe->startPackage(pkgIndex, responseTimeout);

  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    vehiclePtr->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  sleep(1);
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
  while (true)
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

    influxDB->write(
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
        .addTag("hostname", std::getenv("HOST")));
    if (quit)
    {
      std::cout << std::endl << "Ctrl-C pressed, quit loop" << std::endl;
      influxDB->flushBatch();
      break;
    }
    usleep(1e6 / freq);
    std::cout << "." << std::flush;
  }
  std::cout << "Done!" << std::endl;

  return true;
}

void
INThandler(int sig)
{
  quit = true;
}