
#include "influx-metrics.hpp"
#include <dji_telemetry.hpp>
#include <signal.h>
#include <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static std::unique_ptr<influxdb::InfluxDB>
connectInflux(const std::string& host,
              const std::string& port,
              const std::string& database)
{
  const std::string url = "http://" + host + ":" + port + "?db=" + database;
  return influxdb::InfluxDBFactory::Get(url);
}

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
  int       freq        = 10; // Hz
  TopicName topicList[] = {
    TopicName::TOPIC_VELOCITY,
    TopicName::TOPIC_GPS_POSITION,
    TopicName::TOPIC_GPS_VELOCITY,
    TopicName::TOPIC_GPS_FUSED,
    TopicName::TOPIC_RTK_CONNECT_STATUS,
    TopicName::TOPIC_RTK_POSITION,
    TopicName::TOPIC_RTK_VELOCITY,
    TopicName::TOPIC_RTK_POSITION_INFO,
    TopicName::TOPIC_HEIGHT_FUSION,
    TopicName::TOPIC_ALTITUDE_FUSIONED,
    TopicName::TOPIC_ALTITUDE_OF_HOMEPOINT,
  };
  int  numTopics       = sizeof(topicList) / sizeof(topicList[0]);
  bool enableTimestamp = true;

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    pkgIndex, numTopics, topicList, enableTimestamp, freq);

  std::cout << "Subscribed to topics" << std::endl;

  if (!pkgStatus)
  {
    std::cout << "Subscription failed" << std::endl;
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

  TypeMap<TopicName::TOPIC_VELOCITY>::type              velocity;
  TypeMap<TopicName::TOPIC_GPS_POSITION>::type          gpsPosition;
  TypeMap<TopicName::TOPIC_GPS_VELOCITY>::type          gpsVelocity;
  TypeMap<TopicName::TOPIC_GPS_FUSED>::type             gpsFused;
  TypeMap<TopicName::TOPIC_RTK_CONNECT_STATUS>::type    rtkConnectStatus;
  TypeMap<TopicName::TOPIC_RTK_POSITION>::type          rtkPosition;
  TypeMap<TopicName::TOPIC_RTK_VELOCITY>::type          rtkVelocity;
  TypeMap<TopicName::TOPIC_RTK_POSITION_INFO>::type     rtkPositionInfo;
  TypeMap<TopicName::TOPIC_HEIGHT_FUSION>::type         heightFusion;
  TypeMap<TopicName::TOPIC_ALTITUDE_FUSIONED>::type     altitudeFusioned;
  TypeMap<TopicName::TOPIC_ALTITUDE_OF_HOMEPOINT>::type altitudeOfHomepoint;

  while (true)
  {
    velocity = vehiclePtr->subscribe->getValue<TopicName::TOPIC_VELOCITY>();
    gpsPosition =
      vehiclePtr->subscribe->getValue<TopicName::TOPIC_GPS_POSITION>();
    gpsVelocity =
      vehiclePtr->subscribe->getValue<TopicName::TOPIC_GPS_VELOCITY>();
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

    influxDB->write(
      influxdb::Point{ "drone_metrics" }
        .addField("velocity_x", velocity.data.x)
        .addField("velocity_y", velocity.data.y)
        .addField("velocity_z", velocity.data.z)
        .addField("gps_position_x", gpsPosition.x)
        .addField("gps_position_y", gpsPosition.y)
        .addField("gps_position_z", gpsPosition.z)
        .addField("gps_lat", gpsFused.latitude)
        .addField("gps_lon", gpsFused.longitude)
        .addField("gps_alt", gpsFused.altitude)
        .addField("rtk_connect_status", rtkConnectStatus.rtkConnected)
        .addField("rtk_lat", rtkPosition.latitude)
        .addField("rtk_lon", rtkPosition.longitude)
        .addField("rtk_hfsl", rtkPosition.HFSL)
        .addField("rtk_velocity_x", rtkVelocity.x)
        .addField("rtk_velocity_y", rtkVelocity.y)
        .addField("rtk_velocity_z", rtkVelocity.z)
        .addField("rtk_position_info", rtkPositionInfo)
        .addField("height_fusion", heightFusion)
        .addField("altitude_fusion", altitudeFusioned)
        .addField("altitude_of_homepoint", altitudeOfHomepoint));

    if (quit)
    {
      std::cout << "Ctrl-C pressed, quit loop" << std::endl;
      influxDB->flushBatch();
      break;
    }
    usleep(1e6 / freq);
  }
  std::cout << "Done!\n";

  return true;
}

void
INThandler(int sig)
{
  quit = true;
}