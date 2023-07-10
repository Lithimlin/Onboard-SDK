
#include "influx-metrics.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <dji_telemetry.hpp>
#include <stdlib.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

using boost::asio::steady_timer;

static bool         quitFlag = false;
static steady_timer timer;

// subscription settings
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
getMetricsAndWrite(const boost::system::error_code& e,
                   steady_timer*                    timer,
                   DJI::OSDK::Vehicle*              vehiclePtr,
                   influxdb::InfluxDB*              influxDB)
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

  if (quitFlag)
  {
    std::cout << "\nCtrl-C pressed, quit loop" << std::endl;
    influxDB->flushBatch();
    return;
  }

  std::cout << "." << std::flush;

  timer->expires_at(timer->expiry() +
                    boost::asio::chrono::milliseconds((int)1e3 / freq));

  timer->async_wait(
    boost::bind(getMetricsAndWrite, timer, vehiclePtr, influxDB));
}

bool
subscribeMetrics(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout)
{
  // subscribe to vehicle telemetry
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehiclePtr->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  bool pkgStatus = vehiclePtr->subscribe->initPackageFromTopicList(
    pkgIndex, numTopics, topicList, enableTimestamp, freq);

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
  return true;
}

void
quit(steady_timer* timer)
{
  quitFlag = true;
  timer->expires_after(boost::asio::chrono::milliseconds(50));
}
