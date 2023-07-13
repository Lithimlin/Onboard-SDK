#ifndef DJIOSDK_INFLUX_METRICS_HPP
#define DJIOSDK_INFLUX_METRICS_HPP

// System Includes
#include <boost/asio.hpp>
#include <iostream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// InfluxDB
#include <InfluxDBFactory.h>

namespace influxMetrics
{
bool
subscribeMetrics(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);

DJI::OSDK::Telemetry::TypeMap<
  DJI::OSDK::Telemetry::TopicName::TOPIC_GPS_FUSED>::type
getGpsPosition(Vehicle* vehiclePtr);

void
getMetricsAndWrite(const boost::system::error_code& e,
                   boost::asio::steady_timer*       timer,
                   DJI::OSDK::Vehicle*              vehiclePtr,
                   influxdb::InfluxDB*              influxDB);

}
#endif // DJIOSDK_INFLUX_METRICS_HPP
