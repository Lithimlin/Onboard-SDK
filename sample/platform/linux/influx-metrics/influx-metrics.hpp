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

void
getMetricsAndWrite(const boost::system::error_code& e,
                   boost::asio::steady_timer*       timer,
                   DJI::OSDK::Vehicle*              vehiclePtr,
                   influxdb::InfluxDB*              influxDB);

void
quit(boost::asio::steady_timer* timer);
}
#endif // DJIOSDK_INFLUX_METRICS_HPP
