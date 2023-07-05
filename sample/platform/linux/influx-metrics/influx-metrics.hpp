#ifndef DJIOSDK_INFLUX_METRICS_HPP
#define DJIOSDK_INFLUX_METRICS_HPP

// System Includes
#include <iostream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// InfluxDB
#include <InfluxDBFactory.h>

static std::unique_ptr<influxdb::InfluxDB>
connectInflux(const std::string& host,
              const std::string& port,
              const std::string& database);

bool
subscribeAndWriteToInflux(DJI::OSDK::Vehicle* vehiclePtr,
                          influxdb::InfluxDB* influxDB,
                          int                 responseTimeout = 1);

#endif
