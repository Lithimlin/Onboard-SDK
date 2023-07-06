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

bool
subscribeAndWriteToInflux(DJI::OSDK::Vehicle* vehiclePtr,
                          influxdb::InfluxDB* influxDB,
                          int                 responseTimeout = 1);

#endif
