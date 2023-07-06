#include "dotenv/dotenv.h"
#include "influx-metrics.hpp"
#include <InfluxDBFactory.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::string
getenvvar(const std::string& key)
{
  char const* value = std::getenv(key.c_str());
  return value ? std::string(value) : std::string();
}

int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  dotenv::load(".env");

  std::string influxHost = getenvvar("INFLUXDB_HOST");
  std::string influxPort = getenvvar("INFLUXDB_PORT");
  std::string influxDB   = getenvvar("INFLUXDB_DB");

  std::string influxUrl =
    std::string("http://" + influxHost + ":" + influxPort + "/" + influxDB);
  std::cout << "Connecting to InfluxDB at " << influxUrl << std::endl;

  auto db = influxdb::InfluxDBFactory::Get(influxUrl);

  if (db == nullptr)
  {
    std::cout << "Could not connect to InfluxDB, exiting.\n";
    return -1;
  }

  subscribeAndWriteToInflux(vehicle, db.get());
  db.release();
  return 0;
}