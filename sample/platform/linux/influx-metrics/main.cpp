#include "dotenv/dotenv.h"
#include "influx-metrics.hpp"
#include <InfluxDBFactory.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

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

  std::string influxHost = std::getenv("INFLUXDB_HOST");
  std::string influxPort = std::getenv("INFLUXDB_PORT");
  std::string influxDB   = std::getenv("INFLUXDB_DB");

  std::string influxUrl =
    "http://" + influxHost + ":" + influxPort + "/" + influxDB;
  std::cout << "Connecting to InfluxDB at " << influxUrl
            << std::endl

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