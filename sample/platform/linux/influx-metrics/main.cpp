#include "dotenv/dotenv.h"
#include "influx-metrics.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  dotenv::load(".env");
  auto db = connectInflux(std::getenv("INFLUXDB_HOST"),
                          std::getenv("INFLUXDB_PORT"),
                          std::getenv("INFLUXDB_DB"));
  subscribeAndWriteToInflux(vehicle, db.get());
  db.release();
  return 0;
}