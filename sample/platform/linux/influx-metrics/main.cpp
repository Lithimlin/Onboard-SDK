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

  std::string influxHost = getenvvar("INFLUXDB_HOST");
  std::string influxPort = getenvvar("INFLUXDB_PORT");
  std::string influxDB   = getenvvar("INFLUXDB_DB");
  std::string influxUser = getenvvar("INFLUXDB_USER");
  std::string influxPass = getenvvar("INFLUXDB_PASS");

  std::string influxUrl =
    std::string("http://" + influxUser + ":" + influxPass + "@" + influxHost +
                ":" + influxPort + "?db=" + influxDB);
  std::cout << "Connecting to InfluxDB at " << influxUrl << std::endl;

  std::unique_ptr<influxdb::InfluxDB> db =
    influxdb::InfluxDBFactory::Get(influxUrl);

  if (!db)
  {
    std::cout << "Could not connect to InfluxDB, exiting.\n";
    return -1;
  }

  subscribeAndWriteToInflux(vehicle, db.get(), 2);
  db.release();
  return 0;
}