#include "dotenv.h"
#include "metrics-mission.hpp"
#include "parser.hpp"
#include <InfluxDBFactory.h>
#include <signal.h>
#include <stdio.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::string
getInfluxUrl();

int
main(int argc, char** argv)
{
  // TODO: SIGINT

  dotenv::env.load_dotenv();
  int responseTimeout = 1;

  // Setup OSDK
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehiclePtr = linuxEnvironment.getVehicle();
  if (vehiclePtr == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Setup InfluxDB
  std::string influxUrl = getInfluxUrl();
  auto        db        = influxdb::InfluxDBFactory::Get(influxUrl);
  if (!db.get())
  {
    std::cout << "Could not connect to InfluxDB, exiting.\n";
    return -1;
  }

  // Obtain Mission Type
  std::string missionTypeStr = dotenv::env["MISSION_TYPE"];
  PointType   missionType;
  if (missionTypeStr.empty())
  {
    missionType = PointType::waypoint;
  }
  else if (missionTypeStr == "waypoint")
  {
    missionType = PointType::waypoint;
  }
  else if (missionTypeStr == "hotpoint")
  {
    missionType = PointType::hotpoint;
  }
  else
  {
    std::cout << "Invalid mission type, defaulting to \"Waypoint\".\n";
    missionType = PointType::waypoint;
  }

  // Read Missions
  std::string                missionsPath = dotenv::env["MISSIONS_PATH"];
  std::vector<MissionConfig> missions     = load_mission_config(missionsPath);

  // Init Missions
  MetricsMission mm =
    MetricsMission(vehiclePtr, db.get(), missionType, responseTimeout);

  // Obtain Control Authority
  vehiclePtr->flightController->obtainJoystickCtrlAuthoritySync(
    responseTimeout);

  // Run Missions
  for (auto& mission : missions)
  {
    std::cout << "Running mission: (" << mission.toString() << ")\n";

    bool status = mm.runMission(&mission);
    if (!status)
    {
      std::cout << "Could not run mission.";
      continue;
    }

    while (mm.missionStatus != MissionStatus::completed)
    {
    }
  }

  return 0;
}

std::string
getInfluxUrl()
{
  std::string influxHost   = dotenv::env["INFLUXDB_HOST"];
  std::string influxPort   = dotenv::env["INFLUXDB_PORT"];
  std::string influxUser   = dotenv::env["INFLUXDB_USER"];
  std::string influxPass   = dotenv::env["INFLUXDB_PASS"];
  std::string influxBucket = dotenv::env["INFLUXDB_BUCKET"];

  std::cout << "Connecting to InfluxDB at "
            << "http://" + influxUser + ":<REDACTED>@" + influxHost + ":" +
                 influxPort + "?db=" + influxBucket
            << std::endl;

  return std::string("http://" + influxUser + ":" + influxPass + "@" +
                     influxHost + ":" + influxPort + "?db=" + influxBucket);
}