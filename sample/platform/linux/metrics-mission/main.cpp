#include "dotenv.h"
#include "metrics-mission.hpp"
#include "parser.hpp"

#include <InfluxDBFactory.h>

#include <atomic>
#include <boost/algorithm/string.hpp>
#include <signal.h>
#include <stdio.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::string
getInfluxUrl();

void
INThandler(int sig);

std::atomic<bool> g_quit(false);

int
main(int argc, char** argv)
{
  signal(SIGINT, INThandler);

  dotenv::env.load_dotenv();
  int responseTimeout = 1;

  // Setup OSDK
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehiclePtr = linuxEnvironment.getVehicle();
  if (vehiclePtr == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting." << std::endl;
    return -1;
  }

  // Setup InfluxDB
  std::string influxUrl = getInfluxUrl();
  auto        db        = influxdb::InfluxDBFactory::Get(influxUrl);
  if (!db.get())
  {
    std::cout << "Could not connect to InfluxDB, exiting." << std::endl;
    return -1;
  }

  // Obtain Mission Type
  std::string missionTypeStr = dotenv::env["MISSION_TYPE"];
  PointType   missionType;
  boost::algorithm::to_lower(missionTypeStr);

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
    std::cout << "Invalid mission type, defaulting to \"waypoint\"."
              << std::endl;
    missionType = PointType::waypoint;
  }

  // Read Missions
  std::cout << "Reading missions..." << std::endl;
  std::string                missionsPath = dotenv::env["MISSIONS_PATH"];
  std::vector<MissionConfig> missions     = load_mission_config(missionsPath);

  // Init Missions
  std::cout << "Initializing missions..." << std::endl;
  auto mmPtr = std::make_unique<MetricsMission>(
    vehiclePtr, db.get(), missionType, responseTimeout);

  vehiclePtr->missionManager->wpMission->setWaypointEventCallback(
    &waypointEventCallback, mmPtr.get());

  // Obtain Control Authority
  vehiclePtr->flightController->obtainJoystickCtrlAuthoritySync(
    responseTimeout);

  // Run Missions
  // std::thread metricsThread([&] { mmPtr->runContext(); });
  std::cout << std::endl << "Press Ctrl+C to exit." << std::endl;
  std::cout << "Running " << missions.size() << " missions..." << std::endl;
  ;
  for (auto& mission : missions)
  {
    std::cout << "Running mission: (" << mission << ")\n";

    bool status = mmPtr->runMission(&mission);
    if (!status)
    {
      std::cout << "Could not run mission." << std::endl;
      continue;
    }

    while (mmPtr->missionStatus != MissionStatus::completed)
    {
      if (g_quit.load())
        break;
    }
    if (g_quit.load())
      break;
  }

  mmPtr.reset();
  // metricsThread.join();
  db.release();
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

void
INThandler(int sig)
{
  std::cout << "Exiting..." << std::endl;
  g_quit.store(true);
}