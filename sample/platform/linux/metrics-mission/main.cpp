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

void
printUserData(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData);

int
main(int argc, char** argv)
{

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

  // Extra logging setup
  // DJI::OSDK::Log::instance().enableDebugLogging();

  // Obtain Mission Type
  std::string      missionTypeStr = dotenv::env["MISSION_TYPE"];
  DJI_MISSION_TYPE missionType;
  boost::algorithm::to_lower(missionTypeStr);

  if (missionTypeStr.empty())
  {
    missionType = DJI_MISSION_TYPE::WAYPOINT;
  }
  else if (missionTypeStr == "waypoint")
  {
    missionType = DJI_MISSION_TYPE::WAYPOINT;
  }
  else if (missionTypeStr == "hotpoint")
  {
    missionType = DJI_MISSION_TYPE::HOTPOINT;
  }
  else
  {
    std::cout << "Invalid mission type, defaulting to \"waypoint\"."
              << std::endl;
    missionType = DJI_MISSION_TYPE::WAYPOINT;
  }

  // Read Missions
  std::cout << "Reading missions..." << std::endl;
  std::string                missionsPath = dotenv::env["MISSIONS_PATH"];
  auto                       configs      = load_mission_config(missionsPath);
  std::vector<MissionConfig> missions;
  std::vector<PointConfig>   points;
  bool                       status;

  // Init Missions
  std::cout << "Initializing missions..." << std::endl;
  auto mmPtr = std::make_unique<MetricsMission>(
    vehiclePtr, db.get(), missionType, responseTimeout);

  // Obtain Control Authority
  vehiclePtr->flightController->obtainJoystickCtrlAuthoritySync(
    responseTimeout);

  if (std::holds_alternative<std::vector<MissionConfig>>(configs))
  {
    missions = std::get<std::vector<MissionConfig>>(configs);
    status   = mmPtr->initMission(&missions);
    if (!status)
    {
      std::cout << "Could not initialize missions, exiting." << std::endl;
      return -1;
    }
  }
  else if (std::holds_alternative<std::vector<PointConfig>>(configs))
  {
    points = std::get<std::vector<PointConfig>>(configs);
    status = mmPtr->initMission(&points);
    if (!status)
    {
      std::cout << "Could not initialize points, exiting." << std::endl;
      return -1;
    }
  }
  else
  {
    std::cout << "Invalid mission or point config, exiting." << std::endl;
    return -1;
  }

  // Enable custom interrupt
  signal(SIGINT, INThandler);

  // Run Missions
  status = mmPtr->runMissions();
  if (!status)
  {
    std::cout << "Could not run missions, exiting." << std::endl;
    return -1;
  }

  std::thread metricsThread([&] { mmPtr->runContext(); });
  std::cout << std::endl << "Press Ctrl+C to exit." << std::endl;
  while (mmPtr->missionStatus != MissionStatus::completed)
  {
    if (g_quit.load())
      break;
  }

  mmPtr.reset();
  metricsThread.join();
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