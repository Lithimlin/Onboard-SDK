#include "influx-metrics.hpp"
#include "mission.hpp"
#include <InfluxDBFactory.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <signal.h>
#include <stdio.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

using boost::asio::steady_timer;

static bool                    quit                 = false;
static bool                    waitForSecondMission = true;
static boost::asio::io_context ctx;
static steady_timer metricsTimer(ctx, boost::asio::chrono::seconds(1));

static std::array<char, 1024> pipe_buffer;

std::string
getenvvar(const std::string& key, const bool required = true);

std::string
getInfluxUrl();

void
INThandler(int sig);

void
waypointEventCallback(Vehicle*      vehiclePtr,
                      RecvContainer recvFrame,
                      UserData      userData);

// main
int
main(int argc, char** argv)
{
  signal(SIGINT, INThandler);

  int responseTimeout = 1;

  // Setup OSDK
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Setup InfluxDB
  std::string influxUrl = getInfluxUrl();
  auto        db        = influxdb::InfluxDBFactory::Get(influxUrl);
  if (!db)
  {
    std::cout << "Could not connect to InfluxDB, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->flightController->obtainJoystickCtrlAuthoritySync(responseTimeout);
  // Setup Waypoint Mission
  float radius   = 25.0f;
  float altitude = 10.0f;
  int   numStops = 8;
  int   waitTime = 30;
  mission::runWaypointMission(
    vehicle, radius, altitude, numStops, waitTime, responseTimeout);

  vehicle->missionManager->wpMission->setWaypointEventCallback(
    &waypointEventCallback, vehicle);

  // Setup Metrics
  bool status = influxMetrics::subscribeMetrics(vehicle, responseTimeout);
  if (!status)
  {
    std::cout << "Could not subscribe to metrics, exiting.\n";
    return -1;
  }

  // Start Metrics Timer
  std::cout << "Starting timer..." << std::endl;
  metricsTimer.async_wait(boost::bind(influxMetrics::getMetricsAndWrite,
                                      boost::asio::placeholders::error,
                                      &metricsTimer,
                                      vehicle,
                                      db.get()));

  std::cout << "Running context..." << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;

  ctx.run();
  while (!quit)
  {
  }

  std::cout << "Done!" << std::endl;

  db.release();
  return 0;
}

// functions
void
INThandler(int sig)
{
  std::cout << "\nExiting..." << std::endl;
  metricsTimer.expires_after(boost::asio::chrono::milliseconds(50));
  quit = true;
}

std::string
getenvvar(const std::string& key, const bool required)
{
  char*       value = std::getenv(key.c_str());
  std::string result;
  if (required && !value)
  {
    std::cout << "Could not find environment variable: " << key << std::endl;
    std::cout << "Please enter a value:" << std::endl;
    std::cin >> result;
    return result;
  }
  return value ? std::string(value) : std::string();
}

std::string
getInfluxUrl()
{
  std::string influxHost = getenvvar("INFLUXDB_HOST");
  std::string influxPort = getenvvar("INFLUXDB_PORT");
  std::string influxDB   = getenvvar("INFLUXDB_BUCKET");
  std::string influxUser = getenvvar("INFLUXDB_USER");
  std::string influxPass = getenvvar("INFLUXDB_PASS");

  std::cout << "Connecting to InfluxDB at "
            << "http://" + influxUser + ":<REDACTED>@" + influxHost + ":" +
                 influxPort + "?db=" + influxDB
            << std::endl;

  return std::string("http://" + influxUser + ":" + influxPass + "@" +
                     influxHost + ":" + influxPort + "?db=" + influxDB);
}

void
waypointEventCallback(Vehicle*      vehiclePtr,
                      RecvContainer recvFrame,
                      UserData      userData)
{
  if (recvFrame.recvData.wayPointReachedData.incident_type ==
        WayPointIncidentType::NAVI_MISSION_FINISH &&
      recvFrame.recvData.wayPointReachedData.current_status == 0)
  {
    DSTATUS("Mission finished.");
    if (waitForSecondMission)
    {
      // Setup Waypoint Mission
      float radius          = 35.0f;
      float altitude        = 5.0f;
      int   numStops        = 5;
      int   waitTime        = 30;
      int   responseTimeout = 1;
      mission::runWaypointMission(
        vehiclePtr, radius, altitude, numStops, waitTime, responseTimeout);

      vehiclePtr->missionManager->wpMission->setWaypointEventCallback(
        &waypointEventCallback, vehiclePtr);
      waitForSecondMission = false;
    }
    else
    {
      kill(getpid(), SIGINT);
    }
    return;
  }

  if (recvFrame.recvData.wayPointReachedData.incident_type !=
      WayPointIncidentType::NAVI_MISSION_WP_REACH_POINT)
  {
    return;
  }

  if (recvFrame.recvData.wayPointReachedData.current_status != 4)
  {
    return;
  }

  std::unique_ptr<FILE, decltype(&pclose)> pipe(
    popen("cd ~/wireless-measurements && "
          ". env/bin/activate && "
          "./collect_metrics.py",
          "r"),
    pclose);

  if (!pipe)
  {
    std::cout << "popen() failed!" << std::endl;
    return;
  }

  while (fgets(pipe_buffer.data(), pipe_buffer.size(), pipe.get()) != nullptr)
  {
    printf("%s", pipe_buffer.data());
  }
}