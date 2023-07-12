#include "influx-metrics.hpp"
#include "mission.hpp"
#include <InfluxDBFactory.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <signal.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

using boost::asio::steady_timer;

static boost::asio::io_context ctx;
static steady_timer metricsTimer(ctx, boost::asio::chrono::seconds(1));
static steady_timer missionTimer(ctx, boost::asio::chrono::seconds(1));

std::string
getenvvar(const std::string& key);

std::string
getInfluxUrl();

void
INThandler(int sig);

// main
int
main(int argc, char** argv)
{
  // signal(SIGINT, INThandler);

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == nullptr)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // std::string influxUrl = getInfluxUrl();

  // auto db = influxdb::InfluxDBFactory::Get(influxUrl);

  // if (!db)
  // {
  //   std::cout << "Could not connect to InfluxDB, exiting.\n";
  //   return -1;
  // }

  // bool status = influxMetrics::subscribeMetrics(vehicle, 1);
  // if (!status)
  // {
  //   std::cout << "Could not subscribe to metrics, exiting.\n";
  //   return -1;
  // }

  // Obtain Control Authority
  vehicle->flightController->obtainJoystickCtrlAuthoritySync(1);

  mission::runWaypointMission(&missionTimer, vehicle, 10.0f, 5.0f, 4, 5, 1);

  // std::cout << "Starting timer..." << std::endl;
  // metricsTimer.async_wait(boost::bind(influxMetrics::getMetricsAndWrite,
  //                                     boost::asio::placeholders::error,
  //                                     &metricsTimer,
  //                                     vehicle,
  //                                     db.get()));

  // std::cout << "Running context..." << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;

  // ctx.run();
  while (true)
  {
    OsdkOsal_TaskSleepMs(1000);
  }

  std::cout << "Done!" << std::endl;

  // db.release();
  return 0;
}

// functions
void
INThandler(int sig)
{
  std::cout << "\nExiting..." << std::endl;
  metricsTimer.expires_after(boost::asio::chrono::milliseconds(50));
  missionTimer.expires_after(boost::asio::chrono::milliseconds(100));
}

std::string
getenvvar(const std::string& key)
{
  char const* value = std::getenv(key.c_str());
  return value ? std::string(value) : std::string();
}

std::string
getInfluxUrl()
{
  std::string influxHost = getenvvar("INFLUXDB_HOST");
  std::string influxPort = getenvvar("INFLUXDB_PORT");
  std::string influxDB   = getenvvar("INFLUXDB_DB");
  std::string influxUser = getenvvar("INFLUXDB_USER");
  std::string influxPass = getenvvar("INFLUXDB_PASS");

  std::cout << "Connecting to InfluxDB at "
            << "http://" + influxUser + ":<REDACTED>@" + influxHost + ":" +
                 influxPort + "?db=" + influxDB
            << std::endl;

  return std::string("http://" + influxUser + ":" + influxPass + "@" +
                     influxHost + ":" + influxPort + "?db=" + influxDB);
}