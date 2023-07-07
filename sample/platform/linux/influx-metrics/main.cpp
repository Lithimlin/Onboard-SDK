#include "influx-metrics.hpp"
#include <InfluxDBFactory.h>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::string
getenvvar(const std::string& key);

std::string
getInfluxUrl();

// main
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

  std::string influxUrl = getInfluxUrl();

  auto db = influxdb::InfluxDBFactory::Get(influxUrl);

  if (!db)
  {
    std::cout << "Could not connect to InfluxDB, exiting.\n";
    return -1;
  }

  bool status = influxMetrics::subscribeMetrics(vehicle, 1);
  if (!status)
  {
    std::cout << "Could not subscribe to metrics, exiting.\n";
    return -1;
  }

  boost::asio::io_context   ctx;
  boost::asio::steady_timer timer(ctx, boost::asio::chrono::seconds(1));
  timer.async_wait(boost::bind(influxMetrics::getMetricsAndWrite,
                               boost::asio::placeholders::error,
                               &timer,
                               vehicle,
                               db.get()));

  ctx.run();

  std::cout << "Done!" << std::endl;

  db.release();
  return 0;
}

// functions
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