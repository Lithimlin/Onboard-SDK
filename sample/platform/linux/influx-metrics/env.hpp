#ifndef DJIOSDK_INFLUX_METRICS_ENV_H
#define DJIOSDK_INFLUX_METRICS_ENV_H

#include <string>

std::string
getenvvar(const std::string& key)
{
  char const* value = std::getenv(key.c_str());
  return value ? std::string(value) : std::string();
}

#endif // DJIOSDK_INFLUX_METRICS_ENV_H