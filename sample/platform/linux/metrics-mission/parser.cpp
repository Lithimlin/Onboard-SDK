#include "parser.hpp"
#include "dotenv.h"

#include <fstream>
#include <json/json.h>
#include <json/reader.h>
#include <stdlib.h>

std::vector<MissionConfig>
load_mission_config(const std::string& filename)
{
  std::ifstream              file(filename);
  Json::Value                root;
  Json::CharReaderBuilder    rbuilder;
  std::string                errs;
  std::vector<MissionConfig> missions;

  rbuilder["allowComments"]       = true;
  rbuilder["allowTrailingCommas"] = true;

  bool parsingSuccessful = Json::parseFromStream(rbuilder, file, &root, &errs);
  if (!parsingSuccessful)
  {
    std::cerr << "Failed to parse mission config file: " << filename
              << std::endl;
    std::cerr << errs << std::endl;
    exit(1);
  }

  uint8_t waitTime;
  if (!root.isMember("waitTime"))
  {
    std::string waitTimeStr = dotenv::env["WAIT_TIME"];

    if (waitTimeStr.empty())
    {
      std::cerr << "No waitTime specified in mission config file " << filename
                << " or in environment file." << std::endl;
      exit(1);
    }

    waitTime = std::stoi(waitTimeStr);
  }
  else
  {
    waitTime = root["waitTime"].asInt();
  }

  if (!root.isMember("missions"))
  {
    std::cerr << "No missions specified in mission config file: " << filename
              << std::endl;
    exit(1);
  }
  for (Json::Value::const_iterator it = root["missions"].begin();
       it != root["missions"].end();
       ++it)
  {
    Json::Value   obj      = *it;
    float         altitude = obj["altitude"].asFloat();
    float         radius   = obj["radius"].asFloat();
    uint8_t       numStops = obj["numStops"].asInt();
    MissionConfig mission = MissionConfig(altitude, radius, numStops, waitTime);

    missions.push_back(mission);
  }

  return missions;
}