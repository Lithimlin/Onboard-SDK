#include "parser.hpp"
#include "dotenv.h"

#include <fstream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/reader.h>
#include <stdlib.h>

std::variant<std::vector<MissionConfig>, std::vector<PointConfig>>
load_mission_config(const std::string& filename)
{
  std::ifstream           file(filename);
  Json::Value             root;
  Json::CharReaderBuilder rbuilder;
  std::string             errs;

  std::cout << "Loading missions from " << filename << std::endl;

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

  if (!root.isMember("missions") || !root.isMember("points"))
  {
    std::cerr << "No missions or points specified in mission config file: "
              << filename << std::endl;
    exit(1);
  }

  if (root.isMember("missions"))
  {
    std::vector<MissionConfig> missions;
    bool status = parse_missions(&root, waitTime, &missions);
    if (!status)
    {
      std::cerr << "Failed to parse missions" << std::endl;
      exit(1);
    }
    return missions;
  }
  else if (root.isMember("points"))
  {
    std::vector<PointConfig> points;
    bool                     status = parse_points(&root, waitTime, &points);
    if (!status)
    {
      std::cerr << "Failed to parse points" << std::endl;
      exit(1);
    }
    return points;
  }

  std::cerr << "No missions or points specified in mission config file. How "
               "did we get here?"
            << std::endl;
  exit(-1);
}

///////////////////////////////////////////////////////////////////////////////////

bool
parse_missions(Json::Value*                rootVal,
               uint8_t                     waitTime,
               std::vector<MissionConfig>* missions)
{
  Json::Value root = *rootVal;

  if (!root.isMember("missions"))
  {
    return false;
  }

  for (Json::Value::const_iterator it = root["missions"].begin();
       it != root["missions"].end();
       ++it)
  {
    Json::Value   obj      = *it;
    float         altitude = obj["altitude"].asFloat();
    float         radius   = obj["radius"].asFloat();
    uint8_t       numStops = obj["numStops"].asInt();
    MissionConfig mission(altitude, radius, numStops, waitTime);

    missions->push_back(mission);
  }

  return true;
}

bool
parse_points(Json::Value*              rootVal,
             uint8_t                   waitTime,
             std::vector<PointConfig>* points)
{
  Json::Value root = *rootVal;

  if (!root.isMember("points"))
  {
    return false;
  }

  for (Json::Value::const_iterator it = root["points"].begin();
       it != root["points"].end();
       ++it)
  {
    Json::Value obj      = *it;
    float       altitude = obj["altitude"].asFloat();
    float       dlat     = obj["dlat"].asFloat();
    float       dlon     = obj["dlon"].asFloat();
    PointConfig point(dlat, dlon, altitude, waitTime);

    points->push_back(point);
  }
  return true;
}
