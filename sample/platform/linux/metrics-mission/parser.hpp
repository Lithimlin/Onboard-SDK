#pragma once

#include "metrics-mission.hpp"

std::variant<std::vector<MissionConfig>, std::vector<PointConfig>>
load_mission_config(const std::string& filename);