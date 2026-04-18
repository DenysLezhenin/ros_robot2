#pragma once

#include <vector>
#include <memory>
#include <string>
#include "my_project/core/Obstacle.hpp"

class Environment {
public:
    void loadFromFile(const std::string& path);

private:
    std::string map_;
    std::vector<std::shared_ptr<Obstacle>> obstacles_;

    double station_x_;
    double station_y_;
};