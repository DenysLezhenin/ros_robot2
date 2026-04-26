#include "my_project/core/Environment.hpp"
#include <yaml-cpp/yaml.h>

#include <iostream>

void Environment::loadFromFile(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);

    if(!config["map"]){
        throw std::runtime_error("Map not defined!");
    } 

    map_ = config["map"].as<std::string>();

    for(const auto& obstacle_config : config["obstacles"]){
        std::string type = obstacle_config["type"].as<std::string>();

        if (type == "circle") {
            auto c = std::make_shared<CircleObstacle>();
            c->x = obstacle_config["x"].as<double>();
            c->y = obstacle_config["y"].as<double>();
            c->radius = obstacle_config["radius"].as<double>();
            obstacles_.push_back(c);
        }
        

        else if (type == "rectangle") {
            auto r = std::make_shared<RectangleObstacle>();
            r->x = obstacle_config["x"].as<double>();
            r->y = obstacle_config["y"].as<double>();
            r->width = obstacle_config["width"].as<double>();
            r->height = obstacle_config["height"].as<double>();
            obstacles_.push_back(r);
        }
    }

    station_x_ = config["station"]["x"].as<double>();
    station_y_ = config["station"]["y"].as<double>();
}