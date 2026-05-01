#include "my_project/core/Environment.hpp"
#include <yaml-cpp/yaml.h>

namespace environment {

void Environment::loadFromFile(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);

    if(!config["map"]){
        throw std::runtime_error("Map not defined!");
    } 

    map_ = config["map"].as<std::string>();

    map_color_ = cv::imread(map_, cv::IMREAD_COLOR);
    if(map_color_.empty()){
        throw std::runtime_error("Failed to load map image: " + map_);
    }
    cv::cvtColor(map_color_, map_gray_, cv::COLOR_BGR2GRAY);

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

    resolution_ = config["resolution"].as<double>();
}

bool Environment::isOccupied(double x, double y) const {
    int col = static_cast<int>(std::floor(x / resolution_));
    int row = static_cast<int>(std::floor(y / resolution_));
    
    if (col < 0 || col >= map_gray_.cols || row < 0 || row >= map_gray_.rows) {
        return true;
    }
    return map_gray_.at<uchar>(row, col) < 128;
}

double Environment::getWidth() const{
    return map_gray_.cols * resolution_;
}

double Environment::getHeight() const{
    return map_gray_.rows * resolution_;
}

double Environment::getResolution() const{
    return resolution_;
}

const cv::Mat& Environment::getColorMap() const {
    return map_color_;
}

}