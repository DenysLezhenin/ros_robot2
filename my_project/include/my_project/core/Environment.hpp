#pragma once

#include <vector>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "my_project/core/Obstacle.hpp"


namespace environment {

class Environment {
private:
    cv::Mat map_color_;
    cv::Mat map_gray_;
    double resolution_;

    std::string map_;
    std::vector<std::shared_ptr<Obstacle>> obstacles_;

    double station_x_;
    double station_y_;

public:
    void loadFromFile(const std::string& path);

    bool isOccupied(double x, double y) const;

    double getWidth() const;
    double getHeight() const;
    double getResolution() const;
    const cv::Mat& getColorMap() const;


};

} // namespace environment