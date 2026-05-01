#ifndef CANVAS_H
#define CANVAS_H

#include <opencv2/opencv.hpp>
#include "my_project/core/Geometry.hpp"
#include <vector>

namespace canvas {

class Canvas {
public:
    Canvas(const cv::Mat& map, double resolution);
    
    void drawRobot(const geometry::RobotState& state);
    void drawLidarPoints(const std::vector<geometry::Point2d>& scan_points);
    void drawLine(const geometry::Point2d& start, const geometry::Point2d& end);
    void show();
    void clear();
    
    const cv::Mat& getMap() const { return map_; }
    cv::Mat& getMap() { return map_; }

private:
    cv::Mat map_;
    cv::Mat origin_map_;
    double resolution_;
};

}

#endif // CANVAS_H
