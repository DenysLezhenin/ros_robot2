#include "canvas/Canvas.h"

namespace canvas {

Canvas::Canvas(const cv::Mat& map, double resolution) {
    map_ = map.clone();
    origin_map_ = map_.clone();
    resolution_ = resolution;
}

void Canvas::drawRobot(const geometry::RobotState& state) {
    int radius = 10;
    cv::Point center(static_cast<int>(state.x / resolution_), 
                    map_.rows - 1 - static_cast<int>(state.y / resolution_));
    cv::circle(map_, center, radius, cv::Scalar(0, 255, 0), -1);
    
    // Draw direction arrow
    double arrow_length = 20;
    cv::Point arrow_end(
        static_cast<int>(center.x + arrow_length * cos(state.theta)),
        static_cast<int>(center.y - arrow_length * sin(state.theta))
    );
    cv::arrowedLine(map_, center, arrow_end, cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);
}

void Canvas::drawLidarPoints(const std::vector<geometry::Point2d>& scan_points) {
    for (const auto& point : scan_points) {
        cv::Point center(static_cast<int>(point.x / resolution_), 
                        map_.rows - 1 - static_cast<int>(point.y / resolution_));
        cv::circle(map_, center, 2, cv::Scalar(0, 0, 255), -1);
    }
}

void Canvas::drawLine(const geometry::Point2d& start, const geometry::Point2d& end) {
    cv::Point pts(static_cast<int>(start.x / resolution_), 
                 map_.rows - 1 - static_cast<int>(start.y / resolution_));
    cv::Point pte(static_cast<int>(end.x / resolution_), 
                 map_.rows - 1 - static_cast<int>(end.y / resolution_));
    cv::line(map_, pts, pte, cv::Scalar(0, 0, 255), 1);
}

void Canvas::show() {
    cv::imshow("Canvas", map_);
}

void Canvas::clear() {
    map_ = origin_map_.clone();
}

}
