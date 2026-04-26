#include "my_project/core//Lidar.hpp"
#include "my_project/core/Geometry.hpp"

namespace lidar {

Lidar::Lidar(const Config& config, std::shared_ptr<environment::Environment> env){
    config_ = config;
    env_ = env;
}

std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState& state) const{
    std::vector<geometry::Point2d> points;

    double step_angle = (config_.last_ray_angle - config_.first_ray_angle) / 
        (config_.beam_count - 1);

    for(int i = 0; i < config_.beam_count; i++){
        double ray_angle = state.theta + config_.first_ray_angle + i * step_angle;

        for(double j = 0; j < config_.max_range; j += env_->getResolution()){
            double x = state.x + j * cos(ray_angle);
            double y = state.y + j * sin(ray_angle);

            if(env_->isOccupied(x, y)){
                points.push_back({x, y});
                break;
            }

        }
    }
    return points;
}

}