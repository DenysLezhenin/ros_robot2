#include "my_project/core/Lidar.hpp"
#include "my_project/core/Geometry.hpp"

namespace lidar {

Lidar::Lidar(const Config& config, std::shared_ptr<environment::Environment> env){
    config_ = config;
    env_ = env;
}

std::vector<geometry::Point2d> Lidar::scan(const geometry::RobotState& state) const{
    std::vector<geometry::Point2d> points;
    points.reserve(config_.beam_count);

    double step_angle = (config_.last_ray_angle - config_.first_ray_angle) / 
        (config_.beam_count - 1);

    // Pre-calculate cos/sin for robot theta
    double cos_theta = cos(state.theta);
    double sin_theta = sin(state.theta);

    for(int i = 0; i < config_.beam_count; i++){
        double ray_angle = config_.first_ray_angle + i * step_angle;
        
        // Pre-calculate cos/sin for ray
        double cos_ray = cos(ray_angle);
        double sin_ray = sin(ray_angle);
        
        // Rotate ray by robot theta
        double rotated_cos = cos_theta * cos_ray - sin_theta * sin_ray;
        double rotated_sin = sin_theta * cos_ray + cos_theta * sin_ray;

        // Coarse search with larger steps (0.2m)
        double hit_distance = config_.max_range;
        for(double j = 0; j < config_.max_range; j += 0.2){
            double x = state.x + j * rotated_cos;
            double y = state.y + j * rotated_sin;

            if(env_->isOccupied(x, y)){
                hit_distance = j;
                break;
            }
        }

        if(hit_distance < config_.max_range){
            points.push_back({
                state.x + hit_distance * rotated_cos,
                state.y + hit_distance * rotated_sin
            });
        }
    }
    return points;
}

}