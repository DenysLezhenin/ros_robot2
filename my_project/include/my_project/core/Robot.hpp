#pragma once

#include <functional>
#include <thread>
#include <mutex>
#include <cmath>
#include "my_project/core/Geometry.hpp"

namespace robot {

struct Config {
    geometry::Twist accelerations;
    geometry::Twist emergency_decelerations;
    double command_duration;
    int simulation_period_ms;
    double resolution;
};


class Robot {
public:
    using CollisionCb = std::function<bool(geometry::RobotState)>;

    Robot(const Config& config, const CollisionCb& collision_cb = nullptr);
    ~Robot();
    void stopThread();
    void setVelocity(const geometry::Twist& velocity);
    geometry::RobotState getState() const;
    bool isInCollision() const;

private:
    Config config_;
    geometry::RobotState state_;
    mutable std::mutex mutex_;
    std::thread thread_;
    bool running_;
    geometry::Twist target_velocity_;
    std::chrono::steady_clock::time_point last_command_time_;
    CollisionCb collision_cb_;
    double resolution_;

protected:
    void update(const geometry::Twist& velocity, double dt, bool emergency);
    void run_thread();
    bool inCollision(const geometry::RobotState& state) const;

};
} // namespace robot

