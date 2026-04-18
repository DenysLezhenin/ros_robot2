#include "robot/Robot.h"


namespace robot {

Robot::Robot(const Config& config, const CollisionCb& collision_cb){
    config_ = config;
    resolution_ = config.resolution;
    state_ = {20.0,20.0,0.0,{0.0,0.0}};

    last_command_time_ = std::chrono::steady_clock::now();
    target_velocity_ = {0,0};

    running_ = true;
    thread_ = std::thread(&Robot::run_thread, this);
    collision_cb_ = collision_cb;
}

Robot::~Robot(){
    running_ = false;
    if(thread_.joinable()){
        thread_.join();
    }
}

void Robot::run_thread(){

    double dt = config_.simulation_period_ms / 1000.0;

    while(running_){

        geometry::Twist vel;
        bool emergency = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_command_time_).count();

            if(elapsed > config_.command_duration){
                vel = {0.0, 0.0};
                emergency = true;
            } else {
                vel = target_velocity_;
            }
        }

        update(vel, dt, emergency);

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.simulation_period_ms)
        );
    }
}

void Robot::update(const geometry::Twist& velocity, double dt, bool emergency){

    std::lock_guard<std::mutex> lock(mutex_);

    auto acc = emergency ? config_.emergency_decelerations : config_.accelerations;

    // --- LINEAR ---
    if(state_.velocity.linear < velocity.linear){
        state_.velocity.linear += acc.linear * dt;
        if(state_.velocity.linear > velocity.linear)
            state_.velocity.linear = velocity.linear;
    }
    else if(state_.velocity.linear > velocity.linear){
        state_.velocity.linear -= acc.linear * dt;
        if(state_.velocity.linear < velocity.linear)
            state_.velocity.linear = velocity.linear;
    }

    // --- ANGULAR ---
    if(state_.velocity.angular < velocity.angular){
        state_.velocity.angular += acc.angular * dt;
        if(state_.velocity.angular > velocity.angular)
            state_.velocity.angular = velocity.angular;
    }
    else if(state_.velocity.angular > velocity.angular){
        state_.velocity.angular -= acc.angular * dt;
        if(state_.velocity.angular < velocity.angular)
            state_.velocity.angular = velocity.angular;
    }

    // --- POSITION ---
    geometry::RobotState new_state = state_;

    new_state.theta += state_.velocity.angular * dt;

    new_state.x += state_.velocity.linear * cos(state_.theta) * dt;
    new_state.y += state_.velocity.linear * sin(state_.theta) * dt;

    if(inCollision(new_state)){
        state_.velocity = {0.0, 0.0};
    }   
    else{
        state_ = new_state;
    }
}

bool Robot::inCollision(const geometry::RobotState& state) const{
    int radius = 10;

    for(double angle = 0; angle < 2*M_PI; angle += 0.5){

        geometry::RobotState check_state = state;

        check_state.x += radius * resolution_ * cos(angle);
        check_state.y += radius * resolution_ * sin(angle);

        if(collision_cb_(check_state))
            return true;
    }

    return false;
}

bool Robot::isInCollision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return inCollision(state_);
}

void Robot::setVelocity(const geometry::Twist& velocity){
    std::lock_guard<std::mutex> lock(mutex_);
    target_velocity_ = velocity;
    last_command_time_ = std::chrono::steady_clock::now();
}

geometry::RobotState Robot::getState() const{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

};
