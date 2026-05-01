#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "my_project/core/Robot.hpp"
#include "my_project/core/Environment.hpp"
#include <memory>
#include <yaml-cpp/yaml.h>

class RobotNode : public rclcpp::Node {
public:
    RobotNode() : Node("robot_node") {
        // Declare parameters
        this->declare_parameter("config_file", "/home/ros/ros2_ws/config/game_config.yaml");
        this->declare_parameter("robot_radius", 10);
        
        std::string config_file = this->get_parameter("config_file").as_string();
        
        // Load configuration
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Initialize environment
        try {
            env_ = std::make_shared<environment::Environment>();
            if (config["environment"] && config["environment"]["map_config"]) {
                std::string map_config = config["environment"]["map_config"].as<std::string>();
                env_->loadFromFile(map_config);
            } else {
                throw std::runtime_error("Map config not found in YAML");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load environment: %s", e.what());
            throw;
        }
        
        // Initialize robot with collision callback
        robot::Config robot_config;
        robot_config.accelerations = {1.0, 1.0};
        robot_config.emergency_decelerations = {2.0, 2.0};
        robot_config.command_duration = 1.0;
        robot_config.simulation_period_ms = 50;
        robot_config.resolution = env_->getResolution();
        
        auto collision_cb = [this](const geometry::RobotState& state) {
            return env_->isOccupied(state.x, state.y);
        };
        
        robot_ = std::make_shared<robot::Robot>(robot_config, collision_cb);
        
        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&RobotNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // Create publisher for robot status
        robot_status_pub_ = this->create_publisher<robot_msgs::msg::RobotStatus>("/robot_status", 10);
        
        // Create timer for publishing robot status at 10 Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotNode::publish_status, this));
        
        RCLCPP_INFO(this->get_logger(), "RobotNode initialized successfully");
    }
    
    ~RobotNode() {
        robot_->stopThread();
    }

private:
    std::shared_ptr<robot::Robot> robot_;
    std::shared_ptr<environment::Environment> env_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<robot_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry::Twist twist;
        twist.linear = msg->linear.x;
        twist.angular = msg->angular.z;
        robot_->setVelocity(twist);
        
        // Log when command received (with throttling)
        static int cmd_count = 0;
        if (cmd_count++ % 10 == 0 || (twist.linear != 0.0 || twist.angular != 0.0)) {
            RCLCPP_DEBUG(this->get_logger(), "[CMD] v=%.2f m/s, ω=%.2f rad/s", twist.linear, twist.angular);
        }
    }
    
    void publish_status() {
        auto state = robot_->getState();
        
        auto status = robot_msgs::msg::RobotStatus();
        status.x = state.x;
        status.y = state.y;
        status.theta = state.theta;
        status.linear_velocity = state.velocity.linear;
        status.angular_velocity = state.velocity.angular;
        status.current_capacity = 0;  // Will be updated by game_node
        status.max_capacity = 0;
        status.at_station = false;
        
        robot_status_pub_->publish(status);
        
        // Log position changes
        static geometry::RobotState last_state;
        if (state.x != last_state.x || state.y != last_state.y) {
            RCLCPP_DEBUG(this->get_logger(), "[POS] x=%.1f, y=%.1f, θ=%.2f rad", state.x, state.y, state.theta);
            last_state = state;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}