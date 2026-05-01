#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "my_project/core/Lidar.hpp"
#include "my_project/core/Environment.hpp"
#include <yaml-cpp/yaml.h>

class LidarNode : public rclcpp::Node {
public:
    LidarNode() : Node("lidar_node") {
        // Declare parameters
        this->declare_parameter("config_file", "/home/ros/ros2_ws/config/game_config.yaml");
        std::string config_file = this->get_parameter("config_file").as_string();
        
        // Load configuration
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Initialize environment
        env_ = std::make_shared<environment::Environment>();
        if (config["environment"] && config["environment"]["map_config"]) {
            std::string map_config = config["environment"]["map_config"].as<std::string>();
            env_->loadFromFile(map_config);
        }
        
        // Initialize lidar
        lidar::Config lidar_config;
        lidar_config.beam_count = config["lidar"]["beam_count"].as<int>(360);
        lidar_config.max_range = config["lidar"]["max_range"].as<double>(10.0);
        lidar_config.first_ray_angle = -M_PI;
        lidar_config.last_ray_angle = M_PI;
        
        lidar_ = std::make_shared<lidar::Lidar>(lidar_config, env_);
        
        // Create subscription to robot status
        robot_status_sub_ = this->create_subscription<robot_msgs::msg::RobotStatus>(
            "/robot_status", 10, std::bind(&LidarNode::robot_status_callback, this, std::placeholders::_1));
        
        // Create publisher for laser scan
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        
        RCLCPP_INFO(this->get_logger(), "LidarNode initialized - %d beams, range: %.2f m",
                   lidar_config.beam_count, lidar_config.max_range);
    }

private:
    std::shared_ptr<environment::Environment> env_;
    std::shared_ptr<lidar::Lidar> lidar_;
    rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    
    void robot_status_callback(const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        // Convert robot status to robot state
        geometry::RobotState state;
        state.x = msg->x;
        state.y = msg->y;
        state.theta = msg->theta;
        state.velocity.linear = msg->linear_velocity;
        state.velocity.angular = msg->angular_velocity;
        
        // Perform scan
        auto scan_points = lidar_->scan(state);
        
        // Create LaserScan message
        auto laser_scan = sensor_msgs::msg::LaserScan();
        laser_scan.header.stamp = this->get_clock()->now();
        laser_scan.header.frame_id = "laser_frame";
        laser_scan.angle_min = -M_PI;
        laser_scan.angle_max = M_PI;
        laser_scan.angle_increment = 2 * M_PI / 360;
        laser_scan.time_increment = 0.0;
        laser_scan.scan_time = 0.1;
        laser_scan.range_min = 0.0;
        laser_scan.range_max = 10.0;
        
        // Initialize ranges with max range
        laser_scan.ranges.resize(360, 10.0);
        
        // Fill in the scan points
        for (const auto& point : scan_points) {
            double dx = point.x - state.x;
            double dy = point.y - state.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angle = std::atan2(dy, dx) - state.theta;
            
            // Normalize angle to [-pi, pi]
            while (angle > M_PI) angle -= 2 * M_PI;
            while (angle < -M_PI) angle += 2 * M_PI;
            
            // Convert angle to index
            int index = static_cast<int>((angle + M_PI) / (2 * M_PI / 360));
            if (index >= 0 && index < 360) {
                laser_scan.ranges[index] = distance;
            }
        }
        
        laser_scan_pub_->publish(laser_scan);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
