#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/game_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "canvas/Canvas.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <yaml-cpp/yaml.h>

class InteractiveTeleopNode : public rclcpp::Node {
public:
    InteractiveTeleopNode() : Node("interactive_teleop_node") {
        // Load map from config
        load_map();
        
        // Create publishers/subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        status_sub_ = this->create_subscription<robot_msgs::msg::RobotStatus>(
            "/robot_status", 10, 
            std::bind(&InteractiveTeleopNode::robot_status_callback, this, std::placeholders::_1));
        
        game_state_sub_ = this->create_subscription<robot_msgs::msg::GameState>(
            "/game_state", 10,
            std::bind(&InteractiveTeleopNode::game_state_callback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&InteractiveTeleopNode::scan_callback, this, std::placeholders::_1));
        
        // Get parameters
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 1.0);
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        
        // Setup terminal
        setup_terminal();
        show_menu();
        
        // Start threads
        input_thread_ = std::thread(&InteractiveTeleopNode::input_loop, this);
        display_thread_ = std::thread(&InteractiveTeleopNode::display_loop, this);
    }
    
    ~InteractiveTeleopNode() {
        restore_terminal();
        running_ = false;
        if (input_thread_.joinable()) input_thread_.join();
        if (display_thread_.joinable()) display_thread_.join();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<robot_msgs::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    std::thread input_thread_, display_thread_;
    bool running_ = true;
    struct termios old_settings_, new_settings_;
    
    // Robot state
    geometry::RobotState robot_state_{0, 0, 0, 0, 0};
    robot_msgs::msg::GameState game_state_;
    std::vector<geometry::Point2d> lidar_points_;
    
    // Canvas
    std::unique_ptr<canvas::Canvas> canvas_;
    cv::Mat map_;
    double resolution_ = 0.1;
    double linear_speed_, angular_speed_;
    
    void load_map() {
        try {
            YAML::Node config = YAML::LoadFile("/home/ros/ros2_ws/src/my_project/config/game_config.yaml");
            std::string map_path = config["environment"]["map_file"].as<std::string>();
            
            map_ = cv::imread(map_path, cv::IMREAD_COLOR);
            if (map_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", map_path.c_str());
                map_ = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
            }
            
            resolution_ = config["environment"]["resolution"].as<double>();
            canvas_ = std::make_unique<canvas::Canvas>(map_, resolution_);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load config: %s", e.what());
            map_ = cv::Mat(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
            canvas_ = std::make_unique<canvas::Canvas>(map_, resolution_);
        }
    }
    
    void robot_status_callback(const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        robot_state_.x = msg->x;
        robot_state_.y = msg->y;
        robot_state_.theta = msg->theta;
        robot_state_.velocity.linear = msg->linear_velocity;
        robot_state_.velocity.angular = msg->angular_velocity;
    }
    
    void game_state_callback(const robot_msgs::msg::GameState::SharedPtr msg) {
        game_state_ = *msg;
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidar_points_.clear();
        
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float range = msg->ranges[i];
            
            if (range > msg->range_min && range < msg->range_max) {
                double px = robot_state_.x + range * cos(robot_state_.theta + angle);
                double py = robot_state_.y + range * sin(robot_state_.theta + angle);
                lidar_points_.push_back({px, py});
            }
        }
    }
    
    void setup_terminal() {
        tcgetattr(STDIN_FILENO, &old_settings_);
        new_settings_ = old_settings_;
        new_settings_.c_lflag &= ~(ICANON | ECHO);
        new_settings_.c_cc[VMIN] = 0;
        new_settings_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings_);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    
    void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
    }
    
    void show_menu() {
        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════╗\n";
        std::cout << "║  🎮 INTERACTIVE TELEOP VISUALIZATION  ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "  W - Move FORWARD   \n";
        std::cout << "  S - Move BACKWARD  \n";
        std::cout << "  A - Turn LEFT      \n";
        std::cout << "  D - Turn RIGHT     \n";
        std::cout << "  Space - STOP       \n";
        std::cout << "  Q - QUIT           \n";
        std::cout << "\n";
        std::cout << "Visual feedback: OpenCV canvas display\n";
        std::cout << "════════════════════════════════════════\n\n";
        std::cout.flush();
    }
    
    void input_loop() {
        while (running_ && rclcpp::ok()) {
            int ch = getchar();
            
            if (ch != EOF && ch != -1) {
                auto twist = geometry_msgs::msg::Twist();
                bool send = false;
                std::string feedback;
                
                switch (ch) {
                    case 'W':
                    case 'w':
                        twist.linear.x = linear_speed_;
                        feedback = "⬆️  FORWARD";
                        send = true;
                        break;
                    case 'S':
                    case 's':
                        twist.linear.x = -linear_speed_;
                        feedback = "⬇️  BACKWARD";
                        send = true;
                        break;
                    case 'A':
                    case 'a':
                        twist.angular.z = angular_speed_;
                        feedback = "⬅️  LEFT";
                        send = true;
                        break;
                    case 'D':
                    case 'd':
                        twist.angular.z = -angular_speed_;
                        feedback = "➡️  RIGHT";
                        send = true;
                        break;
                    case ' ':
                        feedback = "⏹️  STOP";
                        send = true;
                        break;
                    case 'Q':
                    case 'q':
                        std::cout << "\n👋 QUITTING...\n";
                        running_ = false;
                        send = false;
                        break;
                    default:
                        send = false;
                }
                
                if (send) {
                    std::cout << "\r" << feedback << "                     " << std::flush;
                    cmd_vel_pub_->publish(twist);
                }
            }
            
            usleep(50000);
        }
    }
    
    void display_loop() {
        while (running_ && rclcpp::ok()) {
            try {
                if (!canvas_) continue;
                
                // Clear and redraw
                canvas_->clear();
                
                // Draw robot
                canvas_->drawRobot(robot_state_);
                
                // Draw LIDAR points
                if (!lidar_points_.empty()) {
                    canvas_->drawLidarPoints(lidar_points_);
                }
                
                // Draw info overlay
                cv::Mat display = canvas_->getMap().clone();
                
                // Info panel
                cv::putText(display, "INTERACTIVE TELEOP", cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
                
                cv::putText(display, 
                           "Pos: (" + std::to_string(static_cast<int>(robot_state_.x * 10) / 10.0) + ", " +
                           std::to_string(static_cast<int>(robot_state_.y * 10) / 10.0) + ")",
                           cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                
                cv::putText(display,
                           "Theta: " + std::to_string(static_cast<int>(robot_state_.theta * 180 / M_PI)),
                           cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                
                cv::putText(display,
                           "Vel: " + std::to_string(static_cast<int>(robot_state_.velocity.linear * 100) / 100.0),
                           cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                
                cv::putText(display,
                           "Score: " + std::to_string(game_state_.score),
                           cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                
                cv::imshow("Interactive Teleop", display);
                cv::waitKey(1);
                
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(this->get_logger(), "Display error: %s", e.what());
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
