#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node") {
        // Create publisher for /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Declare parameters
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 1.0);
        
        // Get parameters
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        
        // Setup terminal
        setup_terminal();
        
        // Show controls
        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════╗\n";
        std::cout << "║     ⌨️  ROBOT TELEOPERATION CONTROL   ║\n";
        std::cout << "╚════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "  W - Move FORWARD   (v = " << linear_speed_ << " m/s)\n";
        std::cout << "  S - Move BACKWARD  (v = " << -linear_speed_ << " m/s)\n";
        std::cout << "  A - Turn LEFT      (ω = " << angular_speed_ << " rad/s)\n";
        std::cout << "  D - Turn RIGHT     (ω = " << -angular_speed_ << " rad/s)\n";
        std::cout << "  Space - STOP\n";
        std::cout << "  Q - QUIT\n";
        std::cout << "\n";
        std::cout << "Ready! Press keys to control robot...\n";
        std::cout << "════════════════════════════════════════\n\n";
        std::cout.flush();
        
        // Start input thread
        input_thread_ = std::thread(&TeleopNode::input_loop, this);
    }
    
    ~TeleopNode() {
        restore_terminal();
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }
    
    bool is_running() const { return running_; }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_speed_;
    double angular_speed_;
    std::thread input_thread_;
    bool running_ = true;
    struct termios old_settings_, new_settings_;
    
    void setup_terminal() {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &old_settings_);
        new_settings_ = old_settings_;
        
        // Disable canonical mode and echo
        new_settings_.c_lflag &= ~(ICANON | ECHO);
        
        // Set minimum chars to read and timeout
        new_settings_.c_cc[VMIN] = 0;  // Non-blocking
        new_settings_.c_cc[VTIME] = 0;
        
        // Apply new settings
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings_);
        
        // Also set non-blocking via fcntl
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    
    void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings_);
    }
    
    void input_loop() {
        while (running_ && rclcpp::ok()) {
            int ch = getchar();
            
            if (ch != EOF && ch != -1) {
                auto twist = geometry_msgs::msg::Twist();
                bool should_quit = false;
                std::string feedback;
                
                switch (ch) {
                    case 'w':
                    case 'W':
                        twist.linear.x = linear_speed_;
                        feedback = "⬆️  FORWARD";
                        break;
                    case 's':
                    case 'S':
                        twist.linear.x = -linear_speed_;
                        feedback = "⬇️  BACKWARD";
                        break;
                    case 'a':
                    case 'A':
                        twist.angular.z = angular_speed_;
                        feedback = "⬅️  LEFT";
                        break;
                    case 'd':
                    case 'D':
                        twist.angular.z = -angular_speed_;
                        feedback = "➡️  RIGHT";
                        break;
                    case ' ':
                        feedback = "⏹️  STOP";
                        break;
                    case 'q':
                    case 'Q':
                        feedback = "👋 QUITTING...";
                        should_quit = true;
                        running_ = false;
                        break;
                    default:
                        break;
                }
                
                if (!feedback.empty()) {
                    std::cout << "\r" << feedback << "                     " << std::flush;
                    publisher_->publish(twist);
                }
                
                if (should_quit) {
                    std::cout << "\n";
                    break;
                }
            }
            
            usleep(50000);  // 50ms - smooth feedback
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    
    // Keep node alive while input thread is running
    while (rclcpp::ok() && node->is_running()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    rclcpp::shutdown();
    return 0;
}