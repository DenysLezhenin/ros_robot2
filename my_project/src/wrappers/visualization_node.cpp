#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/game_state.hpp"
#include "my_project/core/Environment.hpp"
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <iomanip>
#include <sstream>

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("visualization_node") {
        // Disable Qt platform for headless environments
        cv::setUseOptimized(false);
        
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
        
        // Load station position
        station_x_ = config["environment"]["station"]["x"].as<double>();
        station_y_ = config["environment"]["station"]["y"].as<double>();
        station_radius_ = config["environment"]["station"]["radius"].as<double>();
        
        // Get map dimensions
        map_width_ = static_cast<int>(env_->getWidth());
        map_height_ = static_cast<int>(env_->getHeight());
        
        RCLCPP_INFO(this->get_logger(), "Map size: %dx%d", map_width_, map_height_);
        
        // Create subscriptions
        robot_status_sub_ = this->create_subscription<robot_msgs::msg::RobotStatus>(
            "/robot_status", 10, std::bind(&VisualizationNode::robot_status_callback, this, std::placeholders::_1));
        
        game_state_sub_ = this->create_subscription<robot_msgs::msg::GameState>(
            "/game_state", 10, std::bind(&VisualizationNode::game_state_callback, this, std::placeholders::_1));
        
        // Create visualization timer
        viz_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&VisualizationNode::draw_visualization, this));
        
        // Create stats timer (1 Hz)
        stats_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&VisualizationNode::print_stats, this));
        
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "🤖 ROBOT VACUUM CLEANER VISUALIZATION");
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
    }

private:
    std::shared_ptr<environment::Environment> env_;
    rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<robot_msgs::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // Robot state
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    double robot_linear_vel_ = 0.0;
    double robot_angular_vel_ = 0.0;
    
    // Game state
    int score_ = 0;
    int current_capacity_ = 0;
    int max_capacity_ = 0;
    int paper_count_ = 0;
    int plastic_count_ = 0;
    int glass_count_ = 0;
    
    // Map
    int map_width_ = 0;
    int map_height_ = 0;
    double station_x_ = 0.0;
    double station_y_ = 0.0;
    double station_radius_ = 1.0;
    
    // Trash items
    std::vector<std::tuple<double, double, std::string>> trash_items_;
    
    // Robot path
    std::deque<cv::Point> robot_path_;
    static const size_t MAX_PATH_POINTS = 1000;
    
    void robot_status_callback(const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        robot_x_ = msg->x;
        robot_y_ = msg->y;
        robot_theta_ = msg->theta;
        robot_linear_vel_ = msg->linear_velocity;
        robot_angular_vel_ = msg->angular_velocity;
        current_capacity_ = msg->current_capacity;
        max_capacity_ = msg->max_capacity;
    }
    
    void game_state_callback(const robot_msgs::msg::GameState::SharedPtr msg) {
        score_ = msg->score;
        paper_count_ = msg->paper_count;
        plastic_count_ = msg->plastic_count;
        glass_count_ = msg->glass_count;
        
        // Update trash items
        trash_items_.clear();
        for (const auto& item : msg->trash_items) {
            trash_items_.push_back({item.x, item.y, item.trash_type});
        }
    }
    
    void print_stats() {
        std::stringstream ss;
        ss << "📊 STATS | Score: " << score_ 
           << " | Capacity: " << current_capacity_ << "/" << max_capacity_
           << " | Paper: " << paper_count_ << " | Plastic: " << plastic_count_ 
           << " | Glass: " << glass_count_
           << " | Pos: (" << std::fixed << std::setprecision(1) 
           << robot_x_ << "," << robot_y_ << ")"
           << " | Vel: " << std::setprecision(2) << robot_linear_vel_ << " m/s";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
    
    void draw_visualization() {
        // Create visualization image
        cv::Mat viz = env_->getColorMap().clone();
        
        if (viz.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map image is empty");
            return;
        }
        
        // Scale for display
        int display_width = std::min(1024, viz.cols * 2);
        int display_height = std::min(768, viz.rows * 2);
        cv::Mat display;
        cv::resize(viz, display, cv::Size(display_width, display_height));
        
        double scale_x = static_cast<double>(display_width) / viz.cols;
        double scale_y = static_cast<double>(display_height) / viz.rows;
        
        // Draw station (green circle)
        cv::Point station_center(
            static_cast<int>(station_x_ * scale_x),
            static_cast<int>(station_y_ * scale_y)
        );
        int station_radius_px = std::max(5, static_cast<int>(station_radius_ * scale_x));
        cv::circle(display, station_center, station_radius_px, cv::Scalar(0, 255, 0), -1);
        cv::circle(display, station_center, station_radius_px, cv::Scalar(0, 0, 0), 2);
        
        // Draw trash items
        for (const auto& [x, y, type] : trash_items_) {
            cv::Point trash_pos(static_cast<int>(x * scale_x), static_cast<int>(y * scale_y));
            cv::Scalar color;
            
            if (type == "paper") {
                color = cv::Scalar(200, 200, 100);  // Light blue
            } else if (type == "plastic") {
                color = cv::Scalar(100, 200, 200);  // Cyan
            } else if (type == "glass") {
                color = cv::Scalar(150, 150, 255);  // Light red
            } else {
                color = cv::Scalar(200, 200, 200);  // Gray
            }
            
            cv::circle(display, trash_pos, 3, color, -1);
            cv::circle(display, trash_pos, 3, cv::Scalar(0, 0, 0), 1);
        }
        
        // Draw robot path
        if (robot_path_.size() > 1) {
            for (size_t i = 1; i < robot_path_.size(); ++i) {
                cv::line(display, robot_path_[i-1], robot_path_[i], cv::Scalar(150, 150, 150), 1);
            }
        }
        
        // Add current robot position to path
        cv::Point robot_pos(static_cast<int>(robot_x_ * scale_x), static_cast<int>(robot_y_ * scale_y));
        if (robot_path_.empty() || cv::norm(robot_path_.back() - robot_pos) > 2) {
            robot_path_.push_back(robot_pos);
            if (robot_path_.size() > MAX_PATH_POINTS) {
                robot_path_.pop_front();
            }
        }
        
        // Draw robot (red circle with direction indicator)
        cv::circle(display, robot_pos, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(display, robot_pos, 5, cv::Scalar(0, 0, 0), 1);
        
        // Draw direction indicator
        int arrow_length = 15;
        cv::Point arrow_end(
            robot_pos.x + static_cast<int>(arrow_length * cos(robot_theta_)),
            robot_pos.y + static_cast<int>(arrow_length * sin(robot_theta_))
        );
        cv::arrowedLine(display, robot_pos, arrow_end, cv::Scalar(255, 0, 0), 2);
        
        // Create info panel
        cv::Mat info_panel(200, display_width, CV_8UC3, cv::Scalar(50, 50, 50));
        
        // Draw info text
        int y_offset = 25;
        int line_height = 20;
        
        putText(info_panel, "=== ROBOT VACUUM CLEANER ===", cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        
        y_offset += line_height + 5;
        putText(info_panel, "Score: " + std::to_string(score_), cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        
        y_offset += line_height;
        putText(info_panel, "Capacity: " + std::to_string(current_capacity_) + "/" + std::to_string(max_capacity_), 
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 200, 0), 1);
        
        y_offset += line_height;
        putText(info_panel, "Paper: " + std::to_string(paper_count_) + 
                " | Plastic: " + std::to_string(plastic_count_) + 
                " | Glass: " + std::to_string(glass_count_), 
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        y_offset += line_height;
        putText(info_panel, "Pos: (" + std::to_string(static_cast<int>(robot_x_)) + ", " + 
                std::to_string(static_cast<int>(robot_y_)) + ") Theta: " + 
                std::to_string(static_cast<int>(robot_theta_ * 180 / M_PI)) + "°",
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 200, 255), 1);
        
        y_offset += line_height;
        putText(info_panel, "Vel: " + std::to_string(static_cast<int>(robot_linear_vel_ * 100) / 100.0) + 
                " m/s | Angular: " + std::to_string(static_cast<int>(robot_angular_vel_ * 100) / 100.0) + " rad/s",
                cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 200, 255), 1);
        
        // Combine display and info panel
        cv::Mat final_display;
        cv::vconcat(display, info_panel, final_display);
        
        // Always save frames
        static int frame_count = 0;
        if (frame_count % 5 == 0) {
            try {
                std::string filename = "/tmp/robot_viz_" + std::to_string(frame_count / 5) + ".png";
                cv::imwrite(filename, final_display);
            } catch (...) {
                // Ignore file write errors
            }
        }
        frame_count++;
        
        // Try to display if available (not critical for headless)
        try {
            cv::imshow("Robot Vacuum Cleaner - Visualization", final_display);
            cv::waitKey(1);
        } catch (...) {
            // Headless environment - no display available, but frames are saved
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
