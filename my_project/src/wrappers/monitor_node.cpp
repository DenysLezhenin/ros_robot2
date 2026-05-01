#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/game_state.hpp"
#include <iostream>
#include <iomanip>

class MonitorNode : public rclcpp::Node {
public:
    MonitorNode() : Node("monitor_node") {
        RCLCPP_INFO(this->get_logger(), "\n╔════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║     🤖 ROBOT VACUUM CLEANER - MONITOR (CONSOLE)      ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════╝\n");

        // Subscribe to topics
        robot_status_sub_ = this->create_subscription<robot_msgs::msg::RobotStatus>(
            "/robot_status", 10, 
            std::bind(&MonitorNode::robot_status_callback, this, std::placeholders::_1));
        
        game_state_sub_ = this->create_subscription<robot_msgs::msg::GameState>(
            "/game_state", 10,
            std::bind(&MonitorNode::game_state_callback, this, std::placeholders::_1));
        
        // Create display timer (1 Hz)
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MonitorNode::display_stats, this));

        RCLCPP_INFO(this->get_logger(), "✅ Monitor started - waiting for data...\n");
    }

private:
    rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<robot_msgs::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    // Robot state
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    int current_capacity_ = 0;
    int max_capacity_ = 0;

    // Game state
    int score_ = 0;
    int paper_count_ = 0;
    int plastic_count_ = 0;
    int glass_count_ = 0;
    std::vector<std::pair<double, double>> trash_positions_;
    std::vector<std::string> trash_types_;

    void robot_status_callback(const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        robot_x_ = msg->x;
        robot_y_ = msg->y;
        robot_theta_ = msg->theta;
        linear_vel_ = msg->linear_velocity;
        angular_vel_ = msg->angular_velocity;
        current_capacity_ = msg->current_capacity;
        max_capacity_ = msg->max_capacity;
    }

    void game_state_callback(const robot_msgs::msg::GameState::SharedPtr msg) {
        score_ = msg->score;
        paper_count_ = msg->paper_count;
        plastic_count_ = msg->plastic_count;
        glass_count_ = msg->glass_count;

        // Extract trash positions
        trash_positions_.clear();
        trash_types_.clear();
        for (const auto& item : msg->trash_items) {
            trash_positions_.push_back({item.x, item.y});
            trash_types_.push_back(item.trash_type);
        }
    }

    void display_stats() {
        // Clear screen
        system("clear");

        std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
        std::cout << "║          🤖 ROBOT VACUUM CLEANER - LIVE DATA          ║\n";
        std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

        // ROBOT STATUS
        std::cout << "📍 ROBOT STATUS:\n";
        std::cout << "  ├─ Position:  X=" << std::fixed << std::setprecision(2) 
                  << std::setw(8) << robot_x_ << "  Y=" << std::setw(8) << robot_y_ << "\n";
        std::cout << "  ├─ Rotation:  θ=" << std::setw(8) << (robot_theta_ * 180.0 / M_PI) << "° (radians: " 
                  << robot_theta_ << ")\n";
        std::cout << "  ├─ Velocity:  V=" << std::setw(8) << linear_vel_ << " m/s  ω=" 
                  << std::setw(8) << angular_vel_ << " rad/s\n";
        std::cout << "  └─ Battery:   " << current_capacity_ << "/" << max_capacity_;
        
        // Battery bar
        std::cout << " [";
        int filled = max_capacity_ > 0 ? (current_capacity_ * 20) / max_capacity_ : 0;
        for (int i = 0; i < 20; i++) {
            std::cout << (i < filled ? "█" : "░");
        }
        std::cout << "]\n\n";

        // GAME STATE
        std::cout << "🎮 GAME STATE:\n";
        std::cout << "  ├─ Score:      " << score_ << " points\n";
        std::cout << "  ├─ Items collected:\n";
        std::cout << "  │  ├─ 📄 Paper:   " << paper_count_ << "\n";
        std::cout << "  │  ├─ 🔵 Plastic: " << plastic_count_ << "\n";
        std::cout << "  │  └─ 🔴 Glass:   " << glass_count_ << "\n";
        std::cout << "  └─ Total waste on map: " << trash_positions_.size() << " items\n\n";

        // TRASH ON MAP
        if (!trash_positions_.empty()) {
            std::cout << "🗑️  TRASH ITEMS ON MAP:\n";
            for (size_t i = 0; i < trash_positions_.size() && i < 10; i++) {
                std::string icon = "❓";
                if (trash_types_[i] == "paper") icon = "📄";
                else if (trash_types_[i] == "plastic") icon = "🔵";
                else if (trash_types_[i] == "glass") icon = "🔴";

                std::cout << "  [" << std::setfill('0') << std::setw(2) << (i + 1) 
                          << "] " << icon << " " << std::setfill(' ') 
                          << std::setw(8) << trash_types_[i] 
                          << " at X=" << std::setw(7) << std::fixed << std::setprecision(1) 
                          << trash_positions_[i].first 
                          << "  Y=" << std::setw(7) << trash_positions_[i].second << "\n";
            }
            if (trash_positions_.size() > 10) {
                std::cout << "  ... and " << (trash_positions_.size() - 10) << " more items\n";
            }
            std::cout << "\n";
        }

        // DISTANCE TO TRASH
        if (!trash_positions_.empty()) {
            std::cout << "📏 DISTANCES TO NEAREST ITEMS:\n";
            std::vector<std::pair<double, size_t>> distances;
            for (size_t i = 0; i < trash_positions_.size(); i++) {
                double dx = trash_positions_[i].first - robot_x_;
                double dy = trash_positions_[i].second - robot_y_;
                double dist = std::sqrt(dx * dx + dy * dy);
                distances.push_back({dist, i});
            }
            std::sort(distances.begin(), distances.end());
            
            for (size_t i = 0; i < std::min(size_t(5), distances.size()); i++) {
                size_t idx = distances[i].second;
                double dist = distances[i].first;
                std::string icon = "❓";
                if (trash_types_[idx] == "paper") icon = "📄";
                else if (trash_types_[idx] == "plastic") icon = "🔵";
                else if (trash_types_[idx] == "glass") icon = "🔴";
                
                std::cout << "  " << (i + 1) << ". " << icon << " " 
                          << std::setw(8) << trash_types_[idx] << " - " 
                          << std::fixed << std::setprecision(2) << std::setw(7) << dist << " m away\n";
            }
            std::cout << "\n";
        }

        // STATUS LINE
        std::cout << "═══════════════════════════════════════════════════════\n";
        std::cout << "⏱️  Last update: " << std::fixed << std::setprecision(1) 
                  << this->get_clock()->now().seconds() << "s\n";
        std::cout << "💡 Press Ctrl+C to stop\n\n";
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
