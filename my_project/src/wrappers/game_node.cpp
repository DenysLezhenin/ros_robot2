#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/game_state.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/trash_item.hpp"
#include "my_project/core/Waste.hpp"
#include "my_project/core/Environment.hpp"
#include "my_project/core/Geometry.hpp"
#include <random>
#include <yaml-cpp/yaml.h>

class GameNode : public rclcpp::Node {
public:
    GameNode() : Node("game_node") {
        // Load configuration
        this->declare_parameter("config_file", "/home/ros/ros2_ws/config/game_config.yaml");
        std::string config_file = this->get_parameter("config_file").as_string();
        
        YAML::Node config = YAML::LoadFile(config_file);
        
        // Load parameters
        max_capacity_ = config["robot"]["max_capacity"].as<int>(10);
        min_trash_radius_ = config["trash"]["min_radius"].as<double>(0.1);
        max_trash_radius_ = config["trash"]["max_radius"].as<double>(0.3);
        generation_interval_ms_ = config["game"]["trash_generation_interval_ms"].as<int>(2000);
        
        // Load environment
        env_ = std::make_shared<environment::Environment>();
        if (config["environment"] && config["environment"]["map_config"]) {
            std::string map_config = config["environment"]["map_config"].as<std::string>();
            env_->loadFromFile(map_config);
        }
        
        station_x_ = config["environment"]["station"]["x"].as<double>();
        station_y_ = config["environment"]["station"]["y"].as<double>();
        station_radius_ = config["environment"]["station"]["radius"].as<double>();
        
        // Create subscriptions
        robot_status_sub_ = this->create_subscription<robot_msgs::msg::RobotStatus>(
            "/robot_status", 10, std::bind(&GameNode::robot_status_callback, this, std::placeholders::_1));
        
        // Create publishers
        game_state_pub_ = this->create_publisher<robot_msgs::msg::GameState>("/game_state", 10);
        
        // Create timers
        trash_generator_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(generation_interval_ms_),
            std::bind(&GameNode::generate_trash, this));
        
        state_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GameNode::publish_game_state, this));
        
        RCLCPP_INFO(this->get_logger(), "GameNode initialized - Trash Generation Started");
        
        // Generate initial trash
        generate_trash();
        generate_trash();
    }

private:
    std::shared_ptr<environment::Environment> env_;
    std::vector<std::shared_ptr<Waste>> trash_items_;
    
    geometry::RobotState current_robot_state_;
    int current_capacity_ = 0;
    int max_capacity_;
    int score_ = 0;
    double distance_traveled_ = 0.0;
    geometry::RobotState last_robot_pos_;
    bool first_update_ = true;
    
    // Config parameters
    double min_trash_radius_;
    double max_trash_radius_;
    int generation_interval_ms_;
    double station_x_;
    double station_y_;
    double station_radius_;
    
    // Waste counters
    int paper_collected_ = 0;
    int plastic_collected_ = 0;
    int glass_collected_ = 0;
    
    rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<robot_msgs::msg::GameState>::SharedPtr game_state_pub_;
    rclcpp::TimerBase::SharedPtr trash_generator_timer_;
    rclcpp::TimerBase::SharedPtr state_publisher_timer_;
    
    void robot_status_callback(const robot_msgs::msg::RobotStatus::SharedPtr msg) {
        if (first_update_) {
            last_robot_pos_ = {msg->x, msg->y, msg->theta, {}};
            first_update_ = false;
        } else {
            // Calculate distance traveled
            double dx = msg->x - last_robot_pos_.x;
            double dy = msg->y - last_robot_pos_.y;
            distance_traveled_ += std::sqrt(dx * dx + dy * dy);
            last_robot_pos_ = {msg->x, msg->y, msg->theta, {}};
        }
        
        current_robot_state_ = {msg->x, msg->y, msg->theta, {msg->linear_velocity, msg->angular_velocity}};
        
        // Check if robot is at station
        double dist_to_station = std::sqrt(
            std::pow(current_robot_state_.x - station_x_, 2) +
            std::pow(current_robot_state_.y - station_y_, 2)
        );
        
        if (dist_to_station < station_radius_ && current_capacity_ > 0) {
            // Unload trash at station
            score_ += current_capacity_ * 10;
            RCLCPP_INFO(this->get_logger(), "Trash unloaded at station! Score: %d", score_);
            current_capacity_ = 0;
        }
        
        // Check for trash collection
        check_trash_collection();
    }
    
    void check_trash_collection() {
        const double collection_radius = 0.5;
        
        auto it = trash_items_.begin();
        while (it != trash_items_.end()) {
            double dist = std::sqrt(
                std::pow(current_robot_state_.x - (*it)->x, 2) +
                std::pow(current_robot_state_.y - (*it)->y, 2)
            );
            
            if (dist < collection_radius && current_capacity_ < max_capacity_) {
                // Collect trash
                Waste::Type type = (*it)->getType();
                switch (type) {
                    case Waste::PAPER:
                        paper_collected_++;
                        break;
                    case Waste::PLASTIC:
                        plastic_collected_++;
                        break;
                    case Waste::GLASS:
                        glass_collected_++;
                        break;
                }
                
                current_capacity_++;
                RCLCPP_DEBUG(this->get_logger(), "Collected %s - Capacity: %d/%d",
                           (*it)->getTypeName().c_str(), current_capacity_, max_capacity_);
                
                it = trash_items_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void generate_trash() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        
        std::uniform_real_distribution<> x_dist(1.0, env_->getWidth() - 1.0);
        std::uniform_real_distribution<> y_dist(1.0, env_->getHeight() - 1.0);
        std::uniform_real_distribution<> radius_dist(min_trash_radius_, max_trash_radius_);
        
        bool valid_position = false;
        double x, y;
        
        // Try to find a valid position
        for (int attempts = 0; attempts < 10; ++attempts) {
            x = x_dist(gen);
            y = y_dist(gen);
            
            if (!env_->isOccupied(x, y)) {
                valid_position = true;
                break;
            }
        }
        
        if (valid_position) {
            double radius = radius_dist(gen);
            auto trash = createRandomWaste(x, y, radius);
            trash_items_.push_back(trash);
            RCLCPP_DEBUG(this->get_logger(), "Generated %s at (%.2f, %.2f)",
                        trash->getTypeName().c_str(), x, y);
        }
    }
    
    void publish_game_state() {
        auto state = robot_msgs::msg::GameState();
        state.score = score_;
        state.wave = 0;  // Can be enhanced for wave-based modes
        state.paper_count = paper_collected_;
        state.plastic_count = plastic_collected_;
        state.glass_count = glass_collected_;
        state.current_capacity = current_capacity_;
        state.max_capacity = max_capacity_;
        state.distance_traveled = distance_traveled_;
        state.game_over = false;
        state.result_message = "Game in progress";
        
        // Add trash items to state
        for (const auto& trash : trash_items_) {
            robot_msgs::msg::TrashItem item;
            item.id = trash->getId();
            item.trash_type = trash->getTypeName();
            item.x = trash->x;
            item.y = trash->y;
            item.radius = trash->radius;
            state.trash_items.push_back(item);
        }
        
        game_state_pub_->publish(state);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GameNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
