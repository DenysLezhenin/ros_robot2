#include "rclcpp/rclcpp.hpp"


class EnvironmentNode : public rclcpp::Node{

}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}

