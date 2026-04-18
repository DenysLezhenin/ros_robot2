#include "my_project/core/Environment.hpp"

int main() {
    Environment env;
    env.loadFromFile("/home/ros/ros2_ws/src/my_project/config/config.yaml");
    return 0;
}