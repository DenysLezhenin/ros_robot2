#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TestPublisher : public rclcpp::Node {
public:
    TestPublisher() : Node("test_cmd_publisher") {

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot_command", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TestPublisher::publishCommand, this)
        );
    }

private:
    void publishCommand() {

        geometry_msgs::msg::Twist msg;

        // 🔁 міняємо поведінку
        static int step = 0;

        if(step < 10){
            msg.linear.x = 1.0;   // вперед
            msg.angular.z = 0.0;
        }
        else if(step < 20){
            msg.linear.x = 0.0;
            msg.angular.z = 1.0;  // обертання
        }
        else{
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;  // стоп
        }

        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "Send: linear=%.2f angular=%.2f",
            msg.linear.x, msg.angular.z
        );

        step++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
    return 0;
}