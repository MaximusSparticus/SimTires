#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "chrono_ros_interface/msg/driver_inputs.hpp"

class TeleopDriver : public rclcpp::Node {
 public:
    TeleopDriver()
    : Node("teleop_driver") {
        // Get the topic name from the parameter
        std::string topic_name;
        this->declare_parameter<std::string>("topic_name", "/driver_inputs");
        this->get_parameter("topic_name", topic_name);

        // Create publisher
        m_publisher = this->create_publisher<chrono_ros_interface::msg::DriverInputs>(topic_name, 10);
    
        // Create subscriber for joystick inputs
        m_joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&TeleopDriver::joyCallback, this, std::placeholders::_1));
    }

 private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto driver_inputs = chrono_ros_interface::msg::DriverInputs();
        driver_inputs.header.stamp = this->get_clock()->now();
        driver_inputs.steering = msg->axes[0];  // Assuming axes[0] for steering
        driver_inputs.throttle = msg->axes[1];  // Assuming axes[1] for throttle
        driver_inputs.braking = msg->axes[2];   // Assuming axes[2] for braking
        driver_inputs.clutch = msg->axes[3];    // Assuming axes[3] for clutch

        m_publisher->publish(driver_inputs);
    }

    rclcpp::Publisher<chrono_ros_interface::msg::DriverInputs>::SharedPtr m_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_subscription;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopDriver>());
    rclcpp::shutdown();
    return 0;
}
