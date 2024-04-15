#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("velocity_publisher_node");
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

    RCLCPP_INFO(node->get_logger(), "Velocity Publisher Node Started");

    std_msgs::msg::Float64MultiArray commands;
    commands.data.resize(6);  // Resize to match the number of joints you want to control

    while (rclcpp::ok()) {
        // Example velocities for each joint. Adjust these values as necessary.
        commands.data[0] = 0.5;  // velocity for joint1
        commands.data[1] = -0.5; // velocity for joint2
        commands.data[2] = 0.3;  // velocity for joint3
        commands.data[3] = -0.3; // velocity for joint4
        commands.data[4] = 0.1;  // velocity for joint5
        commands.data[5] = -0.1; // velocity for joint6

        publisher->publish(commands);
        RCLCPP_INFO(node->get_logger(), "Published velocity commands.");

        std::this_thread::sleep_for(1s);  // Send commands every 1 second

        // Setting velocities back to zero
        for (auto& vel : commands.data) {
            vel = 0.0;
        }
        publisher->publish(commands);
        RCLCPP_INFO(node->get_logger(), "Stopping all joints.");

        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    return 0;
}
