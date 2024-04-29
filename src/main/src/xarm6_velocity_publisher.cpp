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
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/xarm6_velo_traj_controller/commands", 10);

    RCLCPP_INFO(node->get_logger(), "Velocity Publisher Node Started");

    std_msgs::msg::Float64MultiArray commands;
    // Properly configuring the layout dimensions
    commands.layout.dim.resize(1);
    commands.layout.dim[0].label = "joints";
    commands.layout.dim[0].size = 6;
    commands.layout.dim[0].stride = 6;
    commands.layout.data_offset = 0;
    commands.data.resize(6);

    int state = 0;
    rclcpp::Time last_time = node->now();
    rclcpp::Duration interval(3s);  // Time to spend in each state

    while (rclcpp::ok()) {
        // Check if it's time to change the state
        if((node->now() - last_time) > interval) {
            state = (state + 1) % 4;  // Cycle through 4 states
            last_time = node->now();

            // Set velocities according to the current state
            switch (state) {
                case 0:  // Move to the first point
                    commands.data[0] = 0.5;
                    commands.data[1] = 0.5;
                    commands.data[2] = 0.0;
                    commands.data[3] = 0.0;
                    commands.data[4] = 0.0;
                    commands.data[5] = 0.0;
                    break;
                case 1:  // Move to the second point
                    commands.data[0] = 0.0;
                    commands.data[1] = 0.0;
                    commands.data[2] = 0.5;
                    commands.data[3] = 0.5;
                    commands.data[4] = 0.0;
                    commands.data[5] = 0.0;
                    break;
                case 2:  // Move to the third point
                    commands.data[0] = -0.5;
                    commands.data[1] = -0.5;
                    commands.data[2] = 0.0;
                    commands.data[3] = 0.0;
                    commands.data[4] = 0.0;
                    commands.data[5] = 0.0;
                    break;
                case 3:  // Move to the fourth point
                    commands.data[0] = 0.0;
                    commands.data[1] = 0.0;
                    commands.data[2] = -0.5;
                    commands.data[3] = -0.5;
                    commands.data[4] = 0.0;
                    commands.data[5] = 0.0;
                    break;
            }
        }

        publisher->publish(commands);
        RCLCPP_INFO(node->get_logger(), "Published velocity commands.");
        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    return 0;
}
