#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("robot_controller_node") {
        // Initialize service clients in the constructor
        load_controller_client_ = this->create_client<controller_manager_msgs::srv::LoadController>("/controller_manager/load_controller");
        configure_controller_client_ = this->create_client<controller_manager_msgs::srv::ConfigureController>("/controller_manager/configure_controller");
        switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    }

    void initialize() {
        // Load and configure controller
        auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
        load_request->name = "your_controller_name";
        auto load_future = load_controller_client_->async_send_request(load_request);

        // Wait for the controller to be loaded
        if (rclcpp::spin_until_future_complete(shared_from_this(), load_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Controller loaded successfully.");
            
            auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
            configure_request->name = load_request->name;
            auto configure_future = configure_controller_client_->async_send_request(configure_request);

            // Wait for the controller to be configured
            if (rclcpp::spin_until_future_complete(shared_from_this(), configure_future) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Controller configured successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure controller.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load controller.");
        }
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    node->initialize();  // Call initialize after the node has been created
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
