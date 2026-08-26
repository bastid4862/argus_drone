#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/empty.hpp"

#include "cave_drone_interfaces/msg/sweep_command.hpp"
#include "cave_drone_interfaces/srv/set_angle.hpp"

#include <cstdint>

#include "../../../build/cave_drone_interfaces/rosidl_generator_cpp/cave_drone_interfaces/srv/detail/set_angle__struct.hpp"
#include "../../../build/cave_drone_interfaces/rosidl_typesupport_fastrtps_cpp/cave_drone_interfaces/msg/detail/sweep_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cave_drone_interfaces/srv/sweep.hpp"

class ScanHeadBridgeNode : public rclcpp::Node {
public:
    ScanHeadBridgeNode() : Node("scan_head_bridge") {
        // Publishers:

        // Send target angle commands to the Teensy
        set_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/set_angle",
            10
        );

        // Send HOME command
        home_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/home",
            10
        );

        // Send STOP command
        stop_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/stop",
            10
        );

        // Send SWEEP command
        sweep_pub_ =
                this->create_publisher<cave_drone_interfaces::msg::SweepCommand>(
                    "/sweep",
                    10
                );

        // Send fault reset command
        reset_fault_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/reset_fault",
            10
        );

        // Subscribers:

        // Receive encoder angle from the Teensy
        encoder_angle_sub_ =
                this->create_subscription<std_msgs::msg::Float32>(
                    "/encoder_angle",
                    10,
                    std::bind(
                        &ScanHeadBridgeNode::encoder_angle_callback,
                        this,
                        std::placeholders::_1
                    )
                );

        // Receive current scan mode from the Teensy
        scan_mode_sub_ =
                this->create_subscription<std_msgs::msg::Int8>(
                    "/scan_mode",
                    10,
                    std::bind(
                        &ScanHeadBridgeNode::scan_mode_callback,
                        this,
                        std::placeholders::_1
                    )
                );

        // HOME service
        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scan_head/home",
            std::bind(
                &ScanHeadBridgeNode::home_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        // STOP service
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scan_head/stop",
            std::bind(
                &ScanHeadBridgeNode::stop_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        // RESET_FAULT service
        reset_fault_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/scan_head/reset_fault",
            std::bind(
                &ScanHeadBridgeNode::reset_fault_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        // SET_ANGLE Service
        set_angle_srv_ = this->create_service<cave_drone_interfaces::srv::SetAngle>(
            "/scan_head/set_angle",
            std::bind(
                &ScanHeadBridgeNode::set_angle_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        // SWEEP Service
        sweep_service_ = this -> create_service<cave_drone_interfaces::srv::Sweep>(
            "/scan_head/sweep",
            std::bind(
                &ScanHeadBridgeNode::sweep_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        // Higher-Level Encoder Status Publisher
        encoder_angle_pub_ = this -> create_publisher<std_msgs::msg::Float32>(
            "/scan_head/encoder_angle",
            10
        );

        // Higher-Level Mode Publisher
        mode_pub_ = this -> create_publisher<std_msgs::msg::Int8>(
            "/scan_head/mode",
            10
        );
    }

    void publish_set_angle(float angle_value) {
        // Create ROS 2 message
        std_msgs::msg::Float32 msg;

        // Put angle into message
        msg.data = angle_value;

        // Send command
        set_angle_pub_->publish(msg);
    }

    void publish_home() {
        // HOME does not need any data
        std_msgs::msg::Empty msg;

        home_pub_->publish(msg);
    }


    void publish_stop() {
        // STOP does not need any data
        std_msgs::msg::Empty msg;

        stop_pub_->publish(msg);
    }

    void publish_sweep(
        float min_angle,
        float max_angle,
        float speed,
        int8_t direction) {
        // Create custom sweep message
        cave_drone_interfaces::msg::SweepCommand msg;

        // Fill message fields
        msg.min_angle = min_angle;
        msg.max_angle = max_angle;
        msg.speed = speed;
        msg.direction = direction;

        // Send sweep command
        sweep_pub_->publish(msg);
    }


    void publish_reset_fault() {
        // RESET does not need any data
        std_msgs::msg::Empty msg;

        reset_fault_pub_->publish(msg);
    }

private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr scan_mode_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr set_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr home_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
    rclcpp::Publisher<cave_drone_interfaces::msg::SweepCommand>::SharedPtr sweep_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_fault_pub_;

    // Higher-level Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mode_pub_;

    // Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
    rclcpp::Service<cave_drone_interfaces::srv::SetAngle>::SharedPtr set_angle_srv_;
    rclcpp::Service<cave_drone_interfaces::srv::Sweep>::SharedPtr sweep_service_;

    // Teensy States
    float latest_encoder_angle_ = 0.0f; // stores angle
    int8_t latest_scan_mode_ = 0; // stores mode number

    // Callbacks
    void encoder_angle_callback(
        const std_msgs::msg::Float32::SharedPtr msg) {
        // Save latest encoder angle
        latest_encoder_angle_ = msg->data;
        // Republish the encoder angle as higher-level scan head status
        encoder_angle_pub_->publish(*msg);
    }


    void scan_mode_callback(
        const std_msgs::msg::Int8::SharedPtr msg) {
        // Save latest scan mode
        latest_scan_mode_ = msg->data;
        // Republish the mode as higher-level scan head status
        mode_pub_->publish(*msg);
    }

    void home_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Request has no data
        (void) request;

        // Send HOME command to Teensy
        publish_home();

        // Tell caller the request was handled
        response->success = true;
        response->message = "Home command sent";
    }

    void stop_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Request has no data
        (void) request;

        // Send STOP command to Teensy
        publish_stop();

        response->success = true;
        response->message = "Stop command sent";
    }

    void reset_fault_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Request has no data
        (void)request;

        // Send RESET FAULT command to Teensy
        publish_reset_fault();
        response->success = true;
        response->message = "Reset command sent";
    }

    void set_angle_service_callback(const std::shared_ptr<cave_drone_interfaces::srv::SetAngle::Request> request,
                                        std::shared_ptr<cave_drone_interfaces::srv::SetAngle::Response> response) {
        // Send requested angle to Teensy
        publish_set_angle(request->angle);

        // Tell caller the command was handled
        response->success = true;
        response->message = "Angle command sent";


    }

    void sweep_service_callback(const std::shared_ptr<cave_drone_interfaces::srv::Sweep::Request> request,
                                    std::shared_ptr<cave_drone_interfaces::srv::Sweep::Response> response) {

        // Forward the requested sweep settings to the Teensy
        publish_sweep(request->min_angle, request -> max_angle, request -> speed, request -> direction);

        // Tell the caller the command was sent
        response->success = true;
        response->message = "Sweep command sent";
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanHeadBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
