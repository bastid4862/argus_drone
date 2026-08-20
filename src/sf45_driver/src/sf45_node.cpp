#include "rclcpp/rclcpp.hpp"
#include "cave_drone_interfaces/msg/sf45_measurement.hpp"
#include <chrono>

using namespace std:: chrono_literals;

class SF45DriverNode: public rclcpp::Node{
public:
    SF45DriverNode(): Node("sf45_driver_node"){
        publisher_ = this -> create_publisher<cave_drone_interfaces::msg::Sf45Measurement>("sf45/measurements", 10);
        timer_ = this -> create_wall_timer(
            100ms,
            [this]() -> void{
                auto message = cave_drone_interfaces::msg::Sf45Measurement();
                message.distance = 5.2f;
                message.internal_scan_angle = 0.0f;
                message.measurement_status = 1;
                message.timestamp = this->now();
                publisher_ -> publish(message);

            }
        );

    }
private:
    rclcpp::Publisher<cave_drone_interfaces::msg::Sf45Measurement>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SF45DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}