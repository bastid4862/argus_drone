#include <memory>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "cave_drone_interfaces/msg/encoder_measurement.hpp"

class LidarPanTfBroadcaster : public rclcpp::Node {
public:
    LidarPanTfBroadcaster() : Node("lidar_pan_tf_broadcaster") {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        encoder_sub_ = this->create_subscription<cave_drone_interfaces::msg::EncoderMeasurement>(
            "/encoder_angle",
            10,
            std::bind(
                &LidarPanTfBroadcaster::encoder_callback,
                this,
                std::placeholders::_1
            )
        );
    }

private:
    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Subscriber
    rclcpp::Subscription<cave_drone_interfaces::msg::EncoderMeasurement>::SharedPtr encoder_sub_;

    // Callbacks
    void encoder_callback(const cave_drone_interfaces::msg::EncoderMeasurement::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;

        // Use the encoder measurement time
        transform.header.stamp = msg->timestamp;

        // Parent frame
        transform.header.frame_id = "lidar_pan_base";

        // Rotating child frame
        transform.child_frame_id = "lidar_pan_axis";

        // Both frames are centered on the same pan shaft
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        // Convert encoder angle to our pan angle
        double pan_deg = msg->angle - 90.0;

        // Convert degrees to radians
        double pan_rad = pan_deg * M_PI / 180.0;

        tf2::Quaternion q;

        // roll = 0, pitch = 0, yaw = pan_rad
        q.setRPY(
            0.0,
            0.0,
            pan_rad
        );

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPanTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
