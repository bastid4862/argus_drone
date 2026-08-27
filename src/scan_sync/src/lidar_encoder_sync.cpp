#include <functional>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "cave_drone_interfaces/msg/encoder_measurement.hpp"
#include "cave_drone_interfaces/msg/sf45_measurement.hpp"
#include "cave_drone_interfaces/msg/synchronized_measurement.hpp"

class LidarEncoderSync : public rclcpp::Node {
public:
    LidarEncoderSync() : Node("lidar_encoder_sync") {
        RCLCPP_INFO(this->get_logger(), "LiDAR/encoder sync node started");

        encoder_sub_ = this->create_subscription<cave_drone_interfaces::msg::EncoderMeasurement>(
            "/encoder_angle",
            10,
            std::bind(
                &LidarEncoderSync::encoder_callback,
                this,
                std::placeholders::_1
            )
        );

        sf45_sub_ = this->create_subscription<cave_drone_interfaces::msg::Sf45Measurement>(
            "/sf45/measurements",
            10,
            std::bind(
                &LidarEncoderSync::sf45_callback,
                this,
                std::placeholders::_1
            )
        );

        sync_publisher_ = this -> create_publisher<cave_drone_interfaces::msg::SynchronizedMeasurement>(
            "/scan_sync/measurement",
            10
        );
    }

private:
    void encoder_callback(const cave_drone_interfaces::msg::EncoderMeasurement::SharedPtr msg) {
        // Add newest encoder measurement
        encoder_buffer_.push_back(*msg);

        // Remove oldest measurement if buffer gets too large
        if (encoder_buffer_.size() > 100) {
            encoder_buffer_.pop_front();
        }

    }

    void sf45_callback(const cave_drone_interfaces::msg::Sf45Measurement::SharedPtr msg) {
        // Need at least two encoder measurements to interpolate
        if (encoder_buffer_.size() < 2) {
            return;
        }

        // Convert LiDAR timestamp to nanoseconds
        int64_t lidar_time_ns = static_cast<int64_t>(msg->timestamp.sec) * 1000000000LL + msg->timestamp.nanosec;

        // Check each pair of encoder measurements
        for (size_t i = 0; i + 1 < encoder_buffer_.size(); i++) {
            // Encoder measurement before the LiDAR time
            const auto &before = encoder_buffer_[i];

            // Encoder measurement after the LiDAR time
            const auto &after = encoder_buffer_[i + 1];

            // Convert the "before" timestamp to nanoseconds
            int64_t before_time_ns = static_cast<int64_t>(before.timestamp.sec) * 1000000000LL + before.timestamp.
                                     nanosec;

            // Convert the "after" timestamp to nanoseconds
            int64_t after_time_ns = static_cast<int64_t>(after.timestamp.sec) * 1000000000LL + after.timestamp.nanosec;

            // Check if the LiDAR time is between these two encoder times
            if ((before_time_ns <= lidar_time_ns) &&
                (lidar_time_ns <= after_time_ns)) {
                // Skip if both encoder timestamps are the same
                if (after_time_ns == before_time_ns) {
                    continue;
                }

                // Calculate how far the LiDAR time is between the two encoder times
                double ratio =
                        static_cast<double>(lidar_time_ns - before_time_ns) /
                        static_cast<double>(after_time_ns - before_time_ns);

                // Estimate the encoder angle at the LiDAR timestamp
                double interpolated_angle =
                        before.angle +
                        ratio * (after.angle - before.angle);

                // Create the synchronized measurement
                cave_drone_interfaces::msg::SynchronizedMeasurement sync_msg;

                // Copy the LiDAR distance
                sync_msg.distance = msg->distance;

                // Copy the LiDAR internal scan angle
                sync_msg.internal_scan_angle = msg->internal_scan_angle;

                // Store the interpolated external encoder angle
                sync_msg.external_angle = interpolated_angle;

                // Keep the original LiDAR timestamp
                sync_msg.timestamp = msg->timestamp;

                // Publish the synchronized measurement
                sync_publisher_->publish(sync_msg);

                break;
            }
        }
        
    }


    // Subscribers
    rclcpp::Subscription<cave_drone_interfaces::msg::EncoderMeasurement>::SharedPtr encoder_sub_;
    rclcpp::Subscription<cave_drone_interfaces::msg::Sf45Measurement>::SharedPtr sf45_sub_;

    // Publishers
    rclcpp::Publisher<cave_drone_interfaces::msg::SynchronizedMeasurement>::SharedPtr sync_publisher_;

    // Deque
    std::deque<cave_drone_interfaces::msg::EncoderMeasurement> encoder_buffer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LidarEncoderSync>()
    );

    rclcpp::shutdown();
    return 0;
}
