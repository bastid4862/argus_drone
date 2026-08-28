#include <memory>
#include <functional>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "cave_drone_interfaces/msg/synchronized_measurement.hpp"

struct Point3D {
    float x;
    float y;
    float z;
};

class PointCloudBuilder : public rclcpp::Node {
public:
    PointCloudBuilder() : Node("pointcloud_builder") {
        RCLCPP_INFO(
            this->get_logger(),
            "Point cloud builder node started"
        );
        sync_sub_ = this->create_subscription<cave_drone_interfaces::msg::SynchronizedMeasurement>(
            "/scan_sync/measurement",
            10,
            std::bind(
                &PointCloudBuilder::sync_callback,
                this,
                std::placeholders::_1
            )
        );

        // Publish the reconstructed 3D point cloud
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar/points",
            10
        );
    }

private:
    // Subscribers
    rclcpp::Subscription<cave_drone_interfaces::msg::SynchronizedMeasurement>::SharedPtr sync_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    // Stores all reconstructed 3D points
    std::vector<Point3D> points_;

    // Number of points to collect before publishing
    static constexpr size_t CLOUD_SIZE = 100;

    // Callbacks
    void sync_callback(const cave_drone_interfaces::msg::SynchronizedMeasurement::SharedPtr msg) {
        // Convert internal scan angle from degrees to radians
        double internal_rad = msg->internal_scan_angle * M_PI / 180.0;

        // Convert external tilt angle from degrees to radians
        double external_rad = msg->external_angle * M_PI / 180.0;

        // Calculate the 3D point
        double x = msg->distance * std::cos(external_rad) * std::cos(internal_rad);

        double y = msg->distance * std::cos(external_rad) * std::sin(internal_rad);

        double z = msg->distance * std::sin(external_rad);

        // Store the new 3D point
        Point3D point;

        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = static_cast<float>(z);

        // Add this point to the vector
        points_.push_back(point);

        if (points_.size() >= CLOUD_SIZE) {
            // Create a PointCloud2 message
            sensor_msgs::msg::PointCloud2 cloud_msg;

            // Set the size of the point cloud
            cloud_msg.height = 1;
            cloud_msg.width = points_.size();

            // Define the x, y, z fields in each point
            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

            modifier.setPointCloud2FieldsByString(
                1,
                "xyz"
            );

            // Resize the cloud to hold all stored points
            modifier.resize(points_.size());

            // Create iterators for the x, y, and z fields
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            // Copy each stored point into the PointCloud2 message
            for (const Point3D &point: points_) {
                *iter_x = point.x;
                *iter_y = point.y;
                *iter_z = point.z;

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            // Keep the original measurement timestamp
            cloud_msg.header.stamp = msg->timestamp;

            // Temporary frame name until Phase 13 defines the TF tree
            cloud_msg.header.frame_id = "lidar_tilt_axis";

            // Publish the point cloud
            pointcloud_publisher_->publish(cloud_msg);

            // Confirm that a full point cloud was published
            RCLCPP_INFO(
                this->get_logger(),
                "Published point cloud with %zu points",
                points_.size()
            );

            // Clear the old batch and start collecting the next one
            points_.clear();
        }

    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudBuilder>());
    rclcpp::shutdown();
    return 0;
}
