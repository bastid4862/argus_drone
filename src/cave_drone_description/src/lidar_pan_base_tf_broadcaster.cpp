#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LidarPanBaseTfBroadcaster : public rclcpp::Node{
public:
    LidarPanBaseTfBroadcaster(): Node("lidar_pan_base_tf_broadcaster"){
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Create parameters for the fixed position of the pan base relative to the drone's base_link frame
        this->declare_parameter<double>("pan_base_x", 0.0);
        this->declare_parameter<double>("pan_base_y", 0.0);
        this->declare_parameter<double>("pan_base_z", 0.0);

        // Read the current parameter values
        double x = this->get_parameter("pan_base_x").as_double();
        double y = this->get_parameter("pan_base_y").as_double();
        double z = this->get_parameter("pan_base_z").as_double();

        // Create the fixed transform from base_link to lidar_pan_base
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();

        transform.header.frame_id = "base_link";
        transform.child_frame_id = "lidar_pan_base";

        // Set the pan base position
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;

        // No fixed rotation between base_link and lidar_pan_base
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        // Publish the fixed transform
        static_broadcaster_->sendTransform(transform);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPanBaseTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}