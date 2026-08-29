#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class Sf45TfBroadcaster : public rclcpp::Node{
    public:
        Sf45TfBroadcaster(): Node("sf45_tf_broadcaster"){
            static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // Creates parameters for the SF45 position relative to lidar_pan_axis
            this->declare_parameter<double>("sf45_x", 0.0);
            this->declare_parameter<double>("sf45_y", 0.0);
            this->declare_parameter<double>("sf45_z", 0.0);

            // Reads the parameter values
            double x = this->get_parameter("sf45_x").as_double();
            double y = this->get_parameter("sf45_y").as_double();
            double z = this->get_parameter("sf45_z").as_double();

            // Creates parameters for the SF45 mounting rotation
            this->declare_parameter<double>("sf45_roll", 0.0);
            this->declare_parameter<double>("sf45_pitch", 0.0);
            this->declare_parameter<double>("sf45_yaw", 0.0);

            // Reads the rotation parameters
            double roll = this->get_parameter("sf45_roll").as_double();
            double pitch = this->get_parameter("sf45_pitch").as_double();
            double yaw = this->get_parameter("sf45_yaw").as_double();

            // Creates the fixed transform from lidar_pan_axis to sf45_link
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp = this->get_clock()->now();

            transform.header.frame_id = "lidar_pan_axis";
            transform.child_frame_id = "sf45_link";

            // Sets the SF45 position
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = z;

            // Converts the SF45 mounting rotation to a quaternion
            tf2::Quaternion q;
            q.setRPY(
                roll,
                pitch,
                yaw
            );

            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            // Publish the fixed SF45 transform
            static_broadcaster_->sendTransform(transform);
        }

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sf45TfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}