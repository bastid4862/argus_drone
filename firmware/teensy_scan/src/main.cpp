#include <Arduino.h>
#include "Encoder.hpp"
#include "ServoController.hpp"
#include "ScanController.hpp"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

#include <cmath>

// Use Pin 10 as our CS (Chip Select) wire.
Encoder myEncoder(10);
// Servo signal pin
ServoController myServo(23);

ScanController myScanner(myServo, myEncoder);

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup() {
    // Run the hardware setup we wrote in Encoder.cpp
    myEncoder.begin();
    myServo.begin();

    // USB Transport & micro-ROS initialization
    set_microros_transports();
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "teensy_encoder_node", "", &support);

    // Publisher initialization
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "encoder_angle"
    );
}

void loop() {

    // Check if system has a fault before running
    if (myScanner.hasFault()) {
        return;
    }

    // Execute state logic for active ScanMode
    myScanner.update();

    // Read current position and publish over micro-ROS
    msg.data = myEncoder.readAngle();
    rcl_publish(&publisher, &msg, NULL);

    delay(15);
}