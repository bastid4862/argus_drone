#include <Arduino.h>
#include "Encoder.hpp"
#include "ServoController.hpp"
#include "ScanController.hpp"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/empty.h>
#include <rclc/executor.h>

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

rcl_subscription_t reset_sub;
std_msgs__msg__Empty reset_msg;

rclc_executor_t executor;

void reset_callback(const void * msgin) {
    // Clear the fault state and return to HOME mode
    myScanner.clearFault();
}

void setup() {
    // Run the hardware setup we wrote in Encoder.cpp
    myEncoder.begin();
    myServo.begin();

    // USB Transport & micro-ROS initialization
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

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

    // Initialize executor with capacity for 1 handle (subscriber)
    rclc_executor_init(&executor, &support.context, 1, &allocator);

    // Initialize the subscriber on topic "reset_fault"
    rclc_subscription_init_default(
        &reset_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "reset_fault"
    );

    // Attach subscriber and callback to executor
    rclc_executor_add_subscription(
        &executor,
        &reset_sub,
        &reset_msg,
        &reset_callback,
        ON_NEW_DATA
    );


}

void loop() {
    // Process micro-ROS subscription callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

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