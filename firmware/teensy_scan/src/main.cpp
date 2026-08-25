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
#include <rmw_microros/rmw_microros.h>

#include <cmath>

// Use Pin 10 as our CS (Chip Select) wire.
Encoder myEncoder(10);
// Servo signal pin
ServoController myServo(23);

ScanController myScanner(myServo, myEncoder);

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

rcl_subscription_t reset_sub;
std_msgs__msg__Empty reset_msg;

rcl_subscription_t set_angle_sub;
std_msgs__msg__Float32 set_angle_msg;

rcl_subscription_t home_sub;
std_msgs__msg__Empty home_msg;

rcl_subscription_t stop_sub;
std_msgs__msg__Empty stop_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rclc_executor_t executor;

void reset_callback(const void * msgin) {
    // Clear the fault state and return to HOME mode
    myScanner.clearFault();
}

// Takes a new angle, takes that number, and then switches scanner to POSITION mode
void set_angle_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg =
        static_cast<const std_msgs__msg__Float32 *>(msgin);

    myScanner.setTargetAngle(msg->data);
    myScanner.setMode(ScanMode::POSITION);
}

void home_callback(const void * msgin) {
    (void)msgin;  // We dont need any data from an empty message
    myScanner.setMode(ScanMode::HOME);
}

void stop_callback(const void * msgin) {
    (void)msgin;

    myScanner.setMode(ScanMode::STOPPED);
}

void setup() {
    // Run the hardware setup we wrote in Encoder.cpp
    myEncoder.begin();
    myServo.begin();

    // USB Transport & micro-ROS initialization
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    delay(2000);

    // Wait until the micro-ROS Agent is available
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        delay(100);
    }

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

    // Initialize executor with capacity for 4 callbacks
    rclc_executor_init(&executor, &support.context, 4, &allocator);

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

    // Create subscriber for target angle commands from the Jetson
    rclc_subscription_init_default(
        &set_angle_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "set_angle"
    );

    // Add the set-angle subscriber to the executor
    rclc_executor_add_subscription(
        &executor,
        &set_angle_sub,
        &set_angle_msg,
        &set_angle_callback,
        ON_NEW_DATA
    );

    // Create subscriber for home commands from the Jetson
    rclc_subscription_init_default(
        &home_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "home"
    );

    // Add the home subscriber to the executor
    rclc_executor_add_subscription(
        &executor,
        &home_sub,
        &home_msg,
        &home_callback,
        ON_NEW_DATA
    );

    // Create subscriber for stop commands from the Jetson
    rclc_subscription_init_default(
        &stop_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "stop"
    );

    // Add the stop subscriber to the executor
    rclc_executor_add_subscription(
        &executor,
        &stop_sub,
        &stop_msg,
        &stop_callback,
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