#include <Arduino.h>
#include "Encoder.hpp"
#include "ServoController.hpp"
#include "ScanController.hpp"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <cmath>

#include <cave_drone_interfaces/msg/sweep_command.h>

// Custom message for encoder angle and timestamp
#include <cave_drone_interfaces/msg/encoder_measurement.h>

// Use Pin 10 as our CS (Chip Select) wire.
Encoder myEncoder(10);
// Servo signal pin
ServoController myServo(23);

ScanController myScanner(myServo, myEncoder);

// micro-ROS objects
rcl_publisher_t publisher;

// Stores the encoder angle and timestamp
cave_drone_interfaces__msg__EncoderMeasurement encoder_msg;

rcl_subscription_t reset_sub;
std_msgs__msg__Empty reset_msg;

rcl_subscription_t set_angle_sub;
std_msgs__msg__Float32 set_angle_msg;

rcl_subscription_t home_sub;
std_msgs__msg__Empty home_msg;

rcl_subscription_t stop_sub;
std_msgs__msg__Empty stop_msg;

rcl_subscription_t sweep_sub;
cave_drone_interfaces__msg__SweepCommand sweep_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rclc_executor_t executor;

// ROS 2 publisher for scan mode
rcl_publisher_t scan_mode_publisher;

// Message that stores the current scan mode number
std_msgs__msg__Int8 scan_mode_msg;

// Target Angle variables
rcl_publisher_t target_angle_publisher;
std_msgs__msg__Float32 target_angle_msg;

// Fault variables
rcl_publisher_t fault_publisher;
std_msgs__msg__Bool fault_msg;

// Direction variables
rcl_publisher_t sweep_direction_publisher;
std_msgs__msg__Int8 sweep_direction_msg;

int encoder_publish_fail_count = 0;
int scan_mode_publish_fail_count = 0;
int target_angle_publish_fail_count = 0;
int fault_publish_fail_count = 0;
int sweep_direction_publish_fail_count = 0;

void reset_callback(const void *msgin) {
    (void) msgin;

    // Clear the fault state and return to HOME mode
    myScanner.clearFault();
}

// Takes a new angle, takes that number, and then switches scanner to POSITION mode
void set_angle_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg =
            static_cast<const std_msgs__msg__Float32 *>(msgin);

    myScanner.setTargetAngle(msg->data);
    myScanner.setMode(ScanMode::POSITION);
}

void home_callback(const void *msgin) {
    (void) msgin; // We dont need any data from an empty message
    myScanner.setMode(ScanMode::HOME);
}

void stop_callback(const void *msgin) {
    (void) msgin;

    myScanner.setMode(ScanMode::STOPPED);
}

void sweep_callback(const void *msgin) {
    const cave_drone_interfaces__msg__SweepCommand *msg =
            static_cast<const cave_drone_interfaces__msg__SweepCommand *>(msgin);

    bool valid = myScanner.configureSweep(
        msg->min_angle,
        msg->max_angle,
        msg->speed,
        msg->direction
    );

    if (!valid) {
        return;
    }

    myScanner.setMode(ScanMode::SWEEP);
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

    // Synchronize Teensy time after the micro-ROS session exists
    while (!rmw_uros_epoch_synchronized()) {

        rmw_uros_sync_session(1000);

        delay(100);
    }

    rclc_node_init_default(&node, "teensy_encoder_node", "", &support);

    // Publisher initialization
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(cave_drone_interfaces, msg, EncoderMeasurement),
        "encoder_angle"
    );

    // Initialize executor with capacity for 5 callbacks
    rclc_executor_init(&executor, &support.context, 5, &allocator);

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

    // Create subscriber for sweep commands from the Jetson
    rclc_subscription_init_default(
        &sweep_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            cave_drone_interfaces,
            msg,
            SweepCommand
        ),
        "sweep"
    );

    // Add the sweep subscriber to the executor
    rclc_executor_add_subscription(
        &executor,
        &sweep_sub,
        &sweep_msg,
        &sweep_callback,
        ON_NEW_DATA
    );

    rclc_publisher_init_default(
        &scan_mode_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "scan_mode"
    );

    rclc_publisher_init_default(
        &target_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "target_angle"
    );

    rclc_publisher_init_default(
        &fault_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "fault"
    );

    rclc_publisher_init_default(
        &sweep_direction_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "sweep_direction"
    );
}

void loop() {
    // Process micro-ROS subscription callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Check if system has a fault before running
    if (myScanner.hasFault()) {
        myScanner.setMode(ScanMode::STOPPED);
        myScanner.update();
    } else {
        // Execute state logic for active ScanMode
        myScanner.update();
    }

    // Get the current scan mode as a number
    scan_mode_msg.data = static_cast<int8_t>(myScanner.getMode());

    // Publish the current scan mode to ROS 2
    rcl_ret_t scan_mode_result = rcl_publish(&scan_mode_publisher, &scan_mode_msg, NULL);

    // Read current position
    encoder_msg.angle = myEncoder.readAngle();

    // Get synchronized ROS time in nanoseconds
    int64_t time_ns = rmw_uros_epoch_nanos();

    // Store whole seconds
    encoder_msg.timestamp.sec =
            time_ns / 1000000000;

    // Store remaining nanoseconds
    encoder_msg.timestamp.nanosec =
            time_ns % 1000000000;

    // Publish over micro-ROS
    rcl_ret_t publish_result = rcl_publish(&publisher, &encoder_msg, NULL);

    // Get the current target angle from ScanController
    target_angle_msg.data = myScanner.getTargetAngle();

    // Publish it to /target_angle
    rcl_ret_t target_angle_publish_result = rcl_publish(&target_angle_publisher, &target_angle_msg, NULL);

    // Get the fault notification from ScanController
    fault_msg.data = myScanner.hasFault();

    // Get the sweep direction from ScanController
    sweep_direction_msg.data = myScanner.getSweepDirection();

    // Publish it to /fault
    rcl_ret_t fault_publish_result = rcl_publish(&fault_publisher, &fault_msg, NULL);

    // Publish sweep direction to /sweep_direction
    rcl_ret_t sweep_direction_publish_result = rcl_publish(&sweep_direction_publisher, &sweep_direction_msg, NULL);

    if (publish_result != RCL_RET_OK) {
        encoder_publish_fail_count++;
    } else {
        encoder_publish_fail_count = 0;
    }

    if (encoder_publish_fail_count >= 10) {
        myScanner.setFault();
    }

    if (scan_mode_result != RCL_RET_OK) {
        scan_mode_publish_fail_count++;
    } else {
        scan_mode_publish_fail_count = 0;
    }

    if (scan_mode_publish_fail_count >= 10) {
        myScanner.setFault();
    }

    if (target_angle_publish_result != RCL_RET_OK) {
        target_angle_publish_fail_count++;
    } else {
        target_angle_publish_fail_count = 0;
    }

    if (target_angle_publish_fail_count >= 10) {
        myScanner.setFault();
    }

    if (fault_publish_result != RCL_RET_OK) {
        fault_publish_fail_count++;
    } else {
        fault_publish_fail_count = 0;
    }

    if (fault_publish_fail_count >= 10) {
        myScanner.setFault();
    }

    if (sweep_direction_publish_result != RCL_RET_OK) {
        sweep_direction_publish_fail_count++;
    } else {
        sweep_direction_publish_fail_count = 0;
    }

    if (sweep_direction_publish_fail_count >= 10) {
        myScanner.setFault();
    }

    delay(15);
}
