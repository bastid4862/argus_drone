#include "rclcpp/rclcpp.hpp"
#include "cave_drone_interfaces/msg/sf45_measurement.hpp"
#include <chrono>

// Linux serial port tools
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <vector>
#include <cstdint>

#include <functional>
#include <string>


using namespace std::chrono_literals;

class SF45DriverNode : public rclcpp::Node {
public:
    SF45DriverNode() : Node("sf45_node"), serial_port_fd_(-1) {
        publisher_ = this->create_publisher<cave_drone_interfaces::msg::Sf45Measurement>("sf45/measurements", 10);

        // Open the LiDAR serial port
        init_serial("/dev/ttyACM0");

        // Configure the LiDAR
        configure_sensor();

        // Read serial data every 10 ms
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&SF45DriverNode::read_serial_data, this));
    }

    ~SF45DriverNode() {
        // Close the serial port
        if (serial_port_fd_ >= 0) {
            close(serial_port_fd_);
        }
    }

private:
    void init_serial(const std::string &port_name) {
        // Open the serial port
        serial_port_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        // Check if opening failed
        if (serial_port_fd_ < 0) {
            RCLCPP_ERROR(this -> get_logger(), "Failed to open port: %s", port_name.c_str());
            return;
        }

        // Store the serial port settings
        struct termios tty;

        // Get the current serial settings
        if (tcgetattr(serial_port_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting termios attributes");
            close(serial_port_fd_);
            serial_port_fd_ = -1;
            return;
        }

        // Use raw binary serial data
        cfmakeraw(&tty);

        // Set Baud Rate (115200)
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Configure 8N1 (8 data bits, No parity, 1 stop bit)
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Allow serial reading
        tty.c_cflag |= CREAD | CLOCAL;

        // Save the new serial settings and apply them
        if (tcsetattr(serial_port_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to apply serial settings"
            );
            close(serial_port_fd_);
            serial_port_fd_ = -1;
            return;
        }
    }

    void read_serial_data() {
        // Stop if the serial port is not open
        if (serial_port_fd_ < 0) return;

        // Temporary place for new bytes
        uint8_t temp_buf[256];

        // Read up to 256 bytes from the LiDAR
        ssize_t bytes_read = read(serial_port_fd_, temp_buf, sizeof(temp_buf));

        if (bytes_read > 0) {
            // Save the new bytes
            rx_buffer_.insert(rx_buffer_.end(), temp_buf, temp_buf + bytes_read);

            // Look for complete LiDAR packets
            parse_buffer();
        }
    }

    uint16_t create_crc(const uint8_t *data, size_t size) {
        uint16_t crc = 0;

        for (size_t i = 0; i < size; ++i) {
            uint16_t code = crc >> 8;
            code ^= data[i];
            code ^= code >> 4;

            crc = crc << 8;
            crc ^= code;

            code = code << 5;
            crc ^= code;

            code = code << 7;
            crc ^= code;
        }

        return crc;
    }

    void send_command(uint8_t command_id, bool write_command, const std::vector<uint8_t> &data) {
        std::vector<uint8_t> packet;

        // Start byte
        packet.push_back(0xAA);

        // Payload = command ID + data
        size_t payload_length = 1 + data.size();

        // Build the flags
        uint16_t flags =
                static_cast<uint16_t>(payload_length << 6);

        // Set the write bit
        if (write_command) {
            flags |= 0x01;
        }

        // Add flags, low byte first
        packet.push_back(flags & 0xFF);
        packet.push_back((flags >> 8) & 0xFF);

        // Add command ID
        packet.push_back(command_id);

        // Add command data
        packet.insert(
            packet.end(),
            data.begin(),
            data.end()
        );

        // Calculate CRC
        uint16_t crc =
                create_crc(packet.data(), packet.size());

        // Add CRC, low byte first
        packet.push_back(crc & 0xFF);
        packet.push_back((crc >> 8) & 0xFF);

        // Send the packet
        ssize_t bytes_written =
                write(
                    serial_port_fd_,
                    packet.data(),
                    packet.size()
                );

        if (bytes_written != static_cast<ssize_t>(packet.size())) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to send SF45 command %u",
                command_id
            );
        }
    }

    void configure_sensor() {
        if (serial_port_fd_ < 0) {
            return;
        }

        // Ask for product name twice to start communication
        send_command(0, false, {});
        send_command(0, false, {});

        // Send distance + yaw angle
        // Bit 0 = first return raw distance
        // Bit 8 = yaw angle
        uint32_t output_fields = 0x101;

        std::vector<uint8_t> output_data = {
            static_cast<uint8_t>(output_fields & 0xFF),
            static_cast<uint8_t>((output_fields >> 8) & 0xFF),
            static_cast<uint8_t>((output_fields >> 16) & 0xFF),
            static_cast<uint8_t>((output_fields >> 24) & 0xFF)
        };

        send_command(
            27,
            true,
            output_data
        );

        // Stream command 44 continuously
        uint32_t stream_type = 5;

        std::vector<uint8_t> stream_data = {
            static_cast<uint8_t>(stream_type & 0xFF),
            static_cast<uint8_t>((stream_type >> 8) & 0xFF),
            static_cast<uint8_t>((stream_type >> 16) & 0xFF),
            static_cast<uint8_t>((stream_type >> 24) & 0xFF)
        };

        send_command(
            30,
            true,
            stream_data
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Sent SF45 distance and angle streaming configuration"
        );
    }

    void parse_buffer() {
        // Smallest possible packet:
        // AA | flags low | flags high | ID | CRC low | CRC high
        const size_t MIN_PACKET_SIZE = 6;

        while (rx_buffer_.size() >= MIN_PACKET_SIZE) {
            // Find the start of a packet
            if (rx_buffer_[0] != 0xAA) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // Read the packet length from the flags
            uint16_t flags =
                    static_cast<uint16_t>(rx_buffer_[1]) |
                    (static_cast<uint16_t>(rx_buffer_[2]) << 8);

            size_t payload_length = flags >> 6;

            // Payload must contain at least the command ID
            if (payload_length < 1 || payload_length > 1023) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // 3 header bytes + payload + 2 CRC bytes
            size_t packet_size = 3 + payload_length + 2;

            // Wait if the whole packet has not arrived yet
            if (rx_buffer_.size() < packet_size) {
                return;
            }

            // CRC sent by the LiDAR
            uint16_t received_crc =
                    static_cast<uint16_t>(rx_buffer_[packet_size - 2]) |
                    (static_cast<uint16_t>(rx_buffer_[packet_size - 1]) << 8);

            // CRC calculated by us
            uint16_t calculated_crc =
                    create_crc(rx_buffer_.data(), packet_size - 2);

            // Bad packet: remove the start byte and search again
            if (received_crc != calculated_crc) {
                RCLCPP_WARN(this->get_logger(), "SF45 packet failed CRC");

                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            // Command ID is the first byte of the payload
            uint8_t command_id = rx_buffer_[3];

            // Command 44 = distance data in centimeters
            if (command_id == 44 && payload_length == 5) {
                // Distance is a signed 16-bit value
                int16_t raw_distance = static_cast<int16_t>(
                    static_cast<uint16_t>(rx_buffer_[4]) |
                    (static_cast<uint16_t>(rx_buffer_[5]) << 8)
                );

                // Angle is a signed 16-bit value
                int16_t raw_angle = static_cast<int16_t>(
                    static_cast<uint16_t>(rx_buffer_[6]) |
                    (static_cast<uint16_t>(rx_buffer_[7]) << 8)
                );

                // cm -> meters
                float distance_m = raw_distance / 100.0f;

                // hundredths of a degree -> degrees
                float angle_deg = raw_angle / 100.0f;

                // Create the ROS message
                auto msg =
                        cave_drone_interfaces::msg::Sf45Measurement();

                msg.distance = distance_m;
                msg.internal_scan_angle = angle_deg;
                msg.timestamp = this->now();

                // Send the measurement
                publisher_->publish(msg);
            }

            // Remove the packet we just processed
            rx_buffer_.erase(
                rx_buffer_.begin(),
                rx_buffer_.begin() + packet_size
            );
        }
    }

    // Serial port connection
    int serial_port_fd_;

    // Bytes received from the LiDAR
    std::vector<uint8_t> rx_buffer_;

    rclcpp::Publisher<cave_drone_interfaces::msg::Sf45Measurement>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SF45DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
