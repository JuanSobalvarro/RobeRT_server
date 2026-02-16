#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// Include the header generated from egm.proto
// This file is created automatically in the build folder by CMake
#include "egm.pb.h"

#define EGM_PORT 6510
#define BUFFER_SIZE 1400 // MTU size is usually 1500, so 1400 is safe

class EgmDriver : public rclcpp::Node {
public:
    EgmDriver() : Node("egm_driver_node"), sequence_number_(0) {
        RCLCPP_INFO(this->get_logger(), "Initializing EGM Driver...");
        
        // 1. Setup UDP Socket
        if (!setup_socket()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to setup socket. Exiting.");
            exit(EXIT_FAILURE);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for incoming EGM data on port %d...", EGM_PORT);

        // EGM is driven by the robot's clock (incoming packets), so we block-read in a loop.
        // In a real ROS 2 driver, this might run in a separate thread to allow ROS callbacks to spin.
        run_control_loop();
    }

    ~EgmDriver() {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    int sockfd_;
    struct sockaddr_in server_addr_, client_addr_;
    socklen_t client_len_;
    uint32_t sequence_number_;

    bool setup_socket() {
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return false;
        }

        // Configure server address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
        server_addr_.sin_port = htons(EGM_PORT);

        // Bind socket
        if (bind(sockfd_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed. Is port %d already in use?", EGM_PORT);
            return false;
        }

        return true;
    }

    void run_control_loop() {
        char buffer[BUFFER_SIZE];
        client_len_ = sizeof(client_addr_);

        while (rclcpp::ok()) {
            // A. Receive Data (Blocking Call)
            // The robot sends a packet every 4ms. We wait here until it arrives.
            int n = recvfrom(sockfd_, buffer, BUFFER_SIZE, 0, 
                             (struct sockaddr *)&client_addr_, &client_len_);
            
            if (n < 0) {
                RCLCPP_WARN(this->get_logger(), "recvfrom failed");
                continue;
            }

            // B. Deserialize (Protobuf)
            abb::egm::EgmRobot robot_message;
            if (!robot_message.ParseFromArray(buffer, n)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse EGM message");
                continue;
            }

            // C. Process Data
            // Check if we have a valid header and feedback
            if (robot_message.has_header() && robot_message.has_feedback()) {
                process_robot_message(robot_message);
                
                // D. Send Reply
                send_reply(robot_message);
            }
        }
    }

    void process_robot_message(const abb::egm::EgmRobot &msg) {
        // Log status every 100 messages (approx every 400ms)
        if (msg.header().seqno() % 250 == 0) {
            double j1 = 0.0;
            if (msg.feedback().has_joints() && msg.feedback().joints().joints_size() > 0) {
                j1 = msg.feedback().joints().joints(0);
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "EGM Connected | Seq: %d | Time: %d | J1 Position: %.2f", 
                msg.header().seqno(), msg.header().tm(), j1);
        }
    }

    void send_reply(const abb::egm::EgmRobot &robot_msg) {
        abb::egm::EgmSensor sensor_message;

        // 1. Construct Header
        auto *header = sensor_message.mutable_header();
        header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
        header->set_seqno(sequence_number_++);
        header->set_tm(robot_msg.header().tm()); // Echo the timestamp

        // 2. Construct Body (Planned Position)
        // For this test, we just mirror the current position (Hold Position)
        // In the real driver, this is where you inject the target from ROS
        auto *planned = sensor_message.mutable_planned();
        if (robot_msg.feedback().has_joints()) {
            auto *joints = planned->mutable_joints();
            joints->CopyFrom(robot_msg.feedback().joints());
        }

        // 3. Serialize
        std::string output_buffer;
        if (!sensor_message.SerializeToString(&output_buffer)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to serialize response");
            return;
        }

        // 4. Send
        sendto(sockfd_, output_buffer.c_str(), output_buffer.size(), 0, 
               (struct sockaddr *)&client_addr_, client_len_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EgmDriver>();
    // Note: Since run_control_loop is blocking, we don't call rclcpp::spin here 
    // in this simple example. In a full architecture, we would use a thread.
    rclcpp::shutdown();
    return 0;
}