#ifndef ROBOFLEX_UDP_HARDWARE__ROBOFLEX_UDP_HARDWARE_HPP
#define ROBOFLEX_UDP_HARDWARE__ROBOFLEX_UDP_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <netinet/in.h>
#include <string>
#include <vector>


namespace roboflex_udp_hardware
{
class RoboflexUdpHardware : public hardware_interface::SystemInterface
{
public:
    ~RoboflexUdpHardware() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    static constexpr const char *kDefaultUdpTargetIp = "255.255.255.255";
    static constexpr int kDefaultUdpTargetPort = 9999;
    static constexpr std::chrono::milliseconds kDefaultKeepAliveInterval{200};
    static constexpr double kDefaultCommandChangeEpsilonRad = 1e-4;
    static constexpr std::chrono::seconds kSendErrorLogInterval{2};
    static constexpr size_t kMaxUdpPayloadBytes = 256;

    size_t num_joints_{0};  // Number of joints
    std::vector<double> position_commands_; // Target angles (commands)
    std::vector<double> position_states_;   // Open-loop state mirrors commanded positions
    std::vector<double> last_sent_commands_;

    std::string udp_target_ip_{kDefaultUdpTargetIp};
    int udp_target_port_{kDefaultUdpTargetPort};
    std::chrono::milliseconds keep_alive_interval_{kDefaultKeepAliveInterval};
    double command_change_epsilon_rad_{kDefaultCommandChangeEpsilonRad};
    int udp_socket_fd_{-1};
    sockaddr_in udp_target_addr_{};
    bool udp_ready_{false};
    std::chrono::steady_clock::time_point last_udp_send_time_{};
    std::chrono::steady_clock::time_point last_send_error_log_time_{};

    rclcpp::Logger logger_{rclcpp::get_logger("roboflex_udp_hardware")};

    bool configure_udp_socket();
    void close_udp_socket();
    void mirror_commands_to_states();
    bool should_log_send_error_now();
    bool should_send_udp_now(const std::chrono::steady_clock::time_point &now) const;
    void mark_udp_sent(const std::chrono::steady_clock::time_point &now);
    size_t build_udp_payload(char *buffer, size_t buffer_size) const;
};
} // namespace roboflex_udp_hardware

#endif // ROBOFLEX_UDP_HARDWARE__ROBOFLEX_UDP_HARDWARE_HPP
