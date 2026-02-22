#include "roboflex_udp_hardware/roboflex_udp_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <cstdio>

namespace roboflex_udp_hardware
{

namespace
{
// Startup ROS commands in radians.
// First startup pose set to SRDF "Arm Up" for arm joints.
// joint_gripper is kept at 0.0.
constexpr double kInitialCommandRad[] = {0.0349066, -0.3839724, -0.9250245, -0.3665191, 0.0};
constexpr size_t kInitialCommandCount = sizeof(kInitialCommandRad) / sizeof(kInitialCommandRad[0]);
}  // namespace

RoboflexUdpHardware::~RoboflexUdpHardware()
{
    close_udp_socket();
}

hardware_interface::CallbackReturn RoboflexUdpHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params)
{
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &info = params.hardware_info;
    num_joints_ = info.joints.size();
    position_commands_.assign(num_joints_, 0.0);
    position_states_.assign(num_joints_, 0.0);
    last_sent_commands_.assign(num_joints_, std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < num_joints_ && i < kInitialCommandCount; ++i)
    {
        position_commands_[i] = kInitialCommandRad[i];
        position_states_[i] = kInitialCommandRad[i];
    }

    const auto &params_map = info.hardware_parameters;

    const auto ip_it = params_map.find("udp_target_ip");
    if (ip_it != params_map.end() && !ip_it->second.empty())
    {
        udp_target_ip_ = ip_it->second;
    }

    const auto port_it = params_map.find("udp_target_port");
    if (port_it != params_map.end() && !port_it->second.empty())
    {
        try
        {
            udp_target_port_ = std::stoi(port_it->second);
        }
        catch (const std::exception &)
        {
            RCLCPP_ERROR(
                logger_,
                "Invalid udp_target_port '%s'.",
                port_it->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    const auto keepalive_it = params_map.find("udp_keepalive_ms");
    if (keepalive_it != params_map.end() && !keepalive_it->second.empty())
    {
        try
        {
            const int keepalive_ms = std::stoi(keepalive_it->second);
            if (keepalive_ms <= 0 || keepalive_ms > 10000)
            {
                RCLCPP_ERROR(logger_, "udp_keepalive_ms out of range (1..10000): %d", keepalive_ms);
                return hardware_interface::CallbackReturn::ERROR;
            }
            keep_alive_interval_ = std::chrono::milliseconds(keepalive_ms);
        }
        catch (const std::exception &)
        {
            RCLCPP_ERROR(
                logger_,
                "Invalid udp_keepalive_ms '%s'.",
                keepalive_it->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    const auto epsilon_it = params_map.find("udp_command_change_epsilon_rad");
    if (epsilon_it != params_map.end() && !epsilon_it->second.empty())
    {
        try
        {
            command_change_epsilon_rad_ = std::stod(epsilon_it->second);
            if (command_change_epsilon_rad_ < 0.0 || command_change_epsilon_rad_ > 1.0)
            {
                RCLCPP_ERROR(
                    logger_,
                    "udp_command_change_epsilon_rad out of range (0.0..1.0): %.6f",
                    command_change_epsilon_rad_);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        catch (const std::exception &)
        {
            RCLCPP_ERROR(
                logger_,
                "Invalid udp_command_change_epsilon_rad '%s'.",
                epsilon_it->second.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (udp_target_port_ <= 0 || udp_target_port_ > 65535)
    {
        RCLCPP_ERROR(logger_, "udp_target_port out of range: %d", udp_target_port_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!configure_udp_socket())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        logger_,
        "RoboFlexUdpHardware initialized. target=%s:%d keepalive=%ldms epsilon=%.6f rad",
        udp_target_ip_.c_str(),
        udp_target_port_,
        static_cast<long>(keep_alive_interval_.count()),
        command_change_epsilon_rad_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboflexUdpHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "position", &position_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboflexUdpHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &position_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type RoboflexUdpHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    mirror_commands_to_states();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboflexUdpHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!udp_ready_)
    {
        return hardware_interface::return_type::ERROR;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!should_send_udp_now(now))
    {
        mirror_commands_to_states();
        return hardware_interface::return_type::OK;
    }

    char payload[kMaxUdpPayloadBytes];
    const size_t payload_size = build_udp_payload(payload, sizeof(payload));
    if (payload_size == 0)
    {
        if (should_log_send_error_now())
        {
            RCLCPP_WARN(
                logger_,
                "Failed to build UDP payload for %zu joints; skipping this cycle",
                num_joints_);
        }
        mirror_commands_to_states();
        return hardware_interface::return_type::OK;
    }

    const ssize_t sent = sendto(
        udp_socket_fd_,
        payload,
        payload_size,
        0,
        reinterpret_cast<const sockaddr *>(&udp_target_addr_),
        sizeof(udp_target_addr_));

    if (sent < 0)
    {
        if (should_log_send_error_now())
        {
            RCLCPP_WARN(
                logger_,
                "Failed to send UDP motor command to %s:%d: %s",
                udp_target_ip_.c_str(),
                udp_target_port_,
                std::strerror(errno));
        }
    }
    else if (static_cast<size_t>(sent) != payload_size)
    {
        if (should_log_send_error_now())
        {
            RCLCPP_WARN(
                logger_,
                "Partial UDP send (%zd/%zu bytes) to %s:%d",
                sent,
                payload_size,
                udp_target_ip_.c_str(),
                udp_target_port_);
        }
    }
    else
    {
        mark_udp_sent(now);
    }

    mirror_commands_to_states();
    return hardware_interface::return_type::OK;
}

bool RoboflexUdpHardware::configure_udp_socket()
{
    close_udp_socket();

    udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0)
    {
        RCLCPP_ERROR(logger_, "Failed to create UDP socket: %s", std::strerror(errno));
        return false;
    }

    int broadcast_enable = 1;
    if (
        setsockopt(
            udp_socket_fd_,
            SOL_SOCKET,
            SO_BROADCAST,
            &broadcast_enable,
            sizeof(broadcast_enable)) != 0)
    {
        RCLCPP_WARN(logger_, "Failed to enable UDP broadcast mode: %s", std::strerror(errno));
    }

    std::memset(&udp_target_addr_, 0, sizeof(udp_target_addr_));
    udp_target_addr_.sin_family = AF_INET;
    udp_target_addr_.sin_port = htons(static_cast<uint16_t>(udp_target_port_));
    if (inet_pton(AF_INET, udp_target_ip_.c_str(), &udp_target_addr_.sin_addr) != 1)
    {
        RCLCPP_ERROR(logger_, "Invalid udp_target_ip: '%s'", udp_target_ip_.c_str());
        close_udp_socket();
        return false;
    }

    udp_ready_ = true;
    return true;
}

void RoboflexUdpHardware::close_udp_socket()
{
    if (udp_socket_fd_ >= 0)
    {
        close(udp_socket_fd_);
        udp_socket_fd_ = -1;
    }
    udp_ready_ = false;
}

void RoboflexUdpHardware::mirror_commands_to_states()
{
    for (size_t i = 0; i < num_joints_; ++i)
    {
        position_states_[i] = position_commands_[i];
    }
}

bool RoboflexUdpHardware::should_log_send_error_now()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - last_send_error_log_time_ >= kSendErrorLogInterval)
    {
        last_send_error_log_time_ = now;
        return true;
    }
    return false;
}

bool RoboflexUdpHardware::should_send_udp_now(const std::chrono::steady_clock::time_point &now) const
{
    if (last_sent_commands_.size() != num_joints_)
    {
        return true;
    }

    if (last_udp_send_time_.time_since_epoch().count() == 0)
    {
        return true;
    }

    if (now - last_udp_send_time_ >= keep_alive_interval_)
    {
        return true;
    }

    for (size_t i = 0; i < num_joints_; ++i)
    {
        const double previous = last_sent_commands_[i];
        const double current = position_commands_[i];
        if (std::isnan(previous) || std::fabs(current - previous) > command_change_epsilon_rad_)
        {
            return true;
        }
    }

    return false;
}

void RoboflexUdpHardware::mark_udp_sent(const std::chrono::steady_clock::time_point &now)
{
    last_udp_send_time_ = now;
    last_sent_commands_ = position_commands_;
}

size_t RoboflexUdpHardware::build_udp_payload(char *buffer, size_t buffer_size) const
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return 0;
    }

    int written = std::snprintf(buffer, buffer_size, "CMD");
    if (written < 0 || static_cast<size_t>(written) >= buffer_size)
    {
        return 0;
    }
    size_t used = static_cast<size_t>(written);

    for (size_t i = 0; i < num_joints_; ++i)
    {
        written = std::snprintf(
            buffer + used,
            buffer_size - used,
            " %.6f",
            position_commands_[i]);
        if (written < 0 || static_cast<size_t>(written) >= (buffer_size - used))
        {
            return 0;
        }
        used += static_cast<size_t>(written);
    }

    return used;
}



} // namespace roboflex_udp_hardware

PLUGINLIB_EXPORT_CLASS(roboflex_udp_hardware::RoboflexUdpHardware, hardware_interface::SystemInterface)
